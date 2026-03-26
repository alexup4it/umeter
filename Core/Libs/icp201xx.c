/*
 * ICP-20100 barometric pressure sensor
 *
 * Driver based on TDK InvenSense Arduino ICP-201xx library.
 * Matches the exact begin() + start() + getData() sequence.
 *
 * Pressure formula: P(kPa) = (POUT / 2^17) * 40 + 70
 * Output unit: centipascals (Pa * 100) for integer precision.
 */

#include "icp201xx.h"

#include <string.h>

#include "cmsis_os.h"

#define I2C_TIMEOUT 50

/* 7-bit address 0x63, shifted left for HAL (AD0 = GND) */
#define I2C_ADDRESS (0x63 << 1)

/* Device ID */
#define EXPECTED_DEVICE_ID 0x63

/* Registers */
#define REG_TRIM1_MSB     0x05
#define REG_TRIM2_LSB     0x06
#define REG_TRIM2_MSB     0x07
#define REG_DEVICE_ID     0x0C
#define REG_OTP_CFG1      0xAC
#define REG_OTP_MR_LSB    0xAD
#define REG_OTP_MR_MSB    0xAE
#define REG_OTP_MRA_LSB   0xAF
#define REG_OTP_MRA_MSB   0xB0
#define REG_OTP_MRB_LSB   0xB1
#define REG_OTP_MRB_MSB   0xB2
#define REG_OTP_ADDR      0xB5
#define REG_OTP_CMD       0xB6
#define REG_OTP_RD_DATA   0xB8
#define REG_OTP_STATUS    0xB9
#define REG_OTP_DBG2      0xBC
#define REG_MASTER_LOCK   0xBE
#define REG_OTP_STATUS2   0xBF
#define REG_MODE_SELECT   0xC0
#define REG_INT_STATUS    0xC1
#define REG_INT_MASK      0xC2
#define REG_FIFO_CONFIG   0xC3
#define REG_FIFO_FILL     0xC4
#define REG_DEVICE_STATUS 0xCD
#define REG_VERSION       0xD3
#define REG_FIFO_BASE     0xFA

/* MODE_SELECT bit fields */
#define BIT_MEAS_CONFIG_POS      5
#define BIT_FORCED_MEAS_TRIG_POS 4
#define BIT_MEAS_MODE_POS        3
#define BIT_POWER_MODE_POS       2
#define BIT_FIFO_READOUT_MASK    0x03

/* FIFO level mask */
#define FIFO_LEVEL_MASK 0x1F

/* Max retries for polling loops */
#define MAX_POLL_RETRIES 200

/* FIR filter warmup: first 14 packets must be discarded */
#define WARMUP_PACKETS 14

/******************************************************************************/
/* I2C helpers                                                                */
/******************************************************************************/

static int rd_reg(struct icp201xx* self, uint8_t reg, uint8_t* val) {
    return HAL_I2C_Mem_Read(self->i2c,
                            I2C_ADDRESS,
                            reg,
                            I2C_MEMADD_SIZE_8BIT,
                            val,
                            1,
                            I2C_TIMEOUT) == HAL_OK
               ? 0
               : -1;
}

static int wr_reg(struct icp201xx* self, uint8_t reg, uint8_t val) {
    return HAL_I2C_Mem_Write(self->i2c,
                             I2C_ADDRESS,
                             reg,
                             I2C_MEMADD_SIZE_8BIT,
                             &val,
                             1,
                             I2C_TIMEOUT) == HAL_OK
               ? 0
               : -1;
}

static int rd_reg_multi(struct icp201xx* self,
                        uint8_t reg,
                        uint8_t* buf,
                        uint16_t len) {
    return HAL_I2C_Mem_Read(self->i2c,
                            I2C_ADDRESS,
                            reg,
                            I2C_MEMADD_SIZE_8BIT,
                            buf,
                            len,
                            I2C_TIMEOUT) == HAL_OK
               ? 0
               : -1;
}

/******************************************************************************/
/* MODE_SELECT helpers (read-modify-write with mode_sync wait)                */
/******************************************************************************/

static int wait_mode_sync(struct icp201xx* self) {
    uint8_t status;

    for (int i = 0; i < MAX_POLL_RETRIES; i++) {
        if (rd_reg(self, REG_DEVICE_STATUS, &status) != 0) {
            return -1;
        }
        if (status & 0x01) {
            return 0;
        }
        osDelay(pdMS_TO_TICKS(1));
    }

    return -1;
}

static int wr_mode_select(struct icp201xx* self, uint8_t val) {
    if (wait_mode_sync(self) != 0) {
        return -1;
    }
    return wr_reg(self, REG_MODE_SELECT, val);
}

static int wr_pow_mode(struct icp201xx* self, uint8_t mode) {
    uint8_t reg;
    if (rd_reg(self, REG_MODE_SELECT, &reg) != 0) {
        return -1;
    }
    reg = (reg & ~(1 << BIT_POWER_MODE_POS)) |
          ((mode & 0x01) << BIT_POWER_MODE_POS);
    return wr_mode_select(self, reg);
}

static int wr_fifo_readout_mode(struct icp201xx* self, uint8_t mode) {
    uint8_t reg;
    if (rd_reg(self, REG_MODE_SELECT, &reg) != 0) {
        return -1;
    }
    reg = (reg & ~BIT_FIFO_READOUT_MASK) | (mode & BIT_FIFO_READOUT_MASK);
    return wr_mode_select(self, reg);
}

static int wr_meas_config(struct icp201xx* self, uint8_t mode) {
    uint8_t reg;
    if (rd_reg(self, REG_MODE_SELECT, &reg) != 0) {
        return -1;
    }
    reg = (reg & ~(0x07 << BIT_MEAS_CONFIG_POS)) |
          ((mode & 0x07) << BIT_MEAS_CONFIG_POS);
    return wr_mode_select(self, reg);
}

static int wr_meas_mode(struct icp201xx* self, uint8_t mode) {
    uint8_t reg;
    if (rd_reg(self, REG_MODE_SELECT, &reg) != 0) {
        return -1;
    }
    reg = (reg & ~(1 << BIT_MEAS_MODE_POS)) |
          ((mode & 0x01) << BIT_MEAS_MODE_POS);
    return wr_mode_select(self, reg);
}

static int wr_forced_meas_trigger(struct icp201xx* self, uint8_t trigger) {
    uint8_t reg;
    if (rd_reg(self, REG_MODE_SELECT, &reg) != 0) {
        return -1;
    }
    reg = (reg & ~(1 << BIT_FORCED_MEAS_TRIG_POS)) |
          ((trigger & 0x01) << BIT_FORCED_MEAS_TRIG_POS);
    return wr_mode_select(self, reg);
}

/******************************************************************************/
/* Flush FIFO                                                                 */
/******************************************************************************/

static int flush_fifo(struct icp201xx* self) {
    uint8_t val;

    if (rd_reg(self, REG_FIFO_FILL, &val) != 0) {
        return -1;
    }
    return wr_reg(self, REG_FIFO_FILL, val | 0x80);
}

/******************************************************************************/
/* OTP single-byte read helper                                                */
/******************************************************************************/

static int otp_read_byte(struct icp201xx* self, uint8_t addr, uint8_t* val) {
    uint8_t status;

    wr_reg(self, REG_OTP_ADDR, addr);
    wr_reg(self, REG_OTP_CMD, 0x10);

    for (int i = 0; i < MAX_POLL_RETRIES; i++) {
        if (rd_reg(self, REG_OTP_STATUS, &status) != 0) {
            return -1;
        }
        if (status == 0) {
            return rd_reg(self, REG_OTP_RD_DATA, val);
        }
        if (i == MAX_POLL_RETRIES - 1) {
            return -1;
        }
    }

    return -1;
}

/******************************************************************************/
/* OTP bootup configuration (required for A1 silicon, skipped for B2)         */
/******************************************************************************/

static int otp_bootup(struct icp201xx* self) {
    uint8_t version;
    uint8_t bootup;
    uint8_t offset, gain, hfosc;
    uint8_t reg;

    /* Check version — B2 silicon doesn't need OTP bootup */
    if (rd_reg(self, REG_VERSION, &version) != 0) {
        return -1;
    }
    if (version == 0xB2) {
        return 0;
    }

    /* Check if bootup was already done */
    if (rd_reg(self, REG_OTP_STATUS2, &bootup) != 0) {
        return -1;
    }
    if (bootup & 0x01) {
        return 0;
    }

    /* Set power mode active for OTP access (bit 2 = 1) */
    if (wr_mode_select(self, 0x04) != 0) {
        return -1;
    }
    osDelay(pdMS_TO_TICKS(4));

    /* Unlock main registers */
    if (wr_reg(self, REG_MASTER_LOCK, 0x1F) != 0) {
        return -1;
    }

    /* Enable OTP and write switch */
    if (wr_reg(self, REG_OTP_CFG1, 0x03) != 0) {
        return -1;
    }

    /* Toggle OTP reset */
    if (rd_reg(self, REG_OTP_DBG2, &reg) != 0) {
        return -1;
    }
    if (wr_reg(self, REG_OTP_DBG2, reg | 0x80) != 0) {
        return -1;
    }
    if (wr_reg(self, REG_OTP_DBG2, reg & ~0x80) != 0) {
        return -1;
    }

    /* Program redundant read */
    wr_reg(self, REG_OTP_MRA_LSB, 0x04);
    wr_reg(self, REG_OTP_MRA_MSB, 0x04);
    wr_reg(self, REG_OTP_MRB_LSB, 0x21);
    wr_reg(self, REG_OTP_MRB_MSB, 0x20);
    wr_reg(self, REG_OTP_MR_LSB, 0x10);
    wr_reg(self, REG_OTP_MR_MSB, 0x80);

    /* Read offset, gain, HFOSC from OTP */
    if (otp_read_byte(self, 0xF8, &offset) != 0) {
        return -1;
    }
    if (otp_read_byte(self, 0xF9, &gain) != 0) {
        return -1;
    }
    if (otp_read_byte(self, 0xFA, &hfosc) != 0) {
        return -1;
    }

    /* Disable OTP */
    wr_reg(self, REG_OTP_CFG1, 0x00);

    /* Write trim values to main registers */
    if (rd_reg(self, REG_TRIM1_MSB, &reg) == 0) {
        wr_reg(self, REG_TRIM1_MSB, (reg & 0xC0) | (offset & 0x3F));
    }
    if (rd_reg(self, REG_TRIM2_MSB, &reg) == 0) {
        wr_reg(self, REG_TRIM2_MSB, (reg & 0x8F) | ((gain & 0x07) << 4));
    }
    if (rd_reg(self, REG_TRIM2_LSB, &reg) == 0) {
        wr_reg(self, REG_TRIM2_LSB, (reg & 0x80) | (hfosc & 0x7F));
    }

    /* Lock main registers */
    wr_reg(self, REG_MASTER_LOCK, 0x00);

    /* Standby */
    wr_mode_select(self, 0x00);

    /* Mark bootup done */
    if (rd_reg(self, REG_OTP_STATUS2, &reg) == 0) {
        wr_reg(self, REG_OTP_STATUS2, reg | 0x01);
    }

    return 0;
}

/******************************************************************************/
/* Soft reset (matches inv_icp201xx_soft_reset exactly)                       */
/******************************************************************************/

static int soft_reset(struct icp201xx* self) {
    uint8_t int_status;

    if (wr_mode_select(self, 0x00) != 0) {
        return -1;
    }
    osDelay(pdMS_TO_TICKS(2));

    flush_fifo(self);

    /* set_fifo_notification_config(0, 0, 0) → fifo_config = 0 */
    wr_reg(self, REG_FIFO_CONFIG, 0x00);

    /* Mask all interrupts */
    wr_reg(self, REG_INT_MASK, 0xFF);

    /* Clear pending interrupts */
    if (rd_reg(self, REG_INT_STATUS, &int_status) == 0 && int_status) {
        wr_reg(self, REG_INT_STATUS, int_status);
    }

    return 0;
}

/******************************************************************************/
/* Config: OP_MODE4 (highest accuracy), PRES_TEMP, continuous                 */
/******************************************************************************/

static int config(struct icp201xx* self) {
    uint8_t int_status;

    /* Standby */
    if (wr_mode_select(self, 0x00) != 0) {
        return -1;
    }

    /* Flush FIFO */
    flush_fifo(self);

    /* Clear interrupts */
    if (rd_reg(self, REG_INT_STATUS, &int_status) == 0 && int_status) {
        wr_reg(self, REG_INT_STATUS, int_status);
    }

    /* forced_meas_trigger = STANDBY (0) */
    wr_forced_meas_trigger(self, 0);

    /* power_mode = NORMAL (0) */
    wr_pow_mode(self, 0);

    /* fifo_readout_mode = PRES_TEMP (0) */
    wr_fifo_readout_mode(self, 0);

    /* meas_config = OP_MODE4 (highest accuracy) */
    wr_meas_config(self, 4);

    /* meas_mode = CONTINUOUS (1) */
    wr_meas_mode(self, 1);

    return 0;
}

/******************************************************************************/
/* Warmup: discard first 14 FIR filter packets (matches app_warmup)           */
/******************************************************************************/

static int warmup(struct icp201xx* self) {
    uint8_t fifo_count;
    uint8_t int_status;

    for (int i = 0; i < MAX_POLL_RETRIES * 10; i++) {
        if (rd_reg(self, REG_FIFO_FILL, &fifo_count) != 0) {
            return -1;
        }
        if ((fifo_count & FIFO_LEVEL_MASK) >= WARMUP_PACKETS) {
            /* Discard all warmup packets */
            flush_fifo(self);

            /* Clear interrupts */
            if (rd_reg(self, REG_INT_STATUS, &int_status) == 0 && int_status) {
                wr_reg(self, REG_INT_STATUS, int_status);
            }
            return 0;
        }
        osDelay(pdMS_TO_TICKS(2));
    }

    return -1;
}

/******************************************************************************/
/* Public API                                                                 */
/******************************************************************************/

void icp201xx_init(struct icp201xx* self, I2C_HandleTypeDef* i2c) {
    memset(self, 0, sizeof(*self));
    self->i2c = i2c;
}

/******************************************************************************/
/* Matches Arduino begin(): init → soft_reset → devid check → OTP bootup     */
/******************************************************************************/

int icp201xx_is_available(struct icp201xx* self) {
    uint8_t id;

    /* inv_icp201xx_init: dummy write to 0xEE until success */
    for (int i = 0; i < MAX_POLL_RETRIES; i++) {
        if (wr_reg(self, 0xEE, 0xF0) == 0) {
            break;
        }
        osDelay(pdMS_TO_TICKS(1));
    }

    /* inv_icp201xx_soft_reset */
    if (soft_reset(self) != 0) {
        return -1;
    }

    /* inv_icp201xx_get_devid_version: check WHOAMI */
    if (rd_reg(self, REG_DEVICE_ID, &id) != 0) {
        return -1;
    }
    if (id != EXPECTED_DEVICE_ID) {
        return -1;
    }

    /* inv_icp201xx_OTP_bootup_cfg */
    if (otp_bootup(self) != 0) {
        return -1;
    }

    return 0;
}

/******************************************************************************/
/* Matches Arduino start() + getData():                                       */
/*   soft_reset → config → warmup → wait for data → read FIFO                */
/******************************************************************************/

int icp201xx_read(struct icp201xx* self, int32_t* pressure) {
    uint8_t reg;
    uint8_t fifo_level;
    uint8_t fifo_data[6];
    int32_t raw_press;

    /*
     * After power cycle, is_available() already did:
     *   init → soft_reset → devid → OTP bootup
     *
     * Now do start() sequence:
     *   soft_reset → config → warmup → read
     */

    /* soft_reset (Arduino: start() calls soft_reset again) */
    if (soft_reset(self) != 0) {
        return -1;
    }

    /* config: MODE0 continuous PRES_TEMP */
    if (config(self) != 0) {
        return -1;
    }

    /* Warmup: discard first 14 packets (FIR filter settling) */
    if (warmup(self) != 0) {
        return -1;
    }

    /* Wait for fresh data after warmup */
    for (int i = 0; i < MAX_POLL_RETRIES; i++) {
        if (rd_reg(self, REG_FIFO_FILL, &fifo_level) != 0) {
            return -1;
        }
        if ((fifo_level & FIFO_LEVEL_MASK) > 0) {
            break;
        }
        if (i == MAX_POLL_RETRIES - 1) {
            return -1;
        }
        osDelay(pdMS_TO_TICKS(5));
    }

    /* Read 6 bytes from FIFO (3 pressure + 3 temperature) */
    if (rd_reg_multi(self, REG_FIFO_BASE, fifo_data, 6) != 0) {
        return -1;
    }

    /* Dummy read to register 0x00 (required after FIFO read for I2C) */
    rd_reg(self, 0x00, &reg);

    /* Parse 20-bit pressure (little-endian in FIFO) */
    raw_press = (int32_t)(((fifo_data[2] & 0x0F) << 16) | (fifo_data[1] << 8) |
                          fifo_data[0]);

    /* Sign-extend 20-bit to 32-bit */
    if (raw_press & 0x080000) {
        raw_press |= (int32_t)0xFFF00000;
    }

    /*
     * Convert to centipascals (Pa * 100):
     * P(kPa) = (raw / 2^17) * 40 + 70
     * P(Pa)  = (raw / 131072) * 40000 + 70000
     * P(cPa) = (raw * 4000000 / 131072) + 7000000
     *        = (raw * 62500 / 2048) + 7000000
     *
     * Use 64-bit intermediate to avoid overflow.
     */
    *pressure = (int32_t)(((int64_t)raw_press * 62500) / 2048) + 7000000;

    /* Return to standby */
    wr_mode_select(self, 0x00);

    return 0;
}
