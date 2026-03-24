/*
 * ICP-20100 barometric pressure sensor
 *
 * Simplified driver for forced single-measurement mode.
 * Based on TDK InvenSense ICP-201xx reference driver.
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

/* Operation mode 0 (MODE0): highest accuracy */
#define OP_MODE0 0

/* FIFO readout: pressure + temperature */
#define FIFO_READOUT_PRES_TEMP 0

/* Power mode normal */
#define POWER_MODE_NORMAL (1 << BIT_POWER_MODE_POS)

/* OTP status2 bootup mask */
#define BOOTUP_STATUS_MASK 0x01

/* FIFO level mask */
#define FIFO_LEVEL_MASK 0x1F

/* Max retries for polling loops */
#define MAX_POLL_RETRIES 200

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

/* Wait for MODE_SELECT register to become accessible */
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

/* Write MODE_SELECT with mode-sync wait */
static int wr_mode_select(struct icp201xx* self, uint8_t val) {
    if (wait_mode_sync(self) != 0) {
        return -1;
    }
    return wr_reg(self, REG_MODE_SELECT, val);
}

/******************************************************************************/
/* OTP bootup configuration (required for A1 silicon)                         */
/******************************************************************************/

static int otp_bootup(struct icp201xx* self) {
    uint8_t version;
    uint8_t bootup;
    uint8_t otp_status;
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
    if (bootup & BOOTUP_STATUS_MASK) {
        return 0;
    }

    /* Enter power mode */
    if (wr_mode_select(self, POWER_MODE_NORMAL) != 0) {
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
    if (wr_reg(self, REG_OTP_MRA_LSB, 0x04) != 0) {
        return -1;
    }
    if (wr_reg(self, REG_OTP_MRA_MSB, 0x04) != 0) {
        return -1;
    }
    if (wr_reg(self, REG_OTP_MRB_LSB, 0x21) != 0) {
        return -1;
    }
    if (wr_reg(self, REG_OTP_MRB_MSB, 0x20) != 0) {
        return -1;
    }
    if (wr_reg(self, REG_OTP_MR_LSB, 0x10) != 0) {
        return -1;
    }
    if (wr_reg(self, REG_OTP_MR_MSB, 0x80) != 0) {
        return -1;
    }

    /* Read offset from OTP address 0xF8 */
    if (wr_reg(self, REG_OTP_ADDR, 0xF8) != 0) {
        return -1;
    }
    if (wr_reg(self, REG_OTP_CMD, 0x10) != 0) {
        return -1;
    }
    for (int i = 0; i < MAX_POLL_RETRIES; i++) {
        if (rd_reg(self, REG_OTP_STATUS, &otp_status) != 0) {
            return -1;
        }
        if (otp_status == 0) {
            break;
        }
        if (i == MAX_POLL_RETRIES - 1) {
            return -1;
        }
    }
    if (rd_reg(self, REG_OTP_RD_DATA, &offset) != 0) {
        return -1;
    }

    /* Read gain from OTP address 0xF9 */
    if (wr_reg(self, REG_OTP_ADDR, 0xF9) != 0) {
        return -1;
    }
    if (wr_reg(self, REG_OTP_CMD, 0x10) != 0) {
        return -1;
    }
    for (int i = 0; i < MAX_POLL_RETRIES; i++) {
        if (rd_reg(self, REG_OTP_STATUS, &otp_status) != 0) {
            return -1;
        }
        if (otp_status == 0) {
            break;
        }
        if (i == MAX_POLL_RETRIES - 1) {
            return -1;
        }
    }
    if (rd_reg(self, REG_OTP_RD_DATA, &gain) != 0) {
        return -1;
    }

    /* Read HFOSC from OTP address 0xFA */
    if (wr_reg(self, REG_OTP_ADDR, 0xFA) != 0) {
        return -1;
    }
    if (wr_reg(self, REG_OTP_CMD, 0x10) != 0) {
        return -1;
    }
    for (int i = 0; i < MAX_POLL_RETRIES; i++) {
        if (rd_reg(self, REG_OTP_STATUS, &otp_status) != 0) {
            return -1;
        }
        if (otp_status == 0) {
            break;
        }
        if (i == MAX_POLL_RETRIES - 1) {
            return -1;
        }
    }
    if (rd_reg(self, REG_OTP_RD_DATA, &hfosc) != 0) {
        return -1;
    }

    /* Disable OTP */
    if (wr_reg(self, REG_OTP_CFG1, 0x00) != 0) {
        return -1;
    }

    /* Write trim values to main registers */
    /* offset: TRIM1_MSB[5:0] */
    if (rd_reg(self, REG_TRIM1_MSB, &reg) != 0) {
        return -1;
    }
    if (wr_reg(self, REG_TRIM1_MSB, (reg & 0xC0) | (offset & 0x3F)) != 0) {
        return -1;
    }

    /* gain: TRIM2_MSB[6:4] */
    if (rd_reg(self, REG_TRIM2_MSB, &reg) != 0) {
        return -1;
    }
    if (wr_reg(self, REG_TRIM2_MSB, (reg & 0x8F) | ((gain & 0x07) << 4)) != 0) {
        return -1;
    }

    /* hfosc: TRIM2_LSB[6:0] */
    if (rd_reg(self, REG_TRIM2_LSB, &reg) != 0) {
        return -1;
    }
    if (wr_reg(self, REG_TRIM2_LSB, (reg & 0x80) | (hfosc & 0x7F)) != 0) {
        return -1;
    }

    /* Lock main registers */
    if (wr_reg(self, REG_MASTER_LOCK, 0x00) != 0) {
        return -1;
    }

    /* Standby */
    if (wr_mode_select(self, 0x00) != 0) {
        return -1;
    }

    /* Mark bootup done */
    if (rd_reg(self, REG_OTP_STATUS2, &reg) != 0) {
        return -1;
    }
    if (wr_reg(self, REG_OTP_STATUS2, reg | BOOTUP_STATUS_MASK) != 0) {
        return -1;
    }

    return 0;
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
/* Public API                                                                 */
/******************************************************************************/

void icp201xx_init(struct icp201xx* self, I2C_HandleTypeDef* i2c) {
    memset(self, 0, sizeof(*self));
    self->i2c = i2c;
}

/******************************************************************************/
int icp201xx_is_available(struct icp201xx* self) {
    uint8_t id;

    /* Dummy write to 0xEE to ensure sensor is initialized */
    wr_reg(self, 0xEE, 0xF0);
    osDelay(pdMS_TO_TICKS(1));

    if (rd_reg(self, REG_DEVICE_ID, &id) != 0) {
        return -1;
    }
    if (id != EXPECTED_DEVICE_ID) {
        return -1;
    }

    /* Run OTP bootup configuration */
    if (otp_bootup(self) != 0) {
        return -1;
    }

    return 0;
}

/******************************************************************************/
int icp201xx_read(struct icp201xx* self, int32_t* pressure) {
    uint8_t reg;
    uint8_t fifo_level;
    uint8_t fifo_data[6];
    int32_t raw_press;

    /*
     * Soft-reset sequence:
     * 1. Standby
     * 2. Flush FIFO
     * 3. Clear interrupts
     */
    if (wr_mode_select(self, 0x00) != 0) {
        return -1;
    }
    osDelay(pdMS_TO_TICKS(2));

    if (flush_fifo(self) != 0) {
        return -1;
    }

    /* Mask all interrupts */
    if (wr_reg(self, REG_INT_MASK, 0xFF) != 0) {
        return -1;
    }

    /* Clear pending interrupts */
    if (rd_reg(self, REG_INT_STATUS, &reg) != 0) {
        return -1;
    }
    if (reg) {
        if (wr_reg(self, REG_INT_STATUS, reg) != 0) {
            return -1;
        }
    }

    /*
     * Configure forced measurement:
     * - OP_MODE0 (bits [7:5] = 0)
     * - Forced trigger (bit 4 = 1)
     * - Forced trigger mode (bit 3 = 0)
     * - Normal power mode (bit 2 = 1)
     * - FIFO readout: press + temp (bits [1:0] = 0)
     */
    reg = (OP_MODE0 << BIT_MEAS_CONFIG_POS) | (1 << BIT_FORCED_MEAS_TRIG_POS) |
          POWER_MODE_NORMAL | FIFO_READOUT_PRES_TEMP;
    if (wr_mode_select(self, reg) != 0) {
        return -1;
    }

    /* Wait for measurement (MODE0 takes ~25ms typical) */
    osDelay(pdMS_TO_TICKS(30));

    /* Wait for data in FIFO */
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
