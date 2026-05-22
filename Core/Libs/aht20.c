/*
 * AHT20 humidity and temperature sensor
 */

#include "aht20.h"

#include <string.h>

#include "cmsis_os.h"

#define I2C_ADDRESS 0x70

#define I2C_STATUS_CALIB_MASK 0x08  // 0000 1000
#define I2C_STATUS_BUSY_MASK  0x80  // 1000 0000

#define CRC_INITIAL_VALUE 0xFF
#define CRC_POLYNOMIAL    0x31

#define I2C_TIMEOUT 50
#define AHT20_POLL_DELAY_MS 10
#define AHT20_POLL_RETRIES  20
#define AHT20_RESET_DELAY_MS 20

#define POW2_20  1048576
#define COEFF_RH (1000 * 100)
#define COEFF_T1 (1000 * 200)
#define COEFF_T2 (1000 * 50)

#define I2C_CMD_STATUS 0x71
#define I2C_CMD_RESET  0xBA
static const uint8_t cmd_init[] = {0xBE, 0x08, 0x00};
static const uint8_t cmd_meas[] = {0xAC, 0x33, 0x00};

void aht20_init(struct aht20* self, I2C_HandleTypeDef* i2c) {
    memset(self, 0, sizeof(*self));
    self->i2c = i2c;
}

static int read_status(struct aht20* self, uint8_t* status) {
    return HAL_I2C_Mem_Read(self->i2c,
                            I2C_ADDRESS,
                            I2C_CMD_STATUS,
                            I2C_MEMADD_SIZE_8BIT,
                            status,
                            1,
                            I2C_TIMEOUT) == HAL_OK
               ? 0
               : -1;
}

static int wait_ready(struct aht20* self) {
    uint8_t status;

    for (int i = 0; i < AHT20_POLL_RETRIES; i++) {
        if (read_status(self, &status) != 0) {
            return -1;
        }
        if ((status & I2C_STATUS_BUSY_MASK) == 0) {
            return 0;
        }
        osDelay(pdMS_TO_TICKS(AHT20_POLL_DELAY_MS));
    }

    return -1;
}

static void soft_reset(struct aht20* self) {
    (void)HAL_I2C_Master_Transmit(self->i2c,
                                  I2C_ADDRESS,
                                  (uint8_t*)&(uint8_t){I2C_CMD_RESET},
                                  1,
                                  I2C_TIMEOUT);
    osDelay(pdMS_TO_TICKS(AHT20_RESET_DELAY_MS));
}

static int fail_and_reset(struct aht20* self) {
    soft_reset(self);
    return -1;
}

static uint8_t calc_crc8(uint8_t* data, size_t size) {
    uint8_t crc = CRC_INITIAL_VALUE;

    for (size_t i = 0; i < size; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ CRC_POLYNOMIAL;
            } else {
                crc = (crc << 1);
            }
        }
    }

    return crc;
}

/******************************************************************************/
int aht20_read(struct aht20* self, int32_t* temp, int32_t* hum) {
    HAL_StatusTypeDef status;
    uint32_t raw_temp;
    uint32_t raw_hum;
    uint8_t buf[7];

    /* Status */
    if (read_status(self, buf) != 0) {
        return fail_and_reset(self);
    }

    /* Not calibrated */
    if ((buf[0] & I2C_STATUS_CALIB_MASK) == 0) {
        status = HAL_I2C_Master_Transmit(self->i2c,
                                         I2C_ADDRESS,
                                         (uint8_t*)cmd_init,
                                         sizeof(cmd_init),
                                         I2C_TIMEOUT);
        if (status != HAL_OK) {
            return fail_and_reset(self);
        }
        osDelay(pdMS_TO_TICKS(40));
        if (wait_ready(self) != 0) {
            return fail_and_reset(self);
        }
    }

    /* Trigger measurement */
    status = HAL_I2C_Master_Transmit(self->i2c,
                                     I2C_ADDRESS,
                                     (uint8_t*)cmd_meas,
                                     sizeof(cmd_meas),
                                     I2C_TIMEOUT);
    if (status != HAL_OK) {
        return fail_and_reset(self);
    }
    osDelay(pdMS_TO_TICKS(200));
    if (wait_ready(self) != 0) {
        return fail_and_reset(self);
    }

    /* Receive data */
    status = HAL_I2C_Master_Receive(self->i2c,
                                    I2C_ADDRESS,
                                    buf,
                                    sizeof(buf),
                                    I2C_TIMEOUT);
    if (status != HAL_OK) {
        return fail_and_reset(self);
    }

    /* CRC */
    if (calc_crc8(buf, 6) != buf[6]) {
        return fail_and_reset(self);
    }

    raw_hum  = (uint32_t)buf[1] << 12 | (uint32_t)buf[2] << 4 | buf[3] >> 4;
    raw_temp = (uint32_t)buf[3] << 16 | (uint32_t)buf[4] << 8 | buf[5];
    raw_temp &= 0x000FFFFF;

    *hum  = (int32_t)((int64_t)raw_hum * COEFF_RH / POW2_20);
    *temp = (int32_t)((int64_t)raw_temp * COEFF_T1 / POW2_20 - COEFF_T2);

    return 0;
}
