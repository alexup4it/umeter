/*
 * Sensors task
 */

#include <stdlib.h>
#include <string.h>

#include "actual.h"
#include "aht20.h"
#include "as5600.h"
#include "avoltage.h"
#include "icp201xx.h"
#include "logger.h"
#include "params.h"
#include "ptasks.h"
#include "rtctime.h"

#define TAG "SENSORS"

//#define AVOLTAGE_CALIB
#define ANGLE_MAX 360000
#define VOLTAGE_MEASURE_EVERY 5
#define AHT20_READ_RETRIES    3
#define AHT20_RETRY_DELAY_MS  50
#define AHT20_TEMP_MIN_MDEGC  (-40000)
#define AHT20_TEMP_MAX_MDEGC  85000
#define AHT20_HUM_MIN_MPERMIL 0
#define AHT20_HUM_MAX_MPERMIL 100000

static bool aht20_values_valid(int32_t temperature, int32_t humidity) {
    return temperature >= AHT20_TEMP_MIN_MDEGC &&
           temperature <= AHT20_TEMP_MAX_MDEGC &&
           humidity >= AHT20_HUM_MIN_MPERMIL &&
           humidity <= AHT20_HUM_MAX_MPERMIL;
}

void task_sensors(void* argument) {
    struct task_sensors_ctx* ctx = argument;
    int ret;

    int32_t temperature        = -1;
    int32_t humidity           = -1;
    int32_t pressure           = -1;
    int32_t wind_direction     = -1;
    int32_t wind_direction_raw = -1;
    int voltage                = -1;
    int cached_voltage         = -1;
    uint32_t voltage_skip      = 0;

    LOG_I(ctx->logger, TAG, "init");

#ifdef AVOLTAGE_CALIB
    ret = avoltage_calib(ctx->voltage);
    if (!ret) {
        LOG_I(ctx->logger, TAG, "avoltage calib ok");
    } else {
        LOG_E(ctx->logger, TAG, "avoltage calib fail");
    }
#endif

    for (;;) {
        xEventGroupWaitBits(task_events,
                            TASK_EVENT_SENSORS_START,
                            pdTRUE,
                            pdFALSE,
                            portMAX_DELAY);

        led_blink(2);

        uint32_t current_available = 0;

        /* Voltage */
        if (cached_voltage < 0 || voltage_skip == 0) {
            voltage = avoltage(ctx->voltage);
            if (voltage < 0) {
                LOG_E(ctx->logger, TAG, "voltage fail");
                voltage_skip = 0;
            } else {
                cached_voltage = voltage;
                voltage_skip   = VOLTAGE_MEASURE_EVERY - 1;
            }
        } else {
            voltage = cached_voltage;
            voltage_skip--;
        }

        if (voltage >= 0) {
            current_available |= ACTUAL_VOLTAGE_AVAIL;
        }

        /* AHT20: temperature + humidity */
        ret = -1;
        for (int attempt = 0; attempt < AHT20_READ_RETRIES; attempt++) {
            ctx->aht20_on();
            ret = aht20_read(ctx->aht20, &temperature, &humidity);
            ctx->aht20_off();

            if (ret == 0) {
                break;
            }

            osDelay(pdMS_TO_TICKS(AHT20_RETRY_DELAY_MS));
        }
        if (ret != 0) {
            LOG_E(ctx->logger, TAG, "aht20 fail");
        } else if (!aht20_values_valid(temperature, humidity)) {
            LOG_E(ctx->logger, TAG, "aht20 invalid values");
        } else {
            current_available |=
                ACTUAL_TEMPERATURE_AVAIL | ACTUAL_HUMIDITY_AVAIL;
        }

        /* AS5600 + ICP-201xx share one I2C bus */
        ctx->sens_on();

        /* ICP-201xx: pressure (requires begin() before each read) */
        ret = icp201xx_is_available(ctx->icp201xx);
        if (ret != 0) {
            LOG_E(ctx->logger, TAG, "icp begin fail");
        } else {
            ret = icp201xx_read(ctx->icp201xx, &pressure);
            if (ret != 0) {
                LOG_E(ctx->logger, TAG, "icp read fail");
            } else {
                current_available |= ACTUAL_PRESSURE_AVAIL;
            }
        }

        /* AS5600: wind direction */
        wind_direction_raw = as5600_read(ctx->as5600);
        if (wind_direction_raw >= 0) {
            if (wind_direction_raw >= params.offset_angle) {
                wind_direction = wind_direction_raw - params.offset_angle;
            } else {
                wind_direction =
                    ANGLE_MAX + wind_direction_raw - params.offset_angle;
            }
            current_available |= ACTUAL_WIND_DIR_AVAIL;
        } else {
            LOG_E(ctx->logger, TAG, "as5600 fail");
        }

        ctx->sens_off();

        xSemaphoreTake(ctx->actual->mutex, portMAX_DELAY);
        ctx->actual->available &=
            ~(ACTUAL_VOLTAGE_AVAIL | ACTUAL_TEMPERATURE_AVAIL |
              ACTUAL_HUMIDITY_AVAIL | ACTUAL_PRESSURE_AVAIL |
              ACTUAL_WIND_DIR_AVAIL);
        ctx->actual->available |= current_available;

        if (current_available & ACTUAL_VOLTAGE_AVAIL) {
            ctx->actual->voltage = voltage;
        }
        if (current_available & ACTUAL_TEMPERATURE_AVAIL) {
            ctx->actual->temperature = temperature;
            ctx->actual->humidity    = humidity;
        }
        if (current_available & ACTUAL_PRESSURE_AVAIL) {
            ctx->actual->pressure = pressure;
        }
        if (current_available & ACTUAL_WIND_DIR_AVAIL) {
            ctx->actual->wind_direction = wind_direction;
        }
        xSemaphoreGive(ctx->actual->mutex);

        xEventGroupSetBits(task_events, TASK_EVENT_SENSORS_DONE);
    }
}
