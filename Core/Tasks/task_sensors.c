/*
 * Sensors task
 */

#include <stdlib.h>
#include <string.h>

#include "actual.h"
#include "aht20.h"
#include "as5600.h"
#include "avoltage.h"
#include "freqmeter.h"
#include "icp201xx.h"
#include "logger.h"
#include "params.h"
#include "ptasks.h"
#include "rtctime.h"
#include "sensorq.h"

#define TAG "SENSORS"

//#define AVOLTAGE_CALIB
#define ANGLE_MAX 360000

enum {
    AVAIL_VOL      = 0x01,
    AVAIL_HALL      = 0x02,
    AVAIL_AHT20    = 0x08,
    AVAIL_ICP201XX = 0x40,
    AVAIL_AS5600   = 0x20,
};

void task_sensors(void* argument) {
    struct task_sensors_ctx* ctx = argument;
    int avail                    = 0;
    int ret;

    int32_t temperature        = -1;
    int32_t humidity           = -1;
    int32_t pressure           = -1;
    int32_t wind_direction     = -1;
    int32_t wind_direction_raw = -1;
    int voltage                = -1;
    uint32_t ts;

    char* tmp;

    /* Check sensor availability */
#ifdef AVOLTAGE_CALIB
    ret = avoltage_calib(ctx->voltage);
    if (!ret) {
        avail |= AVAIL_VOL;
    }
#endif
    avail |= AVAIL_VOL;

    ctx->aht20_on();
    ret = aht20_is_available(ctx->aht20);
    ctx->aht20_off();
    if (!ret) {
        avail |= AVAIL_AHT20;
    }

    ctx->sens_on();
    ret = as5600_is_available(ctx->as5600);
    if (!ret) {
        avail |= AVAIL_AS5600;
    }
    ret = icp201xx_is_available(ctx->icp201xx);
    if (!ret) {
        avail |= AVAIL_ICP201XX;
    }
    ctx->sens_off();

    xSemaphoreTake(ctx->actual->mutex, portMAX_DELAY);
    xSemaphoreGive(ctx->actual->mutex);

    tmp = pvPortMalloc(sizeof(avail) * 8 + 2);
    if (tmp) {
        itoa(avail, tmp, 2);
        strcat(tmp, "b");
        LOG_I(ctx->logger, TAG, tmp);
        vPortFree(tmp);
    }

    for (;;) {
        xEventGroupWaitBits(task_events,
                            TASK_EVENT_SENSORS_START,
                            pdTRUE,
                            pdFALSE,
                            portMAX_DELAY);

        led_blink(2);

        if (avail & AVAIL_VOL) {
            voltage = avoltage(ctx->voltage);
        }
        if (avail & AVAIL_AHT20) {
            ctx->aht20_on();
            aht20_read(ctx->aht20, &temperature, &humidity);
            ctx->aht20_off();
        }
        if ((avail & AVAIL_AS5600) || (avail & AVAIL_ICP201XX)) {
            ctx->sens_on();
            if (avail & AVAIL_AS5600) {
                wind_direction_raw = as5600_read(ctx->as5600);
                if (wind_direction_raw >= 0) {
                    if (wind_direction_raw >= params.offset_angle) {
                        wind_direction =
                            wind_direction_raw - params.offset_angle;
                    } else {
                        wind_direction = ANGLE_MAX + wind_direction_raw -
                                         params.offset_angle;
                    }
                }
            }
            if (avail & AVAIL_ICP201XX) {
                icp201xx_read(ctx->icp201xx, &pressure);
            }
            ctx->sens_off();
        }

        xSemaphoreTake(ctx->actual->mutex, portMAX_DELAY);
        ctx->actual->voltage = voltage;
        ctx->actual->temperature = temperature;
        ctx->actual->humidity = humidity;
        ctx->actual->wind_direction = wind_direction;
        ctx->actual->pressure = pressure;
        xSemaphoreGive(ctx->actual->mutex);

        {
            struct freqmeter_accum ca;
            freqmeter_accum_read(ctx->cnt, &ca);

            struct sensor_record rec = {
                .timestamp = timestamp,
                .voltage = (uint16_t)voltage,
                .temperature = (int16_t)(temperature / 10),
                .humidity = (uint16_t)(humidity / 10),
                .pressure = (uint16_t)(pressure / 1000),
                .wind_direction = (uint16_t)(wind_direction / 10),
                .wind_speed_avg = (uint16_t)ca.avg,
                .wind_speed_min = (uint16_t)ca.min,
                .wind_speed_max = (uint16_t)ca.max,
            };

            sensorq_push(ctx->queue, &rec);
        }

        xEventGroupSetBits(task_events, TASK_EVENT_SENSORS_DONE);
    }
}
