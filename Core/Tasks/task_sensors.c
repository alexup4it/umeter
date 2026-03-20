/*
 * Sensors task
 */

#include <stdlib.h>
#include <string.h>

#include "actual.h"
#include "aht20.h"
#include "as5600.h"
#include "avoltage.h"
#include "counter.h"
#include "logger.h"
#include "params.h"
#include "ptasks.h"
#include "rtctime.h"
#include "sensorq.h"
#ifdef LOGGER
#    define TAG "SENSORS"
#endif

//#define AVOLTAGE_CALIB
#define ANGLE_MAX 360000
enum {
    AVAIL_VOL    = 0x01,
    AVAIL_CNT    = 0x02,
    AVAIL_TMPx75 = 0x04,
    AVAIL_AHT20  = 0x08,
    AVAIL_DIST   = 0x10,
    AVAIL_AS5600 = 0x20,
};

enum {
    DRDY_VOL  = 0x01,
    DRDY_CNT  = 0x02,
    DRDY_TMP  = 0x04,
    DRDY_HUM  = 0x08,
    DRDY_DIST = 0x10,
    DRDY_ANG  = 0x20,
};

void task_sensors(void* argument) {
    struct task_sensors_ctx* ctx = argument;
    int avail                    = 0;
    int drdy;
    int ret;

    int32_t temperature = 0;
    int32_t humidity    = 0;
    int32_t angle_wo    = 0;
    int32_t angle       = 0;
    int voltage         = 0;
    uint32_t ts;

    char* tmp;

    /* Check sensor availability */
#ifdef AVOLTAGE_CALIB
    ret = avoltage_calib(ctx->avlt);
    if (!ret) {
        avail |= AVAIL_VOL;
    }
#endif
    avail |= AVAIL_VOL;

    ctx->aht20_on();
    ret = aht20_is_available(ctx->aht);
    ctx->aht20_off();
    if (!ret) {
        avail |= AVAIL_AHT20;
    }

    ctx->as5600_on();
    ret = as5600_is_available(ctx->pot);
    ctx->as5600_off();
    if (!ret) {
        avail |= AVAIL_AS5600;
    }

    xSemaphoreTake(ctx->actual->mutex, portMAX_DELAY);
    ctx->actual->avail = avail;
    xSemaphoreGive(ctx->actual->mutex);

#ifdef LOGGER
    tmp = pvPortMalloc(sizeof(avail) * 8 + 2);
    if (tmp) {
        itoa(avail, tmp, 2);
        strcat(tmp, "b");
        logger_add_str(ctx->logger, TAG, false, tmp);
        vPortFree(tmp);
    }
#endif

    for (;;) {
        xEventGroupWaitBits(task_events,
                            TASK_EVENT_SENSORS_START,
                            pdTRUE,
                            pdFALSE,
                            portMAX_DELAY);

        led_blink(3);

        drdy = 0;

        if (avail & AVAIL_VOL) {
            ctx->avoltage_on();
            voltage = avoltage(ctx->avlt);
            ctx->avoltage_off();
            if (voltage >= 0) {
                drdy |= DRDY_VOL;
            }
        }
        if (avail & AVAIL_AHT20) {
            ctx->aht20_on();
            ret = aht20_read(ctx->aht, &temperature, &humidity);
            ctx->aht20_off();
            if (!ret) {
                drdy |= DRDY_TMP | DRDY_HUM;
            }
        }
        if (avail & AVAIL_AS5600) {
            ctx->as5600_on();
            angle = as5600_read(ctx->pot);
            ctx->as5600_off();
            if (angle >= 0) {
                drdy |= DRDY_ANG;

                if (angle >= params.offset_angle) {
                    angle_wo = angle - params.offset_angle;
                } else {
                    angle_wo = ANGLE_MAX + angle - params.offset_angle;
                }
            }
        }

        ts = timestamp;

        xSemaphoreTake(ctx->actual->mutex, portMAX_DELAY);
        if (drdy & DRDY_VOL) {
            ctx->actual->voltage = voltage;
        }
        if (drdy & DRDY_TMP) {
            ctx->actual->temperature = temperature;
        }
        if (drdy & DRDY_HUM) {
            ctx->actual->humidity = humidity;
        }
        if (drdy & DRDY_ANG) {
            ctx->actual->angle = angle_wo;
        }
        xSemaphoreGive(ctx->actual->mutex);

        {
            struct sensor_record rec;
            struct counter_accum ca;

            memset(&rec, 0, sizeof(rec));
            rec.timestamp = ts;
            if (drdy & DRDY_VOL) {
                rec.voltage = (uint16_t)voltage;
            }
            if (drdy & DRDY_TMP) {
                rec.temperature = (int16_t)(temperature / 10);
            }
            if (drdy & DRDY_HUM) {
                rec.humidity = (uint16_t)(humidity / 10);
            }
            if (drdy & DRDY_ANG) {
                rec.angle = (uint16_t)(angle_wo / 10);
            }

            counter_accum_read(ctx->cnt, &ca);
            rec.count_avg = (uint16_t)ca.avg;
            rec.count_min = (uint16_t)ca.min;
            rec.count_max = (uint16_t)ca.max;

            sensorq_push(ctx->queue, &rec);
        }

        xEventGroupSetBits(task_events, TASK_EVENT_SENSORS_DONE);
    }
}
