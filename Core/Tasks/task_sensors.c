/*
 * Sensors task
 */

#include <stdlib.h>
#include <string.h>

#include "actual.h"
#include "aht20.h"
#include "as5600.h"
#include "atomic.h"
#include "avoltage.h"
#include "counter.h"
#include "iwdg.h"
#include "logger.h"
#include "mqueue.h"
#include "params.h"
#include "ptasks.h"
#include "rtctime.h"
#ifdef LOGGER
#    define TAG "SENSORS"
#endif

static struct task_sensors_ctx* s_ctx;

//#define AVOLTAGE_CALIB
#define ANGLE_MAX            360000
#define SENSORS_QUEUE_SECNUM 48

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

static mqueue_t* queue;
static volatile uint32_t events;

mqueue_t* sensors_queue(void) {
    return queue;
}

static void set_angle_offset(int32_t angle) {
    params_t uparams;

    memcpy(&uparams, &params, sizeof(uparams));
    uparams.offset_angle = angle;

    led_blink(10);

    IWDG_reset();
    vTaskSuspendAll();
    params_set(&uparams);
    xTaskResumeAll();

    params.offset_angle = angle;
}

void sensors_notify(void) {
    atomic_inc(&events);
    xEventGroupSetBits(task_events, TASK_EVENT_SENSORS_START);
}

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

    s_ctx = ctx;

    /* Create sensor record queue */
    queue = mqueue_create(SENSORS_QUEUE_SECNUM, sizeof(struct sensor_record));

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

    xSemaphoreTake(actual.mutex, portMAX_DELAY);
    actual.avail = avail;
    xSemaphoreGive(actual.mutex);

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

        xSemaphoreTake(actual.mutex, portMAX_DELAY);
        if (drdy & DRDY_VOL) {
            actual.voltage = voltage;
        }
        if (drdy & DRDY_TMP) {
            actual.temperature = temperature;
        }
        if (drdy & DRDY_HUM) {
            actual.humidity = humidity;
        }
        if (drdy & DRDY_ANG) {
            actual.angle = angle_wo;
        }
        xSemaphoreGive(actual.mutex);

        {
            struct sensor_record rec;
            struct counter_accum ca;

            memset(&rec, 0, sizeof(rec));
            rec.timestamp = ts;
            if (drdy & DRDY_VOL) {
                rec.voltage = voltage;
            }
            if (drdy & DRDY_TMP) {
                rec.temperature = temperature;
            }
            if (drdy & DRDY_HUM) {
                rec.humidity = humidity;
            }
            if (drdy & DRDY_ANG) {
                rec.angle = angle_wo;
            }

            counter_accum_read(ctx->cnt, &ca);
            rec.count_avg = ca.avg;
            rec.count_min = ca.min;
            rec.count_max = ca.max;

            mqueue_set(queue, &rec);
        }

        if (events) {
            events = 0;

            if (drdy & DRDY_ANG) {
                set_angle_offset(angle);
            }
        }

        xEventGroupSetBits(task_events, TASK_EVENT_SENSORS_DONE);
    }
}
