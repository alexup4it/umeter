/*
 * Task manager (scheduler pipeline)
 */

#include <stdlib.h>
#include <string.h>

#include "logger.h"
#include "params.h"
#include "ptasks.h"
#include "rtctime.h"
#include "sensorq.h"

EventGroupHandle_t task_events;

/* When queue backlog exceeds this threshold, slow down sensor reads */
#define SENSORQ_BACKLOG_THRESHOLD 20
#define SENSORQ_SLOW_PERIOD       300 /* seconds (5 min) */

void task_manager_init(void) {
    task_events = xEventGroupCreate();
}

void task_manager_run(struct task_default_ctx* ctx) {
    uint32_t tick   = 0;
    TickType_t wake = xTaskGetTickCount();

    /*
     * Base period = period_anemometer (smallest interval, GCD of all).
     * Each task receives TASK_EVENT_*_START, does its work,
     * sets TASK_EVENT_*_DONE. Scheduler waits for completion before
     * triggering the next task, guaranteeing:
     *   anemometer finishes → sensors starts → sensors finishes → net starts
     */
    uint32_t period_base = params.period_anemometer ? params.period_anemometer
                           : params.period_sensors  ? params.period_sensors
                                                    : params.period_upload;

    if (period_base == 0) {
        vTaskSuspend(NULL);
        return;
    }

    uint32_t period_sensors =
        params.period_sensors
            ? (params.period_sensors / period_base) * period_base
            : 0;

    uint32_t sensors_step  = period_sensors ? period_sensors : period_base;
    uint32_t period_upload = 0;
    if (params.period_upload && sensors_step) {
        period_upload = (params.period_upload / sensors_step) * sensors_step;
    }

    /* How many base ticks between subsystem activations (0 = disabled) */
    uint32_t anemometer_every = params.period_anemometer ? 1 : 0;
    uint32_t sensors_every = period_sensors ? period_sensors / period_base : 0;
    uint32_t upload_every  = period_upload ? period_upload / period_base : 0;

    /* Slow-mode sensor interval (queue backlog) */
    uint32_t sensors_slow_period =
        (SENSORQ_SLOW_PERIOD / period_base) * period_base;
    uint32_t sensors_slow_every =
        sensors_slow_period ? sensors_slow_period / period_base : sensors_every;

    /* Kick off the network task so it can sync time before we start
     * scheduling sensor measurements (timestamps depend on valid time). */
    xEventGroupSetBits(task_events, TASK_EVENT_NET_START);
    xEventGroupWaitBits(task_events,
                        TASK_EVENT_TIME_SYNCED,
                        pdTRUE,
                        pdFALSE,
                        portMAX_DELAY);

    for (;;) {
        xEventGroupSetBits(task_events, TASK_EVENT_WATCHDOG_START);

        rtctime_read();

        int do_anemometer = anemometer_every && (tick % anemometer_every == 0);
        int do_upload     = upload_every && (tick % upload_every == 0);

        /* Adaptive sensor interval: slow down when queue is backing up */
        uint32_t now_sensors_every = sensors_every;
        if (ctx->sensorq &&
            sensorq_count(ctx->sensorq) > SENSORQ_BACKLOG_THRESHOLD) {
            now_sensors_every = sensors_slow_every;
        }
        int do_sensors = now_sensors_every && (tick % now_sensors_every == 0);

        /* --- Sequential pipeline: anemometer → sensors → net --- */

        if (do_anemometer) {
            xEventGroupSetBits(task_events, TASK_EVENT_ANEMOMETER_START);
            xEventGroupWaitBits(task_events,
                                TASK_EVENT_ANEMOMETER_DONE,
                                pdTRUE,
                                pdFALSE,
                                pdMS_TO_TICKS(period_base * 1000U));
        }

        if (do_sensors) {
            xEventGroupSetBits(task_events, TASK_EVENT_SENSORS_START);
            xEventGroupWaitBits(task_events,
                                TASK_EVENT_SENSORS_DONE,
                                pdTRUE,
                                pdFALSE,
                                pdMS_TO_TICKS(period_sensors * 1000U));
        }

        if (do_upload) {
            xEventGroupSetBits(task_events, TASK_EVENT_NET_START);
        }

        vTaskDelayUntil(&wake, pdMS_TO_TICKS(period_base * 1000U));
        tick++;
    }
}
