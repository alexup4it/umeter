/*
 * Task manager (scheduler pipeline)
 */

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

void task_manager_run(struct sensorq* sensorq) {
    uint32_t tick   = 0;
    TickType_t wake = xTaskGetTickCount();

    /*
     * Base period = mtime_count (smallest interval, GCD of all).
     * Each task receives TASK_EVENT_*_START, does its work,
     * sets TASK_EVENT_*_DONE. Scheduler waits for completion before
     * triggering the next task, guaranteeing:
     *   anemometer finishes → sensors starts → sensors finishes → net starts
     */
    uint32_t period_base = params.mtime_count  ? params.mtime_count
                           : params.period_sen ? params.period_sen
                                               : params.period_app;

    if (period_base == 0) {
        vTaskSuspend(NULL);
        return;
    }

    uint32_t period_sen =
        params.period_sen ? (params.period_sen / period_base) * period_base : 0;

    uint32_t sen_step   = period_sen ? period_sen : period_base;
    uint32_t period_app = 0;
    if (params.period_app && sen_step) {
        period_app = (params.period_app / sen_step) * sen_step;
    }

    /* How many base ticks between subsystem activations (0 = disabled) */
    uint32_t cnt_every = params.mtime_count ? 1 : 0;
    uint32_t sen_every = period_sen ? period_sen / period_base : 0;
    uint32_t app_every = period_app ? period_app / period_base : 0;

    /* Slow-mode sensor interval (queue backlog) */
    uint32_t sen_slow_period =
        (SENSORQ_SLOW_PERIOD / period_base) * period_base;
    uint32_t sen_slow_every =
        sen_slow_period ? sen_slow_period / period_base : sen_every;

    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(period_base * 1000U));

        xEventGroupSetBits(task_events, TASK_EVENT_WATCHDOG_START);

        rtctime_read();

        tick++;

        int do_cnt = cnt_every && (tick % cnt_every == 0);
        int do_app = app_every && (tick % app_every == 0);

        /* Adaptive sensor interval: slow down when queue is backing up */
        uint32_t sen_cur = sen_every;
        if (sensorq && sensorq_count(sensorq) > SENSORQ_BACKLOG_THRESHOLD) {
            sen_cur = sen_slow_every;
        }
        int do_sen = sen_cur && (tick % sen_cur == 0);

        /* --- Sequential pipeline: anemometer → sensors → net --- */

        if (do_cnt) {
            xEventGroupSetBits(task_events, TASK_EVENT_ANEMOMETER_START);
            xEventGroupWaitBits(task_events,
                                TASK_EVENT_ANEMOMETER_DONE,
                                pdTRUE,
                                pdFALSE,
                                pdMS_TO_TICKS(period_base));
        }

        if (do_sen) {
            xEventGroupSetBits(task_events, TASK_EVENT_SENSORS_START);
            xEventGroupWaitBits(task_events,
                                TASK_EVENT_SENSORS_DONE,
                                pdTRUE,
                                pdFALSE,
                                pdMS_TO_TICKS(period_sen));
        }

        if (do_app) {
            xEventGroupSetBits(task_events, TASK_EVENT_NET_START);
        }
    }
}
