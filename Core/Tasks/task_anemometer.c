/*
 * Anemometer task
 *
 * Event-driven: the task sleeps (allowing MCU Stop mode) between
 * hall-sensor pulses.  freqmeter_irq() sends a task notification on
 * each EXTI edge, so the MCU wakes only to record the timestamp
 * and immediately goes back to sleep.
 *
 * Dmitry Proshutinsky <dproshutinsky@gmail.com>
 * 2025-2026
 */

#include "actual.h"
#include "freqmeter.h"
#include "ptasks.h"

/* Measurement window duration (ms) */
#define FREQMETER_MEAS_TIME_MS 3000

/* Stabilization time after power on (ms) */
#define FREQMETER_STABILIZE_MS 10

/*
 * Timeout for the first pulse (ms).
 * If no pulse arrives within this time, assume zero wind.
 */
#define FREQMETER_FIRST_PULSE_TIMEOUT_MS (FREQMETER_MEAS_TIME_MS / 2)

void task_anemometer(void* argument) {
    struct task_anemometer_ctx* ctx = argument;
    uint32_t value;

    for (;;) {
        xEventGroupWaitBits(task_events,
                            TASK_EVENT_ANEMOMETER_START,
                            pdTRUE,
                            pdFALSE,
                            portMAX_DELAY);

        led_blink(1);

        /* Power on and stabilize */
        ctx->anemometer_on();
        osDelay(pdMS_TO_TICKS(FREQMETER_STABILIZE_MS));

        /* Reset counter and register for notifications */
        freqmeter_reset(ctx->cnt);
        ctx->cnt->notify_task = xTaskGetCurrentTaskHandle();

        /* Clear any stale notifications */
        ulTaskNotifyTake(pdTRUE, 0);

        /*
         * Wait for the first pulse (establishes t_start).
         * MCU is free to enter Stop mode during this wait.
         */
        if (ulTaskNotifyTake(pdTRUE,
                             pdMS_TO_TICKS(FREQMETER_FIRST_PULSE_TIMEOUT_MS)) ==
            0) {
            /* Timeout — no wind */
            goto done;
        }

        /*
         * Collect subsequent pulses until we have enough periods
         * or the measurement window expires.
         */
        {
            TickType_t deadline =
                xTaskGetTickCount() + pdMS_TO_TICKS(FREQMETER_MEAS_TIME_MS);

            while (ctx->cnt->period_cnt < FREQMETER_MIN_PERIODS) {
                TickType_t now = xTaskGetTickCount();
                TickType_t remain;

                if (now >= deadline) {
                    break;
                }
                remain = deadline - now;

                if (ulTaskNotifyTake(pdTRUE, remain) == 0) {
                    /* Timeout — no more pulses within window */
                    break;
                }
            }
        }

    done:
        /* Stop listening */
        ctx->cnt->notify_task = NULL;

        value = freqmeter_speed(ctx->cnt);

        ctx->anemometer_off();

        /* Update actual value */
        xSemaphoreTake(ctx->actual->mutex, portMAX_DELAY);
        ctx->actual->wind_speed = value;
        xSemaphoreGive(ctx->actual->mutex);

        /* Update accumulator for min/avg/max aggregation */
        freqmeter_accum_update(ctx->cnt, value);

        /* Signal completion to scheduler */
        xEventGroupSetBits(task_events, TASK_EVENT_ANEMOMETER_DONE);
    }
}
