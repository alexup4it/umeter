/*
 * Anemometer (pulse counter) task
 *
 * Dmitry Proshutinsky <dproshutinsky@gmail.com>
 * 2025-2026
 */

#include "actual.h"
#include "counter.h"
#include "ptasks.h"

/* Measurement window duration (ms) */
#define COUNTER_MEAS_TIME_MS 3000

/* Stabilization time after power on (ms) */
#define COUNTER_STABILIZE_MS 10

/* Polling interval while waiting for pulses (ms) */
#define COUNTER_POLL_MS 50

void task_anemometer(void* argument) {
    struct task_anemometer_ctx* ctx = argument;
    uint32_t value;
    uint32_t elapsed;

    for (;;) {
        xEventGroupWaitBits(task_events,
                            TASK_EVENT_ANEMOMETER_START,
                            pdTRUE,
                            pdFALSE,
                            portMAX_DELAY);

        led_blink(2);

        /* Power on and stabilize */
        ctx->anemometer_on();
        osDelay(pdMS_TO_TICKS(COUNTER_STABILIZE_MS));

        /* Reset counter before measurement */
        counter_reset(ctx->cnt);

        elapsed = 0;
        while (elapsed < COUNTER_MEAS_TIME_MS) {
            osDelay(pdMS_TO_TICKS(COUNTER_POLL_MS));
            elapsed += COUNTER_POLL_MS;

            if (ctx->cnt->period_cnt >= COUNTER_MIN_PERIODS) {
                break;
            }

            if (elapsed >= COUNTER_MEAS_TIME_MS / 2 && !ctx->cnt->started) {
                break;
            }
        }

        value = counter_speed(ctx->cnt);

        ctx->anemometer_off();

        /* Update actual value */
        xSemaphoreTake(ctx->actual->mutex, portMAX_DELAY);
        ctx->actual->count = value;
        xSemaphoreGive(ctx->actual->mutex);

        /* Update accumulator for min/avg/max aggregation */
        counter_accum_update(ctx->cnt, value);

        /* Signal completion to scheduler */
        xEventGroupSetBits(task_events, TASK_EVENT_ANEMOMETER_DONE);
    }
}
