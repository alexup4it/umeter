/*
 * Anemometer task
 *
 * Event-driven: the task sleeps (allowing MCU Stop mode) between
 * hall-sensor pulses.  freqmeter_irq() sends a task notification on
 * each EXTI edge, so the MCU wakes only to record the timestamp
 * and immediately goes back to sleep.
 */

#include "actual.h"
#include "freqmeter.h"
#include "ptasks.h"

/*
 * Timeout for a single period measurement (ms).
 * If no pulse pair arrives within this time, assume zero wind.
 */
#define ANEMOMETER_TIMEOUT_MS 5000

/* Stabilization time after power on (ms) */
#define ANEMOMETER_STABILIZE_MS 10

#define WIND_ACCUM_MIN_INIT 0xFFFFFFFFU

/* Currently returns value in centiHz */
static uint32_t period_to_speed(uint32_t period_subseconds) {
    if (period_subseconds == 0) {
        return 0;
    }
    return 100 * FREQMETER_RTC_FREQ / period_subseconds;
}

void task_anemometer(void* argument) {
    struct task_anemometer_ctx* ctx = argument;

    /* Local accumulator for min/avg/max aggregation */
    uint32_t accum_min = WIND_ACCUM_MIN_INIT;
    uint32_t accum_max = 0;
    uint32_t accum_sum = 0;
    uint32_t accum_cnt = 0;

    for (;;) {
        xEventGroupWaitBits(task_events,
                            TASK_EVENT_ANEM_START,
                            pdTRUE,
                            pdFALSE,
                            portMAX_DELAY);

        led_blink(1);

        /* Power on and stabilize */
        ctx->anemometer_on();
        osDelay(pdMS_TO_TICKS(ANEMOMETER_STABILIZE_MS));

        /* Measure a single period */
        uint32_t period = freqmeter_read(ctx->cnt, ANEMOMETER_TIMEOUT_MS);
        uint32_t value  = period_to_speed(period);

        ctx->anemometer_off();

        xSemaphoreTake(ctx->actual->mutex, portMAX_DELAY);

        /* Reset accumulator if manager has consumed previous aggregation */
        if (TASK_EVENT_ANEM_AGGR_RESET &
            xEventGroupWaitBits(task_events,
                                TASK_EVENT_ANEM_AGGR_RESET,
                                pdTRUE, /* clear on exit */
                                pdFALSE,
                                0)) {
            accum_min = WIND_ACCUM_MIN_INIT;
            accum_max = 0;
            accum_sum = 0;
            accum_cnt = 0;
        }

        /* Update accumulator */
        if (value < accum_min) {
            accum_min = value;
        }
        if (value > accum_max) {
            accum_max = value;
        }
        accum_sum += value;
        accum_cnt++;

        /* Write current speed and aggregated values */
        ctx->actual->wind_speed     = value;
        ctx->actual->wind_speed_avg = accum_sum / accum_cnt;
        ctx->actual->wind_speed_min =
            (accum_min != WIND_ACCUM_MIN_INIT) ? accum_min : 0;
        ctx->actual->wind_speed_max = accum_max;

        xSemaphoreGive(ctx->actual->mutex);

        /* Signal completion to scheduler */
        xEventGroupSetBits(task_events, TASK_EVENT_ANEM_DONE);
    }
}
