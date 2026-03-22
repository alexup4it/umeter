/*
 * Pulse counter (RTC SSR-based timing)
 */

#include "counter.h"

#include <string.h>

#include "atomic.h"
#include "rtc.h"

/*
 * Read RTC timestamp as a single 32-bit value in SSR ticks.
 *
 * SSR is a downcounter: 0 at top of second, PREDIV_S at bottom.
 * We convert to an upcounting value: sub = PREDIV_S - SSR.
 * Full timestamp = seconds * (PREDIV_S + 1) + sub.
 *
 * Only seconds and sub-seconds are used (no minutes/hours) because
 * measurement windows are at most a few seconds long.  The caller
 * must ensure that the total elapsed time does not exceed
 * 0xFFFFFFFF / COUNTER_RTC_FREQ ≈ 524 287 seconds (~6 days).
 *
 * HAL_RTC_GetTime locks the shadow registers; HAL_RTC_GetDate
 * must be called afterwards to unlock them (RM0368 §22.3.6).
 */
static uint32_t rtc_timestamp(void) {
    RTC_TimeTypeDef t;
    RTC_DateTypeDef d;

    HAL_RTC_GetTime(&hrtc, &t, RTC_FORMAT_BIN);
    HAL_RTC_GetDate(&hrtc, &d, RTC_FORMAT_BIN);
    (void)d;

    uint32_t sub = COUNTER_RTC_PREDIV_S - t.SubSeconds;
    return (uint32_t)t.Seconds * COUNTER_RTC_FREQ + sub;
}

/******************************************************************************/
void counter_init(struct counter* cnt) {
    memset(cnt, 0, sizeof(*cnt));
    cnt->count = 0;

    cnt->last_ts    = 0;
    cnt->period_sum = 0;
    cnt->period_cnt = 0;
    cnt->started    = 0;

    cnt->notify_task = NULL;

    cnt->mutex     = xSemaphoreCreateMutex();
    cnt->accum_min = COUNTER_MIN_INIT;
    cnt->accum_max = 0;
    cnt->accum_sum = 0;
    cnt->accum_cnt = 0;
}

/******************************************************************************/
void counter_irq(struct counter* cnt) {
    uint32_t now = rtc_timestamp();

    if (cnt->started) {
        uint32_t period = now - cnt->last_ts;
        cnt->period_sum += period;
        cnt->period_cnt++;
    }

    cnt->last_ts = now;
    cnt->started = 1;

    atomic_inc(&cnt->count);

    /* Wake the anemometer task if it is waiting */
    TaskHandle_t task = cnt->notify_task;
    if (task != NULL) {
        BaseType_t woken = pdFALSE;
        vTaskNotifyGiveFromISR(task, &woken);
        portYIELD_FROM_ISR(woken);
    }
}

/******************************************************************************/
/******************************************************************************/
void counter_reset(struct counter* cnt) {
    cnt->count      = 0;
    cnt->started    = 0;
    cnt->last_ts    = 0;
    cnt->period_sum = 0;
    cnt->period_cnt = 0;
}

/******************************************************************************/
uint32_t counter(struct counter* cnt) {
    return cnt->count;
}

/******************************************************************************/
uint32_t counter_period_avg(struct counter* cnt) {
    if (cnt->period_cnt == 0) {
        return 0;
    }
    return cnt->period_sum / cnt->period_cnt;
}

/******************************************************************************/
uint32_t counter_speed(struct counter* cnt) {
    uint32_t avg_ticks = counter_period_avg(cnt);
    if (avg_ticks == 0) {
        return 0;
    }
    /*
     * speed = COUNTER_SPEED_SCALE * COUNTER_RTC_FREQ / avg_ticks
     *
     * With COUNTER_RTC_FREQ = 8192, COUNTER_SPEED_SCALE = 10000:
     *   1 s period (avg_ticks = 8192) → speed = 10000
     *   Same result as the old formula with 10 ms ticks,
     *   but 82× better resolution.
     */
    return (uint32_t)((uint64_t)COUNTER_SPEED_SCALE * COUNTER_RTC_FREQ /
                      avg_ticks);
}

/******************************************************************************/
void counter_accum_update(struct counter* cnt, uint32_t value) {
    xSemaphoreTake(cnt->mutex, portMAX_DELAY);

    if (value < cnt->accum_min) {
        cnt->accum_min = value;
    }
    if (value > cnt->accum_max) {
        cnt->accum_max = value;
    }
    cnt->accum_sum += value;
    cnt->accum_cnt++;

    xSemaphoreGive(cnt->mutex);
}

/******************************************************************************/
void counter_accum_read(struct counter* cnt, struct counter_accum* out) {
    xSemaphoreTake(cnt->mutex, portMAX_DELAY);

    out->max = cnt->accum_max;
    out->min = (cnt->accum_min != COUNTER_MIN_INIT) ? cnt->accum_min : 0;
    out->avg = cnt->accum_cnt ? (cnt->accum_sum / cnt->accum_cnt) : 0;

    cnt->accum_min = COUNTER_MIN_INIT;
    cnt->accum_max = 0;
    cnt->accum_sum = 0;
    cnt->accum_cnt = 0;

    xSemaphoreGive(cnt->mutex);
}
