/*
 * Frequency meter
 */

#include "freqmeter.h"

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
 * 0xFFFFFFFF / FREQMETER_RTC_FREQ ≈ 524 287 seconds (~6 days).
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

    uint32_t sub = FREQMETER_RTC_PREDIV_S - t.SubSeconds;
    return (uint32_t)t.Seconds * FREQMETER_RTC_FREQ + sub;
}

/******************************************************************************/
void freqmeter_init(struct freqmeter* self) {
    memset(self, 0, sizeof(*self));
    self->count = 0;

    self->last_ts    = 0;
    self->period_sum = 0;
    self->period_cnt = 0;
    self->started    = 0;

    self->notify_task = NULL;

    self->mutex     = xSemaphoreCreateMutex();
    self->accum_min = FREQMETER_MIN_INIT;
    self->accum_max = 0;
    self->accum_sum = 0;
    self->accum_cnt = 0;
}

/******************************************************************************/
void freqmeter_irq(struct freqmeter* self) {
    uint32_t now = rtc_timestamp();

    if (self->started) {
        uint32_t period = now - self->last_ts;
        self->period_sum += period;
        self->period_cnt++;
    }

    self->last_ts = now;
    self->started = 1;

    atomic_inc(&self->count);

    /* Wake the anemometer task if it is waiting */
    TaskHandle_t task = self->notify_task;
    if (task != NULL) {
        BaseType_t woken = pdFALSE;
        vTaskNotifyGiveFromISR(task, &woken);
        portYIELD_FROM_ISR(woken);
    }
}

/******************************************************************************/
/******************************************************************************/
void freqmeter_reset(struct freqmeter* self) {
    self->count      = 0;
    self->started    = 0;
    self->last_ts    = 0;
    self->period_sum = 0;
    self->period_cnt = 0;
}

/******************************************************************************/
uint32_t freqmeter(struct freqmeter* self) {
    return self->count;
}

/******************************************************************************/
uint32_t freqmeter_period_avg(struct freqmeter* self) {
    if (self->period_cnt == 0) {
        return 0;
    }
    return self->period_sum / self->period_cnt;
}

/******************************************************************************/
uint32_t freqmeter_speed(struct freqmeter* self) {
    uint32_t avg_ticks = freqmeter_period_avg(self);
    if (avg_ticks == 0) {
        return 0;
    }
    /*
     * speed = FREQMETER_SPEED_SCALE * FREQMETER_RTC_FREQ / avg_ticks
     *
     * With FREQMETER_RTC_FREQ = 8192, FREQMETER_SPEED_SCALE = 10000:
     *   1 s period (avg_ticks = 8192) → speed = 10000
     *   Same result as the old formula with 10 ms ticks,
     *   but 82× better resolution.
     */
    return (uint32_t)((uint64_t)FREQMETER_SPEED_SCALE * FREQMETER_RTC_FREQ /
                      avg_ticks);
}

/******************************************************************************/
void freqmeter_accum_update(struct freqmeter* self, uint32_t value) {
    xSemaphoreTake(self->mutex, portMAX_DELAY);

    if (value < self->accum_min) {
        self->accum_min = value;
    }
    if (value > self->accum_max) {
        self->accum_max = value;
    }
    self->accum_sum += value;
    self->accum_cnt++;

    xSemaphoreGive(self->mutex);
}

/******************************************************************************/
void freqmeter_accum_read(struct freqmeter* self, struct freqmeter_accum* out) {
    xSemaphoreTake(self->mutex, portMAX_DELAY);

    out->max = self->accum_max;
    out->min = (self->accum_min != FREQMETER_MIN_INIT) ? self->accum_min : 0;
    out->avg = self->accum_cnt ? (self->accum_sum / self->accum_cnt) : 0;

    self->accum_min = FREQMETER_MIN_INIT;
    self->accum_max = 0;
    self->accum_sum = 0;
    self->accum_cnt = 0;

    xSemaphoreGive(self->mutex);
}
