/*
 * Frequency meter
 *
 * Measures a single period between two consecutive pulses
 * using the RTC sub-second counter.
 */

#include "freqmeter.h"

#include <string.h>

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
static uint32_t rtc_subseconds(void) {
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
}

/******************************************************************************/
void freqmeter_irq(struct freqmeter* self) {
    uint32_t now = rtc_subseconds();

    if (self->started) {
        uint32_t period = now - self->last_ts;

        /* Correct for minute-boundary wrap (Seconds: 59 → 0) */
        if (now < self->last_ts) {
            period += FREQMETER_RTC_WRAP;
        }

        self->period = period;
    }

    self->last_ts = now;
    self->started = 1;

    /* Wake the waiting task */
    TaskHandle_t task = self->notify_task;
    if (task != NULL) {
        BaseType_t woken = pdFALSE;
        vTaskNotifyGiveFromISR(task, &woken);
        portYIELD_FROM_ISR(woken);
    }
}

/******************************************************************************/
/*
 * Block until a full period is captured or timeout expires.
 *
 * Resets state, waits for two pulses (start + end), returns the
 * period in RTC SSR ticks.  Returns 0 on timeout (no wind).
 *
 * Must be called from a FreeRTOS task context.
 */
uint32_t freqmeter_read(struct freqmeter* self, uint32_t timeout_ms) {
    TickType_t half = pdMS_TO_TICKS(timeout_ms / 2);

    self->notify_task = xTaskGetCurrentTaskHandle();

    /* Clear any stale notifications */
    ulTaskNotifyTake(pdTRUE, 0);

    /* Reset state — next IRQ becomes the start pulse */
    self->started     = 0;
    self->period      = 0;

    /* Wait for the first pulse (start timestamp) */
    if (ulTaskNotifyTake(pdTRUE, half) == 0) {
        self->notify_task = NULL;
        return 0;
    }

    /* Wait for the second pulse (completes the period) */
    if (!self->period) {
        ulTaskNotifyTake(pdTRUE, half);
    }

    self->notify_task = NULL;

    return self->period;
}
