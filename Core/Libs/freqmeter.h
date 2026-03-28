/*
 * Frequency meter
 *
 * Measures a single period between two consecutive pulses using
 * the RTC sub-second counter.  freqmeter_read() blocks the calling
 * task until a period is captured or the timeout expires.
 */

#ifndef FREQMETER_H_
#define FREQMETER_H_

#include "cmsis_os.h"

/*
 * RTC synchronous prescaler — must match CubeMX RTC config.
 * SSR counts down from FREQMETER_RTC_PREDIV_S to 0 each second,
 * giving a time resolution of 1 / (FREQMETER_RTC_PREDIV_S + 1) seconds.
 */
#define FREQMETER_RTC_PREDIV_S 8191U

/* SSR ticks per second */
#define FREQMETER_RTC_FREQ (FREQMETER_RTC_PREDIV_S + 1U)

/* Total ticks in 60 seconds (for minute-boundary wrap-around correction) */
#define FREQMETER_RTC_WRAP (60U * FREQMETER_RTC_FREQ)

struct freqmeter {
    /* Period tracking (filled in IRQ via RTC SSR) */
    volatile uint32_t last_ts; /* previous timestamp (seconds * FREQ + sub) */
    volatile uint32_t period;  /* last captured period in SSR ticks */
    volatile uint8_t started;  /* first pulse seen since reset */

    /* Task notification target (set before measurement) */
    volatile TaskHandle_t notify_task;
};

void freqmeter_init(struct freqmeter* self);
void freqmeter_irq(struct freqmeter* self);
uint32_t freqmeter_read(struct freqmeter* self, uint32_t timeout_ms);

#endif /* FREQMETER_H_ */
