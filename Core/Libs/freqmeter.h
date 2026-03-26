/*
 * Frequency meter
 */

#ifndef FREQMETER_H_
#define FREQMETER_H_

#include "cmsis_os.h"
#include "semphr.h"

#define FREQMETER_MIN_INIT 0xFFFFFFFF

/* Minimum number of periods (pulse pairs) for a valid measurement */
#define FREQMETER_MIN_PERIODS 2

/*
 * RTC synchronous prescaler — must match CubeMX RTC config.
 * SSR counts down from FREQMETER_RTC_PREDIV_S to 0 each second,
 * giving a time resolution of 1 / (FREQMETER_RTC_PREDIV_S + 1) seconds.
 */
#define FREQMETER_RTC_PREDIV_S 8191U

/* SSR ticks per second */
#define FREQMETER_RTC_FREQ (FREQMETER_RTC_PREDIV_S + 1U)

/*
 * Speed scaling factor.
 * speed = FREQMETER_SPEED_SCALE * FREQMETER_RTC_FREQ / period_avg_ticks
 *
 * With FREQMETER_RTC_FREQ = 8192 and FREQMETER_SPEED_SCALE = 10000:
 *   period 1 s  → speed = 10000 * 8192 / 8192  = 10000
 *   period 0.1s → speed = 10000 * 8192 / 819   = 100024 ≈ 100000
 */
#define FREQMETER_SPEED_SCALE 10000U

struct freqmeter_accum {
    uint32_t avg;
    uint32_t min;
    uint32_t max;
};

struct freqmeter {
    volatile uint32_t count;

    /* Period tracking (filled in IRQ via RTC SSR) */
    volatile uint32_t last_ts; /* previous timestamp (seconds * FREQ + sub) */
    volatile uint32_t period_sum; /* sum of periods in SSR ticks */
    volatile uint32_t period_cnt;
    volatile uint8_t started;

    /* Task notification target (set before measurement) */
    volatile TaskHandle_t notify_task;

    SemaphoreHandle_t mutex;
    uint32_t accum_min;
    uint32_t accum_max;
    uint32_t accum_sum;
    size_t accum_cnt;
};

void freqmeter_init(struct freqmeter* self);
void freqmeter_irq(struct freqmeter* self);
void freqmeter_reset(struct freqmeter* self);
uint32_t freqmeter(struct freqmeter* self);
uint32_t freqmeter_period_avg(struct freqmeter* self);
uint32_t freqmeter_speed(struct freqmeter* self);
void freqmeter_accum_update(struct freqmeter* self, uint32_t value);
void freqmeter_accum_read(struct freqmeter* self, struct freqmeter_accum* out);

#endif /* FREQMETER_H_ */
