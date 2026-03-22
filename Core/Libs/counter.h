/*
 * Pulse counter (RTC SSR-based timing)
 */

#ifndef COUNTER_H_
#define COUNTER_H_

#include "cmsis_os.h"
#include "semphr.h"

#define COUNTER_MIN_INIT 0xFFFFFFFF

/* Minimum number of periods (pulse pairs) for a valid measurement */
#define COUNTER_MIN_PERIODS 2

/*
 * RTC synchronous prescaler — must match CubeMX RTC config.
 * SSR counts down from COUNTER_RTC_PREDIV_S to 0 each second,
 * giving a time resolution of 1 / (COUNTER_RTC_PREDIV_S + 1) seconds.
 */
#define COUNTER_RTC_PREDIV_S 8191U

/* SSR ticks per second */
#define COUNTER_RTC_FREQ (COUNTER_RTC_PREDIV_S + 1U)

/*
 * Speed scaling factor.
 * speed = COUNTER_SPEED_SCALE * COUNTER_RTC_FREQ / period_avg_ticks
 *
 * With COUNTER_RTC_FREQ = 8192 and COUNTER_SPEED_SCALE = 10000:
 *   period 1 s  → speed = 10000 * 8192 / 8192  = 10000
 *   period 0.1s → speed = 10000 * 8192 / 819   = 100024 ≈ 100000
 */
#define COUNTER_SPEED_SCALE 10000U

struct counter_accum {
    uint32_t avg;
    uint32_t min;
    uint32_t max;
};

struct counter {
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

void counter_init(struct counter* cnt);
void counter_irq(struct counter* cnt);
void counter_reset(struct counter* cnt);
uint32_t counter(struct counter* cnt);
uint32_t counter_period_avg(struct counter* cnt);
uint32_t counter_speed(struct counter* cnt);
void counter_accum_update(struct counter* cnt, uint32_t value);
void counter_accum_read(struct counter* cnt, struct counter_accum* out);

#endif /* COUNTER_H_ */
