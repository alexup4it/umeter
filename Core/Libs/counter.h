/*
 * Pulse counter
 *
 * Dmitry Proshutinsky <dproshutinsky@gmail.com>
 * 2024-2026
 */

#ifndef COUNTER_H_
#define COUNTER_H_

#include "stm32f4xx_hal.h"

#include "cmsis_os.h"
#include "semphr.h"

#define COUNTER_MIN_INIT 0xFFFFFFFF

/* Minimum number of periods (pulse pairs) for a valid measurement */
#define COUNTER_MIN_PERIODS 2

/* Speed = COUNTER_SPEED_SCALE / period_avg_ms */
#define COUNTER_SPEED_SCALE 10000

struct counter_accum
{
	uint32_t avg;
	uint32_t min;
	uint32_t max;
};

struct counter
{
	volatile uint32_t count;
	GPIO_TypeDef *pwr_port;
	uint16_t pwr_pin;

	/* Period tracking (filled in IRQ) */
	volatile uint32_t last_tick;
	volatile uint32_t period_sum;
	volatile uint32_t period_cnt;
	volatile uint8_t started;

	SemaphoreHandle_t mutex;
	uint32_t accum_min;
	uint32_t accum_max;
	uint32_t accum_sum;
	size_t accum_cnt;
};


void counter_init(struct counter *cnt, GPIO_TypeDef *pwr_port,
		uint16_t pwr_pin);
void counter_irq(struct counter *cnt);
void counter_power_on(struct counter *cnt);
void counter_power_off(struct counter *cnt);
void counter_reset(struct counter *cnt);
uint32_t counter(struct counter *cnt);
uint32_t counter_period_avg(struct counter *cnt);
uint32_t counter_speed(struct counter *cnt);
void counter_accum_update(struct counter *cnt, uint32_t value);
void counter_accum_read(struct counter *cnt, struct counter_accum *out);

#endif /* COUNTER_H_ */
