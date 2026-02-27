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
void counter_accum_update(struct counter *cnt, uint32_t value);
void counter_accum_read(struct counter *cnt, struct counter_accum *out);

#endif /* COUNTER_H_ */
