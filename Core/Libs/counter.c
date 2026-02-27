/*
 * Pulse counter
 *
 * Dmitry Proshutinsky <dproshutinsky@gmail.com>
 * 2024-2026
 */

#include "counter.h"

#include <string.h>

#include "atomic.h"


/******************************************************************************/
void counter_init(struct counter *cnt, GPIO_TypeDef *pwr_port, uint16_t pwr_pin)
{
	memset(cnt, 0, sizeof(*cnt));
	cnt->count = 0;
	cnt->pwr_port = pwr_port;
	cnt->pwr_pin = pwr_pin;

	cnt->mutex = xSemaphoreCreateMutex();
	cnt->accum_min = COUNTER_MIN_INIT;
	cnt->accum_max = 0;
	cnt->accum_sum = 0;
	cnt->accum_cnt = 0;
}

/******************************************************************************/
void counter_irq(struct counter *cnt)
{
	atomic_inc(&cnt->count);
}

/******************************************************************************/
void counter_power_on(struct counter *cnt)
{
	HAL_GPIO_WritePin(cnt->pwr_port, cnt->pwr_pin, GPIO_PIN_SET);
}

/******************************************************************************/
void counter_power_off(struct counter *cnt)
{
	HAL_GPIO_WritePin(cnt->pwr_port, cnt->pwr_pin, GPIO_PIN_RESET);
}

/******************************************************************************/
void counter_reset(struct counter *cnt)
{
	cnt->count = 0;
}

/******************************************************************************/
uint32_t counter(struct counter *cnt)
{
	return cnt->count;
}

/******************************************************************************/
void counter_accum_update(struct counter *cnt, uint32_t value)
{
	xSemaphoreTake(cnt->mutex, portMAX_DELAY);

	if (value < cnt->accum_min)
		cnt->accum_min = value;
	if (value > cnt->accum_max)
		cnt->accum_max = value;
	cnt->accum_sum += value;
	cnt->accum_cnt++;

	xSemaphoreGive(cnt->mutex);
}

/******************************************************************************/
void counter_accum_read(struct counter *cnt, struct counter_accum *out)
{
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
