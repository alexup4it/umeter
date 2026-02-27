/*
 * Extended counter task
 *
 * Dmitry Proshutinsky <dproshutinsky@gmail.com>
 * 2025-2026
 */

#include "ptasks.h"

/* Measurement window duration (ms) */
#define COUNTER_MEAS_TIME_MS 3000

/* Stabilization time after power on (ms) */
#define COUNTER_STABILIZE_MS 10

static osThreadId_t handle;
static const osThreadAttr_t attributes = {
  .name = "ecounter",
  .stack_size = 112 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};


static void task(void *argument)
{
	struct ecounter *ecnt = argument;
	uint32_t value;

	TickType_t wake = xTaskGetTickCount();

	for (;;)
	{
		/* Power on and stabilize */
		counter_power_on(ecnt->cnt);
		osDelay(COUNTER_STABILIZE_MS);

		/* Reset counter before measurement */
		counter_reset(ecnt->cnt);

		/* Measure for window */
		osDelay(COUNTER_MEAS_TIME_MS);

		/* Read count */
		value = counter(ecnt->cnt);

		/* Power off - sleep between measurements */
		counter_power_off(ecnt->cnt);

		/* Update actual value */
		xSemaphoreTake(ecnt->actual->mutex, portMAX_DELAY);
		ecnt->actual->count = value;
		xSemaphoreGive(ecnt->actual->mutex);

		/* Update accumulator for min/avg/max aggregation */
		counter_accum_update(ecnt->cnt, value);

		/* Sleep until next measurement cycle */
		vTaskDelayUntil(&wake, pdMS_TO_TICKS(ecnt->params->mtime_count * 1000));
	}
}

/******************************************************************************/
void task_ecounter(struct ecounter *ecnt)
{
	handle = osThreadNew(task, ecnt, &attributes);
}

