/*
 * Extended counter task
 *
 * Dmitry Proshutinsky <dproshutinsky@gmail.com>
 * 2025
 */

#include "ptasks.h"

static osThreadId_t handle;
static const osThreadAttr_t attributes = {
  .name = "ecounter",
  .stack_size = 112 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};


static void task(void *argument)
{
	struct ecounter *ecnt = argument;
	uint32_t count = 0;
	uint32_t value;

	TickType_t wake = xTaskGetTickCount();

	counter_power_on(ecnt->cnt);
	osDelay(10);

	count = counter(ecnt->cnt);

	for (;;)
	{
		vTaskDelayUntil(&wake, pdMS_TO_TICKS(ecnt->params->mtime_count * 1000));

		value = counter(ecnt->cnt) - count;
		count = counter(ecnt->cnt);

		xSemaphoreTake(ecnt->actual->mutex, portMAX_DELAY);
		ecnt->actual->count = value;
		xSemaphoreGive(ecnt->actual->mutex);

		counter_accum_update(ecnt->cnt, value);
	}
}

/******************************************************************************/
void task_ecounter(struct ecounter *ecnt)
{
	handle = osThreadNew(task, ecnt, &attributes);
}

