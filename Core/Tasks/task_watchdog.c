/*
 * Watchdog task
 */

#include "ptasks.h"

#include "watchdog.h"

static osThreadId_t handle;
static const osThreadAttr_t attributes = {
  .name = "watchdog",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};


static void task(void *argument)
{
	for (;;)
	{
		xEventGroupWaitBits(sync_events, SYNC_BIT_WATCHDOG,
				pdTRUE, pdFALSE, portMAX_DELAY);
		watchdog_reset();
	}
}

/******************************************************************************/
void task_watchdog()
{
	handle = osThreadNew(task, NULL, &attributes);
}
