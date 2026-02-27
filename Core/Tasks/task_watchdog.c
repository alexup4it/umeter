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
		watchdog_reset();
		osDelay(1000);
	}
}

/******************************************************************************/
void task_watchdog()
{
	handle = osThreadNew(task, NULL, &attributes);
}
