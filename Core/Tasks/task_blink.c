/*
 * Blink task
 */

#include "ptasks.h"
#include "main.h"

static osThreadId_t handle;
static const osThreadAttr_t attributes = {
  .name = "blink",
  .stack_size = 64 * 4,
  .priority = (osPriority_t) osPriorityLow,
};

static volatile uint8_t pending;

void led_blink(uint8_t count)
{
	if (count > pending)
		pending = count;
	xTaskNotifyGive(handle);
}

static void task(void *argument)
{
	uint8_t count;
	for (;;)
	{
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		count = pending;
		pending = 0;
		for (uint8_t i = 0; i < count; i++)
		{
			HAL_GPIO_WritePin(LED_DB_GPIO_Port, LED_DB_Pin, GPIO_PIN_RESET);
			osDelay(10);
			HAL_GPIO_WritePin(LED_DB_GPIO_Port, LED_DB_Pin, GPIO_PIN_SET);
			if (i < count - 1)
				osDelay(200);
		}
	}
}

/******************************************************************************/
void task_blink(void)
{
	handle = osThreadNew(task, NULL, &attributes);
}
