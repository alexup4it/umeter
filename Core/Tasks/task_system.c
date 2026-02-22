/*
 * System task
 *
 * Dmitry Proshutinsky <dproshutinsky@gmail.com>
 * 2024-2026
 */

#include "ptasks.h"

#include <stdlib.h>
#include <string.h>

#include "params.h"
#include "fws.h"

#include "logger.h"
#define TAG "SYSTEM"
extern struct logger logger;

static osThreadId_t handle;
static const osThreadAttr_t attributes = {
  .name = "system",
  .stack_size = 120 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};


static void print_info_str(struct logger *logger, const char *header,
		const char *sub, const char *data)
{
	char *buf = pvPortMalloc(strlen(header) + 1 + strlen(sub) + 1 +
			strlen(data) + 2 + 1);
	if (!buf)
		return;

	strcpy(buf, header);
	strcat(buf, " ");
	strcat(buf, sub);
	strcat(buf, " ");
	strcat(buf, data);

	logger_add_str(logger, TAG, false, buf);
	vPortFree(buf);
}

static void info_base(struct system *sys)
{
	char temp[32];

	utoa(sys->main_stack_size, temp, 10);
	print_info_str(&logger, "MAIN", "stack_size", temp);

	print_info_str(&logger, "BL", "git", (char *) sys->bl->hash);
	utoa(sys->bl->status, temp, 10);
	print_info_str(&logger, "BL", "status", temp);

	print_info_str(&logger, "APP", "git", GIT_COMMIT_HASH);
	print_info_str(&logger, "APP", "name", PARAMS_DEVICE_NAME);
	utoa(PARAMS_FW_VERSION, temp, 10);
	print_info_str(&logger, "APP", "ver", temp);
	print_info_str(&logger, "APP", "MCU", sys->params->mcu_uid);

	utoa(sys->params->offset_angle, temp, 10);
	print_info_str(&logger, "PARAMS", "angle_offset", temp);
}

static void info_mem(void)
{
	const char *t_names[] = {"def", "system", "app", "siface", "ota", "sim800l",
			"sensors", "ecounter", "button", NULL};
	TaskHandle_t t_handle;
	TaskStatus_t details;
	char temp[16];

	utoa(xPortGetMinimumEverFreeHeapSize(), temp, 10);
	print_info_str(&logger, "HEAP", "~", temp);

	for (int i = 0; t_names[i]; i++)
	{
		t_handle = xTaskGetHandle(t_names[i]);
		if (!t_handle)
			continue;

		vTaskGetInfo(t_handle, &details, pdTRUE, eInvalid);
		utoa(details.usStackHighWaterMark * sizeof(StackType_t), temp, 10);
		print_info_str(&logger, "STACK", t_names[i], temp);
	}
}

static void task(void *argument)
{
	TickType_t ticks = xTaskGetTickCount();
	struct system *sys = argument;

	/* todo: fix me */
//	info_base(sys);

	for (;;)
	{
		if ((xTaskGetTickCount() - ticks) >= pdMS_TO_TICKS(20000))
		{
			ticks = xTaskGetTickCount();
			info_base(sys);
			info_mem();
		}

		// 32kHz / 128 / 4095 -> 16,38 seconds
		HAL_IWDG_Refresh(sys->wdg);

		// External watchdog (CBM706TAS8, ADM708TARZ) -> 1,6 seconds
		HAL_GPIO_TogglePin(sys->ext_port, sys->ext_pin);

		osDelay(1000);
	}
}

/******************************************************************************/
void task_system(struct system *sys)
{
	handle = osThreadNew(task, sys, &attributes);
}

