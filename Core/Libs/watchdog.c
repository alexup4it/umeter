/*
 * Watchdog
 */

#include "watchdog.h"

#include <string.h>

static struct
{
	IWDG_HandleTypeDef *wdg;
	GPIO_TypeDef *ext_port;
	uint16_t ext_pin;
} wdog;


void watchdog_init(IWDG_HandleTypeDef *wdg, GPIO_TypeDef *ext_port, uint16_t ext_pin)
{
	wdog.wdg = wdg;
	wdog.ext_port = ext_port;
	wdog.ext_pin = ext_pin;
}


void watchdog_reset(void)
{
	if (!wdog.wdg)
		return;

	// 32kHz / 128 / 4095 -> 16,38 seconds
	HAL_IWDG_Refresh(wdog.wdg);

	// External watchdog (CBM706TAS8, ADM708TARZ) -> 1,6 seconds
	HAL_GPIO_TogglePin(wdog.ext_port, wdog.ext_pin);
}
