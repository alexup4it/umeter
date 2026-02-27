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
	// Toggle between Hi-Z (input mode) and LOW (output mode), never pull high
	GPIO_InitTypeDef gpio = {0};
	gpio.Pin = wdog.ext_pin;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_FREQ_LOW;

	if (wdog.ext_port->MODER & (1 << (__builtin_ctz(wdog.ext_pin) * 2)))
	{
		// Currently output (LOW) -> switch to Hi-Z (input)
		gpio.Mode = GPIO_MODE_INPUT;
	}
	else
	{
		// Currently Hi-Z (input) -> switch to LOW (output)
		gpio.Mode = GPIO_MODE_OUTPUT_OD;
		HAL_GPIO_WritePin(wdog.ext_port, wdog.ext_pin, GPIO_PIN_RESET);
	}

	HAL_GPIO_Init(wdog.ext_port, &gpio);
}
