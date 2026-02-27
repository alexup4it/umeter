/*
 * Watchdog
 */

#ifndef WATCHDOG_H_
#define WATCHDOG_H_

#include "stm32f4xx_hal.h"


void watchdog_init(IWDG_HandleTypeDef *wdg,
		GPIO_TypeDef *ext_port, uint16_t ext_pin);
void watchdog_reset(void);

#endif /* WATCHDOG_H_ */
