/*
 * Watchdog
 */

#include "watchdog.h"

#include "cmsis_os2.h"

static struct {
    IWDG_HandleTypeDef* wdg;
    GPIO_TypeDef* ext_port;
    uint16_t ext_pin;
} wdog;

void watchdog_init(IWDG_HandleTypeDef* wdg,
                   GPIO_TypeDef* ext_port,
                   uint16_t ext_pin) {
    wdog.wdg      = wdg;
    wdog.ext_port = ext_port;
    wdog.ext_pin  = ext_pin;
}

void watchdog_reset(void) {
    if (!wdog.wdg) {
        return;
    }

    // 32kHz / 128 / 4095 -> 16,38 seconds
    HAL_IWDG_Refresh(wdog.wdg);

    HAL_GPIO_WritePin(wdog.ext_port, wdog.ext_pin, GPIO_PIN_RESET);
    osDelay(1);
    HAL_GPIO_WritePin(wdog.ext_port, wdog.ext_pin, GPIO_PIN_SET);
}
