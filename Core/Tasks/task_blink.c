/*
 * Blink task
 */

#include "main.h"
#include "ptasks.h"

static volatile uint8_t pending;

/* task handle is provided by CubeMX (blinkHandle in freertos.c) */
extern osThreadId_t blinkHandle;

void led_blink(uint8_t count) {
    if (count > pending) {
        pending = count;
    }
    xTaskNotifyGive(blinkHandle);
}

void task_blink(void* argument) {
    for (;;) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        uint8_t count = pending;
        pending       = 0;
        for (uint8_t i = 0; i < count; i++) {
            HAL_GPIO_WritePin(LED_DB_GPIO_Port, LED_DB_Pin, GPIO_PIN_RESET);
            osDelay(pdMS_TO_TICKS(10));
            HAL_GPIO_WritePin(LED_DB_GPIO_Port, LED_DB_Pin, GPIO_PIN_SET);
            if (i < count - 1) {
                osDelay(pdMS_TO_TICKS(300));
            }
        }
    }
}
