/*
 * Button
 *
 * Dmitry Proshutinsky <dproshutinsky@gmail.com>
 * 2025
 */

#include "button.h"

#include <string.h>

#include "cmsis_os.h"

#define PERIOD 10
#define DELAY  200

/******************************************************************************/
void button_init(struct button* btn,
                 GPIO_TypeDef* port,
                 uint16_t pin,
                 void* callback) {
    memset(btn, 0, sizeof(*btn));
    btn->port     = port;
    btn->pin      = pin;
    btn->counter  = 0;
    btn->state    = 0;
    btn->callback = callback;
}

/******************************************************************************/
// TODO: Low power optimization
// TODO: Several buttons in one task
void button_task(struct button* btn) {
    void (*callback)(void) = btn->callback;
    TickType_t wake        = xTaskGetTickCount();
    int state;

    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(PERIOD));

        if (btn->counter) {
            btn->counter--;
        };

        if (btn->counter) {
            continue;
        }

        state = ~HAL_GPIO_ReadPin(btn->port, btn->pin) & 0x01;

        if (btn->state != state) {
            btn->state   = state;
            btn->counter = DELAY / PERIOD;

            if (state) {
                callback();
            }
        }
    }
}
