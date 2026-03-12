/*
 * Button
 *
 * Dmitry Proshutinsky <dproshutinsky@gmail.com>
 * 2025-2026
 */

#include "button.h"

#include <string.h>

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
int button_poll(struct button* btn) {
    void (*callback)(void) = btn->callback;
    int state;

    if (btn->counter) {
        btn->counter--;
    }

    if (btn->counter) {
        return 0;
    }

    state = ~HAL_GPIO_ReadPin(btn->port, btn->pin) & 0x01;

    if (btn->state != state) {
        btn->state   = state;
        btn->counter = DELAY / PERIOD;

        if (state) {
            callback();
            return 1;
        }
    }

    return 0;
}
