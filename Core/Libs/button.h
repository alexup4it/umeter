/*
 * Button
 */

#ifndef BUTTON_H_
#define BUTTON_H_

#include <stdint.h>

typedef void (*button_callback_t)(void);

struct button {
    button_callback_t callback;
    uint32_t debounce_ms;
    uint32_t press_started_at_ms;
    uint32_t ignore_until_ms;
    uint8_t is_pressed;
    uint8_t ignore_active;
};

void button_init(struct button* self,
                 button_callback_t callback,
                 uint32_t debounce_ms);
int button_irq_callback(struct button* self, int is_pressed, uint32_t now_ms);
void button_dispatch(struct button* self);

#endif /* BUTTON_H_ */
