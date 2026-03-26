/*
 * Button
 */

#include "button.h"

#include <string.h>

static int time_reached(uint32_t now, uint32_t deadline) {
    return (int32_t)(now - deadline) >= 0;
}

/******************************************************************************/
void button_init(struct button* self,
                 button_callback_t callback,
                 uint32_t debounce_ms) {
    memset(self, 0, sizeof(*self));
    self->callback    = callback;
    self->debounce_ms = debounce_ms;
    if (self->debounce_ms == 0) {
        self->debounce_ms = 1;
    }
}

/******************************************************************************/
int button_irq_callback(struct button* self, int is_pressed, uint32_t now_ms) {
    uint32_t held_ms;

    if (!self) {
        return 0;
    }

    if (self->ignore_active) {
        if (!time_reached(now_ms, self->ignore_until_ms)) {
            return 0;
        }
        self->ignore_active = 0;
    }

    if (is_pressed) {
        if (!self->is_pressed) {
            self->is_pressed          = 1;
            self->press_started_at_ms = now_ms;
        }
        return 0;
    }

    if (!self->is_pressed) {
        return 0;
    }

    self->is_pressed = 0;
    held_ms          = now_ms - self->press_started_at_ms;

    if (held_ms < self->debounce_ms) {
        return 0;
    }

    self->ignore_until_ms = now_ms + self->debounce_ms;
    self->ignore_active   = 1;
    return 1;
}

/******************************************************************************/
void button_dispatch(struct button* self) {
    if (self && self->callback) {
        self->callback();
    }
}
