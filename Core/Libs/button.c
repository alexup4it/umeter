/*
 * Button
 *
 * Dmitry Proshutinsky <dproshutinsky@gmail.com>
 * 2025-2026
 */

#include "button.h"

#include <string.h>

static int time_reached(uint32_t now, uint32_t deadline) {
    return (int32_t)(now - deadline) >= 0;
}

/******************************************************************************/
void button_init(struct button* btn,
                 button_callback_t callback,
                 uint32_t debounce_ms) {
    memset(btn, 0, sizeof(*btn));
    btn->callback    = callback;
    btn->debounce_ms = debounce_ms;
    if (btn->debounce_ms == 0) {
        btn->debounce_ms = 1;
    }
}

/******************************************************************************/
int button_irq_callback(struct button* btn, int is_pressed, uint32_t now_ms) {
    uint32_t held_ms;

    if (!btn) {
        return 0;
    }

    if (btn->ignore_active) {
        if (!time_reached(now_ms, btn->ignore_until_ms)) {
            return 0;
        }
        btn->ignore_active = 0;
    }

    if (is_pressed) {
        if (!btn->is_pressed) {
            btn->is_pressed          = 1;
            btn->press_started_at_ms = now_ms;
        }
        return 0;
    }

    if (!btn->is_pressed) {
        return 0;
    }

    btn->is_pressed = 0;
    held_ms         = now_ms - btn->press_started_at_ms;

    if (held_ms < btn->debounce_ms) {
        return 0;
    }

    btn->ignore_until_ms = now_ms + btn->debounce_ms;
    btn->ignore_active   = 1;
    return 1;
}

/******************************************************************************/
void button_dispatch(struct button* btn) {
    if (btn && btn->callback) {
        btn->callback();
    }
}
