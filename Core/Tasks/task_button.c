/*
 * Button task
 *
 * Dmitry Proshutinsky <dproshutinsky@gmail.com>
 * 2025
 */

#include "ptasks.h"

static osThreadId_t handle;
static const osThreadAttr_t attributes = {
    .name       = "button",
    .stack_size = 64 * 4,
    .priority   = (osPriority_t)osPriorityBelowNormal,
};

static void task(void* argument) {
    struct button* btn = argument;
    button_task(btn);  // <- Infinite loop
}

/******************************************************************************/
void task_button(struct button* btn) {
    handle = osThreadNew(task, btn, &attributes);
}
