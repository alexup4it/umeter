/*
 * SIM800L task
 *
 * Dmitry Proshutinsky <dproshutinsky@gmail.com>
 * 2024-2025
 */

#include "ptasks.h"

static osThreadId_t handle;
static const osThreadAttr_t attributes = {
    .name       = "sim800l",
    .stack_size = 112 * 4,
    .priority   = (osPriority_t)osPriorityNormal,
};

static void task(void* argument) {
    struct sim800l* mod = argument;
    sim800l_task(mod);  // <- Infinite loop
}

/******************************************************************************/
void task_sim800l(struct sim800l* mod) {
    handle = osThreadNew(task, mod, &attributes);
}
