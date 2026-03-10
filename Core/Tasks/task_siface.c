/*
 * Serial interface task
 *
 * Dmitry Proshutinsky <dproshutinsky@gmail.com>
 * 2024
 */

#include "ptasks.h"

static osThreadId_t handle;
static const osThreadAttr_t attributes = {
    .name       = "siface",
    .stack_size = 128 * 4,
    .priority   = (osPriority_t)osPriorityNormal,
};

static void task(void* argument) {
    struct siface* siface = argument;
    siface_task(siface);  // <- Infinite loop
}

/******************************************************************************/
void task_siface(struct siface* siface) {
    handle = osThreadNew(task, siface, &attributes);
}
