/*
 * OTA task
 *
 * Dmitry Proshutinsky <dproshutinsky@gmail.com>
 * 2024-2025
 */

#include "ptasks.h"

static osThreadId_t handle;
static const osThreadAttr_t attributes = {
    .name       = "ota",
    .stack_size = 352 * 4,
    .priority   = (osPriority_t)osPriorityNormal,
};

static void task(void* argument) {
    struct ota* ota = argument;
    ota_task(ota);  // <- Infinite loop
}

/******************************************************************************/
void task_ota(struct ota* ota) {
    handle = osThreadNew(task, ota, &attributes);
}
