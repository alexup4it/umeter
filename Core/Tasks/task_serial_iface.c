/*
 * Serial interface task
 *
 * Dmitry Proshutinsky <dproshutinsky@gmail.com>
 * 2024-2026
 */

#include "ptasks.h"
#include "siface.h"

void task_serial_iface(void* argument) {
    struct task_serial_iface_ctx* ctx = argument;
    siface_task(ctx->siface);
}
