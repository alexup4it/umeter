/*
 * Modem (SIM800L) task
 *
 * Dmitry Proshutinsky <dproshutinsky@gmail.com>
 * 2024-2026
 */

#include "ptasks.h"
#include "sim800l.h"

void task_modem(void* argument) {
    struct task_modem_ctx* ctx = argument;
    sim800l_task(ctx->mod);
}
