/*
 * Button task
 */

#include "button.h"
#include "cmsis_os.h"
#include "ptasks.h"

#define BUTTON_POLL_PERIOD_MS 10

void task_button(void* argument) {
    struct task_button_ctx* ctx = argument;
    TickType_t wake             = xTaskGetTickCount();

    for (;;) {
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(BUTTON_POLL_PERIOD_MS));
        button_poll(ctx->btn);
    }
}
