/*
 * Button task
 */

#include <string.h>

#include "actual.h"
#include "button.h"
#include "iwdg.h"
#include "params.h"
#include "ptasks.h"

static TaskHandle_t s_button_task_handle;

static void save_wind_direction_from_actual(struct actual* actual) {
    int32_t direction;
    params_t uparams;

    /* Reset offset so sensors read raw wind_direction */
    params.offset_angle = 0;

    /* Trigger a fresh sensor reading with zero offset */
    xEventGroupClearBits(task_events, TASK_EVENT_SENSORS_DONE);
    xEventGroupSetBits(task_events, TASK_EVENT_SENSORS_START);
    xEventGroupWaitBits(task_events,
                        TASK_EVENT_SENSORS_DONE,
                        pdTRUE,
                        pdFALSE,
                        portMAX_DELAY);

    /* Now actual->wind_direction == raw wind_direction (offset was 0) */
    xSemaphoreTake(actual->mutex, portMAX_DELAY);
    direction = actual->wind_direction;
    xSemaphoreGive(actual->mutex);

    if (direction < 0) {
        return;
    }

    /* Save raw wind_direction as new offset */
    memcpy(&uparams, &params, sizeof(uparams));
    uparams.offset_angle = (uint32_t)direction;

    IWDG_reset();
    vTaskSuspendAll();
    params_set(&uparams);
    xTaskResumeAll();

    params.offset_angle = (uint32_t)direction;

    /* Update actual to reflect zero wind_direction */
    xSemaphoreTake(actual->mutex, portMAX_DELAY);
    actual->wind_direction = 0;
    xSemaphoreGive(actual->mutex);
}

void task_button_irq_notify_from_isr(void) {
    BaseType_t task_woken = pdFALSE;

    if (!s_button_task_handle) {
        return;
    }

    vTaskNotifyGiveFromISR(s_button_task_handle, &task_woken);
    portYIELD_FROM_ISR(task_woken);
}

void task_button(void* argument) {
    struct task_button_ctx* ctx = argument;
    uint32_t notifications;

    s_button_task_handle = xTaskGetCurrentTaskHandle();

    for (;;) {
        notifications = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        while (notifications--) {
            save_wind_direction_from_actual(ctx->actual);
        }
    }
}
