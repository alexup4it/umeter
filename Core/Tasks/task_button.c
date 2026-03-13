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

static void save_angle_from_actual(void) {
    int32_t angle;
    params_t uparams;

    xSemaphoreTake(actual.mutex, portMAX_DELAY);
    angle = actual.angle;
    xSemaphoreGive(actual.mutex);

    if (angle < 0) {
        return;
    }

    memcpy(&uparams, &params, sizeof(uparams));
    uparams.offset_angle = (uint32_t)angle;

    IWDG_reset();
    vTaskSuspendAll();
    params_set(&uparams);
    xTaskResumeAll();

    params.offset_angle = (uint32_t)angle;
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

    (void)ctx;

    s_button_task_handle = xTaskGetCurrentTaskHandle();

    for (;;) {
        notifications = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        while (notifications--) {
            xEventGroupClearBits(task_events, TASK_EVENT_SENSORS_DONE);
            xEventGroupSetBits(task_events, TASK_EVENT_SENSORS_START);
            xEventGroupWaitBits(task_events,
                                TASK_EVENT_SENSORS_DONE,
                                pdTRUE,
                                pdFALSE,
                                portMAX_DELAY);

            save_angle_from_actual();
        }
    }
}
