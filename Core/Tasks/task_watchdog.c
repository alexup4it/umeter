/*
 * Watchdog task
 */

#include "iwdg.h"
#include "ptasks.h"

void task_watchdog(void* argument) {
    for (;;) {
        xEventGroupWaitBits(task_events,
                            TASK_EVENT_WATCHDOG_START,
                            pdTRUE,
                            pdFALSE,
                            pdMS_TO_TICKS(30000));
        IWDG_reset();
    }
}
