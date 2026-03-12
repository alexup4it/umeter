/*
 * Logging task
 *
 * Dmitry Proshutinsky <dproshutinsky@gmail.com>
 * 2024-2026
 */

#include <stdlib.h>
#include <string.h>

#include "fws.h"
#include "logger.h"
#include "params.h"
#include "ptasks.h"
#include "usb_device.h"

#ifdef LOGGER

#    define TAG "SYSTEM"

extern volatile struct bl_params bl;
extern size_t stack_size(void);

static size_t main_stack_size;
static struct logger* s_logger;

static void print_info_str(struct logger* logger,
                           const char* header,
                           const char* sub,
                           const char* data) {
    char* buf = pvPortMalloc(strlen(header) + 1 + strlen(sub) + 1 +
                             strlen(data) + 2 + 1);
    if (!buf) {
        return;
    }

    strcpy(buf, header);
    strcat(buf, " ");
    strcat(buf, sub);
    strcat(buf, " ");
    strcat(buf, data);

    logger_add_str(logger, TAG, false, buf);
    vPortFree(buf);
}

static void info_base(void) {
    char temp[32];

    utoa(main_stack_size, temp, 10);
    print_info_str(s_logger, "MAIN", "stack_size", temp);

    print_info_str(s_logger, "BL", "git", (char*)bl.hash);
    utoa(bl.status, temp, 10);
    print_info_str(s_logger, "BL", "status", temp);

    print_info_str(s_logger, "APP", "git", GIT_COMMIT_HASH);
    print_info_str(s_logger, "APP", "name", PARAMS_DEVICE_NAME);
    utoa(PARAMS_FW_VERSION, temp, 10);
    print_info_str(s_logger, "APP", "ver", temp);
    print_info_str(s_logger, "APP", "MCU", params.mcu_uid);

    utoa(params.offset_angle, temp, 10);
    print_info_str(s_logger, "PARAMS", "angle_offset", temp);
}

static void info_mem(void) {
    const char* t_names[] = {"def",
                             "logging",
                             "net",
                             "serial_iface",
                             "ota",
                             "modem",
                             "sensors",
                             "anemometer",
                             "button",
                             "watchdog",
                             NULL};
    TaskHandle_t t_handle;
    TaskStatus_t details;
    char temp[16];

    utoa(xPortGetMinimumEverFreeHeapSize(), temp, 10);
    print_info_str(s_logger, "HEAP", "~", temp);

    for (int i = 0; t_names[i]; i++) {
        t_handle = xTaskGetHandle(t_names[i]);
        if (!t_handle) {
            continue;
        }

        vTaskGetInfo(t_handle, &details, pdTRUE, eInvalid);
        utoa(details.usStackHighWaterMark * sizeof(StackType_t), temp, 10);
        print_info_str(s_logger, "STACK", t_names[i], temp);
    }
}

#endif /* LOGGER */

void task_logging(void* argument) {
#ifdef LOGGER
    extern USBD_HandleTypeDef hUsbDeviceFS;
    struct task_logging_ctx* ctx = argument;

    s_logger        = ctx->logger;
    main_stack_size = stack_size();

    osDelay(1000);

    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) {
        MX_USB_DEVICE_DeInit();
        vTaskDelete(NULL);
        return;
    }

    for (;;) {
        info_base();
        info_mem();

        osDelay(20000);
    }
#else
    vTaskDelete(NULL);
#endif
}
