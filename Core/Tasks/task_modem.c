/*
 * Modem management task
 *
 * Owns SIM800L power, initialization, and request processing.
 * Other tasks submit requests via modem_request() and wait for completion.
 */

#include <string.h>

#include "actual.h"
#include "logger.h"
#include "main.h"
#include "ptasks.h"
#include "sim800l.h"
#include "task.h"

#ifdef LOGGER
#    define TAG "MODEM"
#endif

#define MODEM_QUEUE_SIZE 8

#define VOLTAGE_MIN_MV 3400

#define NETWORK_TIMEOUT_MS 30000

#define IDLE_POWER_OFF_MS 500

struct modem_ctx {
    struct sim800l* modem;
    struct logger* logger;
    void (*power_on)(void);
    void (*power_off)(void);
    bool ready;
    bool gprs_open;
};

static QueueHandle_t s_request_queue;

/******************************************************************************/
/* Power management                                                           */
/******************************************************************************/

static void modem_power_on(struct modem_ctx* ctx) {
    if (ctx->ready) {
        return;
    }

    ctx->power_on();

    /* Start DMA reception */
    HAL_UARTEx_ReceiveToIdle_DMA(ctx->modem->uart,
                                 (uint8_t*)ctx->modem->dma_buffer,
                                 SIM800L_UART_BUFFER_SIZE);
}

static void modem_power_off(struct modem_ctx* ctx) {
    if (!ctx->ready) {
        return;
    }

    if (ctx->gprs_open) {
        sim800l_gprs_close(ctx->modem);
        ctx->gprs_open = false;
    }

    ctx->power_off();
    ctx->ready = false;
}

static int modem_ensure_ready(struct modem_ctx* ctx) {
    if (ctx->ready) {
        return 0;
    }

    /* Check battery voltage */
    xSemaphoreTake(actual.mutex, portMAX_DELAY);
    int voltage = actual.voltage;
    xSemaphoreGive(actual.mutex);

    if (voltage > 0 && voltage < VOLTAGE_MIN_MV) {
#ifdef LOGGER
        logger_add_str(ctx->logger, TAG, false, "voltage too low");
#endif
        return -1;
    }

    modem_power_on(ctx);

    if (!sim800l_startup(ctx->modem)) {
#ifdef LOGGER
        logger_add_str(ctx->logger, TAG, false, "startup failed");
#endif
        modem_power_off(ctx);
        return -1;
    }

    if (!sim800l_wait_network(ctx->modem, NETWORK_TIMEOUT_MS)) {
#ifdef LOGGER
        logger_add_str(ctx->logger, TAG, false, "no network");
#endif
        modem_power_off(ctx);
        return -1;
    }

    ctx->ready = true;

    return 0;
}

static int modem_ensure_gprs(struct modem_ctx* ctx) {
    if (ctx->gprs_open) {
        return 0;
    }

    if (modem_ensure_ready(ctx) != 0) {
        return -1;
    }

    if (sim800l_gprs_open(ctx->modem) != 0) {
#ifdef LOGGER
        logger_add_str(ctx->logger, TAG, false, "GPRS open failed");
#endif
        modem_power_off(ctx);
        return -1;
    }

    ctx->gprs_open = true;
    return 0;
}

/******************************************************************************/
/* Request processing                                                         */
/******************************************************************************/

static int process_http_get(struct modem_ctx* ctx,
                            struct modem_request* request) {
    if (modem_ensure_gprs(ctx) != 0) {
        return -1;
    }

    return sim800l_http_get(ctx->modem,
                            request->url,
                            request->auth_header,
                            request->read_auth,
                            request->response);
}

static int process_http_post(struct modem_ctx* ctx,
                             struct modem_request* request) {
    if (modem_ensure_gprs(ctx) != 0) {
        return -1;
    }

    return sim800l_http_post(ctx->modem,
                             request->url,
                             request->auth_header,
                             request->body,
                             request->response);
}

static int process_netscan(struct modem_ctx* ctx,
                           struct modem_request* request) {
    if (modem_ensure_ready(ctx) != 0) {
        return -1;
    }

    return sim800l_netscan(ctx->modem, request->netscan_result);
}

/******************************************************************************/
/* Public API                                                                 */
/******************************************************************************/

void modem_init(void) {
    s_request_queue =
        xQueueCreate(MODEM_QUEUE_SIZE, sizeof(struct modem_request));
}

int modem_submit(struct modem_request* request) {
    if (xQueueSendToBack(s_request_queue, request, 0) != pdTRUE) {
        return -1;
    }
    return 0;
}

int modem_execute(struct modem_request* request) {
    int result;

    request->caller = xTaskGetCurrentTaskHandle();
    request->result = &result;

    if (modem_submit(request) != 0) {
        return -1;
    }

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    return result;
}

/******************************************************************************/
/* Task entry point                                                           */
/******************************************************************************/

void task_modem(void* argument) {
    struct task_modem_ctx* task_ctx = argument;
    struct modem_request request;

    struct modem_ctx ctx = {
        .modem     = task_ctx->modem,
        .logger    = task_ctx->logger,
        .power_on  = task_ctx->power_on,
        .power_off = task_ctx->power_off,
        .ready     = false,
        .gprs_open = false,
    };

    for (;;) {
        TickType_t wait =
            ctx.ready ? pdMS_TO_TICKS(IDLE_POWER_OFF_MS) : portMAX_DELAY;

        if (xQueueReceive(s_request_queue, &request, wait) == pdTRUE) {
            int result;

            switch (request.type) {
                case MODEM_REQ_HTTP_GET:
                    result = process_http_get(&ctx, &request);
                    break;
                case MODEM_REQ_HTTP_POST:
                    result = process_http_post(&ctx, &request);
                    break;
                case MODEM_REQ_NETSCAN:
                    result = process_netscan(&ctx, &request);
                    break;
                default:
                    result = -1;
                    break;
            }

            /* Store result and notify caller */
            if (request.result) {
                *request.result = result;
            }
            if (request.caller) {
                xTaskNotifyGive(request.caller);
            }
        } else if (ctx.ready) {
            /* Queue timed out — no requests for IDLE_POWER_OFF_MS */
#ifdef LOGGER
            logger_add_str(ctx.logger, TAG, false, "idle, powering off");
#endif
            modem_power_off(&ctx);
        }
    }
}
