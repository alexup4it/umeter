/*
 * Modem management task
 *
 * Owns SIM800L power, initialization, and request processing.
 * Other tasks submit requests via modem_request() and wait for completion.
 */

#include <string.h>

#include "actual.h"
#include "avoltage.h"
#include "logger.h"
#include "main.h"
#include "ptasks.h"
#include "sim800l.h"
#include "task.h"

#define TAG "MODEM"

#define MODEM_QUEUE_SIZE 8

#define VOLTAGE_MIN_MV 3600

#define NETWORK_TIMEOUT_MS 30000

#define IDLE_POWER_OFF_MS 500
#define MODEM_FAIL_HARD_RESET_THRESHOLD 3
#define MODEM_HARD_OFF_MS               5000

struct modem_ctx {
    struct actual* actual;
    struct sim800l* modem;
    struct avoltage* voltage;
    struct logger* logger;
    void (*power_on)(void);
    void (*power_off)(void);
    bool ready;
    bool gprs_open;
    uint32_t fail_streak;
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
    if (ctx->gprs_open) {
        sim800l_gprs_close(ctx->modem);
        ctx->gprs_open = false;
    }

    ctx->power_off();
    ctx->ready = false;
}

static void modem_hard_reset(struct modem_ctx* ctx, const char* reason) {
    LOG_W(ctx->logger, TAG, reason);
    modem_power_off(ctx);
    osDelay(pdMS_TO_TICKS(MODEM_HARD_OFF_MS));
}

static int modem_ensure_ready(struct modem_ctx* ctx) {
    if (ctx->ready) {
        return 0;
    }

    /* Check battery voltage (live ADC reading) */
    int voltage = avoltage(ctx->voltage);

    if (voltage < VOLTAGE_MIN_MV) {
        LOG_W(ctx->logger, TAG, "voltage too low");
        return -1;
    }

    modem_power_on(ctx);

    if (sim800l_startup(ctx->modem) != 0) {
        LOG_E(ctx->logger, TAG, "startup failed");
        modem_power_off(ctx);
        return -1;
    }

    if (sim800l_wait_network(ctx->modem, NETWORK_TIMEOUT_MS) != 0) {
        LOG_E(ctx->logger, TAG, "no network");
        modem_power_off(ctx);
        return -1;
    }

    ctx->ready = true;
    LOG_I(ctx->logger, TAG, "ready");

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
        LOG_E(ctx->logger, TAG, "GPRS open failed");
        modem_power_off(ctx);
        return -1;
    }

    ctx->gprs_open = true;
    LOG_I(ctx->logger, TAG, "GPRS open");
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
                             (const char*)request->body,
                             request->response);
}

static int process_http_post_bin(struct modem_ctx* ctx,
                                 struct modem_request* request) {
    if (modem_ensure_gprs(ctx) != 0) {
        return -1;
    }

    return sim800l_http_post_bin(ctx->modem,
                                 request->url,
                                 request->auth_header,
                                 request->body,
                                 request->body_length,
                                 request->response);
}

static int process_netscan(struct modem_ctx* ctx,
                           struct modem_request* request) {
    if (modem_ensure_ready(ctx) != 0) {
        return -1;
    }

    return sim800l_netscan(ctx->modem, request->netscan_result);
}

static int process_request(struct modem_ctx* ctx, struct modem_request* request) {
    switch (request->type) {
        case MODEM_REQ_HTTP_GET:
            return process_http_get(ctx, request);
        case MODEM_REQ_HTTP_POST:
            return process_http_post(ctx, request);
        case MODEM_REQ_HTTP_POST_BIN:
            return process_http_post_bin(ctx, request);
        case MODEM_REQ_NETSCAN:
            return process_netscan(ctx, request);
        default:
            return -1;
    }
}

static bool request_succeeded(enum modem_request_type type, int result) {
    switch (type) {
        case MODEM_REQ_HTTP_GET:
        case MODEM_REQ_HTTP_POST:
        case MODEM_REQ_HTTP_POST_BIN:
            return result >= 200 && result < 300;
        case MODEM_REQ_NETSCAN:
            return result == 0;
        default:
            return false;
    }
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
        .actual    = task_ctx->actual,
        .modem     = task_ctx->modem,
        .voltage   = task_ctx->voltage,
        .logger    = task_ctx->logger,
        .power_on  = task_ctx->power_on,
        .power_off = task_ctx->power_off,
        .ready     = false,
        .gprs_open = false,
        .fail_streak = 0,
    };

    for (;;) {
        TickType_t wait =
            ctx.ready ? pdMS_TO_TICKS(IDLE_POWER_OFF_MS) : portMAX_DELAY;

        if (xQueueReceive(s_request_queue, &request, wait) == pdTRUE) {
            int result = process_request(&ctx, &request);

            if (request_succeeded(request.type, result)) {
                ctx.fail_streak = 0;
            } else {
                ctx.fail_streak++;
                LOG_W(ctx.logger, TAG, "request failed");

                if (ctx.fail_streak >= MODEM_FAIL_HARD_RESET_THRESHOLD) {
                    modem_hard_reset(&ctx, "hard reset after repeated failures");
                    result = process_request(&ctx, &request);

                    if (request_succeeded(request.type, result)) {
                        ctx.fail_streak = 0;
                    } else {
                        ctx.fail_streak = 1;
                    }
                }
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
            LOG_I(ctx.logger, TAG, "idle, powering off");
            modem_power_off(&ctx);
        }
    }
}
