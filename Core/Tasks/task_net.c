/*
 * Network task (HTTP data upload)
 */

#include <ctype.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

#include "actual.h"
#include "params.h"
#include "ptasks.h"
#include "rtctime.h"
#include "sensorq.h"

#define JSMN_HEADER
#include "base64.h"
#include "fws.h"
#include "hmac.h"
#include "jsmn.h"
#include "logger.h"
#include "queue.h"
#include "sim800l.h"
#include "strjson.h"

#define TAG "NET"

#define JSON_MAX_TOKENS       8
#define MAX_RECORDS_PER_BATCH 10
#define SEND_RETRIES          3
#define REQUEST_BODY_SIZE     1024

#define TIME_UPDATE_PERIOD_MS (24 * 60 * 60 * 1000)

#define READ_TAMPER (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15))

extern volatile struct bl_params bl;

struct sensor_item {
    uint32_t value;
    uint32_t timestamp;
};

#define SENSORS_BUF_LEN (MAX_RECORDS_PER_BATCH * sizeof(struct sensor_item))
#define SENSORS_STR_LEN (4 * ((SENSORS_BUF_LEN + 2) / 3) + 1)

struct net_ctx {
    struct logger* logger;
    char* url;
    char* hmac;
    char* body;
    size_t body_size;
};

/******************************************************************************/
/* Helpers                                                                    */
/******************************************************************************/

static void strtolower(char* str) {
    while (*str) {
        *str = tolower(*str);
        str++;
    }
}

static int jsoneq(const char* json, jsmntok_t* tok, const char* s) {
    if (tok->type == JSMN_STRING && (int)strlen(s) == tok->end - tok->start &&
        strncmp(json + tok->start, s, tok->end - tok->start) == 0) {
        return 0;
    }
    return -1;
}

static int parse_time_response(const char* json, uint32_t* timestamp_out) {
    jsmn_parser parser;
    jsmntok_t tokens[JSON_MAX_TOKENS];

    memset(tokens, 0, sizeof(tokens));
    jsmn_init(&parser);
    int count =
        jsmn_parse(&parser, json, strlen(json), tokens, JSON_MAX_TOKENS);

    if (count <= 0 || tokens[0].type != JSMN_OBJECT) {
        return -1;
    }

    for (int i = 1; (i + 1) < count; i += 2) {
        if (jsoneq(json, &tokens[i], "ts") == 0) {
            *timestamp_out = strtoul(json + tokens[i + 1].start, NULL, 0);
            return 0;
        }
    }

    return -1;
}

static bool verify_response(const char* body,
                            size_t length,
                            const char* authorization,
                            char* hmac_buffer) {
    if (!authorization) {
        return false;
    }

    hmac_base64(params.secret, body, length, hmac_buffer);
    strtolower(hmac_buffer);
    return strcmp(hmac_buffer, authorization) == 0;
}

static void response_free(struct sim800l_http_response* response) {
    if (response->body) {
        vPortFree(response->body);
        response->body = NULL;
    }
    if (response->authorization) {
        vPortFree(response->authorization);
        response->authorization = NULL;
    }
}

/******************************************************************************/
/* Sensor data encoding                                                       */
/******************************************************************************/

static int sensor_read(struct sensorq* queue,
                       struct sensor_record* records,
                       int max_count) {
    return sensorq_peek(queue, records, max_count);
}

static void sensor_field_encode(struct sensor_record* records,
                                int count,
                                size_t field_offset,
                                int is_signed,
                                int scale,
                                char* output) {
    struct sensor_item items[MAX_RECORDS_PER_BATCH];
    size_t encoded_length;

    for (int i = 0; i < count; i++) {
        const uint8_t* p = (const uint8_t*)&records[i] + field_offset;
        int32_t raw;
        if (is_signed) {
            raw = *(const int16_t*)p;
        } else {
            raw = *(const uint16_t*)p;
        }
        items[i].timestamp = records[i].timestamp;
        items[i].value     = (uint32_t)(raw * scale);
    }

    base64_encode((unsigned char*)items,
                  sizeof(struct sensor_item) * count,
                  output,
                  &encoded_length);
    output[encoded_length] = '\0';
}

/******************************************************************************/
/* Modem request wrappers                                                     */
/******************************************************************************/

static int request_time(struct net_ctx* ctx) {
    struct sim800l_http_response response = {0};
    struct modem_request request          = {0};
    int ret                               = -1;

    strcpy(ctx->url, params.url_app);
    strcat(ctx->url, "/api/time");

    request.type      = MODEM_REQ_HTTP_GET;
    request.url       = ctx->url;
    request.read_auth = true;
    request.response  = &response;

    do {
        if (modem_execute(&request) != 200) {
            break;
        }

        if (!verify_response(response.body,
                             response.body_length,
                             response.authorization,
                             ctx->hmac)) {
            break;
        }

        uint32_t ts;
        if (parse_time_response(response.body, &ts) != 0) {
            break;
        }

        set_timestamp(ts);

        LOG_I(ctx->logger, TAG, response.body);

        ret = 0;
    } while (0);

    response_free(&response);
    return ret;
}

static int request_post(struct net_ctx* ctx, const char* api) {
    struct sim800l_http_response response = {0};
    struct modem_request request          = {0};

    strcpy(ctx->url, params.url_app);
    strcat(ctx->url, api);

    hmac_base64(params.secret, ctx->body, strlen(ctx->body), ctx->hmac);

    request.type        = MODEM_REQ_HTTP_POST;
    request.url         = ctx->url;
    request.auth_header = ctx->hmac;
    request.body        = ctx->body;
    request.response    = &response;

    int result = modem_execute(&request);

    response_free(&response);

    return (result == 200) ? 0 : -1;
}

static int request_netscan(struct sim800l_netscan_result* result) {
    struct modem_request request = {0};

    request.type           = MODEM_REQ_NETSCAN;
    request.netscan_result = result;

    return modem_execute(&request);
}

/******************************************************************************/
/* Wait                                                                       */
/******************************************************************************/

static void wait_for_net_event(void) {
    xEventGroupWaitBits(task_events,
                        TASK_EVENT_NET_START,
                        pdTRUE,
                        pdFALSE,
                        portMAX_DELAY);
}

/******************************************************************************/
/* Build JSON payloads                                                        */
/******************************************************************************/

static void build_info_payload(char* request,
                               size_t size,
                               int available_sensors) {
    strjson_init(request, size);
    strjson_uint(request, size, "uid", params.id);
    strjson_uint(request, size, "ts", timestamp);
    strjson_str(request, size, "name", PARAMS_DEVICE_NAME);
    strjson_str(request, size, "bl_git", (char*)bl.hash);
    strjson_uint(request, size, "bl_status", bl.status);
    strjson_str(request, size, "app_git", GIT_COMMIT_HASH);
    strjson_uint(request, size, "app_ver", PARAMS_FW_VERSION);
    strjson_str(request, size, "mcu", params.mcu_uid);
    strjson_str(request, size, "apn", params.apn);
    strjson_str(request, size, "url_ota", params.url_ota);
    strjson_str(request, size, "url_app", params.url_app);
    strjson_uint(request, size, "period_app", params.period_app);
    strjson_uint(request, size, "period_sen", params.period_sen);
    strjson_uint(request, size, "mtime_count", params.mtime_count);
    strjson_int(request, size, "sens", available_sensors);
}

static void build_cnet_payload(char* request,
                               size_t size,
                               struct sim800l_netscan_result* scan) {
    /* Pick strongest cell */
    int best_cell_idx   = -1;
    int best_cell_level = -999;

    for (int i = 0; i < scan->count; i++) {
        if (scan->cells[i].level > best_cell_level) {
            best_cell_level = scan->cells[i].level;
            best_cell_idx   = i;
        }
    }

    strjson_init(request, size);
    strjson_uint(request, size, "uid", params.id);
    strjson_uint(request, size, "ts", timestamp);

    if (best_cell_idx >= 0) {
        strjson_int(request, size, "mcc", scan->cells[best_cell_idx].mcc);
        strjson_int(request, size, "mnc", scan->cells[best_cell_idx].mnc);
        strjson_int(request, size, "lac", scan->cells[best_cell_idx].lac);
        strjson_int(request, size, "cid", scan->cells[best_cell_idx].cid);
        strjson_int(request, size, "lev", scan->cells[best_cell_idx].level);
    } else {
        strjson_int(request, size, "mcc", 0);
        strjson_int(request, size, "mnc", 0);
        strjson_int(request, size, "lac", 0);
        strjson_int(request, size, "cid", 0);
        strjson_int(request, size, "lev", -113);
    }
}

static void build_data_payload(char* request,
                               size_t size,
                               struct sensor_record* records,
                               int record_count,
                               int voltage) {
    char encoded[SENSORS_STR_LEN];

    strjson_init(request, size);
    strjson_uint(request, size, "uid", params.id);
    strjson_uint(request, size, "ts", timestamp);
    strjson_uint(request, size, "ticks", xTaskGetTickCount());

    if (voltage) {
        strjson_int(request, size, "bat", voltage);
    }

    if (record_count > 0) {
        sensor_field_encode(records,
                            record_count,
                            offsetof(struct sensor_record, count_avg),
                            0,
                            1,
                            encoded);
        if (*encoded) {
            strjson_str(request, size, "count", encoded);
        }

        sensor_field_encode(records,
                            record_count,
                            offsetof(struct sensor_record, count_min),
                            0,
                            1,
                            encoded);
        if (*encoded) {
            strjson_str(request, size, "count_min", encoded);
        }

        sensor_field_encode(records,
                            record_count,
                            offsetof(struct sensor_record, count_max),
                            0,
                            1,
                            encoded);
        if (*encoded) {
            strjson_str(request, size, "count_max", encoded);
        }

        sensor_field_encode(records,
                            record_count,
                            offsetof(struct sensor_record, temperature),
                            1,
                            10,
                            encoded);
        if (*encoded) {
            strjson_str(request, size, "temp", encoded);
        }

        sensor_field_encode(records,
                            record_count,
                            offsetof(struct sensor_record, humidity),
                            0,
                            10,
                            encoded);
        if (*encoded) {
            strjson_str(request, size, "hum", encoded);
        }

        sensor_field_encode(records,
                            record_count,
                            offsetof(struct sensor_record, angle),
                            0,
                            10,
                            encoded);
        if (*encoded) {
            strjson_str(request, size, "angle", encoded);
        }
    }

    strjson_int(request, size, "tamper", READ_TAMPER);
}

/******************************************************************************/
/* Task entry point                                                           */
/******************************************************************************/

void task_net(void* argument) {
    struct task_net_ctx* task_ctx = argument;

    char url[PARAMS_APP_URL_SIZE + 32];
    char hmac[HMAC_BASE64_LEN];
    char request_body[REQUEST_BODY_SIZE];
    struct sensorq* queue = task_ctx->queue;

    struct net_ctx ctx = {
        .logger    = task_ctx->logger,
        .url       = url,
        .hmac      = hmac,
        .body      = request_body,
        .body_size = sizeof(request_body),
    };

    TickType_t last_time_update = 0;

    /* ------------------------------------------------------------------ */
    /* Startup sequence                                                    */
    /* ------------------------------------------------------------------ */

    wait_for_net_event();
    LOG_I(ctx.logger, TAG, "started");

    /* 1. Get server time */
    for (int attempt = 0; attempt < SEND_RETRIES; attempt++) {
        if (request_time(&ctx) == 0) {
            last_time_update = xTaskGetTickCount();
            break;
        }
        wait_for_net_event();
    }

    LOG_I(ctx.logger, TAG, "time synced");

    /* 2. Send station info → /api/info */
    xSemaphoreTake(task_ctx->actual->mutex, portMAX_DELAY);
    int available_sensors = task_ctx->actual->avail;
    xSemaphoreGive(task_ctx->actual->mutex);

    build_info_payload(request_body, sizeof(request_body), available_sensors);
    request_post(&ctx, "/api/info");

    LOG_I(ctx.logger, TAG, "info sent");

    /* 3. Network scan → /api/cnet */
    {
        struct sim800l_netscan_result scan_result;
        request_netscan(&scan_result);
        build_cnet_payload(request_body, sizeof(request_body), &scan_result);
    }
    request_post(&ctx, "/api/cnet");
    LOG_I(ctx.logger, TAG, "cnet sent");

    /* ------------------------------------------------------------------ */
    /* Main loop                                                           */
    /* ------------------------------------------------------------------ */

    for (;;) {
        /* Wait for scheduler trigger */
        wait_for_net_event();

        LOG_I(ctx.logger, TAG, "triggered");

        /* Update time if needed */
        if ((xTaskGetTickCount() - last_time_update) >=
            pdMS_TO_TICKS(TIME_UPDATE_PERIOD_MS)) {
            if (request_time(&ctx) == 0) {
                last_time_update = xTaskGetTickCount();
            }
        }

        /* Read voltage */
        xSemaphoreTake(task_ctx->actual->mutex, portMAX_DELAY);
        int voltage = task_ctx->actual->voltage;
        xSemaphoreGive(task_ctx->actual->mutex);

        /* Read sensor records from queue */
        struct sensor_record records[MAX_RECORDS_PER_BATCH];
        int record_count = sensor_read(queue, records, MAX_RECORDS_PER_BATCH);

        /* Build data payload */
        build_data_payload(ctx.body,
                           ctx.body_size,
                           records,
                           record_count,
                           voltage);

        /* Try to send with retries */
        led_blink(4);
        bool sent = false;
        for (int attempt = 0; attempt < SEND_RETRIES; attempt++) {
            if (request_post(&ctx, "/api/data") == 0) {
                sent = true;
                break;
            }
        }

        /* Drop records from queue only after successful send */
        if (sent) {
            sensorq_drop(queue, record_count);
        } else {
            LOG_E(ctx.logger, TAG, "data send failed");
        }

        /* If more data in queue, loop immediately */
        if (!sensorq_is_empty(queue)) {
            continue;
        }
    }
}
