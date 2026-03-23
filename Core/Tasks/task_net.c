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
#include "fws.h"
#include "hmac.h"
#include "jsmn.h"
#include "logger.h"
#include "queue.h"
#include "sim800l.h"
#include "strjson.h"

#define TAG "NET"

#define JSON_MAX_TOKENS       8
#define MAX_RECORDS_PER_BATCH 56
#define SEND_RETRIES          3
#define REQUEST_BODY_SIZE     1024

#define TIME_UPDATE_PERIOD_MS (24 * 60 * 60 * 1000)

#define READ_TAMPER (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15))

extern volatile struct bl_params bl;

struct net_ctx {
    struct logger* logger;
    char* url;
    char* hmac;
    void* body;
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
/* Sensor data encoding (binary)                                              */
/******************************************************************************/

#define DATA_HEADER_SIZE 14
#define DATA_RECORD_SIZE 18

static int sensor_read(struct sensorq* queue,
                       struct sensor_record* records,
                       int max_count) {
    return sensorq_peek(queue, records, max_count);
}

/*
 * Build binary data payload into `out` buffer.
 * Reads sensor records directly into the payload body (zero-copy).
 *
 * Header (14 bytes, little-endian):
 *   0..3   uint32  uid
 *   4..7   uint32  ts
 *   8..11  uint32  ticks
 *   12     uint8   tamper (0/1)
 *   13     uint8   records count
 *
 * Records (N × 18 bytes each, little-endian):
 *   0..3   uint32  timestamp
 *   4..5   uint16  voltage (mV)
 *   6..7   int16   temperature (centidegrees C)
 *   8..9   uint16  humidity (centipercent RH)
 *   10..11 uint16  wind_direction (centidegrees)
 *   12..13 uint16  wind_speed_avg
 *   14..15 uint16  wind_speed_min
 *   16..17 uint16  wind_speed_max
 *
 * Returns total payload size in bytes.
 * Writes number of records read into *record_count_out.
 */
static size_t build_data_binary(void* out,
                                size_t out_size,
                                struct sensorq* queue,
                                int* record_count_out) {
    uint8_t* p      = (uint8_t*)out;
    int max_records = (int)((out_size - DATA_HEADER_SIZE) / DATA_RECORD_SIZE);

    if (max_records > MAX_RECORDS_PER_BATCH) {
        max_records = MAX_RECORDS_PER_BATCH;
    }

    /* Read records directly into payload body (struct layout = wire format) */
    struct sensor_record* records =
        (struct sensor_record*)(p + DATA_HEADER_SIZE);
    int record_count  = sensor_read(queue, records, max_records);
    *record_count_out = record_count;

    /* Header */
    memcpy(p + 0, &params.id, 4);
    memcpy(p + 4, (const void*)&timestamp, 4);
    uint32_t ticks = xTaskGetTickCount();
    memcpy(p + 8, &ticks, 4);
    p[12] = READ_TAMPER ? 1 : 0;
    p[13] = (uint8_t)record_count;

    return DATA_HEADER_SIZE + (size_t)record_count * DATA_RECORD_SIZE;
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

    hmac_base64(params.secret,
                ctx->body,
                strlen((const char*)ctx->body),
                ctx->hmac);

    request.type        = MODEM_REQ_HTTP_POST;
    request.url         = ctx->url;
    request.auth_header = ctx->hmac;
    request.body        = ctx->body;
    request.response    = &response;

    int result = modem_execute(&request);

    response_free(&response);

    return (result == 200) ? 0 : -1;
}

static int request_post_bin(struct net_ctx* ctx,
                            const char* api,
                            size_t body_len) {
    struct sim800l_http_response response = {0};
    struct modem_request request          = {0};

    strcpy(ctx->url, params.url_app);
    strcat(ctx->url, api);

    hmac_base64(params.secret, ctx->body, body_len, ctx->hmac);

    request.type        = MODEM_REQ_HTTP_POST_BIN;
    request.url         = ctx->url;
    request.auth_header = ctx->hmac;
    request.body        = ctx->body;
    request.body_length = body_len;
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
    strjson_uint(request, size, "period_upload", params.period_upload);
    strjson_uint(request, size, "period_sensors", params.period_sensors);
    strjson_uint(request, size, "period_anemometer", params.period_anemometer);
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

        /* Build binary data payload (reads from queue into body buffer) */
        int record_count;
        size_t body_len =
            build_data_binary(ctx.body, ctx.body_size, queue, &record_count);

        /* Try to send with retries */
        led_blink(4);
        bool sent = false;
        for (int attempt = 0; attempt < SEND_RETRIES; attempt++) {
            if (request_post_bin(&ctx, "/api/data", body_len) == 0) {
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
