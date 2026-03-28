/*
 * OTA task
 */

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"

#define JSMN_HEADER
#include "fws.h"
#include "hmac.h"
#include "jsmn.h"
#include "logger.h"
#include "params.h"
#include "ptasks.h"
#include "sim800l.h"
#include "w25q.h"
#include "w25q_s.h"

#define TAG "OTA"

static struct w25q_s* s_mem;

#define JSON_MAX_TOKENS 16

#define FILE_PART_SIZE   2048
#define FLASH_WRITE_SIZE 128
#define FLASH_READ_SIZE  32

#define RETRIES 5

extern const uint32_t* _app_len;
#define APP_LENGTH ((uint32_t)&_app_len)

// https://github.com/zserge/jsmn/blob/master/example/simple.c
static int jsoneq(const char* json, jsmntok_t* tok, const char* s) {
    if (tok->type == JSMN_STRING && (int)strlen(s) == tok->end - tok->start &&
        strncmp(json + tok->start, s, tok->end - tok->start) == 0) {
        return 0;
    }
    return -1;
}

static void jsoncpy(const char* json, jsmntok_t* tok, char* dst, size_t mlen) {
    size_t len = tok->end - tok->start;
    if (len > mlen - 1) {
        len = mlen - 1;
    }
    strncpy(dst, json + tok->start, len);
    dst[len] = '\0';
}

static int parse_json(const char* response,
                      struct fws* fws,
                      char* filename,
                      size_t mlen) {
    jsmn_parser parser;
    jsmntok_t tokens[JSON_MAX_TOKENS];
    jsmntok_t *token_file = NULL, *token_cs = NULL, *token_size = NULL;
    int ret;

    memset(tokens, 0, sizeof(tokens));
    jsmn_init(&parser);
    ret = jsmn_parse(&parser,
                     response,
                     strlen(response),
                     tokens,
                     JSON_MAX_TOKENS);

    if (ret <= 0) {
        return -1;
    }

    if (tokens[0].type != JSMN_OBJECT) {
        return -1;
    }

    for (int i = 1; (i + 1) < ret; i += 2) {
        if (jsoneq(response, &tokens[i], "file") == 0) {
            token_file = &tokens[i + 1];
        } else if (jsoneq(response, &tokens[i], "checksum") == 0) {
            token_cs = &tokens[i + 1];
        } else if (jsoneq(response, &tokens[i], "size") == 0) {
            token_size = &tokens[i + 1];
        }
    }

    if (!token_file || !token_cs || !token_size) {
        return -1;
    }

    char temp[16];

    fws->loaded = 0;

    jsoncpy(response, token_cs, temp, sizeof(temp));
    fws->checksum = strtoul(temp, NULL, 0);

    jsoncpy(response, token_size, temp, sizeof(temp));
    fws->size = strtoul(temp, NULL, 0);

    jsoncpy(response, token_file, filename, mlen);

    return 0;
}

static void strtolower(char* data) {
    while (*data) {
        *data = tolower(*data);
        data++;
    }
}

static int ota_http_get(const char* url,
                        char* hmac_buf,
                        struct sim800l_http_response* response) {
    struct modem_request request = {0};

    request.type      = MODEM_REQ_HTTP_GET;
    request.url       = url;
    request.read_auth = true;
    request.response  = response;

    return modem_execute(&request);
}

static bool ota_verify_response(struct sim800l_http_response* response,
                                char* hmac_buf) {
    if (!response->authorization || !response->body) {
        return false;
    }

    hmac_base64(params.secret, response->body, response->body_length, hmac_buf);
    strtolower(hmac_buf);
    return strcmp(hmac_buf, response->authorization) == 0;
}

static void ota_response_free(struct sim800l_http_response* response) {
    if (response->body) {
        vPortFree(response->body);
        response->body = NULL;
    }
    if (response->authorization) {
        vPortFree(response->authorization);
        response->authorization = NULL;
    }
}

static void flash_erase(uint32_t addr, uint32_t size) {
    while (size) {
        w25q_s_sector_erase(s_mem, addr);

        if (size > W25Q_SECTOR_SIZE) {
            size -= W25Q_SECTOR_SIZE;
        } else {
            size = 0;
        }
        addr += W25Q_SECTOR_SIZE;
    }
}

static void flash_write(uint32_t addr, uint8_t* buf, uint16_t size) {
    uint16_t ws;

    while (size) {
        if (size > FLASH_WRITE_SIZE) {
            ws = FLASH_WRITE_SIZE;
        } else {
            ws = size;
        }

        w25q_s_write_data(s_mem, addr, buf, ws);

        size -= ws;
        addr += ws;
        buf += ws;
    }
}

static void flash_write_header(struct fws* fws) {
    w25q_s_sector_erase(s_mem, FWS_HEADER_ADDR);
    flash_write(FWS_HEADER_ADDR, (uint8_t*)fws, sizeof(struct fws));
}

static uint32_t flash_checksum(uint32_t addr, uint32_t size) {
    uint32_t checksum = FWS_CHECKSUM_INIT;
    uint8_t buf[FLASH_READ_SIZE];
    uint32_t ws;

    while (size) {
        if (size > FLASH_READ_SIZE) {
            ws = FLASH_READ_SIZE;
        } else {
            ws = size;
        }

        w25q_s_read_data(s_mem, addr, buf, ws);

        size -= ws;
        addr += ws;

        for (uint32_t i = 0; i < ws; i += sizeof(uint32_t)) {
            checksum += *((uint32_t*)&buf[i]);
        }
    }

    return checksum;
}

/******************************************************************************/
static void ota_check_update(struct logger* logger) {
    struct sim800l_http_response response;
    char filename[64];
    struct fws fws = {0};
    uint32_t addr;
    int retries;
    int ret;

    char url[PARAMS_OTA_URL_SIZE + 64];
    char hmac_buf[HMAC_BASE64_LEN];

    /* Request firmware list */
    LOG_I(logger, TAG, "checking for update");
    strcpy(url, params.url_ota);
    strcat(url, "/api/ota?uid=");
    {
        char uid_str[12];
        utoa(params.id, uid_str, 10);
        strcat(url, uid_str);
    }

    memset(&response, 0, sizeof(response));
    ret = ota_http_get(url, hmac_buf, &response);

    /* No update available */
    if (ret == 204) {
        LOG_I(logger, TAG, "no update available");
        ota_response_free(&response);
        return;
    }

    if (ret != 200 || !ota_verify_response(&response, hmac_buf)) {
        LOG_W(logger, TAG, "ota request failed");
        ota_response_free(&response);
        return;
    }

    /* Parse firmware info */
    ret = parse_json(response.body, &fws, filename, sizeof(filename));
    ota_response_free(&response);

    if (ret != 0) {
        LOG_E(logger, TAG, "invalid ota response");
        return;
    }

    /* Too big firmware */
    if (fws.size > APP_LENGTH) {
        LOG_E(logger, TAG, "firmware too big");
        return;
    }

    /* Erase SPI FLASH */
    LOG_I(logger, TAG, "erasing flash");
    flash_erase(FWS_PAYLOAD_ADDR, fws.size);

    /* Download firmware in chunks */
    LOG_I(logger, TAG, "downloading firmware");
    addr    = 0;
    retries = RETRIES;
    while (retries && addr < fws.size) {
        /* Build file request URL */
        strcpy(url, params.url_ota);
        strcat(url, "/api/ota/file?file=");
        strcat(url, filename);
        strcat(url, "&addr=");
        utoa(addr, &url[strlen(url)], 10);
        strcat(url, "&size=");
        utoa(FILE_PART_SIZE, &url[strlen(url)], 10);

        memset(&response, 0, sizeof(response));
        ret = ota_http_get(url, hmac_buf, &response);
        if (ret != 200 || !ota_verify_response(&response, hmac_buf)) {
            LOG_W(logger, TAG, "chunk download failed");
            ota_response_free(&response);
            retries--;
            continue;
        }

        /* Write data to SPI FLASH */
        flash_write(FWS_PAYLOAD_ADDR + addr,
                    (uint8_t*)response.body,
                    response.body_length);

        addr += response.body_length;
        retries = RETRIES;

        ota_response_free(&response);
    }

    /* Expand to a multiple of 4 */
    if (fws.size % sizeof(uint32_t)) {
        int expand   = sizeof(uint32_t) - fws.size % sizeof(uint32_t);
        uint8_t byte = 0xFF;

        fws.size += expand;
        while (expand) {
            flash_write(FWS_PAYLOAD_ADDR + addr, &byte, 1);
            expand--;
            addr++;
        }
    }

    /* Checksum */
    if (flash_checksum(FWS_PAYLOAD_ADDR, fws.size) != fws.checksum) {
        LOG_E(logger, TAG, "checksum mismatch");
        return;
    }

    fws.loaded = 0;
    flash_write_header(&fws);

    /* Update SPI FLASH header */
    LOG_I(logger, TAG, "update verified, rebooting");

    /* Reset */
    vTaskDelay(pdMS_TO_TICKS(100));
    NVIC_SystemReset();
}

/******************************************************************************/
void task_ota(void* argument) {
    struct task_ota_ctx* ctx = argument;

    s_mem = ctx->mem;

    w25q_power_on(&s_mem->mem);
    if (w25q_s_get_manufacturer_id(s_mem) != FWS_WINBOND_MANUFACTURER_ID) {
        LOG_E(ctx->logger, TAG, "flash not found");
        w25q_power_down(&s_mem->mem);
        vTaskDelete(NULL);
    }
    w25q_power_down(&s_mem->mem);

    for (;;) {
        xEventGroupWaitBits(task_events,
                            TASK_EVENT_OTA_START,
                            pdTRUE,
                            pdFALSE,
                            portMAX_DELAY);

        w25q_power_on(&s_mem->mem);
        ota_check_update(ctx->logger);
        w25q_power_down(&s_mem->mem);
    }
}
