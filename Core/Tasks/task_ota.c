/*
 * OTA task
 *
 * Over-the-air firmware update (merged from ota lib)
 *
 * Dmitry Proshutinsky <dproshutinsky@gmail.com>
 * 2024-2026
 */

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

#include "cmsis_os.h"
#include "task.h"

#define JSMN_HEADER
#include "fws.h"
#include "hmac.h"
#include "jsmn.h"
#include "params.h"
#include "ptasks.h"
#include "sim800l.h"
#include "w25q_s.h"

static struct sim800l* s_mod;
static struct w25q_s* s_mem;

#define OTA_CHECK_INTERVAL_MS (2 * 60 * 60 * 1000)

#define DELAY_HTTP_MS    60000
#define DELAY_SIM800L_MS (DELAY_HTTP_MS * 5)

#define JSON_MAX_TOKENS 32

#define FILE_PART_SIZE   512
#define FLASH_WRITE_SIZE 128
#define FLASH_READ_SIZE  32

#define RETRIES 5

extern const uint32_t* _app_len;
#define APP_LENGTH ((uint32_t)&_app_len)

static TaskHandle_t ota_handle;

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
    jsmntok_t* object;
    jsmntok_t *token_file, *token_ver, *token_cs, *token_size;
    uint32_t vermax = 0;
    int i, ret;

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

    if (tokens[0].type != JSMN_ARRAY) {
        return -1;
    }

    i = 1;
    for (int objs = 0; objs < tokens[0].size && i < ret; objs++) {
        object = &tokens[i];
        if (object->type != JSMN_OBJECT) {
            return -1;
        }

        i++;
        token_file = NULL;
        token_ver  = NULL;
        token_cs   = NULL;
        token_size = NULL;
        for (int items = 0; items < object->size && (i + 1) < ret; items++) {
            if (jsoneq(response, &tokens[i], "file") == 0) {
                token_file = &tokens[i + 1];
            } else if (jsoneq(response, &tokens[i], "ver") == 0) {
                token_ver = &tokens[i + 1];
            } else if (jsoneq(response, &tokens[i], "checksum") == 0) {
                token_cs = &tokens[i + 1];
            } else if (jsoneq(response, &tokens[i], "size") == 0) {
                token_size = &tokens[i + 1];
            }

            i += 2;
        }

        if (token_file && token_ver && token_cs && token_size) {
            char temp[16];
            uint32_t ver;

            jsoncpy(response, token_ver, temp, sizeof(temp));
            ver = strtoul(temp, NULL, 0);

            if (ver > vermax) {
                vermax = ver;

                fws->loaded  = 0;
                fws->version = ver;
                jsoncpy(response, token_cs, temp, sizeof(temp));
                fws->checksum = strtoul(temp, NULL, 0);
                jsoncpy(response, token_size, temp, sizeof(temp));
                fws->size = strtoul(temp, NULL, 0);

                jsoncpy(response, token_file, filename, mlen);
            }
        }
    }

    return vermax;
}

static void ota_callback(int status, void* data) {
    struct sim800l_http* http = data;
    TaskHandle_t task         = *((TaskHandle_t*)http->context);

    BaseType_t woken = pdFALSE;
    vTaskNotifyGiveFromISR(task, &woken);
}

static int request_list(struct sim800l_http* http) {
    strcpy(http->url, params.url_ota);
    strcat(http->url, "/api/list?name=");
    strcat(http->url, PARAMS_DEVICE_NAME);
    http->req_auth     = NULL;
    http->res_auth     = NULL;
    http->res_auth_get = true;
    http->request      = NULL;
    http->response     = NULL;
    http->context      = &ota_handle;

    return sim800l_http(s_mod, http, ota_callback, DELAY_HTTP_MS);
}

static int request_file(struct sim800l_http* http,
                        const char* filename,
                        uint32_t addr,
                        uint32_t size) {
    strcpy(http->url, params.url_ota);
    strcat(http->url, "/api/file?file=");
    strcat(http->url, filename);
    strcat(http->url, "&addr=");
    utoa(addr, &http->url[strlen(http->url)], 10);
    strcat(http->url, "&size=");
    utoa(size, &http->url[strlen(http->url)], 10);
    http->req_auth     = NULL;
    http->res_auth     = NULL;
    http->res_auth_get = true;
    http->request      = NULL;
    http->response     = NULL;
    http->context      = &ota_handle;

    return sim800l_http(s_mod, http, ota_callback, DELAY_HTTP_MS);
}

static void flash_erase(uint32_t addr, uint16_t size) {
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

static void strtolower(char* data) {
    while (*data) {
        *data = tolower(*data);
        data++;
    }
}

/******************************************************************************/
void task_ota(void* argument) {
    struct task_ota_ctx* ctx = argument;
    struct sim800l_http http;
    char filename[64];
    struct fws fws;
    uint32_t addr;
    int retries;
    int ret;

    char url[PARAMS_OTA_URL_SIZE + 64];
    char hmac_buf[HMAC_BASE64_LEN];

    s_mod = ctx->mod;
    s_mem = ctx->mem;

    if (w25q_s_get_manufacturer_id(s_mem) != FWS_WINBOND_MANUFACTURER_ID) {
        vTaskDelete(NULL);
    }

    ota_handle = xTaskGetCurrentTaskHandle();
    http.url   = url;

    goto startup;

    for (;;) {
        osDelay(OTA_CHECK_INTERVAL_MS);

    startup:
        // Request firmware list
        ret = request_list(&http);
        if (ret) {
            continue;
        }
        ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(DELAY_SIM800L_MS));

        // Error
        if (!http.res_auth || !http.response) {
            if (http.res_auth) {
                vPortFree(http.res_auth);
            }
            if (http.response) {
                vPortFree(http.response);
            }
            continue;
        }

        // Authorization
        hmac_base64(params.secret, http.response, http.rlen, hmac_buf);
        strtolower(hmac_buf);
        if (strcmp(hmac_buf, http.res_auth) != 0) {
            vPortFree(http.res_auth);
            vPortFree(http.response);
            continue;
        }

        // Parse firmware list
        ret = parse_json(http.response, &fws, filename, sizeof(filename));
        vPortFree(http.res_auth);
        vPortFree(http.response);

        // No update or error
        if (ret <= PARAMS_FW_VERSION) {
            continue;
        }

        // Too big firmware
        if (fws.size > APP_LENGTH) {
            continue;
        }

        // Erase SPI FLASH
        flash_erase(FWS_PAYLOAD_ADDR, fws.size);

        // Updating
        addr    = 0;
        retries = RETRIES;
        while (retries && addr < fws.size) {
            // Request newest firmware file
            ret = request_file(&http, filename, addr, FILE_PART_SIZE);
            if (ret) {
                retries--;
                continue; /* while */
            }
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(DELAY_SIM800L_MS));

            // Error
            if (!http.res_auth || !http.response) {
                if (http.res_auth) {
                    vPortFree(http.res_auth);
                }
                if (http.response) {
                    vPortFree(http.response);
                }
                retries--;
                continue; /* while */
            }

            // Authorization
            hmac_base64(params.secret, http.response, http.rlen, hmac_buf);
            strtolower(hmac_buf);
            if (strcmp(hmac_buf, http.res_auth) != 0) {
                vPortFree(http.res_auth);
                vPortFree(http.response);
                retries--;
                continue; /* while */
            }

            // Write data to SPI FLASH
            flash_write(FWS_PAYLOAD_ADDR + addr,
                        (uint8_t*)http.response,
                        http.rlen);

            addr += http.rlen;
            retries = RETRIES;  // Reset retries

            vPortFree(http.res_auth);
            vPortFree(http.response);
        }

        // Expand to a multiple of 4
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

        // Checksum
        if (flash_checksum(FWS_PAYLOAD_ADDR, fws.size) != fws.checksum) {
            continue;
        }

        // Update SPI FLASH header
        fws.loaded = 0;
        flash_write_header(&fws);

        // Reset
        osDelay(100);
        NVIC_SystemReset();
    }
}
