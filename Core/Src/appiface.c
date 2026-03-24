/*
 * Application interface
 */

#include "appiface.h"

#include <stdlib.h>
#include <string.h>

#include "actual.h"
#include "cmsis_os.h"
#include "fws.h"
#include "rtctime.h"
#include "siface.h"
#include "strjson.h"
#include "task.h"

#define JSMN_HEADER
#include "jsmn.h"

#define JSON_MAX_TOKENS 8

#define APPIFACE_KEY "iface-appum-0.1"

struct appiface appif;

// https://github.com/zserge/jsmn/blob/master/example/simple.c
static int jsoneq(const char* json, jsmntok_t* tok, const char* s) {
    if (tok->type == JSMN_STRING && (int)strlen(s) == tok->end - tok->start &&
        strncmp(json + tok->start, s, tok->end - tok->start) == 0) {
        return 0;
    }
    return -1;
}

static int hextostr(const char* hexstr,
                    size_t strlen,
                    uint8_t* hexbuf,
                    size_t buflen) {
    char tmpbuf[3];

    if (strlen != buflen * 2) {
        return -1;
    }

    for (size_t i = 0; i < buflen; i++) {
        memcpy(tmpbuf, &hexstr[i * 2], 2);
        tmpbuf[2] = '\0';

        hexbuf[i] = strtoul(tmpbuf, NULL, 16);
    }

    return 0;
}

static int parse(struct appiface* self,
                 const char* request,
                 char* response,
                 size_t resp_size) {
    jsmn_parser parser;
    jsmntok_t tokens[JSON_MAX_TOKENS];
    jsmntok_t* tcmd   = NULL;
    jsmntok_t* tparam = NULL;
    jsmntok_t* tvalue = NULL;
    size_t len, tmplen;
    uint32_t tmp;
    int ret;

    extern volatile struct bl_params bl;

    memset(tokens, 0, sizeof(tokens));
    jsmn_init(&parser);
    ret =
        jsmn_parse(&parser, request, strlen(request), tokens, JSON_MAX_TOKENS);

    if (ret <= 0) {
        return -1;
    }

    if (tokens[0].type != JSMN_OBJECT) {
        return -1;
    }

    for (int i = 1; (i + 1) < ret; i += 2) {
        if (jsoneq(request, &tokens[i], "cmd") == 0) {
            tcmd = &tokens[i + 1];
        } else if (jsoneq(request, &tokens[i], "param") == 0) {
            tparam = &tokens[i + 1];
        } else if (jsoneq(request, &tokens[i], "value") == 0) {
            tvalue = &tokens[i + 1];
        }
    }

    /* Commands */
    if (!tcmd) {
        return -1;
    }

    if (jsoneq(request, tcmd, "iface") == 0) {
        strjson_str(response, resp_size, "iface", APPIFACE_KEY);
    } else if (jsoneq(request, tcmd, "rd_param") == 0) {
        if (!tparam) {
            return -1;
        }

        if (jsoneq(request, tparam, "uid") == 0) {
            strjson_uint(response, resp_size, "uid", params.id);
        } else if (jsoneq(request, tparam, "ts") == 0) {
            strjson_uint(response, resp_size, "ts", timestamp);
        } else if (jsoneq(request, tparam, "ticks") == 0) {
            strjson_uint(response, resp_size, "ticks", xTaskGetTickCount());
        } else if (jsoneq(request, tparam, "name") == 0) {
            strjson_str(response, resp_size, "name", PARAMS_DEVICE_NAME);
        } else if (jsoneq(request, tparam, "bl_git") == 0) {
            strjson_str(response, resp_size, "bl_git", (char*)bl.hash);
        } else if (jsoneq(request, tparam, "bl_status") == 0) {
            strjson_uint(response, resp_size, "bl_status", bl.status);
        } else if (jsoneq(request, tparam, "app_git") == 0) {
            strjson_str(response, resp_size, "app_git", GIT_COMMIT_HASH);
        } else if (jsoneq(request, tparam, "app_ver") == 0) {
            strjson_uint(response, resp_size, "app_ver", PARAMS_FW_VERSION);
        } else if (jsoneq(request, tparam, "mcu") == 0) {
            strjson_str(response, resp_size, "mcu", params.mcu_uid);
        } else if (jsoneq(request, tparam, "apn") == 0) {
            strjson_str(response, resp_size, "apn", self->uparams.apn);
        } else if (jsoneq(request, tparam, "url_ota") == 0) {
            strjson_str(response, resp_size, "url_ota", self->uparams.url_ota);
        } else if (jsoneq(request, tparam, "url_app") == 0) {
            strjson_str(response, resp_size, "url_app", self->uparams.url_app);
        } else if (jsoneq(request, tparam, "period_upload") == 0) {
            strjson_uint(response,
                         resp_size,
                         "period_upload",
                         self->uparams.period_upload);
        } else if (jsoneq(request, tparam, "period_sensors") == 0) {
            strjson_uint(response,
                         resp_size,
                         "period_sensors",
                         self->uparams.period_sensors);
        } else if (jsoneq(request, tparam, "period_anemometer") == 0) {
            strjson_uint(response,
                         resp_size,
                         "period_anemometer",
                         self->uparams.period_anemometer);
        } else if (jsoneq(request, tparam, "bat") == 0) {
            xSemaphoreTake(self->actual->mutex, portMAX_DELAY);
            strjson_uint(response, resp_size, "bat", self->actual->voltage);
            xSemaphoreGive(self->actual->mutex);
        } else if (jsoneq(request, tparam, "wind_speed") == 0) {
            xSemaphoreTake(self->actual->mutex, portMAX_DELAY);
            strjson_uint(response,
                         resp_size,
                         "wind_speed",
                         self->actual->wind_speed);
            xSemaphoreGive(self->actual->mutex);
        } else if (jsoneq(request, tparam, "temp") == 0) {
            xSemaphoreTake(self->actual->mutex, portMAX_DELAY);
            strjson_int(response, resp_size, "temp", self->actual->temperature);
            xSemaphoreGive(self->actual->mutex);
        } else if (jsoneq(request, tparam, "hum") == 0) {
            xSemaphoreTake(self->actual->mutex, portMAX_DELAY);
            strjson_int(response, resp_size, "hum", self->actual->humidity);
            xSemaphoreGive(self->actual->mutex);
        } else if (jsoneq(request, tparam, "pressure") == 0) {
            xSemaphoreTake(self->actual->mutex, portMAX_DELAY);
            strjson_int(response,
                        resp_size,
                        "pressure",
                        self->actual->pressure);
            xSemaphoreGive(self->actual->mutex);
        } else if (jsoneq(request, tparam, "wind_direction") == 0) {
            xSemaphoreTake(self->actual->mutex, portMAX_DELAY);
            strjson_int(response,
                        resp_size,
                        "wind_direction",
                        self->actual->wind_direction);
            xSemaphoreGive(self->actual->mutex);
        } else if (jsoneq(request, tparam, "tamper") == 0) {
            return -1;  // TODO
        } else {
            return -1;
        }
    } else if (jsoneq(request, tcmd, "wr_param") == 0) {
        if (!tparam || !tvalue) {
            return -1;
        }

        len = tvalue->end - tvalue->start;

        if (jsoneq(request, tparam, "uid") == 0) {
            tmp = strtoul(request + tvalue->start, NULL, 10);
            if (tmp == 0) {
                return -1;
            }

            self->uparams.id = tmp;
        } else if (jsoneq(request, tparam, "apn") == 0) {
            tmplen = (sizeof(self->uparams.apn) - 1) < len
                         ? (sizeof(self->uparams.apn) - 1)
                         : len;
            strncpy(self->uparams.apn, request + tvalue->start, tmplen);
            self->uparams.apn[tmplen] = '\0';
        } else if (jsoneq(request, tparam, "url_ota") == 0) {
            tmplen = (sizeof(self->uparams.url_ota) - 1) < len
                         ? (sizeof(self->uparams.url_ota) - 1)
                         : len;
            strncpy(self->uparams.url_ota, request + tvalue->start, tmplen);
            self->uparams.url_ota[tmplen] = '\0';
        } else if (jsoneq(request, tparam, "url_app") == 0) {
            tmplen = (sizeof(self->uparams.url_app) - 1) < len
                         ? (sizeof(self->uparams.url_app) - 1)
                         : len;
            strncpy(self->uparams.url_app, request + tvalue->start, tmplen);
            self->uparams.url_app[tmplen] = '\0';
        } else if (jsoneq(request, tparam, "period_upload") == 0) {
            tmp = strtoul(request + tvalue->start, NULL, 10);
            if (tmp == 0) {
                return -1;
            }

            self->uparams.period_upload = tmp;
        } else if (jsoneq(request, tparam, "period_sensors") == 0) {
            tmp = strtoul(request + tvalue->start, NULL, 10);
            if (tmp == 0) {
                return -1;
            }

            if (tmp < self->uparams.period_anemometer) {
                return -1;
            }

            self->uparams.period_sensors = tmp;
        } else if (jsoneq(request, tparam, "period_anemometer") == 0) {
            tmp = strtoul(request + tvalue->start, NULL, 10);
            if (tmp == 0) {
                return -1;
            }

            if (tmp > self->uparams.period_sensors) {
                return -1;
            }

            self->uparams.period_anemometer = tmp;
        } else if (jsoneq(request, tparam, "secret") == 0) {
            ret = hextostr(request + tvalue->start,
                           len,
                           self->uparams.secret,
                           sizeof(self->uparams.secret));
            if (ret < 0) {
                return -1;
            }
        } else {
            return -1;
        }
    } else if (jsoneq(request, tcmd, "mem") == 0) {
        extern size_t stack_size(void);

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

        strjson_uint(response,
                     resp_size,
                     "heap",
                     xPortGetMinimumEverFreeHeapSize());
        strjson_uint(response, resp_size, "main_stack", stack_size());

        for (int i = 0; t_names[i]; i++) {
            TaskHandle_t handle = xTaskGetHandle(t_names[i]);
            if (!handle) {
                continue;
            }

            TaskStatus_t details;
            vTaskGetInfo(handle, &details, pdTRUE, eInvalid);
            strjson_uint(response,
                         resp_size,
                         t_names[i],
                         details.usStackHighWaterMark * sizeof(StackType_t));
        }
    } else if (jsoneq(request, tcmd, "save") == 0) {
        vTaskSuspendAll();
        params_set(&self->uparams);

        NVIC_SystemReset();  // --> RESET
    } else if (jsoneq(request, tcmd, "reset") == 0) {
        NVIC_SystemReset();  // --> RESET
    }

    strjson_str(response, resp_size, "status", "ok");
    return 0;
}

int appiface(void* iface) {
    struct siface* siface  = iface;
    struct appiface* appif = siface->context;
    char* response;
    char* tmp;
    int ret;

    tmp = pvPortMalloc(256);  // NOTE: ?
    if (!tmp) {
        return -1;
    }

    strcpy(tmp, "@,,");
    response         = tmp + strlen(tmp);
    size_t resp_size = 256 - strlen(tmp);
    strjson_init(response, resp_size);

    ret = parse(appif, siface->buf, response, resp_size);
    if (ret < 0) {
        strjson_str(response, resp_size, "status", "error");
    }

    strcat(response, "\r\n");

    ret = siface_add(siface, tmp);
    if (ret < 0) {
        vPortFree(tmp);
        return -1;
    }

    return 0;
}
