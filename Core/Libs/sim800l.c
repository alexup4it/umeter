/*
 * SIM800L driver (synchronous blocking API)
 */

#include "sim800l.h"

#include <stdlib.h>
#include <string.h>

#include "logger.h"
#include "task.h"

#ifdef LOGGER
#    define TAG "SIM800L"
#endif

/******************************************************************************/
/* Low-level UART helpers                                                     */
/******************************************************************************/

void sim800l_init(struct sim800l* self,
                  UART_HandleTypeDef* uart,
                  const char* apn,
                  struct logger* logger) {
    memset(self, 0, sizeof(*self));
    self->uart   = uart;
    self->logger = logger;
    self->stream = xStreamBufferCreate(SIM800L_BUFFER_SIZE, 1);

    strncpy(self->apn, apn, sizeof(self->apn) - 1);
    self->apn[sizeof(self->apn) - 1] = '\0';
}

void sim800l_irq(struct sim800l* self, size_t length) {
    BaseType_t woken = pdFALSE;
    xStreamBufferSendFromISR(self->stream, self->dma_buffer, length, &woken);
}

static void clear_rx(struct sim800l* self) {
    xStreamBufferReset(self->stream);
    self->rx_length    = 0;
    self->rx_current   = self->rx_buffer;
    self->rx_buffer[0] = '\0';
}

static void transmit(struct sim800l* self, const char* data) {
    size_t length = strlen(data);
    if (length > SIM800L_BUFFER_SIZE) {
        length = SIM800L_BUFFER_SIZE;
    }

    memcpy(self->tx_buffer, data, length);
    self->tx_buffer[length]     = '\r';
    self->tx_buffer[length + 1] = '\n';
    length += 2;

#ifdef LOGGER
    logger_add(self->logger, TAG, false, (char*)self->tx_buffer, length);
#endif

    clear_rx(self);
    while (
        HAL_UART_Transmit_DMA(self->uart, self->tx_buffer, length) == HAL_BUSY
    );
}

/**
 * Wait until substring appears in rx buffer.
 * if end is NULL, then searches only start
 * @return pointer next to start, NULL on timeout
 */
static char* read_to(struct sim800l* self,
                     const uint32_t timeout_ms,
                     const char* substr) {
    const TickType_t started_at    = xTaskGetTickCount();
    const TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    char* result                   = NULL;

    for (;;) {
        char* found = strstr(self->rx_current, substr);
        if (found) {
            found[0] = '\0';

            result           = self->rx_current;
            self->rx_current = found + strlen(substr);
            break;
        }

        const TickType_t elapsed = xTaskGetTickCount() - started_at;
        if (elapsed >= timeout_ticks) {
            break;
        }

        if (self->rx_length >= SIM800L_BUFFER_SIZE) {
            break;
        }

        uint8_t* dest = self->rx_buffer + self->rx_length;
        size_t received =
            xStreamBufferReceive(self->stream,
                                 dest,
                                 SIM800L_BUFFER_SIZE - self->rx_length,
                                 timeout_ticks - elapsed);
        self->rx_length += received;
        self->rx_buffer[self->rx_length] = '\0';

#ifdef LOGGER
        if (received) {
            logger_add(self->logger, TAG, false, (char*)dest, received);
        }
#endif
    }

    return result;
}

static char* read_n(struct sim800l* self, const uint32_t timeout_ms, size_t n) {
    const TickType_t started_at    = xTaskGetTickCount();
    const TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);

    for (;;) {
        size_t available = self->rx_buffer + self->rx_length - self->rx_current;
        if (available >= n) {
            char* result = self->rx_current;
            self->rx_current += n;
            return result;
        }

        const TickType_t elapsed = xTaskGetTickCount() - started_at;
        if (elapsed >= timeout_ticks) {
            break;
        }

        if (self->rx_length >= SIM800L_BUFFER_SIZE) {
            break;
        }

        uint8_t* dest = self->rx_buffer + self->rx_length;
        size_t received =
            xStreamBufferReceive(self->stream,
                                 dest,
                                 SIM800L_BUFFER_SIZE - self->rx_length,
                                 timeout_ticks - elapsed);
        self->rx_length += received;
        self->rx_buffer[self->rx_length] = '\0';

#ifdef LOGGER
        if (received) {
            logger_add(self->logger, TAG, false, (char*)dest, received);
        }
#endif
    }

    return NULL;
}

/**
 * Wait for OK or ERROR.
 * @return true if OK, false on ERROR/timeout
 */
static bool wait_for_ok(struct sim800l* self, const uint32_t timeout_ms) {
    const TickType_t started_at    = xTaskGetTickCount();
    const TickType_t timeout_ticks = pdMS_TO_TICKS(timeout_ms);
    bool found_ok                  = false;

    for (;;) {
        if (strstr((char*)self->rx_buffer, "OK")) {
            found_ok = true;
            break;
        }

        if (strstr((char*)self->rx_buffer, "ERROR")) {
            break;
        }

        const TickType_t elapsed = xTaskGetTickCount() - started_at;
        if (elapsed >= timeout_ticks) {
            break;
        }

        if (self->rx_length >= SIM800L_BUFFER_SIZE) {
            break;
        }

        uint8_t* dest = self->rx_buffer + self->rx_length;
        size_t received =
            xStreamBufferReceive(self->stream,
                                 dest,
                                 SIM800L_BUFFER_SIZE - self->rx_length,
                                 timeout_ticks - elapsed);
        self->rx_length += received;
        self->rx_buffer[self->rx_length] = '\0';

#ifdef LOGGER
        if (received) {
            logger_add(self->logger, TAG, false, (char*)dest, received);
        }
#endif
    }

    return found_ok;
}

void shuft_rx_to_current(struct sim800l* self) {
    size_t offset    = self->rx_current - self->rx_buffer;
    size_t remaining = self->rx_length - offset;

    for (size_t i = 0; i < remaining; i++) {
        self->rx_buffer[i] = self->rx_current[i];
    }

    self->rx_buffer[remaining] = '\0';
    self->rx_length            = remaining;
    self->rx_current           = self->rx_buffer;
}

/**
 * Send AT command and wait for OK/ERROR.
 * @return true if OK, false otherwise
 */
static bool send_command(struct sim800l* self,
                         const char* command,
                         uint32_t timeout_ms) {
    transmit(self, command);
    return wait_for_ok(self, timeout_ms);
}

/******************************************************************************/
/* Startup / shutdown                                                         */
/******************************************************************************/

int sim800l_startup(struct sim800l* self) {
    /* Wait for AT response */
    TickType_t started_at = xTaskGetTickCount();
    bool at_ok            = false;
    bool echo             = false;

    while ((xTaskGetTickCount() - started_at) < pdMS_TO_TICKS(2000)) {
        transmit(self, "AT");
        if (wait_for_ok(self, 500)) {
            if (strstr((char*)self->rx_buffer, "AT")) {
                echo = true;
            }
            at_ok = true;
            break;
        }
    }

    if (!at_ok) {
        return -1;
    }

    /* Disable echo if present */
    if (echo) {
        if (!send_command(self, "ATE0", 500) ||
            !send_command(self, "AT&W", 500)) {
            return -1;
        }
    }

    /* Delete all SMS */
    send_command(self, "AT+CMGDA=6", 5000);

    return 0;
}

int sim800l_wait_network(struct sim800l* self, uint32_t timeout_ms) {
    const TickType_t started_at = xTaskGetTickCount();

    while ((xTaskGetTickCount() - started_at) < pdMS_TO_TICKS(timeout_ms)) {
        transmit(self, "AT+CREG?");
        if (read_to(self, 5000, "+CREG: 0,1")) {
            return 0;
        }
        osDelay(1000);
    }

    return -1;
}

/******************************************************************************/
/* GPRS bearer                                                                */
/******************************************************************************/

int sim800l_gprs_open(struct sim800l* self) {
    char cmd[96];

    if (!send_command(self, "AT+SAPBR=3,1,CONTYPE,GPRS", 500)) {
        return -1;
    }

    strcpy(cmd, "AT+SAPBR=3,1,APN,");
    strcat(cmd, self->apn);
    if (!send_command(self, cmd, 500)) {
        return -1;
    }

    if (!send_command(self, "AT+SAPBR=1,1", 20000)) {
        return -1;
    }

    transmit(self, "AT+SAPBR=2,1");
    if (!read_to(self, 20000, "+SAPBR: 1,1")) {
        return -1;
    }

    return 0;
}

int sim800l_gprs_close(struct sim800l* self) {
    send_command(self, "AT+SAPBR=0,1", 10000);
    return 0;
}

/******************************************************************************/
/* HTTP response parsers                                                      */
/******************************************************************************/

/* Parse +HTTPACTION: <method>,<status>,<datalen>
 * Returns HTTP status code or -1 on failure. Writes datalen to *out_length. */
static int parse_http_action(struct sim800l* self,
                             int* data_length,
                             uint32_t timeout_ms) {
    read_to(self, timeout_ms, "+HTTPACTION: ");
    char* p = read_to(self, timeout_ms, "\r\n");

    if (!p) {
        return -1;
    }

    p = strstr(p, ",");
    if (!p || !*(++p)) {
        return -1;
    }

    int status = strtoul(p, &p, 10);
    if (*p != ',' || !*(++p)) {
        return -1;
    }

    if (data_length) {
        *data_length = strtoul(p, NULL, 10);
    }

    return status;
}

/* Parse Authorization header from HTTPHEAD response.
 * Returns allocated string or NULL. */
static char* parse_http_head_authorization(struct sim800l* self,
                                           uint32_t timeout_ms) {
    read_to(self, timeout_ms, "authorization: ");
    char* p = read_to(self, timeout_ms, "\r\n");
    if (!p) {
        return NULL;
    }

    int length   = strlen(p) + 1;
    char* result = pvPortMalloc(length);
    if (!result) {
        return NULL;
    }

    memcpy(result, p, length);
    return result;
}

/* Parse HTTPREAD response body.
 * Returns allocated body and sets *out_length, or NULL on failure. */
static char* parse_http_read(struct sim800l* self,
                             size_t* out_length,
                             uint32_t timeout_ms) {
    read_to(self, timeout_ms, "+HTTPREAD: ");
    char* p = read_to(self, timeout_ms, "\r\n");

    if (!p) {
        return NULL;
    }

    int length = strtoul(p, &p, 10);
    if (!*p) {
        return NULL;
    }

    read_to(self, timeout_ms, "\r\n");
    if (!p) {
        return NULL;
    }

    p = read_n(self, timeout_ms, length);
    if (!p) {
        return NULL;
    }

    char* body = pvPortMalloc(length + 1);
    if (!body) {
        return NULL;
    }

    memcpy(body, p, length);
    body[length] = '\0';
    *out_length  = length;
    return body;
}

/******************************************************************************/
/* HTTP session (internal)                                                    */
/******************************************************************************/

static int http_setup(struct sim800l* self,
                      const char* url,
                      const char* auth_header,
                      const char* body) {
    char cmd[96];

    strcpy(cmd, "AT+HTTPPARA=URL,");
    strcat(cmd, url);
    if (!send_command(self, cmd, 2000)) {
        return -1;
    }

    if (auth_header) {
        strcpy(cmd, "AT+HTTPPARA=USERDATA,Authorization: ");
        strcat(cmd, auth_header);
        if (!send_command(self, cmd, 2000)) {
            return -1;
        }
    }

    if (body) {
        if (!send_command(self, "AT+HTTPPARA=CONTENT,application/json", 500)) {
            return -1;
        }

        strcpy(cmd, "AT+HTTPDATA=");
        utoa(strlen(body), &cmd[strlen(cmd)], 10);
        strcat(cmd, ",1000");
        transmit(self, cmd);
        if (!read_to(self, 1000, "DOWNLOAD")) {
            return -1;
        }

        if (!send_command(self, body, 500)) {
            return -1;
        }
    }

    return 0;
}

static int http_execute(struct sim800l* self,
                        const char* url,
                        const char* auth_header,
                        const char* body,
                        bool read_auth,
                        struct sim800l_http_response* response) {
    int is_post = (body != NULL);
    int result  = -1;

    memset(response, 0, sizeof(*response));

    if (!send_command(self, "AT+HTTPINIT", 2000) ||
        !send_command(self, "AT+HTTPPARA=CID,1", 2000)) {
        return -1;
    }

    do {
        if (http_setup(self, url, auth_header, body) != 0) {
            break;
        }

        /* Execute request */
        transmit(self, is_post ? "AT+HTTPACTION=1" : "AT+HTTPACTION=0");
        int http_status = parse_http_action(self, NULL, 30000);
        if (http_status != 200) {
            break;
        }

        /* Read authorization header if requested */
        if (read_auth) {
            transmit(self, "AT+HTTPHEAD");
            response->authorization = parse_http_head_authorization(self, 1000);
        }

        /* Read response body */
        transmit(self, "AT+HTTPREAD");
        response->body = parse_http_read(self, &response->body_length, 1000);
        if (!response->body) {
            break;
        }
        result = 200;
    } while (0);

    /* Terminate HTTP session */
    send_command(self, "AT+HTTPTERM", 2000);

    return result;
}

/******************************************************************************/
/* Public HTTP API                                                            */
/******************************************************************************/

int sim800l_http_get(struct sim800l* self,
                     const char* url,
                     const char* auth_header,
                     bool read_auth,
                     struct sim800l_http_response* response) {
    return http_execute(self, url, auth_header, NULL, read_auth, response);
}

int sim800l_http_post(struct sim800l* self,
                      const char* url,
                      const char* auth_header,
                      const char* body,
                      struct sim800l_http_response* response) {
    return http_execute(self, url, auth_header, body, false, response);
}

/******************************************************************************/
/* Network scan                                                               */
/******************************************************************************/

static int32_t get_param_value(const char* line, const char* param, int base) {
    const char* p = line;

    while (*p) {
        p = strstr(p, param);
        if (!p) {
            return -1;
        }
        p += strlen(param);

        if (*p == ':') {
            p++;
            break;
        }
    }

    if (!*p) {
        return -1;
    }

    return strtoul(p, NULL, base);
}

int sim800l_netscan(struct sim800l* self,
                    struct sim800l_netscan_result* result) {
    const TickType_t started_at_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;

    memset(result, 0, sizeof(*result));

    if (!send_command(self, "AT+CNETSCAN=1", 500)) {
        return -1;
    }

    transmit(self, "AT+CNETSCAN");

    for (;;) {
        char* p = read_to(self, 45000 - started_at_ms, "\r\n");

        if (strcmp(p, "OK") == 0) {
            return 0;
        }

        if (strlen(p) == 0) {
            continue;
        }

        if (result->count < SIM800L_NETSCAN_MAX_CELLS) {
            struct sim800l_cell* cell = &result->cells[result->count];

            cell->mcc   = get_param_value(p, "MCC", 10);
            cell->mnc   = get_param_value(p, "MNC", 10);
            cell->lac   = get_param_value(p, "Lac", 16);
            cell->cid   = get_param_value(p, "Cellid", 16);
            cell->level = get_param_value(p, "Rxlev", 10);

            if (cell->mcc >= 0 && cell->mnc >= 0 && cell->lac >= 0 &&
                cell->cid >= 0 && cell->level >= 0) {
                cell->level = cell->level - 113;
                result->count++;
            }
        }

        shuft_rx_to_current(self);
    }
}
