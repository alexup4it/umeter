/*
 * SIM800L driver (synchronous blocking API)
 */

#ifndef SIM800L_H_
#define SIM800L_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "cmsis_os.h"
#include "stm32f4xx_hal.h"
#include "stream_buffer.h"

struct logger;

#define SIM800L_BUFFER_SIZE      1024
#define SIM800L_UART_BUFFER_SIZE 128
#define SIM800L_APN_SIZE         32

/* HTTP response (caller must free response/auth with vPortFree) */
struct sim800l_http_response {
    char* body;
    size_t body_length;
    char* authorization; /* NULL if not requested */
};

/* Network scan result (single cell) */
struct sim800l_cell {
    int32_t mcc;
    int32_t mnc;
    int32_t lac;
    int32_t cid;
    int32_t level; /* dBm */
};

#define SIM800L_NETSCAN_MAX_CELLS 8

/* Network scan results */
struct sim800l_netscan_result {
    struct sim800l_cell cells[SIM800L_NETSCAN_MAX_CELLS];
    int count;
};

struct sim800l {
    UART_HandleTypeDef* uart;
    StreamBufferHandle_t stream;

    char tx_buffer[SIM800L_BUFFER_SIZE + 2]; /* +\r\n */
    char rx_buffer[SIM800L_BUFFER_SIZE + 1]; /* +\0 */
    char* rx_current;

    size_t rx_length;

    char apn[SIM800L_APN_SIZE];
    struct logger* logger;

    char dma_buffer[SIM800L_UART_BUFFER_SIZE];
};

/**
 * Initialize the driver (does NOT power on the modem).
 */
void sim800l_init(struct sim800l* self,
                  UART_HandleTypeDef* uart,
                  const char* apn,
                  struct logger* logger);

/**
 * Feed received UART DMA data into the driver.
 * Call from HAL_UARTEx_RxEventCallback.
 */
void sim800l_irq(struct sim800l* self, size_t length);

/**
 * Power on the modem, wait for AT response, disable echo, delete SMS.
 * Caller must power on hardware before calling this.
 * @return 0 on success, -1 on failure
 */
int sim800l_startup(struct sim800l* self);

/**
 * Wait for network registration (CREG 0,1).
 * @param timeout_ms maximum wait time in ms
 * @return 0 on success, -1 on timeout
 */
int sim800l_wait_network(struct sim800l* self, uint32_t timeout_ms);

/**
 * Open GPRS bearer (SAPBR).
 * @return 0 on success, -1 on failure
 */
int sim800l_gprs_open(struct sim800l* self);

/**
 * Close GPRS bearer.
 * @return 0 on success, -1 on failure
 */
int sim800l_gprs_close(struct sim800l* self);

/**
 * HTTP GET request.
 * @param url        full URL
 * @param auth_header  Authorization header value to send (NULL = none)
 * @param read_auth  if true, parse Authorization from response headers
 * @param response   output (caller frees body/authorization with vPortFree)
 * @return HTTP status code (200 = OK), or -1 on failure
 */
int sim800l_http_get(struct sim800l* self,
                     const char* url,
                     const char* auth_header,
                     bool read_auth,
                     struct sim800l_http_response* response);

/**
 * HTTP POST request (Content-Type: application/json).
 * @param url        full URL
 * @param auth_header  Authorization header value to send (NULL = none)
 * @param body       JSON body string
 * @param response   output (caller frees body/authorization with vPortFree)
 * @return HTTP status code (200 = OK), or -1 on failure
 */
int sim800l_http_post(struct sim800l* self,
                      const char* url,
                      const char* auth_header,
                      const char* body,
                      struct sim800l_http_response* response);

/**
 * HTTP POST request with binary body (Content-Type: application/octet-stream).
 * @param url        full URL
 * @param auth_header  Authorization header value to send (NULL = none)
 * @param body       binary body (may contain NUL bytes)
 * @param body_len   body length in bytes
 * @param response   output (caller frees body/authorization with vPortFree)
 * @return HTTP status code (200 = OK), or -1 on failure
 */
int sim800l_http_post_bin(struct sim800l* self,
                          const char* url,
                          const char* auth_header,
                          const void* body,
                          size_t body_len,
                          struct sim800l_http_response* response);

/**
 * Perform network scan (AT+CNETSCAN).
 * @param result  output with found cells
 * @return 0 on success, -1 on failure
 */
int sim800l_netscan(struct sim800l* self,
                    struct sim800l_netscan_result* result);

#endif /* SIM800L_H_ */
