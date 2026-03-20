/*
 * Serial interface
 *
 * Dmitry Proshutinsky <dproshutinsky@gmail.com>
 * 2024-2026
 */

#include "siface.h"

#include <string.h>

#include "usbd_cdc_if.h"

#define EVENT_TX     (1 << 0)
#define EVENT_RX     (1 << 1)
#define EVENT_WAKEUP (1 << 2)

/******************************************************************************/
void siface_init(struct siface* siface, /*UART_HandleTypeDef *uart,*/
                 size_t qsize,
                 siface_cb callback,
                 void* context) {
    memset(siface, 0, sizeof(*siface));
    //	siface->uart = uart;
    siface->events   = xEventGroupCreate();
    siface->stream   = xStreamBufferCreate(SIFACE_BUFFER_SIZE, 1);
    siface->queue    = xQueueCreate(qsize, sizeof(void*));
    siface->callback = callback;
    siface->context  = context;
}

/******************************************************************************/
void siface_rx_irq(struct siface* siface, const char* buf, size_t len) {
    BaseType_t woken = pdFALSE;

    xStreamBufferSendFromISR(siface->stream, buf, len, &woken);
    xEventGroupSetBitsFromISR(siface->events, EVENT_RX, &woken);
}

/******************************************************************************/
void siface_tx_irq(struct siface* siface) {
    BaseType_t woken = pdFALSE;

    xEventGroupSetBitsFromISR(siface->events, EVENT_TX, &woken);
}

/******************************************************************************/
void siface_wakeup(struct siface* siface) {
    xEventGroupSetBits(siface->events, EVENT_WAKEUP);
}

inline static void clear_buf(struct siface* siface) {
    xStreamBufferReset(siface->stream);
    siface->buflen = 0;
}

static size_t receive_buf(struct siface* siface) {
    size_t rec;

    rec = xStreamBufferReceive(siface->stream,
                               &siface->buf[siface->buflen],
                               SIFACE_BUFFER_SIZE - siface->buflen,
                               0);
    siface->buflen += rec;

    siface->buf[siface->buflen] = '\0';

    return rec;
}

static int command(struct siface* siface, int wait_time) {
    size_t received;
    int ret;

    received = receive_buf(siface);
    if (!received) {
        return -1;
    }

    ret = siface->callback(siface);
    if (!ret) {
        return ret;
    }

    /* Try again */
    osDelay(pdMS_TO_TICKS(wait_time));

    received = receive_buf(siface);
    if (!received) {
        return -1;
    }

    ret = siface->callback(siface);
    return ret;
}

/******************************************************************************/
void siface_task(struct siface* siface) {
    EventBits_t events;
    UBaseType_t num;
    char* string;

    for (;;) {
        events = xEventGroupWaitBits(siface->events,
                                     EVENT_RX | EVENT_WAKEUP,
                                     pdTRUE,
                                     pdFALSE,
                                     pdMS_TO_TICKS(1000));

        if (events & EVENT_RX) {
            command(siface, 100);
            clear_buf(siface);
        }

        if (events & EVENT_WAKEUP) {
            num = uxQueueMessagesWaiting(siface->queue);

            while (num) {
                num--;

                xQueueReceive(siface->queue, &string, portMAX_DELAY);
                //				HAL_UART_Transmit_DMA(siface->uart, (uint8_t *) string,
                //						strlen(string));
                CDC_Transmit_FS((uint8_t*)string, strlen(string));
                xEventGroupWaitBits(siface->events,
                                    EVENT_TX,
                                    pdTRUE,
                                    pdFALSE,
                                    pdMS_TO_TICKS(1000));
                vPortFree(string);
            }
        }
    }
}

/******************************************************************************/
int siface_add(struct siface* siface, const char* buf) {
    BaseType_t status;

    status = xQueueSendToBack(siface->queue, &buf, 0);
    if (status != pdTRUE) {
        return -1;
    }

    siface_wakeup(siface);
    return 0;
}
