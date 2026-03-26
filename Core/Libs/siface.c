/*
 * Serial interface
 */

#include "siface.h"

#include <string.h>

#include "usbd_cdc_if.h"

#define EVENT_TX     (1 << 0)
#define EVENT_RX     (1 << 1)
#define EVENT_WAKEUP (1 << 2)

/******************************************************************************/
void siface_init(struct siface* self, /*UART_HandleTypeDef *uart,*/
                 size_t qsize,
                 siface_cb callback,
                 void* context) {
    memset(self, 0, sizeof(*self));
    //	self->uart = uart;
    self->events   = xEventGroupCreate();
    self->stream   = xStreamBufferCreate(SIFACE_BUFFER_SIZE, 1);
    self->queue    = xQueueCreate(qsize, sizeof(void*));
    self->callback = callback;
    self->context  = context;
}

/******************************************************************************/
void siface_rx_irq(struct siface* self, const char* buf, size_t len) {
    BaseType_t woken = pdFALSE;

    xStreamBufferSendFromISR(self->stream, buf, len, &woken);
    xEventGroupSetBitsFromISR(self->events, EVENT_RX, &woken);
}

/******************************************************************************/
void siface_tx_irq(struct siface* self) {
    BaseType_t woken = pdFALSE;

    xEventGroupSetBitsFromISR(self->events, EVENT_TX, &woken);
}

/******************************************************************************/
void siface_wakeup(struct siface* self) {
    xEventGroupSetBits(self->events, EVENT_WAKEUP);
}

inline static void clear_buf(struct siface* self) {
    xStreamBufferReset(self->stream);
    self->buflen = 0;
}

static size_t receive_buf(struct siface* self) {
    size_t rec;

    rec = xStreamBufferReceive(self->stream,
                               &self->buf[self->buflen],
                               SIFACE_BUFFER_SIZE - self->buflen,
                               0);
    self->buflen += rec;

    self->buf[self->buflen] = '\0';

    return rec;
}

static int buf_has_json(struct siface* self) {
    for (size_t i = 0; i < self->buflen; i++) {
        if (self->buf[i] == '}') {
            return 1;
        }
    }
    return 0;
}

static int command(struct siface* self, int timeout_ms) {
    TickType_t deadline;
    size_t received;

    deadline = xTaskGetTickCount() + pdMS_TO_TICKS(timeout_ms);

    /* Accumulate data until we see a complete JSON object or timeout */
    for (;;) {
        received = receive_buf(self);

        if (buf_has_json(self)) {
            return self->callback(self);
        }

        if (!received && (int32_t)(xTaskGetTickCount() - deadline) >= 0) {
            return -1;
        }

        /* Yield briefly to let more data arrive */
        osDelay(pdMS_TO_TICKS(1));
    }
}

static void drain_queue(struct siface* self) {
    UBaseType_t num;
    char* string;

    num = uxQueueMessagesWaiting(self->queue);

    while (num) {
        num--;

        xQueueReceive(self->queue, &string, portMAX_DELAY);
        CDC_Transmit_FS((uint8_t*)string, strlen(string));
        xEventGroupWaitBits(self->events,
                            EVENT_TX,
                            pdTRUE,
                            pdFALSE,
                            pdMS_TO_TICKS(1000));
        vPortFree(string);
    }
}

/******************************************************************************/
void siface_task(struct siface* self) {
    EventBits_t events;

    for (;;) {
        events = xEventGroupWaitBits(self->events,
                                     EVENT_RX | EVENT_WAKEUP,
                                     pdTRUE,
                                     pdFALSE,
                                     pdMS_TO_TICKS(1000));

        if (events & EVENT_RX) {
            command(self, 100);
            clear_buf(self);
        }

        drain_queue(self);
    }
}

/******************************************************************************/
int siface_add(struct siface* self, const char* buf) {
    BaseType_t status;

    status = xQueueSendToBack(self->queue, &buf, 0);
    if (status != pdTRUE) {
        return -1;
    }

    siface_wakeup(self);
    return 0;
}
