/*
 * Serial interface
 *
 * Dmitry Proshutinsky <dproshutinsky@gmail.com>
 * 2024-2026
 */

#ifndef SIFACE_H_
#define SIFACE_H_

#include "cmsis_os.h"
#include "event_groups.h"
#include "queue.h"
#include "stm32f4xx_hal.h"
#include "stream_buffer.h"

#define SIFACE_BUFFER_SIZE      128
#define SIFACE_UART_BUFFER_SIZE 128

typedef int (*siface_cb)(void*);

struct siface {
    //	UART_HandleTypeDef *uart;

    EventGroupHandle_t events;
    StreamBufferHandle_t stream;
    xQueueHandle queue;

    char buf[SIFACE_BUFFER_SIZE + 1];
    size_t buflen;

    siface_cb callback;
    void* context;
};

void siface_init(struct siface* siface, /*UART_HandleTypeDef *uart,*/
                 size_t qsize,
                 siface_cb callback,
                 void* context);
void siface_wakeup(struct siface* siface);
void siface_rx_irq(struct siface* siface, const char* buf, size_t len);
void siface_tx_irq(struct siface* siface);
void siface_task(struct siface* siface);

/**
 * @brief: Add string to serial interface queue
 * @note: buf must be dynamically allocated with pvPortMalloc
 */
int siface_add(struct siface* siface, const char* buf);

#endif /* SIFACE_H_ */
