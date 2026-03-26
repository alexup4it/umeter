/*
 * W25Q Serial FLASH memory with mutex lock
 */

#ifndef W25Q_S_H_
#define W25Q_S_H_

#include "cmsis_os.h"
#include "semphr.h"
#include "w25q.h"

#define W25Q_S_MUTEX_TIMEOUT pdMS_TO_TICKS(500)

struct w25q_s {
    SemaphoreHandle_t mutex;
    struct w25q mem;
};

inline static void w25q_s_init(struct w25q_s* self,
                               SPI_HandleTypeDef* spi,
                               GPIO_TypeDef* cs_port,
                               uint16_t cs_pin,
                               void (*spi_init)(void),
                               void (*spi_deinit)(void)) {
    self->mutex = xSemaphoreCreateMutex();
    w25q_init(&self->mem, spi, cs_port, cs_pin, spi_init, spi_deinit);
}

inline static int w25q_s_sector_erase(struct w25q_s* self, uint32_t address) {
    if (xSemaphoreTake(self->mutex, W25Q_S_MUTEX_TIMEOUT) == pdFALSE) {
        return -1;
    }

    w25q_sector_erase(&self->mem, address);
    xSemaphoreGive(self->mutex);
    return 0;
}

inline static int w25q_s_read_data(struct w25q_s* self,
                                   uint32_t address,
                                   uint8_t* data,
                                   uint16_t size) {
    if (xSemaphoreTake(self->mutex, W25Q_S_MUTEX_TIMEOUT) == pdFALSE) {
        return -1;
    }

    w25q_read_data(&self->mem, address, data, size);
    xSemaphoreGive(self->mutex);
    return 0;
}

inline static int w25q_s_write_data(struct w25q_s* self,
                                    uint32_t address,
                                    uint8_t* data,
                                    uint16_t size) {
    if (xSemaphoreTake(self->mutex, W25Q_S_MUTEX_TIMEOUT) == pdFALSE) {
        return -1;
    }

    w25q_write_data(&self->mem, address, data, size);
    xSemaphoreGive(self->mutex);
    return 0;
}

inline static size_t w25q_s_get_capacity(struct w25q_s* self) {
    size_t cap;
    if (xSemaphoreTake(self->mutex, W25Q_S_MUTEX_TIMEOUT) == pdFALSE) {
        return 0;
    }

    cap = w25q_get_capacity(&self->mem);
    xSemaphoreGive(self->mutex);
    return cap;
}

inline static uint8_t w25q_s_get_manufacturer_id(struct w25q_s* self) {
    uint8_t mid;
    if (xSemaphoreTake(self->mutex, W25Q_S_MUTEX_TIMEOUT) == pdFALSE) {
        return 0;
    }

    mid = w25q_get_manufacturer_id(&self->mem);
    xSemaphoreGive(self->mutex);
    return mid;
}

#endif /* W25Q_S_H_ */
