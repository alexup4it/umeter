/*
 * W25Q Serial FLASH memory
 *
 * Dmitry Proshutinsky <dproshutinsky@gmail.com>
 * 2024-2025
 */

#include "w25q.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define W25Q_CMD_WRITE_ENABLE       0x06
#define W25Q_CMD_RELEASE_POWER_DOWN 0xAB
#define W25Q_CMD_JEDEC_ID           0x9F
#define W25Q_CMD_FAST_READ          0x0B
#define W25Q_CMD_PAGE_PROGRAM       0x02
#define W25Q_CMD_SECTOR_ERASE       0x20
#define W25Q_CMD_BLOCK_ERASE        0xD8
#define W25Q_CMD_CHIP_ERASE         0xC7
#define W25Q_CMD_READ_STATUS_REG1   0x05
#define W25Q_CMD_WRITE_STATUS_REG1  0x01
#define W25Q_CMD_READ_STATUS_REG2   0x35
#define W25Q_CMD_WRITE_STATUS_REG2  0x31
#define W25Q_CMD_READ_STATUS_REG3   0x15
#define W25Q_CMD_WRITE_STATUS_REG3  0x11
#define W25Q_CMD_POWER_DOWN         0xB9

#define W25Q_BUSY_FLAG_MASK 0x01

#define BLOCK_ERASE_TIMEOUT 5000
#define CHIP_ERASE_TIMEOUT  (5 * 60 * 1000)
#define BUSY_TIMEOUT        1000
#define SPI_TIMEOUT         100

enum capacity {
    W25Q_CAPACITY_2    = 0x11,  // W25Q10 (2 * 64KB)
    W25Q_CAPACITY_4    = 0x12,  // W25Q20 (4 * 64KB)
    W25Q_CAPACITY_8    = 0x13,  // W25Q40 (8 * 64KB)
    W25Q_CAPACITY_16   = 0x14,  // W25Q80 (16 * 64KB)
    W25Q_CAPACITY_32   = 0x15,  // W25Q16 (32 * 64KB)
    W25Q_CAPACITY_64   = 0x16,  // W25Q32 (64 * 64KB)
    W25Q_CAPACITY_128  = 0x17,  // W25Q64 (128 * 64KB)
    W25Q_CAPACITY_256  = 0x18,  // W25Q128 (256 * 64KB)
    W25Q_CAPACITY_512  = 0x19,  // W25Q256 (512 * 64KB)
    W25Q_CAPACITY_1024 = 0x20   // W25Q512 (1024 * 64KB)
};

inline static void cs_low(struct w25q* mem) {
    HAL_GPIO_WritePin(mem->cs_port, mem->cs_pin, GPIO_PIN_RESET);
}

inline static void cs_high(struct w25q* mem) {
    HAL_GPIO_WritePin(mem->cs_port, mem->cs_pin, GPIO_PIN_SET);
}

inline static void delay(void) {
    for (uint8_t i = 0; i < 0xFF; i++) {
        asm("NOP");
    }
}

static uint32_t spi_read_id(struct w25q* mem) {
    uint8_t buffer[3];
    uint8_t header = W25Q_CMD_JEDEC_ID;
    uint32_t id;

    cs_low(mem);
    HAL_SPI_Transmit(mem->spi, &header, 1, SPI_TIMEOUT);
    HAL_SPI_Receive(mem->spi, buffer, 3, SPI_TIMEOUT);
    cs_high(mem);

    id = (buffer[0] << 16) | (buffer[1] << 8) | buffer[2];

    return id;
}

static uint8_t spi_read_status_reg(struct w25q* mem, uint8_t status_reg) {
    uint8_t buffer;
    uint8_t header = status_reg;

    cs_low(mem);
    HAL_SPI_Transmit(mem->spi, &header, 1, SPI_TIMEOUT);
    HAL_SPI_Receive(mem->spi, &buffer, 1, SPI_TIMEOUT);
    cs_high(mem);

    return buffer;
}

static void write_enable(struct w25q* mem) {
    uint8_t header = W25Q_CMD_WRITE_ENABLE;

    cs_low(mem);
    HAL_SPI_Transmit(mem->spi, &header, 1, SPI_TIMEOUT);
    cs_high(mem);
}

static void power_down(struct w25q* mem) {
    uint8_t header = W25Q_CMD_POWER_DOWN;

    cs_low(mem);
    HAL_SPI_Transmit(mem->spi, &header, 1, SPI_TIMEOUT);
    cs_high(mem);

    delay();
}

static void release_power_down(struct w25q* mem) {
    uint8_t header = W25Q_CMD_RELEASE_POWER_DOWN;

    cs_low(mem);
    HAL_SPI_Transmit(mem->spi, &header, 1, SPI_TIMEOUT);
    cs_high(mem);

    delay();
}

/******************************************************************************/
void w25q_init(struct w25q* mem,
               SPI_HandleTypeDef* spi,
               GPIO_TypeDef* cs_port,
               uint16_t cs_pin,
               w25q_hw_cb hw_init_cb,
               w25q_hw_cb hw_deinit_cb) {
    memset(mem, 0, sizeof(*mem));
    mem->spi          = spi;
    mem->cs_port      = cs_port;
    mem->cs_pin       = cs_pin;
    mem->hw_init_cb   = hw_init_cb;
    mem->hw_deinit_cb = hw_deinit_cb;
}

/******************************************************************************/
void w25q_power_down(struct w25q* mem) {
    if (mem->hw_init_cb) {
        mem->hw_init_cb();
    }
    release_power_down(mem);
}

/******************************************************************************/
void w25q_power_on(struct w25q* mem) {
    power_down(mem);
    if (mem->hw_deinit_cb) {
        mem->hw_deinit_cb();
    }
}

/******************************************************************************/
// 4KB
void w25q_sector_erase(struct w25q* mem, uint32_t address) {
    uint8_t header[4];
    uint32_t ms;

    header[0] = W25Q_CMD_SECTOR_ERASE;
    header[1] = address >> 16;
    header[2] = address >> 8;
    header[3] = address;

    write_enable(mem);

    cs_low(mem);
    HAL_SPI_Transmit(mem->spi, header, 4, SPI_TIMEOUT);
    cs_high(mem);

    delay();
    ms = HAL_GetTick();
    while (spi_read_status_reg(mem, W25Q_CMD_READ_STATUS_REG1) &
           W25Q_BUSY_FLAG_MASK) {
        if ((HAL_GetTick() - ms) > BUSY_TIMEOUT) {
            return;
        }
    }
}

/******************************************************************************/
// 64KB
void w25q_block_erase(struct w25q* mem, uint32_t address) {
    uint8_t header[4];
    uint32_t ms;

    header[0] = W25Q_CMD_BLOCK_ERASE;
    header[1] = address >> 16;
    header[2] = address >> 8;
    header[3] = address;

    write_enable(mem);

    cs_low(mem);
    HAL_SPI_Transmit(mem->spi, header, 4, SPI_TIMEOUT);
    cs_high(mem);

    delay();
    ms = HAL_GetTick();
    while (spi_read_status_reg(mem, W25Q_CMD_READ_STATUS_REG1) &
           W25Q_BUSY_FLAG_MASK) {
        if ((HAL_GetTick() - ms) > BLOCK_ERASE_TIMEOUT) {
            return;
        }
    }
}

/******************************************************************************/
void w25q_chip_erase(struct w25q* mem) {
    uint8_t header = W25Q_CMD_CHIP_ERASE;
    uint32_t ms;

    write_enable(mem);

    cs_low(mem);
    HAL_SPI_Transmit(mem->spi, &header, 1, SPI_TIMEOUT);
    cs_high(mem);

    delay();
    ms = HAL_GetTick();
    while (spi_read_status_reg(mem, W25Q_CMD_READ_STATUS_REG1) &
           W25Q_BUSY_FLAG_MASK) {
        if ((HAL_GetTick() - ms) > CHIP_ERASE_TIMEOUT) {
            return;
        }
    }
}

/******************************************************************************/
void w25q_read_data(struct w25q* mem,
                    uint32_t address,
                    uint8_t* data,
                    uint16_t size) {
    uint8_t header[5];
    header[0] = W25Q_CMD_FAST_READ;
    header[1] = address >> 16;
    header[2] = address >> 8;
    header[3] = address;
    header[4] = 0x00;

    cs_low(mem);
    HAL_SPI_Transmit(mem->spi, header, 5, SPI_TIMEOUT);
    HAL_SPI_Receive(mem->spi, data, size, SPI_TIMEOUT);
    cs_high(mem);
}

/******************************************************************************/
// Up to W25Q_PAGE_SIZE bytes
void w25q_write_data(struct w25q* mem,
                     uint32_t address,
                     uint8_t* data,
                     uint16_t size) {
    uint8_t header[4];
    uint32_t ms;

    header[0] = W25Q_CMD_PAGE_PROGRAM;
    header[1] = address >> 16;
    header[2] = address >> 8;
    header[3] = address;

    write_enable(mem);

    cs_low(mem);
    HAL_SPI_Transmit(mem->spi, header, 4, SPI_TIMEOUT);
    HAL_SPI_Transmit(mem->spi, data, size, SPI_TIMEOUT);
    cs_high(mem);

    delay();
    ms = HAL_GetTick();
    while (spi_read_status_reg(mem, W25Q_CMD_READ_STATUS_REG1) &
           W25Q_BUSY_FLAG_MASK) {
        if ((HAL_GetTick() - ms) > BUSY_TIMEOUT) {
            return;
        }
    }
}

/******************************************************************************/
size_t w25q_get_capacity(struct w25q* mem) {
    uint32_t capacity = spi_read_id(mem) & 0xFF;

    switch ((enum capacity)capacity) {
        case W25Q_CAPACITY_2:
            return 2 * 64 * 1024;
        case W25Q_CAPACITY_4:
            return 4 * 64 * 1024;
        case W25Q_CAPACITY_8:
            return 8 * 64 * 1024;
        case W25Q_CAPACITY_16:
            return 16 * 64 * 1024;
        case W25Q_CAPACITY_32:
            return 32 * 64 * 1024;
        case W25Q_CAPACITY_64:
            return 64 * 64 * 1024;
        case W25Q_CAPACITY_128:
            return 128 * 64 * 1024;
        case W25Q_CAPACITY_256:
            return 256 * 64 * 1024;
        case W25Q_CAPACITY_512:
            return 256 * 64 * 1024;  // NOTE: 16 MB only
        case W25Q_CAPACITY_1024:
            return 256 * 64 * 1024;  // NOTE: 16 MB only
        default:
            return 0;
    }
}

/******************************************************************************/
uint8_t w25q_get_manufacturer_id(struct w25q* mem) {
    return spi_read_id(mem) >> 16;
}
