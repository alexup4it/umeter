/*
 * FLASH memory FIFO implementation
 *
 * Dmitry Proshutinsky <sodaspace@yandex.ru>
 * 2025
 */

#include "mfifo.h"

#include <string.h>

#define CRC6_INIT 0x00
#define CRC6_POLY 0x30

#define MUTEX_TIMEOUT pdMS_TO_TICKS(100)

#define sector_addr(addr) ((addr / MFIFO_SECTOR_SIZE) * MFIFO_SECTOR_SIZE)

#define phys_element_size(m) (m->esize + sizeof(struct header))

struct header {
    uint8_t wr : 1;
    uint8_t rd : 1;
    uint8_t cs : 6;
};

/******************************************************************************/
int mfifo_init(struct mfifo* mfifo,
               struct w25q_s* mem,
               size_t pagesize,
               size_t secsize,
               size_t esize,
               uint32_t addr,
               size_t secnum) {
    if (!mfifo || !mem || !secsize || !esize || !secnum) {
        return -MFIFO_ERR_INVALID_ARGUMENT;
    }

    if (addr % secsize) {
        return -MFIFO_ERR_INVALID_ARGUMENT;
    }

    if (secsize % pagesize) {
        return -MFIFO_ERR_INVALID_ARGUMENT;
    }

    memset(mfifo, 0, sizeof(*mfifo));
    mfifo->mem      = mem;
    mfifo->pagesize = pagesize;
    mfifo->secsize  = secsize;
    mfifo->esize    = esize;
    mfifo->addr     = addr;
    mfifo->secnum   = secnum;

    mfifo->set = 0;
    mfifo->get = 0;

    mfifo->mutex = xSemaphoreCreateMutex();
    if (!mfifo->mutex) {
        return -MFIFO_ERR_ALLOC;
    }

    mfifo->buf = pvPortMalloc(phys_element_size(mfifo));
    if (!mfifo->buf) {
        vSemaphoreDelete(mfifo->mutex);
        return -MFIFO_ERR_ALLOC;
    }

    return 0;
}

inline static int phys_read(struct mfifo* mfifo,
                            uint32_t addr,
                            uint8_t* data,
                            size_t len) {
    w25q_s_read_data(mfifo->mem, mfifo->addr + addr, data, len);
    return 0;
}

inline static int phys_write(struct mfifo* mfifo,
                             uint32_t addr,
                             const uint8_t* data,
                             size_t len) {
    w25q_s_write_data(mfifo->mem, mfifo->addr + addr, (uint8_t*)data, len);
    return 0;
}

inline static int phys_erase(struct mfifo* mfifo, uint32_t addr) {
    w25q_s_sector_erase(mfifo->mem, mfifo->addr + addr);
    return 0;
}

static uint32_t next_addr(struct mfifo* mfifo, uint32_t addr) {
    uint32_t next;

    // Next address in current sector
    next = addr + phys_element_size(mfifo);

    // Does element cross the page border?
    if ((next % mfifo->pagesize + phys_element_size(mfifo)) > mfifo->pagesize) {
        next = (next / mfifo->pagesize + 1) * mfifo->pagesize;
    }

    // Does the address exceed area?
    if (next / mfifo->secsize == mfifo->secnum) {
        next = 0;
    }

    return next;
}

/******************************************************************************/
int mfifo_recover(struct mfifo* mfifo) {
    return -MFIFO_ERR_NOT_SUPPORTED;  // TODO
}

inline static int is_empty(struct mfifo* mfifo) {
    if (mfifo->set == mfifo->get) {
        return 1;
    }
    return 0;
}

/******************************************************************************/
int mfifo_is_empty(struct mfifo* mfifo) {
    int ret;

    if (!mfifo) {
        return -MFIFO_ERR_INVALID_ARGUMENT;
    }

    if (xSemaphoreTake(mfifo->mutex, MUTEX_TIMEOUT) == pdFALSE) {
        return -MFIFO_ERR_LOCK;
    }

    ret = is_empty(mfifo);
    xSemaphoreGive(mfifo->mutex);
    return ret;
}

static uint8_t crc6(const uint8_t* data, size_t len) {
    uint8_t crc = CRC6_INIT;

    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if ((crc & 1) == 0) {
                crc >>= 1;
            } else {
                crc = (crc >> 1) ^ CRC6_POLY;
            }
        }
    }

    return crc & 0x3F;
}

inline static int is_diff_sectors(struct mfifo* mfifo,
                                  uint32_t addr,
                                  uint32_t addr_next) {
    if ((addr / mfifo->secsize) != (addr_next / mfifo->secsize)) {
        return 1;
    }
    return 0;
}

inline static int rectag_set(struct mfifo* mfifo) {
    return -1;  // TODO
}

inline static int rectag_get(struct mfifo* mfifo) {
    return -1;  // TODO
}

inline static int element_set(struct mfifo* mfifo, const void* element) {
    uint8_t* payload = &mfifo->buf[sizeof(struct header)];
    struct header header;
    uint32_t next;
    int ret;

    // Get next "set"
    next = next_addr(mfifo, mfifo->set);
    if (next == mfifo->get) {
        return -MFIFO_ERR_NO_FREE_SPACE;
    }

    // Sector beginning
    if ((mfifo->set % mfifo->secsize) == 0) {
        // mfifo is not empty
        // "set" and "get" in the same sector
        if ((mfifo->set != mfifo->get) &&
            ((mfifo->set / mfifo->secsize) == (mfifo->get / mfifo->secsize))) {
            return -MFIFO_ERR_NO_FREE_SPACE;
        }
        // Erase
        else {
            ret = phys_erase(mfifo, mfifo->set);
            if (ret < 0) {
                return -MFIFO_ERR_IO;  // Can not erase sector
            }
        }
    }

    // Copy payload to buffer
    memcpy(payload, element, mfifo->esize);

    // Form header
    header.wr = 0;
    header.rd = 1;
    header.cs = crc6(payload, mfifo->esize);

    // Copy header to buffer
    memcpy(mfifo->buf, &header, sizeof(struct header));

    // Write element
    ret = phys_write(mfifo, mfifo->set, mfifo->buf, phys_element_size(mfifo));
    if (ret < 0) {
        return -MFIFO_ERR_IO;  // Can not write
    }

    // Recovery
    if (is_diff_sectors(mfifo, mfifo->set, next)) {
        rectag_set(mfifo);
    }

    // Update "set"
    mfifo->set = next;

    return 0;
}

/******************************************************************************/
int mfifo_set(struct mfifo* mfifo, const void* element) {
    int ret;

    if (!mfifo || !element) {
        return -MFIFO_ERR_INVALID_ARGUMENT;
    }

    if (xSemaphoreTake(mfifo->mutex, MUTEX_TIMEOUT) == pdFALSE) {
        return -MFIFO_ERR_LOCK;
    }

    ret = element_set(mfifo, element);
    xSemaphoreGive(mfifo->mutex);
    return ret;
}

inline static int element_get(struct mfifo* mfifo, void* element) {
    uint8_t* payload = &mfifo->buf[sizeof(struct header)];
    struct header header;
    uint32_t next;
    int ret;

    if (is_empty(mfifo)) {
        return -MFIFO_ERR_EMPTY;
    }

    // Read element
    ret = phys_read(mfifo, mfifo->get, mfifo->buf, phys_element_size(mfifo));
    if (ret < 0) {
        return -MFIFO_ERR_IO;  // Can not read
    }

    // Copy header from buffer
    memcpy(&header, mfifo->buf, sizeof(struct header));

    // Update header
    header.rd = 0;

    // Write header (for recovery)
    ret = phys_write(mfifo, mfifo->get, (uint8_t*)&header, sizeof(header));
    if (ret < 0) {
        return -MFIFO_ERR_IO;  // Can not write
    }

    // Get next "get"
    next = next_addr(mfifo, mfifo->get);

    // Recovery
    if (is_diff_sectors(mfifo, mfifo->get, next)) {
        rectag_get(mfifo);
    }

    // Update "get"
    mfifo->get = next;

    // Checksum
    if (header.cs != crc6(payload, mfifo->esize)) {
        return -MFIFO_ERR_INVALID_CRC;  // Invalid CRC
    }

    // Copy payload from buffer
    memcpy(element, payload, mfifo->esize);

    return 0;
}

/******************************************************************************/
int mfifo_get(struct mfifo* mfifo, void* element) {
    int ret;

    if (!mfifo || !element) {
        return -MFIFO_ERR_INVALID_ARGUMENT;
    }

    if (xSemaphoreTake(mfifo->mutex, MUTEX_TIMEOUT) == pdFALSE) {
        return -MFIFO_ERR_LOCK;
    }

    ret = element_get(mfifo, element);
    xSemaphoreGive(mfifo->mutex);
    return ret;
}
