/*
 * FLASH memory FIFO implementation
 *
 * Dmitry Proshutinsky <sodaspace@yandex.ru>
 * 2025
 */

#include <stddef.h>
#include <stdint.h>
#include <w25q_s.h>

#include "cmsis_os.h"
#include "semphr.h"

/*
 * @brief return status values
 */
enum mfifo_status {
    MFIFO_OK = 0,
    MFIFO_ERR_NOT_SUPPORTED,
    MFIFO_ERR_INVALID_ARGUMENT,
    MFIFO_ERR_INVALID_CRC,
    MFIFO_ERR_NO_FREE_SPACE,
    MFIFO_ERR_EMPTY,
    MFIFO_ERR_ALLOC,
    MFIFO_ERR_LOCK,
    MFIFO_ERR_IO
};

/*
 * @brief: mfifo handle
 */
struct mfifo {
    SemaphoreHandle_t mutex;
    struct w25q_s* mem;

    size_t pagesize;
    size_t secsize;
    size_t esize;
    uint8_t* buf;

    uint32_t addr;
    size_t secnum;

    uint32_t set;
    uint32_t get;
};

/*
 * @brief: mfifo handle initialization
 * @param mfifo: mfifo handle
 * @param mem: w25q_s handle
 * @param pagesize: hardware storage page size
 * @param secsize: hardware storage sector size
 * @param esize: stored element size
 * @param addr: physical address to start from
 * @param secnum: number of sectors to use
 * @retval: 0 on success, negative error value on failure (enum mfifo_status)
 */
int mfifo_init(struct mfifo* mfifo,
               struct w25q_s* mem,
               size_t pagesize,
               size_t secsize,
               size_t esize,
               uint32_t addr,
               size_t secnum);

/*
 * @brief: recover fifo state after startup: find "set" and "get" addresses
 * @param mfifo: mfifo handle
 * @retval: 0 on success, negative error value on failure (enum mfifo_status)
 */
int mfifo_recover(struct mfifo* mfifo);

/*
 * @brief: check is mfifio empty or not
 * @param mfifo: mfifo handle
 * @retval: 1 - empty, 0 - not empty, negative error value on failure
 *     (enum mfifo_status)
 */
int mfifo_is_empty(struct mfifo* mfifo);

/*
 * @brief: store a new element in fifo
 * @param mfifo: mfifo handle
 * @param element: new element to store, size of element must be equal to
 *     MFIFO_ELEMENT_SIZE
 * @retval: 0 on success, negative error value on failure (enum mfifo_status)
 */
int mfifo_set(struct mfifo* mfifo, const void* element);

/*
 * @brief: get stored element
 * @param mfifo: mfifo handle
 * @param element: buffer to store element, size of buffer must be equal to
 *     MFIFO_ELEMENT_SIZE
 * @retval: 0 on success, negative error value on failure (enum mfifo_status)
 */
int mfifo_get(struct mfifo* mfifo, void* element);
