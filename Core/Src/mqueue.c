/*
 * "mfifo" based FLASH storage queue
 *
 * Dmitry Proshutinsky <dproshutinsky@gmail.com>
 * 2025
 */

#include "mqueue.h"

#include <string.h>

#include "cmsis_os.h"
#include "semphr.h"

#include "fws.h"

extern const uint32_t *_app_len;
#define APP_LENGTH ((uint32_t) &_app_len)

#define APP_END_ADDR (FWS_PAYLOAD_ADDR + APP_LENGTH)
#define FIFO_SECTOR ((APP_END_ADDR + (W25Q_SECTOR_SIZE - 1)) / W25Q_SECTOR_SIZE)
#define FIFO_ADDR (FIFO_SECTOR * W25Q_SECTOR_SIZE)

static struct
{
	SemaphoreHandle_t mutex;
	struct w25q_s *mem;
	size_t address;
	size_t msize;
} mqueue;


/******************************************************************************/
int mqueue_init(struct w25q_s *mem)
{
	memset(&mqueue, 0, sizeof(mqueue));
	mqueue.mem = mem;
	mqueue.mutex = xSemaphoreCreateMutex();

	if (w25q_s_get_manufacturer_id(mqueue.mem) != FWS_WINBOND_MANUFACTURER_ID)
	{
		mqueue.msize = 0;
		return -1;
	}

	mqueue.address = FIFO_ADDR;
	mqueue.msize = w25q_s_get_capacity(mqueue.mem);

	return 0;
}

inline static mqueue_t *create(size_t secnum, size_t elsize)
{
	size_t len = secnum * W25Q_SECTOR_SIZE;
	mqueue_t *q;
	int ret;

	if ((mqueue.address + len) > mqueue.msize)
		return NULL;

	q = pvPortMalloc(sizeof(mqueue_t));
	if (!q)
		return NULL;

	ret = mfifo_init(&q->mfifo, mqueue.mem, W25Q_PAGE_SIZE, W25Q_SECTOR_SIZE,
			elsize, mqueue.address, secnum);
	if (ret)
	{
		vPortFree(q);
		return NULL;
	}

	mqueue.address += len;
	return q;
}

/******************************************************************************/
mqueue_t *mqueue_create(size_t secnum, size_t elsize)
{
	mqueue_t *q;

	xSemaphoreTake(mqueue.mutex, portMAX_DELAY);
	q = create(secnum, elsize);
	xSemaphoreGive(mqueue.mutex);

	return q;
}

/******************************************************************************/
int mqueue_set(mqueue_t *queue, const void *element)
{
	if (!queue)
		return -MFIFO_ERR_INVALID_ARGUMENT;
	return mfifo_set(&queue->mfifo, element);
}

/******************************************************************************/
int mqueue_get(mqueue_t *queue, void *element)
{
	if (!queue)
		return -MFIFO_ERR_INVALID_ARGUMENT;
	return mfifo_get(&queue->mfifo, element);
}

/******************************************************************************/
int mqueue_is_empty(mqueue_t *queue)
{
	if (!queue)
		return -MFIFO_ERR_INVALID_ARGUMENT;
	return mfifo_is_empty(&queue->mfifo);
}
