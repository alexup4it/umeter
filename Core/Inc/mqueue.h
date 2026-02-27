/*
 * "mfifo" based FLASH storage queue
 *
 * Dmitry Proshutinsky <dproshutinsky@gmail.com>
 * 2025
 */

#ifndef MQUEUE_H_
#define MQUEUE_H_

#include "w25q_s.h"

#include "mfifo.h"

typedef struct
{
	struct mfifo mfifo;
} mqueue_t;


int mqueue_init(struct w25q_s *mem);
mqueue_t *mqueue_create(size_t secnum, size_t elsize);
int mqueue_set(mqueue_t *queue, const void *element);
int mqueue_get(mqueue_t *queue, void *element);
int mqueue_is_empty(mqueue_t *queue);

#endif /* MQUEUE_H_ */
