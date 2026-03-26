/*
 * RAM-based sensor record queue (LIFO pop, eviction on overflow)
 *
 * - push: appends newest record; if full, overwrites the oldest
 * - pop:  returns the newest record (LIFO order)
 *
 */

#ifndef SENSORQ_H_
#define SENSORQ_H_

#include <stddef.h>

#include "cmsis_os.h"
#include "semphr.h"

struct sensor_record;

struct sensorq {
    SemaphoreHandle_t mutex;
    struct sensor_record* buf;
    size_t capacity;
    size_t head;  /* next write position (ring index) */
    size_t count; /* current number of elements        */
};

/*
 * Initialise queue with a caller-provided static buffer.
 * Returns 0 on success, -1 on error.
 */
int sensorq_init(struct sensorq* self,
                 struct sensor_record* buf,
                 size_t capacity);

/*
 * Push a new record. If the queue is full the oldest record is evicted.
 */
void sensorq_push(struct sensorq* self, const struct sensor_record* rec);

/*
 * Pop the newest record (LIFO).
 * Returns 0 on success, -1 if empty or lock timeout.
 */
int sensorq_pop(struct sensorq* self, struct sensor_record* rec);

/*
 * Returns 1 if empty, 0 if not empty, -1 on lock timeout.
 */
int sensorq_is_empty(struct sensorq* self);

/*
 * Returns current number of elements, or -1 on lock timeout.
 */
int sensorq_count(struct sensorq* self);

/*
 * Copy up to `max_count` newest records into `out` (LIFO order)
 * without removing them from the queue.
 * Returns the number of records copied, or -1 on lock timeout.
 */
int sensorq_peek(struct sensorq* self,
                 struct sensor_record* out,
                 int max_count);

/*
 * Remove `count` newest records from the queue.
 * Call after a successful send to confirm consumption.
 */
void sensorq_drop(struct sensorq* self, int count);

#endif /* SENSORQ_H_ */
