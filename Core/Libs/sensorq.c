/*
 * RAM-based sensor record queue (LIFO pop, eviction on overflow)
 */

#include "sensorq.h"

#include <string.h>

#include "ptasks.h" /* struct sensor_record */

#define MUTEX_TIMEOUT pdMS_TO_TICKS(100)

/******************************************************************************/
int sensorq_init(struct sensorq* self,
                 struct sensor_record* buf,
                 size_t capacity) {
    if (!self || !buf || !capacity) {
        return -1;
    }

    memset(self, 0, sizeof(*self));
    self->buf      = buf;
    self->capacity = capacity;
    self->mutex    = xSemaphoreCreateMutex();
    if (!self->mutex) {
        return -1;
    }

    return 0;
}

/******************************************************************************/
void sensorq_push(struct sensorq* self, const struct sensor_record* rec) {
    if (!self || !rec) {
        return;
    }
    if (xSemaphoreTake(self->mutex, MUTEX_TIMEOUT) == pdFALSE) {
        return;
    }

    self->buf[self->head] = *rec;
    self->head            = (self->head + 1) % self->capacity;

    if (self->count < self->capacity) {
        self->count++;
    }

    xSemaphoreGive(self->mutex);
}

/******************************************************************************/
int sensorq_pop(struct sensorq* self, struct sensor_record* rec) {
    if (!self || !rec) {
        return -1;
    }
    if (xSemaphoreTake(self->mutex, MUTEX_TIMEOUT) == pdFALSE) {
        return -1;
    }

    if (self->count == 0) {
        xSemaphoreGive(self->mutex);
        return -1;
    }

    /* LIFO: return the most recently pushed element */
    self->head = (self->head + self->capacity - 1) % self->capacity;
    *rec       = self->buf[self->head];
    self->count--;

    xSemaphoreGive(self->mutex);
    return 0;
}

/******************************************************************************/
int sensorq_is_empty(struct sensorq* self) {
    if (!self) {
        return -1;
    }
    if (xSemaphoreTake(self->mutex, MUTEX_TIMEOUT) == pdFALSE) {
        return -1;
    }

    int empty = (self->count == 0);
    xSemaphoreGive(self->mutex);
    return empty;
}

/******************************************************************************/
int sensorq_count(struct sensorq* self) {
    if (!self) {
        return -1;
    }
    if (xSemaphoreTake(self->mutex, MUTEX_TIMEOUT) == pdFALSE) {
        return -1;
    }

    int n = (int)self->count;
    xSemaphoreGive(self->mutex);
    return n;
}

/******************************************************************************/
int sensorq_peek(struct sensorq* self,
                 struct sensor_record* out,
                 int max_count) {
    if (!self || !out || max_count <= 0) {
        return -1;
    }
    if (xSemaphoreTake(self->mutex, MUTEX_TIMEOUT) == pdFALSE) {
        return -1;
    }

    int n = (int)self->count;
    if (n > max_count) {
        n = max_count;
    }

    /* Read newest-first (LIFO) without modifying head/count */
    size_t idx = self->head;
    for (int i = 0; i < n; i++) {
        idx    = (idx + self->capacity - 1) % self->capacity;
        out[i] = self->buf[idx];
    }

    xSemaphoreGive(self->mutex);
    return n;
}

/******************************************************************************/
void sensorq_drop(struct sensorq* self, int count) {
    if (!self || count <= 0) {
        return;
    }
    if (xSemaphoreTake(self->mutex, MUTEX_TIMEOUT) == pdFALSE) {
        return;
    }

    if ((size_t)count > self->count) {
        count = (int)self->count;
    }

    /* Remove newest elements (move head backwards) */
    self->head = (self->head + self->capacity - (size_t)count) % self->capacity;
    self->count -= (size_t)count;

    xSemaphoreGive(self->mutex);
}
