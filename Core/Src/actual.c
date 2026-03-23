/*
 * Shared sensor state (actual values)
 */

#include "actual.h"

#include <string.h>

void actual_init(struct actual* self) {
    memset(self, 0, sizeof(*self));
    self->mutex = xSemaphoreCreateMutex();
}
