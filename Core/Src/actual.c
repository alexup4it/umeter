/*
 * Shared sensor state (actual values)
 */

#include "actual.h"

#include <string.h>

void actual_init(struct actual* a) {
    memset(a, 0, sizeof(*a));
    a->mutex = xSemaphoreCreateMutex();
}
