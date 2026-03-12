/*
 * Shared sensor state (actual values)
 */

#include "actual.h"

#include <string.h>

struct actual actual;

void actual_init(void) {
    memset(&actual, 0, sizeof(actual));
    actual.mutex = xSemaphoreCreateMutex();
}
