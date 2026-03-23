/*
 * Shared sensor state (actual values)
 */

#ifndef ACTUAL_H_
#define ACTUAL_H_

#include "cmsis_os.h"
#include "semphr.h"

struct actual {
    SemaphoreHandle_t mutex;

    int avail;

    int voltage;
    uint32_t wind_speed;
    int32_t wind_direction;
    int32_t humidity;
    int32_t temperature;
};

void actual_init(struct actual* self);

#endif /* ACTUAL_H_ */
