/*
 * Shared sensor state (actual values)
 */

#ifndef ACTUAL_H_
#define ACTUAL_H_

#include "cmsis_os.h"
#include "semphr.h"

struct actual {
    SemaphoreHandle_t mutex;

    int voltage;
    int32_t humidity;
    int32_t temperature;
    int32_t pressure;
    uint32_t wind_speed;
    int32_t wind_direction;

    /* Aggregated wind speed (written by task_anemometer, read by task_manager) */
    uint32_t wind_speed_avg;
    uint32_t wind_speed_min;
    uint32_t wind_speed_max;
};

void actual_init(struct actual* self);

#endif /* ACTUAL_H_ */
