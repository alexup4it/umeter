/*
 * Shared sensor state (actual values)
 */

#ifndef ACTUAL_H_
#define ACTUAL_H_

#include "cmsis_os.h"
#include "semphr.h"

#define ACTUAL_VOLTAGE_AVAIL     (1 << 0)
#define ACTUAL_HUMIDITY_AVAIL    (1 << 1)
#define ACTUAL_TEMPERATURE_AVAIL (1 << 2)
#define ACTUAL_PRESSURE_AVAIL    (1 << 3)
#define ACTUAL_WIND_DIR_AVAIL    (1 << 4)

struct actual {
    SemaphoreHandle_t mutex;

    uint32_t available;

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
