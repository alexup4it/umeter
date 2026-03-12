/*
 * Application tasks — shared definitions
 */

#ifndef UMETER_TASKS_H_
#define UMETER_TASKS_H_

#include "FreeRTOS.h"
#include "event_groups.h"
#include "mqueue.h"

extern EventGroupHandle_t task_events;

/* Scheduler → task trigger bits */
#define TASK_EVENT_ANEMOMETER_START (1 << 0)
#define TASK_EVENT_SENSORS_START    (1 << 1)
#define TASK_EVENT_NET_START        (1 << 2)
#define TASK_EVENT_WATCHDOG_START   (1 << 3)

/* Task → scheduler completion bits */
#define TASK_EVENT_ANEMOMETER_DONE (1 << 4)
#define TASK_EVENT_SENSORS_DONE    (1 << 5)

/**
 * Create event group. Call before scheduler starts.
 */
void task_manager_init(void);

/**
 * Run the scheduling loop (never returns).
 */
void task_manager_run(void);

struct sensor_record {
    uint32_t timestamp;
    int32_t voltage;
    int32_t temperature;
    int32_t humidity;
    int32_t angle;
    uint32_t count_avg;
    uint32_t count_min;
    uint32_t count_max;
};

/* --- Utility functions (callable from any context) --- */

void led_blink(uint8_t count);

/* --- ISR forwarders (called from main.c HAL callbacks) --- */

void sensors_notify(void);

/* --- Queue accessor (sensors → net) --- */

mqueue_t* sensors_queue(void);

/*---------------------------------------------------------------------------*/
/* Task context structures                                                   */
/*---------------------------------------------------------------------------*/

/* Forward declarations */
struct as5600;
struct aht20;
struct avoltage;
struct counter;
struct button;
struct siface;
struct sim800l;
struct w25q_s;
struct logger;

typedef void (*pm_fn)(void);

struct task_default_ctx {
    int _unused;
};

struct task_blink_ctx {
    int _unused;
};

struct task_button_ctx {
    struct button* btn;
};

struct task_watchdog_ctx {
    int _unused;
};

struct task_anemometer_ctx {
    struct counter* cnt;
    pm_fn anemometer_on;
    pm_fn anemometer_off;
};

struct task_sensors_ctx {
    struct as5600* pot;
    struct aht20* aht;
    struct avoltage* avlt;
    struct counter* cnt;
    struct logger* logger;
    pm_fn as5600_on;
    pm_fn as5600_off;
    pm_fn aht20_on;
    pm_fn aht20_off;
    pm_fn avoltage_on;
    pm_fn avoltage_off;
};

struct task_modem_ctx {
    struct sim800l* mod;
};

struct task_serial_iface_ctx {
    struct siface* siface;
};

struct task_logging_ctx {
    struct logger* logger;
};

struct task_net_ctx {
    struct sim800l* mod;
    struct logger* logger;
};

struct task_ota_ctx {
    struct sim800l* mod;
    struct w25q_s* mem;
    struct logger* logger;
};

#endif /* UMETER_TASKS_H_ */
