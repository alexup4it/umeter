/*
 * Application tasks — shared definitions
 */

#ifndef UMETER_TASKS_H_
#define UMETER_TASKS_H_

#include <stdbool.h>

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "event_groups.h"
#include "ptasks.h"
#include "task.h"

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

struct sensorq;

struct __attribute__((packed)) sensor_record {
    uint32_t timestamp;
    uint16_t voltage;        /* millivolts              */
    int16_t temperature;     /* centidegrees C (0.01°C) */
    uint16_t humidity;       /* centipercent RH (0.01%) */
    uint16_t wind_direction; /* centidegrees (0.01°)    */
    uint16_t wind_speed_avg;
    uint16_t wind_speed_min;
    uint16_t wind_speed_max;
};

_Static_assert(sizeof(struct sensor_record) == 18,
               "sensor_record must be 18 bytes (binary wire format)");

/* --- Utility functions (callable from any context) --- */

void led_blink(uint8_t count);

/* --- ISR forwarders (called from main.c HAL callbacks) --- */

void task_button_irq_notify_from_isr(void);

/* --- Queue accessor (sensors → net) --- */

#define SENSORS_QUEUE_CAPACITY 100

/*---------------------------------------------------------------------------*/
/* Task context structures                                                   */
/*---------------------------------------------------------------------------*/

/* Forward declarations */
struct as5600;
struct aht20;
struct avoltage;
struct freqmeter;
struct button;
struct siface;
struct sim800l;
struct w25q_s;
struct logger;
struct actual;

typedef void (*pm_fn)(void);

struct task_default_ctx {
    struct logger* logger;
    struct sensorq* sensorq;
};

struct task_blink_ctx {
    int _unused;
};

struct task_button_ctx {
    struct actual* actual;
    struct button* btn;
};

struct task_watchdog_ctx {
    int _unused;
};

struct task_anemometer_ctx {
    struct actual* actual;
    struct freqmeter* cnt;
    pm_fn anemometer_on;
    pm_fn anemometer_off;
};

struct task_sensors_ctx {
    struct actual* actual;
    struct sensorq* queue;
    struct as5600* as5600;
    struct aht20* aht20;
    struct avoltage* voltage;
    struct freqmeter* cnt;
    struct logger* logger;
    pm_fn as5600_on;
    pm_fn as5600_off;
    pm_fn aht20_on;
    pm_fn aht20_off;
};

struct task_modem_ctx {
    struct actual* actual;
    struct sim800l* modem;
    struct avoltage* voltage;
    struct logger* logger;
    pm_fn power_on;
    pm_fn power_off;
};

struct task_serial_iface_ctx {
    struct siface* siface;
};

struct task_logging_ctx {
    struct logger* logger;
};

struct task_net_ctx {
    struct actual* actual;
    struct sensorq* queue;
    struct logger* logger;
};

struct task_ota_ctx {
    struct w25q_s* mem;
    struct logger* logger;
};

/**
 * Run the scheduling loop (never returns).
 */
void task_manager_run(struct task_default_ctx* ctx);

/*---------------------------------------------------------------------------*/
/* Modem request interface (task_modem ↔ task_net / task_ota)                */
/*---------------------------------------------------------------------------*/

struct sim800l_http_response;
struct sim800l_netscan_result;

enum modem_request_type {
    MODEM_REQ_HTTP_GET,
    MODEM_REQ_HTTP_POST,
    MODEM_REQ_HTTP_POST_BIN,
    MODEM_REQ_NETSCAN,
};

struct modem_request {
    enum modem_request_type type;

    /* HTTP fields */
    const char* url;
    const char* auth_header;
    const void* body;
    size_t body_length; /* used by HTTP_POST_BIN */
    bool read_auth;
    struct sim800l_http_response* response;

    /* Netscan field */
    struct sim800l_netscan_result* netscan_result;

    /* Completion signaling (set by modem_execute) */
    TaskHandle_t caller;
    int* result;
};

/**
 * Initialize modem request queue. Call before scheduler starts.
 */
void modem_init(void);

/**
 * Submit a request to the modem task (non-blocking).
 * @return 0 on success, -1 if queue is full
 */
int modem_submit(struct modem_request* request);

/**
 * Submit a request and block until it completes.
 * @return result code from modem processing
 */
int modem_execute(struct modem_request* request);

#endif /* UMETER_TASKS_H_ */
