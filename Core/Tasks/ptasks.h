/*
 * Application tasks
 */

#ifndef UMETER_TASKS_H_
#define UMETER_TASKS_H_

#include "cmsis_os.h"
#include "semphr.h"

#include "avoltage.h"
#include "counter.h"
#include "sim800l.h"
#include "button.h"
#include "siface.h"
#include "params.h"
#include "mqueue.h"
#include "ota.h"

#define SENSORS_QUEUE_SECNUM 16

struct actual
{
	SemaphoreHandle_t mutex;

	int avail;

	int voltage;
	uint32_t count;
	int32_t angle;
	int32_t humidity;
	int32_t temperature;
};

struct sensors
{
	mqueue_t *qtmp;
	mqueue_t *qhum;
	mqueue_t *qang;

	struct avoltage *avlt;
	struct as5600 *pot;
	struct aht20 *aht;
	volatile uint32_t *timestamp;
	params_t *params;

	struct actual *actual;
	volatile uint32_t events;
};

struct ecounter
{
	mqueue_t *qec_avg;
	mqueue_t *qec_max;
	mqueue_t *qec_min;

	struct counter *cnt;
	volatile uint32_t *timestamp;
	params_t *params;

	struct actual *actual;
};

struct app
{
	struct sim800l *mod;
	struct sensors *sens;
	struct ecounter *ecnt;

	volatile uint32_t *timestamp;
	volatile struct bl_params *bl;
	params_t *params;
};

struct system
{
	volatile struct bl_params *bl;
	params_t *params;

	size_t main_stack_size;
};


struct watchdog;

void task_siface(struct siface *siface);
void task_sim800l(struct sim800l *mod);
void task_ota(struct ota *ota);
void task_app(struct app *app);
void task_watchdog();
void task_logging(struct system *sys);
void task_button(struct button *btn);
void task_sensors(struct sensors *sens);
void task_ecounter(struct ecounter *ecnt);

void task_sensors_notify(struct sensors *sens);

#endif /* UMETER_TASKS_H_ */
