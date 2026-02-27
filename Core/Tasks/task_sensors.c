/*
 * Sensors task
 */

#include "ptasks.h"

#include <stdlib.h>
#include <string.h>

#include "atomic.h"
#include "as5600.h"
#include "aht20.h"
#include "watchdog.h"

#include "logger.h"
#ifdef LOGGER
#define TAG "SENSORS"
extern struct logger logger;
#endif

//#define AVOLTAGE_CALIB
#define ANGLE_MAX 360000

enum
{
	AVAIL_VOL    = 0x01,
	AVAIL_CNT    = 0x02, // Not used
	AVAIL_TMPx75 = 0x04, // Not used
	AVAIL_AHT20  = 0x08,
	AVAIL_DIST   = 0x10, // Not used
	AVAIL_AS5600 = 0x20,
};

enum
{
	DRDY_VOL  = 0x01,
	DRDY_CNT  = 0x02, // Not used
	DRDY_TMP  = 0x04,
	DRDY_HUM  = 0x08,
	DRDY_DIST = 0x10, // Not used
	DRDY_ANG  = 0x20,
};

static osThreadId_t handle;
static const osThreadAttr_t attributes = {
  .name = "sensors",
  .stack_size = 112 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

static void set_angle_offset(params_t *params, int32_t angle)
{
	params_t uparams;

	memcpy(&uparams, params, sizeof(uparams));
	uparams.offset_angle = angle;

	/* todo: log offset angle value */

	watchdog_reset();
	vTaskSuspendAll();
	params_set(&uparams);
	xTaskResumeAll();

	params->offset_angle = angle;
}

static void task(void *argument)
{
	struct sensors *sens = argument;
	int avail = 0; // Sensor is available (bit mask)
	int drdy; // Sensor data was successfully read (bit mask)
	int ret;

	int32_t temperature = 0;
	int32_t humidity = 0;
	int32_t angle_wo = 0; // With offset
	int32_t angle = 0;
	int voltage = 0;
	uint32_t ts;

	char *tmp;

//	TickType_t wake = xTaskGetTickCount();

	// Voltage
#ifdef AVOLTAGE_CALIB
	ret = avoltage_calib(sens->avlt);
	if (!ret)
		avail |= AVAIL_VOL;
#endif /* AVOLTAGE_CALIB */
	avail |= AVAIL_VOL;

	// AHT20
	ret = aht20_is_available(sens->aht);
	if (!ret)
		avail |= AVAIL_AHT20;

	// AS5600
	ret = as5600_is_available(sens->pot);
	if (!ret)
		avail |= AVAIL_AS5600;

	// Available sensors
	xSemaphoreTake(sens->actual->mutex, portMAX_DELAY);
	sens->actual->avail = avail;
	xSemaphoreGive(sens->actual->mutex);

	#ifdef LOGGER
	tmp = pvPortMalloc(sizeof(avail) * 8 + 2); // + "b\0"
	if (tmp)
	{
		itoa(avail, tmp, 2);
		strcat(tmp, "b");
		logger_add_str(&logger, TAG, false, tmp);
		vPortFree(tmp);
	}
	#endif

	for (;;)
	{
		// Update sensor readings
		drdy = 0;

		if (avail & AVAIL_VOL)
		{
			voltage = avoltage(sens->avlt);
			if (voltage >= 0)
				drdy |= DRDY_VOL;
		}
		if (avail & AVAIL_AHT20)
		{
			ret = aht20_read(sens->aht, &temperature, &humidity);
			if (!ret)
				drdy |= DRDY_TMP | DRDY_HUM;
		}
		if (avail & AVAIL_AS5600)
		{
			angle = as5600_read(sens->pot);
			if (angle >= 0)
			{
				drdy |= DRDY_ANG;

				if (angle >= sens->params->offset_angle)
					angle_wo = angle - sens->params->offset_angle;
				else
					angle_wo = ANGLE_MAX + angle - sens->params->offset_angle;
			}
		}

		ts = *sens->timestamp;

		// Save sensor readings
		xSemaphoreTake(sens->actual->mutex, portMAX_DELAY);
		if (drdy & DRDY_VOL)
			sens->actual->voltage = voltage;
		if (drdy & DRDY_TMP)
			sens->actual->temperature = temperature;
		if (drdy & DRDY_HUM)
			sens->actual->humidity = humidity;
		if (drdy & DRDY_ANG)
			sens->actual->angle = angle_wo;
		xSemaphoreGive(sens->actual->mutex);

		// Build combined sensor record
		{
			struct sensor_record rec;
			struct counter_accum ca;

			memset(&rec, 0, sizeof(rec));
			rec.timestamp = ts;
			if (drdy & DRDY_VOL)
				rec.voltage = voltage;
			if (drdy & DRDY_TMP)
				rec.temperature = temperature;
			if (drdy & DRDY_HUM)
				rec.humidity = humidity;
			if (drdy & DRDY_ANG)
				rec.angle = angle_wo;

			counter_accum_read(sens->cnt, &ca);
			rec.count_avg = ca.avg;
			rec.count_min = ca.min;
			rec.count_max = ca.max;

			mqueue_set(sens->queue, &rec);
		}

		if (sens->events)
		{
			sens->events = 0;

			if (drdy & DRDY_ANG)
				set_angle_offset(sens->params, angle);
		}

//		vTaskDelayUntil(&wake, pdMS_TO_TICKS(sens->params->period_sen * 1000));
		ulTaskNotifyTake(pdTRUE,
				pdMS_TO_TICKS(sens->params->period_sen * 1000));
	}
}

/******************************************************************************/
void task_sensors(struct sensors *sens)
{
	handle = osThreadNew(task, sens, &attributes);
}

/******************************************************************************/
void task_sensors_notify(struct sensors *sens)
{
	BaseType_t woken = pdFALSE;

	atomic_inc(&sens->events);
	vTaskNotifyGiveFromISR(handle, &woken);
}
