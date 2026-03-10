/*
 * Analog voltage meter
 *
 * Dmitry Proshutinsky <dproshutinsky@gmail.com>
 * 2024-2026
 */

// TODO: Wrong ADC values

#include "avoltage.h"

#include <string.h>

#define ADC_TIMEOUT 100
#define ADC_MAX     4095
#define ADC_REF     3300

/* Settling time for voltage divider after enabling MOSFET (ms) */
#define MEAS_SETTLE_MS 2


/******************************************************************************/
void avoltage_init(struct avoltage *avlt, ADC_HandleTypeDef *adc, int ratio,
				   avoltage_power_cb power_on, avoltage_power_cb power_off)
{
	memset(avlt, 0, sizeof(*avlt));
	avlt->adc = adc;
	avlt->ratio = ratio;
	avlt->power_on = power_on;
	avlt->power_off = power_off;
	avlt->mutex = xSemaphoreCreateMutex();

	/* Start with divider off */
	avlt->power_off();
}

/******************************************************************************/
int avoltage_calib(struct avoltage *avlt)
{
	HAL_StatusTypeDef status;

	xSemaphoreTake(avlt->mutex, portMAX_DELAY);
//	status = HAL_ADCEx_Calibration_Start(avlt->adc);
	status = HAL_OK; /* todo: remove calibration */
	xSemaphoreGive(avlt->mutex);

	if (status != HAL_OK)
		return -1;

	return 0;
}

/******************************************************************************/
int avoltage(struct avoltage *avlt)
{
	HAL_StatusTypeDef status;
	uint32_t value;

	xSemaphoreTake(avlt->mutex, portMAX_DELAY);

	/* Enable voltage divider */
	avlt->power_on();
	osDelay(MEAS_SETTLE_MS);

	status = HAL_ADC_Start(avlt->adc);
	if (status != HAL_OK)
	{
		avlt->power_off();
		xSemaphoreGive(avlt->mutex);
		return -1;
	}

	status = HAL_ADC_PollForConversion(avlt->adc, ADC_TIMEOUT);
	if (status != HAL_OK)
	{
		avlt->power_off();
		xSemaphoreGive(avlt->mutex);
		return -1;
	}

	value = HAL_ADC_GetValue(avlt->adc);

	/* Disable voltage divider to eliminate quiescent current */
	avlt->power_off();

	xSemaphoreGive(avlt->mutex);

	return value * avlt->ratio * ADC_REF / ADC_MAX;
}
