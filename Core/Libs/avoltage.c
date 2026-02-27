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
		GPIO_TypeDef *en_port, uint16_t en_pin)
{
	memset(avlt, 0, sizeof(*avlt));
	avlt->adc = adc;
	avlt->ratio = ratio;
	avlt->en_port = en_port;
	avlt->en_pin = en_pin;
	avlt->mutex = xSemaphoreCreateMutex();

	/* Start with divider off */
	HAL_GPIO_WritePin(avlt->en_port, avlt->en_pin, GPIO_PIN_RESET);
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
	HAL_GPIO_WritePin(avlt->en_port, avlt->en_pin, GPIO_PIN_SET);
	osDelay(MEAS_SETTLE_MS);

	status = HAL_ADC_Start(avlt->adc);
	if (status != HAL_OK) {
		HAL_GPIO_WritePin(avlt->en_port, avlt->en_pin, GPIO_PIN_RESET);
		xSemaphoreGive(avlt->mutex);
		return -1;
	}

	status = HAL_ADC_PollForConversion(avlt->adc, ADC_TIMEOUT);
	if (status != HAL_OK) {
		HAL_GPIO_WritePin(avlt->en_port, avlt->en_pin, GPIO_PIN_RESET);
		xSemaphoreGive(avlt->mutex);
		return -1;
	}

	value = HAL_ADC_GetValue(avlt->adc);

	/* Disable voltage divider to eliminate quiescent current */
	HAL_GPIO_WritePin(avlt->en_port, avlt->en_pin, GPIO_PIN_RESET);

	xSemaphoreGive(avlt->mutex);

	return value * avlt->ratio * ADC_REF / ADC_MAX;
}
