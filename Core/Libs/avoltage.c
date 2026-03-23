/*
 * Analog voltage meter
 */

// TODO: Wrong ADC values

#include "avoltage.h"

#include <string.h>

#define ADC_TIMEOUT 100
#define ADC_MAX     4095
#define ADC_REF     3300

/* Settling time for voltage divider after enabling MOSFET (ms) */
#define MEAS_SETTLE_MS 10

/******************************************************************************/
void avoltage_init(struct avoltage* self, ADC_HandleTypeDef* adc, int ratio) {
    memset(self, 0, sizeof(*self));
    self->adc   = adc;
    self->ratio = ratio;
    self->mutex = xSemaphoreCreateMutex();

    /* Start with divider off */
}

/******************************************************************************/
int avoltage_calib(struct avoltage* self) {
    HAL_StatusTypeDef status;

    xSemaphoreTake(self->mutex, portMAX_DELAY);
    //	status = HAL_ADCEx_Calibration_Start(self->adc);
    status = HAL_OK; /* todo: remove calibration */
    xSemaphoreGive(self->mutex);

    if (status != HAL_OK) {
        return -1;
    }

    return 0;
}

/******************************************************************************/
int avoltage(struct avoltage* self) {
    HAL_StatusTypeDef status;
    uint32_t value;

    xSemaphoreTake(self->mutex, portMAX_DELAY);

    /* Enable voltage divider */
    osDelay(pdMS_TO_TICKS(MEAS_SETTLE_MS));

    status = HAL_ADC_Start(self->adc);
    if (status != HAL_OK) {
        xSemaphoreGive(self->mutex);
        return -1;
    }

    status = HAL_ADC_PollForConversion(self->adc, ADC_TIMEOUT);
    if (status != HAL_OK) {
        xSemaphoreGive(self->mutex);
        return -1;
    }

    value = HAL_ADC_GetValue(self->adc);

    xSemaphoreGive(self->mutex);

    return value * self->ratio * ADC_REF / ADC_MAX;
}
