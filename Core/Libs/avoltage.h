/*
 * Analog voltage meter
 */

#ifndef AVOLTAGE_H_
#define AVOLTAGE_H_

#include "cmsis_os.h"
#include "semphr.h"
#include "stm32f4xx_hal.h"

struct avoltage {
    SemaphoreHandle_t mutex;
    ADC_HandleTypeDef* adc;
    int ratio;
};

void avoltage_init(struct avoltage* self, ADC_HandleTypeDef* adc, int ratio);
int avoltage_calib(struct avoltage* self);
int avoltage(struct avoltage* self);

#endif /* AVOLTAGE_H_ */
