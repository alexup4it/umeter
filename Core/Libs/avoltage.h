/*
 * Analog voltage meter
 */

#ifndef AVOLTAGE_H_
#define AVOLTAGE_H_

#include "cmsis_os.h"
#include "semphr.h"
#include "stm32f4xx_hal.h"

typedef void (*avoltage_pm_fn)(void);

struct avoltage {
    SemaphoreHandle_t mutex;
    ADC_HandleTypeDef* adc;
    int ratio;
    avoltage_pm_fn power_on;
    avoltage_pm_fn power_off;
};

void avoltage_init(struct avoltage* self,
                   ADC_HandleTypeDef* adc,
                   int ratio,
                   avoltage_pm_fn power_on,
                   avoltage_pm_fn power_off);
int avoltage_calib(struct avoltage* self);
int avoltage(struct avoltage* self);

#endif /* AVOLTAGE_H_ */
