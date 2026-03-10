/*
 * Analog voltage meter
 *
 * Dmitry Proshutinsky <dproshutinsky@gmail.com>
 * 2024-2026
 */

#ifndef AVOLTAGE_H_
#define AVOLTAGE_H_

#include "cmsis_os.h"
#include "semphr.h"
#include "stm32f4xx_hal.h"

typedef void (*avoltage_power_cb)(void);

struct avoltage {
    SemaphoreHandle_t mutex;
    ADC_HandleTypeDef* adc;
    int ratio;
    avoltage_power_cb power_on;
    avoltage_power_cb power_off;
};

void avoltage_init(struct avoltage* avlt,
                   ADC_HandleTypeDef* adc,
                   int ratio,
                   avoltage_power_cb power_on,
                   avoltage_power_cb power_off);
int avoltage_calib(struct avoltage* avlt);
int avoltage(struct avoltage* avlt);

#endif /* AVOLTAGE_H_ */
