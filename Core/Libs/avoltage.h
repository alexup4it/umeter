/*
 * Analog voltage meter
 *
 * Dmitry Proshutinsky <dproshutinsky@gmail.com>
 * 2024-2026
 */

#ifndef AVOLTAGE_H_
#define AVOLTAGE_H_

#include "stm32f4xx_hal.h"

#include "cmsis_os.h"
#include "semphr.h"

struct avoltage
{
	SemaphoreHandle_t mutex;
	ADC_HandleTypeDef *adc;
	int ratio;
	GPIO_TypeDef *en_port;
	uint16_t en_pin;
};


void avoltage_init(struct avoltage *avlt, ADC_HandleTypeDef *adc, int ratio,
		GPIO_TypeDef *en_port, uint16_t en_pin);
int avoltage_calib(struct avoltage *avlt);
int avoltage(struct avoltage *avlt);

#endif /* AVOLTAGE_H_ */
