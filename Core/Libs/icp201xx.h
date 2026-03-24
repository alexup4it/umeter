/*
 * ICP-20100 barometric pressure sensor
 */

#ifndef ICP201XX_H_
#define ICP201XX_H_

#include "stm32f4xx_hal.h"

struct icp201xx {
    I2C_HandleTypeDef* i2c;
};

void icp201xx_init(struct icp201xx* self, I2C_HandleTypeDef* i2c);
int icp201xx_is_available(struct icp201xx* self);

/*
 * Read pressure in Pa * 100 (centipascals).
 * Returns 0 on success, -1 on error.
 */
int icp201xx_read(struct icp201xx* self, int32_t* pressure);

#endif /* ICP201XX_H_ */
