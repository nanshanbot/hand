#ifndef PCA9685_H
#define PCA9685_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

#define PCA9685_CHANNEL_COUNT  16U

HAL_StatusTypeDef PCA9685_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef PCA9685_SetAngle(I2C_HandleTypeDef *hi2c, uint8_t channel,
                                  uint8_t angle);
HAL_StatusTypeDef PCA9685_SetAllAngle(I2C_HandleTypeDef *hi2c, uint8_t angle);

#ifdef __cplusplus
}
#endif

#endif /* PCA9685_H */
