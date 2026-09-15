#include "pca9685.h"

#define PCA9685_I2C_ADDRESS       (0x40U << 1)

#define PCA9685_MODE1             0x00U
#define PCA9685_MODE2             0x01U
#define PCA9685_LED0_ON_L         0x06U
#define PCA9685_PRESCALE          0xFEU

#define PCA9685_MODE1_RESTART     0x80U
#define PCA9685_MODE1_AUTO_INC    0x20U
#define PCA9685_MODE1_SLEEP       0x10U
#define PCA9685_MODE2_OUTDRV      0x04U

#define PCA9685_PWM_FREQUENCY_HZ  60U
#define PCA9685_OSCILLATOR_HZ     25000000UL
#define PCA9685_PWM_RESOLUTION    4096UL
#define PCA9685_SERVO_MIN_COUNT   150U
#define PCA9685_SERVO_MAX_COUNT   600U
#define PCA9685_I2C_TIMEOUT_MS    100U

static HAL_StatusTypeDef PCA9685_WriteRegister(I2C_HandleTypeDef *hi2c,
                                                uint8_t reg,
                                                uint8_t value)
{
  return HAL_I2C_Mem_Write(hi2c, PCA9685_I2C_ADDRESS, reg,
                           I2C_MEMADD_SIZE_8BIT, &value, 1U,
                           PCA9685_I2C_TIMEOUT_MS);
}

static HAL_StatusTypeDef PCA9685_ReadRegister(I2C_HandleTypeDef *hi2c,
                                               uint8_t reg,
                                               uint8_t *value)
{
  return HAL_I2C_Mem_Read(hi2c, PCA9685_I2C_ADDRESS, reg,
                          I2C_MEMADD_SIZE_8BIT, value, 1U,
                          PCA9685_I2C_TIMEOUT_MS);
}

HAL_StatusTypeDef PCA9685_Init(I2C_HandleTypeDef *hi2c)
{
  HAL_StatusTypeDef status;
  uint8_t old_mode;
  uint8_t sleep_mode;
  uint8_t prescale;

  if (hi2c == NULL)
  {
    return HAL_ERROR;
  }

  status = PCA9685_WriteRegister(hi2c, PCA9685_MODE1, 0x00U);
  if (status != HAL_OK)
  {
    return status;
  }

  status = PCA9685_ReadRegister(hi2c, PCA9685_MODE1, &old_mode);
  if (status != HAL_OK)
  {
    return status;
  }

  sleep_mode = (uint8_t)((old_mode & (uint8_t)~PCA9685_MODE1_RESTART) |
                         PCA9685_MODE1_SLEEP);
  status = PCA9685_WriteRegister(hi2c, PCA9685_MODE1, sleep_mode);
  if (status != HAL_OK)
  {
    return status;
  }

  prescale = (uint8_t)((PCA9685_OSCILLATOR_HZ +
                        (PCA9685_PWM_RESOLUTION * PCA9685_PWM_FREQUENCY_HZ / 2U)) /
                       (PCA9685_PWM_RESOLUTION * PCA9685_PWM_FREQUENCY_HZ) - 1U);
  status = PCA9685_WriteRegister(hi2c, PCA9685_PRESCALE, prescale);
  if (status != HAL_OK)
  {
    return status;
  }

  status = PCA9685_WriteRegister(hi2c, PCA9685_MODE2, PCA9685_MODE2_OUTDRV);
  if (status != HAL_OK)
  {
    return status;
  }

  status = PCA9685_WriteRegister(hi2c, PCA9685_MODE1,
                                 (uint8_t)(old_mode | PCA9685_MODE1_AUTO_INC));
  if (status != HAL_OK)
  {
    return status;
  }

  HAL_Delay(5U);
  return PCA9685_WriteRegister(hi2c, PCA9685_MODE1,
                               (uint8_t)(old_mode | PCA9685_MODE1_RESTART |
                                         PCA9685_MODE1_AUTO_INC | 0x01U));
}

HAL_StatusTypeDef PCA9685_SetAllAngle(I2C_HandleTypeDef *hi2c, uint8_t angle)
{
  uint8_t channel;
  HAL_StatusTypeDef status;

  if ((hi2c == NULL) || (angle > 180U))
  {
    return HAL_ERROR;
  }

  for (channel = 0U; channel < PCA9685_CHANNEL_COUNT; ++channel)
  {
    status = PCA9685_SetAngle(hi2c, channel, angle);
    if (status != HAL_OK)
    {
      return status;
    }
  }

  return HAL_OK;
}

HAL_StatusTypeDef PCA9685_SetAngle(I2C_HandleTypeDef *hi2c, uint8_t channel,
                                  uint8_t angle)
{
  uint16_t off_count;
  uint8_t pwm_data[4];

  if ((hi2c == NULL) || (channel >= PCA9685_CHANNEL_COUNT) || (angle > 180U))
  {
    return HAL_ERROR;
  }

  off_count = (uint16_t)(PCA9685_SERVO_MIN_COUNT +
                         (((PCA9685_SERVO_MAX_COUNT - PCA9685_SERVO_MIN_COUNT) *
                           (uint16_t)angle) / 180U));

  pwm_data[0] = 0U;
  pwm_data[1] = 0U;
  pwm_data[2] = (uint8_t)(off_count & 0xFFU);
  pwm_data[3] = (uint8_t)((off_count >> 8U) & 0x0FU);

  return HAL_I2C_Mem_Write(hi2c, PCA9685_I2C_ADDRESS,
                           (uint8_t)(PCA9685_LED0_ON_L + (4U * channel)),
                           I2C_MEMADD_SIZE_8BIT, pwm_data, sizeof(pwm_data),
                           PCA9685_I2C_TIMEOUT_MS);
}
