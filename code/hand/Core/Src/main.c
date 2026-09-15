/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "pca9685.h"
#include "serial_port.h"
#include <stdlib.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HAND_SERVO_COUNT  11U
#define HAND_ANGLE_MIN    5U
#define HAND_ANGLE_MAX    170U
#define HAND_THUMB_ANGLE_MAX  140U
#define HAND_THUMB_ROTATION_CHANNEL  0U
#define HAND_THUMB_DISTAL_CHANNEL    2U
#define RX_LINE_SIZE      48U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static uint8_t servo_angles[HAND_SERVO_COUNT];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void Hand_ProcessSerial(void);
static void Hand_ProcessCommand(char *line);
static void Hand_Send(const char *text);
static void Hand_SendState(void);
static uint8_t Hand_GetAngleMax(uint8_t channel);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void Hand_Send(const char *text)
{
  SerialPort_Write((const uint8_t *)text, (uint16_t)strlen(text));
}

static uint8_t Hand_GetAngleMax(uint8_t channel)
{
  if ((channel == HAND_THUMB_ROTATION_CHANNEL) ||
      (channel == HAND_THUMB_DISTAL_CHANNEL))
  {
    return HAND_THUMB_ANGLE_MAX;
  }
  return HAND_ANGLE_MAX;
}

static char *Hand_AppendAngle(char *dst, uint8_t angle)
{
  if (angle >= 100U)
  {
    *dst++ = (char)('0' + (angle / 100U));
  }
  if (angle >= 10U)
  {
    *dst++ = (char)('0' + ((angle / 10U) % 10U));
  }
  *dst++ = (char)('0' + (angle % 10U));
  return dst;
}

static void Hand_SendState(void)
{
  char response[64];
  char *cursor = response;
  uint8_t channel;

  memcpy(cursor, "STATE ", 6U);
  cursor += 6U;
  for (channel = 0U; channel < HAND_SERVO_COUNT; ++channel)
  {
    cursor = Hand_AppendAngle(cursor, servo_angles[channel]);
    *cursor++ = (channel == (HAND_SERVO_COUNT - 1U)) ? '\r' : ',';
  }
  *cursor++ = '\n';
  SerialPort_Write((uint8_t *)response, (uint16_t)(cursor - response));
}

static uint8_t Hand_ParseNumber(const char *text, uint32_t *value)
{
  char *end;

  if ((text == NULL) || (*text == '\0'))
  {
    return 0U;
  }
  *value = strtoul(text, &end, 10);
  return (*end == '\0') ? 1U : 0U;
}

static void Hand_ProcessCommand(char *line)
{
  char *command = strtok(line, " ");
  char *first = strtok(NULL, " ");
  char *second = strtok(NULL, " ");
  char *extra = strtok(NULL, " ");
  uint32_t channel;
  uint32_t angle;
  uint8_t i;

  if (command == NULL)
  {
    return;
  }

  if ((strcmp(command, "PING") == 0) && (first == NULL))
  {
    Hand_Send("PONG\r\n");
  }
  else if ((strcmp(command, "GET") == 0) && (first == NULL))
  {
    Hand_SendState();
  }
  else if ((strcmp(command, "SET") == 0) && (extra == NULL) &&
           Hand_ParseNumber(first, &channel) && Hand_ParseNumber(second, &angle))
  {
    if (channel >= HAND_SERVO_COUNT)
    {
      Hand_Send("ERR CHANNEL 0-10\r\n");
    }
    else if ((angle < HAND_ANGLE_MIN) ||
             (angle > Hand_GetAngleMax((uint8_t)channel)))
    {
      if (Hand_GetAngleMax((uint8_t)channel) == HAND_THUMB_ANGLE_MAX)
      {
        Hand_Send("ERR ANGLE 5-140\r\n");
      }
      else
      {
        Hand_Send("ERR ANGLE 5-170\r\n");
      }
    }
    else if (PCA9685_SetAngle(&hi2c1, (uint8_t)channel,
                              (uint8_t)angle) != HAL_OK)
    {
      Hand_Send("ERR I2C\r\n");
    }
    else
    {
      servo_angles[channel] = (uint8_t)angle;
      Hand_Send("OK\r\n");
    }
  }
  else if ((strcmp(command, "ALL") == 0) && (second == NULL) &&
           Hand_ParseNumber(first, &angle))
  {
    if ((angle < HAND_ANGLE_MIN) || (angle > HAND_ANGLE_MAX))
    {
      Hand_Send("ERR ANGLE 5-170\r\n");
      return;
    }
    for (i = 0U; i < HAND_SERVO_COUNT; ++i)
    {
      uint8_t channel_angle = (uint8_t)angle;
      uint8_t channel_max = Hand_GetAngleMax(i);

      if (channel_angle > channel_max)
      {
        channel_angle = channel_max;
      }
      if (PCA9685_SetAngle(&hi2c1, i, channel_angle) != HAL_OK)
      {
        Hand_Send("ERR I2C\r\n");
        return;
      }
      servo_angles[i] = channel_angle;
    }
    Hand_Send("OK\r\n");
  }
  else
  {
    Hand_Send("ERR COMMAND\r\n");
  }
}

static void Hand_ProcessSerial(void)
{
  static char line[RX_LINE_SIZE];
  static uint8_t length = 0U;
  uint8_t byte;

  if (!SerialPort_ReadByte(&byte, 5U))
  {
    return;
  }

  if ((byte == '\r') || (byte == '\n'))
  {
    if (length > 0U)
    {
      line[length] = '\0';
      Hand_ProcessCommand(line);
      length = 0U;
    }
  }
  else if (length < (RX_LINE_SIZE - 1U))
  {
    line[length++] = (char)byte;
  }
  else
  {
    length = 0U;
    Hand_Send("ERR LINE TOO LONG\r\n");
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  uint8_t channel;

  SerialPort_Init();
  HAL_Delay(100U);
  if (PCA9685_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  for (channel = 0U; channel < HAND_SERVO_COUNT; ++channel)
  {
    servo_angles[channel] = HAND_ANGLE_MIN;
    if (PCA9685_SetAngle(&hi2c1, channel, HAND_ANGLE_MIN) != HAL_OK)
    {
      Error_Handler();
    }
  }
  Hand_Send("READY HAND11 5-170 THUMB-ROT-DIST 5-140\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    Hand_ProcessSerial();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
