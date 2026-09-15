#include "serial_port.h"

#define SERIAL_BAUD_RATE  115200UL

void SerialPort_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  uint32_t peripheral_clock;

  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_USART1_CLK_ENABLE();

  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  USART1->CR1 = 0U;
  USART1->CR2 = 0U;
  USART1->CR3 = 0U;
  peripheral_clock = HAL_RCC_GetPCLK2Freq();
  USART1->BRR = (peripheral_clock + (SERIAL_BAUD_RATE / 2U)) /
                SERIAL_BAUD_RATE;
  USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_UE;
}

uint8_t SerialPort_ReadByte(uint8_t *byte, uint32_t timeout_ms)
{
  uint32_t started_at = HAL_GetTick();

  if (byte == NULL)
  {
    return 0U;
  }

  while ((USART1->SR & USART_SR_RXNE) == 0U)
  {
    if ((HAL_GetTick() - started_at) >= timeout_ms)
    {
      return 0U;
    }
  }
  *byte = (uint8_t)USART1->DR;
  return 1U;
}

void SerialPort_Write(const uint8_t *data, uint16_t length)
{
  uint16_t index;

  if (data == NULL)
  {
    return;
  }

  for (index = 0U; index < length; ++index)
  {
    while ((USART1->SR & USART_SR_TXE) == 0U)
    {
    }
    USART1->DR = data[index];
  }
  while ((USART1->SR & USART_SR_TC) == 0U)
  {
  }
}
