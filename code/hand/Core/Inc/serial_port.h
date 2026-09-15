#ifndef SERIAL_PORT_H
#define SERIAL_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f1xx_hal.h"

void SerialPort_Init(void);
uint8_t SerialPort_ReadByte(uint8_t *byte, uint32_t timeout_ms);
void SerialPort_Write(const uint8_t *data, uint16_t length);

#ifdef __cplusplus
}
#endif

#endif /* SERIAL_PORT_H */
