#ifndef __USART_H
#define __USART_H

#include "stdint.h"
#include "stm32f4xx_usart.h"

void USART1_Init(uint32_t BaudRate);
void UART5_Init(uint32_t BaudRate);
void UART4_Init(uint32_t BaudRate);
void GYR_Init(uint32_t BaudRate);
void USART6_Init(uint32_t BaudRate);
void DebugBLE_Init(uint32_t BaudRate);
void USART2_Init(uint32_t BaudRate);
void USART_OUT(USART_TypeDef* USARTx, const char *Data,...);
char *itoa(int value, char *string, int radix);
void USART_OUT_F(float value);
void USART_Enter(void);

#define DEBUG_USART USART1

#endif

