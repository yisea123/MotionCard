#ifndef __GPIO_H
#define __GPIO_H

#include "stm32f4xx_gpio.h"

#define KEYSWITCH		    	(GPIO_ReadInputDataBit(GPIOE, GPIO_Pin_0))

#define BEEP_OFF				GPIO_ResetBits(GPIOC, GPIO_Pin_3)

#define BEEP_ON				GPIO_SetBits(GPIOC, GPIO_Pin_3)

void GPIO_Init_Pins(GPIO_TypeDef * GPIOx,
					uint16_t GPIO_Pin,
					GPIOMode_TypeDef GPIO_Mode);

void KeyInit(void);
void Beep_Init(void);
void PhotoelectricityInit(void);
void CheckWipeWheel(void);
#endif
