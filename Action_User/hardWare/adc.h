#ifndef __ADC_H
#define __ADC_H	
//#include "sys.h" 
#include "stdint.h"
#include "stm32f4xx_adc.h"

void Laser_Init(void);
void LaserAB_Init(void); 	
void LaserD_Init(void);
uint16_t  Get_Adc(uint8_t channel); 				
float Get_Adc_Average(uint8_t channel,int times);
#endif 
