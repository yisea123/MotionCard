#include "customer.h"
#include "string.h"
#include "stm32f4xx_usart.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "usart.h"
#include "gasvalveControl.h"
#include "task.h"
#include "elmo.h"
#include  <includes.h>
#include "update.h"
#include "dma.h"
#include "timer.h"

extern Robot_t gRobot;


void USART1_IRQHandler(void)
{
	uint8_t data;
	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();/* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if(USART_GetITStatus(USART1,USART_IT_RXNE)==SET)
	{
		data = USART_ReceiveData(USART1);
		data = data;
		USART_ClearITPendingBit( USART1,USART_IT_RXNE);
	}
	else
	{
		USART_ReceiveData(USART1);
	}
	OSIntExit();
}


void USART6_IRQHandler(void)
{
	uint8_t data;
	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();/* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if(USART_GetITStatus(USART6,USART_IT_RXNE)==SET)
	{
		data = USART_ReceiveData(USART6);
		data = data;

		USART_ClearITPendingBit( USART6,USART_IT_RXNE);
	}
	else
	{
		USART_ReceiveData(USART6);
	}
	OSIntExit();
}

void USART2_IRQHandler(void)
{
	static uint8_t ch;
	static union
	{
		uint8_t data[4];
		float ActVal;
	}posture;
	static uint8_t count=0;
	static uint8_t i=0;
	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();/* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	if(USART_GetITStatus(USART2, USART_IT_RXNE)==SET)   
	{
		USART_ClearITPendingBit(USART2,USART_IT_RXNE);
		ch=USART_ReceiveData(USART2);
		gRobot.PosSysError_t.receive=1;			
		 switch(count)
		 {
			 case 0:
				 if(ch==0x0d||ch=='O')
					 count++;
				 else
					 count=0;
				 break;
				 
			 case 1:
				 if(ch==0x0a)
				 {
					 i=0;
					 count++;
				 }
				 else if(ch==0x0d);
				 else if(ch=='K')
				 {
						count=0;
				 }
				 else
					 count=0;
				 break;
				 
			 case 2:
				posture.data[i]=ch;
			   i++;
			   if(i>=4)
				 {
					 i=0;
					 count++;
				 }
				 break;
				 
			 case 3:
				 if(ch==0x0a)
					 count++;
				 else
					 count=0;
				 break;
				 
			 case 4:
				 if(ch==0x0d)
				 {
						gRobot.PPS_baseInfo.angularV2=posture.ActVal;
						//证明收到正确的数了
						gRobot.PosSysError_t.receiveRight=1;		
						gRobot.PPS_baseInfo.angularV1=gRobot.PPS_baseInfo.angularV1;
				 }
			   count=0;
				 break;
			 
			 default:
				 count=0;
			   break;		 
		 }
	 }
	else
	{
		USART_ReceiveData(USART2);
	}
	OSIntExit();
}

float FilterSpeedX(float newValue);
float FilterSpeedY(float newValue);

