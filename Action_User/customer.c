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

void ChectThePosSystemHF(uint8_t data);
void ChectThePosSystem2HF(uint8_t data);
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
		ChectThePosSystem2HF(ch);
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

/****************陀螺仪串口接收中断****start****************/
void USART3_IRQHandler(void)
{
	static uint8_t ch;
	static uint8_t count=0;
	static uint8_t i=0;
	static union
	{
		uint8_t data[32];
		float ActVal[8];
	}posture;
	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();/* Tell uC/OS-II that we are starting an ISR*/
	OSIntNesting++;
	OS_EXIT_CRITICAL();

	if(USART_GetITStatus(USART3, USART_IT_RXNE)==SET)   
	{
		USART_ClearITPendingBit(USART3,USART_IT_RXNE);
		ch=USART_ReceiveData(USART3);
		ChectThePosSystemHF(ch);
		//证明收到数了
		gRobot.PosSysError_t.receive=1;		
		if(gRobot.robotFlag&POSSYSTEM_NOT_RECEIVERIGHT)
		{
			USARTDMASendData(DEBUG_USART,ch,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY);
		}
//		USART_SendData(USART6,ch);			
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
			   if(i>=32)
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
					gRobot.PPS_baseInfo.encoderTimes[0]=posture.ActVal[1];
					gRobot.PPS_baseInfo.encoderTimes[1]=posture.ActVal[2];
					gRobot.PPS_baseInfo.angularV1=posture.ActVal[5];
					gRobot.PPS_baseInfo.encoder1.code1=posture.ActVal[6];
					gRobot.PPS_baseInfo.encoder1.code2=posture.ActVal[7];
					debugInfo.ppsAngle=posture.ActVal[0];
					debugInfo.ppsX = posture.ActVal[3] - (DISX_GYRO2CENTER*cosf(ANGLE_TO_RAD(gRobot.walk_t.angle))-DISY_GYRO2CENTER*sinf(ANGLE_TO_RAD(gRobot.walk_t.angle))) + DISX_GYRO2CENTER;
					debugInfo.ppsY = -posture.ActVal[4] + POS_Y_SHIFT - (DISX_GYRO2CENTER*sinf(ANGLE_TO_RAD(gRobot.walk_t.angle))+DISY_GYRO2CENTER*cosf(ANGLE_TO_RAD(gRobot.walk_t.angle))) + DISY_GYRO2CENTER; 
					
					//证明收到正确的数了
					gRobot.PosSysError_t.receiveRight=1;		
					
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
		USART_ReceiveData(USART3);
	}
	OSIntExit();
}

void ChectThePosSystemHF(uint8_t data)
{
	static uint8_t step=0;
	
	switch(step)
	{
		case 0:
			if(data=='h')
				step++;
			else
			{
				step=0;
			}
			break;
		case 1:
			if(data=='a')
				step++;
			else
			{
				step=0;
			}
			break;
		case 2:
			if(data=='r')
				step++;
			else
			{
				step=0;
			}
			break;
		case 3:
			if(data=='d')
				step++;
			else
			{
				step=0;
			}
			break;
		case 4:
			if(data=='f')
				step++;
			else
			{
				step=0;
			}
			break;
		case 5:
			if(data=='a')
				step++;
			else
			{
				step=0;
			}
			break;			
		case 6:
			if(data=='u')
				step++;
			else
			{
				step=0;
			}
			break;
		case 7:
			if(data=='l')
				step++;
			else
			{
				step=0;
			}
			break;
		case 8:
			if(data=='t')
				step++;
			else
			{
				step=0;
			}
		break;
		case 9:
			if(data=='\r')
				step++;
			else
			{
				step=0;
			}

			break;
		case 10:
			step=0;
			break;
	}
	
}


void ChectThePosSystem2HF(uint8_t data)
{
	static uint8_t step=0;
	
	switch(step)
	{
		case 0:
			if(data=='h')
				step++;
			else
			{
				step=0;
			}
			break;
		case 1:
			if(data=='a')
				step++;
			else
			{
				step=0;
			}
			break;
		case 2:
			if(data=='r')
				step++;
			else
			{
				step=0;
			}
			break;
		case 3:
			if(data=='d')
				step++;
			else
			{
				step=0;
			}
			break;
		case 4:
			if(data=='f')
				step++;
			else
			{
				step=0;
			}
			break;
		case 5:
			if(data=='a')
				step++;
			else
			{
				step=0;
			}
			break;			
		case 6:
			if(data=='u')
				step++;
			else
			{
				step=0;
			}
			break;
		case 7:
			if(data=='l')
				step++;
			else
			{
				step=0;
			}
			break;
		case 8:
			if(data=='t')
				step++;
			else
			{
				step=0;
			}
		break;
		case 9:
			if(data=='\r')
				step++;
			else
			{
				step=0;
			}

			break;
		case 10:
			step=0;

			break;
	}
	
}



static float g_zAngle = 0;
static float g_PosX = 0;
static float g_PosY = 0;



/***********************************************************************************
* @name 		setAngleZ
* @brief  	中断读取设定zangle值
* @param  	temp
* @retval 	无
**********************************************************************************/
void SetAngleZ(float temp)
{
	g_zAngle = temp;
}



/***********************************************************************************
* @name 		GetAngleZ
* @brief  	读取zangle值   人为将角度0旋转到与x轴正方向对齐
* @param  	无
* @retval 	陀螺仪角度
**********************************************************************************/
float GetAngleZ(void)
{
	return g_zAngle;
}


/***********************************************************************************
* @name 		SetPosx
* @brief  	中断读取设定X坐标
* @param  	temp
* @retval 	无
**********************************************************************************/
void SetPosx(float temp)
{
	g_PosX = temp;
}






/***********************************************************************************
* @name 		GetPosx
* @brief  	读取X坐标
* @param  	无
* @retval 	X坐标
**********************************************************************************/

float GetPosx(void)
{
	return g_PosX+gRobot.walk_t.gyroError_X;
}





/***********************************************************************************
* @name 		SetPosy
* @brief  	中断读取设定Y坐标
* @param  	temp
* @retval 	无
**********************************************************************************/
void SetPosy(float temp)
{
	g_PosY = temp;
}





/***********************************************************************************
* @name 		GetPosy
* @brief  	读取Y坐标
* @param  	无
* @retval 	Y坐标
**********************************************************************************/

float GetPosy(void)
{
	return g_PosY+gRobot.walk_t.gyroError_Y;
}



