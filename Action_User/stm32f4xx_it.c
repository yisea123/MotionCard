/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Template/stm32f4xx_it.c
  * @author  MCD Application Team
  * @version V1.0.1
  * @date    13-April-2012
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "stm32f4xx.h"
#include  <ucos_ii.h>
#include "app_cfg.h"
#include <math.h>
#include "usart.h"
#include "timer.h"
#include "can.h"
#include "gpio.h"
#include "elmo.h"
#include "task.h"
#include "dma.h"
#include "Move.h"
#include "iwdg.h"
#include "laser.h"
/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/
extern Robot_t gRobot;
//用来处理CAN接收数据
union MSG
{
	uint8_t data8[8];
	int data32[2];
	float dataf[2];
}msg;

void CAN1_RX0_IRQHandler(void)
{
	OS_CPU_SR  cpu_sr;
//	uint8_t buffer[8];
	uint32_t StdId=0;
	uint8_t canNodeId = 0;
//	int32_t i = 0;

	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	CAN_RxMsg(CAN1, &StdId, msg.data8, 8);
	canNodeId = StdId - SDO_RESPONSE_COB_ID_BASE;

	if(msg.data32[0]==0x00111111)
	{
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
		(uint8_t *)"CANError:");
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
		(uint8_t *)"id:%d",StdId);
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
		(uint8_t *)"msg:%d",msg.data32[1]);
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
		(uint8_t *)"\r\n");
		BEEP_ON;
	}
	
	if(msg.data32[0]==0x00111112)
	{
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
		(uint8_t *)"CANError_2:");
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
		(uint8_t *)"id:%d",StdId);
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
		(uint8_t *)"msg:%d",msg.data32[1]);
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
		(uint8_t *)"\r\n");
		BEEP_ON;
	}
	
	switch(canNodeId)
	{
		case  1:
			
			//位置
			if(msg.data32[0]==0x00005850)
			{
				gRobot.wheelPosition.wheelPosition2 = msg.data32[1];
			}
		
			//速度
			if(msg.data32[0]==0x00005856)
			{
				gRobot.wheelVel.v2 = msg.data32[1];
				debugInfo.wheelActVel.wheel2 = gRobot.wheelVel.v2;
				if(gRobot.wheelHB[1]<1)
				{
					gRobot.wheelHB[1]=1;
				}
				gRobot.wheelHB[1]--;
			}
			if(msg.data32[0]==0x22222222)
			{
				if(msg.data32[1]==0x22222222)
				{
					gRobot.wheelHardfault[1] = 1;
				}
			}
			if(msg.data32[0]==*(uint32_t *)"STAW")
			{
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"\r\nWheel Id 1 mga:%d\r\n",msg.data32[1]);
			}
			break;
		case  2:
			
			//位置
			if(msg.data32[0]==0x00005850)
			{
				gRobot.wheelPosition.wheelPosition3 = msg.data32[1];
			}
		
			//速度
			if(msg.data32[0]==0x00005856)
			{
				gRobot.wheelVel.v3 = msg.data32[1];
				debugInfo.wheelActVel.wheel3 = gRobot.wheelVel.v3;
				if(gRobot.wheelHB[2]<1)
				{
					gRobot.wheelHB[2]=1;
				}
				gRobot.wheelHB[2]--;
			}
			if(msg.data32[0]==0x22222222)
			{
				if(msg.data32[1]==0x22222222)
				{
					gRobot.wheelHardfault[2] = 1;
				}
			}
			if(msg.data32[0]==*(uint32_t *)"STAW")
			{
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"\r\nWheel Id 2 mga:%d\r\n",msg.data32[1]);
			}
			break;
		case  3:
			
			//位置
			if(msg.data32[0]==0x00005850)
			{
				gRobot.wheelPosition.wheelPosition4 = msg.data32[1];
			}
		
			//速度
			if(msg.data32[0]==0x00005856)
			{
				gRobot.wheelVel.v4 = msg.data32[1];
				debugInfo.wheelActVel.wheel4 = gRobot.wheelVel.v4;
				if(gRobot.wheelHB[3]<1)
				{
					gRobot.wheelHB[3]=1;
				}
				gRobot.wheelHB[3]--;
			}
			if(msg.data32[0]==0x22222222)
			{
				if(msg.data32[1]==0x22222222)
				{
					gRobot.wheelHardfault[3] = 1;
				}
			}
			if(msg.data32[0]==*(uint32_t *)"STAW")
			{
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"\r\nWheel Id 3 mga:%d\r\n",msg.data32[1]);
			}
			break;
		case  4:
			
			//位置
			if(msg.data32[0]==0x00005850)
			{
				gRobot.wheelPosition.wheelPosition1 = msg.data32[1];
			}
		
			//速度
			if(msg.data32[0]==0x00005856)
			{
				gRobot.wheelVel.v1 = msg.data32[1];
				debugInfo.wheelActVel.wheel1 = gRobot.wheelVel.v1;
				if(gRobot.wheelHB[0]<1)
				{
					gRobot.wheelHB[0]=1;
				}
				gRobot.wheelHB[0]--;
			}
			
			if(msg.data32[0]==0x22222222)
			{
				if(msg.data32[1]==0x22222222)
				{
					gRobot.wheelHardfault[0] = 1;
				}
			}
			
			if(msg.data32[0]==*(uint32_t *)"STAW")
			{
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"\r\nWheel Id 4 mga:%d\r\n",msg.data32[1]);
			}
			break;
		default:
			break;
	}

	CAN_ClearFlag(CAN1, CAN_FLAG_EWG);
	CAN_ClearFlag(CAN1, CAN_FLAG_EPV);
	CAN_ClearFlag(CAN1, CAN_FLAG_BOF);
	CAN_ClearFlag(CAN1, CAN_FLAG_LEC);

	CAN_ClearFlag(CAN1, CAN_FLAG_FMP0);
	CAN_ClearFlag(CAN1, CAN_FLAG_FF0);
	CAN_ClearFlag(CAN1, CAN_FLAG_FOV0);
	CAN_ClearFlag(CAN1, CAN_FLAG_FMP1);
	CAN_ClearFlag(CAN1, CAN_FLAG_FF1);
	CAN_ClearFlag(CAN1, CAN_FLAG_FOV1);
	OSIntExit();
}

void CAN2_RX0_IRQHandler(void)
{
	OS_CPU_SR cpu_sr;
	uint8_t buffer[8];
	uint32_t StdId=0;

	OS_ENTER_CRITICAL(); /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	
	CAN_RxMsg(CAN2, &StdId, buffer, 8);
	
	CAN_ClearFlag(CAN2, CAN_FLAG_EWG);
	CAN_ClearFlag(CAN2, CAN_FLAG_EPV);
	CAN_ClearFlag(CAN2, CAN_FLAG_BOF);
	CAN_ClearFlag(CAN2, CAN_FLAG_LEC);
	CAN_ClearFlag(CAN2, CAN_FLAG_FMP0);
	CAN_ClearFlag(CAN2, CAN_FLAG_FF0);
	CAN_ClearFlag(CAN2, CAN_FLAG_FOV0);
	CAN_ClearFlag(CAN2, CAN_FLAG_FMP1);
	CAN_ClearFlag(CAN2, CAN_FLAG_FF1);
	CAN_ClearFlag(CAN2, CAN_FLAG_FOV1);
	OSIntExit();
}



/**
  * @brief  CAN1 SCE interrupt  handler
  * @note
  * @param  None
  * @retval None
  */
void CAN1_SCE_IRQHandler(void)
{
	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"\r\nCAN1 BUS Off %d!!\r\n",CAN_GetLastErrorCode(CAN1));

	CAN_DeInit(CAN1);
	CAN_Config(CAN1, 500, GPIOB, GPIO_Pin_8, GPIO_Pin_9);

	CAN_ClearFlag(CAN1, CAN_FLAG_BOF);
	OSIntExit();
}

void CAN2_SCE_IRQHandler(void)
{
	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"\r\nCAN2 BUS Off %d!!\r\n",CAN_GetLastErrorCode(CAN2));
		
	CAN_DeInit(CAN2);
	CAN_Config(CAN2, 500, GPIOB, GPIO_Pin_5, GPIO_Pin_6);

	CAN_ClearFlag(CAN2, CAN_FLAG_BOF);
	OSIntExit();
}

/*************定时器2******start************/
//每1ms调用一次

extern  OS_EVENT 		*PeriodSem;

void JudgePosSystemError(void);

extern OS_EVENT *velCtrPeriodSem;

extern int cpuUsage;
void TIM2_IRQHandler(void)
{

	#define  VEL_CTR_PERIOD (5)
	//用来计数10次，产生10ms的定时器
	static uint8_t periodCounter = PERIOD_COUNTER;
	static  uint8_t velCtrPeriodCounter = VEL_CTR_PERIOD;


	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();


	if(TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{


		//实现10ms 发送1次信号量
		periodCounter--;
		
		velCtrPeriodCounter--;
		
		if (periodCounter == 0)
		{		
			OSSemPost(PeriodSem);
			periodCounter = PERIOD_COUNTER;
			if(cpuUsage!=0)
				cpuUsage++;
		}
		
		if(velCtrPeriodCounter==0)
		{
			OSSemPost(velCtrPeriodSem);
			velCtrPeriodCounter = VEL_CTR_PERIOD;
		}
		
		IWDG_Feed();

		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
	OSIntExit();
}

void JudgePosSystemError(void)
{
	#define  PERIOD_COUNTER_HALF (PERIOD_COUNTER/2)
	//收到正确的数才开始
	if(gRobot.robotFlag&POS_1_SYSTEM_READY)
	{
		//如果收到数了
		if(gRobot.PosSysError_t.receive)
		{
			gRobot.PosSysError_t.receive=0;
			gRobot.PosSysError_t.notReceiveNum=0;
		}
		else
		{
			gRobot.PosSysError_t.notReceiveNum++;
		}
		//如果收到正确数了
		if(gRobot.PosSysError_t.receiveRight)
		{
			gRobot.PosSysError_t.receiveRight=0;
			gRobot.PosSysError_t.notReceiveRightNum=0;
		}
		else
		{
			gRobot.PosSysError_t.notReceiveRightNum++;
		}
		if(gRobot.PosSysError_t.notReceiveNum*PERIOD_COUNTER_HALF>30)
		{

			debugInfo.opsStatus|=0x08;
		}
		if(gRobot.PosSysError_t.notReceiveRightNum*PERIOD_COUNTER_HALF>30)
		{
		}
	}
}

void TIM1_UP_TIM10_IRQHandler(void)
{
	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if(TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
	}
	OSIntExit();
}


void TIM8_UP_TIM13_IRQHandler(void)
{
	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if(TIM_GetITStatus(TIM8, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM8, TIM_IT_Update);
	}
	OSIntExit();
}

void TIM5_IRQHandler(void)
{
	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if(TIM_GetITStatus(TIM5, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
	}
	OSIntExit();
}

void TIM3_IRQHandler(void)
{
	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if(TIM_GetITStatus(TIM3, TIM_IT_Update) == SET)
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);
	}
	OSIntExit();
}

void TIM4_IRQHandler(void)
{
	OS_CPU_SR  cpu_sr;
	OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
	OSIntNesting++;
	OS_EXIT_CRITICAL();
	if(TIM_GetITStatus(TIM4, TIM_IT_Update)==SET)
	{
		TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
	}
	OSIntExit();
}

//void TIM7_IRQHandler(void)
//{ 
//	OS_CPU_SR  cpu_sr;
//  OS_ENTER_CRITICAL();                         /* Tell uC/OS-II that we are starting an ISR          */
//  OSIntNesting++;
//  OS_EXIT_CRITICAL();
//  if(TIM_GetITStatus(TIM7, TIM_IT_Update)==SET)
//  {	
//		if(startCnt==1)
//		{
//			Cnt++;
//		}
//    TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
//	}
//  OSIntExit();
//}


/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
   while (1)
   {
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"NMI EXCEPTION\r\n");   
   }
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void Hex_To_Str(uint8_t * pHex,char * s,float num)
{
  char        hex[] = "0123456789ABCDEF";
  char        *pStr = s;
  for (uint8_t i = 0; i < (int)(num/2.f+0.5f); i++)//(int)(x+0.5f)是把x四舍五入的意思
  {
    
    /*
    1.*pStr++右结合,并且*索引的是没有++之前的地址
    2.f.移位不会改变指针指向的那个空间的值
    3.对指针指向空间的移位也不会改变指针的指向
    */
    if (((num<((int)(num / 2.f + 0.5f))*2.f)&&i>0)|| (num==((int)(num / 2.f + 0.5f)) * 2.f))
      *pStr++ = hex[*(pHex + (int)(num / 2.f + 0.5f) - i - 1) >> 4];
    *pStr++ = hex[*(pHex + (int)(num / 2.f + 0.5f) - i - 1) & 0x0F];
  }
}
void HardFault_Handler(void)
{
/*屏蔽掉的只是为了返回进硬件中断的错误语句，但是比赛的时候不可能出现错误，只可能有静电等突发因素，所以不用返回了*/
  static uint32_t r_sp ;
	
  ThreeWheelVelControl(0,0,0);
  

  /*判断发生异常时使用MSP还是PSP*/
  if(__get_PSP()!=0x00) //获取SP的值
    r_sp = __get_PSP(); 
  else
    r_sp = __get_MSP(); 
  /*因为经历中断函数入栈之后，堆栈指针会减小0x10，所以平移回来（可能不具有普遍性）*/
  r_sp = r_sp+0x10;
  /*串口发数通知*/
  char sPoint[2]={0};
  USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"%s","0x");
  /*获取出现异常时程序的地址*/
  for(int i=3;i>=-28;i--){
    Hex_To_Str((uint8_t*)(r_sp+i+28),sPoint,2);
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"%s",sPoint);
    if(i%4==0)
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"\r\n");		
  }
  /*发送回车符*/
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"\r\n");		
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"Hard Fault\r\n");
	  
	ThreeWheelVelControl(0,0,0);
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"Memory Manage exception\r\n");   
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{

  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"Bus fault\r\n");   
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{

  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"Usage fault\r\n");   
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
	while(1)
	{
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
					(uint8_t *)"SVCall exceptiion\r\n");
	}
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
	while(1)
	{
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
					(uint8_t *)"Debug monitor exceptiion\r\n");	\
	}
}

