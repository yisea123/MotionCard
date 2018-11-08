#include "includes.h"
#include <app_cfg.h>
#include "misc.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "timer.h"
#include "gpio.h"
#include "usart.h"
#include "can.h"
#include "elmo.h"
#include "stm32f4xx_it.h"
#include "stm32f4xx_usart.h"
#include "gasvalveControl.h"
#include "dma.h"
#include "task.h"
#include "customer.h"
#include "MotionCard.h"
#include "update.h"
#include "adc.h"
#include "laser.h"
#include "iwdg.h"
#include "pathFollowing.h"
#include "Move.h"
#include "posSystem.h"
#include "pps.h"
/*
===============================================================
						信号量定义
===============================================================
*/
//CPU使用率变量
OS_EXT INT8U OSCPUUsage;

//任务周期用信号量定义
OS_EVENT *PeriodSem;

OS_EVENT *velCtrPeriodSem;
OS_EVENT *velCtrCmdSem;

int cpuUsage=0;
//自检任务声明
static  void  SelfTestTask(void);

static void VelCtrTask(void);

//定义互斥型信号量用于管理CAN发送资源
OS_EVENT *CANSendMutex;

//定义APPtask堆栈
static  OS_STK  App_ConfigStk[Config_TASK_START_STK_SIZE];

//定义ShootTask堆栈
static 	OS_STK  ShootTaskStk[SHOOT_TASK_STK_SIZE];

//定义SelfTestTask堆栈
static 	OS_STK  SelfTestTaskStk[SELF_TEST_TASK_STK_SIZE];

static OS_STK VelCtrTaskStk[VEL_CTR_TASK_STK_SIZE-1];

void App_Task()
{
	CPU_INT08U  os_err;
	os_err = os_err;		  /*防止警告...*/

	/*创建信号量*/
	PeriodSem				=	OSSemCreate(0);
	
	velCtrPeriodSem =  OSSemCreate(0);

	velCtrCmdSem = OSSemCreate(0);
	
	//创建互斥型信号量
	CANSendMutex			=   OSMutexCreate(9,&os_err);

	/*创建任务*/
	os_err = OSTaskCreate(	(void (*)(void *)) ConfigTask,				/*初始化任务*/
							(void		  * ) 0,
							(OS_STK		* )&App_ConfigStk[Config_TASK_START_STK_SIZE-1],
							(INT8U		   ) Config_TASK_START_PRIO);

	os_err = OSTaskCreate(	(void (*)(void *)) ShootTask,
							(void		  * ) 0,
							(OS_STK		* )&ShootTaskStk[SHOOT_TASK_STK_SIZE-1],
							(INT8U		   ) SHOOT_TASK_PRIO);
							
	os_err = OSTaskCreate(	(void (*)(void *)) SelfTestTask,
							(void		  * ) 0,
							(OS_STK		* )&SelfTestTaskStk[SELF_TEST_TASK_STK_SIZE-1],
							(INT8U		   ) SELF_TEST_TASK_PRIO);	

	os_err = OSTaskCreate(	(void (*)(void *))VelCtrTask,
							(void		  * ) 0,
							(OS_STK		* )&VelCtrTaskStk[VEL_CTR_TASK_STK_SIZE-1],
							(INT8U		   ) VEL_CTR_TASK_PRIO);									

}

Robot_t gRobot = {0};

//调试用数据结构体变量
debugInfo_t debugInfo ={0};

//外设初始化函数
void HardWareInit(void)
{
	//初始化调试串口DMA
	USARTDMASendInit(DEBUG_USART,DebugUSARTDMASendBuf,&DebugBLE_Init,921600);
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"\r\nConfiguration Start!!\r\n");
	Delay_ms(500);
	//初始化定位系统用串口USART3
	GYR_Init(115200);
	USART1_Init(921600);
//	ReStartThePosSystem();
	Delay_ms(3000);
	
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"GYR Init!!\r\n");
	
	//定时器初始化1ms主定时器
	TIM_Init(TIM2, 99, 839, 1, 0);
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"Timer Init!!\r\n");
	
	//涵道电调PWM初始化50Hz
	TIM3_Pwm_Init(20000-1,84-1);
	
	//初始化电机驱动器用CAN1
	CAN_Config(CAN1, 500, GPIOB, GPIO_Pin_8, GPIO_Pin_9);
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"CAN1 Init!!\r\n");


	//初始化和主控通信用CAN2
	CAN_Config(CAN2, 500, GPIOB, GPIO_Pin_5, GPIO_Pin_6);
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"CAN2 Init!!\r\n");

	
	//初始化和主控之间的串口USART6
	USART6_Init(921600);
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"USART6 Init!!\r\n");

  //擦轮行程开关
//	KeyInit();

	Beep_Init();
	
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"Gyro USART3 Init!!\r\n");

	
	//初始化激光采集用ADC和DMA
	ADC1mixed_DMA_Config();
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"Laser ADC&DMA Init!!\r\n");

	
	//初始化独立看门狗
	IWDG_Init(1,50); // 11ms-11.2ms
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"Hardware Init Finish!!\r\n");

}

void MotorInit(void)
{
	//电机初始化及使能
	ElmoInit(CAN1);
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"Elmo Init!!\r\n");


	//配置电机速度环
	VelLoopCfg(CAN1, 1, 5000000, 5000000);
	VelLoopCfg(CAN1, 2, 5000000, 5000000);
	VelLoopCfg(CAN1, 3, 5000000, 5000000);
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"Vel loop Init!!\r\n");

	
	//电机使能
	MotorOn(CAN1, 1);
	MotorOn(CAN1, 2);
	MotorOn(CAN1, 3);
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"Motor On!!\r\n");

	
	//给电机发送速度0命令
	VelCrl(CAN1,1,0);
	VelCrl(CAN1,2,0);
	VelCrl(CAN1,3,0);
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"Motor Init Finish!!\r\n");
}

void ConfigTask(void)
{
	CPU_INT08U  os_err;
	os_err = os_err;
	//初始化中断优先级分组
	Delay_ms(500);
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);	
	
	//硬件初始化
	HardWareInit();

//	//涵道电调初始化时间	
	#ifndef SUMMER
	//电机初始化
	MotorInit();
	#endif
	WaitOpsPrepare();

//	Delay_ms(2000);
//	while(1)
//	{
//		SetDuckedFanDutyCycle(DUCKED_FAN_RATIO);
//	}
	//初始化控制卡用循环数组
	BufferZizeInit(500);
	
//	Pose_t points[5] = 
//	{
//		{0.0,0.0,0.0},
//		{0.0,500.0,0.0},
//		{0.0,1000.0,0.0},
//		{0.0,1500.0,0.0},
//		{0.0,2000.0,0.0}
//	};
	Pose_t points[5] = 
	{
		{0.0,0.0,0.0},
		{500.0,0.0,0.0},
		{1000.0,0.0,0.0},
		{1500.0,0.0,0.0},
		{2000.0,0.0,0.0}
	};

	velPlan_t velPlan = {0.0,0.0,500.0};
	//规划第一条路径
	InputPoints2RingBuffer(points,5, 1 , velPlan);
	//发送初始化完成
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"\r\nFinish Init!Finish Init!Finish Init!Finish Init!Finish Init!Finish Init!Finish Init!Finish Init!Finish Init!\\r\n");
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"\r\nFinish Init!Finish Init!Finish Init!Finish Init!Finish Init!Finish Init!Finish Init!Finish Init!Finish Init!\\r\n");
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"\r\nFinish Init!Finish Init!Finish Init!Finish Init!Finish Init!Finish Init!Finish Init!Finish Init!Finish Init!\\r\n");
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"\r\nFinish Init!Finish Init!Finish Init!Finish Init!Finish Init!Finish Init!Finish Init!Finish Init!Finish Init!\\r\n");
	OSTaskSuspend(OS_PRIO_SELF);
}

void VelCtrTask(void)
{
	CPU_INT08U  os_err;
	os_err = os_err;
	
	OSSemSet(velCtrPeriodSem, 0, &os_err);
	while(1)
	{
		OSSemPend(velCtrCmdSem,0,&os_err);
		OSSemPend(velCtrPeriodSem, 0, &os_err);
		OSSemSet(velCtrPeriodSem,0,&os_err);
		
		VelControl((carVel_t){sqrtf(GetSpeedX()*GetSpeedX() + GetSpeedY()*GetSpeedY()),\
								atan2f(GetSpeedY(),GetSpeedX())*CHANGE_TO_ANGLE});		
	
	}
}
void ShootTask(void)
{
	CPU_INT08U  os_err;
	os_err = os_err;
	//将信号量清零
	OSSemSet(PeriodSem, 0, &os_err);
	while(1)
	{
		//等待信号量
		OSSemPend(PeriodSem, 0, &os_err);
		
		//将信号量清零避免因为使用延时导致的信号量积累
		OSSemSet(PeriodSem,0,&os_err);

			PathFollowing(0.1,1);
		
			ReadActualVel(CAN1,1);
			ReadActualVel(CAN1,2);
			ReadActualVel(CAN1,3);
			gRobot.wheelHB[0]++;
			gRobot.wheelHB[1]++;
			gRobot.wheelHB[2]++;

		if(gRobot.wheelHardfault[0]==1)
		{
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
						(uint8_t *)"Wheel1HardFualt!");			
		}
		if(gRobot.wheelHardfault[1]==1)
		{
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
						(uint8_t *)"Wheel2HardFualt!");			
		}
		if(gRobot.wheelHardfault[2]==1)
		{
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
						(uint8_t *)"Wheel3HardFualt!");			
		}
		if(gRobot.wheelHB[0]>50||gRobot.wheelHB[1]>50||gRobot.wheelHB[2]>50)
		{
				if(gRobot.wheelHB[0]>50)
				{
					USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
										(uint8_t *)"WHEEL1LOST!");
					gRobot.wheelHB[0]=51;
				}
				if(gRobot.wheelHB[1]>50)
				{
					USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
										(uint8_t *)"WHEEL2LOST!");
					gRobot.wheelHB[1]=51;
				}
				if(gRobot.wheelHB[2]>50)
				{
					USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
										(uint8_t *)"WHEEL3LOST!");
					gRobot.wheelHB[2]=51;
				}
		}
		

		if((gRobot.robotFlag&PPS_DATA_BAD)||(gRobot.robotFlag&OUTPUT_VEL_OVERFLOW)||(gRobot.robotFlag&POSSYSTEM_NOT_RECEIVE)||(gRobot.robotFlag&POSSYSTEM_NOT_RECEIVERIGHT))
		{
			int i=5;
			while(i--)
			{
				if(gRobot.robotFlag&PPS_DATA_BAD)
					USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,(uint8_t *)"BAD\r\n");
				else if(gRobot.robotFlag&OUTPUT_VEL_OVERFLOW)
					USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,(uint8_t *)"LOW\r\n");
				else if(gRobot.robotFlag&POSSYSTEM_NOT_RECEIVE)
					USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,(uint8_t *)"POSSYSTEM_NOT_RECEIVE\r\n");
				else if(gRobot.robotFlag&POSSYSTEM_NOT_RECEIVERIGHT)
					USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,(uint8_t *)"POSSYSTEM_NOT_RECEIVERIGHT\r\n");
			}

		}

		cpuUsage--;
	}
}





void SelfTestTask(void)
{
	CPU_INT08U  os_err;
	os_err = os_err;
	

	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"Self Check Start!!\r\n");
	
	
	//将信号量清零避免因为使用延时导致的信号量积累
	OSSemSet(PeriodSem,0,&os_err);

	while(1)
	{
		//等待信号量
		OSSemPend(PeriodSem, 0, &os_err);
		
		//将信号量清零避免因为使用延时导致的信号量积累
		OSSemSet(PeriodSem,0,&os_err);

	}
}
