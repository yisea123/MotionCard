#ifndef __TASK_H
#define __TASK_H
#include "stdint.h"
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
#include "arm_math.h"
#include "calculate.h"
/**************#define area**********/

//减速比
#define REDUCTION (91.0f/6.0f)
//车轮半径 单位:mm
#define WHEELRADIUS 63.5f
//每圈脉冲数
#define STDPULSE 2000.0f

//#define SUMMER

#define READ_WHEEL_VEL (1)


#define DUCKED_FAN_RATIO (0.70f)

#define DUCKED_FAN_LOW_LEVEL (0.70f)

//常量定义
#define ANGLE_TO_RAD(angle) 	((angle)/180.f*PI)
#define RAD_TO_ANGLE(rad) 		((rad)/PI*180.f)

#define DISX_GYRO2CENTER (-164.0f)
#define DISY_GYRO2CENTER (-316.0f)

#define POS_Y_SHIFT (-40.0f)

#define ROBOT_LENGTH_IN_X (956.0f)
#define ROBOT_LENGTH_IN_Y (946.0f)

//TZ1抱死速度条件
#define TZ_1_STOP_VEL (400.0f)
//TZ1投球速度条件
#define TZ_1_SHOOT_VEL (500.0f)
//TZ1投球姿态条件
#define TZ_1_SHOOT_POSTURE (1.0f)

#define TZ_1_SHOOT_POSERR (50.0f)

//TZ2投球速度条件
#define TZ_2_SHOOT_VEL (100.0f)
//TZ2投球姿态条件
#define TZ_2_SHOOT_POSTURE (1.0f)
//TZ2投球位置误差条件
#define TZ_2_SHOOT_POSERR (50.0f)

//TZ3投球速度条件
#define TZ_3_SHOOT_VEL (300.0f)
//TZ3投球位置误差条件
#define TZ_3_SHOOT_POSERR (50.0f)

//控制模式解释
#define CORRECT_X_PERMIT								0X01u
#define CORRECT_Y_PERMIT								0X02u
#define CORRECT_X_PERMIT_FORCE					0X04u
#define CORRECT_Y_PERMIT_FORCE					0X08u
#define POS_1_SYSTEM_READY							0X10u
#define MAIN_BOARD_READY								0X20u
#define ENTER_SELF_CHECK								0x40u
#define DEBUG_SEND_MODE									0x80u
#define FIRST_BALL_GET									0x100u
#define TALK_TO_POS_1_SUCCESS						0x200u
#define IS_CORRECT_POS_X 								0x400u
#define PPS_1_HARDFAULT		 							0x800u
#define PPS_NAN		 											0x1000u
#define PPS_GYRO_BAD		 								0x2000u
#define PPS_DATA_BAD									 (0x4000u)
#define OUTPUT_VEL_OVERFLOW 					 (0x8000u)
#define LASER_Y_USE 									 (0x10000u)
#define POSSYSTEM_NOT_RECEIVE					 (0x20000u)
#define POSSYSTEM_NOT_RECEIVERIGHT		 (0x40000u)
#define TALK_TO_POS_2_SUCCESS					 (0x80000u)
#define POS_2_SYSTEM_READY						 (0x100000u)
#define POS_SYSTEM_START							 (0x200000u)
#define PPS_1_BAD											 (0x400000u)
#define PPS_2_HARDFAULT		 							0x800000u
#define NO_ROADBLOCK (0)
#define HAVE_ROADBLOCK (1)

#define BLUE_COURT (1)
#define RED_COURT (2)

/**************typedef area**********/
//用来表示四个电机相关数据结构体类型
typedef struct
{
	int32_t wheel1;
	int32_t wheel2;
	int32_t wheel3;
}wheel_t;

//机器人位姿结构体类型
typedef struct
{
	float posX;
	float posY;
	float angle;
}position_t;

//机器人速度大小和方向结构体类型
typedef struct
{
	float carVel;
	float velAngle;
}carVel_t;

//调试用数据结构体类型
typedef struct
{
	wheel_t wheelExpVel;
	wheel_t wheelActVel;
	carVel_t expCarVel;
	carVel_t planVel;
	position_t robotPos;
	position_t expPos;
	position_t laserPos;
	position_t gyroPos;
	float angularVel;
	float lineAngularVel;
	float lineAngularErr;
	float sectionTime;
	float expAngularVel;
	float robotJourney;
	float exactLength;
	float totalLength;
	float viewLength;
	uint8_t opsStatus;
	float testDebugInfo[2];
	float ppsX,ppsY,ppsAngle;
}debugInfo_t;

typedef struct
{
	int8_t isGetBall;
	uint8_t isStartMove;
	uint8_t isTestPara;
	uint8_t ifMotorOn;
	uint8_t isReset;
	uint8_t isWashWheel;
	uint8_t resetMode;
	uint8_t isShow;
}mainMsg_t;

typedef struct{
	float x;
	float y;
	float speedX;
	float speedY;
}DirectionPoint_t;
	
typedef struct{
	
	struct{
		float angle;
		float gyroX;
		float gyroY;
		float laserX;
		float laserY;
		float realX;
		float realY;
	  float gyroError_X;
	  float gyroError_Y;
		int code[2];
		float wz;
	}walk_t;
	
	struct{
		int receive;
		int receiveRight;
		int notReceiveNum;
		int notReceiveRightNum;
	}PosSysError_t;
	
	struct{
		float angle;
		float x;
		float y;
	}walk_tOld;
	
	TriWheelVel_t wheelVel;
	
	struct{
		int wheelVel1Want;
		int wheelVel2Want;
		int wheelVel3Want;
	}wheelVelWant;
	
	struct{
		int wheelPosition1;
		int wheelPosition2;
		int wheelPosition3;
	}wheelPosition;
	
	int posSystemReady;
	
	/*激光的初始值*/
	float laserInit;
	
	float laserValue;
	
	struct{
		float aimX;
		float aimY;
		float realX;
		float realY;
		float vel;
	} speed;
	
	struct{
		double angularV1;
		double angularV2;
		struct{
			uint16_t code1;
			uint16_t code2;
		} encoder1;
		struct{
			uint16_t code1;
			uint16_t code2;
		} encoder2;
		int encoderTimes[2];
	}PPS_baseInfo;
	
	DirectionPoint_t startPoint;

	DirectionPoint_t endPoint;
	
	uint32_t robotFlag;
	uint16_t wheelHB[3];
	uint8_t wheelHardfault[3];
	uint8_t courtInfo;
}Robot_t;

extern debugInfo_t debugInfo;
extern uint8_t selfTestCmd;
extern mainMsg_t mainMsg;
extern carVel_t targetCarVel;
extern Robot_t gRobot;

int SteadyJudgeSolid(int mode);

int steadyJudge(void);

int steadyJudge2(void);

int steadyJudgeForSelf(void);

#endif

