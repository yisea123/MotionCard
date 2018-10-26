#include "Move.h"
#include "math.h"
#include "calculate.h"
#include "task.h"
#include "arm_math.h"
#include "ucos_ii.h"

#define ALPHA 45.0f 
extern Robot_t gRobot;
extern OS_EVENT *velCtrCmdSem;

static carVel_t targetVel = {0.0f};
static float targetOmega = 0.0f;

void SetTargetVel(float vel,float velDir,float omega)
{
	targetVel.carVel = vel;
	targetVel.velAngle = velDir;
	targetOmega = omega;
	OSSemPost(velCtrCmdSem);
	OSSemPost(velCtrCmdSem);
}

#define GAIN_COMPENSATION (1.173321f)

void VelControl(carVel_t actVel)
{
	
	carVel_t velErr = {0.0f};
	carVel_t outputVel = {0.0f};
	float velXErr , velYErr = 0.0f;
	float velXOutput , velYOutput = 0.0f;
	float KpX = 1.0f , KpY = 1.0f;
	float expVelX , expVelY = 0.0f;
	float actVelX , actVelY = 0.0f;
	
	expVelX = targetVel.carVel*cosf(targetVel.velAngle*CHANGE_TO_RADIAN);
	expVelY = targetVel.carVel*sinf(targetVel.velAngle*CHANGE_TO_RADIAN);
	
	actVelX = actVel.carVel*cosf(actVel.velAngle*CHANGE_TO_RADIAN);
	actVelY = actVel.carVel*sinf(actVel.velAngle*CHANGE_TO_RADIAN);
	
	velXErr = expVelX - actVelX;
	velYErr = expVelY - actVelY;
	
	if(gRobot.courtInfo==RED_COURT)
	{
		if(expVelX*actVelX>0.0f&&fabs(expVelX)>fabs(actVelX))
		{
			KpX = 0.9f;
		}
		else
		{
			KpX = 0.6f;
		}
		if(expVelY*actVelY>0.0f&&fabs(expVelY)>fabs(actVelY))
		{
			KpY = 0.9f;
		}
		else
		{
			KpY = 0.6f;
		}	
//		if(expVelX*actVelX>0.0f&&fabs(expVelX)>fabs(actVelX))
//		{
//			KpX = 1.5f;
//		}
//		else
//		{
//			KpX = 0.8f;
//		}

//		if(expVelY*actVelY>0.0f&&fabs(expVelY)>fabs(actVelY))
//		{
//			KpY = 1.5f;
//		}
//		else
//		{
//			KpY = 0.8f;
//		}
	}
	else if(gRobot.courtInfo==BLUE_COURT)
	{
		if(expVelX*actVelX>0.0f&&fabs(expVelX)>fabs(actVelX))
		{
			KpX = 0.9f;
		}
		else
		{
			KpX = 0.6f;
		}
		if(expVelY*actVelY>0.0f&&fabs(expVelY)>fabs(actVelY))
		{
			KpY = 0.9f;
		}
		else
		{
			KpY = 0.6f;
		}		
	}
	else
	{
		if(expVelX*actVelX>0.0f&&fabs(expVelX)>fabs(actVelX))
		{
			KpX = 0.9f;
		}
		else
		{
			KpX = 0.6f;
		}
		if(expVelY*actVelY>0.0f&&fabs(expVelY)>fabs(actVelY))
		{
			KpY = 0.9f;
		}
		else
		{
			KpY = 0.6f;
		}			
	}
	
	if(fabs(velXErr)>=1500.0f)
	{
		velXErr= velXErr / fabs(velXErr) * 1500.0f;
	}
	else if(fabs(velXErr)<=50.0f)
	{
		velXErr = 0.0f;
	}
	
	if(fabs(velYErr)>=1500.0f)
	{
		velYErr= velYErr / fabs(velYErr) * 1500.0f;
	}
	else if(fabs(velYErr)<=50.0f)
	{
		velYErr = 0.0f;
	}	

	velXErr*=KpX;
	velYErr*=KpY;
	
	velErr.carVel = sqrtf(velXErr*velXErr + velYErr*velYErr);
	
	velErr.velAngle = atan2f(velYErr,velXErr)*CHANGE_TO_ANGLE;
	
	if(velErr.carVel>=1500.0f)
	{
		velErr.carVel = 1500.0f;
	}
	
	if(velErr.carVel<50.0f)
	{
		velErr.carVel = 0.0f;
		velErr.velAngle = targetVel.velAngle;
	}
	
	velXOutput = velErr.carVel*cosf(velErr.velAngle*CHANGE_TO_RADIAN) + targetVel.carVel*cosf(targetVel.velAngle*CHANGE_TO_RADIAN);
	velYOutput = velErr.carVel*sinf(velErr.velAngle*CHANGE_TO_RADIAN) + targetVel.carVel*sinf(targetVel.velAngle*CHANGE_TO_RADIAN);
	
	outputVel.carVel = sqrtf(velXOutput*velXOutput + velYOutput*velYOutput);
	outputVel.velAngle = atan2f(velYOutput , velXOutput)*CHANGE_TO_ANGLE;
	
	if(outputVel.carVel>=GetVelMax())
	{
		outputVel.carVel = GetVelMax();
	}
	
	ThreeWheelVelControl(outputVel.carVel, outputVel.velAngle, targetOmega);
	
	
}

//三轮控制函数
//speed     单位mm/s
//direction 速度的方向  单位 度
//rotationVell 旋转速度 单位 度/s 
void ThreeWheelVelControl(float speed, float direction, float rotationVell)
{

	TriWheelVel_t vell;
	float Vx, Vy;
	float theta;
	float robotR = 0.0f;
	static uint8_t outputErrCounter[3] = {0};
	
	speed*=GAIN_COMPENSATION;
	
	rotationVell*=GAIN_COMPENSATION;
	
	if(speed>(GetVelMax()+1000.0f))
	{
		speed = GetVelMax();
		outputErrCounter[0]++;
		if(outputErrCounter[0]>10)
		{
			outputErrCounter[0] = 11;

			debugInfo.opsStatus|=0x10;
			USART_OUT(DEBUG_USART,"\r\n2\r\n");
		}
	}
	
	if(rotationVell>200.0f)
	{
		rotationVell = 200.0f;
		outputErrCounter[1]++;
		if(outputErrCounter[1]>10)
		{
			outputErrCounter[1]=11;
			USART_OUT(DEBUG_USART,"\r\n3\r\n");

			debugInfo.opsStatus|=0x20;	
		}
	}
	else if(rotationVell < -200.0f)
	{
		rotationVell = -200.0f;
		outputErrCounter[2]++;
		if(outputErrCounter[2]>10)
		{
			USART_OUT(DEBUG_USART,"\r\n4\r\n");
			outputErrCounter[2]=11;

			debugInfo.opsStatus|=0x40;	
		}
	}
	
	robotR = 100.0f;
	rotationVell = rotationVell / CHANGE_TO_ANGLE;
	Vx = speed * arm_cos_f32(direction * CHANGE_TO_RADIAN);
	Vy = speed * arm_sin_f32(direction * CHANGE_TO_RADIAN);

	gRobot.speed.aimX=Vx;
	gRobot.speed.aimY=Vy;
	
	if(isnan(speed))
		USART_OUT(USART2,"\tspnan\t");
	if(isnan(direction))
		USART_OUT(DEBUG_USART,"\tdinan\t");
	
	theta = GetAngleZ();
	
	vell.v1 = (float)(-arm_cos_f32((ALPHA + theta) * CHANGE_TO_RADIAN) * Vx - arm_sin_f32((ALPHA + theta) * CHANGE_TO_RADIAN) * Vy + rotationVell * robotR + 1 * 0);
	vell.v2 = (float)( arm_cos_f32((ALPHA - theta) * CHANGE_TO_RADIAN) * Vx - arm_sin_f32((ALPHA - theta) * CHANGE_TO_RADIAN) * Vy + rotationVell * robotR + 2 * 0);
	vell.v3 = (float)( arm_cos_f32((ALPHA + theta) * CHANGE_TO_RADIAN) * Vx + arm_sin_f32((ALPHA + theta) * CHANGE_TO_RADIAN) * Vy + rotationVell * robotR + 3 * 0);
	vell.v4 = (float)(-arm_cos_f32((ALPHA - theta) * CHANGE_TO_RADIAN) * Vx + arm_sin_f32((ALPHA - theta) * CHANGE_TO_RADIAN) * Vy + rotationVell * robotR + 4 * 0);
	
	debugInfo.expCarVel.carVel = speed;
	debugInfo.expCarVel.velAngle = direction;
	debugInfo.expAngularVel = rotationVell * CHANGE_TO_ANGLE;
	
	VelControlTriWheel(vell.v1,vell.v2,vell.v3,vell.v4);
	
}

void VelControlTriWheel(float v1,float v2,float v3,float v4)
{
		gRobot.wheelVelWant.wheelVel1Want = v1;
		gRobot.wheelVelWant.wheelVel2Want = v2;
		gRobot.wheelVelWant.wheelVel3Want = v3;
		gRobot.wheelVelWant.wheelVel4Want = v4;
	
		debugInfo.wheelExpVel.wheel1 = Vel2Pulse(v1);
		debugInfo.wheelExpVel.wheel2 = Vel2Pulse(v2);
		debugInfo.wheelExpVel.wheel3 = Vel2Pulse(v3);
		debugInfo.wheelExpVel.wheel4 = Vel2Pulse(v4);
	
	
		VelCrl(CAN1, 4, Vel2Pulse(v1));
		VelCrl(CAN1, 1, Vel2Pulse(v2));
		VelCrl(CAN1, 2, Vel2Pulse(v3));
		VelCrl(CAN1, 3, Vel2Pulse(v4));
}

/**
  * @brief  标准单位速度转化为脉冲速度
  * @param  vel:速度 mm/s
  * @retval velpulse:速度 脉冲/s
  */
float Vel2Pulse(float vel)
{
	float pulse = 0.0f;
	pulse = vel / (2.0f * PI * WHEELRADIUS) * STDPULSE * REDUCTION;
	return pulse;
}


/**
  * @brief  脉冲速度转化为标准单位速度
  * @param  pulse:速度 脉冲/s
  * @retval vel:速度 mm/s
  */
float Pulse2Vel(float pulse)
{
	float vel = 0.0f;
	vel = pulse * (2.0f * PI * WHEELRADIUS) / REDUCTION / STDPULSE;
	return vel;
}

//返回三个轮子轮速
//speed       单位mm/s
//direction 速度的方向  单位 度
//rotationVell 旋转速度 单位 度/s 
//posAngle 机器人的姿态  单位 度
TriWheelVel_t CaculateThreeWheelVel(float speed, float direction, float rotationVell,float angleZ)
{
	TriWheelVel_t vell;
	float Vx, Vy;
	float theta;
	float robotR = 0.0f;
	
	robotR = 100.0f;
	rotationVell = rotationVell / CHANGE_TO_ANGLE;
	Vx = speed * arm_cos_f32(direction * CHANGE_TO_RADIAN);
	Vy = speed * arm_sin_f32(direction * CHANGE_TO_RADIAN);

	theta = angleZ;

	vell.v1 = (float)(-arm_cos_f32((ALPHA + theta) * CHANGE_TO_RADIAN) * Vx - arm_sin_f32((ALPHA + theta) * CHANGE_TO_RADIAN) * Vy + rotationVell * robotR + 1 * 0);
	vell.v2 = (float)( arm_cos_f32((ALPHA - theta) * CHANGE_TO_RADIAN) * Vx - arm_sin_f32((ALPHA - theta) * CHANGE_TO_RADIAN) * Vy + rotationVell * robotR + 2 * 0);
	vell.v3 = (float)( arm_cos_f32((ALPHA + theta) * CHANGE_TO_RADIAN) * Vx + arm_sin_f32((ALPHA + theta) * CHANGE_TO_RADIAN) * Vy + rotationVell * robotR + 3 * 0);
	vell.v4 = (float)(-arm_cos_f32((ALPHA - theta) * CHANGE_TO_RADIAN) * Vx + arm_sin_f32((ALPHA - theta) * CHANGE_TO_RADIAN) * Vy + rotationVell * robotR + 4 * 0);

	return vell;
}



/*************************************************************************************
* @name        GetTrueVEll
* @param      brief       由三轮的转速计算实际的速度
* @param      wheelVell   三轮的速度
*
*
*************************************************************************************/
TriWheelVel2_t GetTrueVell(TriWheelVel_t wheelVell, float zAngle)
{
	float **B; 
	float **M;
	TriWheelVel2_t trueVell;

	B = CreateMemory(4,4);
	M = CreateMemory(4,4);

	float robotR = 100.0f;
	M[0][0] = -arm_cos_f32((45 + zAngle) * CHANGE_TO_RADIAN);
	M[0][1] = -arm_sin_f32((45 + zAngle) * CHANGE_TO_RADIAN);
	M[0][2] = robotR;
	M[0][3] = 1;
	M[1][0] =  arm_cos_f32((45 - zAngle) * CHANGE_TO_RADIAN);
	M[1][1] = -arm_sin_f32((45 - zAngle) * CHANGE_TO_RADIAN);
	M[1][2] = robotR;
	M[1][3] = 2;
	M[2][0] =  arm_cos_f32((45 + zAngle) * CHANGE_TO_RADIAN); 
	M[2][1] =  arm_sin_f32((45 + zAngle) * CHANGE_TO_RADIAN);
	M[2][2] = robotR;
	M[2][3] = 3;
	M[3][0] = -arm_cos_f32((45 - zAngle) * CHANGE_TO_RADIAN);
	M[3][1] =  arm_sin_f32((45 - zAngle) * CHANGE_TO_RADIAN);
	M[3][2] = robotR;
	M[3][3] = 4;

	Gauss(M, B, 4);

	float xVell = B[0][0] * wheelVell.v1 + B[0][1] * wheelVell.v2 + B[0][2] * wheelVell.v3 + B[0][3] * wheelVell.v4;
	float yVell = B[1][0] * wheelVell.v1 + B[1][1] * wheelVell.v2 + B[1][2] * wheelVell.v3 + B[1][3] * wheelVell.v4;
	trueVell.rotationVell = B[2][0] * wheelVell.v1 + B[2][1] * wheelVell.v2 + B[2][2] * wheelVell.v3 + B[2][3] * wheelVell.v4;

	trueVell.rotationVell *= (CHANGE_TO_ANGLE);
	trueVell.speed = sqrt(xVell * xVell + yVell * yVell);
	trueVell.direction = atan2f(yVell, xVell)*CHANGE_TO_ANGLE;
	FreeMemory(B, 4);
	FreeMemory(M, 4);
	return trueVell;
}










