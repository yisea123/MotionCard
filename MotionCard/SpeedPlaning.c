#include "SpeedPlaning.h"
#include <stdlib.h>
#include "MotionCard.h"
#include "ringBuffer.h"
#include "math.h"
#include "Move.h"
#include "usart.h"
#include "calculate.h"
#include "dma.h"
#include "timer.h"

float GetAxisAccMax(void)
{
		return (6000.0f);
}

float GetAccMax(void)
{  
	return GetAxisAccMax()*1.414214f; 
}

float GetVelMax(void)
{
	return 8265.3f;
}


//通过配置的轮子最大加速度进行降速
//适当比例的降速，算完后记得把最新速度数据放在ringbuffer里
//wheelOne 一号轮速度数组首地址
//wheelTwo 	二号轮速度数组首地址
//wheelThree 三号轮速度数组首地址
void DynamicalAjusting(float* wheelOne, float* wheelTwo, float* wheelThree)
{
	float time = 0.0f;
	int n = GetCount();
	float tempAcc = 0.0f;

	int8_t velDirection  = 0;
	
	//每次加速度降低至上次的百分值
	float percent = 0.9;

	//先正向削减速度
	for (int i = 2; i < n + 1; i++)
	{

		//粗略计算每两示教点之间的运动的时间
		time = (GetRingBufferPointLen(i) - GetRingBufferPointLen(i - 1)) / (GetRingBufferPointVell(i) + GetRingBufferPointVell(i - 1)) * 2;
		//轮1
		//只处理速度同向的情况
		velDirection = wheelOne[i - 1] - wheelOne[i - 2] >= 0 ? 1 : -1;

		tempAcc = fabs(wheelOne[i - 1] - wheelOne[i - 2]) / time;

		if (tempAcc > GetAxisAccMax())
		{
			//每次削减0.05的加速度
			wheelOne[i - 1] =  wheelOne[i - 2] + velDirection*tempAcc*percent * time;
		}
		//轮2
		velDirection = wheelTwo[i - 1] - wheelTwo[i - 2] >= 0 ? 1 : -1;

		tempAcc = fabs(wheelTwo[i - 1] - wheelTwo[i - 2]) / time;

		if (tempAcc > GetAxisAccMax())
		{
			//每次削减0.05的加速度
			wheelTwo[i - 1] =  wheelTwo[i - 2] + velDirection*tempAcc*percent * time;
		}

		//轮3
		velDirection = wheelThree[i - 1] - wheelThree[i - 2] >= 0 ? 1 : -1;

		tempAcc = fabs(wheelThree[i - 1] - wheelThree[i - 2]) / time;

		if (tempAcc > GetAxisAccMax())
		{
			//每次削减0.05的加速度
			wheelThree[i - 1] =  wheelThree[i - 2] + velDirection*tempAcc*percent * time;
		}		

	}

	for (int i = 0; i < n - 1; i++)
	{
		TriWheelVel_t tempTrueVell;
		TriWheelVel2_t tempVel2;
		tempTrueVell.v1 = wheelOne[i];
		tempTrueVell.v2 = wheelTwo[i];
		tempTrueVell.v3 = wheelThree[i];
		
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
		(uint8_t *)"tempTrueVell.v1:\t%d\ttempTrueVell.v2:\t%d\ttempTrueVell.v3:\t%d\tGetRingBufferPointPoseAngle(i+1):%d\r\n",\
			(int)tempTrueVell.v1,(int)tempTrueVell.v2,(int)tempTrueVell.v3,(int)GetRingBufferPointPoseAngle(i+1));
		Delay_ms(2);
		tempVel2 = GetTrueVell(tempTrueVell,GetRingBufferPointPoseAngle(i+1));
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
		(uint8_t *)"tempVel2.speed:\t%d\r\n",(int)tempVel2.speed);
		SetRingBufferPointVell(i + 1, tempVel2.speed);
	}
}










//通过ringBuffer里的数据计算每一点处三个轮子的速度
//目的更新wheelOne wheelTwo wheelThree这三个数组里的三轮速度，便于下一次的速度削减
//wheelOne 一号轮速度数组首地址
//wheelTwo 	二号轮速度数组首地址
//wheelThree 三号轮速度数组首地址
void CalculateThreeWheelVell(float* wheelOne,float* wheelTwo,float* wheelThree)
{
	//分解到三个轮对全局速度进行规划
	TriWheelVel_t threeVell;
	float n = GetCount();


	for (int i = 2; i < n + 1; i++)
	{
		float angErr = GetRingBufferPointPoseAngle(i) - GetRingBufferPointPoseAngle(i - 1);
		angErr = angErr > 180 ? angErr - 360 : angErr;
		angErr = angErr < -180 ? 360 + angErr : angErr;

		float time = 0.0f;

		time = (GetRingBufferPointLen(i) - GetRingBufferPointLen(i - 1)) / (GetRingBufferPointVell(i) + GetRingBufferPointVell(i - 1)) * 2;

		float rotationVell = angErr / time;


		threeVell = CaculateThreeWheelVel(GetRingBufferPointVell(i), GetRingBufferPointAngle(i), rotationVell, GetRingBufferPointPoseAngle(i));

//			
//		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//		(uint8_t *)"directionK %d direction %d\r\n",(int)(directionK * 100),(int)direction);

		wheelOne[i - 1] = threeVell.v1;

		wheelTwo[i - 1] = threeVell.v2;

		wheelThree[i - 1] = threeVell.v3;
	}
	wheelOne[0] = wheelOne[1];
	wheelTwo[0] = wheelTwo[1];
	wheelThree[0] = wheelThree[1];
}





//通过降低合速度保证某轮的速度要求
//vellCar 降速前的前进合速度 单位 mm/s
//orientation 速度朝向 单位 度
//rotationalVell 旋转速度 单位 度每秒
//wheelNum  所降速的轮号
 // targetWheelVell   所降速的 标/ 回降低的合速度
float DecreseVellByOneWheel(float vellCar, float orientation, float rotationalVell,float zAngle, int wheelNum, float targetWheelVell)
{
	TriWheelVel_t vell;
	int i;
	switch (wheelNum)
	{
		case 1:
			//每次合速度乘0.9,直到满足一号轮速度降低至目标速度。对于一些不能满足的，循环10次后自动退出
			for (i = 0; i < 10; i++)
			{

				vellCar *= 0.9f;
				vell = CaculateThreeWheelVel(vellCar, orientation, rotationalVell, zAngle);
				if (fabs(vell.v1) < fabs(targetWheelVell))
				{
					break;
				}
			}
		break;

		case 2:
			for (i = 0; i < 10; i++)
			{

				vellCar *= 0.9f;
				vell = CaculateThreeWheelVel(vellCar, orientation, rotationalVell, zAngle);
				if (fabs(vell.v2) < fabs(targetWheelVell))
				{
					break;
				}
			}
		break;

		case 3:
			for (i = 0; i < 10; i++)
			{

				vellCar *= 0.9f;
				vell = CaculateThreeWheelVel(vellCar, orientation, rotationalVell, zAngle);
				if (fabs(vell.v3) < fabs(targetWheelVell))
				{
					break;
				}
			}
			break;
	}
	return vellCar;
}

float GetAccLimit(float direction)
{
	float directionK = 0.0f;
	float accLimitX = 0.0f , accLimitY = 0.0f;
	
	directionK = tan((double)(direction*CHANGE_TO_RADIAN));
	
//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//		(uint8_t *)"directionK %d direction %d\r\n",(int)(directionK * 100),(int)direction);

	if(fabs(directionK)>2000)
	{
		return GetAxisAccMax();
	}
	else
	{
		if(direction>=0.0f && direction<60.0f)
		{
			accLimitX = 2 * sqrt(3) * GetAxisAccMax()/(directionK + sqrt(3));
		}
		else if(direction>=60.0f && direction<120.0f)
		{
			accLimitX = sqrt(3) * GetAxisAccMax()/directionK;
		}
		else if(direction>=120.0f && direction<=180.0f)
		{
			accLimitX = 2 * sqrt(3) * GetAxisAccMax()/(directionK - sqrt(3));
		}
		else if(direction>=-60.0f && direction<0.0f)
		{
			accLimitX = -2 * sqrt(3) * GetAxisAccMax()/(directionK - sqrt(3));
		}
		else if(direction>=60.0f && direction<120.0f)
		{
			accLimitX = -sqrt(3) * GetAxisAccMax()/directionK;
		}
		else if(direction>=-180.0f && direction<-120.0f)
		{
			accLimitX = -2 * sqrt(3) * GetAxisAccMax()/(directionK + sqrt(3));
		}
		
	}
	
	accLimitY = directionK * accLimitX;
	
	return sqrtf(accLimitX * accLimitX + accLimitY * accLimitY);
	
}

float GetVelLimit(float direction)
{
	float velLimitOnAccAxisX = 0.0f , velLimitOnAccAxisY = 0.0f , velLimitOnAccAxisZ = 0.0f;
	int velLimitSign = 0;
	float velLimit = 0.0f;
	
	velLimitOnAccAxisX = CalculateVectorProject((vector_t){GetAxisAccMax()  , direction} , (vector_t){GetAxisAccMax(),0.0f});
	velLimitOnAccAxisY = CalculateVectorProject((vector_t){GetAxisAccMax()  , direction} , (vector_t){GetAxisAccMax(),60.0f});
	velLimitOnAccAxisZ = CalculateVectorProject((vector_t){GetAxisAccMax()  , direction} , (vector_t){GetAxisAccMax(),120.0f});

	if(velLimitOnAccAxisX >= velLimitOnAccAxisY)
	{
		velLimitSign = velLimitOnAccAxisX > velLimitOnAccAxisZ ? 1 : 2;
	}
	else
	{
		velLimitSign = velLimitOnAccAxisY > velLimitOnAccAxisZ ? 2 : 3;
	}
	
	switch(velLimitSign)
	{
		case 1 :
		{			
			velLimit = CalculateVectorFromProject(velLimitOnAccAxisX , direction , 0.0f).module;
			break;
		}
		case 2 :
		{			
			velLimit = CalculateVectorFromProject(velLimitOnAccAxisY , direction , 60.0f).module;
			break;
		}
		case 3 :
		{			
			velLimit = CalculateVectorFromProject(velLimitOnAccAxisZ , direction , 120.0f).module;
			break;
		}	

	}
	return velLimit;
}

float CalculateAccT(float accN , float accNDirection , float accTDirection)
{
	float accNOnAccAxisX = 0.0f , accNOnAccAxisY = 0.0f , accNOnAccAxisZ = 0.0f;
	float accTLimitOnAccAxisX = 0.0f , accTLimitOnAccAxisY = 0.0f ,accTLimitOnAccAxisZ = 0.0f;
	float accTFromX = 0.0f , accTFromY = 0.0f , accTFromZ = 0.0f;
	float accTLimit = 0.0f;
	
	accNOnAccAxisX = CalculateVectorProject((vector_t){accN, accNDirection} , (vector_t){GetAxisAccMax(),0.0f});
	accNOnAccAxisY = CalculateVectorProject((vector_t){accN, accNDirection} , (vector_t){GetAxisAccMax(),60.0f});
	accNOnAccAxisZ = CalculateVectorProject((vector_t){accN, accNDirection} , (vector_t){GetAxisAccMax(),120.0f});
	
	if(accTDirection>=-90.0f&&accTDirection<=90.0f)
	{
		accTLimitOnAccAxisX = GetAxisAccMax() - accNOnAccAxisX;		
	}
	else
	{
		accTLimitOnAccAxisX = -GetAxisAccMax() - accNOnAccAxisX;				
	}
	
	if(accTDirection>=-30.0f&&accTDirection<=150.0f)
	{
		accTLimitOnAccAxisY = GetAxisAccMax() - accNOnAccAxisY;		
	}
	else
	{
		accTLimitOnAccAxisY = -GetAxisAccMax() - accNOnAccAxisY;		
	}
	
	if(accTDirection>=30.0f||accTDirection<=-150.0f)
	{
		accTLimitOnAccAxisZ = GetAxisAccMax() - accNOnAccAxisZ;		
	}
	else
	{
		accTLimitOnAccAxisZ = -GetAxisAccMax() - accNOnAccAxisZ;		
	}
	
	accTFromX = CalculateVectorFromProject(accTLimitOnAccAxisX , accTDirection , 0.0f).module;
	
	accTFromY = CalculateVectorFromProject(accTLimitOnAccAxisY , accTDirection , 60.0f).module;
	
	accTFromZ = CalculateVectorFromProject(accTLimitOnAccAxisZ , accTDirection , 120.0f).module;
	
	accTLimit = ((fabs(accTFromX)>=fabs(accTFromY)?fabs(accTFromY):fabs(accTFromX))>=fabs(accTFromZ)?fabs(accTFromZ):(fabs(accTFromX)>=fabs(accTFromY)?fabs(accTFromY):fabs(accTFromX)));
	
	if(accTDirection==0.0f||accTDirection==60.0f||accTDirection==120.0f||accTDirection==180.0f||accTDirection==-60.0f||accTDirection==-120.0f)
	{
		accTLimit = GetAxisAccMax();
	}
	
	return accTLimit;
	
}

#define MIN_VELL 70
//速度规划函数
void SpeedPlaning(velPlan_t velPlan)
{
	float* vell = NULL;
	float* curvature = NULL;
	float* wheelOne = NULL;
	float* wheelTwo = NULL;
	float* wheelThree = NULL;
	float* curveNormalDirection = NULL;


	int n = GetCount();

	vell = (float *)malloc(n*sizeof(float));
	curvature = (float *)malloc(n*sizeof(float));
	wheelOne = (float *)malloc(n*sizeof(float));
	wheelTwo = (float *)malloc(n*sizeof(float));
	wheelThree = (float *)malloc(n*sizeof(float));
	curveNormalDirection = (float *)malloc(n*sizeof(float));


	//记录每一个点的曲率半径并计算每个点的法向方向
	for (int i = 0; i < n; i++)
	{
		curvature[i] = GetRingBufferAverCurvature(i + 1);
		curveNormalDirection[i] = GetRingBufferPointAngle(i + 1) + 90.0f;
		if(curveNormalDirection[i]>180.0f)
		{
			curveNormalDirection[i]-=360.0f;
		}
		else if(curveNormalDirection[i]<-180.0f)
		{
			curveNormalDirection[i]+=360.0f;
		}
	}

	curvature[0] = curvature[1];
	curvature[n - 1] = curvature[n - 2];


	//通过曲率半径计算该段能满足的最大速度
	for (int i = 0; i < n; i++)
	{
		Delay_ms(5);
		vell[i] = sqrt((1.0f * GetAccLimit(curveNormalDirection[i])) * curvature[i]);
		
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
		(uint8_t *)"NormalDirection:\t%d\t",(int)curveNormalDirection[i]);
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
		(uint8_t *)"curvature[%d]\t:\t%d\tvell[%d]\t:\t%d\r\n",\
		i,(int)curvature[i],i,(int)(vell[i]));
		
		if(vell[i]>GetVelLimit(GetRingBufferPointAngle(i + 1)))
		{
			vell[i] = GetVelLimit(GetRingBufferPointAngle(i + 1));
		}
	}

	//将初始速度和终止速度改为给定值
	vell[n-1] = velPlan.endVel;
	vell[0]=velPlan.startVel;
	
	if(vell[0]<150.0f)
	{
		vell[0] = 150.0f;
	}
	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
	(uint8_t *)"init speed:\r\n");
	for(int i = 0;i<n;i++)
	{
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"%d\t%d\t",(int)curvature[i],(int)GetRingBufferPointAngle(i+1));
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"%d\t",(int)curveNormalDirection[i]);
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"%d\r\n",(int)vell[i]);
		Delay_ms(5);
	}
	
	//临时计算速度变量
	float tempVell = 0.0f;
	//临时计算该段轨迹结束的最大速度
	float tempVirtualVel = 0.0f;
	//估计的该段轨迹平均速度
	float tempTargetVel = 0.0f;
	//法向加速度
	float accN = 0.0f;
	//切向加速度
	float accT = 0.0f;
	//法向加速度方向
	float accNAngle = 0.0f;
	//估计每段轨迹的速度方向
	float tempAngle = 0.0f;
	//速度变化方向
	float angleChange = 0.0f;
	//留有的加速度余量
	#define ACC_REDUNDANT (0.05f)
	//通过v2^2 - v1^2 = 2*a*s对速度再次规划
	for (int i = 0; i < n - 1; i++)
	{
		if (vell[i + 1] > vell[i])
		{
			//计算两点间的角度平均值
			tempAngle = (GetRingBufferPointAngle(i + 1) + GetRingBufferPointAngle(i + 2));
			//计算速度方向的变化
			angleChange = GetRingBufferPointAngle(i + 2) - GetRingBufferPointAngle(i + 1);
			
			if(fabs(angleChange)>180.0f)
			{
				tempAngle+=360.0f;
			}
			
			tempAngle/=2.0f;
			
			if(tempAngle>180.0f)
			{
				tempAngle-=360.0f;
			}
			else if(tempAngle<-180.0f)
			{
				tempAngle+=360.0f;
			}
			
			if(angleChange>180.0f)
			{
				angleChange-=360.0f;
			}
			else if(angleChange<-180.0f)
			{
				angleChange+=360.0f;
			}
			
			//根据速度方向变化计算切向加速度方向
			if(angleChange>=0.0f)
			{
				accNAngle = tempAngle + 90.0f;			
			}
			else
			{
				accNAngle = tempAngle - 90.0f;
			}
			
			if(accNAngle>180.0f)
			{
				accNAngle-=360.0f;
			}
			else if(accNAngle<-180.0f)
			{
				accNAngle+=360.0f;
			}
			//估算该段轨迹结束时的速度替代之前的最大速度
			tempVirtualVel = sqrt(2 * GetAccLimit(tempAngle) * (GetRingBufferPointLen(i + 2) - GetRingBufferPointLen(i + 1)) + vell[i] * vell[i]);
			//该段轨迹结束的最大速度
			tempTargetVel = vell[i+1];
			//如果估算速度小于最大速度，用估算速度来估算法向加速度
			if(tempVirtualVel<vell[i+1])
			{
				tempTargetVel = tempVirtualVel;
			}
			//计算法向加速度
			accN = pow(tempTargetVel + vell[i],2)/(2.0f * (curvature[i] + curvature[i + 1]));
			//法向加速度较小时忽略不计
			if(accN<=100.0f)
			{
				accN = 0.0f;
			}
			//计算切向加速度
			accT = CalculateAccT(accN , accNAngle , tempAngle);
			//根据切向加速度估计该段轨迹结束时的速度
			tempVell = sqrt(2 * ((1.0f - ACC_REDUNDANT) * accT) * (GetRingBufferPointLen(i + 2) - GetRingBufferPointLen(i + 1)) + vell[i] * vell[i]);
			if (tempVell < vell[i + 1])
			{
				vell[i + 1] = tempVell;
			}
		}
	}
	
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
		(uint8_t *)"middle 1 speed:\r\n");
	for(int i = 0;i<n;i++)
	{
//		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//			(uint8_t *)"%d\t%d\t",(int)curvature[i],(int)GetRingBufferPointAngle(i+1));
//		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//			(uint8_t *)"%d\t",(int)curveNormalDirection[i]);
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"%d\r\n",(int)vell[i]);
		Delay_ms(5);
	}
//	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//	(uint8_t *)"UP1 speed:\r\n");
//	for(int i = 0;i<n;i++)
//	{
//		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//			(uint8_t *)"%d\r\n",(int)vell[i]);
//		Delay_ms(5);
//	}	
	for (int i = n - 1; i > 0; i--)
	{
		if (vell[i - 1] > vell[i])
		{		
			//计算两点间的角度平均值
			tempAngle = (GetRingBufferPointAngle(i) + GetRingBufferPointAngle(i + 1));
			//计算速度方向的变化
			angleChange = GetRingBufferPointAngle(i + 1) - GetRingBufferPointAngle(i);
			
			if(fabs(angleChange)>180.0f)
			{
				tempAngle+=360.0f;
			}
			
			tempAngle/=2.0f;
			
			if(tempAngle>180.0f)
			{
				tempAngle-=360.0f;
			}
			else if(tempAngle<-180.0f)
			{
				tempAngle+=360.0f;
			}
			
			if(angleChange>180.0f)
			{
				angleChange-=360.0f;
			}
			else if(angleChange<-180.0f)
			{
				angleChange+=360.0f;
			}
			
			//根据速度方向变化计算切向加速度方向
			if(angleChange>=0.0f)
			{
				accNAngle = tempAngle + 90.0f;			
			}
			else
			{
				accNAngle = tempAngle - 90.0f;
			}
			
			if(accNAngle>180.0f)
			{
				accNAngle-=360.0f;
			}
			else if(accNAngle<-180.0f)
			{
				accNAngle+=360.0f;
			}
			//估算该段轨迹结束时的速度替代之前的最大速度
			tempVirtualVel = sqrt(2 * GetAccLimit(tempAngle) * (GetRingBufferPointLen(i + 1) - GetRingBufferPointLen(i)) + vell[i] * vell[i]);
			//该段轨迹结束的最大速度
			tempTargetVel = vell[i - 1];
			//如果估算速度小于最大速度，用估算速度来估算法向加速度
			if(tempVirtualVel<vell[i - 1])
			{
				tempTargetVel = tempVirtualVel;
			}
			//计算法向加速度
			accN = pow(tempTargetVel + vell[i],2)/(2.0f * (curvature[i] + curvature[i - 1]));
			//法向加速度较小时忽略不计
			if(accN<=100.0f)
			{
				accN = 0.0f;
			}
			//计算切向加速度
			accT = CalculateAccT(accN , accNAngle , tempAngle);

			//根据切向加速度估计该段轨迹结束时的速度
			tempVell = sqrt(2 * ((1.0f - ACC_REDUNDANT) * accT) * (GetRingBufferPointLen(i + 1) - GetRingBufferPointLen(i)) + vell[i] * vell[i]);
			if (tempVell < vell[i - 1])
			{
				vell[i - 1] = tempVell;
			}
		}
	}

	USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
		(uint8_t *)"middle 2 speed:\r\n");
	for(int i = 0;i<n;i++)
	{
//		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//			(uint8_t *)"%d\t%d\t",(int)curvature[i],(int)GetRingBufferPointAngle(i+1));
//		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
//			(uint8_t *)"%d\t",(int)curveNormalDirection[i]);
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"%d\r\n",(int)vell[i]);
		Delay_ms(5);
	}
	//将暂时规划的速度放入环形数组里
	for (int i = 0; i < n; i++)
	{
		SetRingBufferPointVell(i + 1, vell[i]);
	}

	//计算此时三个轮的速度
	CalculateThreeWheelVell(wheelOne, wheelTwo, wheelThree);

	for (int i = 1; i < n; i++)
	{
		USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
		(uint8_t *)"whe elOne:%d\twheelTwo:%d\twheelThree:%d\r\n",(int)wheelOne[i],(int)wheelTwo[i],(int)wheelThree[i]);
		Delay_ms(2);	
	}
			
	//动态的对速度进行平衡
	while (1)
	{
		int ipoint = 0;

		for (ipoint = 3; ipoint < n; ipoint++)
		{
			float time = 0.0f;

			float lll;
			float vvv;
			lll = (GetRingBufferPointLen(ipoint) - GetRingBufferPointLen(ipoint - 1));
			vvv = (GetRingBufferPointVell(ipoint) + GetRingBufferPointVell(ipoint - 1)) / 2;
			time = lll / vvv;


			float a1, a2, a3;
			//如果判断某一个轮子加速度大于最大加速度时，进行调节

			a1 = (wheelOne[ipoint - 1] - wheelOne[ipoint - 2]) / time;
			a2 = (wheelTwo[ipoint - 1] - wheelTwo[ipoint - 2]) / time;
			a3 = (wheelThree[ipoint - 1] - wheelThree[ipoint - 2]) / time;
			if ( ((a1 > GetAxisAccMax()) && (wheelOne[ipoint - 1] * wheelOne[ipoint - 2] > 0))\
				|| ((a2 > GetAxisAccMax()) && (wheelTwo[ipoint - 1] * wheelTwo[ipoint - 2] > 0))\
				|| ((a3 > GetAxisAccMax()) && (wheelThree[ipoint - 1] * wheelThree[ipoint - 2] > 0)))
			{
				//平衡法规划速度
				DynamicalAjusting(wheelOne, wheelTwo, wheelThree);
				break;
			}
		}

		
		if (ipoint == n)
		{

			for (int i = 1; i < n; i++)
			{
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"wheelOne:%d\twheelTwo:%d\twheelThree:%d\t",(int)wheelOne[i],(int)wheelTwo[i],(int)wheelThree[i]);
				Delay_ms(5);
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"%d\r\n",(int)GetRingBufferPointVell(i));
				
			}
	
			for (int i = 1; i < n; i++)
			{
				TriWheelVel_t tempTrueVell;
				tempTrueVell.v1 = wheelOne[i];
				tempTrueVell.v2 = wheelTwo[i];
				tempTrueVell.v3 = wheelThree[i];
				float vellCar1, vellCar2, vellCar3, vellCar;
				float angErr = GetRingBufferPointPoseAngle(i + 2) - GetRingBufferPointPoseAngle(i + 1);
				angErr = angErr > 180 ? angErr - 360 : angErr;
				angErr = angErr < -180 ? 360 + angErr : angErr;
				//粗略计算每两示教点之间的运动的时间
				float time = (GetRingBufferPointLen(i + 2) - GetRingBufferPointLen(i + 1)) / (GetRingBufferPointVell(i + 2) + GetRingBufferPointVell(i + 1)) * 2;


				vellCar1 = DecreseVellByOneWheel(GetRingBufferPointVell(i + 1), GetRingBufferPointAngle(i + 1), angErr / time, GetRingBufferPointPoseAngle(i + 1), 1, tempTrueVell.v1);

				vellCar2 = DecreseVellByOneWheel(GetRingBufferPointVell(i + 1), GetRingBufferPointAngle(i + 1), angErr / time, GetRingBufferPointPoseAngle(i + 1), 2, tempTrueVell.v2);

				vellCar3 = DecreseVellByOneWheel(GetRingBufferPointVell(i + 1), GetRingBufferPointAngle(i + 1), angErr / time, GetRingBufferPointPoseAngle(i + 1), 3, tempTrueVell.v3);

				
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
					(uint8_t *)"RingBufferPointVell:\t%d\tvellCar1:%d\tvellCar2:%d\tvellCar3:%d\t\r\n",(int)GetRingBufferPointVell(i+1),(int)vellCar1,(int)vellCar2,(int)vellCar3);

				if (fabs(vellCar1) >= fabs(vellCar2) && fabs(vellCar1) >= fabs(vellCar3))
				{
					vellCar = vellCar1;
					//将计算的最新合速度放入缓存池中
					SetRingBufferPointVell(i + 1, vellCar);
				}
				else if (fabs(vellCar2) >= fabs(vellCar1) && fabs(vellCar2) >= fabs(vellCar3))
				{
					vellCar = vellCar2;
					//将计算的最新合速度放入缓存池中
					SetRingBufferPointVell(i + 1, vellCar);
				}
				else if (fabs(vellCar3) >= fabs(vellCar1) && fabs(vellCar3) >= fabs(vellCar2))
				{
					vellCar = vellCar3;
					//将计算的最新合速度放入缓存池中
					SetRingBufferPointVell(i + 1, vellCar);
				}
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"vellCar:\t%d\r\n",(int)vellCar);
			}
		}


		if (ipoint == n)
		{
			//轨迹起始速度较小时进行放大，避免输出速度较小时不会动
			if(GetRingBufferPointVell(1)<100.0f)
			{
				SetRingBufferPointVell(1, 100);
			}
//			SetRingBufferPointVell(1, 100);
//			SetRingBufferPointVell(n, 100);
			for (int i = 0; i < n - 1; i++)
			{
				if (GetRingBufferPointVell(i + 2) > GetRingBufferPointVell(i + 1))
				{
					
					//计算两点间的角度平均值
					tempAngle = (GetRingBufferPointAngle(i + 1) + GetRingBufferPointAngle(i + 2));
					//计算速度方向的变化
					angleChange = GetRingBufferPointAngle(i + 2) - GetRingBufferPointAngle(i + 1);

					if(fabs(angleChange)>180.0f)
					{
						tempAngle+=360.0f;
					}

					tempAngle/=2.0f;

					if(tempAngle>180.0f)
					{
						tempAngle-=360.0f;
					}
					else if(tempAngle<-180.0f)
					{
						tempAngle+=360.0f;
					}

					if(angleChange>180.0f)
					{
						angleChange-=360.0f;
					}
					else if(angleChange<-180.0f)
					{
						angleChange+=360.0f;
					}

					//根据速度方向变化计算切向加速度方向
					if(angleChange>=0.0f)
					{
						accNAngle = tempAngle + 90.0f;			
					}
					else
					{
						accNAngle = tempAngle - 90.0f;
					}

					if(accNAngle>180.0f)
					{
						accNAngle-=360.0f;
					}
					else if(accNAngle<-180.0f)
					{
						accNAngle+=360.0f;
					}
					//估算该段轨迹结束时的速度替代之前的最大速度
					tempVirtualVel = sqrt(2 * GetAccLimit(tempAngle) * (GetRingBufferPointLen(i + 2) - GetRingBufferPointLen(i + 1)) + GetRingBufferPointVell(i + 1) *GetRingBufferPointVell(i + 1));
					//该段轨迹结束的最大速度
					tempTargetVel = GetRingBufferPointVell(i + 2);
					//如果估算速度小于最大速度，用估算速度来估算法向加速度
					if(tempVirtualVel<GetRingBufferPointVell(i + 2))
					{
						tempTargetVel = tempVirtualVel;
					}
					//计算法向加速度
					accN = pow(tempTargetVel + GetRingBufferPointVell(i + 1),2)/(2.0f * (curvature[i] + curvature[i + 1]));
					//法向加速度较小时忽略不计
					if(accN<=100.0f)
					{
						accN = 0.0f;
					}
					//计算切向加速度
					accT = CalculateAccT(accN , accNAngle , tempAngle);
					//根据切向加速度估计该段轨迹结束时的速度
					tempVell = sqrt(2 * ((1.0f - ACC_REDUNDANT) * accT) * (GetRingBufferPointLen(i + 2) - GetRingBufferPointLen(i + 1)) + GetRingBufferPointVell(i + 1) * GetRingBufferPointVell(i + 1));
					if (tempVell < GetRingBufferPointVell(i + 2))
					{
						SetRingBufferPointVell(i + 2, tempVell);
						USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
					(uint8_t *)"middle 3 speed: %d\r\n",tempVell);
					}
				}
			}

			for (int i = n - 1; i > 0; i--)
			{
				if (GetRingBufferPointVell(i) > GetRingBufferPointVell(i + 1))
				{
					
					//计算两点间的角度平均值
					tempAngle = (GetRingBufferPointAngle(i) + GetRingBufferPointAngle(i + 1));
					//计算速度方向的变化
					angleChange = GetRingBufferPointAngle(i + 1) - GetRingBufferPointAngle(i);

					if(fabs(angleChange)>180.0f)
					{
						tempAngle+=360.0f;
					}

					tempAngle/=2.0f;

					if(tempAngle>180.0f)
					{
						tempAngle-=360.0f;
					}
					else if(tempAngle<-180.0f)
					{
						tempAngle+=360.0f;
					}

					if(angleChange>180.0f)
					{
						angleChange-=360.0f;
					}
					else if(angleChange<-180.0f)
					{
						angleChange+=360.0f;
					}

					//根据速度方向变化计算切向加速度方向
					if(angleChange>=0.0f)
					{
						accNAngle = tempAngle + 90.0f;			
					}
					else
					{
						accNAngle = tempAngle - 90.0f;
					}

					if(accNAngle>180.0f)
					{
						accNAngle-=360.0f;
					}
					else if(accNAngle<-180.0f)
					{
						accNAngle+=360.0f;
					}
					//估算该段轨迹结束时的速度替代之前的最大速度
					tempVirtualVel = sqrt(2 * GetAccLimit(tempAngle) * (GetRingBufferPointLen(i + 1) - GetRingBufferPointLen(i)) + GetRingBufferPointVell(i + 1) * GetRingBufferPointVell(i + 1));
					//该段轨迹结束的最大速度
					tempTargetVel = GetRingBufferPointVell(i);
					//如果估算速度小于最大速度，用估算速度来估算法向加速度
					if(tempVirtualVel<GetRingBufferPointVell(i))
					{
						tempTargetVel = tempVirtualVel;
					}
					//计算法向加速度
					accN = pow(tempTargetVel + GetRingBufferPointVell(i + 1),2)/(2.0f * (curvature[i] + curvature[i - 1]));
					//法向加速度较小时忽略不计
					if(accN<=100.0f)
					{
						accN = 0.0f;
					}
					//计算切向加速度
					accT = CalculateAccT(accN , accNAngle , tempAngle);

					//根据切向加速度估计该段轨迹结束时的速度
					tempVell = sqrt(2 * ((1.0f - ACC_REDUNDANT) * accT) * (GetRingBufferPointLen(i + 1) - GetRingBufferPointLen(i)) + GetRingBufferPointVell(i + 1) * GetRingBufferPointVell(i + 1));
					if (tempVell < GetRingBufferPointVell(i))
					{
						SetRingBufferPointVell(i, tempVell);
						USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
					(uint8_t *)"middle 4 speed:\t%d\r\n",tempVell);
					}
				}
			}
			//将速度小于最小速度的做处理
			for (int i = 2; i < n; i++)
			{
				if (GetRingBufferPointVell(i) < MIN_VELL)
				{
					SetRingBufferPointVell(i, MIN_VELL);
				}
			}
			
			if(velPlan.endVel<10.0f)
			{
				SetRingBufferPointVell(n - 1, GetRingBufferPointVell(n-2)/2);
				
				SetRingBufferPointVell(n, 0);
			}
			else
			{
				SetRingBufferPointVell(n - 1, (GetRingBufferPointVell(n-2) + GetRingBufferPointVell(n))/2);		
			}
			
			USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
				(uint8_t *)"Final Speed:\r\n");
			
			for (int i = 1; i < n; i++)
			{
				SetRingBufferPointVell(i, GetRingBufferPointVell(i));	
			}
			for (int i = 1; i < n+1; i++)
			{
				USARTDMAOUT(DEBUG_USART,DebugUSARTSendBuf,&DebugUSARTSendBuffCnt,DebugUSARTDMASendBuf,DEBUG_USART_SEND_BUF_CAPACITY,\
			(uint8_t *)"%d\r\n",(int)GetRingBufferPointVell(i));
				Delay_ms(5);
			}

			free(wheelOne);
			free(wheelTwo);
			free(wheelThree);
			break;
		}
	}
	free(curvature);
	free(vell);
	free(curveNormalDirection);
}
