#include "laser.h"
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_dma.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_rcc.h"
#include "task.h"
#include "adc.h"

static uint16_t laserBuffer[LASER_BUFF_SIZE][LASER_NUM]={0};
extern Robot_t gRobot;
float FilterLaseX(float newValue);
float FilterLaseY(float newValue);

#define LASERB_TO_CENTER_Y (276.0f)
#define LASERC_TO_CENTER_Y (120.0f)
#define CENTER_TO_EDGE_X (495.0f)
/*
*结合激光的值（laserXY）与定位系统的值（gyroXY）以及实际情况，解算出最终定位信息（RealXY）
*方案是，用激光矫正定位系统
*/
extern uint8_t tempisXSameDirect;
extern uint8_t tempisYSameDirect;


#define PLAT_NUM	5
float FilterLaseX(float newValue)
{
	static float datas[PLAT_NUM]={0.f};
	float temp=0.f;
	
	for(int i=0;i<PLAT_NUM-1;i++)
		datas[i]=datas[i+1];
	
	datas[PLAT_NUM-1]=newValue;
	
	for(int i=0;i<PLAT_NUM;i++)
		temp+=datas[i]/PLAT_NUM;
	
	return temp;
}
float FilterLaseY(float newValue)
{
	static float datas[PLAT_NUM]={0.f};
	float temp=0.f;
	
	for(int i=0;i<PLAT_NUM-1;i++)
		datas[i]=datas[i+1];
	
	datas[PLAT_NUM-1]=newValue;
	
	for(int i=0;i<PLAT_NUM;i++)
		temp+=datas[i]/PLAT_NUM;
	
	return temp;
}

void ADC1mixed_DMA_Config(void)
{
	ADC_InitTypeDef       ADC_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	DMA_InitTypeDef       DMA_InitStructure;
	GPIO_InitTypeDef      GPIO_InitStructure;
	
	ADC_StructInit(&ADC_InitStructure);
	DMA_StructInit(&DMA_InitStructure);
	GPIO_StructInit(&GPIO_InitStructure);
	ADC_CommonStructInit(&ADC_CommonInitStructure);
	
	/* Enable ADC, DMA2 and GPIO clocks ****************************************/
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	
	/************************************************************************/
	DMA_InitStructure.DMA_Channel = DMA_Channel_0;     
	DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t)&(ADC1->DR); 			// peripheral address, = & ADC1->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&laserBuffer;					// memory address to save DMA data
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;               	// data dirction: peripheral to memory, ie receive maggage from peripheral
	DMA_InitStructure.DMA_BufferSize = LASER_BUFF_SIZE * LASER_NUM;                          	//the buffer size, in data unit
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;//16 bit data
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;        //16 bit data  32??MCU?1?half-word?16 bits
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;         
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_HalfFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream0, &DMA_InitStructure);
	DMA_Cmd(DMA2_Stream0, ENABLE);

	/* Configure ADCx Pin (ADC Channel) as analog input -------------------------*/
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;   //Track
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz ;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;   //Track
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz ;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;   //Track
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz ;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* ADC Common Init **********************************************************/
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
	ADC_CommonInit(&ADC_CommonInitStructure);
	
		/* ADC1 Init ****************************************************************/
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;	//12λ����
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;			//��ͨ��,ʹ��ɨ��ģʽ
	ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;		//����ת��
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//ת�����������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;	//�����Ҷ���
	ADC_InitStructure.ADC_NbrOfConversion = LASER_NUM;				//˳����й���ת����ADCͨ����Ŀ
	ADC_Init(ADC1, &ADC_InitStructure);

	
	/* ADC1 regular channel13 configuration *************************************/
	ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 1, ADC_SampleTime_480Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_10, 2, ADC_SampleTime_480Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 3, ADC_SampleTime_480Cycles);	
	//ContinuousMode
	ADC_ContinuousModeCmd(ADC1,ENABLE);
	/* Enable DMA request after last transfer (Single-ADC mode) */
	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

	/* Enable ADCx DMA */
	ADC_DMACmd(ADC1, ENABLE);

	/* Enable ADCx */
	ADC_Cmd(ADC1, ENABLE);
	
	ADC_SoftwareStartConv(ADC1);
}

float GetLaserCValue(void)
{
	uint32_t tempSum = 0;
	
	for(uint8_t i = 0; i < LASER_BUFF_SIZE;i++)
	{
		tempSum+=(laserBuffer[i][2]);
	}
	return ((float)(((float)tempSum)/LASER_BUFF_SIZE));
}

float GetLaserBValue(void)
{
	uint32_t tempSum = 0;
	
	for(uint8_t i = 0; i < LASER_BUFF_SIZE;i++)
	{
		tempSum+=(laserBuffer[i][0]);
	}
	return ((float)(((float)tempSum)/LASER_BUFF_SIZE));
}

float GetLaserAValue(void)
{
	uint32_t tempSum = 0;
	
	for(uint8_t i = 0; i < LASER_BUFF_SIZE;i++)
	{
		tempSum+=(laserBuffer[i][1]);
	}
	return ((float)(((float)tempSum)/LASER_BUFF_SIZE));
}


/*延时大，不用了！*/
#define Threshold_1 		 10				//阈值1用于一阶带参滤波器，变化角度大于此值时，计数增加
#define Threshold_2			 50				//阈值2用于一阶带参滤波器，计数值大于此值时，增大参数，增强滤波跟随
double LowPassFilterB(float newValue)
{
  static double K_x=0.0; //滤波系数
  static double k=0.0;
  static uint32_t num_x=0;//滤波计数器
  static uint8_t new_flag_x=0;//本次数据变化方向
  static uint8_t old_flag_x=0;
  static double valueLast=0.0;
  
  //角度变化方向，new_flag=1表示角度增加，=0表示角度正在减小
  if((newValue-valueLast)>0.0)
  {
    new_flag_x=1;
  }
  if((newValue-valueLast)<0.0)
  {
    new_flag_x=0;
  }
  
  if(new_flag_x==old_flag_x)  //此次变化与前一次变化方向是否一致，相等表示角度变化方向一致
  {
    num_x=num_x+1;
    if(fabs(newValue-valueLast)>Threshold_1)
    {
      //当变化角度大于Threshold_1度的时候，进行计数器num快速增加，以达到快速增大K值，提高跟随性
      num_x=num_x+5;   
    }
  
		if(num_x>Threshold_2)   //计数阈值设置，当角度递增或递减速度达到一定速率时，增大K值
		{
			K_x=k+0.5;  //0.2为K_x的增长值，看实际需要修改
			if(K_x>1.0) 
				K_x=1.0; 
			num_x=0;
		}
	}
  else 
  {
    num_x=0;
    K_x=0.01; //角度变化稳定时K_x值，看实际修改
  }
  valueLast=(1-K_x)*valueLast+K_x*newValue;
  old_flag_x=new_flag_x;
  
  return valueLast;
  
}


double LowPassFilterD(float newValue)
{
  static double K_x=0.0; //滤波系数
  static double k=0.0;
  static uint32_t num_x=0;//滤波计数器
  static uint8_t new_flag_x=0;//本次数据变化方向
  static uint8_t old_flag_x=0;
  static double valueLast=0.0;
  
  //角度变化方向，new_flag=1表示角度增加，=0表示角度正在减小
  if((newValue-valueLast)>0.0)
  {
    new_flag_x=1;
  }
  if((newValue-valueLast)<0.0)
  {
    new_flag_x=0;
  }
  
  if(new_flag_x==old_flag_x)  //此次变化与前一次变化方向是否一致，相等表示角度变化方向一致
  {
    num_x=num_x+1;
    if(fabs(newValue-valueLast)>Threshold_1)
    {
      //当变化角度大于Threshold_1度的时候，进行计数器num快速增加，以达到快速增大K值，提高跟随性
      num_x=num_x+5;   
    }
  
		if(num_x>Threshold_2)   //计数阈值设置，当角度递增或递减速度达到一定速率时，增大K值
		{
			K_x=k+0.5;  //0.2为K_x的增长值，看实际需要修改
			if(K_x>1.0) 
				K_x=1.0; 
			num_x=0;
		}
	}
  else 
  {
    num_x=0;
    K_x=0.01; //角度变化稳定时K_x值，看实际修改
  }
  valueLast=(1-K_x)*valueLast+K_x*newValue;
  old_flag_x=new_flag_x;
  
  return valueLast;
}

int search(float aim, float data[], int size) {
	//二分法搜索   
	int det = -1;
	int left = 0;//定义left整数变量  
	int right = size - 1;//定义right  
	while (left <= right) {//在while循环中直到有一个条件结束搜索   
		int mid = (left + right) / 2;
		if (data[mid]<aim) {
			if (left == right)
			{
				det = left;
				break;
			}
			else
			left = mid + 1;
		}
		else if (data[mid]>aim) {
			if ((right - left) == 1)
			{
				det = left;
				break;
			}
			else if (right == left)
			{
				det = left - 1;
				break;
			}
			else
			right = mid - 1;
		}
		else {
			det = mid;
			break;//一定要break跳出循环   
		}
	}
	return det;
}


/*用的是B面激光*/
#define para_K_B			0.743802244492483f

#define BAIS_B				-11.1417375571228f

#define para_K_C			0.738878156335477f

#define BAIS_C				15.9504837216373f

static uint8_t useC=0;
float value_B = 0.f;
float value_C = 0.f;
float FigureLaserY(void)
{
	/*激光在第一象限，定位系统在第三象限，所以加‘-’*/
	float rawLaser_B=GetLaserBValue();
	
	float rawLaser_C=GetLaserCValue();

	float value=0.f;

	value_B = para_K_B*rawLaser_B+BAIS_B;

	value_C = para_K_C*rawLaser_C+BAIS_C;
	
	if(useC)
		value=value_C;
	else
		value=value_B;
	
	return -value;
}
/*用的是D面激光*/

#define para_K_A			0.49513f

#define BAIS_A				12.907f
float FigureLaserX(void)
{
	/*激光在第一象限，定位系统在第三象限，所以加‘-’*/
	float rawLaser=GetLaserAValue();
	
	float value=0.0f;
	
	value = para_K_A*rawLaser+BAIS_A;
	
	return -value;
}


