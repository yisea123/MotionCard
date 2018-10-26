#ifndef __LASER_H
#define __LASER_H


#ifndef LASER_NUM
#define LASER_NUM (3)
#endif

#ifndef LASER_BUFF_SIZE
#define LASER_BUFF_SIZE (30)
#endif

void ADC1mixed_DMA_Config(void);
void ChangeUseC(void);
float GetLaserAValue(void);

float GetLaserBValue(void);
float GetLaserCValue(void);
int getUseC(void);
void resetAngleByLaser(void);
//float GetLaserAValue(void);
float FigureLaserX(void);
float FigureLaserY(void);
void SetMotionFlag(unsigned int status);
void FigureTheRealPostion(void);
double LowPassFilterB(float newValue);
double LowPassFilterD(float newValue);
float getValueB(void);
float getValueC(void);
#endif
