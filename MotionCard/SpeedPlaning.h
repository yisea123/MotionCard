#ifndef _SPEEDPLANING_H
#define _SPEEDPLANING_H

#include "MotionCard.h"
float GetVelLimit(float direction);
float GetAccLimit(float direction);
float CalculateAccT(float accN , float accNDirection , float accTDirection);
void CalculateThreeWheelVell(float* wheelOne,float* wheelTwo,float* wheelThree, float* wheelFour);
void DynamicalAjusting(float* wheelOne, float* wheelTwo, float* wheelThree, float* wheelFour);
float DecreseVellByOneWheel(float vellCar, float orientation, float rotationalVell,float zAngle, int wheelNum, float targetWheelVell);
void SpeedPlaning(velPlan_t velPlan);
















#endif

