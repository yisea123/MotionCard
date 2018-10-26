#ifndef __GASVALVECONTROL_H
#define __GASVALVECONTROL_H
#include  "can.h"

#define CLAMP_IO_ID1 							6
#define CLAMP_IO_ID2 							7

#define CLUTCH_IO_ID 							5

#define GASVALVE_BOARD_ID 				0

void gasMotion(void);
void SetMotionFlag(uint32_t status);
void GasValveControl(uint8_t boardNum , uint8_t valveNum , uint8_t valveState);
#endif
