#include  "can.h"
#include "gasvalveControl.h"
#include "gpio.h"
#include "usart.h"
#include "task.h"

#define GAS_VALVE_PRINT_LOG 0U



extern Robot_t gRobot;

/**
* @brief  气阀控制
* @param  boardNum：气阀板号
* @param  valveNum：气阀号
* @param  valveState： 气阀状态，0为关，1为开
* @author ACTION
*/
void GasValveControl(uint8_t boardNum , uint8_t valveNum , uint8_t valveState)
{
	uint8_t data = 0x00;
//	uint8_t mbox;
	CanTxMsg TxMessage;
	TxMessage.StdId = 0x0001 ;					 // standard identifier=1
	TxMessage.ExtId = 0x0001 ;				 	 // extended identifier=StdId
	TxMessage.IDE = CAN_Id_Standard;			 // type of identifier for the message is Standard
	TxMessage.RTR = CAN_RTR_Data;			 	 // the type of frame for the message that will be transmitted
	TxMessage.DLC = 1;
	
	data = boardNum<<5|valveNum<<1|valveState;
	
	TxMessage.Data[0] = data;

	OSCANSendCmd(CAN2, &TxMessage);
//	mbox = CAN_Transmit(CAN2, &TxMessage);         
//	while ((CAN_TransmitStatus(CAN2, mbox) != CAN_TxStatus_Ok));
}

void gasMotion(void)
{
	
	if((gRobot.gasMotionFlag&CLAMP_OPEN)&&(!(gRobot.gasMotionFlag&CLAMP_STATUS))){
		GasValveControl(GASVALVE_BOARD_ID , CLAMP_IO_ID1 , 0);
		GasValveControl(GASVALVE_BOARD_ID , CLAMP_IO_ID2 , 1);
	//夹子开		
		SetMotionFlag(CLAMP_STATUS);
	}else if((gRobot.gasMotionFlag&CLAMP_STATUS)&&(!(gRobot.gasMotionFlag&CLAMP_OPEN))){
	//夹子关		
		SetMotionFlag(~CLAMP_STATUS);
		GasValveControl(GASVALVE_BOARD_ID , CLAMP_IO_ID1 , 1);
		GasValveControl(GASVALVE_BOARD_ID , CLAMP_IO_ID2 , 0);
	}
	
	if((gRobot.gasMotionFlag&CLUTCH_SHUT)&&(!(gRobot.gasMotionFlag&CLUTCH_STATUS))){
	//离合器闭合		
		GasValveControl(GASVALVE_BOARD_ID , CLUTCH_IO_ID , 1);
		SetMotionFlag(CLUTCH_STATUS);
	}else if((gRobot.gasMotionFlag&CLUTCH_STATUS)&&(!(gRobot.gasMotionFlag&CLUTCH_SHUT))){
	//离合器分离		
		GasValveControl(GASVALVE_BOARD_ID , CLUTCH_IO_ID , 0);
		SetMotionFlag(~CLUTCH_STATUS);
	}
	
}


void SetMotionFlag(uint32_t status){
	
	switch(status){
		case CLAMP_OPEN:
			gRobot.gasMotionFlag|=CLAMP_OPEN;
			break;
		case ~CLAMP_OPEN:
			gRobot.gasMotionFlag&=~CLAMP_OPEN;
			break;
		case CLAMP_STATUS:
			gRobot.gasMotionFlag|=CLAMP_STATUS;
			break;
		case ~CLAMP_STATUS:
			gRobot.gasMotionFlag&=~CLAMP_STATUS;
			break;
		case CLUTCH_SHUT:
			gRobot.gasMotionFlag|=CLUTCH_SHUT;
			break;
		case ~CLUTCH_SHUT:
			gRobot.gasMotionFlag&=~CLUTCH_SHUT;
			break;
		case CLUTCH_STATUS:
			gRobot.gasMotionFlag|=CLUTCH_STATUS;
			break;
		case ~CLUTCH_STATUS:
			gRobot.gasMotionFlag&=~CLUTCH_STATUS;
			break;
	}
}


