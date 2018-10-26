#ifndef __CUSTOMER_H
#define __CUSTOMER_H

/* Includes ------------------------------------------------------------------*/
#include "can.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported functions ------------------------------------------------------- */


void AT_CMD_Handle(void);
void RemoteCtr(void);
void SetRemoteCtrFlag(uint32_t status);


/***********************************************************************************
* @name 		SetAngleZ
* @brief  	??????zangle?
* @param  	temp
* @retval 	?
**********************************************************************************/
void  SetAngleZ(float temp);


/***********************************************************************************
* @name 		GetAngleZ
* @brief  	??zangle?   ?????0????x??????
* @param  	?
* @retval 	?????
**********************************************************************************/
float GetAngleZ(void);

/***********************************************************************************
* @name 		SetPosx
* @brief  	??????X??
* @param  	temp
* @retval 	?
**********************************************************************************/
void  SetPosx(float temp);


/***********************************************************************************
* @name 		GetPosx
* @brief  	??X??
* @param  	?
* @retval 	X??
**********************************************************************************/
float GetPosx(void);


/***********************************************************************************
* @name 		SetPosy
* @brief  	??????Y??
* @param  	temp
* @retval 	?
**********************************************************************************/
void  SetPosy(float temp);


/***********************************************************************************
* @name 		GetPosy
* @brief  	??Y??
* @param  	?
* @retval 	Y??
**********************************************************************************/
float GetPosy(void);

#endif
