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

#endif
