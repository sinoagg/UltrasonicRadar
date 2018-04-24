#ifndef __HARDWARE_INIT_H
#define __HARDWARE_INIT_H

#include "stm32f1xx_hal.h"
#include "InternalFlash.h"

#define DEFAULT_RADAR_LIMIT_DIST	0x96				//Ĭ��1.5��
#define DEFAULT_VOLUME						0x03				//Ĭ���������Ϊ3

uint32_t LoadSetVal(uint32_t ParaFlashAddr);
void LoadSetArray(uint32_t ParaFlashAddr, uint32_t *pData, uint8_t num);

#endif

