#ifndef __INTERNALFLASH_H
#define __INTERNALFLASH_H

#include "stm32f1xx_hal.h"

#define FLASH_USER_START_ADDR        ((uint32_t)0x08009000)   /* Start @ of user Flash area */

#define RADAR_LIMIT_OFFSET_ADDR 0x00   	//À×´ï¼ì²â¾àÀëÆ«ÒÆÁ¿´æ´¢µØÖ·
#define RADAR_PROBE_VERSION_ADDR 0x04		//À×´ï8Ì½»òÕß10Ì½°æ±¾
#define WTN6_VOLUME_OFFSET_ADDR 0x08

void GetFlashData_U32(uint32_t* pdata, uint32_t flash_addr, uint8_t length);
uint32_t GetFlashData_SingleUint32(uint32_t flash_addr);
void FlashErase(uint32_t start_address,uint32_t end_address);
void FlashWrite_SingleUint32(uint32_t start_address, uint32_t data);
uint32_t FlashRead32bit(uint32_t ReadAddr);

#endif

