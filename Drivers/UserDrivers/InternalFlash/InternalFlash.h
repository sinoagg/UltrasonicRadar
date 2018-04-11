#ifndef __INTERNALFLASH_H
#define __INTERNALFLASH_H

#include "stm32f1xx_hal.h"

#define FLASH_USER_START_ADDR        ((uint32_t)0x08009000)   /* Start @ of user Flash area */

#define RADAR_LIMIT_OFFSET_ADDR 0x00   	//雷达检测距离偏移量存储地址
#define RADAR_PROBE_VERSION_ADDR 0x04		//雷达8探或者10探版本
#define WTN6_VOLUME_OFFSET_ADDR 0x08
#define RADAR_PROBE_OFFSET_ADDR 0x0C //雷达要交换的探头编号地址

void GetFlashData_U32(uint32_t* pdata, uint32_t flash_addr, uint8_t length);
uint32_t GetFlashData_SingleUint32(uint32_t flash_addr);
void FlashErase(uint32_t start_address,uint32_t end_address);
void FlashWrite_SingleUint32(uint32_t start_address, uint32_t data);
uint32_t FlashRead32bit(uint32_t ReadAddr);

#endif

