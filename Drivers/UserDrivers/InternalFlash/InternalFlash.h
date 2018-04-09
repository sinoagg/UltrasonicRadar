#ifndef __INTERNALFLASH_H
#define __INTERNALFLASH_H

#include "stm32f1xx_hal.h"

#define FLASH_USER_START_ADDR        ((uint32_t)0x08009000)   /* Start @ of user Flash area */

#define RADAR_LIMIT_OFFSET_ADDR 0x00   	//�״������ƫ�����洢��ַ
#define RADAR_PROBE_VERSION_ADDR 0x04		//�״�8̽����10̽�汾
#define WTN6_VOLUME_OFFSET_ADDR 0x08
#define RADAR_EXCHANGE1_OFFSET_ADDR 0x0C //�״�Ҫ������̽ͷ��ŵ�ַ
#define RADAR_EXCHANGE2_OFFSET_ADDR 0x10 //�״�Ҫ������̽ͷ��ŵ�ַ

void GetFlashData_U32(uint32_t* pdata, uint32_t flash_addr, uint8_t length);
uint32_t GetFlashData_SingleUint32(uint32_t flash_addr);
void FlashErase(uint32_t start_address,uint32_t end_address);
void FlashWrite_SingleUint32(uint32_t start_address, uint32_t data);
uint32_t FlashRead32bit(uint32_t ReadAddr);

#endif

