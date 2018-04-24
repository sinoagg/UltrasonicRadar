#include "hardware_init.h"

uint32_t LoadSetVal(uint32_t ParaFlashAddr)
{
	uint32_t Para;
	Para=FlashRead32bit(ParaFlashAddr);
	if((Para&0xFF)==0xFF)		//如果此地址为空
	{
		switch(ParaFlashAddr)
		{
			case FLASH_USER_START_ADDR+RADAR_LIMIT_OFFSET_ADDR:
				Para=DEFAULT_RADAR_LIMIT_DIST;
				FlashWrite_SingleUint32(ParaFlashAddr, Para);
				break;
			case FLASH_USER_START_ADDR+WTN6_VOLUME_OFFSET_ADDR:
				Para=DEFAULT_VOLUME;
				FlashWrite_SingleUint32(ParaFlashAddr, Para);
				break;
			default:
				break;
		}
	}
	return Para;
}

void LoadSetArray(uint32_t ParaFlashAddr, uint8_t *pData, uint8_t num)					//读取一串uint32_t参数
{
	uint32_t Para;
	uint8_t i;
	Para=FlashRead32bit(ParaFlashAddr);
	if((Para&0xFF)==0xFF)		//如果此地址为空
	{
		FlashWrite_ArrayUint32(ParaFlashAddr, (uint32_t *)pData, num);														//写入默认数组
	}
	else
	{
		for(i=0;i<num;i++)
			*(pData+i)=(uint8_t)FlashRead32bit(ParaFlashAddr);
	}
}

