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

