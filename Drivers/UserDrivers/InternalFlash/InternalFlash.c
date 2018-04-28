#include "InternalFlash.h"

uint32_t GetFlashData_SingleUint32(uint32_t flash_addr)
{
	return FlashRead32bit(flash_addr);
}

void GetFlashData_U32(uint32_t* pdata, uint32_t flash_addr, uint8_t length)
{
	uint8_t i;
	for(i=0;i<length;i++)
	{
		*pdata=FlashRead32bit(flash_addr);
		flash_addr+=4;
	}
}

uint32_t FlashRead32bit(uint32_t ReadAddr)
{
	uint32_t rtn_value;
	rtn_value=*(__IO uint32_t*)ReadAddr;//读取4个字节.		
	return rtn_value;
}

void FlashErase(uint32_t start_address,uint32_t end_address)
{
	FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PAGEError = 0;
	EraseInitStruct.TypeErase   = FLASH_TYPEERASE_PAGES;
  EraseInitStruct.PageAddress = start_address;
  EraseInitStruct.NbPages     = (end_address - start_address+1) / FLASH_PAGE_SIZE;
	while(HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK);
}


void FlashWrite_SingleUint32(uint32_t start_address, uint32_t data)
{
	HAL_FLASH_Unlock();													//解锁
	FlashErase(start_address,start_address+FLASH_PAGE_SIZE-1);
	HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, start_address, data); 
  /* Lock the Flash to disable the flash control register access (recommended
     to protect the FLASH memory against possible unwanted operation) *********/
  HAL_FLASH_Lock(); 
}

void FlashWrite_ArrayUint32(uint32_t start_address, uint32_t *pData, uint8_t num)
{
	uint8_t i=0;
	HAL_FLASH_Unlock();
	FlashErase(start_address,start_address+FLASH_PAGE_SIZE-1);
	for(i=0;i<num;i++)
		HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, start_address + 4*i,*(pData+i));
	HAL_FLASH_Lock();
	
}
