#include "DWINTFT_Radar.h"

void TFT_DispRadarDist(UART_HandleTypeDef *huart, uint8_t *pRadarDist, uint8_t MaxProbeNum)
{
	uint8_t TxBuf[14]={0x5A,0xA5,0x05,0x82,0x12,0x11};
	uint8_t i;
	for(i=0;i<MaxProbeNum;i++)
		TxBuf[6+i]=*(pRadarDist+i);
	HAL_UART_Transmit(huart, TxBuf, 14, 100);
}

void TFT_SetRadarOrder(UART_HandleTypeDef *huart, uint8_t *pRadarOrder, uint8_t MaxProbeNum)
{
	uint8_t TxBuf[22]={0x5A,0xA5,0x13,0x82,0x20,0x04};
	/* unfinished*/
	HAL_UART_Transmit(huart, TxBuf, 22, 100);
}

void TFT_DispVechileSpeed(uint8_t speed)
{
	
}

void TFT_SetProbeVersion(UART_HandleTypeDef *huart, uint8_t ProbeVersion)
{
	uint8_t TxBuf[22]={0x5A,0xA5,0x07,0x82,0x00,0x84,0x5A,0x01,0x00,ProbeVersion};
	HAL_UART_Transmit(huart, TxBuf, 10, 100);
}

