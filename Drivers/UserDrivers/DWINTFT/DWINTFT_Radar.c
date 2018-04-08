#include "DWINTFT_Radar.h"
/**
 * [TFT_DispRadarDist display Radar distination of one probe]
 * @param huart       [huart index]
 * @param n           [number of probe]
 * @param pRadarDist  [Radar probe pointer]
 */
void TFT_DispRadarDist(UART_HandleTypeDef *huart, uint8_t *pRadarDist, uint8_t n)
{
	uint8_t TxBuf[8]={0x5A,0xA5,0x05,0x82,0x12,0x11};//command header,1211 means dinstination
	TxBuf[6] = 0x00;
	if(*(pRadarDist + n) <= 0x96)//雷达探头距离数据范围
		TxBuf[7] = *(pRadarDist + n);//发送数组的7#为要显示距离的探头的数据
	HAL_UART_Transmit(huart, TxBuf, 8, 100);//8 means size,100 timeout
}

/**
 * [TFT_DispRadarColor]
 * @param huart       [huart index]
 * @param pRadarColor [Radar probe pointer]
 * @param MaxProbeNum [8 or 10]
 */
void TFT_DispRadarColor(UART_HandleTypeDef *huart, uint8_t *pRadarColor, uint8_t MaxProbeNum)
{
	uint8_t i;
	uint8_t TxBuf[8] = {0x5A,0xA5,0x05,0x82,0x10};
	if(8 == MaxProbeNum)//8探头变量地址从00开始，每个探头+2
	{
		for(i = 0; i < MaxProbeNum; i++)
		{
			TxBuf[5] = 2 * i;//探头地址
			TxBuf[6] = 0x00;
			if(*(pRadarColor + i) <= 0x96)//雷达探头距离数据范围
			{
				if(*(pRadarColor + i) >= 0x64)//1m~1.5m之间是绿色
				{
					TxBuf[7] = 0x02;//写颜色，0001黄色，0002绿色，0004闪烁，0000红色
				}
				else if(*(pRadarColor + i) >= 0x32)//0.5m~1m之间是黄色
				{
					TxBuf[7] = 0x01;
				}
				else
				{
					TxBuf[7] = 0x00;//0.5m以内是红色
				}
				HAL_UART_Transmit(huart, TxBuf, 8, 100);//8 means size,100 timeout
			}
		}
	}
	else	//10探头变量地址从10开始，每个探头+2
	{
		for(i = 0; i < MaxProbeNum; i++)
		{
			TxBuf[5] = 2 * i + 0x10;//探头地址
			TxBuf[6] = 0x00;
			if(*(pRadarColor + i) <= 0x96)//雷达探头距离数据范围
			{
				if(*(pRadarColor + i) >= 0x64)//1m~1.5m之间是绿色
				{
					TxBuf[7] = 0x02;//写颜色，0001黄色，0002绿色，0004闪烁，0000红色
				}
				else if(*(pRadarColor + i) >= 0x32)//0.5m~1m之间是黄色
				{
					TxBuf[7] = 0x01;
				}
				else
				{
					TxBuf[7] = 0x00;//0.5m以内是红色
				}
				HAL_UART_Transmit(huart, TxBuf, 8, 100);//8 means size,100 timeout
			}
		}
	}
}
/**
 * [TFT_SetRadarOrder]
 * @param huart       [huart index]
 * @param pRadarOrder [Radar probe pointer]
 * @param MaxProbeNum [8 or 10]
 */
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

