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
 * @param pRadarOrder [Radar probe order pointer]
 * @param MaxProbeNum [8 or 10]
 */
void TFT_SetRadarOrder(UART_HandleTypeDef *huart, uint8_t *pRadarOrder, uint8_t MaxProbeNum)
{
	uint8_t TxBuf[26] = {0x5A,0xA5,0x13,0x82,0x20,0x04};//command header,to send probe order
	if(10 == MaxProbeNum)
		TxBuf[2] = 0x17;
	uint8_t i;
	for(i = 0; i < MaxProbeNum; i++)
	{
		TxBuf[6 + 2 * i] = 0x00;
		TxBuf[7 + 2 * i] = *(pRadarOrder + i);
	}
	HAL_UART_Transmit(huart, TxBuf, 26, 100);
}

/**
 * [TFT_ExchangeRadarOrder]
 * @param huart       [huart index]
 * @param n1          [probe index 1]
 * @param n2          [probe index 2]
 * @param MaxProbeNum [8 or 10]
 */
void TFT_ExchangeRadarOrder(UART_HandleTypeDef *huart, uint8_t n1, uint8_t n2, uint8_t MaxProbeNum)
{
	//uint8_t i;
	//uint8_t RadarOrder[10];
	/*for(i = 0; i < MaxProbeNum; i++)
	{
		RadarOrder[i] = i;
	}*/
	RadarProbeOrder[n1] = n2;//交换探头顺序序号
	RadarProbeOrder[n2] = n1;
	TFT_SetRadarOrder(huart, RadarProbeOrder, MaxProbeNum);
}

/**
 * [TFT_DispVechileSpeed]
 * @param huart [huart index]
 * @param speed [speed low(1) or high(0)]
 */
void TFT_DispVechileSpeed(UART_HandleTypeDef *huart, uint8_t speed)
{
	uint8_t TxBuf[8] = {0x5A,0xA5,0x05,0x82,0x12,0x10,0x00};//command header,to send speed
	TxBuf[7] = speed;
	HAL_UART_Transmit(huart, TxBuf, 8, 100);
}

/**
 * [TFT_SetProbeVersion]
 * @param huart        [huart index]
 * @param ProbeVersion [PROBE_VERSION_8 or PROBE_VERSION_10 in main.c]
 */
void TFT_SetProbeVersion(UART_HandleTypeDef *huart, uint8_t ProbeVersion)
{
	uint8_t TxBuf[22]={0x5A,0xA5,0x07,0x82,0x00,0x84,0x5A,0x01,0x00,ProbeVersion, 0x5A, 0xA5, 0x05, 0x82, 0x12, 0x01, 0x00, ProbeVersion};
	HAL_UART_Transmit(huart, TxBuf, 18, 100);
}

/**
 * [TFT_ReadProbeOrder send instruction to read probe order]
 * @param huart [huart index]
 */
void TFT_ReadProbeOrder(UART_HandleTypeDef *huart)
{
	uint8_t TxBuf[] = {0x5A,0xA5,0x04,0x83,0x20,0x04,0x0A};
	HAL_UART_Transmit(huart, TxBuf, 7, 100);
}
