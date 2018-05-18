#include "Radar_usart.h"
#include "delay.h"

extern uint8_t Radar_RxBuf1[];            //探头串口1数据buf
extern uint8_t Radar_RxBuf2[];            //探头串口2数据buf
extern uint8_t Radar_TxBuf[];            //发送串口数据buf

uint8_t i=0;

void CalRadar1Data(uint8_t *pRxBuf1)
{
	if(0)
	{
		for(i=0;i<8;i++)
			Radar_TxBuf[i]=Radar_RxBuf1[i];	
	}	
}

void CalRadar2Data(uint8_t *pRxBuf2)
{
	if(0)
	{
		for(i=0;i<8;i++)
			Radar_TxBuf[i]=Radar_RxBuf1[i];	
	}	
}

void RadarSend(uint8_t *pTxBuf)
{
	uint8_t sum=0;
	for(;;)
		sum=sum+Radar_TxBuf[i];
}