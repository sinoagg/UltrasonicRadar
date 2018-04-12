#include "WTN6.h"
#include "delay.h"
#include "main.h"

uint8_t WTN6_Broadcast(uint8_t addr)
{
	if(GPIO_PIN_SET==HAL_GPIO_ReadPin(BELL_BUSY_GPIO_Port, BELL_DATA_Pin))
	{
		uint8_t bit_data;
		uint8_t j;
		HAL_GPIO_WritePin(BELL_DATA_GPIO_Port, BELL_DATA_Pin, GPIO_PIN_RESET);
		Delay_ms(5); 
		for(j=0;j<8;j++)
		{
			bit_data = addr&0X01;
			if(bit_data == 1)
			{
				HAL_GPIO_WritePin(BELL_DATA_GPIO_Port, BELL_DATA_Pin, GPIO_PIN_SET);
				Delay_us(600); 
				HAL_GPIO_WritePin(BELL_DATA_GPIO_Port, BELL_DATA_Pin, GPIO_PIN_RESET);
				Delay_us(200);
			}
			else
			{
				HAL_GPIO_WritePin(BELL_DATA_GPIO_Port, BELL_DATA_Pin, GPIO_PIN_SET);
				Delay_us(200);
				HAL_GPIO_WritePin(BELL_DATA_GPIO_Port, BELL_DATA_Pin, GPIO_PIN_RESET);
				Delay_us(600); 
			}
			addr = addr>>1;
		}
		HAL_GPIO_WritePin(BELL_DATA_GPIO_Port, BELL_DATA_Pin, GPIO_PIN_SET);
		return 0;
	}
	else 
		return 1;
}

void WTN6_Repeat(void)
{
	WTN6_Broadcast(0xF2);
}
	
void WTN6_SetVolume(uint8_t volume)
{
	WTN6_Broadcast(0xE1+volume*5-1);//E0������С��EF�������
}
