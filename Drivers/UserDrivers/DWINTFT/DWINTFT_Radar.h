#ifndef __DWINTFT_RADAR_H
#define __DWINTFT_RADAR_H

#include "stm32f1xx_hal.h"

#define TFT_RED 0x00
#define TFT_YELLOW 0x01
#define TFT_GREEN 0x02
#define TFT_BLINK 0x04
#define TFT_MUTE 0x05

void TFT_SetProbeVersion(UART_HandleTypeDef *huart, uint8_t ProbeVersion);
void TFT_SetRadarOrder(UART_HandleTypeDef *huart, uint32_t *pRadarOrder, uint8_t MaxProbeNum);
void TFT_ExchangeRadarOrder(UART_HandleTypeDef *huart, uint8_t n1, uint8_t n2, uint8_t MaxProbeNum);
void TFT_DispRadarDist(UART_HandleTypeDef *huart, uint8_t RadarMinDist);
void TFT_DispRadarColor(UART_HandleTypeDef *huart, uint8_t *pRadarColor, uint8_t MaxProbeNum);
void TFT_DispVehicleSpeed(UART_HandleTypeDef *huart, uint8_t speed);
void TFT_ReadProbeOrder(UART_HandleTypeDef *huart);

extern uint32_t RadarProbeOrder[];
extern uint32_t RadarLimitDist;

#endif
