#ifndef __DWINTFT_RADAR_H
#define __DWINTFT_RADAR_H

#include "stm32f1xx_hal.h"

void TFT_SetProbeVersion(UART_HandleTypeDef *huart, uint8_t ProbeVersion);
void TFT_SetRadarOrder(UART_HandleTypeDef *huart, uint8_t *pRadarOrder, uint8_t MaxProbeNum);
void TFT_DispRadarDist(UART_HandleTypeDef *huart, uint8_t *pRadarDist, uint8_t n, uint8_t MaxProbeNum);

#endif
