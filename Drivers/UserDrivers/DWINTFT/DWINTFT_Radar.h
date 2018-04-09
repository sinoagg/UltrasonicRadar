#ifndef __DWINTFT_RADAR_H
#define __DWINTFT_RADAR_H

#include "stm32f1xx_hal.h"

void TFT_SetProbeVersion(UART_HandleTypeDef *huart, uint8_t ProbeVersion);
void TFT_ExchangeRadarOrder(UART_HandleTypeDef *huart, uint8_t n1, uint8_t n2, uint8_t MaxProbeNum);
void TFT_DispRadarDist(UART_HandleTypeDef *huart, uint8_t *pRadarDist, uint8_t n);
void TFT_DispRadarColor(UART_HandleTypeDef *huart, uint8_t *pRadarColor, uint8_t MaxProbeNum);
void TFT_DispVechileSpeed(UART_HandleTypeDef *huart, uint8_t speed);

#endif
