#ifndef __WTN6_H
#define __WTN6_H

#include "stm32f1xx_hal.h"

uint8_t WTN6_Broadcast(uint8_t addr);
void WTN6_Repeat(void);
void WTN6_SetVolume(uint8_t volume);

#endif
