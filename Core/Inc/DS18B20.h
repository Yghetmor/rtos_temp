#ifndef DS18B20_H
#define DS18B20_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

uint8_t initialise_DS18B20(GPIO_TypeDef* port, uint16_t pin, TIM_HandleTypeDef* timer);

#endif // !DS18B20_H
