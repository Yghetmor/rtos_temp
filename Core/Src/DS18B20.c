#include "DS18B20.h"
#include <stdint.h>

uint8_t initialise_DS18B20(GPIO_TypeDef *port, uint16_t pin, TIM_HandleTypeDef *timer)
{
    HAL_TIM_Base_Start(timer);
    LL_GPIO_SetPinMode(port, pin, LL_GPIO_MODE_OUTPUT);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    __HAL_TIM_SET_COUNTER(timer, 0);
    while((uint16_t)__HAL_TIM_GET_COUNTER(timer) < 480)
    {
        __NOP();
    }
    LL_GPIO_SetPinMode(port, pin, LL_GPIO_MODE_INPUT);

    __HAL_TIM_SET_COUNTER(timer, 0);
    while(HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET)
    {
        if ((uint16_t)__HAL_TIM_GET_COUNTER(timer) > 70)
        {
            return 1;
        }
    }

    __HAL_TIM_SET_COUNTER(timer, 0);
    while(HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET)
    {
        if ((uint16_t)__HAL_TIM_GET_COUNTER(timer) > 500)
        {
            return 2;
        }
    }

    HAL_TIM_Base_Stop(timer);
    return 0;
}
