#include "DHT11.h"
#include "cmsis_os2.h"
#include "main.h"
#include "stm32f1xx_hal_gpio.h"
#include <stdint.h>

void set_dht11_gpio_mode(GPIO_TypeDef* port, uint16_t pin, uint8_t pMode)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};

	if(pMode == 1)
	{
	  GPIO_InitStruct.Pin = pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	  HAL_GPIO_Init(port, &GPIO_InitStruct);
	}else if(pMode == 0)
	{
	  GPIO_InitStruct.Pin = pin;
	  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	  GPIO_InitStruct.Pull = GPIO_PULLUP;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	  HAL_GPIO_Init(port, &GPIO_InitStruct);
	}
}

struct dht11_t read_dht(GPIO_TypeDef* port, uint16_t pin, TIM_HandleTypeDef* timer)
{
    struct dht11_t dht_response = {0};
    uint16_t time = 0;
    uint8_t data_buf[40];
    uint8_t checksum = 0, calculated_checksum = 0;

    set_dht11_gpio_mode(port, pin, 1);
    HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    HAL_Delay(18);
    __disable_irq();
    HAL_TIM_Base_Start(timer);
    set_dht11_gpio_mode(port, pin, 0);
    // HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);

    __HAL_TIM_SET_COUNTER(timer, 0);
    while(HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET)
    {
        if ((uint16_t)__HAL_TIM_GET_COUNTER(timer) > 500)
        {
            __enable_irq();
            dht_response.status = ERR_START_SEND_HIGH;
            return dht_response;
        }
    }

    __HAL_TIM_SET_COUNTER(timer, 0);
    while(HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET)
    {
        if ((uint16_t)__HAL_TIM_GET_COUNTER(timer) > 200)
        {
            __enable_irq();
            dht_response.status = ERR_START_RESP_LOW;
            return dht_response;
        }
    }

    __HAL_TIM_SET_COUNTER(timer, 0);
    while(HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET)
    {
        if ((uint16_t)__HAL_TIM_GET_COUNTER(timer) > 200)
        {
            __enable_irq();
            dht_response.status = ERR_START_RESP_HIGH;
            return dht_response;
        }
    }

    for (int i = 0 ; i < 40 ; i++)
    {
        __HAL_TIM_SET_COUNTER(timer, 0);
        while(HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_RESET)
        {
            if ((uint16_t)__HAL_TIM_GET_COUNTER(timer) > 100)
            {
                __enable_irq();
                dht_response.status = ERR_DATA_LOW;
                return dht_response;
            }
        }
        __HAL_TIM_SET_COUNTER(timer, 0);
        while(HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET)
        {
            if ((uint16_t)__HAL_TIM_GET_COUNTER(timer) > 100)
            {
                __enable_irq();
                dht_response.status = ERR_DATA_HIGH;
                return dht_response;
            }
        }
        time = (uint16_t)__HAL_TIM_GET_COUNTER(timer);

        if (time < 45)
        {
            data_buf[i] = 0;
        }
        else if (time < 120)
        {
            data_buf[i] = 1;
        }
        else 
        {
            __enable_irq();
            dht_response.status = ERR_DATA;
            return dht_response;
        }
    }

    HAL_TIM_Base_Stop(timer);
    __enable_irq();

    for(int i = 0 ; i < 8 ; i++)
    {
        dht_response.humidity_int += data_buf[i];
        dht_response.humidity_int <<= 1;
    }
    for(int i = 8 ; i < 16 ; i++)
    {
        dht_response.humidity_dec += data_buf[i];
        dht_response.humidity_dec <<= 1;
    }
    for(int i = 16 ; i < 24 ; i++)
    {
        dht_response.temperature_int += data_buf[i];
        dht_response.temperature_int <<= 1;
    }
    for(int i = 24 ; i < 32 ; i++)
    {
        dht_response.temperature_dec += data_buf[i];
        dht_response.temperature_dec <<= 1;
    }
    for(int i = 32 ; i < 40 ; i++)
    {
        checksum += data_buf[i];
        checksum <<= 1;
    }

    calculated_checksum = (dht_response.temperature_int + dht_response.temperature_dec + dht_response.humidity_dec + dht_response.humidity_int) & 0xFF;

    if (checksum != calculated_checksum)
    {
        dht_response.status = ERR_CHKSUM;
        return dht_response;
    }
    else 
    {
        dht_response.status = OK;
        return dht_response;
    }
}
