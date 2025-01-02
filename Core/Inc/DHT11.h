#ifndef DHT11_H
#define DHT11_H

#include "stm32f1xx_hal.h"
#include <stdint.h>

enum dht_status {
    OK,
    ERR_START_SEND_HIGH,
    ERR_START_RESP_LOW,
    ERR_START_RESP_HIGH,
    ERR_DATA_LOW,
    ERR_DATA_HIGH,
    ERR_DATA,
    ERR_CHKSUM,
};

struct dht11_t {
    uint8_t temperature_int;
    uint8_t temperature_dec;
    uint8_t humidity_int;
    uint8_t humidity_dec;
    enum dht_status status;
};

struct dht11_t read_dht(GPIO_TypeDef* port, uint16_t pin, TIM_HandleTypeDef* timer);

#endif // !DHT11_H
