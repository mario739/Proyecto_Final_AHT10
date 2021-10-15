

#ifndef AHT10_PORT_H
#define AHT10_PORT_H

#include <stdint.h>
#include <stdio.h>

void   read_I2C_STM32L432_port(uint8_t addr, void* buffer, uint8_t amount);
void   write_I2C_STM32L432_port(uint8_t addr, void* buffer, uint8_t amount);
void   delay_STM32L432_port(uint8_t delay);

#endif