#include <stdio.h>
#include "../inc/aht10.h"
#include "../inc/aht10_STM32L432_port.h"

aht10Data_t aht10Data;
#define AHT10_ADDRESS_SLAVE                 (uint8_t)0x38 
//gcc aht10.c wrappers.c main.c -o main
int main(int argc, char const *argv[])
{
    aht10Init(&aht10Data,write_I2C_STM32L432_port,read_I2C_STM32L432_port,delay_STM32L432_port,AHT10_ADDRESS_SLAVE); 
    aht10SoftReset(&aht10Data);
    return 0;
}

