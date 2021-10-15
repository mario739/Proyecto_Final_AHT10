

#include "../inc/aht10_STM32L432_port.h"


void  write_I2C_STM32L432_port(uint8_t addr, void* buffer, uint8_t amount){
       //HAL_I2C_Master_Transmit(&hi2c1,addr<<1, buffer,amount,MAX);
}
void  read_I2C_STM32L432_port(uint8_t addr, void* buffer, uint8_t amount){
       //HAL_I2C_Master_Receive(&hi2c1, addr<<1, buffer,amount,800);
}
void delay_STM32L432_port(uint8_t delay){
       //HAL_Delay(delay);
}
