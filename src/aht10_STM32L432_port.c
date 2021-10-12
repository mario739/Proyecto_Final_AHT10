
#include <stdio.h>
#include "../inc/aht10_STM32L432_port.h"





void  read_I2C_STM32L432_port(uint8_t addr, void* buffer, uint8_t amount){
        /*fncion de i2c de microcontrolador*/
        printf("WrapperParaescribir\n");

}




//typedef void ( *aht10ReadFcn_t )(uint8_t , void*, uint8_t, uint8_t);
void  write_I2C_STM32L432_port(uint8_t addr, void* buffer, uint8_t amount){
      /*fncion de i2c de microcontrolador*/
      /*i2cstart
      i2cWrite(slave)
      i2cWrite(0x02)
      i2cread(buffer)
      i2cstop*/
     // aht10Data.dataRaw ,
      printf("i2cstart\n");
      printf("i2cWrite(addrslave)\n");
      printf("i2cWrite(buffer)\n");
      printf("i2cread(buffer)\n");
      printf("i2cstop\n");
}

void delay_STM32L432_port(uint8_t delay)
{

}