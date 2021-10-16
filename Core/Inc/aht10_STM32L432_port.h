/**
  ******************************************************************************
  * @file    aht10_STM32L432_port.h
  * @author  Mario Aguilar Montoya
  * @date    10/15/2021
  * @brief   Header file driver modulo aht10.
  ******************************************************************************
  */


/* Define para prevenir inclusion recursiva -------------------------------------*/
#ifndef AHT10_PORT_H
#define AHT10_PORT_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>
#include <stdio.h>
#include "stm32l4xx_hal.h"



/** @defgroup Prototipo de funciones
 *
 */
void   MX_I2C1_Init(void);
void   read_I2C_STM32L432_port(uint8_t addr, void* buffer, uint8_t amount);
void   write_I2C_STM32L432_port(uint8_t addr, void* buffer, uint8_t amount);
void   delay_STM32L432_port(uint8_t delay);
/**
  * @}
  */

#endif
