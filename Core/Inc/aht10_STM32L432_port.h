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
#include "main.h"
#include "aht10.h"

extern  UART_HandleTypeDef huart2;
/** @defgroup Prototipo de funciones
 *
 */
//void   MX_I2C1_Init(void);
aht10_status_fnc  read_I2C_STM32L432_port(uint8_t addr,uint8_t * buffer, uint8_t amount);
aht10_status_fnc  write_I2C_STM32L432_port(uint8_t addr, uint8_t * buffer, uint8_t amount);
void   MX_I2C1_Init(void);
void   MX_USART2_UART_Init(void);
void   delay_STM32L432_port(uint8_t delay);


void   bg96_UART_Transmit(void *data,uint8_t size, uint32_t timeout);
void   bg96_UART_Receive (void *data,uint8_t size, uint32_t timeout);
/**
  * @}
  */

#endif
