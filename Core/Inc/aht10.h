/**
  ******************************************************************************
  * @file    aht10.h
  * @author  Mario Aguilar Montoya 
  * @date    10/15/2021
  * @brief   Header file driver modulo aht10.
  ******************************************************************************
  */

/* Define para prevenir inclusion recursiva -------------------------------------*/
#ifndef AHT10_H
#define AHT10_H

/* Includes ------------------------------------------------------------------*/
#include <stdint.h>

#define AHT10_FALSE                         0  
#define AHT10_TRUE                          1     

/** @defgroup Comandos del driver
 *  @{
 */
#define  AHT10_ADDRESS_SLAVE                (uint8_t) 0x38    /*!<Direccion de esclavo 7 bits       */
#define AHT10_CMD_INITIALIZE                (uint8_t) 0xE1    /*!< Comando de inicializacion        */
#define AHT10_CMD_TRIGGER_MEASUREMENT       (uint8_t) 0xAC    /*!< Comando para iniciar la lectura  */
#define AHT10_CMD_SOFT_RESET                (uint8_t) 0xBA    /*!< Comando para reiniciar el sensor */
#define AHT10_DATA_0                        (uint8_t) 0x33    /*!< Comando auxiliar                 */
#define AHT10_DATA_1                        (uint8_t) 0x00    /*!< Comando auxiliar                 */
/**
  * @}
  */

/** @defgroup Definicion de retardos 
 * 
 */
#define AHT10_DELAY_POWER_ON                 (uint8_t) 40      /*!<Retardo para el encendido        */
#define AHT10_DELAY_RESET                    (uint8_t) 25      /*!<Retardo de reset                 */
#define AHT10_DELAY_MEASUREMENT              (uint8_t) 100     /*!<Retardo inicializacion           */
#define AHT10_DELAY_LAUNCH_MEASUREMENT       (uint8_t) 80      /*!<Retardo para la medicion         */
/**
  * @}
  */

/** @defgroup Macros para la transformacion de a magnitudes reales
 * 
 */
#define TEMPERATURE(A)                       (uint8_t) ((A *0.000191)-50)    /*!<Macro con la formula para obtener la temepratura en grados Centigrados */    
#define HUMEDITY(A)                          (uint8_t) (A *0.000095)         /*!<Macro para obtener la humedad en porcentaje                            */
/**
  * @}
  */

/** @defgroup Punteros a funciones para apuntar a las funciones que no seran portables 
 * 
 */
typedef void ( *aht10WriteFcn_t )(uint8_t , void*, uint8_t); /*!<Puntero a funcion  que escribira en I2C      */

typedef void ( *aht10ReadFcn_t )(uint8_t , void*, uint8_t);  /*!<Puntero a funcion  que leera    en I2C       */

typedef void (*delay1ms_t)(uint8_t);                         /*!<Puntero a funcion para el delay              */
/**
  * @}
  */


/** @defgroup Estructura simple con punteros a funciones para separar la capa mas baja del driver  
 * 
 */
typedef struct aht10Data {

    aht10WriteFcn_t writeI2C;
    aht10ReadFcn_t  readI2C;
    delay1ms_t      delay_ms_I2C;    
    uint8_t 		addresSlave;

}aht10Data_t;

/**
  * @}
  */

/** @defgroup Prototipos de funciones 
 * 
 */
void aht10Init(aht10Data_t *obj, aht10WriteFcn_t fncWritePort, aht10ReadFcn_t fncReadPort, delay1ms_t fncDelayPort,uint16_t addressSlave);
void aht10SoftReset(aht10Data_t *obj);
uint8_t aht10GetTemperature(aht10Data_t *obj);
uint8_t aht10GetHumedity(aht10Data_t *obj);
uint8_t aht10GetStatus(aht10Data_t *obj);
/**
  * @}
  */

#endif
