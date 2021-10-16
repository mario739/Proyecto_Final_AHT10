/**
  ******************************************************************************
  * @file    aht10.c
  * @author  Mario Aguilar Montoya 
  * @date    10/15/2021
  * @brief   Driver aht10
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "aht10.h"



static void aht10Read(aht10Data_t *obj, uint8_t *regToRead , uint8_t amount);
static void aht10Write(aht10Data_t *obj, uint8_t *data, uint8_t amount);
static void aht10launchmeasurement(aht10Data_t *obj);

uint8_t bufferInit[3]={AHT10_CMD_TRIGGER_MEASUREMENT,AHT10_DATA_0,AHT10_DATA_1};


/*************************************************************************************************
	 *  @brief Inicializacion del driver AHT10
     *
     *  @details
     *   	Se copian los punteros a funciones pasados por argumentos a la estructura interna
     *   	del driver.
     *
	 *  @param		obj	Estructura de configuracion para el driver.
   *  @param    fncWritePort   
   *  @param    fncWritePort  
   *  @param    fncDelayPor
   *  @param    addressSlave
	 *  @return     None.
	 *  
***************************************************************************************************/
void aht10Init(aht10Data_t *obj, aht10WriteFcn_t fncWritePort, aht10ReadFcn_t fncReadPort, delay1ms_t fncDelayPort,uint16_t addressSlave ){

    obj->addresSlave = addressSlave;
    obj->readI2C =fncReadPort;
    obj->writeI2C =fncWritePort;
    obj->delay_ms_I2C=fncDelayPort;

}

/*************************************************************************************************
	 *  @brief 
     *
     *  @details
     *   	Se copian los punteros a funciones pasados por argumentos a la estructura interna
     *   	del driver.
     *
	 *  @param		obj	Estructura de configuracion para el driver.
   *  @param    data  
   *  @param    amount 
	 *  @return     None.
	 *  
***************************************************************************************************/
static void aht10Write(aht10Data_t *obj, uint8_t *data, uint8_t amount)
{
      obj->writeI2C(obj->addresSlave,data,amount);
}
/*************************************************************************************************
	 *  @brief 
     *
     *  @details
     *   	Se copian los punteros a funciones pasados por argumentos a la estructura interna
     *   	del driver.
     *
	 *  @param		obj	Estructura de configuracion para el driver.
   *  @param    regToRead 
   *  @param    amount 
	 *  @return     None.
	 *  
***************************************************************************************************/
static void aht10Read(aht10Data_t *obj, uint8_t *regToRead , uint8_t amount)
{
    obj->readI2C(obj->addresSlave, regToRead ,amount);
}

/*************************************************************************************************
	 *  @brief 
     *
     *  @details
     *   	Se copian los punteros a funciones pasados por argumentos a la estructura interna
     *   	del driver.
     *
	 *  @param		obj	Estructura de configuracion para el driver.
	 *  @return       None.
	 *  
***************************************************************************************************/
void aht10SoftReset(aht10Data_t *obj)
{
  aht10Write(obj,AHT10_CMD_SOFT_RESET,1);
  obj->delay_ms_I2C(AHT10_DELAY_RESET);
}

/*************************************************************************************************
	 *  @brief 
     *
     *  @details
     *   	Se copian los punteros a funciones pasados por argumentos a la estructura interna
     *   	del driver.
     *
	 *  @param		obj	Estructura de configuracion para el driver.
	 *  @return       None.
	 *  
***************************************************************************************************/

void aht10StartMeasurement(aht10Data_t *obj)
{
  aht10Write(obj,AHT10_CMD_INITIALIZE,1);
  obj->delay_ms_I2C(AHT10_DELAY_MEASUREMENT);
}


/*************************************************************************************************
	 *  @brief 
     *
     *  @details
     *   	Se copian los punteros a funciones pasados por argumentos a la estructura interna
     *   	del driver.
     *
	 *  @param		obj	Estructura de configuracion para el driver.
	 *  @return       None.
	 *  
***************************************************************************************************/
static void aht10launchmeasurement(aht10Data_t *obj)
{
  aht10Write(obj,bufferInit,3);
  obj->delay_ms_I2C(AHT10_DELAY_LAUNCH_MEASUREMENT);
}

/*************************************************************************************************
	 *  @brief 
     *
     *  @details
     *   	Se copian los punteros a funciones pasados por argumentos a la estructura interna
     *   	del driver.
     *
	 *  @param		obj	Estructura de configuracion para el driver.
	 *  @return       None.
	 *  
***************************************************************************************************/
uint8_t aht10GetTemperature(aht10Data_t *obj)
{
  uint8_t bufferRead[6]={0,0,0,0,0,0};
  aht10launchmeasurement(obj);
  aht10Read(obj,bufferRead,6);
  uint32_t Data_Temperature=((uint32_t)(bufferRead[3] & 0x0F)<<16) | ((uint16_t) bufferRead[4]<<8)| bufferRead[5];
  return TEMPERATURE(Data_Temperature);

}
/*************************************************************************************************
	 *  @brief 
     *
     *  @details
     *   	Se copian los punteros a funciones pasados por argumentos a la estructura interna
     *   	del driver.
     *
	 *  @param		obj	Estructura de configuracion para el driver.
	 *  @return       None.
	 *  
***************************************************************************************************/
uint8_t aht10GetHumedity(aht10Data_t *obj)
{
  uint8_t bufferRead[6]={0,0,0,0,0,0};
  aht10launchmeasurement(obj);
  aht10Read(obj,bufferRead,6);
  uint32_t Data_Humedity=(((uint32_t)bufferRead[1]<<16) | ((uint16_t)bufferRead[2]<<8) | (bufferRead[3]))>>4;

  return HUMEDITY(Data_Humedity);
}

/*************************************************************************************************
	 *  @brief 
     *
     *  @details
     *   	Se copian los punteros a funciones pasados por argumentos a la estructura interna
     *   	del driver.
     *
	 *  @param		obj	Estructura de configuracion para el driver.
	 *  @return       None.
	 *  
***************************************************************************************************/
uint8_t aht10GetStatus(aht10Data_t *obj)
{
  uint8_t byteStatus;
  aht10launchmeasurement(obj);
  aht10Read(obj,&byteStatus,1);

  return  byteStatus;
}
