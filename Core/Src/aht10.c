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


/* Prototipos de funciones privadas ------------------------------------------*/
static void aht10Read(aht10Data_t *obj, uint8_t *regToRead , uint8_t amount);
static void aht10Write(aht10Data_t *obj, uint8_t *data, uint8_t amount);
static void aht10launchmeasurement(aht10Data_t *obj);

//Buffer de comandos 
uint8_t bufferInit[3]={AHT10_CMD_TRIGGER_MEASUREMENT,AHT10_DATA_0,AHT10_DATA_1};


/*************************************************************************************************
	 *  @brief Inicializacion del driver AHT10
     *
     *  @details
     *   Se asignan las funciones pasadas por parametros a la estructura que tambien se pasa por parametro
     *
	 *  @param		obj	            structura del tipo ahtData_t que contiene las funciones de mas bajo nivel
   *  @param    fncWritePort    Funcion de escritura por i2c propia del hardware
   *  @param    fncReadPort     Funcion de leer por i2c propia del hardware
   *  @param    fncDelayPor     Retardos
   *  @param    addressSlave    Direccion del esclavo
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
	 *  @brief Escribe datos en el modulo 
     *
     *
	 *  @param		obj	     Estructura del tipo ahtData_t que contiene las funciones de mas bajo nivel
   *  @param    data     Datos a ser enviados 
   *  @param    amount   Cantidad de bytes a ser enviados 
	 *  @return     None.
	 *  
***************************************************************************************************/
static void aht10Write(aht10Data_t *obj, uint8_t *data, uint8_t amount)
{
      obj->writeI2C(obj->addresSlave,data,amount);
}
/*************************************************************************************************
	 *  @brief Lee los datos del sensor 
     *
     *  @details
     *   Se manda los datos que llegan a la funcion a la funcion de lectura 
     *
	 *  @param		obj	        Estructura del tipo ahtData_t que contiene las funciones de mas bajo nivel
   *  @param    regToRead   Buffer para guardar los datos
   *  @param    amount      Cantidad de registros datos a leer 
	 *  @return     None.
	 *  
***************************************************************************************************/
static void aht10Read(aht10Data_t *obj, uint8_t *regToRead , uint8_t amount)
{
    obj->readI2C(obj->addresSlave, regToRead ,amount);
}

/*************************************************************************************************
	 *  @brief Resetea el modulo 
     *
	 *  @param		obj  Estructura del tipo ahtData_t que contiene las funciones de mas bajo nivel
	 *  @return       None.
	 *  
***************************************************************************************************/
void aht10SoftReset(aht10Data_t *obj)
{
  aht10Write(obj,AHT10_CMD_SOFT_RESET,1);
  obj->delay_ms_I2C(AHT10_DELAY_RESET);
}

/*************************************************************************************************
	 *  @brief Se inicializa el sensor 
     *
	 *  @param		obj	Estructura del tipo ahtData_t que contiene las funciones de mas bajo nivel
	 *  @return       None.
	 *  
***************************************************************************************************/

void aht10StartMeasurement(aht10Data_t *obj)
{
  aht10Write(obj,AHT10_CMD_INITIALIZE,1);
  obj->delay_ms_I2C(AHT10_DELAY_MEASUREMENT);
}


/*************************************************************************************************
	 *  @brief Lanza el inicio la conversion 
     *
     *  @details
     *   	Se inicializa la medicion de temperatura y humedad .
     *
	 *  @param		obj	Estructura del tipo ahtData_t que contiene las funciones de mas bajo nivel
	 *  @return       None.
	 *  
***************************************************************************************************/
static void aht10launchmeasurement(aht10Data_t *obj)
{
  aht10Write(obj,bufferInit,3);
  obj->delay_ms_I2C(AHT10_DELAY_LAUNCH_MEASUREMENT);
}

/*************************************************************************************************
	 *  @brief Obtencion de la temperatura actual 
     *
     *  @details
     *   	Se obtiene la temperatura actual del sensor leyendo los datos por I2C 
     *
	 *  @param		obj	Estructura del tipo ahtData_t que contiene las funciones de mas bajo nivel
	 *  @return    TEMPERATURE(Data_Temperature)
	 *  
***************************************************************************************************/
uint8_t aht10GetTemperature(aht10Data_t *obj)
{
  uint8_t bufferRead[6]={0,0,0,0,0,0};

   /* Se mandan los comandos para que empieze la medicion */ 
  aht10launchmeasurement(obj);
  /*Leemos el dato crudo desde el dispositivo y terminamos la transaccion*/
  aht10Read(obj,bufferRead,6);  

  uint32_t Data_Temperature=((uint32_t)(bufferRead[3] & 0x0F)<<16) | ((uint16_t) bufferRead[4]<<8)| bufferRead[5];
  return TEMPERATURE(Data_Temperature);

}
/*************************************************************************************************
	 *  @brief Obtencion de la humedad actual 
     *
     *  @details
     *   	Se obtiene la humedad actual del sensor leyendo los datos por I2C 
     *
	 *  @param		obj	Estructura del tipo ahtData_t que contiene las funciones que mas bajo nivel 
	 *  @return       None.
	 *  
***************************************************************************************************/
uint8_t aht10GetHumedity(aht10Data_t *obj)
{
  uint8_t bufferRead[6]={0,0,0,0,0,0};

  /* Se mandan los comandos para que empieze la medicion */ 
  aht10launchmeasurement(obj);

  
	/*Leemos el dato crudo desde el dispositivo y terminamos la transaccion*/
	 
  aht10Read(obj,bufferRead,6);
  
	 /*Aplicamos la conversion correspondiente segun la hoja de datos y retornamos*/
  uint32_t Data_Humedity=(((uint32_t)bufferRead[1]<<16) | ((uint16_t)bufferRead[2]<<8) | (bufferRead[3]))>>4;
  return HUMEDITY(Data_Humedity);
}

/*************************************************************************************************
	 *  @brief Obtencion del estado del sensor 
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
  /* Se mandan los comandos para que empieze la medicion */ 
  aht10launchmeasurement(obj);
  /*Leemos el dato crudo desde el dispositivo*/
  aht10Read(obj,&byteStatus,1);
  return  byteStatus;
}
