/**
 * @file aht10.c
 * @brief sensor dirver aht10
 * @author .....
 * @date 12-09-2021
*/


#include "../inc/aht10.h"
#include <string.h>

static void aht10Read(aht10Data_t *obj, uint8_t *regToRead , uint8_t amount);
static void aht10Write(aht10Data_t *obj, uint8_t *data, uint8_t amount);

uint8_t bufferInit[3]={AHT10_CMD_TRIGGER_MEASUREMENT,AHT10_DATA_0,AHT10_DATA_1};

//Nota: temp y hum son de 20 bits
void aht10Init(aht10Data_t *obj, aht10WriteFcn_t fncWritePort, aht10ReadFcn_t fncReadPort, delay1ms_t fncDelayPort,uint16_t addressSlave ){

    obj->addresSlave = addressSlave;
    obj->readI2C =fncReadPort;
    obj->writeI2C =fncWritePort;
    obj->delay_ms_I2C=fncDelayPort;

}


//se usan para reutilizar codigo de lectura o escritura de registro
static void aht10Write(aht10Data_t *obj, uint8_t *data, uint8_t amount){
 
    if(obj->writeI2C != NULL)
    {
      obj->writeI2C(obj->addresSlave,data,amount);
    }
}

static void aht10Read(aht10Data_t *obj, uint8_t *regToRead , uint8_t amount)
{
    obj->readI2C(obj->addresSlave, (void*)regToRead ,amount);//read(uint8_t addr, void* data, uint8_t amount, uint8_t leng_register)
    
}

void aht10SoftReset(aht10Data_t *obj)
{
  uint8_t a= (uint8_t)0xBA;
  aht10Write(obj,&a,1);
  obj->delay_ms_I2C(AHT10_DELAY_RESET);

}
void aht10StartMeasurement(aht10Data_t *obj)
{
  aht10Write(obj,(void*)AHT10_CMD_INITIALIZE,1);
  aht10Write(obj,(void*)AHT10_CMD_TRIGGER_MEASUREMENT,1);
  obj->delay_ms_I2C(AHT10_DELAY_MEASUREMENT);
}


float aht10GetTemperature(aht10Data_t *obj)
{
  aht10Write(obj,bufferInit,3);
  obj->delay_ms_I2C(20);
  uint8_t bufferRead[6]={0,0,0,0,0,0};
  aht10Read(obj,bufferRead,6);
  uint32_t Data_Temperature=((uint32_t)(bufferRead[3]& 0X0F)<<16) | ((uint16_t) bufferRead[4]<<8)| bufferRead[5];
 
  return Data_Temperature;

}

float aht10GetHumedity(aht10Data_t *obj)
{
  aht10Write(obj,bufferInit,3);
  obj->delay_ms_I2C(20);
  uint8_t bufferRead[6]={0,0,0,0,0,0};
  aht10Read(obj,bufferRead,6);
  uint32_t Data_Humedity=(((uint32_t)bufferRead[1]<<16) | ((uint16_t)bufferRead[2]<<8) | (bufferRead[3]))>>4;

  return Data_Humedity;
}

uint8_t aht10GetStatus(aht10Data_t *obj)
{
  aht10Write(obj,bufferInit,3);
  obj->delay_ms_I2C(20);
  uint8_t byteStatus;
  aht10Read(obj,&byteStatus,1);

  return  byteStatus;
}
