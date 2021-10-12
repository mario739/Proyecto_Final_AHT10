#ifndef AHT10_H
#define AHT10_H


#include <stdint.h>

#define AHT10_FALSE                         0  
#define AHT10_TRUE                          1  
#define AHT10_SIZE_REG                      1    


   /* Direccion del esclavo de 7 bits*/


#define AHT10_CMD_INITIALIZE                0xE1    /*!< comando de inicializacion */
#define AHT10_CMD_TRIGGER_MEASUREMENT       0xAC    /*!< comando para iniciar medicion */
#define AHT10_CMD_SOFT_RESET                (uint8_t 0xBA)  /*!< comando para resetear*/
#define AHT10_DATA_0                        0x33
#define AHT10_DATA_1                        0x00



#define AHT10_DELAY_RESET                    25
#define AHT10_DELAY_STARD                    40
#define AHT10_DELAY_MEASUREMENT              75






//      void  escribir(uint8_t addr, void* data, uint8_t amount)
typedef void ( *aht10WriteFcn_t )(uint8_t , void*, uint8_t);

//      void  read(uint8_t addr, void* data, uint8_t amount, uint8_t leng_register)
typedef void ( *aht10ReadFcn_t )(uint8_t , void*, uint8_t);

//      void retardo(uint_8t retardo ms)
typedef void (*delay1ms_t)(uint8_t);

//-----------------------------------------------------
//variables
//Comando de medicion NOrmal 
//eSensorMeasureCmd[3]   = {0xAC, 0x33, 0x00};

//funciones que leen y escriben internamente en mi driver, no dependen del microcnotrolador
typedef struct aht10Data {
    /*write i2c */
    aht10WriteFcn_t writeI2C;
    /*read i2c*/
    aht10ReadFcn_t  readI2C;
    /*retardo i2c */
    delay1ms_t      delay_ms_I2C;    

    uint8_t addresSlave;
    uint8_t dataRaw[6];

}aht10Data_t;





void aht10Init(aht10Data_t *obj, aht10WriteFcn_t fncWritePort, aht10ReadFcn_t fncReadPort, delay1ms_t fncDelayPort,uint16_t addressSlave);
float aht10GetTemperature(aht10Data_t *obj);
float aht10GetHumidity(aht10Data_t *obj);
void aht10SoftReset(aht10Data_t *obj);
uint8_t aht10GetStatus(aht10Data_t *obj);
#endif
