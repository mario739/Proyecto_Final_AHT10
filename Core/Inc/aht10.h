#ifndef AHT10_H
#define AHT10_H


#include <stdint.h>

#define AHT10_FALSE                         0  
#define AHT10_TRUE                          1  
#define AHT10_SIZE_REG                      1    

#define AHT10_CMD_INITIALIZE                (uint8_t) 0xE1    /*!< comando de inicializacion */
#define AHT10_CMD_TRIGGER_MEASUREMENT       (uint8_t) 0xAC    /*!< comando para iniciar medicion */
#define AHT10_CMD_SOFT_RESET                (uint8_t) 0xBA    /*!< comando para resetear*/
#define AHT10_DATA_0                        (uint8_t) 0x33
#define AHT10_DATA_1                        (uint8_t) 0x00

#define AHT10_DELAY_POWER_ON                 (uint8_t) 40
#define AHT10_DELAY_RESET                    (uint8_t) 25
#define AHT10_DELAY_MEASUREMENT              (uint8_t) 350
#define AHT10_DELAY_LAUNCH_MEASUREMENT       (uint8_t) 80



#define TEMPERATURE(A)                       (uint8_t) ((A *0.000191)-50)
#define HUMEDITY(A)                          (uint8_t) (A *0.000095)





typedef void ( *aht10WriteFcn_t )(uint8_t , void*, uint8_t);

typedef void ( *aht10ReadFcn_t )(uint8_t , void*, uint8_t);

typedef void (*delay1ms_t)(uint8_t);



typedef struct aht10Data {

    aht10WriteFcn_t writeI2C;
    aht10ReadFcn_t  readI2C;
    delay1ms_t      delay_ms_I2C;    
    uint8_t 		addresSlave;
    uint8_t 		dataRaw[6];

}aht10Data_t;


void aht10Init(aht10Data_t *obj, aht10WriteFcn_t fncWritePort, aht10ReadFcn_t fncReadPort, delay1ms_t fncDelayPort,uint16_t addressSlave);
void aht10SoftReset(aht10Data_t *obj);



uint8_t aht10GetTemperature(aht10Data_t *obj);
uint8_t aht10GetHumedity(aht10Data_t *obj);
uint8_t aht10GetStatus(aht10Data_t *obj);

#endif
