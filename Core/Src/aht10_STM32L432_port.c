
/**
  ******************************************************************************
  * @file    aht10_STM32L432_port.c
  * @author  Mario Aguilar Montoya 
  * @date    10/15/2021
  * @brief   port aht10_STM32L432
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "aht10_STM32L432_port.h"

//Estructura tipo I2C
I2C_HandleTypeDef hi2c1;

HAL_StatusTypeDef status;
/*************************************************************************************************
	 *  @brief Funcion para escribir por I2C
     *
     *
     *  @details
     *   	Esta funcion es especifica para el hardware utilizado. Notar que se basa en una capa
     *   	HAL existente de STM.
     *
	 *  @param		addr	  Direccion del sensor
   *  @param    buffer  Buffer de datos para ser trasmitidos
   *  @param    amount  Cantidad de bytes a transmitir
	 *  @return     None.
	 *
***************************************************************************************************/

aht10_status_fnc  write_I2C_STM32L432_port(uint8_t addr, uint8_t * buffer, uint8_t amount){
     HAL_I2C_Master_Transmit(&hi2c1,(addr<<1), buffer,amount,500);
     return AHT10_OK;
}

/*************************************************************************************************
	 *  @brief Funcion para leer por I2C
     *
     *  @details
     *   	Esta funcion es especifica para el hardware utilizado. Notar que se basa en una capa
     *   	HAL existente de STM.
     *
	 *  @param		addr	       Direccion del sensor
   *  @param    buffer       Buffer para resivir los datos
   *  @param    amount       Cantidad de bytes a leer
	 *  @return     None.
	 *
***************************************************************************************************/
aht10_status_fnc  read_I2C_STM32L432_port(uint8_t addr,uint8_t * buffer, uint8_t amount){

	HAL_I2C_Master_Receive(&hi2c1,(addr<<1),buffer,amount,500);
	return AHT10_OK;
}

/*************************************************************************************************
	 *  @brief Funcion para hacer un retardo
     *
     *  @details
     *   Esta funcion es especifica para el hardware utilizado. Notar que se basa en una capa
     *   HAL existente de STM.
     *
	 *  @param		delay 	Tiempo del retardo en ms
	 *  @return     None.
	 *
***************************************************************************************************/
void delay_STM32L432_port(uint8_t delay){
        HAL_Delay(delay);
}

void   bg96_UART_Transmit(void *data,uint8_t size, uint32_t timeout)
{

	//HAL_UART_Transmit(&huart1, (uint8_t*)data, size, timeout);

}
void   bg96_UART_Receive (void *data,uint8_t size, uint32_t timeout)
{
   // HAL_UART_Receive(&huart1, (uint8_t*)data, size, timeout);//
}



/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00702991;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}


void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}



