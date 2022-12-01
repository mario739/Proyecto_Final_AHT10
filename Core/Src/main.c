/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @author  Mario Aguilar Montoya 
  * @date    10/15/2021 
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "driver_bg96.h"
#include "aht10_STM32L432_port.h"
#include "aht10.h"
#include "MAX17043.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
aht10_config_t aht10_config;
max17043_data  max17043_config;
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
//MX_USART2_UART_Init(void);
//MX_I2C1_Init(void);
void MX_USART1_UART_Init(void);
em_bg96_error_handling send_data(char*cmd, char*expect,char *buffer,uint32_t timeout);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

typedef enum
{
	VERIFICARCONEXION,
	SETEAR_APN,
	ACTIVAR_PDP,
	OPEN_SERVER,
	CONECTAR_MQTT,
	PUBLICAR_MQTT,
	DESCONECTAR_MQTT,
	DESCONECTAR_PDP,
}estados_mqtt;
st_bg96_config config_module={.send_data_device=NULL,.mode_echo=STATE_ECHO_OFF,.format_response=SHORT_RESULT};
char buffer[150];
char bufferhuart2[150];
char data2[60]="{\"temperatura\":40,\"humedad\":40,\"bateria\":40}";
uint8_t cont1=2;
uint8_t cont2=5;
uint8_t cont3=8;
uint8_t cont4=0;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  aht10Init(&aht10_config,write_I2C_STM32L432_port,read_I2C_STM32L432_port,delay_STM32L432_port);
  init_driver(&config_module,send_data);
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  //aht10_start_measurement(&aht10_config);
  int8_t temperatura=0;
  uint8_t humedad=0;
  char buffersensore[50];
  while (1)
  {
	  aht10_launch_measurement(&aht10_config);
	  aht10_get_humedity(&aht10_config, &humedad);
	  aht10_get_temperature(&aht10_config, &temperatura);
	  sprintf(buffersensore,"humedad:%u  temperatura:%i \r\n",humedad,temperatura);
	  HAL_UART_Transmit(&huart2,buffersensore,50,200);
	  HAL_Delay(200);

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */


/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
    huart1.Instance = USART1;
    huart1.Init.BaudRate = 115200;
    huart1.Init.WordLength = UART_WORDLENGTH_8B;
    huart1.Init.StopBits = UART_STOPBITS_1;
    huart1.Init.Parity = UART_PARITY_NONE;
    huart1.Init.Mode = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart1) != HAL_OK)
    {
    	Error_Handler();
    }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

	}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
//void MX_USART2_UART_Init(void)
//{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
//  huart2.Instance = USART2;
 // huart2.Init.BaudRate = 115200;
 // huart2.Init.WordLength = UART_WORDLENGTH_8B;
  //huart2.Init.StopBits = UART_STOPBITS_1;
 // huart2.Init.Parity = UART_PARITY_NONE;
 // huart2.Init.Mode = UART_MODE_TX_RX;
 // huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
 // huart2.Init.OverSampling = UART_OVERSAMPLING_16;
 // huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
 // huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  //if (HAL_UART_Init(&huart2) != HAL_OK)
 // {
  //  Error_Handler();
 // }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

//}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD3_GPIO_Port, LD3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LD3_Pin */
  GPIO_InitStruct.Pin = LD3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD3_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
em_bg96_error_handling send_data(char*cmd, char*expect,char *buffer,uint32_t timeout)
{
	HAL_UART_Transmit(&huart2,cmd,strlen(cmd),300);
	HAL_UART_Transmit(&huart1,cmd,strlen(cmd),300);

	HAL_UART_Receive_IT(&huart1,buffer,150);
	for (uint8_t i= 0; i<timeout; i++)
	{
		HAL_Delay(1);
		if (expect !=NULL)
			{
				if(strstr(buffer,expect)!=NULL)
					{
						HAL_UART_Transmit(&huart2,buffer,strlen(buffer),500);
						HAL_UART_AbortReceive_IT(&huart1);
						return FT_BG96_OK;
					}
			}
	}
	HAL_UART_AbortReceive_IT(&huart1);
	return FT_BG96_TIMEOUT;
}


void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *huart)
{
	memset(buffer,0,150);
	//HAL_UART_Receive_IT(&huart1,buffer,150);
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
