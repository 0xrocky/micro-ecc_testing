/* USER CODE BEGIN Header */
/**
  ******************************************************************************
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
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "micro-ecc/uECC.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define WELCOME "Welcome to the micro-ecc Discovery menu\r\n"
#define MENU "Select the option you want:\r\n1: Toggle a led\r\n2: Read the led status\r\n3: Make keys with ECC and sign\r\n4: Clear screen and print this message"
#define PROMPT "\r\n< "
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
UART_HandleTypeDef huart2;
/* USER CODE BEGIN PV */
uECC_Curve curve;
uint8_t privateKey[21] = { 0 }; /* 21 bytes long with first byte 0 in secp160r1 */
uint8_t publicKey[40] = { 0 }; /* 2 * curve size bytes long (secp160r1: 160*2/8) */
uint8_t privateKey2[21] = { 0x00, 0x75, 0x96, 0xC8, 0xE2, 0xFD, 0xAD, 0x27, 0x52, 0x5B, 0x3A, 0xE4, 0xCC, 0x77, 0x37, 0x46, 0xBC, 0x62, 0xA7, 0x67, 0x6E };
uint8_t publicKey2[42] = {0x07, 0x0C, 0x00, 0x5A, 0x6D, 0xC5, 0x96, 0x0B, 0x2C, 0x7D, 0x8E, 0x85, 0x47, 0x73, 0xA9, 0xC8, 0x77, 0x58, 0xC3, 0xBE, 0x97, 0x07, 0x6E, 0xE6, 0xF5, 0x00, 0x67, 0xC3, 0x9E, 0x4F, 0xD1, 0xCA, 0x12, 0xAB, 0x23, 0x27, 0x45, 0xF0, 0x5C, 0x22, 0xA1, 0xDB};
uint8_t hashMsg[32] = { 0x2C, 0xF2, 0x4D, 0xBA, 0x5F, 0xB0, 0xA3, 0x0E, 0x26, 0xE8, 0x3B, 0x2A, 0xC5, 0xB9, 0xE2, 0x9E, 0x1B, 0x16, 0x1E, 0x5C, 0x1F, 0xA7, 0x42, 0x5E, 0x73, 0x04, 0x33, 0x62, 0x93, 0x8B, 0x98, 0x24 }; /* "Hello" SHA-256 hash */
uint8_t signHash[40] = { 0 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void printWelcomeMsg( void );
uint8_t readeUserInput( void );
uint8_t processUserInput( uint8_t opt );
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint8_t opt = 0;
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
  /* USER CODE BEGIN 2 */
  /* DWT */
  volatile uint32_t *DWT_CONTROL = (uint32_t *) 0xE0001000;
  volatile uint32_t *DWT_CYCCNT = (uint32_t *) 0xE0001004;
  volatile uint32_t *DEMCR = (uint32_t *) 0xE000EDFC;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  #if uECC_SUPPORTS_secp160r1
  	  curve = uECC_secp160r1();
  #endif

  /* USER CODE END 2 */
printMessage:
	printWelcomeMsg();
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  opt = readeUserInput();
	  int tempCode = processUserInput( opt );
	  if ( opt == 4 && tempCode == 2 )
		  goto printMessage;
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void printWelcomeMsg( void )
{
	HAL_UART_Transmit( &huart2, ( uint8_t* )"\033[0;0H", strlen( "\033[0;0H" ), HAL_MAX_DELAY );
	HAL_UART_Transmit( &huart2, ( uint8_t* )"\033[2J", strlen( "\033[2J" ), HAL_MAX_DELAY );
	HAL_UART_Transmit( &huart2, ( uint8_t* )WELCOME, strlen ( WELCOME ), HAL_MAX_DELAY );
	HAL_UART_Transmit( &huart2, ( uint8_t* )MENU, strlen ( MENU ), HAL_MAX_DELAY );
}

uint8_t readeUserInput( void )
{
	char buf[ 1 ];
	HAL_UART_Transmit( &huart2, ( uint8_t* )PROMPT, strlen ( PROMPT ), HAL_MAX_DELAY );
	HAL_UART_Receive( &huart2, ( uint8_t* )buf, 1, HAL_MAX_DELAY );
	return atoi( buf );
}

uint8_t processUserInput( uint8_t opt )
{
	char msg[ 40 ];
	uint32_t start = 0, end = 0, elapsed = 0;

	if ( opt < 1 || opt > 4 )
		return 0;

	sprintf( msg, "%d", opt );
	HAL_UART_Transmit( &huart2, ( uint8_t* )msg, strlen ( msg ), HAL_MAX_DELAY );

	switch( opt )
	{
		case 1:
			start = DWT->CYCCNT;
			HAL_GPIO_TogglePin( GPIOD, GPIO_PIN_15 );
			HAL_Delay( 1000 );
			end = DWT->CYCCNT;
			break;
		case 2:
			start = DWT->CYCCNT;
			sprintf( msg, "\r\nLED status: %s", HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15) == GPIO_PIN_SET ? "ON" : "OFF" );
			HAL_UART_Transmit( &huart2, ( uint8_t* )msg, strlen ( msg ), HAL_MAX_DELAY );
			end = DWT->CYCCNT;
			break;
		case 3:
			start = DWT->CYCCNT;
			if( uECC_make_key( publicKey, privateKey, curve ) )
				sprintf( msg, "\r\nuECC make key success!\r\n" );
			else
				sprintf( msg, "\r\nuECC make key failure!\r\n" );
			HAL_UART_Transmit( &huart2, ( uint8_t* )msg, strlen ( msg ), HAL_MAX_DELAY );

			if( uECC_sign( privateKey2, hashMsg, sizeof( hashMsg ), signHash, curve ) )
				sprintf( msg, "\r\nuECC sign success!\r\n" );
			else
				sprintf( msg, "\r\nuECC sign failure!\r\n" );
			HAL_UART_Transmit( &huart2, ( uint8_t* )msg, strlen ( msg ), HAL_MAX_DELAY );

			sprintf( msg, "%x", signHash );
			HAL_UART_Transmit( &huart2, ( uint8_t* )msg, strlen ( msg ), HAL_MAX_DELAY );

			if( uECC_verify( publicKey2, hashMsg, sizeof( hashMsg ), signHash, curve ) )
				sprintf( msg, "\r\nuECC verify success!\r\n" );
			else
				sprintf( msg, "\r\nuECC verify failure!\r\n" );
			HAL_UART_Transmit( &huart2, ( uint8_t* )msg, strlen ( msg ), HAL_MAX_DELAY );

			end = DWT->CYCCNT;
			break;
		case 4:
			HAL_Delay( 1000 );
			return 2;
	}
	if( end >= start )
	{
		elapsed = end - start;
		sprintf( msg, "\r\n%lu cycles", elapsed );
		HAL_UART_Transmit( &huart2, ( uint8_t* )msg, strlen ( msg ), HAL_MAX_DELAY );
		return 1;
	}
	else
		return 0;
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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
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
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin : PD15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
