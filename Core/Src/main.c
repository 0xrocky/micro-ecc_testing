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
#define WELCOME "*** Welcome to the micro-ecc STM32F407VG Discovery menu ***\r\n"
#define MENU "Select the option you want:\r\n1: Toggle a led\r\n2: Read the led status\r\n3: Create a uECC_secp160r1 curve and make a key couple\r\n4: Sign an \"HELLO\" message with micro-ecc\r\n5: Verify the previous sign with micro-ecc\r\n6: Clear your screen and print this message"
#define PROMPT "\r\n< "
#define RESET "\033[0;0H"	/* Reset screen point to the top */
#define CLEAR "\033[2J"		/* Clear out the terminal program */
/* USER CODE END PD */
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define FLAG_MAKE 0
#define FLAG_SIGN 1
/* USER CODE END PM */
/* Private variables ---------------------------------------------------------*/
RNG_HandleTypeDef hrng;
UART_HandleTypeDef huart2;
/* USER CODE BEGIN PV */
uECC_Curve curve;
uint8_t privateKey[21] = { 0 }; /* 21 bytes long with first byte 0 in secp160r1 */
uint8_t publicKey[40] = { 0 }; /* 2 * curve size bytes long (secp160r1: 160*2/8) */
const uint8_t hashMsg[32] = { 0x2C, 0xF2, 0x4D, 0xBA, 0x5F, 0xB0, 0xA3, 0x0E, 0x26, 0xE8, 0x3B, 0x2A, 0xC5, 0xB9, 0xE2, 0x9E, 0x1B, 0x16, 0x1E, 0x5C, 0x1F, 0xA7, 0x42, 0x5E, 0x73, 0x04, 0x33, 0x62, 0x93, 0x8B, 0x98, 0x24 }; /* "Hello" SHA-256 hash */
uint8_t signHash[40] = { 0 };
uint8_t flags;
/* USER CODE END PV */
/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_RNG_Init(void);
/* USER CODE BEGIN PFP */
void printWelcomeMsg( void );
uint8_t readUserInput( void );
uint8_t processUserInput( uint8_t choice );
static int trng( uint8_t *dest, unsigned size );
static void forEachByteArray( uint8_t *array, size_t size );
static void flag_make_set();
static void flag_make_clr();
static uint8_t flag_make_get();
static void flag_sign_set();
static void flag_sign_clr();
static uint8_t flag_sign_get();
static void flag_verify_set();
static void flag_verify_clr();
static uint8_t flag_verify_get();
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
	uint8_t choice = 0, tempcode = 0;
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
  MX_RNG_Init();
  /* USER CODE BEGIN 2 */
  /* DWT for counting CPU cycles */
  volatile uint32_t *DWT_CONTROL = (uint32_t *) 0xE0001000;
  volatile uint32_t *DWT_CYCCNT = (uint32_t *) 0xE0001004;
  volatile uint32_t *DEMCR = (uint32_t *) 0xE000EDFC;
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* Creating the uECC curve */
  #if uECC_SUPPORTS_secp160r1
  	  curve = uECC_secp160r1();
  #endif
  /* Setting the random number generator function used by the curve */
  uECC_set_rng( &trng );

  /* USER CODE END 2 */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	printWelcomeMsg();
	do
	{
		choice = readUserInput();
		tempcode = processUserInput( choice );
	}
	while( !( choice == 6 && tempcode == 2 ) );
  }
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  /* USER CODE END 3 */
}
/* Function that prints on video the welcome message */
void printWelcomeMsg( void )
{
	HAL_UART_Transmit( &huart2, ( uint8_t* )RESET, strlen( RESET ), HAL_MAX_DELAY );
	HAL_UART_Transmit( &huart2, ( uint8_t* )CLEAR, strlen( CLEAR ), HAL_MAX_DELAY );
	HAL_UART_Transmit( &huart2, ( uint8_t* )WELCOME, strlen ( WELCOME ), HAL_MAX_DELAY );
	HAL_UART_Transmit( &huart2, ( uint8_t* )MENU, strlen ( MENU ), HAL_MAX_DELAY );
}
/* Function that reads the input of the user expected as a single number */
uint8_t readUserInput( void )
{
	char buf;
	HAL_UART_Transmit( &huart2, ( uint8_t* )PROMPT, strlen ( PROMPT ), HAL_MAX_DELAY );
	HAL_UART_Receive( &huart2, ( uint8_t* )&buf, 1, HAL_MAX_DELAY );
	return atoi( &buf );
}
/* Function that processes the input of the user executing a specific procedure driven by the number chosen */
uint8_t processUserInput( uint8_t choice )
{
	char msg[ 40 ];
	uint32_t start = 0, end = 0, elapsed = 0;
	uint8_t result = 0;

	if ( choice < 1 || choice > 6 )
		return 0;

	sprintf( msg, "%d", choice );
	HAL_UART_Transmit( &huart2, ( uint8_t* )msg, strlen ( msg ), HAL_MAX_DELAY );

	switch ( choice )
	{
		/* Toggle a led */
		case 1:
			start = DWT->CYCCNT;
			HAL_GPIO_TogglePin( GPIOD, GPIO_PIN_15 );
			end = DWT->CYCCNT;
			break;
		/* Read a led status */
		case 2:
			start = DWT->CYCCNT;
			result = HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_15); /* uint8_t vs uint16_t */
			end = DWT->CYCCNT;
			sprintf( msg, "\r\nLED status: %s", result == GPIO_PIN_SET ? "ON" : "OFF" );
			HAL_UART_Transmit( &huart2, ( uint8_t* )msg, strlen ( msg ), HAL_MAX_DELAY );
			break;
		/* Create a curve and make a key pair */
		case 3:
			start = DWT->CYCCNT;
			result = uECC_make_key( publicKey, privateKey, curve );
			end = DWT->CYCCNT;
			if ( !result )
			{
		        flag_make_clr(); /* clear the make flags */
				sprintf( msg, "\r\nuECC make key failure!" );
			}
			else
			{
				flag_make_set(); /* set the make flags */
				sprintf( msg, "\r\nuECC make key success!" );
			}
			HAL_UART_Transmit( &huart2, ( uint8_t* )msg, strlen ( msg ), HAL_MAX_DELAY );

			sprintf( msg, "\r\nThe public key is: " );
			HAL_UART_Transmit( &huart2, ( uint8_t* )msg, strlen ( msg ), HAL_MAX_DELAY );
			forEachByteArray( publicKey, sizeof ( publicKey ) );
			sprintf( msg, "\r\nThe private key is: " );
			HAL_UART_Transmit( &huart2, ( uint8_t* )msg, strlen ( msg ), HAL_MAX_DELAY );
			forEachByteArray( privateKey, sizeof ( privateKey ) );
			break;
		/* Signing */
		case 4:
			if( !( flag_make_get() ) ) /* verify the make flags */
			{
				sprintf( msg, "\r\nKeys pair not created!" );
				HAL_UART_Transmit( &huart2, ( uint8_t* )msg, strlen ( msg ), HAL_MAX_DELAY );
				return 2;
			}
			sprintf( msg, "\r\n\"HELLO\" SHA-256 hash: " );
			HAL_UART_Transmit( &huart2, ( uint8_t* )msg, strlen ( msg ), HAL_MAX_DELAY );
			forEachByteArray( hashMsg, sizeof ( hashMsg ) );

			start = DWT->CYCCNT;
			result = uECC_sign( privateKey, hashMsg, sizeof( hashMsg ), signHash, curve );
			end = DWT->CYCCNT;
			if ( !result )
			{
		        flag_sign_clr(); /* clear the sign flags */
				sprintf( msg, "\r\nuECC sign failure!" );
			}
			else
			{
		        flag_sign_set(); /* set the sign flags */
				sprintf( msg, "\r\nuECC sign success!" );
			}
			HAL_UART_Transmit( &huart2, ( uint8_t* )msg, strlen ( msg ), HAL_MAX_DELAY );

			sprintf( msg, "\r\nThe sign of the HELLO SHA-256 hash: " );
			HAL_UART_Transmit( &huart2, ( uint8_t* )msg, strlen ( msg ), HAL_MAX_DELAY );
			forEachByteArray( signHash, sizeof( signHash ) );
			break;
			/* Verifying */
			case 5:
			if( !( flag_sign_get() ) ) /* verify the sign flags */
			{
				sprintf( msg, "\r\nMessage not signed!" );
				HAL_UART_Transmit( &huart2, ( uint8_t* )msg, strlen ( msg ), HAL_MAX_DELAY );
				return 2;
			}
			start = DWT->CYCCNT;
			result = uECC_verify( publicKey, hashMsg, sizeof( hashMsg ), signHash, curve );
			end = DWT->CYCCNT;
			if ( !result )
				sprintf( msg, "\r\nuECC verify of sign failure!" );
			else
				sprintf( msg, "\r\nuECC verify of sign success!" );
			HAL_UART_Transmit( &huart2, ( uint8_t* )msg, strlen ( msg ), HAL_MAX_DELAY );
			break;
		case 6:
			HAL_Delay( 1000 );
			return 2;
	}
	/* Computing clock cycles */
	if ( end >= start )
	{
		elapsed = end - start;
		sprintf( msg, "\r\n%lu CPU clock cycles", elapsed );
		HAL_UART_Transmit( &huart2, ( uint8_t* )msg, strlen ( msg ), HAL_MAX_DELAY );
		return 1;
	}
	else
		return 0;
}
/* Function that sets a true random number generator for the micro-ecc lib */
static int trng( uint8_t *dest, unsigned size )
{
	while ( size )
	{
		/* uint8_t random = rand() % 1000 + 1; */
		uint32_t random = 0;
		if ( HAL_RNG_GenerateRandomNumber( &hrng, &random ) != HAL_OK )
			Error_Handler();
	    *dest = random;
	    ++dest;
	    --size;
	}
	return 1;
}
/* Support function that prints on video from UART an array byte by byte */
static void forEachByteArray( uint8_t *array, size_t size )
{
	char msg[ 40 ];
	for ( int i = 0; i < size; i++ )
	{
		HAL_UART_Transmit( &huart2, ( uint8_t* )"0x", strlen( "0x" ), HAL_MAX_DELAY );
		sprintf( msg, "%.2X", array[ i ] );
		HAL_UART_Transmit( &huart2, ( uint8_t* )msg, strlen( msg ), HAL_MAX_DELAY );
		if ( i != size - 1 )
			HAL_UART_Transmit( &huart2, ( uint8_t* )", ", strlen( ", " ), HAL_MAX_DELAY );
	}
	HAL_UART_Transmit( &huart2, ( uint8_t* )"\r\n", strlen ( "\r\n" ), HAL_MAX_DELAY );
}
/* Support function that set a flag if the keys pair is made */
static void flag_make_set()
{
	flags |= ( 1 << FLAG_MAKE );
}
/* Support function that reset a flag for the keys pair */
static void flag_make_clr()
{
	flags &= ~( 1 << FLAG_MAKE );
}
/* Support function that get a flag if the keys pair is created */
static uint8_t flag_make_get()
{
	return flags & ( 1 << FLAG_MAKE );
}
/* Support function that set a flag if the sign is made */
static void flag_sign_set()
{
	flags |= ( 1 << FLAG_SIGN );
}
/* Support function that reset a flag for the sign */
static void flag_sign_clr()
{
	flags &= ~( 1 << FLAG_SIGN );
}
/* Support function that get a flag if the sign is made */
static uint8_t flag_sign_get()
{
	return flags & ( 1 << FLAG_SIGN );
}
/* Support function that set a flag if the verify sign is made */
static void flag_verify_set()
{
	flags |= ( 1 << FLAG_MAKE );
	flags |= ( 1 << FLAG_SIGN );
}
/* Support function that reset a flag for the verify sign */
static void flag_verify_clr()
{
	flags &= ~( 1 << FLAG_MAKE );
	flags &= ~( 1 << FLAG_SIGN );
}
/* Support function that get a flag if the verify sign is made */
static uint8_t flag_verify_get()
{
	return flags & ( 1 << FLAG_SIGN ) & ( 1 << FLAG_MAKE );
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 192;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  * @brief RNG Initialization Function
  * @param None
  * @retval None
  */
static void MX_RNG_Init(void)
{
  /* USER CODE BEGIN RNG_Init 0 */
  /* USER CODE END RNG_Init 0 */

  /* USER CODE BEGIN RNG_Init 1 */
  /* USER CODE END RNG_Init 1 */
  hrng.Instance = RNG;
  if (HAL_RNG_Init(&hrng) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RNG_Init 2 */
  /* USER CODE END RNG_Init 2 */
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
