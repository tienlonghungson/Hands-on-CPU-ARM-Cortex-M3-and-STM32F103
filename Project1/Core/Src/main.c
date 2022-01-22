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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct
{
	GPIO_TypeDef * LCD_GPIO;		//LCD port
	uint16_t D0_PIN;				//Data pins - 8 bit mode
	uint16_t D1_PIN;
	uint16_t D2_PIN;
	uint16_t D3_PIN;
	uint16_t D4_PIN;
	uint16_t D5_PIN;
	uint16_t D6_PIN;
	uint16_t D7_PIN;
}LCD_Databus_Config;
typedef struct
{
	GPIO_TypeDef * LCD_GPIO;		//LCD port
	uint16_t EN_PIN;				//EN
	uint16_t RW_PIN;				//RW (usually 0)
	uint16_t RS_PIN;				//RS
}LCD_Controlbus_Config;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;

uint16_t count, timer,display=1;

/* USER CODE BEGIN PV */
const LCD_Databus_Config LCDDataBus =
{
    GPIOA,
    GPIO_PIN_0,
    GPIO_PIN_1,
    GPIO_PIN_2,
    GPIO_PIN_3,
    GPIO_PIN_4,
    GPIO_PIN_5,
    GPIO_PIN_6,
    GPIO_PIN_7
};
const LCD_Controlbus_Config LCDControlBus =
{
	GPIOB,
    GPIO_PIN_9,
    GPIO_PIN_8,
    GPIO_PIN_7,
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
void UartPrint(char *);
void LCD_Init();
void LCD_WriteChar(uint8_t);
void LCD_WriteString(char*);
void LCD_Command(uint8_t);
void LCD_Data(uint8_t);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void UartPrint(char *_out)
{
	HAL_UART_Transmit(&huart1, (uint8_t *)_out, strlen(_out), 20);

	HAL_UART_Receive(&huart1, (uint8_t *)_out, strlen(_out), 20);
}

void LCD_Init()
{
	HAL_GPIO_WritePin(LCDControlBus.LCD_GPIO, LCDControlBus.RW_PIN, GPIO_PIN_RESET);
	LCD_Command(0x38); 	//Chon che do 8 bit, 2 hang cho LCD
	LCD_Command(0x0E); 	//Bat hien thi, nhap nhay con tro
	LCD_Command(0x01); 	//Xoa man hinh
	LCD_Command(0x80); 	//Ve dau dong
}

void LCD_Data(uint8_t data)
{
	if (data & 0x01)
		HAL_GPIO_WritePin(LCDDataBus.LCD_GPIO, LCDDataBus.D0_PIN, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(LCDDataBus.LCD_GPIO, LCDDataBus.D0_PIN, GPIO_PIN_RESET);

	if (data & 0x02)
		HAL_GPIO_WritePin(LCDDataBus.LCD_GPIO, LCDDataBus.D1_PIN, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(LCDDataBus.LCD_GPIO, LCDDataBus.D1_PIN, GPIO_PIN_RESET);

	if (data & 0x04)
		HAL_GPIO_WritePin(LCDDataBus.LCD_GPIO, LCDDataBus.D2_PIN, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(LCDDataBus.LCD_GPIO, LCDDataBus.D2_PIN, GPIO_PIN_RESET);

	if (data & 0x08)
		HAL_GPIO_WritePin(LCDDataBus.LCD_GPIO, LCDDataBus.D3_PIN, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(LCDDataBus.LCD_GPIO, LCDDataBus.D3_PIN, GPIO_PIN_RESET);

	if (data & 0x10)
		HAL_GPIO_WritePin(LCDDataBus.LCD_GPIO, LCDDataBus.D4_PIN, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(LCDDataBus.LCD_GPIO, LCDDataBus.D4_PIN, GPIO_PIN_RESET);

	if (data & 0x20)
		HAL_GPIO_WritePin(LCDDataBus.LCD_GPIO, LCDDataBus.D5_PIN, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(LCDDataBus.LCD_GPIO, LCDDataBus.D5_PIN, GPIO_PIN_RESET);

	if (data & 0x40)
		HAL_GPIO_WritePin(LCDDataBus.LCD_GPIO, LCDDataBus.D6_PIN, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(LCDDataBus.LCD_GPIO, LCDDataBus.D6_PIN, GPIO_PIN_RESET);

	if (data & 0x80)
		HAL_GPIO_WritePin(LCDDataBus.LCD_GPIO, LCDDataBus.D7_PIN, GPIO_PIN_SET);
	else
		HAL_GPIO_WritePin(LCDDataBus.LCD_GPIO, LCDDataBus.D7_PIN, GPIO_PIN_RESET);
}

void LCD_Command(uint8_t cmd)
{
	LCD_Data(cmd);
	HAL_GPIO_WritePin(LCDControlBus.LCD_GPIO, LCDControlBus.RS_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCDControlBus.LCD_GPIO, LCDControlBus.RW_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCDControlBus.LCD_GPIO, LCDControlBus.EN_PIN, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(LCDControlBus.LCD_GPIO, LCDControlBus.EN_PIN, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(LCDControlBus.LCD_GPIO, LCDControlBus.EN_PIN, GPIO_PIN_SET);
}

void LCD_WriteChar(uint8_t c)
{
	LCD_Data(c); 		//Dua du lieu vao thanh ghi
	HAL_GPIO_WritePin(LCDControlBus.LCD_GPIO, LCDControlBus.RS_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(LCDControlBus.LCD_GPIO, LCDControlBus.RW_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(LCDControlBus.LCD_GPIO, LCDControlBus.EN_PIN, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(LCDControlBus.LCD_GPIO, LCDControlBus.EN_PIN, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(LCDControlBus.LCD_GPIO, LCDControlBus.EN_PIN, GPIO_PIN_SET);
}

void LCD_WriteString(char *str)
{
    int i;
    for(i=0;str[i]!='\0';i++)
       LCD_WriteChar(str[i]);
}
/* USER CODE END 0 */

void LCD_print_time(uint16_t timer){
	uint16_t hour = timer/3600;
	uint16_t minute = (timer/60)%60;
	uint16_t second = timer%60;
	char LCD_data[9]="00:00:00";
	sprintf(LCD_data,"%02u:%02u:%02u",hour,minute,second);
	LCD_WriteString(LCD_data);
}

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
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Base_Start_IT(&htim2);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  UartPrint("UART Initialized!");
  LCD_Init();
  LCD_print_time(0);
//  LCD_WriteString("Hello STM32F4");
//  LCD_Command(0xC0);
//  LCD_WriteString("Let's fight Covid");
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if (display){
		  LCD_Init();
		  LCD_print_time(timer);
		  display=0;
	  }
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSE;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 800;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 500;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|LCD_RS_Pin|LCD_EN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LCD_RW_GPIO_Port, LCD_RW_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 LCD_RS_Pin LCD_RW_Pin LCD_EN_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|LCD_RS_Pin|LCD_RW_Pin|LCD_EN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

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

