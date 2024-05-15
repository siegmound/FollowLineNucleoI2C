/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "liquidcrystal_i2c.h"
#include "time.h"
#include"string.h"
#include <stdio.h>
#include <stdbool.h>

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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int x=0;
void stampa(int elem[],time_t now,int moob2[]);
void spersonaggio(int A);
int verifica(int elem[],int A,int deel,int moob2[]);
int updatea(int elem[],int c1);
int updatea2(int elem[],int c2,int moob[]);
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	char buf[1024]={'0'};

	GPIO_PinState GPIO_state1;
	GPIO_PinState GPIO_state2;
	GPIO_PinState GPIO_state3;
	bool isgoingveryleft = false;
	bool isgoingveryright = false;
//	GPIO_PinState GPIO_state;
	//GPIO_PinState GPIO_state2;

	 time_t now=time(NULL);
	 int j=0;
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
  MX_TIM3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  TIM2->CCR3=300;
  TIM3->CCR1=300;



  HD44780_Init(4);
  HD44780_Clear();
  HD44780_SetCursor(0,0);
  HD44780_PrintStr("HELLO");
  HD44780_SetCursor(10,1);
  HD44780_PrintStr("WORLD");
  HD44780_SetCursor(0,3);
  HD44780_PrintStr("____________________");
  HAL_Delay(1000);

   
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {




    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  j++;

	  GPIO_state1 =HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);

	  GPIO_state2 =HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4);

	  GPIO_state3=HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0);


	 	/////////////////////////////////////////////////////////////////////////////////////////////////

	  if(GPIO_state2 == GPIO_PIN_SET && GPIO_state1 == GPIO_PIN_RESET && GPIO_state3 == GPIO_PIN_RESET){
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);		//DESTRA
		  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);  	//SINISTRA
		  TIM2->CCR3=450;
		  TIM3->CCR1=450;
		  stampaSArrow(j%4);
		  HAL_Delay(200);
		  HD44780_Clear();

	  }
	  else if(GPIO_state2 == GPIO_PIN_SET && GPIO_state1 == GPIO_PIN_SET && GPIO_state3 == GPIO_PIN_RESET){


			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);				// RUOTA DESTRA  NORMALE
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);				// CAMBIO VERSO RUOTA SINISTRA
			  TIM3->CCR1 = 450;													//RUOTA DX
			  TIM2->CCR3 = 350;

		  isgoingveryleft = false;
		  isgoingveryright = false;
		  stampalArrow(j%4);
		  HAL_Delay(200);
		  HD44780_Clear();

	  }
	  else if(GPIO_state2 == GPIO_PIN_RESET && GPIO_state1 == GPIO_PIN_SET && GPIO_state3 == GPIO_PIN_RESET){

			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);				// RUOTA DESTRA  NORMALE
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);	            // RUOTA SINISTRA INDIETRO

			  TIM3->CCR1 = 500;
			  TIM2->CCR3 = 400;
			  stampalArrow(j%4);
			  HAL_Delay(200);
			  HD44780_Clear();
		  isgoingveryleft = true;
	  }
	  else if(GPIO_state2 == GPIO_PIN_SET && GPIO_state1 == GPIO_PIN_RESET && GPIO_state3 == GPIO_PIN_SET){

			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);				//CAMBIO VERSO RUOTA DESTRA
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);  			//SINISTRA NORMALE

			  TIM3->CCR1 = 350;
			  TIM2->CCR3 = 450;
			  stamparArrow(j%4);
			  HAL_Delay(200);
			  HD44780_Clear();
		  isgoingveryright = false;
		  isgoingveryleft = false;

	  }
	  else if (GPIO_state2 == GPIO_PIN_RESET && GPIO_state1 == GPIO_PIN_RESET && GPIO_state3 == GPIO_PIN_SET){


			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);				//RUOTA DESTRA
			 HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);  			//SINISTRA NORMALE

			 TIM3->CCR1 = 400;
			 TIM2->CCR3 = 500;
			 stamparArrow(j%4);
			  HAL_Delay(200);
			  HD44780_Clear();
		  isgoingveryright = true;
	  }
	  else if(GPIO_state2 == GPIO_PIN_RESET && GPIO_state1 == GPIO_PIN_RESET && GPIO_state3 == GPIO_PIN_RESET){
		  if(isgoingveryleft){
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);				// RUOTA DESTRA  NORMALE
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);	            // RUOTA SINISTRA INDIETRO
			  TIM3->CCR1 = 600;
			  TIM2->CCR3 = 400;
			  stampalArrow(j%4);
			  HAL_Delay(200);
			  HD44780_Clear();

		  }
		  if(isgoingveryright){
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);				//RUOTA DESTRA
			  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);  			//SINISTRA NORMALE
		  	 TIM3->CCR1 = 400;
		  	 TIM2->CCR3 = 600;
		  	 stamparArrow(j%4);
		  	 HAL_Delay(200);
		  	HD44780_Clear();

		   }
	  }
	  HAL_Delay(1);

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 8400;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 9999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 800;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 8400;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 800;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart2.Init.BaudRate = 9600;
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA4 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void stampa(int elem[],time_t now,int moob2[]){

	for(int i=0;i<23;i++){
		if(elem[i]==1&&i<20){
		    	HD44780_SetCursor(i,2);
		    	HD44780_PrintSpecialChar(7);
		}
		if(moob2[i]==1&&i<20){
				    	HD44780_SetCursor(i,0);
				    	HD44780_PrintSpecialChar(0);
				}
		if(i<22 /*&& elem[i]==1 && elem[i+1]==0*/){
			elem[i]=elem[i+1];
			moob2[i]=moob2[i+1];
		}
	}

	elem[22]=0;
	moob2[22]=0;

}
void stamparArrow(int j){

	  HD44780_SetCursor(15+j,1);
	  HD44780_PrintSpecialChar(7);
	  HD44780_SetCursor(15+j,2);
	  HD44780_PrintSpecialChar(6);
	  HD44780_SetCursor(14+j,1);
	  HD44780_PrintSpecialChar(5);
	  HD44780_SetCursor(14+j,2);
	  HD44780_PrintSpecialChar(5);
	  HD44780_SetCursor(13+j,0);
	  HD44780_PrintSpecialChar(7);
	  HD44780_SetCursor(13+j,1);
	  HD44780_PrintSpecialChar(5);
	  HD44780_SetCursor(13+j,2);
	  HD44780_PrintSpecialChar(5);
	  HD44780_SetCursor(13+j,3);
	  HD44780_PrintSpecialChar(6);
	  HD44780_SetCursor(12+j,0);
	  HD44780_PrintSpecialChar(5);
	  HD44780_SetCursor(12+j,1);
	  HD44780_PrintSpecialChar(5);
	  HD44780_SetCursor(12+j,2);
	  HD44780_PrintSpecialChar(5);
	  HD44780_SetCursor(12+j,3);
	  HD44780_PrintSpecialChar(5);
	  HD44780_SetCursor(11+j,1);
	  HD44780_PrintSpecialChar(5);
	  HD44780_SetCursor(11+j,2);
	  HD44780_PrintSpecialChar(5);
	  HD44780_SetCursor(10+j,1);
	  HD44780_PrintSpecialChar(5);
	  HD44780_SetCursor(10+j,2);
	  HD44780_PrintSpecialChar(5);
	  HD44780_SetCursor(9+j,1);
	  HD44780_PrintSpecialChar(5);
	  HD44780_SetCursor(9+j,2);
	  HD44780_PrintSpecialChar(5);


}

void stampalArrow(int j){

	  HD44780_SetCursor(4-j,1);
	  HD44780_PrintSpecialChar(7);
	  HD44780_SetCursor(4-j,2);
	  HD44780_PrintSpecialChar(6);
	  HD44780_SetCursor(5-j,1);
	  HD44780_PrintSpecialChar(5);
	  HD44780_SetCursor(5-j,2);
	  HD44780_PrintSpecialChar(5);
	  HD44780_SetCursor(6-j,0);
	  HD44780_PrintSpecialChar(7);
	  HD44780_SetCursor(6-j,1);
	  HD44780_PrintSpecialChar(5);
	  HD44780_SetCursor(6-j,2);
	  HD44780_PrintSpecialChar(5);
	  HD44780_SetCursor(6-j,3);
	  HD44780_PrintSpecialChar(6);
	  HD44780_SetCursor(7-j,0);
	  HD44780_PrintSpecialChar(5);
	  HD44780_SetCursor(7-j,1);
	  HD44780_PrintSpecialChar(5);
	  HD44780_SetCursor(7-j,2);
	  HD44780_PrintSpecialChar(5);
	  HD44780_SetCursor(7-j,3);
	  HD44780_PrintSpecialChar(5);
	  HD44780_SetCursor(8-j,1);
	  HD44780_PrintSpecialChar(5);
	  HD44780_SetCursor(8-j,2);
	  HD44780_PrintSpecialChar(5);
	  HD44780_SetCursor(9-j,1);
	  HD44780_PrintSpecialChar(5);
	  HD44780_SetCursor(9-j,2);
	  HD44780_PrintSpecialChar(5);
	  HD44780_SetCursor(10-j,1);
	  HD44780_PrintSpecialChar(5);
	  HD44780_SetCursor(10-j,2);
	  HD44780_PrintSpecialChar(5);


}

int verifica(int elem[],int A,int deel,int moob2[]){
	if((elem[1]==1 && A==1) ||(moob2[1]==1 && A==0) ){
		HD44780_Clear();
		HD44780_SetCursor(10,0);
		HD44780_PrintStr("GAME");
		HD44780_SetCursor(10,1);
		HD44780_PrintStr("OVER");
		HAL_Delay (2000);
		for(int i=0;i<23;i++){
			elem[i]=0;
			moob2[i]=0;
		}
		deel=0;
	}
	return 1;
}

void spersonaggio(int A){
						  HD44780_SetCursor(0,A);
	    	        	  HD44780_PrintSpecialChar(1);
	    	        	  HD44780_SetCursor(1,A);
	    	        	  HD44780_PrintSpecialChar(2);
	    	        	  HD44780_SetCursor(2,A);
	    	        	  HD44780_PrintSpecialChar(3);
	    	        	  HD44780_SetCursor(0,A+1);
	    	        	  HD44780_PrintSpecialChar(4);
	    	        	  HD44780_SetCursor(1,A+1);
	    	        	  HD44780_PrintSpecialChar(5);
	    	        	  HD44780_SetCursor(2,A+1);
	    	        	  HD44780_PrintSpecialChar(6);


}
void stampaSArrow(int j){
						  HD44780_SetCursor(9,2-j);
	    	        	  HD44780_PrintSpecialChar(0);
	    	        	  HD44780_SetCursor(10,2-j);
	    	        	  HD44780_PrintSpecialChar(1);
	    	        	  HD44780_SetCursor(9,3-j);
	    	        	  HD44780_PrintSpecialChar(5);
	    	        	  HD44780_SetCursor(10,3-j);
	    	        	  HD44780_PrintSpecialChar(5);
	    	        	  HD44780_SetCursor(8,4-j);
	    	        	  HD44780_PrintSpecialChar(0);
	    	        	  HD44780_SetCursor(9,4-j);
	    	        	  HD44780_PrintSpecialChar(5);
	    	        	  HD44780_SetCursor(10,4-j);
	    	        	  HD44780_PrintSpecialChar(5);
	    	        	  HD44780_SetCursor(11,4-j);
	    	        	  HD44780_PrintSpecialChar(1);
	    	        	  HD44780_SetCursor(8,5-j);
	    	        	  HD44780_PrintSpecialChar(5);
	    	        	  HD44780_SetCursor(9,5-j);
	    	        	  HD44780_PrintSpecialChar(5);
	    	        	  HD44780_SetCursor(10,5-j);
	    	        	  HD44780_PrintSpecialChar(5);
	    	        	  HD44780_SetCursor(11,5-j);
	    	        	  HD44780_PrintSpecialChar(5);

}
int updatea(int elem[],int c1){

    int y=0;
    y = rand()%6;
	  //printf("numero random : %d",y);
	  if(c1==5){
      //printf("condizione ...\n");
      c1=0;
      switch(y){
       case 0:
          elem[20]=0;
          elem[21]=1;
          elem[22]=0;
       break;

        case 1:
          elem[20]=1;
          elem[21]=1;
          elem[22]=0;
        break;

        case 2:
          elem[20]=1;
          elem[21]=1;
          elem[22]=1;
        break;
        case 3:
          elem[20]=1;
          elem[21]=0;
          elem[22]=1;
        break;
        case 4:
          elem[20]=0;
          elem[21]=1;
          elem[22]=1;
       break;
       case 5:
          elem[20]=1;
          elem[21]=0;
          elem[22]=0;
       break;
       default:
          elem[20]=0;
          elem[21]=0;
          elem[22]=0;
      }
	 }
    return c1;
}

int updatea2(int elem[],int c2,int moob[]){



    for(int i=0;i<16;i++){
        if(elem[i]==0 && elem[i+1]==0 && elem[i+2]==0){
          moob[i]=0;
          moob[i+1]=1;
          moob[i+2]=0;
        }
        if(elem[i]==1&&moob[i]==1){
          moob[i]=0;
        }

    }
    return c2;
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
