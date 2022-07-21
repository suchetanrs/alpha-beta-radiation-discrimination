/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "stdio.h"
#include "string.h"
#include "lcd_txt.h"

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
 ADC_HandleTypeDef hadc1;

TIM_HandleTypeDef htim1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

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
	uint raw;
	uint adc_result;
	char alphabeta[100];
	uint loopcounter;
	char uart_buf[100];
	int uart_buf_len;
	int digi_val;
	uint16_t signal_above_threshold_flag;
	uint16_t signal_below_threshold_flag;

	float signal_moved_above_time;
	float signal_moved_below_time;

	float timer_val_start;
	float timer_val_end;

	uint16_t counter_flag;

	uint16_t threshold_counter;

	uint16_t amplitude_bw_LLD_ULD;
	uint16_t amplitude_greaterthan_ULD;
	uint16_t width_status;

	uint16_t no_of_alpha;
	uint16_t no_of_beta;


	float user_threshold_value;
	uint16_t max_amp;

	double pulse_width;

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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
lcd_init();
loopcounter=0;
counter_flag=0;
threshold_counter=0;
user_threshold_value=2048;

signal_above_threshold_flag=10;
signal_below_threshold_flag=10;

signal_moved_below_time=10;
signal_moved_above_time=10;

pulse_width=0;

no_of_alpha=0;
no_of_beta=0;
max_amp=0;

//start timer
HAL_TIM_Base_Start(&htim1);
//get current counter
timer_val_start = __HAL_TIM_GET_COUNTER(&htim1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  //**********************ADC READING START*****************************

	  //Get ADC Value
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1,HAL_MAX_DELAY);
	  raw = HAL_ADC_GetValue(&hadc1);
	  HAL_Delay(20);

	  adc_result=raw;

	  //**********************ADC READING END***********************
	  loopcounter=loopcounter+1;

	  //**********************DIGITAL READING***********************

	  digi_val=HAL_GPIO_ReadPin (GPIOA, GPIO_PIN_2);


	  //**********************DIGITAL READING END*******************

	  //**********************Measuring pulse amplitude*************
	  if(adc_result>max_amp){
		  max_amp=adc_result;
	  }


	  //**********************Measuring pulse width*****************

	  	  //if(digi_val==0){
	  	    if(adc_result<410){
	  		signal_below_threshold_flag=1;
	  	  }

	  	  if(signal_below_threshold_flag==1){
	  		  //if(digi_val==1){
	  		  if(adc_result>410){
	  			  signal_moved_above_time = __HAL_TIM_GET_COUNTER(&htim1);
	  			  signal_above_threshold_flag=1;
	  			  signal_below_threshold_flag=0;
	  		  }
//	  		  else{
//				sprintf(alphabeta, "Signal: Below LLD \r \n");
//
//				uart_buf_len = sprintf(uart_buf, "PA < LLD \r\n");
//				lcd_puts(1,0,(int8_t*)uart_buf);
//	  		  }
	  	  }
	  		if(signal_above_threshold_flag==1){
	  			//if(digi_val==0){
	  			if(adc_result<410){
	  			  signal_moved_below_time = __HAL_TIM_GET_COUNTER(&htim1);
	  			  signal_below_threshold_flag=1;
	  			  signal_above_threshold_flag=0;
	  			  pulse_width = (signal_moved_below_time - signal_moved_above_time)/100;

	  			  uart_buf_len = sprintf(uart_buf, "PW %f \r\n", pulse_width);
	  			  //HAL_UART_Transmit(&huart1,(uint8_t*)uart_buf,strlen(uart_buf),HAL_MAX_DELAY);
	  			  lcd_puts(0,0,(int8_t*)uart_buf);

	  			  uart_buf_len = sprintf(uart_buf, "Max Amp %u \r\n", max_amp);
	  			  HAL_UART_Transmit(&huart1,(uint8_t*)uart_buf,strlen(uart_buf),HAL_MAX_DELAY);

	  			  //uart_buf_len = sprintf(uart_buf, "PA %f \r\n", max_amp);
	  			  //lcd_puts(1,0,(int8_t*)uart_buf);

	  	  		if(max_amp>2048){
	  	  			if(pulse_width>0.5){
	  	  				sprintf(alphabeta, "Signal: Alpha \r \n");
	  	  				no_of_alpha=no_of_alpha+1;
	  	  			}
	  	  			if(pulse_width<0.5){
	  	  				sprintf(alphabeta, "Signal: Beta \r \n");
	  	  				no_of_beta=no_of_beta+1;
	  	  			}

	  	  			uart_buf_len = sprintf(uart_buf, "PA > ULD \r\n");
	  	  			lcd_puts(1,0,(int8_t*)uart_buf);
	  	  		}
	  	  		//if(max_amp<410){
	  	  		else if(max_amp>410){
	  	  			if(max_amp<2048){

	  	  			sprintf(alphabeta, "SIGNAL: Beta \r \n");
	  	  			no_of_beta=no_of_beta+1;

	  	  			uart_buf_len = sprintf(uart_buf, "LLD<PA<ULD \r\n");
	  	  			lcd_puts(1,0,(int8_t*)uart_buf);
	  	  			}
	  	  		}

	  	  		max_amp=0;
			  }
		  }

	  //**********************Measuring pulse width ended**********************

	  uart_buf_len = sprintf(uart_buf, alphabeta);
	  //HAL_UART_Transmit(&huart1,(uint8_t*)uart_buf,strlen(uart_buf),HAL_MAX_DELAY);

	  //uart_buf_len = sprintf(uart_buf, "Digi %u \r\n", digi_val);

      //lcd_puts(0,0,(int8_t*)uart_buf);

      timer_val_end = __HAL_TIM_GET_COUNTER(&htim1);
      if((timer_val_end - timer_val_start) > 100){
    	  timer_val_start=__HAL_TIM_GET_COUNTER(&htim1);
    	  uart_buf_len = sprintf(uart_buf, "No of Beta pulse/sec: %u \r \n",no_of_beta);
    	  HAL_UART_Transmit(&huart1,(uint8_t*)uart_buf,strlen(uart_buf),HAL_MAX_DELAY);

    	  uart_buf_len = sprintf(uart_buf, "No of Alpha pulse/sec: %u \r \n",no_of_alpha);
    	  HAL_UART_Transmit(&huart1,(uint8_t*)uart_buf,strlen(uart_buf),HAL_MAX_DELAY);

    	  uart_buf_len = sprintf(uart_buf, "********************************** \r \n");
    	  HAL_UART_Transmit(&huart1,(uint8_t*)uart_buf,strlen(uart_buf),HAL_MAX_DELAY);

    	  no_of_alpha=0;
    	  no_of_beta=0;
      }
      //HAL_Delay();
      //lcd_clear();
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 64000 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65536 - 1;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  huart1.Init.BaudRate = 115200;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 PB12 PB13
                           PB14 PB15 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13
                          |GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
