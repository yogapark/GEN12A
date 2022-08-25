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
#include <stdio.h>
#include "ssd1306.h"
#include "ssd1306_tests.h"
//#include "OLED_SSD1306.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define INPUT_CURRENT_MAX 950 //920
#define CELL1_VOLTAGE_MIN 710 // margine 20
#define CELL2_VOLTAGE_MIN 696
#define CELLS_VOLTAGE_MIN 1406
#define OUTPUT_CURRENT_MAX 1950  //1940
#define OUTPUT_VOLTAGE_MAX 2890  //2850
#define OUTPUT_VOLTAGE_MIN 2750  //2800
#define OUTPUT_VOLTAGE_DELAY 2000  //3000
#define OUTPUT_CURRENT_FAN_ON 600


#define VERSION 100
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
volatile uint16_t adcval[8];
char RcvData[32];
char* currentline;
uint32_t adc_tick=0;
uint32_t error_para = 0;
uint32_t error_count = 0;

uint16_t InputCurrent = 0;
uint16_t Cell1Voltage = 0;
uint16_t Cell2Voltage = 0;
uint16_t CellsVoltage = 0;

uint16_t OutputVoltage = 0;
uint16_t OutputCurrent = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __cplusplus
extern "C" int _write(int32_t file, uint8_t *ptr, int32_t len) {
#else
int _write(int32_t file, uint8_t *ptr, int32_t len) {
#endif
    if( HAL_UART_Transmit(&huart1, ptr, len, len) == HAL_OK ) return len;
    else return 0;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
  int cnt = 0;
  currentline = RcvData;
  uint32_t adc_start_tick= 0,uiFramRate = 0;
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
  MX_DMA_Init();
  MX_I2C1_Init();
  MX_USART1_UART_Init();
  MX_ADC_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_PWM_Start(&htim3 , TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim3 , TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim3 , TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3 , TIM_CHANNEL_4);
  HAL_TIM_Base_Start_IT(&htim3); // 10ms interrupt
  TIM3->CCR1 = 0;

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
  HAL_Delay(20);

  ssd1306_Init();

  HAL_ADCEx_Calibration_Start(&hadc);

  HAL_GPIO_WritePin(SW_LED_GPIO_Port, SW_LED_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(CH1_GREEN_GPIO_Port, CH1_GREEN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(CH1_RED_GPIO_Port, CH1_RED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CH2_GREEN_GPIO_Port, CH2_GREEN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(CH2_RED_GPIO_Port, CH2_RED_Pin, GPIO_PIN_RESET);

  HAL_Delay(500);

  HAL_GPIO_WritePin(SW_LED_GPIO_Port, SW_LED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CH1_GREEN_GPIO_Port, CH1_GREEN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CH1_RED_GPIO_Port, CH1_RED_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(CH2_GREEN_GPIO_Port, CH2_GREEN_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CH2_RED_GPIO_Port, CH2_RED_Pin, GPIO_PIN_SET);

  HAL_Delay(500);

  HAL_GPIO_WritePin(SW_LED_GPIO_Port, SW_LED_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(CH1_GREEN_GPIO_Port, CH1_GREEN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(CH1_RED_GPIO_Port, CH1_RED_Pin, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(CH2_GREEN_GPIO_Port, CH2_GREEN_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(CH2_RED_GPIO_Port, CH2_RED_Pin, GPIO_PIN_RESET);

  HAL_Delay(500);

  error_count = HAL_GetTick() ;


//  HAL_GPIO_WritePin(OUT_EN_GPIO_Port, OUT_EN_Pin, GPIO_PIN_SET);
//  HAL_GPIO_WritePin(OUT_EN_GPIO_Port, OUT_EN_Pin, GPIO_PIN_RESET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//    ssd1306_TestAll();
    adc_start_tick = HAL_GetTick();
    HAL_ADC_Start_DMA(&hadc, &adcval[0], 7);



#if 0
	ssd1306_Fill(Black);

    ssd1306_SetCursor(2, 0);
    ssd1306_WriteString("[#ADC TEST@]", Font_6x8, White);

    ssd1306_SetCursor(10, 10);
	sprintf(currentline,"cs %dms",HAL_GetTick());
    ssd1306_WriteString(currentline, Font_6x8, White);
    ssd1306_SetCursor(10, 20);
	sprintf(currentline,"Fr %dms as %dms",uiFramRate,adc_tick-adc_start_tick);
    ssd1306_WriteString(currentline, Font_6x8, White);
    ssd1306_SetCursor(10, 34);
	sprintf(currentline,"%4d %4d %4d %4d",adcval[0],adcval[1],adcval[2],adcval[3]);
    ssd1306_WriteString(currentline, Font_6x8, White);
    ssd1306_SetCursor(10, 44);
	sprintf(currentline,"%4d %4d %4d",adcval[4],adcval[5],adcval[6]);
    ssd1306_WriteString(currentline, Font_6x8, White);

    ssd1306_UpdateScreen();

//#else

	ssd1306_Fill(Black);

	ssd1306_SetCursor(2, 0);
	sprintf(currentline,"[GEN_12A-V%d] %dms",VERSION,uiFramRate);
    ssd1306_WriteString(currentline, Font_6x8, White);

    ssd1306_SetCursor(10, 10);
	sprintf(currentline,"IN_V : %4d> %4d",CellsVoltage,CELLS_VOLTAGE_MIN);
    ssd1306_WriteString(currentline, Font_6x8, White);
    ssd1306_SetCursor(10, 19);
	sprintf(currentline,"IN_A : %4d< %4d",InputCurrent,INPUT_CURRENT_MAX);
    ssd1306_WriteString(currentline, Font_6x8, White);
    ssd1306_SetCursor(10, 28);
	sprintf(currentline,"OUT_V: %4d> %4d",OutputVoltage,OUTPUT_VOLTAGE_MIN);
    ssd1306_WriteString(currentline, Font_6x8, White);
    ssd1306_SetCursor(10, 37);
	sprintf(currentline,"OUT_A: %4d< %4d",OutputCurrent,OUTPUT_CURRENT_MAX);
    ssd1306_WriteString(currentline, Font_6x8, White);

    ssd1306_SetCursor(2, 47);
    if( HAL_GPIO_ReadPin(OUT_EN_GPIO_Port,OUT_EN_Pin)== GPIO_PIN_SET )
    	ssd1306_WriteString("[OUT_EN]", Font_6x8, White);
    else
    	ssd1306_WriteString("[ OFF  ]", Font_6x8, White);

    ssd1306_SetCursor(56, 47);
    if( HAL_GPIO_ReadPin(FAN_GPIO_Port,FAN_Pin)== GPIO_PIN_SET )
    	ssd1306_WriteString("[FAN]", Font_6x8, White);
    else
    	ssd1306_WriteString("[off]", Font_6x8, White);

    ssd1306_SetCursor(2, 56);
    if( HAL_GPIO_ReadPin(SW_LED_GPIO_Port,SW_LED_Pin)== GPIO_PIN_SET )
    	ssd1306_WriteString("[SWLED]", Font_6x8, White);
    else
    	ssd1306_WriteString("[ off ]", Font_6x8, White);

    ssd1306_SetCursor(56, 56);
    if( HAL_GPIO_ReadPin(CH1_GREEN_GPIO_Port,CH1_GREEN_Pin)== GPIO_PIN_SET )
    	ssd1306_WriteChar('G',Font_6x8,White);
    if( HAL_GPIO_ReadPin(CH1_RED_GPIO_Port,CH1_RED_Pin)== GPIO_PIN_SET )
    	ssd1306_WriteChar('R',Font_6x8,White);

    ssd1306_SetCursor(74, 56);
    if( HAL_GPIO_ReadPin(CH2_GREEN_GPIO_Port,CH2_GREEN_Pin)== GPIO_PIN_SET )
    	ssd1306_WriteChar('G',Font_6x8,White);
    if( HAL_GPIO_ReadPin(CH2_RED_GPIO_Port,CH2_RED_Pin)== GPIO_PIN_SET )
    	ssd1306_WriteChar('R',Font_6x8,White);

    ssd1306_SetCursor(100, 47);
    sprintf(currentline,"%4d",Cell1Voltage); //,adcval[5]);
    ssd1306_WriteString(currentline, Font_6x8, White);

    ssd1306_SetCursor(100, 56);
    sprintf(currentline,"%4d",Cell2Voltage);
    ssd1306_WriteString(currentline, Font_6x8, White);

//    ssd1306_SetCursor(10, 56);
//	sprintf(currentline,"%4d %4d",adcval[5],adcval[6]);
//    ssd1306_WriteString(currentline, Font_6x8, White);

    ssd1306_UpdateScreen();

#endif

    HAL_Delay(10);

    printf("%u ms Fr:%dms iV:%d>%d iC:%d<%d oV:%d>%d oC:%d<%d c1:%d>%d c2:%d>%d ",
    		HAL_GetTick(),uiFramRate,CellsVoltage,CELLS_VOLTAGE_MIN,InputCurrent,INPUT_CURRENT_MAX,
			OutputVoltage,OUTPUT_VOLTAGE_MIN,OutputCurrent,OUTPUT_CURRENT_MAX,
			Cell1Voltage,CELL1_VOLTAGE_MIN,Cell2Voltage,CELL2_VOLTAGE_MIN);

    if( HAL_GPIO_ReadPin(OUT_EN_GPIO_Port,OUT_EN_Pin)== GPIO_PIN_SET )
    	printf("[OUT_EN]");
    else
    	printf("[ OFF  ]");

    if( HAL_GPIO_ReadPin(FAN_GPIO_Port,FAN_Pin)== GPIO_PIN_SET )
    	printf("[FAN]");
    else
    	printf("[off]");

    if( HAL_GPIO_ReadPin(SW_LED_GPIO_Port,SW_LED_Pin)== GPIO_PIN_SET )
    	printf("[SWLED]");
    else
    	printf("[ off ]");

    if( HAL_GPIO_ReadPin(CH1_GREEN_GPIO_Port,CH1_GREEN_Pin)== GPIO_PIN_SET )
    	printf("G" );
    if( HAL_GPIO_ReadPin(CH1_RED_GPIO_Port,CH1_RED_Pin)== GPIO_PIN_SET )
    	printf("R" );

    if( HAL_GPIO_ReadPin(CH2_GREEN_GPIO_Port,CH2_GREEN_Pin)== GPIO_PIN_SET )
    	printf("G");
    if( HAL_GPIO_ReadPin(CH2_RED_GPIO_Port,CH2_RED_Pin)== GPIO_PIN_SET )
    	printf("R" );





    if( OutputCurrent > OUTPUT_CURRENT_MAX )
    {
    	error_para = error_para | 1;
    }

    if( OutputVoltage > OUTPUT_VOLTAGE_MAX)
    {
//    	error_para = error_para | 2;
    }

    if( OutputVoltage <= OUTPUT_VOLTAGE_MIN )
    {
    	if(HAL_GetTick() - error_count >= OUTPUT_VOLTAGE_DELAY)
    	{
    		error_para = error_para | 4;
    	}
    }
    else
    {
    	error_count = HAL_GetTick() ;
    }

	printf(" E%d",error_para  );
    printf("\r\n");



//    printf("%u ms [ADC TEST] Fr= %dms as= %d ccr= %4d %4d %4d %4d\r\n",HAL_GetTick(),uiFramRate,adc_tick-adc_start_tick,TIM3->CCR1,TIM3->CCR2,TIM3->CCR3,TIM3->CCR4);


    uiFramRate = HAL_GetTick() - adc_start_tick;
//    HAL_Delay(1000);

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = DISABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_3;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

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
  hi2c1.Init.Timing = 0x0000020B;
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

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 7;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 9999;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

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
  huart1.Init.BaudRate = 38400;
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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  HAL_GPIO_WritePin(GPIOA, CH1_RED_Pin|CH1_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, CH2_RED_Pin|CH2_GREEN_Pin|FAN_Pin|SW_LED_Pin
                          |OUT_EN_Pin|OLEDreset_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CH1_RED_Pin CH1_GREEN_Pin */
  GPIO_InitStruct.Pin = CH1_RED_Pin|CH1_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : CH2_RED_Pin CH2_GREEN_Pin FAN_Pin SW_LED_Pin
                           OUT_EN_Pin OLEDreset_Pin */
  GPIO_InitStruct.Pin = CH2_RED_Pin|CH2_GREEN_Pin|FAN_Pin|SW_LED_Pin
                          |OUT_EN_Pin|OLEDreset_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	static uint32_t swLedtoggleTime=0;


	adc_tick = HAL_GetTick();

	InputCurrent = adcval[0];
	Cell1Voltage = adcval[3]-adcval[4];
	Cell2Voltage = adcval[4];
	CellsVoltage = adcval[3];

	OutputVoltage = adcval[2];
	OutputCurrent = adcval[1];

	// OUT_EN work
	if(( InputCurrent <= INPUT_CURRENT_MAX )&&( CellsVoltage >= CELLS_VOLTAGE_MIN )&&
			( OutputVoltage <= OUTPUT_VOLTAGE_MAX )&&( OutputCurrent <= OUTPUT_CURRENT_MAX ))
	{
		/* Check the parameters */
		// OutputVoltage or OutputCurrent error
		if( error_para >= 1 )
			 HAL_GPIO_WritePin(OUT_EN_GPIO_Port, OUT_EN_Pin, GPIO_PIN_RESET);
		else
			HAL_GPIO_WritePin(OUT_EN_GPIO_Port, OUT_EN_Pin, GPIO_PIN_SET);
		//  HAL_GPIO_WritePin(OUT_EN_GPIO_Port, OUT_EN_Pin, GPIO_PIN_RESET);
	}
	else
	{
		//  HAL_GPIO_WritePin(OUT_EN_GPIO_Port, OUT_EN_Pin, GPIO_PIN_SET);
		  HAL_GPIO_WritePin(OUT_EN_GPIO_Port, OUT_EN_Pin, GPIO_PIN_RESET);
	}

	// FAN work
	if(OutputCurrent >= OUTPUT_CURRENT_FAN_ON)
	{
		// do FAN
		HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_SET);
		//  HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_RESET);
	}
	else
	{
		// don't FAN
		//HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(FAN_GPIO_Port, FAN_Pin, GPIO_PIN_RESET);
	}

	// SW LED work
	if( HAL_GPIO_ReadPin(OUT_EN_GPIO_Port,OUT_EN_Pin)== GPIO_PIN_SET )
	{
		HAL_GPIO_WritePin(SW_LED_GPIO_Port, SW_LED_Pin, GPIO_PIN_SET);
	}
	else
	{
		if(HAL_GetTick()-swLedtoggleTime > 1000)
		{
			swLedtoggleTime = HAL_GetTick();
			HAL_GPIO_TogglePin(SW_LED_GPIO_Port, SW_LED_Pin);
		}
	}

	// LED ch1
	if(Cell1Voltage > CELL1_VOLTAGE_MIN)
	{
		// Green LED On, RED LED Off
		HAL_GPIO_WritePin(CH1_GREEN_GPIO_Port, CH1_GREEN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CH1_RED_GPIO_Port, CH1_RED_Pin, GPIO_PIN_SET);
	}
	else
	{
		// Green LED Off, RED LED On
		HAL_GPIO_WritePin(CH1_GREEN_GPIO_Port, CH1_GREEN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(CH1_RED_GPIO_Port, CH1_RED_Pin, GPIO_PIN_RESET);
	}

	// LED CH2
	if(Cell2Voltage > CELL2_VOLTAGE_MIN)
	{
		// Green LED On, RED LED Off
		HAL_GPIO_WritePin(CH2_GREEN_GPIO_Port, CH2_GREEN_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(CH2_RED_GPIO_Port, CH2_RED_Pin, GPIO_PIN_SET);
	}
	else
	{
		// Green LED Off, RED LED On
		HAL_GPIO_WritePin(CH2_GREEN_GPIO_Port, CH2_GREEN_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(CH2_RED_GPIO_Port, CH2_RED_Pin, GPIO_PIN_RESET);
	}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	static uint32_t count=0;
	uint32_t c1,c2,c3,c4;

	if(htim->Instance == TIM3) // 10ms interrupt
	{
		count += 5;
		c1 = TIM3->ARR/2; // half
		c2 = count % c1;  // increase helf
		c3 = c1 - c2;	  // decrease helf
		c4 = c1/4;

		TIM3->CCR1 = TIM3->ARR-c2;//c2;
		TIM3->CCR2 = TIM3->ARR-c3;//c3;
		if(c2 < c4)
		{
			TIM3->CCR3 = TIM3->ARR;
			TIM3->CCR4 = TIM3->ARR;
		}
		else if(c2 < (c4*2))
		{
			TIM3->CCR3 = TIM3->ARR;
			TIM3->CCR4 = 0;
		}
		else if(c2 < (c4*3))
		{
			TIM3->CCR3 = 0;
			TIM3->CCR4 = TIM3->ARR;
		}
		else
		{
			TIM3->CCR3 = 0;
			TIM3->CCR4 = 0;
		}
	}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
