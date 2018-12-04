
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include "filter.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define 			DEVICE_ID_DEF				0x0001
#define 			REMOTE_DEVICE_ID_DEF		0x0100
#define 			CAN_TIME_TX_DATA			1000							// ms
#define 			ADC_NUM_QUANT				4096
#define				ADC_REF_V					3.3								// Volt
#define				ADC_IN_R1					100000.0						// Ohm
#define				ADC_IN_R2					10000.0							// Ohm
#define				ADC_OUT_R1					100000.0						// Ohm
#define				ADC_OUT_R2					10000.0							// Ohm
#define				LIM_IN_V_DOWN_DEF			9.0								// Volt
#define				LIM_IN_V_UP_DEF				18.0							// Volt
#define				LIM_OUT_V_DOWN_DEF			9.0								// Volt
#define				LIM_OUT_V_UP_DEF			18.0							// Volt
#define				SENS_I_VCC					3.3								// Volt
#define				SENS_I_SLOPE				26.4e-3							// Volt per 1 Ampere
#define				SENS_I_LIM_DEF				1.0								// Ampere
#define				SENS_I_MAX_CURRENT			50.0							// Ampere, see datasheet
#define				SENS_TEMP_SLOPE				6.25e-3							// Volt per 1 degree
#define				SWITCH_QUANT				0.25							// Ampere

#define				ADC_IN_PRESCALER			(float)ADC_IN_R2 / (float)(ADC_IN_R1 + ADC_IN_R2)
#define				ADC_OUT_PRESCALER			(float)ADC_OUT_R2 / (float)(ADC_OUT_R1 + ADC_OUT_R2)
#define 			SENS_I_LIM_DEF_A			((float)SENS_I_LIM_DEF * (float)SENS_I_SLOPE)	// Ampere
#define				SENS_I_MAX_CURRENT_A		((float)SENS_I_MAX_CURRENT * (float)SENS_I_SLOPE)	// Ampere
#define				SENS_I_VOUT_Q				(float)SENS_I_VCC / 2.0
#define 			GATE_ENABLE					1
#define 			GATE_DISABLE				0
#define				ADC_SAMPLE_RATE				10000


uint32_t 			Device_ID = 				DEVICE_ID_DEF;					// Set device ID  (CAN bus)
uint32_t 			Remote_device_ID =			REMOTE_DEVICE_ID_DEF;			// Set remote device ID (CAN bus)
uint8_t				Gate_state =				GATE_DISABLE;
float 				Lim_in_V_down =				((float)ADC_IN_PRESCALER * (float)LIM_IN_V_DOWN_DEF);
float				Lim_in_V_up =				((float)ADC_IN_PRESCALER * (float)LIM_IN_V_UP_DEF);
float				Lim_out_V_down =			((float)ADC_OUT_PRESCALER * (float)LIM_OUT_V_DOWN_DEF);
float				Lim_out_V_up =				((float)ADC_OUT_PRESCALER * (float)LIM_OUT_V_UP_DEF);
float				Lim_I_up =					SENS_I_VOUT_Q + SENS_I_LIM_DEF_A;		// Forward current limit
float				Lim_I_down =				SENS_I_VOUT_Q - SENS_I_LIM_DEF_A;		// Backward current limit
CanTxMsgTypeDef 	CAN_Tx_Buffer;
CanRxMsgTypeDef 	CAN_Rx_Buffer;
uint16_t 			ADC_Data[4];
float	 			ADC_Data_f[4];



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_CAN_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void CAN_filter_set(void);
void CAN_ADCdata_send(uint16_t *ADC_array, uint8_t array_size);
void Gate_set(uint8_t state);
void Gate_toggle(void);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
uint8_t Switch_read(void);
//void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc1);
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* CanHandle);
void Set_current_lim(void);
void ADC_check(uint16_t *ADC_Data, float *ADC_Data_f);
void Sample_rate_set(uint16_t sample_rate);
void CAN_ADCdata_send_f(float *ADC_array_f, uint8_t array_size);


/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_CAN_Init();
  MX_ADC1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	
	Sample_rate_set(ADC_SAMPLE_RATE);

  Set_current_lim();
  CAN_filter_set();
  hcan.pRxMsg = &CAN_Rx_Buffer;
  HAL_CAN_Receive_IT(&hcan, CAN_FIFO0);
	HAL_NVIC_DisableIRQ(DMA1_Channel1_IRQn);
  HAL_ADC_Start_DMA(&hadc1,(uint32_t*) &ADC_Data, 4);
  HAL_TIM_Base_Start_IT(&htim3);
  
	//Gate_set(GATE_ENABLE);
  //HAL_ADCEx_InjectedStart_IT(&hadc1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
	  //if (ADC_conversion_status == ADC_CONVERSION_COMPLETE)
	  //{
		  CAN_ADCdata_send_f(ADC_Data_f, sizeof(ADC_Data)/sizeof(uint16_t));
		  //ADC_conversion_status = ADC_CONVERSION_EXPECT;
	  //}
	  HAL_Delay(CAN_TIME_TX_DATA);
		//Gate_set(GATE_ENABLE);
		//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
		//HAL_Delay(10);
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 4;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* CAN init function */
static void MX_CAN_Init(void)
{

  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 36;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SJW = CAN_SJW_1TQ;
  hcan.Init.BS1 = CAN_BS1_13TQ;
  hcan.Init.BS2 = CAN_BS2_2TQ;
  hcan.Init.TTCM = DISABLE;
  hcan.Init.ABOM = DISABLE;
  hcan.Init.AWUM = DISABLE;
  hcan.Init.NART = ENABLE;
  hcan.Init.RFLM = DISABLE;
  hcan.Init.TXFP = ENABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 71;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 50000;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

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

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA8 */
  GPIO_InitStruct.Pin = GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB4 PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 3, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

}

/* USER CODE BEGIN 4 */
void CAN_filter_set(void)
{
	CAN_FilterConfTypeDef CAN_filter_config;
	CAN_filter_config.FilterNumber = 			1;  							// 0 - not filtered data
	CAN_filter_config.FilterMode = 				CAN_FILTERMODE_IDLIST;
	CAN_filter_config.FilterScale = 			CAN_FILTERSCALE_16BIT;
	CAN_filter_config.FilterIdHigh = 			Device_ID << 5;
	CAN_filter_config.FilterIdLow = 			0x0;							// not use, reserved
	CAN_filter_config.FilterMaskIdHigh = 		0x0;							//
	CAN_filter_config.FilterMaskIdLow = 		0x0;							//
	CAN_filter_config.FilterFIFOAssignment = 	CAN_FILTER_FIFO0;
	CAN_filter_config.FilterActivation = 		ENABLE;
	HAL_CAN_ConfigFilter(&hcan, &CAN_filter_config);
}

void CAN_ADCdata_send(uint16_t *ADC_array, uint8_t array_size)
{
	uint8_t data_size = array_size * 2;
	uint8_t buff[data_size];

	uint8_t j = 0;
	for (int i = 0; i < data_size; i += 2)
	{
		buff[i] = (uint8_t)(ADC_array[j]);
		buff[i + 1] = (uint8_t)(ADC_array[j] >> 8);
		j++;
	}

	CAN_Tx_Buffer.RTR = 	CAN_RTR_DATA;						// Data frame
	CAN_Tx_Buffer.IDE = 	CAN_ID_STD;							// ID 11 bit
	CAN_Tx_Buffer.DLC = 	data_size;							// Data length
	CAN_Tx_Buffer.StdId = 	Remote_device_ID;

	for (int i = 0; i < data_size; i++)
	{
		CAN_Tx_Buffer.Data[i] = buff[i];
	}

	hcan.pTxMsg = &CAN_Tx_Buffer;
	HAL_CAN_Transmit_IT(&hcan);
}

void CAN_ADCdata_send_f(float *ADC_array_f, uint8_t array_size)
{
	volatile uint16_t ADC_array[array_size];
	uint8_t data_size = array_size * 2;
	uint8_t buff[data_size];

	for (int i = 0; i < data_size; i++)
	{
		ADC_array[i] = (uint16_t)(ADC_array_f[i] / (float)ADC_REF_V * (float)ADC_NUM_QUANT);
	}
	
	uint8_t j = 0;
	for (int i = 0; i < data_size; i += 2)
	{
		buff[i] = (uint8_t)(ADC_array[j]);
		buff[i + 1] = (uint8_t)(ADC_array[j] >> 8);
		j++;
	}

	CAN_Tx_Buffer.RTR = 	CAN_RTR_DATA;						// Data frame
	CAN_Tx_Buffer.IDE = 	CAN_ID_STD;							// ID 11 bit
	CAN_Tx_Buffer.DLC = 	data_size;							// Data length
	CAN_Tx_Buffer.StdId = 	Remote_device_ID;

	for (int i = 0; i < data_size; i++)
	{
		CAN_Tx_Buffer.Data[i] = buff[i];
	}

	hcan.pTxMsg = &CAN_Tx_Buffer;
	HAL_CAN_Transmit_IT(&hcan);
}

void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* CanHandle)
{
	if (CanHandle->pRxMsg->Data[0] == GATE_ENABLE)
	{
		Gate_set(GATE_ENABLE);
	} else if (CanHandle->pRxMsg->Data[0] == GATE_DISABLE)
		{
			Gate_set(GATE_DISABLE);
		}

	for(int i = 0; i < 8; i++)
	{
		CanHandle->pRxMsg->Data[i] = 0;
	}

	HAL_CAN_Receive_IT(CanHandle, CAN_FIFO0);
}

void Gate_set(uint8_t state)
{
	if (state == GATE_ENABLE)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
		Gate_state = GATE_ENABLE;
	} else if (state == GATE_DISABLE)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
			Gate_state = GATE_DISABLE;
		}
}

void Gate_toggle(void)
{
	if (Gate_state == GATE_ENABLE)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
		Gate_state = GATE_DISABLE;
	} else if (Gate_state == GATE_DISABLE)
		{
			HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
			Gate_state = GATE_ENABLE;
		}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	if (GPIO_Pin == GPIO_PIN_8)										// GPIOA PA8 Button
	{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_1);
		if (!(htim2.Instance->DIER & TIM_DIER_UIE))					// Protection, if TIM interrupt enable wont used HAL_TIM_Base_Start_IT
		{
			HAL_TIM_Base_Start_IT(&htim2);							// Enable TIM 50 ms, for eliminate the effect of "bounce contacts"
		}
	}
}


uint8_t Switch_read(void)
{
	uint8_t value = 0;
	GPIO_InitTypeDef GPIO_InitStruct;

	// 	-----  	GPIO enable PULL UP -----
	// Default GPIO initialization as PULL DOWN because switch has not current limited resistors
	// GPIO enable in PULL UP mode only in measurement time
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	// ----------------------------------------

	HAL_Delay(50);

	value  = (!HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15) & 0x1)	<< 0;
	value |= (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4)  & 0x1) 	<< 1;
	value |= (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_5)  & 0x1) 	<< 2;
	value |= (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6)  & 0x1) 	<< 3;
	value |= (!HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7)  & 0x1)  << 4;
	value |= (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13) & 0x1)  << 5;
	value |= (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) & 0x1)  << 6;
	value |= (!HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_15) & 0x1)  << 7;

	// 	-----  	GPIO enable PULL DOWN -----
	GPIO_InitStruct.Pin = GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLDOWN;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	// ----------------------------------------

	return value;
}

void Set_current_lim(void)
{
	float lim = 0;
	float sw_state = 0;

	sw_state = Switch_read();
	lim = sw_state * (float)SWITCH_QUANT;
	if (lim > SENS_I_MAX_CURRENT)
		{
			lim = SENS_I_MAX_CURRENT;
		}
	lim *= (float)SENS_I_SLOPE; 
	Lim_I_up = SENS_I_VOUT_Q + lim;
	Lim_I_down = SENS_I_VOUT_Q - lim;
}

void ADC_check(uint16_t *ADC_Data, float *ADC_Data_f)
{
	ADC_Data_f[0] = dFilter_1(ADC_Data[0] / (float)ADC_NUM_QUANT * (float)ADC_REF_V);
	ADC_Data_f[1] = dFilter_2(ADC_Data[1] / (float)ADC_NUM_QUANT * (float)ADC_REF_V);
	ADC_Data_f[2] = dFilter_3(ADC_Data[2] / (float)ADC_NUM_QUANT * (float)ADC_REF_V);
	ADC_Data_f[3] = ADC_Data[3] / (float)ADC_NUM_QUANT * (float)ADC_REF_V;

	if (((ADC_Data_f[0] < Lim_out_V_down) || (ADC_Data_f[0] > Lim_out_V_up)) && (Gate_state != GATE_DISABLE))
	{
				Gate_set(GATE_DISABLE);
	}
	if (((ADC_Data_f[1] < Lim_in_V_down) || (ADC_Data_f[1] > Lim_in_V_up)) && (Gate_state != GATE_DISABLE))
	{
				Gate_set(GATE_DISABLE);
	}
	if (((ADC_Data_f[2] < Lim_I_down) || (ADC_Data_f[2] > Lim_I_up)) && (Gate_state != GATE_DISABLE))
	{
				Gate_set(GATE_DISABLE);
	}
}

void Sample_rate_set(uint16_t sample_rate)
{
	TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
	
  HAL_TIM_Base_DeInit(&htim3);
	
	htim3.Instance = TIM3;
  htim3.Init.Prescaler = 71;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	
	//-----------------
	uint32_t clock = HAL_RCC_GetPCLK1Freq() * 2; // TIM3 clock = PCLK1 * 2  // MHz

	uint32_t prescaler_freq = clock /(htim3.Init.Prescaler + 1);  // MHz

	uint32_t period = (prescaler_freq/sample_rate) - 1;			// us

	htim3.Init.Period = period;
	//-----------------

  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}


/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
