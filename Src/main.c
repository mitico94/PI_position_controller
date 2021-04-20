/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    UART/UART_HyperTerminal_TxPolling_RxIT/Src/main.c
 * @author  MCD Application Team
 * @brief   This sample code shows how to use UART HAL and LL APIs to transmit
 *          data in polling mode while receiving data in Interrupt mode, by mixing
 *          use of LL and HAL APIs;
 *          HAL driver is used to perform UART configuration,
 *          then TX transfer procedure is based on HAL APIs use (polling)
 *               RX transfer procedure is based on LL APIs use (IT)
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* corretti */




//#define Kp_position 2
//#define Ki_position 0.003
//
//#define Kp_RPM 2
//#define Ki_RPM 0.002
//
//#define Kp_current 3
//#define Ki_current 0.003

//#define Kp_position 4
//#define Ki_position 0.01
//
//#define Kp_RPM 7
//#define Ki_RPM 0.03
//
//#define Kp_current 5
//#define Ki_current 0.1

//
#define Kp_position 7
#define Ki_position 0.003

#define Kp_RPM 8
#define Ki_RPM 0.002

#define Kp_current 6
#define Ki_current 0.001





/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc2;

UART_HandleTypeDef hlpuart1;

OPAMP_HandleTypeDef hopamp2;
OPAMP_HandleTypeDef hopamp3;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim16;

/* USER CODE BEGIN PV */

/* Buffer used for transmission
uint8_t aTxStartMessage[] = "\n\r **** ciao ****\n\r Enter characters using keyboard ...\n\r";
uint8_t ubSizeToSend = sizeof(aTxStartMessage); */

uint8_t b = 'b';
uint8_t e = 'e';

/* Buffer used for reception */
uint8_t aRXBufferA[RX_BUFFER_SIZE];
uint8_t aRXBufferB[RX_BUFFER_SIZE];
__IO uint32_t uwNbReceivedChars = 0;
__IO uint32_t uwBufferReadyIndication = 0;
uint8_t *pBufferReadyForUser;
uint8_t *pBufferReadyForReception;

int16_t pulse;
float voltage;
uint32_t force;
uint16_t control;
uint16_t duty;
uint32_t buffer;

float rms = 0;
float value = 0;


/* Captured Values */
uint32_t uwIC2Value1 = 0;
uint32_t uwIC2Value2 = 0;
uint32_t uwDiffCapture = 0;

/* Capture index */
uint16_t uhCaptureIndex = 0;

/* Frequency Value */
uint32_t uwFrequency = 0;

/* Speed in Revloution per minute */
int16_t RPM_target = 0;
int16_t RPM_read = 0;

int16_t  error_RPM;
int16_t Integral_error_RPM=0;

/* Current */
int16_t current_target = 0;
int16_t current_read = 0;

int16_t  error_current;
int16_t Integral_error_current=0;

/* Position */
int16_t Encoder_position = 0;
int16_t error_position;
int16_t Position_target = 0;
int16_t Position_read = 0;
int16_t Integral_error_position;

uint16_t gu16_TIM2_OVC;
uint16_t raw;
uint8_t masage[100];

uint8_t stop_flag;

uint16_t previous_time = 0;
uint16_t current_time = 0;
uint16_t elapsed_time = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM16_Init(void);
static void MX_TIM2_Init(void);
static void MX_OPAMP2_Init(void);
static void MX_ADC2_Init(void);
static void MX_OPAMP3_Init(void);
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


	/* STM32G4xx HAL library initialization:
       - Configure the Flash prefetch
       - Systick timer is configured by default as source of time base, but user
         can eventually implement his proper time base source (a general purpose
         timer for example or other time source), keeping in mind that Time base
         duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and
         handled in milliseconds basis.
       - Set NVIC Group Priority to 4
       - Low Level Initialization
	 */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	/* Configure leds */
	BSP_LED_Init(LED2);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LPUART1_UART_Init();
  MX_TIM3_Init();
  MX_TIM16_Init();
  MX_TIM2_Init();
  MX_OPAMP2_Init();
  MX_ADC2_Init();
  MX_OPAMP3_Init();
  /* USER CODE BEGIN 2 */

	/*## Configure UART peripheral for reception process (using LL) ##########*/
	/* Initializes Buffer swap mechanism :
     2 physical buffers aRXBufferA and aRXBufferB (RX_BUFFER_SIZE length)
     Any data received will be stored in active buffer : the number max of 
     data received is RX_BUFFER_SIZE */
	pBufferReadyForReception = aRXBufferA;
	pBufferReadyForUser      = aRXBufferB;
	uwNbReceivedChars = 0;
	uwBufferReadyIndication = 0;
	pulse = 0;
	voltage = 0;
	/* Enable RXNE and Error interrupts */
	LL_USART_EnableIT_RXNE(LPUART1);
	LL_USART_EnableIT_ERROR(LPUART1);

	if(HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2) != HAL_OK)
	{
		/* Starting Error */
		Error_Handler();
	}

	if(HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_3) != HAL_OK)
	{
		/* Starting Error */
		Error_Handler();
	}

	/*##- Start OPAMP    #####################################################*/
	/* Enable OPAMP */
	if(HAL_OK != HAL_OPAMP_Start(&hopamp2))
	{
		Error_Handler();
	}

	if(HAL_OK != HAL_OPAMP_Start(&hopamp3))
		{
			Error_Handler();
		}

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10,1);
    // Calibrate The ADC On Power-Up For Better Accuracy

//    HAL_ADCEx_Calibration_Start(&hadc2);

	if(HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1) != HAL_OK)
	{
		/* Starting Error */
		Error_Handler();
	}

	if(HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1) != HAL_OK)
	{
		/* Starting Error */
		Error_Handler();
	}


	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		current_time = HAL_GetTick();
		elapsed_time = current_time - previous_time;
//		HAL_Delay(10);

		/* Transmission begin */
		if(HAL_UART_Transmit(&hlpuart1, &b, 1, 1000)!= HAL_OK)
		{
			/* Transfer error in transmission process */
			Error_Handler();
		}

		/* Send Duty Cycle */
//		if(HAL_UART_Transmit(&hlpuart1, &duty, 2, 1000)!= HAL_OK)
//		{
//			/* Transfer error in transmission process */
//			Error_Handler();
//		}

		if(HAL_UART_Transmit(&hlpuart1, &current_read, 2, 1000)!= HAL_OK)
		{
			/* Transfer error in transmission process */
			Error_Handler();
		}


//	    HAL_UART_Transmit(&hlpuart1, masage, sprintf(masage,"%d\n\r", buffer),1000);

		if(HAL_UART_Transmit(&hlpuart1, &Position_target, 2, 1000)!= HAL_OK)
		{
			/* Transfer error in transmission process */
			Error_Handler();
		}

		if(HAL_UART_Transmit(&hlpuart1, &Position_read, 2, 1000)!= HAL_OK)
		{
			/* Transfer error in transmission process */
			Error_Handler();
		}
//
		if(HAL_UART_Transmit(&hlpuart1, &RPM_read, 2, 1000)!= HAL_OK)
		{
			/* Transfer error in transmission process */
			Error_Handler();
		}

		if(HAL_UART_Transmit(&hlpuart1, &e, 1, 1000)!= HAL_OK)
		{
			/* Transfer error in transmission process */
			Error_Handler();
		}

		/* Checks if Buffer full indication has been set */
		if (uwBufferReadyIndication != 0)
		{
			/* Reset indication */
			uwBufferReadyIndication = 0;


			/* USART IRQ handler is not anymore routed to HAL_UART_IRQHandler() function
         and is now based on LL API functions use. 
         Therefore, use of HAL IT based services is no more possible : use TX HAL polling services */

			/* Increase Duty Cycle */
			if(*pBufferReadyForUser== 'i')	{
				Position_target += 45;
			}

			/* Decrease Duty Cycle */
			if(*pBufferReadyForUser== 'd')	{
				Position_target -= 45;
			}

			/* Data received Stop ('s'), Clockwise('c'), Counterclockwise('k') */
			if(*pBufferReadyForUser== 'x'|| *pBufferReadyForUser== 'c' || *pBufferReadyForUser== 'k')
			{

				/* Electric Brake for 6 */
				if(*pBufferReadyForUser== 'x')	{
					pulse = 0;
					RPM_target = 0;
					RPM_read = 0;
					Position_target = 0;
					Position_read = 0;
					Encoder_position = 0;
					current_read = 0;
					current_target = 0;
					duty = 0;
					stop_flag = 1;
					elapsed_time = 0;
					previous_time = 0;
					current_time = 0;
					error_position = 0;
					Integral_error_RPM = 0;
					error_current = 0;
					error_RPM = 0;
					Integral_error_current = 0;
					Integral_error_position = 0;

				}

				/* Clockwise */
				if(*pBufferReadyForUser == 'c')
				{
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
				}

				/* Counterclockwise */
				if(*pBufferReadyForUser== 'k')	{
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
				}
			}
			/* Toggle LED2 */
			BSP_LED_Toggle(LED2);
		}



		Position_read = (Encoder_position);
		error_position = (Position_target*24) - Position_read;
		Integral_error_position += error_position*elapsed_time;
		/* Sum proportional and integral part Kp = 10 and Ki = 0.0001  */
		RPM_target += Kp_position*error_position + Ki_position*Integral_error_position;

		if(RPM_target>25)	{
			RPM_target = 25;
		}
		else if(RPM_target<-25)	{
			RPM_target = -25;
		}

		/* Speed error between speed target and real speed of the system */
		error_RPM = RPM_target - RPM_read;
		/* Error for ingeral part */
		Integral_error_RPM += error_RPM*elapsed_time;
		/* Sum proportional and integral part Kp = 10 and Ki = 0.0001  */
		current_target += Kp_RPM*error_RPM + Ki_RPM*Integral_error_RPM;

		if(current_target>1000)	{
			current_target = 1000;
		}
		if(current_target<-1000)	{
			current_target = -1000;
		}
		rms = 0;

		/* Get ADC value */
		/* Slow decay create 0 V */
		for(uint8_t j=0; j<100; j++){

//			HAL_Delay(2);
			HAL_ADC_Start(&hadc2);
			HAL_ADC_PollForConversion(&hadc2, HAL_MAX_DELAY);

			value = HAL_ADC_GetValue(&hadc2);
			value = (value)/1240;
			value = value*value;
			rms = rms + value;
		}

	    voltage = sqrt(rms)/10;
		/* 375 = 3.03 * 1000 / 8 */
		current_read = (voltage * 380);

		if(RPM_read<0)	{
			current_read = current_read*(-1);
		}

		/* Speed error between speed target and real speed of the system */
		error_current = current_target - current_read;
		Integral_error_current += error_current*elapsed_time;
		pulse += Kp_current*error_current + Ki_current*Integral_error_current;

		if(pulse>2150)	{
			pulse = 2200;
		}
		if(pulse<-2150)	{
			pulse = -2200;
		}

		/* Saturation */

		if(pulse<0)	{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 1);
			duty = pulse*(-1);
		}
		else	{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, 0);
			duty = pulse;
		}

		if(error_position < 600 && error_position > - 600)	{
			duty = duty - 700;
		}


		if(error_position < 300 && error_position > - 300)	{
			duty = duty - 300;
		}

		if(pulse == 0 || stop_flag == 1 || (error_position < 10 && error_position > -10))  {
			duty = 0;
			stop_flag = 0;
			elapsed_time = 0;
			current_time = 0;
			previous_time = 0;

		}

		__HAL_TIM_SET_COMPARE(&htim16, TIM_CHANNEL_1, duty);

		previous_time = current_time;
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

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_LPUART1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Lpuart1ClockSelection = RCC_LPUART1CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VOPAMP3_ADC2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_9B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_EVEN;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief OPAMP2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP2_Init(void)
{

  /* USER CODE BEGIN OPAMP2_Init 0 */

  /* USER CODE END OPAMP2_Init 0 */

  /* USER CODE BEGIN OPAMP2_Init 1 */

  /* USER CODE END OPAMP2_Init 1 */
  hopamp2.Instance = OPAMP2;
  hopamp2.Init.PowerMode = OPAMP_POWERMODE_NORMAL;
  hopamp2.Init.Mode = OPAMP_STANDALONE_MODE;
  hopamp2.Init.InvertingInput = OPAMP_INVERTINGINPUT_IO0;
  hopamp2.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO0;
  hopamp2.Init.InternalOutput = DISABLE;
  hopamp2.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp2.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP2_Init 2 */

  /* USER CODE END OPAMP2_Init 2 */

}

/**
  * @brief OPAMP3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_OPAMP3_Init(void)
{

  /* USER CODE BEGIN OPAMP3_Init 0 */

  /* USER CODE END OPAMP3_Init 0 */

  /* USER CODE BEGIN OPAMP3_Init 1 */

  /* USER CODE END OPAMP3_Init 1 */
  hopamp3.Instance = OPAMP3;
  hopamp3.Init.PowerMode = OPAMP_POWERMODE_NORMAL;
  hopamp3.Init.Mode = OPAMP_PGA_MODE;
  hopamp3.Init.NonInvertingInput = OPAMP_NONINVERTINGINPUT_IO2;
  hopamp3.Init.InternalOutput = ENABLE;
  hopamp3.Init.TimerControlledMuxmode = OPAMP_TIMERCONTROLLEDMUXMODE_DISABLE;
  hopamp3.Init.PgaConnect = OPAMP_PGA_CONNECT_INVERTINGINPUT_IO0;
  hopamp3.Init.PgaGain = OPAMP_PGA_GAIN_8_OR_MINUS_7;
  hopamp3.Init.UserTrimming = OPAMP_TRIMMING_FACTORY;
  if (HAL_OPAMP_Init(&hopamp3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN OPAMP3_Init 2 */

  /* USER CODE END OPAMP3_Init 2 */

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

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1700;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  sConfigOC.Pulse = 1700;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
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
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 170;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
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
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 0;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 2200;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim16, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim16, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */
  HAL_TIM_MspPostInit(&htim16);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(DIRECTION_GPIO_Port, DIRECTION_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ENABLE_GPIO_Port, ENABLE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : ENCODER_B_Pin */
  GPIO_InitStruct.Pin = ENCODER_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ENCODER_B_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RESET_Pin */
  GPIO_InitStruct.Pin = RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : DIRECTION_Pin */
  GPIO_InitStruct.Pin = DIRECTION_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(DIRECTION_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ENABLE_Pin */
  GPIO_InitStruct.Pin = ENABLE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ENABLE_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	uint8_t negative_flag= 0;

	if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)	{
		if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_0) == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_4)){
			Encoder_position ++;
		}
		else	{
			Encoder_position --;
			negative_flag = 1;
		}
		if(uhCaptureIndex == 0)	{
			/* Get the 1st Input Capture value */
			uwIC2Value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
			uhCaptureIndex = 1;
			gu16_TIM2_OVC = 0;
		}
		else if(uhCaptureIndex == 1)	{
			/* Get the 2nd Input Capture value */
			uwIC2Value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);

			/* Capture computation */
			if (uwIC2Value2 > uwIC2Value1)	{
				uwDiffCapture = (uwIC2Value2 - uwIC2Value1);
			}
			else if (uwIC2Value2 < uwIC2Value1)	{
				/* 0xFFFF is max TIM1_CCRx value */
				uwDiffCapture = ((0xFFFF - uwIC2Value1) + uwIC2Value2) + 1;
				//				uwDiffCapture = ((0xFFFF * gu16_TIM2_OVC) + uwIC2Value2) - uwIC2Value1;
			}
			else	{
				/* If capture values are equal, we have reached the limit of frequency measures */
				Error_Handler();
			}

			/* Frequency computation: for this example TIMx (TIM1) is clocked by APB2Clk */
			uwFrequency = 1000000/ uwDiffCapture;
			RPM_read = (uwFrequency*60)/(32*131);
			if(negative_flag == 1){
				RPM_read = RPM_read*(-1);
			}
			uhCaptureIndex = 0;
		}
	}
}



//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	gu16_TIM2_OVC++;
//}

/**
 * @brief  Rx Transfer completed callback
 * @note   This example shows a simple way to report end of IT Rx transfer, and
 *         you can add your own implementation.
 * @retval None
 */
void UART_CharReception_Callback(void)
{
	uint8_t *ptemp;

	/* Read Received character. RXNE flag is cleared by reading of RDR register */
	pBufferReadyForReception[uwNbReceivedChars++] = LL_USART_ReceiveData8(LPUART1);

	/* Checks if Buffer full indication has to be set */
	if (uwNbReceivedChars >= RX_BUFFER_SIZE)
	{
		/* Set Buffer swap indication */
		uwBufferReadyIndication = 1;

		/* Swap buffers for next bytes to be received */
		ptemp = pBufferReadyForUser;
		pBufferReadyForUser = pBufferReadyForReception;
		pBufferReadyForReception = ptemp;
		uwNbReceivedChars = 0;
	}
}

//void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
//{
//	uint8_t i;
//	uint32_t value=0;
//	uint32_t rms = 0;
//
//    // Read & Update The ADC Result
////	for(i=0; i<100; i++){
////
////		value = ((HAL_ADC_GetValue(&hadc2))*3.3)/(4095);
//////		HAL_Delay(1);
////		value = value * value;
////		rms = value + rms;
////	}
////    voltage = sqrt(rms)/10;
//}


//uint16_t PI_Target(uint16_t Read, uint16_t Target)
//{
//	uint16_t  Bias;
//	uint16_t Last_bias=0;
//	uint16_t PI=0;
//
//	Bias = Read - Target;
//	pulse += Kp*(Bias-Last_bias)+ Ki*Bias;
//	Last_bias = Bias;
//	return PI;
//}

/**
 * @brief  UART error callbacks
 * @note   This example shows a simple way to report transfer error, and you can
 *         add your own implementation.
 * @retval None
 */
void UART_Error_Callback(void)
{
	__IO uint32_t isr_reg;

	/* Disable USARTx_IRQn */
	NVIC_DisableIRQ(LPUART1_IRQn);

	/* Error handling example :
    - Read USART ISR register to identify flag that leads to IT raising
    - Perform corresponding error handling treatment according to flag
	 */
	isr_reg = LL_USART_ReadReg(LPUART1, ISR);
	if (isr_reg & LL_USART_ISR_NE)
	{
		/* Turn LED2 on: Transfer error in reception/transmission process */
		BSP_LED_On(LED2);
	}
	else
	{
		/* Turn LED2 on: Transfer error in reception/transmission process */
		BSP_LED_On(LED2);
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
	/* Turn LED2 to on for error */
	BSP_LED_On(LED2);
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
