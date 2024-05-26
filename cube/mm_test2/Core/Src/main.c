/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
#include "stdbool.h"

/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

typedef enum {
	DIST_FL,
	DIST_FR,
	DIST_L,
	DIST_R
	}dist_t;

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

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


uint16_t enc_left = 0;
uint16_t enc_right = 0;
int16_t enc_left_typeC = 0;	//type-casted encoder values
int16_t enc_right_typeC = 0;
int32_t position_left = 0; //changed to 32bit integer to prevent overflow, may need to typecast?
int16_t speed_left =0;	//speed var for export
int32_t position_right = 0;
int16_t speed_right = 0; //speed var for export into it.c where speed is calculated
extern float speed_diff_left_cm;


//var for angle error control loop
uint16_t goal_angle = 0;
uint16_t angle_from_enc_error = 0;
float target_speed_left = 0;
float target_speed_right = 0;

//P and D variables for PID loop for encoders
int enc_error_current = 0;
int enc_error_past = 0;
int encoder_derivative = 0;
int encoder_integral = 0;
float enc_Kp = 1;
float enc_Ki = 0;
float enc_Kd = 0;
float enc_control_signal = 0;

//var for battery voltage reading
uint16_t battery_volt_reading = 0;
float battery_volt_intitial = 0;
float battery_volt_analog = 0;


//var's for distance from IR sensors
int16_t dis_FR = 0;
int16_t dis_FL = 0;
int16_t dis_L = 0;
int16_t dis_R = 0;
_Bool a = false;

//function for PD controller for distance
	//void PD_angle_controller(){	//will also need an distance controller
		//encoder_derivative = enc_error_current - enc_error_past;


//}

//function to obtain encoder values and convert them ro encoder revolutions
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
	// this is the left encoder timer
	/*if (htim->Instance == TIM3) {
		enc_left = __HAL_TIM_GET_COUNTER(htim);
}*/
	if (htim->Instance == TIM3) {
		enc_left = __HAL_TIM_GET_COUNTER(htim);
		enc_left_typeC = (int16_t)enc_left*(-1);
		position_left =  enc_left_typeC/360;
	}
	if (htim->Instance == TIM4) {
		enc_right = __HAL_TIM_GET_COUNTER(htim);
		enc_right_typeC = (int16_t)enc_right*(-1);
		position_right = enc_right_typeC/360;

	}


	enc_error_current = enc_left_typeC - enc_right_typeC;	//consider moving into SYS tick handler to use interrupts to calculate errors
	encoder_derivative = enc_error_current - enc_error_past; //settles around 0?? sum of errors
	encoder_integral += enc_error_current;
	enc_control_signal = enc_Kd*encoder_derivative + enc_Ki*encoder_integral + enc_Kp*enc_error_current;

	enc_error_past = enc_error_current;




}

uint32_t calculatePWM(uint32_t)
{
	//given a goal speed, return the value to set CCR4/CCR3
	return 0;
}

//float desired_speed_vs_duty_cycle(){
		//target_speed_right_slope = (TIM2->CCCR3)/current_speed_left_cm;
		//target_speed_left_slope = (TIM2->CCR4)/current_speed_right_cm;


//}


//functions to move the mouse in specific directions
void move_bwd(uint32_t delay, uint32_t Lspeed, uint32_t Rspeed)
{
	  TIM2->CCR4 = 1300;
	  TIM2 ->CCR3 = 1300;

	  HAL_GPIO_WritePin(ML_FWD_GPIO_Port, ML_FWD_Pin, 1); // ml fwd high
	  HAL_GPIO_WritePin(ML_BWD_GPIO_Port, ML_BWD_Pin, 0);	//mlbwd low

	  HAL_GPIO_WritePin(MR_FWD_GPIO_Port, MR_FWD_Pin, 1); //mr fwd high
	  HAL_GPIO_WritePin(MR_BWD_GPIO_Port, MR_BWD_Pin, 0);
	  HAL_Delay(delay);

}

void move_fwd(uint32_t delay, uint32_t Lspeed, uint32_t Rspeed)	//need a parameter to adjust TIM2 CCR4/3 and pass to while loop
{
	TIM2->CCR4 = Lspeed;	//2047 is the max
	TIM2 ->CCR3 = Rspeed;
	//bwd
	HAL_GPIO_WritePin(ML_FWD_GPIO_Port, ML_FWD_Pin, 0); // ml fwd high
	HAL_GPIO_WritePin(ML_BWD_GPIO_Port, ML_BWD_Pin, 1);	//mlbwd low

	HAL_GPIO_WritePin(MR_FWD_GPIO_Port, MR_FWD_Pin, 0); //mr fwd high
	HAL_GPIO_WritePin(MR_BWD_GPIO_Port, MR_BWD_Pin, 1);

	HAL_Delay(delay);
}

void rotate_right(uint32_t delay, uint32_t Lspeed, uint32_t Rspeed)
{
	TIM2->CCR4 = 1300;
	TIM2 ->CCR3 = 1300;
	//bwd
	HAL_GPIO_WritePin(ML_FWD_GPIO_Port, ML_FWD_Pin, 0); // ml fwd high
	HAL_GPIO_WritePin(ML_BWD_GPIO_Port, ML_BWD_Pin, 1);	//mlbwd low

	HAL_GPIO_WritePin(MR_FWD_GPIO_Port, MR_FWD_Pin, 1); //mr fwd high
	HAL_GPIO_WritePin(MR_BWD_GPIO_Port, MR_BWD_Pin, 0);

	HAL_Delay(delay);
}

void rotate_left(uint32_t delay, uint32_t Lspeed, uint32_t Rspeed)
{

	TIM2->CCR4 = calculatePWM(Lspeed);
	TIM2 ->CCR3 = calculatePWM(Rspeed);
	//bwd
	HAL_GPIO_WritePin(ML_FWD_GPIO_Port, ML_FWD_Pin, 1); // ml fwd high
	HAL_GPIO_WritePin(ML_BWD_GPIO_Port, ML_BWD_Pin, 0);	//mlbwd low

	HAL_GPIO_WritePin(MR_FWD_GPIO_Port, MR_FWD_Pin, 0); //mr fwd high
	HAL_GPIO_WritePin(MR_BWD_GPIO_Port, MR_BWD_Pin, 1);

	HAL_Delay(delay);
}






//functions to select and initialize adc CHANNELS

//ADC for battery reading
static void ADC1_Select_CH1(void){
ADC_ChannelConfTypeDef sConfig = {0};
		sConfig.Channel = ADC_CHANNEL_1;
		sConfig.Rank = ADC_REGULAR_RANK_1;
		sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
		if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
	}


static void ADC1_Select_CH4(void){
ADC_ChannelConfTypeDef sConfig = {0};
		sConfig.Channel = ADC_CHANNEL_4;
		sConfig.Rank = ADC_REGULAR_RANK_1;
		sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
		if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
		{
			Error_Handler();
		}
	}


static void ADC1_Select_CH9(void){
		ADC_ChannelConfTypeDef sConfig = {0};
			sConfig.Channel = ADC_CHANNEL_9;
			sConfig.Rank = ADC_REGULAR_RANK_1;
			sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
			if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
			{
				Error_Handler();
			}
	}

static void ADC1_Select_CH5(void){
		ADC_ChannelConfTypeDef sConfig = {0};
			sConfig.Channel = ADC_CHANNEL_5;
			sConfig.Rank = ADC_REGULAR_RANK_1;
			sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
			if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
			{
				Error_Handler();
			}
	}

static void ADC1_Select_CH8(void){
		ADC_ChannelConfTypeDef sConfig = {0};
			sConfig.Channel = ADC_CHANNEL_8;
			sConfig.Rank = ADC_REGULAR_RANK_1;
			sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
			if(HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
			{
				Error_Handler();
			}
	}



uint16_t measure_dist(dist_t dist){
	GPIO_TypeDef* emitter_port;
	uint16_t emitter_pin;
	GPIO_TypeDef* receiver_port;
	uint16_t receiver_pin;
	switch(dist) {
	case DIST_FL:
		emitter_port	= EMIT_FL_GPIO_Port; //assign ports to variables
		emitter_pin		= EMIT_FL_Pin;	//assign pins to variables
		receiver_port	= RECIV_FL_GPIO_Port;
		receiver_pin 	= RECIV_FL_Pin;
		ADC1_Select_CH9();
		break;
	case DIST_FR:
			emitter_port	= EMIT_FR_GPIO_Port;
			emitter_pin		= EMIT_FR_Pin;
			receiver_port	= RECIV_FR_GPIO_Port;
			receiver_pin 	= RECIV_FR_Pin;
			ADC1_Select_CH4();
			break;
	case DIST_R:
				emitter_port	= EMIT_R_GPIO_Port;
				emitter_pin		= EMIT_R_Pin;
				receiver_port	= RECIV_R_GPIO_Port;
				receiver_pin 	= RECIV_R_Pin;
				ADC1_Select_CH5();
				break;
	case DIST_L:
					emitter_port	= EMIT_L_GPIO_Port;
					emitter_pin		= EMIT_L_Pin;
					receiver_port	= RECIV_L_GPIO_Port;
					receiver_pin 	= RECIV_L_Pin;
					ADC1_Select_CH8();
					break;
	default:
			break;
	}
	HAL_GPIO_WritePin(emitter_port, emitter_pin, GPIO_PIN_SET); //this function enables us to write to a pin
	HAL_Delay(5);
	HAL_ADC_Start(&hadc1); //activate the ADC
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	uint16_t adc_val = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
	return adc_val;
}

uint16_t measure_battery_voltage(){
		ADC1_Select_CH1();
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		battery_volt_reading = HAL_ADC_GetValue(&hadc1); //raw ADC value for battery voltage
		return battery_volt_reading;	//at 7.2V ADC value should be between 2400 and 4000
		//7.22 is current voltage, R23 =10KOHM R22= 20KOHM, voltage at ADC should be 1/3 of battery voltage
}


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
  MX_TIM2_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
  HAL_TIM_Encoder_Start_IT(&htim3, TIM_CHANNEL_ALL);
  HAL_TIM_Encoder_Start_IT(&htim4, TIM_CHANNEL_ALL);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  while (1)
  {
	  battery_volt_analog = battery_volt_reading *(9/4096); //conversion factor for ADC reading to an analog voltage
	  //calls to measure distance FR,FL,L,R can adjust delay in timing function to make the IR sensors more responsive
	  dis_FR = measure_dist(DIST_FR);
	  dis_FL = measure_dist(DIST_FL);
	  dis_R = measure_dist(DIST_R);
	  dis_L = measure_dist(DIST_L);

	  battery_volt_analog = measure_battery_voltage();



	  if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_4) == GPIO_PIN_RESET){ //code for reading a button input/output


		  	  //move_fwd(5000,1000,1000);40
		  	  //move_fwd(5000,1100,1100);50
		  	  //move_fwd(5000,1200,1200);60
		  	  //move_fwd(5000,1300,1300);70
		  	  //move_fwd(5000,1400,1400);80
		  	  //move_fwd(5000,1500,1500);90
		  	  //move_fwd(5000,1600,1600);100
		  	  //move_fwd(5000,1700,1700);120
		  	  //move_fwd(5000,1800,1800);140
		  	  //move_fwd(5000,1900,1900);160

		  	  //if (speed_diff_left_cm == 0) move_fwd(1000, 1800, 1800);
			  //if (speed_diff_left_cm > 0){
				//  move_fwd(500, 0000, 1800);
			  	  //move_fwd(1000, 1800, 1800);
			  //}
			  //else if (speed_diff_left_cm < 0) move_fwd(500, 2200, 1800);


	  }

	  if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_14) == GPIO_PIN_RESET){

	 			  //move_bwd(1000);

	 		  }


		  //rotate_right(1000);
		  //rotate_left(1000);

		 // move_bwd(1000);
		  //HAL_Delay(500);
		  //a = true;

		  //HAL_Delay(500);
		  //a = false;




	  //avoid_front_wall();






	  //rotate_right(2000);

	  //rotate_left(4000);

	  //rotate_right(3000);

	  //move_fwd(2000);

	  //move_bwd(2000);

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
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
  sConfig.Channel = ADC_CHANNEL_4;
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
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2047;
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
  sConfigOC.Pulse = 25;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 1024;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
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
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

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
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, EMIT_R_Pin|EMIT_L_Pin|EMIT_FL_Pin|MR_FWD_Pin
                          |ML_BWD_Pin|MR_BWD_Pin|EMIT_FR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(ML_FWD_GPIO_Port, ML_FWD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_2_Pin */
  GPIO_InitStruct.Pin = Button_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EMIT_R_Pin EMIT_L_Pin EMIT_FL_Pin MR_FWD_Pin
                           ML_BWD_Pin MR_BWD_Pin EMIT_FR_Pin */
  GPIO_InitStruct.Pin = EMIT_R_Pin|EMIT_L_Pin|EMIT_FL_Pin|MR_FWD_Pin
                          |ML_BWD_Pin|MR_BWD_Pin|EMIT_FR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : ML_FWD_Pin */
  GPIO_InitStruct.Pin = ML_FWD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(ML_FWD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PA9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_1_Pin */
  GPIO_InitStruct.Pin = Button_1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_1_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
