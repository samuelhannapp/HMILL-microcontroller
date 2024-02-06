/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
//#include "structs.h"
#include "C:/Users/Samuel/source/repos/motion_planner_oop/oop_test/microcontroller_data_set.h"
#include "macros.h"
#include "structs.h"
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
CAN_HandleTypeDef hcan1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim10;
TIM_HandleTypeDef htim11;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim13;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN1_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM10_Init(void);
static void MX_TIM11_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM13_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
static void start_first_move();
static void setup();
static void go_to_home_position();
static void start_program();
static void move_axis(int32_t increment,int32_t direction,char axis);
static void check_command();
void send_position_message(uint32_t number_1,uint32_t number_2,uint32_t identifier);
void send_data();//thats all kinds of data for displaying standpoint, codeblock, all kinds of things
//thats done via uart because its not time and savety critical
static int get_mailbox();
static int is_mailbox_empty(int mailbox_nr);
static void measure_tool();
static void homing_cycle();

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

extern void request_receiving_data();



CAN_TxHeaderTypeDef tx_header;
//CAN_TxHeaderTypeDef rx_header;

volatile struct microcontroller_data_set fifo_buffer[FIFO_BUFFER_SIZE];
volatile uint32_t fifo_write_ctr=0;
volatile uint32_t fifo_read_ctr=FIFO_BUFFER_SIZE;

volatile struct axis_data x_data;
volatile struct axis_data y_data;
volatile struct axis_data z_data;
volatile struct axis_data b_data;
volatile struct axis_data c_data;
volatile uint32_t n_line_data;


volatile uint32_t flags_global_mc=0;

//those are only for the manual controll
volatile uint32_t commands=0;
volatile uint16_t timer_speed=SPEED_1;
volatile uint32_t increment=20;

volatile uint32_t motor_start_status=0;
volatile uint32_t motor_x_direction=0;
volatile uint32_t motor_y_direction=0;
volatile uint32_t motor_z_direction=0;
volatile uint32_t motor_b_direction=0;
volatile uint32_t motor_c_direction=0;

//uint8_t tx_data[8];

//uint32_t tx_mailbox;

//uint32_t ctr=0;

volatile int32_t x_target=0;
volatile int32_t y_target=0;
volatile int32_t z_target=0;
volatile int32_t b_target=0;
volatile int32_t c_target=0;

volatile int32_t x_standpoint=0;
volatile int32_t y_standpoint=0;
volatile int32_t z_standpoint=0;
volatile int32_t b_standpoint=0;
volatile int32_t c_standpoint=0;

volatile uint32_t gcode_line_number=0;

volatile int32_t x_standpoint_previous=0;
volatile int32_t y_standpoint_previous=0;
volatile int32_t z_standpoint_previous=0;
volatile int32_t b_standpoint_previous=0;
volatile int32_t c_standpoint_previous=0;






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
  MX_CAN1_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_TIM12_Init();
  MX_TIM13_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);

  // I changed this on 28.09.2023
  //commands&=~START_PROGRAM;
  flags_global_mc|=MACHINE_HOMED;
  //flags|=FIRST_MOVE_NO_Z;




  setup();



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	check_command();
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 3;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_10TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_3TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */
  CAN_FilterTypeDef canfilterconfig;
  canfilterconfig.FilterActivation=CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank=10;
  canfilterconfig.FilterFIFOAssignment=CAN_RX_FIFO0;
  canfilterconfig.FilterIdHigh=0;
  canfilterconfig.FilterIdLow=0;
  canfilterconfig.FilterMaskIdHigh=0;
  canfilterconfig.FilterMaskIdLow=0;
  canfilterconfig.FilterMode=CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale=CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank=0;

  HAL_CAN_ConfigFilter(&hcan1,&canfilterconfig);
  /* USER CODE END CAN1_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
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
  htim3.Init.Prescaler = 0;
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
  if (HAL_TIM_OC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  TIM3->ARR=60000;
  TIM3->PSC=10;

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 0;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 65535;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 0;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 65535;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim10, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */
  TIM10->ARR=60000;
  TIM10->PSC=5;
  /* USER CODE END TIM10_Init 2 */
  HAL_TIM_MspPostInit(&htim10);

}

/**
  * @brief TIM11 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM11_Init(void)
{

  /* USER CODE BEGIN TIM11_Init 0 */

  /* USER CODE END TIM11_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM11_Init 1 */

  /* USER CODE END TIM11_Init 1 */
  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 0;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 65535;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim11) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TOGGLE;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim11, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM11_Init 2 */
  TIM11->ARR=60000;
  TIM11->PSC=3;
  /* USER CODE END TIM11_Init 2 */
  HAL_TIM_MspPostInit(&htim11);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 0;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 65535;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 65000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */

}

/**
  * @brief TIM13 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM13_Init(void)
{

  /* USER CODE BEGIN TIM13_Init 0 */

  /* USER CODE END TIM13_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM13_Init 1 */

  /* USER CODE END TIM13_Init 1 */
  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 0;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim13.Init.Period = 65535;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OC_Init(&htim13) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_OC_ConfigChannel(&htim13, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM13_Init 2 */

  /* USER CODE END TIM13_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_0
                          |GPIO_PIN_1, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PE2 PE3 PE4 PE0
                           PE1 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_0
                          |GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PowerSwitchOn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PB1 BOOT1_Pin PB3 PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|BOOT1_Pin|GPIO_PIN_3|GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PD2 OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_2|OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

static void start_first_move()
{
	//thats to reinitialize the timers
	//because before the GPIO of the Compare pins
	//where otherwise used
	MX_TIM9_Init();
	MX_TIM10_Init();
	MX_TIM11_Init();
	MX_TIM12_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
	commands&=~START_PROGRAM;
	flags_global_mc&=~FIRST_MOVE_IN_PROCESS;
	flags_global_mc|=PROGRAM_RUNNING;
	flags_global_mc&=~BUFFER_FULL;
	//thats the timer to start everything
	init_interrupt_enable();
	init_compare_enable();
	init_timer_start();
	//start position timer
	//I think this is not neccesary anymore,
	//this was to send position data to computer,
	//but this is obsolete I think
	/*
	position_timer_compare_enable();
	position_timer_start();
	*/
	return;
}

static void setup()
{
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
	//TIM2->CR1=1;

	flags_global_mc|=WRITE_CTR_UNDER_READ_CTR;
	flags_global_mc|=FIRST_MOVE;

	x_interrupt_enable();
	y_interrupt_enable();
	z_interrupt_enable();
	b_interrupt_enable();
	c_interrupt_enable();
	manual_interrupt_enable();
	//position_timer_interrupt_enable();



	//I have to figure out why that is so important
	tx_header.DLC=8;
	tx_header.IDE=CAN_ID_STD;
	tx_header.RTR=CAN_RTR_DATA;
	CAN1->sTxMailBox->TDTR=8;
}

static void check_command()
{
	if(!commands)
		return;

	if(commands&RESET_MICROCONTROLLER)
		NVIC_SystemReset();

	//POSITION STANDPOINT REQUESTS
	if(commands&X_POSITION_REQUEST_FLAG){
		uint32_t input_nr=(uint32_t)x_standpoint;
		send_position_message(input_nr,0,CAN_ID_GET_X_POSITION_ANSWER); //from where the standpoint is should be considered!!
		commands&=~X_POSITION_REQUEST_FLAG;
		return;
	}
	if(commands&Y_POSITION_REQUEST_FLAG){
		uint32_t input_nr=(uint32_t)y_standpoint;
		send_position_message(input_nr,0,CAN_ID_GET_Y_POSITION_ANSWER);
		commands&=~Y_POSITION_REQUEST_FLAG;

		return;
	}
	if(commands&Z_POSITION_REQUEST_FLAG){
		uint32_t input_nr=(uint32_t)z_standpoint;
		send_position_message(input_nr,0,CAN_ID_GET_Z_POSITION_ANSWER);
		commands&=~Z_POSITION_REQUEST_FLAG;

		return;
	}
	if(commands&B_POSITION_REQUEST_FLAG){
		uint32_t input_nr=(uint32_t)b_standpoint;
		send_position_message(input_nr,0,CAN_ID_B_POSITION_ANSWER);
		commands&=~B_POSITION_REQUEST_FLAG;

		return;
	}
	if(commands&C_POSITION_REQUEST_FLAG){
		uint32_t input_nr=(uint32_t)c_standpoint;
		send_position_message(input_nr,0,CAN_ID_C_POSITION_ANSWER);
		commands&=~C_POSITION_REQUEST_FLAG;

		return;
	}


	if(!NO_ACTIVE_MOVE)	//that can also be switched on when a programm is runnung
		return;
	else if(commands&HOMING_CYCLE_FLAG){
		homing_cycle();			//thats hardware goes to hardwareswitches
		return;
	}

	//that has to be done from the home_position after the machine was homed
	if(commands&MEASURE_TOOL_FLAG){
		measure_tool();
		return;
	}

	//POSITION MOVE_REQUESTS
	else if(commands&MOVE_X_POSITIVE){
		move_axis(increment,POSITIVE,'x');
		return;
	}
	else if(commands&MOVE_X_NEGATIVE){
		move_axis(increment,NEGATIVE,'x');
		return;
	}
	else if(commands&MOVE_Y_POSITIVE){
		move_axis(increment,POSITIVE,'y');
		return;
	}
	else if(commands&MOVE_Y_NEGATIVE){
		move_axis(increment,NEGATIVE,'y');
		return;
	}
	else if(commands&MOVE_Z_POSITIVE){
		move_axis(increment,POSITIVE,'z');
		return;
	}
	else if(commands&MOVE_Z_NEGATIVE){
		move_axis(increment,NEGATIVE,'z');
		return;
	}
	else if(commands&MOVE_B_POSITIVE){
		move_axis(increment,POSITIVE,'b');
		return;
	}
	else if(commands&MOVE_B_NEGATIVE){
		move_axis(increment,NEGATIVE,'b');
		return;
	}
	else if(commands&MOVE_C_POSITIVE){
		move_axis(increment,POSITIVE,'c');
		return;
	}
	else if(commands&MOVE_C_NEGATIVE){
		move_axis(increment,NEGATIVE,'c');
		return;
	}

	//HOME REQUESTS
	if(commands&GO_TO_HOME){
		go_to_home_position(); //thats software, just goes back to targetxyz 0
		return;
	}


	//PROGRAM START REQUEST

	else if(commands&START_PROGRAM){
		if(!(flags_global_mc&BUFFER_FILLING_IN_PROGRESS))
			start_program();
		if(flags_global_mc&BUFFER_FULL)
			start_first_move();
		return;
	}
			
	return;
}


static void move_axis(int32_t increment,int32_t direction,char axis)
{
	if(!NO_ACTIVE_MOVE)
		return;
	//delete all requests
	commands&=~MOVE_X_POSITIVE&~MOVE_X_NEGATIVE&~MOVE_Y_POSITIVE&~MOVE_Y_NEGATIVE&~MOVE_Z_POSITIVE&~MOVE_Z_NEGATIVE&~MOVE_B_POSITIVE&~MOVE_B_NEGATIVE&~MOVE_C_POSITIVE&~MOVE_C_NEGATIVE;

	if(axis=='c'||axis=='b')
		increment= INCREMENT_3;

	if(!direction)
		increment*=-1;


	switch(axis){
		case 'x':	flags_global_mc|=X_MANUAL_MOVE;
					if(increment)
						x_target+=increment;
					else
						x_target=0;
					motor_x_direction=direction;
					if(direction){
						SET_X_AXIS_POSITIVE_DIRECTION();
					}
					else
						SET_X_AXIS_NEGATIVE_DIRECTION();
					break;
		
		case 'y':	flags_global_mc|=Y_MANUAL_MOVE;
					if(increment)
						y_target+=increment;
					else
						y_target=0;
					motor_y_direction=direction;
					if(direction){
						SET_Y_AXIS_POSITIVE_DIRECTION();
					}
					else
						SET_Y_AXIS_NEGATIVE_DIRECTION();
					break;
		
		case 'z':	flags_global_mc|=Z_MANUAL_MOVE;
					if(increment)
						z_target+=increment;
					else
						z_target=0;
					motor_z_direction=direction;
					if(direction){
						SET_Z_AXIS_POSITIVE_DIRECTION();
					}
					else
						SET_Z_AXIS_NEGATIVE_DIRECTION();
					break;
		case 'b':
					flags_global_mc|=B_MANUAL_MOVE;
					if(increment)
						b_target+=increment;
					else
						b_target=0;
					motor_b_direction=direction;
					if(direction){
						SET_B_AXIS_POSITIVE_DIRECTION();
					}
					else
						SET_B_AXIS_NEGATIVE_DIRECTION();
					break;
		case 'c':

					flags_global_mc|=C_MANUAL_MOVE;
					if(increment)
						c_target+=increment;
					else
						c_target=0;
					motor_c_direction=direction;
					if(direction){
						SET_C_AXIS_POSITIVE_DIRECTION();
					}
					else
						SET_C_AXIS_NEGATIVE_DIRECTION();
					break;

		default:	return;
	}

	//setup for xaxis
	GPIO_InitTypeDef gpio_init_1;
	gpio_init_1.Pin = GPIO_PIN_5;
	gpio_init_1.Mode = GPIO_MODE_OUTPUT_PP;
	gpio_init_1.Pull = GPIO_NOPULL;
	gpio_init_1.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOE, &gpio_init_1);

	//setup for yaxis
	GPIO_InitTypeDef gpio_init_2;
	gpio_init_2.Pin = GPIO_PIN_8;
	gpio_init_2.Mode = GPIO_MODE_OUTPUT_PP;
	gpio_init_2.Pull = GPIO_NOPULL;
	gpio_init_2.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &gpio_init_2);

	//setup for zaxis
	GPIO_InitTypeDef gpio_init_3;
	gpio_init_3.Pin = GPIO_PIN_9;
	gpio_init_3.Mode = GPIO_MODE_OUTPUT_PP;
	gpio_init_3.Pull = GPIO_NOPULL;
	gpio_init_3.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &gpio_init_3);

	//setup for baxis
	GPIO_InitTypeDef gpio_init_4;
	gpio_init_4.Pin = GPIO_PIN_6;
	gpio_init_4.Mode = GPIO_MODE_OUTPUT_PP;
	gpio_init_4.Pull = GPIO_NOPULL;
	gpio_init_4.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &gpio_init_4);

	//setup for caxis
	GPIO_InitTypeDef gpio_init_5;
	gpio_init_5.Pin = GPIO_PIN_6;
	gpio_init_5.Mode = GPIO_MODE_OUTPUT_PP;
	gpio_init_5.Pull = GPIO_NOPULL;
	gpio_init_5.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &gpio_init_5);

	if(axis=='b'||axis=='c'){
		TIM13->PSC=400;

	}
	else
		TIM13->PSC=1;
	TIM13->ARR=SPEED_1;
	//x_compare_enable();
	manual_compare_enable();
	manual_timer_start();
	
	return;
}


//is machine homed, is machine at 0, is there no manual move going on

static void start_program()
{
	if(MACHINE_READY_FOR_PROGRAM){
		flags_global_mc|=BUFFER_FILLING_IN_PROGRESS;
		request_receiving_data();
	}
		
	return;
}

//this should be not neccesary anymore because to go every time to the end switches would be much better

void go_to_home_position()
{
	//first home the z axis, else you could drive into the workpiece

	if(z_standpoint>0)
		move_axis(MOVE_TO_ZERO,NEGATIVE,'z');
	else if(z_standpoint<0)
		move_axis(MOVE_TO_ZERO,POSITIVE,'z');

	else if(x_standpoint>0)
		move_axis(MOVE_TO_ZERO,NEGATIVE,'x');
	else if(x_standpoint<0)
		move_axis(MOVE_TO_ZERO,POSITIVE,'x');
	
	else if(y_standpoint>0)
		move_axis(MOVE_TO_ZERO,NEGATIVE,'y');
	else if(y_standpoint<0)
		move_axis(MOVE_TO_ZERO,POSITIVE,'y');

	else commands&=~GO_TO_HOME;

	return;
}

static void homing_cycle(){
//first home the z axis, else you could drive into the workpiece
	if(!(flags_global_mc&Z_HOMED))
		move_axis(1000000,POSITIVE,'z');
	else if(!(flags_global_mc&X_HOMED))
		move_axis(1000000,NEGATIVE,'x');
	else if(!(flags_global_mc&Y_HOMED))
		move_axis(1000000,POSITIVE,'y');
	else{
		commands&=~HOMING_CYCLE_FLAG;
		flags_global_mc&=~Z_HOMED;
		flags_global_mc&=~X_HOMED;
		flags_global_mc&=~Y_HOMED;
		flags_global_mc|=MACHINE_HOMED;
		//that would be not good because the home position would mean we are at the endswitches,
		//witch means we are not allowed to move, that would mean if we would start anything from there,
		//we would  need again an exeption...
		/*
		x_standpoint=0;
		y_standpoint=0;
		z_standpoint=0;
		*/
		//There has to be a figured out a good place to move after the homing cycle so that the axis are not
		//at the endswitches anymore
		x_standpoint=-6400;//it woudl move a bit to the right in case of go to home
		y_standpoint=6400;
		z_standpoint=6400;
		//after that go to home
		commands|=GO_TO_HOME;//move all axis to 0
	}

	return;

}
//homeposition means all axis are at 0, that is slightly besides the endswitches, in order that they are not pressed
#define MACHINE_AT_HOME_POSITION (x_standpoint==0&&y_standpoint==0&&z_standpoint==0)

static void measure_tool()
{
	//if Machine is homed, and if its at xyz 0;
	//else first home or at least go xyz to 0 then measure the tool
	//
	if((flags_global_mc&MACHINE_HOMED)&&MACHINE_AT_HOME_POSITION)
		move_axis(1000000,NEGATIVE,'z');
	else if(flags_global_mc&MEASURED_TOOL){
		uint32_t input_nr=(uint32_t)z_standpoint;//if this number is -1(should not happen, than the value of the input_nr will be 0xffffffff) thats a problem, its oviously not because in the gui there is still the right value...
		send_position_message(input_nr,0,MEASURE_TOOL_ANSWER_ID);
		flags_global_mc&=~MEASURED_TOOL;
		commands&=~MEASURE_TOOL_FLAG;
		timer_speed=SPEED_1;
		commands|=GO_TO_HOME;//move all axis to 0
	}

	return;

}

//the number is allways an int32_t, the identifier is 11 bit, means max2048
void send_position_message(uint32_t number_1,uint32_t number_2,uint32_t identifier)
{
        int mailbox_nr=0;
        mailbox_nr=get_mailbox();       //returns number of empty mailboxor at least lowest priority pending mailbox

        if(is_mailbox_empty(mailbox_nr)){       //in case of an request for new data this whould be forced
                CAN1->sTxMailBox[mailbox_nr].TDTR=(8<<CAN_TDT0R_DLC_Pos); //because its an int32_t... 4 bytes
                CAN1->sTxMailBox[mailbox_nr].TDLR=number_1;
                CAN1->sTxMailBox[mailbox_nr].TDHR=number_2;
                CAN1->sTxMailBox[mailbox_nr].TIR=0;
                CAN1->sTxMailBox[mailbox_nr].TIR|=identifier;
                CAN1->sTxMailBox[mailbox_nr].TIR|=CAN_TI0R_TXRQ;        //that means start to transmit the message
        }
        return;
}

static int get_mailbox()
{
        int ret=0;
        ret=CAN1->TSR&CAN_TSR_CODE;
        return ret;
}

static int is_mailbox_empty(int mailbox_nr)
{
        int ret=0;
        ret=CAN1->TSR&(1<<(CAN_TSR_TME_Pos+mailbox_nr));
        return ret;
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
