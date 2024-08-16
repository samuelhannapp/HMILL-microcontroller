/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "structs.h"
#include "C:/Users/Samuel/source/repos/motion_planner_oop/oop_test/microcontroller_data_set.h"
#include "macros.h"
#include "math.h"
#include "structs.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
//they have to be static, else it will not compile!!!
static inline void get_data();
//void stop_receiving_data();
void request_receiving_data();
extern void send_position_message(uint32_t number_1,uint32_t number_2,uint32_t identifier);

static inline void update_standpoint_x();
static inline void check_target_reached_x();
static inline void update_standpoint_y();
static inline void check_target_reached_y();
static inline void update_standpoint_z();
static inline void check_target_reached_z();
static inline void update_standpoint_b();
static inline void check_target_reached_b();
static inline void update_standpoint_c();
static inline void check_target_reached_c();

static inline void prepare_next_move();
static inline void save_targets();
static int  	   set_timer_speeds();
static inline void set_targets();
//static inline void set_directions();
static inline void start_motors();

static inline void manual_motor_controll();

static void calc_prescaler(float frequency,int timer);

static inline void x_axis_start(void);

static inline void y_axis_start(void);

static inline void z_axis_start(void);

static inline void xy_axis_start(void);

static inline void xz_axis_start(void);

static inline void yz_axis_start(void);

static inline void xyz_axis_start(void);

static inline void b_axis_start(void);

static inline void c_axis_start(void);

static inline void toggle_pin_x_axis(void);
static inline void toggle_pin_y_axis(void);
static inline void toggle_pin_z_axis(void);
static inline void toggle_pin_b_axis(void);
static inline void toggle_pin_c_axis(void);

static inline void handle_mc_data_part_1();
static inline void handle_mc_data_part_2();
static inline void handle_mc_data_part_3();
static inline void handle_mc_data_part_4();

static inline void wait();

int this_is_just_that_we_get_an_commit = 0;





/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern CAN_HandleTypeDef hcan1;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim9;
extern TIM_HandleTypeDef htim10;
extern TIM_HandleTypeDef htim11;
extern TIM_HandleTypeDef htim12;
extern TIM_HandleTypeDef htim13;
/* USER CODE BEGIN EV */
extern volatile struct microcontroller_data_set fifo_buffer[FIFO_BUFFER_SIZE];
extern volatile uint32_t fifo_write_ctr;
extern volatile uint32_t fifo_read_ctr;


extern volatile uint32_t flags_global_mc;
extern volatile uint32_t motor_start_status;
extern volatile uint32_t motor_x_direction;
extern volatile uint32_t motor_y_direction;
extern volatile uint32_t motor_z_direction;
extern volatile uint32_t motor_b_direction;
extern volatile uint32_t motor_c_direction;

extern volatile int32_t x_target;
extern volatile int32_t y_target;
extern volatile int32_t z_target;
extern volatile int32_t b_target;
extern volatile int32_t c_target;

extern volatile int32_t x_standpoint;
extern volatile int32_t y_standpoint;
extern volatile int32_t z_standpoint;
extern volatile int32_t b_standpoint;
extern volatile int32_t c_standpoint;
extern volatile uint32_t n_line_data;

extern volatile struct axis_data x_data;
extern volatile struct axis_data y_data;
extern volatile struct axis_data z_data;
extern volatile struct axis_data b_data;
extern volatile struct axis_data c_data;


extern volatile uint32_t gcode_line_number;

extern volatile int32_t x_standpoint_previous;
extern volatile int32_t y_standpoint_previous;
extern volatile int32_t z_standpoint_previous;
extern volatile int32_t b_standpoint_previous;
extern volatile int32_t c_standpoint_previous;
static volatile int32_t motor_start;

extern volatile uint32_t commands;
extern volatile uint16_t timer_speed;
extern volatile uint32_t increment;

static volatile uint32_t debug_ctr=0;

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
  while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles CAN1 RX0 interrupts.
  */
void CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN CAN1_RX0_IRQn 0 */

  /* USER CODE END CAN1_RX0_IRQn 0 */
  HAL_CAN_IRQHandler(&hcan1);
  /* USER CODE BEGIN CAN1_RX0_IRQn 1 */
	/*if(!(CAN1->RF0R&CAN_RF0R_FOVR0))*/			//thats maybe a wast of time... (look for fifo overrun)	//thats good, but thats only for fifo mb0, there is also one more
  get_data();
	//that should be either in the handler or only here, but not in both...
  CAN1->RF0R|=CAN_RF0R_RFOM0;
  	  //request_receiving_data();

	/*ignore this for now
	else{
		return;
		flags|=CAN1_FIFO_OVERRUN; //kriese!!! alarm...
		//for nor clear the interrupt flag
		CAN1->RF0R&=~CAN_RF0R_FOVR0;
		//the stop thing supposed to be here
	}*/
	return;

  /* USER CODE END CAN1_RX0_IRQn 1 */
}

/**
  * @brief This function handles TIM1 break interrupt and TIM9 global interrupt.
  */
void TIM1_BRK_TIM9_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 0 */

  /* USER CODE END TIM1_BRK_TIM9_IRQn 0 */
  HAL_TIM_IRQHandler(&htim9);
  /* USER CODE BEGIN TIM1_BRK_TIM9_IRQn 1 */
  update_standpoint_x();
  check_target_reached_x();
  /* USER CODE END TIM1_BRK_TIM9_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt and TIM10 global interrupt.
  */
void TIM1_UP_TIM10_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 0 */

  /* USER CODE END TIM1_UP_TIM10_IRQn 0 */
  HAL_TIM_IRQHandler(&htim10);
  /* USER CODE BEGIN TIM1_UP_TIM10_IRQn 1 */
  update_standpoint_y();
  check_target_reached_y();
  /* USER CODE END TIM1_UP_TIM10_IRQn 1 */
}

/**
  * @brief This function handles TIM1 trigger and commutation interrupts and TIM11 global interrupt.
  */
void TIM1_TRG_COM_TIM11_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_TRG_COM_TIM11_IRQn 0 */

  /* USER CODE END TIM1_TRG_COM_TIM11_IRQn 0 */
  HAL_TIM_IRQHandler(&htim11);
  /* USER CODE BEGIN TIM1_TRG_COM_TIM11_IRQn 1 */
  update_standpoint_z();
  check_target_reached_z();
  /* USER CODE END TIM1_TRG_COM_TIM11_IRQn 1 */
}

/**
  * @brief This function handles TIM3 global interrupt.
  */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */
	update_standpoint_b();
  	check_target_reached_b();
  /* USER CODE END TIM3_IRQn 1 */
}

/**
  * @brief This function handles TIM4 global interrupt.
  */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */

  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */
  update_standpoint_c();
  check_target_reached_c();

  /* USER CODE END TIM4_IRQn 1 */
}

/**
  * @brief This function handles TIM8 break interrupt and TIM12 global interrupt.
  */
void TIM8_BRK_TIM12_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_BRK_TIM12_IRQn 0 */
	init_timer_stop();
	init_compare_disable();
	init_interrupt_disable();
	init_clear_interrupt();
  /* USER CODE END TIM8_BRK_TIM12_IRQn 0 */
  HAL_TIM_IRQHandler(&htim12);
  /* USER CODE BEGIN TIM8_BRK_TIM12_IRQn 1 */

  check_target_reached_z();
  //thats that the first move function doesn't start again
  flags_global_mc&=~BUFFER_FULL;
  /* USER CODE END TIM8_BRK_TIM12_IRQn 1 */
}

/**
  * @brief This function handles TIM8 update interrupt and TIM13 global interrupt.
  */
void TIM8_UP_TIM13_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 0 */

  /* USER CODE END TIM8_UP_TIM13_IRQn 0 */
  HAL_TIM_IRQHandler(&htim13);
  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 1 */
  manual_motor_controll();
  /* USER CODE END TIM8_UP_TIM13_IRQn 1 */
}

/* USER CODE BEGIN 1 */
//if there are not so much options no switch should be used because of performance
//the fifo mailbox[1] has to be included in this routing!!
//and maybe every transmit should be done with the function in main.c
static inline void get_data()
{
	if((fifo_write_ctr<fifo_read_ctr&&(flags_global_mc&WRITE_CTR_UNDER_READ_CTR)&&(!(flags_global_mc&FIRST_MOVE_IN_PROCESS)))||(fifo_write_ctr>fifo_read_ctr)){

		switch(CAN1->sFIFOMailBox->RIR){	//look up identifier of mailbox
			case CAN_ID_STOP_PROGRAM:
				CAN1->RF0R|=CAN_RF0R_RFOM0;
				commands |= RESET_MICROCONTROLLER;
				return;
			case MC_DATA_PART_1_ID:	handle_mc_data_part_1();
									return;

			case MC_DATA_PART_2_ID:	handle_mc_data_part_2();
									return;

			case MC_DATA_PART_3_ID:
									handle_mc_data_part_3();
									return;

			case MC_DATA_PART_4_ID:
									handle_mc_data_part_4();
									return;

			case CAN_ID_MOVE_X_POS: commands|=MOVE_X_POSITIVE;
						CAN1->RF0R|=CAN_RF0R_RFOM0;
						return;
			case CAN_ID_MOVE_X_NEG: commands|=MOVE_X_NEGATIVE;
						CAN1->RF0R|=CAN_RF0R_RFOM0;
						return;
			case CAN_ID_MOVE_Y_POS: commands|=MOVE_Y_POSITIVE;
						CAN1->RF0R|=CAN_RF0R_RFOM0;
						return;
			case CAN_ID_MOVE_Y_NEG: commands|=MOVE_Y_NEGATIVE;
						CAN1->RF0R|=CAN_RF0R_RFOM0;
						return;
			case CAN_ID_MOVE_Z_POS: commands|=MOVE_Z_POSITIVE;
						CAN1->RF0R|=CAN_RF0R_RFOM0;
						return;
			case CAN_ID_MOVE_Z_NEG: commands|=MOVE_Z_NEGATIVE;
						CAN1->RF0R|=CAN_RF0R_RFOM0;
						return;
			case CAN_ID_MOVE_B_POS: commands|=MOVE_B_POSITIVE;
						CAN1->RF0R|=CAN_RF0R_RFOM0;
						return;
			case CAN_ID_MOVE_B_NEG: commands|=MOVE_B_NEGATIVE;
						CAN1->RF0R|=CAN_RF0R_RFOM0;
						return;
			case CAN_ID_MOVE_C_POS: commands|=MOVE_C_POSITIVE;
						CAN1->RF0R|=CAN_RF0R_RFOM0;
						return;
			case CAN_ID_MOVE_C_NEG: commands|=MOVE_C_NEGATIVE;
						CAN1->RF0R|=CAN_RF0R_RFOM0;
						return;
			case CAN_ID_MOVE_SPEED_1: 	timer_speed=SPEED_1;
										increment=INCREMENT_1;
							CAN1->RF0R|=CAN_RF0R_RFOM0;
							return;
			case CAN_ID_MOVE_SPEED_2: 	timer_speed=SPEED_2;
										increment=INCREMENT_2;
							CAN1->RF0R|=CAN_RF0R_RFOM0;
							return;
			case CAN_ID_MOVE_SPEED_3: 	timer_speed=SPEED_3;
										increment=INCREMENT_3;
							CAN1->RF0R|=CAN_RF0R_RFOM0;
							return;
			case CAN_ID_START_PROGRAM:
							if(!(flags_global_mc&PROGRAM_RUNNING))
								commands|=START_PROGRAM;
							CAN1->RF0R|=CAN_RF0R_RFOM0;
							return;

			//those increment things are not neccesary because they are connected to the speed
			/*case CAN_ID_MOVE_INCREMENT_1: 	increment=INCREMENT_1;
							CAN1->RF0R|=CAN_RF0R_RFOM0;
							return;
			case CAN_ID_MOVE_INCREMENT_2: 	increment=INCREMENT_2;
							CAN1->RF0R|=CAN_RF0R_RFOM0;
							return;
			case CAN_ID_MOVE_INCREMENT_3: 	increment=INCREMENT_3;
							CAN1->RF0R|=CAN_RF0R_RFOM0;
							return;
							*/
			case CAN_ID_GET_X_POSITION_REQUEST: 	commands|=X_POSITION_REQUEST_FLAG;
								CAN1->RF0R|=CAN_RF0R_RFOM0;
								return;
			case CAN_ID_GET_Y_POSITION_REQUEST: 	commands|=Y_POSITION_REQUEST_FLAG;
								CAN1->RF0R|=CAN_RF0R_RFOM0;
								return;
			case CAN_ID_GET_Z_POSITION_REQUEST: 	commands|=Z_POSITION_REQUEST_FLAG;
								CAN1->RF0R|=CAN_RF0R_RFOM0;
								return;
			/*
			case CAN_ID_B_POSITION_REQUEST: 	commands|=B_POSITION_REQUEST_FLAG;
								CAN1->RF0R|=CAN_RF0R_RFOM0;
								return;
			case CAN_ID_C_POSITION_REQUEST: 	commands|=C_POSITION_REQUEST_FLAG;
								CAN1->RF0R|=CAN_RF0R_RFOM0;
								return;
			*/
			case CAN_ID_GO_TO_MACHINE_ZERO:	commands|=GO_TO_HOME;
								CAN1->RF0R|=CAN_RF0R_RFOM0;
								return;
			case CAN_ID_HOMING_CYCLE:		commands|=HOMING_CYCLE_FLAG;
								CAN1->RF0R|=CAN_RF0R_RFOM0;
								return;
			case CAN_ID_MEASURE_WCS_TOOL: commands|=MEASURE_WCS_TOOL_FLAG;
								CAN1->RF0R|=CAN_RF0R_RFOM0;
								return;
			case CAN_ID_MEASURE_ACTUAL_TOOL: commands|=MEASURE_ACTUAL_TOOL_FLAG;
								CAN1->RF0R|=CAN_RF0R_RFOM0;
								return;
			default:				return;
		}
	}
	else{
		//thats impossible allmost
		flags_global_mc|=BUFFER_NOT_EMPTY;
		//flags|=BUFFER_FULL;
		//put_data_into_emergency_buffer();
	}
	return;
}

static inline void handle_mc_data_part_1()
{
	fifo_buffer[fifo_write_ctr].x_coordinate=CAN1->sFIFOMailBox->RDLR;	//read low word of can fifo buffer
	fifo_buffer[fifo_write_ctr].y_coordinate=CAN1->sFIFOMailBox->RDHR;	//read high word of can fifo buffer
	CAN1->RF0R|=CAN_RF0R_RFOM0; 	//acknowlege that message box was empitied;
	return;
}

static inline void handle_mc_data_part_2()
{
	fifo_buffer[fifo_write_ctr].z_coordinate=CAN1->sFIFOMailBox->RDLR;	//read low word of can fifo buffer
	fifo_buffer[fifo_write_ctr].B_axis=CAN1->sFIFOMailBox->RDHR;	//read high word of can fifo buffer
	CAN1->RF0R|=CAN_RF0R_RFOM0; 	//acknowlege that message box was empitied;
	return;
}

static inline void handle_mc_data_part_3()
{
	fifo_buffer[fifo_write_ctr].C_axis=CAN1->sFIFOMailBox->RDLR;	//read low word of can fifo buffer
	fifo_buffer[fifo_write_ctr].feedrate=CAN1->sFIFOMailBox->RDHR;	//read high word of can fifo buffer
	CAN1->RF0R|=CAN_RF0R_RFOM0; 	//acknowlege that message box was empitied;
	return;
}

static inline void handle_mc_data_part_4()
{
	fifo_buffer[fifo_write_ctr].flags=CAN1->sFIFOMailBox->RDLR;	//read low word of can fifo buffer
	fifo_buffer[fifo_write_ctr].gcode_line=CAN1->sFIFOMailBox->RDHR;	//read high word of can fifo buffer
	CAN1->RF0R|=CAN_RF0R_RFOM0; 	//acknowlege that message box was empitied;
	if(++fifo_write_ctr>=FIFO_BUFFER_SIZE){
		if(fifo_read_ctr>0){
			fifo_write_ctr=0;
			//this has to be cleared when the fifo_read_ctr goes to 0
			flags_global_mc|=WRITE_CTR_UNDER_READ_CTR;
		}
		if(flags_global_mc&FIRST_MOVE){
			flags_global_mc|=BUFFER_FULL;
			//flags&=~FIRST_MOVE;
		}

		//stop_receiving_data();
	}
	if((flags_global_mc&FIRST_MOVE)&&(fifo_write_ctr!=0)){
		if(!(fifo_write_ctr%100)){	//this 100 should be named frame_size
			request_receiving_data();
			debug_ctr++;
		}
	}
}
/*
void stop_receiving_data()
{
	  //this has to be the Tx Mailbox with the lowest Identifier that it will be transmittet allways first
	  //Tx mailbox 0 could be used only for urgent messages like this
	  while(!(CAN1->TSR&CAN_TSR_TME0)){
			  	  ;
      }
	  CAN1->sTxMailBox[0].TDTR=0;
	  CAN1->sTxMailBox[0].TIR=0;
	  CAN1->sTxMailBox[0].TIR=STOP_RECEIVING_DATA_IDENTIFIER;
	  CAN1->sTxMailBox[0].TIR|=CAN_TI0R_TXRQ;
	  //request_receiving_data();
	  return;
}
*/
static inline void store_data()
{
	if(flags_global_mc&DATA_SET_PENDING)	//if a complete set xyz and gcode_line_nr was not aqured do not store a new one
		return;

	//there could also be a time_stamp,
	//the gcode_line_nr should be also included

	x_data.step_count=x_standpoint;
	y_data.step_count=y_standpoint;
	z_data.step_count=z_standpoint;
	b_data.step_count=b_standpoint;
	c_data.step_count=c_standpoint;
	n_line_data=gcode_line_number;

	//x_data.glass_scale_count=TIM2->CNT;
	flags_global_mc|=DATA_SET_PENDING;

	return;
}

static inline void send_data()
{
	static uint8_t counter=0;
	static uint8_t delay_ctr=0;
	delay_ctr++;
	if(delay_ctr>=10){
		delay_ctr=0;
		switch(++counter){
		/*
		case 1:
			send_position_message(x_data.step_count,y_data.step_count,MC_DATA_PART_1_ID);
			break;
		case 2:
			send_position_message(z_data.step_count,b_data.step_count,MC_DATA_PART_2_ID);
			break;
		case 3:
			send_position_message(c_data.step_count,n_line_data,MC_DATA_PART_3_ID);
			counter=0;
			flags_global_mc&=~DATA_SET_PENDING;
			break;
			*/
		case 1:
			send_position_message(x_standpoint ,y_standpoint,MC_DATA_PART_1_ID);
			break;
		case 2:
			send_position_message(z_standpoint ,b_data.step_count,MC_DATA_PART_2_ID);
			break;
		case 3:
			send_position_message(c_data.step_count,n_line_data,MC_DATA_PART_3_ID);
			counter=0;
			flags_global_mc&=~DATA_SET_PENDING;
			break;
		default:
			counter=0;//we should never get in here
			break;
		}
	}
}

void request_receiving_data()
{
	  //this has to be the Tx Mailbox with the lowest Identifier that it will be transmittet allways first
	  //Tx mailbox 0 could be used only for urgent messages like this
	  /*
	  while(!(CAN1->TSR&CAN_TSR_TME0)){
		  	  ;
	  }
	  */



	  CAN1->sTxMailBox[0].TDTR=0;
	  CAN1->sTxMailBox[0].TIR=0;
	  CAN1->sTxMailBox[0].TIR=CONTINUE_RECEIVING_DATA_IDENTIFIER;
	  CAN1->sTxMailBox[0].TIR|=CAN_TI0R_TXRQ;
	  return;
}

//maybe all the is motordirections could be made a seperate
static inline void update_standpoint_x()
{
		if(X_AXIS_POSITIV)
			x_standpoint++;
		else
			x_standpoint--;
		return;
}
static inline void check_target_reached_x()
{
if(x_standpoint==x_target){
		x_timer_stop();
		if(ALL_MOTORS_STOPPED)
			prepare_next_move();
	}
	return;
}

static inline void update_standpoint_y()
{
	if(Y_AXIS_POSITIV)
		y_standpoint++;
	else
		y_standpoint--;
	return;
}
static inline void check_target_reached_y()
{
	if(y_standpoint==y_target){
		y_timer_stop();
		if(ALL_MOTORS_STOPPED)
			prepare_next_move();
	}
	return;
}
static inline void update_standpoint_z()
{
	if(Z_AXIS_POSITIV)
		z_standpoint++;
	else
		z_standpoint--;
	return;
}
static inline void check_target_reached_z()
{
	if(z_standpoint==z_target){
			z_timer_stop();
			if(ALL_MOTORS_STOPPED)
				prepare_next_move();
		}
	return;
}
static inline void update_standpoint_b()
{
	if(B_AXIS_POSITIV)
		b_standpoint++;
	else
		b_standpoint--;
	return;
}

static inline void check_target_reached_b()
{
	if(b_standpoint==b_target){
		b_timer_stop();
		if(ALL_MOTORS_STOPPED)
			prepare_next_move();
	}
	return;
}
static inline void update_standpoint_c()
{
	if(C_AXIS_POSITIV)
		c_standpoint++;
	else
		c_standpoint--;
	return;
}

static inline void check_target_reached_c()
{
	if(c_standpoint==c_target){
		c_timer_stop();
		if(ALL_MOTORS_STOPPED)
			prepare_next_move();
	}
	return ;
}

static inline void prepare_next_move()
{
	beginning:

	fifo_read_ctr++;
	if(fifo_read_ctr>=(FIFO_BUFFER_SIZE-1))
		fifo_read_ctr=0;

	if((fifo_read_ctr!=0)&&(!(fifo_read_ctr%100))){ //this shoudl come maybe first
		request_receiving_data();
	}

	/*int stop=0;
	if(fifo_buffer[fifo_read_ctr].mc_data_part_3_HIGH==1692)
		stop=1;
	*/

	store_data();
	//send_data();

	//send_position_message(fifo_buffer[fifo_read_ctr].mc_data_part_3_HIGH,CAN_ID_GET_X_POSITION_ANSWER); //that was for debugging
	if(fifo_buffer[fifo_read_ctr].flags&(1<<FILE_END_BIT)){
		send_position_message(0,0,CAN_ID_PROGRAM_FINISHED);
		//flags|=PROGRAM_FINISHED;
		for(int i=0;i<1000000;i++) //wait to send the message before reseting the microcontroller
			;
		commands|=RESET_MICROCONTROLLER;
		return;
	}

	if((fifo_read_ctr<fifo_write_ctr)||((fifo_read_ctr>fifo_write_ctr)&&(flags_global_mc&WRITE_CTR_UNDER_READ_CTR))||flags_global_mc&FIRST_MOVE){
		flags_global_mc&=~FIRST_MOVE;
		save_targets();
		set_targets();
		int ret=0;
		ret=set_timer_speeds();//directions included, maybe also motor_start could be included
		if(ret){
			goto beginning;
		}
		//set_directions();
		start_motors();
	}
	else{
		flags_global_mc|=BUFFER_NOT_READY;
	}

	return;
}
//this has to be done with the floating point unit of the microcontroller!!!
//this is bad because it not only sets timer speeds but also the directions...
//this should be not the task of this function
static int set_timer_speeds()
{
	int32_t axis_not_starting=0;//if all dont start we have a problem
	//uint8_t motor_start_pc=((fifo_buffer[fifo_read_ctr].mc_data_part_4_LOW)>>8);
	float speed=fifo_buffer[fifo_read_ctr].feedrate;
	float x_line=0;
	float y_line=0;
	float z_line=0;
	//if(motor_start_pc&(1<<0))
	x_line=(float)(((float)x_target)-((float)x_standpoint_previous));
	//if(motor_start_pc&(1<<1))
	y_line=(float)(((float)y_target)-((float)y_standpoint_previous));
	//if(motor_start_pc&(1<<2))
	z_line=(float)(((float)z_target)-((float)z_standpoint_previous));
	float real_line=sqrt((x_line*x_line)+(y_line*y_line)+(z_line*z_line));


	motor_start=0;

	motor_b_direction=0;
	motor_c_direction=0;

	if(real_line==0){
		//if(b_target > 0)
			//b_target -= 6400;
		//if(c_target > 0)
			//c_target -= 6400;
		int b_move=b_target-b_standpoint_previous;
		int c_move=c_target-c_standpoint_previous;
		if(b_move||c_move){
			if(b_move){
				motor_start|=B_START;
				if(b_move<0)
					SET_B_AXIS_NEGATIVE_DIRECTION();
				if(b_move>0){
					SET_B_AXIS_POSITIVE_DIRECTION();
					motor_b_direction=1;
				}
			}
			if(c_move){
				motor_start|=C_START;
				if(c_move<0)
					SET_C_AXIS_NEGATIVE_DIRECTION();
				if(c_move>0){
					SET_C_AXIS_POSITIVE_DIRECTION();
					motor_c_direction=1;
				}
			}
			TIM3->ARR=65000;//just determine a slow speed
			TIM3->PSC=4;
			TIM4->ARR=65000;
			TIM4->PSC=4;
			return 0;
		}
		return 1;
	}

	float x_ratio=0;
	float y_ratio=0;
	float z_ratio=0;
	motor_x_direction=0;
	motor_y_direction=0;
	motor_z_direction=0;
	//there has to be a solution for if no motor starts!
	if(x_line){
		if(x_line<0){
			SET_X_AXIS_NEGATIVE_DIRECTION();
			x_line*=-1;
		}
		else{
			SET_X_AXIS_POSITIVE_DIRECTION();
			motor_x_direction=1;
		}
		if(x_line>1.0){//could be also >=1.0
			//if(motor_start_pc&(1<<0)){
				motor_start|=X_START;
			//}
		}
		else
			axis_not_starting|=(1<<0);

		x_ratio=real_line/x_line;
	}
	if(y_line){
		//motor_start|=Y_START; //whats that???!!!
		if(y_line<0){
			SET_Y_AXIS_NEGATIVE_DIRECTION();
			y_line*=-1;
		}
		else{
			SET_Y_AXIS_POSITIVE_DIRECTION();
			motor_y_direction=1;
		}
		if(y_line>1.0){
			//if(motor_start_pc&(1<<1)){
				motor_start|=Y_START;
			//}
		}
		else
			axis_not_starting|=(1<<1);

		y_ratio=real_line/y_line;
	}
	if(z_line){
		//motor_start|=Z_START; //whats that??????!!!!!
		if(z_line<0){
			SET_Z_AXIS_NEGATIVE_DIRECTION();
			z_line*=-1;
		}
		else{
			SET_Z_AXIS_POSITIVE_DIRECTION();
			motor_z_direction=1;
		}
		if(z_line>1.0){
			//if(motor_start_pc&(1<<2)){
				motor_start|=Z_START;
			//}
		}
		else
			axis_not_starting|=(1<<2);

		z_ratio=real_line/z_line;
	}

	if(axis_not_starting==7){
		motor_start=0;//now we got a problem
		return 1;
	}

	//removed this on 24.10.2023
	/*
	if(flags&FIRST_MOVE_NO_Z){
		motor_start&=~Z_START;
		flags&=~FIRST_MOVE_NO_Z;
	}
	*/


	float x_speed=speed*x_ratio;
	float y_speed=speed*y_ratio;
	float z_speed=speed*z_ratio;
	if(x_speed<65535){
		TIM9->ARR=x_speed;
		TIM9->PSC=1;
	}
	else
		calc_prescaler(x_speed,9);

	if(y_speed<65535){
		TIM10->ARR=y_speed;
		TIM10->PSC=1;
	}
	else
		calc_prescaler(y_speed,10);

	if(z_speed<65535){
		TIM11->ARR=z_speed;
		TIM11->PSC=1;
	}
	else
		calc_prescaler(z_speed,11);

	return 0;
}

static inline void set_targets()
{
	x_target=fifo_buffer[fifo_read_ctr].x_coordinate;
	y_target=fifo_buffer[fifo_read_ctr].y_coordinate;
	z_target=fifo_buffer[fifo_read_ctr].z_coordinate;
	b_target=fifo_buffer[fifo_read_ctr].B_axis;
	c_target=fifo_buffer[fifo_read_ctr].C_axis;
	gcode_line_number=fifo_buffer[fifo_read_ctr].gcode_line;

	return;
}

static void calc_prescaler(float frequency,int timer)
{
	float divisor=frequency/65535.0;
	divisor+=1;
	uint16_t prescaler=(uint16_t)divisor;
	//prescaler*=1000;//experiment for debugging
	float timer_value=frequency/prescaler;
	uint16_t timer_value_int=(int16_t)timer_value;
	if(timer==9){
		TIM9->PSC=prescaler;
		TIM9->ARR=timer_value_int;
	}
	else if(timer==10){
		TIM10->PSC=prescaler;
		TIM10->ARR=timer_value_int;
	}
	else if(timer==11){
			TIM11->PSC=prescaler;
			TIM11->ARR=timer_value_int;
		}
	return;
}
//this is obsolete:
/*
static inline void set_directions()
{
	uint8_t motor_direction=(uint8_t)(fifo_buffer[fifo_read_ctr].mc_data_part_3_LOW);
	uint8_t set_part=motor_direction;
	set_part&=0b00000111;
	uint8_t clear_part=motor_direction;
	clear_part&=0b00111000;
	MOTOR_DIRECTION_PORT->BSRR|=set_part;
	MOTOR_DIRECTION_PORT->BSRR|=(clear_part<<11);
	motor_x_direction=0;
	motor_y_direction=0;
	motor_z_direction=0;
	if(set_part&(1<<0))
		motor_x_direction=1;
	if(set_part&(1<<1))
		motor_y_direction=1;
	if(set_part&(1<<2))
		motor_z_direction=1;
	return;
}
*/

static inline void start_motors()
{
	switch(motor_start){
	 	 case 0b00000111:        xyz_axis_start();break;

	 	 case 0b00000011:        xy_axis_start();break;

	 	 case 0b00000101:        xz_axis_start();break;

	 	 case 0b00000110:        yz_axis_start();break;

	     case 0b00000010:        y_axis_start();break;

	     case 0b00000100:        z_axis_start();break;

	     case 0b00000001:        x_axis_start();break;

	     case 0b00001000:		 b_axis_start();break;

	     case 0b00010000:		 c_axis_start();break;
	}
	return;
}

static inline void x_axis_start(void){
        x_compare_enable();
        x_timer_start();
        motor_start_status=0b00000001;
}
static inline void y_axis_start(void){
        y_compare_enable();
        y_timer_start();
        motor_start_status=0b00000010;
}
static inline void z_axis_start(void){
        z_compare_enable();
        z_timer_start();
        motor_start_status=0b00000100;
}
static inline void xy_axis_start(void){
        x_compare_enable();
        y_compare_enable();
        x_timer_start();
        y_timer_start();
        motor_start_status=0b00000011;
}
static inline void xz_axis_start(void){
        x_compare_enable();
        z_compare_enable();
        x_timer_start();
        z_timer_start();
        motor_start_status=0b00000101;
}
static inline void yz_axis_start(void){
        y_compare_enable();
        z_compare_enable();
        y_timer_start();
        z_timer_start();
        motor_start_status=0b00000110;
}
static inline void xyz_axis_start(void){
        x_compare_enable();
        y_compare_enable();
        z_compare_enable();
        x_timer_start();
        y_timer_start();
        z_timer_start();
        motor_start_status=0b00000111;
}

static inline void b_axis_start(void){
	b_compare_enable();
	b_timer_start();
	motor_start_status=0b00001000;
}

static inline void c_axis_start(void){
	c_compare_enable();
	c_timer_start();
	motor_start_status=0b00010000;
}

static inline void save_targets()
{
	x_standpoint_previous=x_standpoint;
	y_standpoint_previous=y_standpoint;
	z_standpoint_previous=z_standpoint;
	b_standpoint_previous=b_standpoint;
	c_standpoint_previous=c_standpoint;
	return;
}

static inline void manual_motor_controll()
{
	if(flags_global_mc&X_MANUAL_MOVE)
		toggle_pin_x_axis();
	else if(flags_global_mc&Y_MANUAL_MOVE)
		toggle_pin_y_axis();
	else if(flags_global_mc&Z_MANUAL_MOVE)
		toggle_pin_z_axis();
	else if(flags_global_mc&B_MANUAL_MOVE)
		toggle_pin_b_axis();
	else if(flags_global_mc&C_MANUAL_MOVE)
		toggle_pin_c_axis();
}

static inline void toggle_pin_x_axis()
{

	if(GPIOE->ODR&GPIO_ODR_OD5)
		GPIOE->BSRR|=GPIO_BSRR_BR5;
	else
		GPIOE->BSRR|=GPIO_BSRR_BS5;

	if(motor_x_direction)
		x_standpoint++;
	else
		x_standpoint--;
	if(commands&HOMING_CYCLE_FLAG){
		if((GPIOB->IDR&(1<<1))){//if the endswitch of the x_axis is pressed
			wait();
			if((GPIOB->IDR&(1<<1))){
				x_standpoint=x_target;//that will make the motor stop
				//but it would be not good to stop at a uneven standpointnumber because that would be a half executed step
				if(commands&HOMING_CYCLE_FLAG)
					flags_global_mc|=X_HOMED;
			}
		}
	}
	if(x_standpoint==x_target){
		//send_data();
		manual_timer_stop();
		manual_compare_disable();
		flags_global_mc&=~X_MANUAL_MOVE;
	}
	static int delay_ctr = 0;
	delay_ctr++;
	if(delay_ctr == 40){
		delay_ctr = 0;
		//send_data();
	}
}

static inline void toggle_pin_y_axis()
{
	if(GPIOB->ODR&GPIO_ODR_OD8)
		GPIOB->BSRR|=GPIO_BSRR_BR8;
	else
		GPIOB->BSRR|=GPIO_BSRR_BS8;

	if(motor_y_direction)
		y_standpoint++;
	else
		y_standpoint--;
	if(commands&HOMING_CYCLE_FLAG){
		if((GPIOD->IDR&(1<<2))){//if the endswitch of the x_axis is pressed
			wait();
			if((GPIOD->IDR&(1<<2))){
				y_standpoint=y_target;//that will make the motor stop
				//but it would be not good to stop at a uneven standpointnumber because that would be a half executed step
				if(commands&HOMING_CYCLE_FLAG)
					flags_global_mc|=Y_HOMED;
			}
		}
	}

	if(y_standpoint==y_target){
		//send_data();
		manual_timer_stop();
		flags_global_mc&=~Y_MANUAL_MOVE;
	}
	static int delay_ctr = 0;
	delay_ctr++;
	if(delay_ctr == 40){
		delay_ctr = 0;
		//send_data();
	}
}

static inline void toggle_pin_z_axis()
{
	if(GPIOB->ODR&GPIO_ODR_OD9)
		GPIOB->BSRR|=GPIO_BSRR_BR9;
	else
		GPIOB->BSRR|=GPIO_BSRR_BS9;

	if(motor_z_direction)
		z_standpoint++;
	else
		z_standpoint--;
	if(commands&HOMING_CYCLE_FLAG){
		if((GPIOB->IDR&(1<<3))){//if the endswitch of the x_axis is pressed
			wait();
			if((GPIOB->IDR&(1<<3))){
				z_standpoint=z_target;//that will make the motor stop
				//but it would be not good to stop at a uneven standpointnumber because that would be a half executed step
				if(commands&HOMING_CYCLE_FLAG)
					flags_global_mc|=Z_HOMED;
			}
		}
	}
	if(commands & MEASURE_WCS_TOOL_FLAG || commands & MEASURE_ACTUAL_TOOL_FLAG){
		if(z_standpoint==-166400)
			TIM13->ARR=SPEED_2; //that direct change in hardware register, because there is no function for that yet
		if((GPIOB->IDR&(1<<4))){//if the endswitch of the x_axis is pressed
			wait();
			if((GPIOB->IDR&(1<<4))){
				if(commands & MEASURE_WCS_TOOL_FLAG || commands & MEASURE_ACTUAL_TOOL_FLAG){
					flags_global_mc|=MEASURED_TOOL;
					manual_timer_stop();
					flags_global_mc&=~Z_MANUAL_MOVE;
				}
			}
		}
	}

	if(z_standpoint==z_target){
		manual_timer_stop();
		flags_global_mc&=~Z_MANUAL_MOVE;
		return;
	}
	static int delay_ctr = 0;
	delay_ctr++;
	if(delay_ctr == 40){
		delay_ctr = 0;
		//send_data();
	}
}

static inline void toggle_pin_b_axis()
{
	if(GPIOA->ODR&GPIO_ODR_OD6)
		GPIOA->BSRR|=GPIO_BSRR_BR6;
	else
		GPIOA->BSRR|=GPIO_BSRR_BS6;

	if(motor_b_direction)
		b_standpoint++;
	else
		b_standpoint--;
	/*
	if(commands&HOMING_CYCLE_FLAG){
		if((GPIOD->IDR&(1<<2))){//if the endswitch of the x_axis is pressed
			wait();
			if((GPIOD->IDR&(1<<2))){
				y_standpoint=y_target;//that will make the motor stop
				//but it would be not good to stop at a uneven standpointnumber because that would be a half executed step
				if(commands&HOMING_CYCLE_FLAG)
					flags|=Y_HOMED;
			}
		}
	}
	*/

	if(b_standpoint==b_target){

		flags_global_mc&=~B_MANUAL_MOVE;
	}
}

static inline void toggle_pin_c_axis()
{
	if(GPIOB->ODR&GPIO_ODR_OD6)
		GPIOB->BSRR|=GPIO_BSRR_BR6;
	else
		GPIOB->BSRR|=GPIO_BSRR_BS6;

	if(motor_c_direction)
		c_standpoint++;
	else
		c_standpoint--;
	/*
	if(commands&HOMING_CYCLE_FLAG){
		if((GPIOD->IDR&(1<<2))){//if the endswitch of the x_axis is pressed
			wait();
			if((GPIOD->IDR&(1<<2))){
				y_standpoint=y_target;//that will make the motor stop
				//but it would be not good to stop at a uneven standpointnumber because that would be a half executed step
				if(commands&HOMING_CYCLE_FLAG)
					flags|=Y_HOMED;
			}
		}
	}
	*/

	if(c_standpoint==c_target){
		manual_timer_stop();
		flags_global_mc&=~C_MANUAL_MOVE;
	}
}

static inline void wait()
{
	for(int i=0;i<20;i++)
		;
}


/* USER CODE END 1 */
