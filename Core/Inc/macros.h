#ifndef MACROH
#define MACROH
//#include "C:/Users/Samuel/source/repos/HMILL/communication_protocol.cs"
#include "C:/Users/Samuel/Documents/HMILL_gui_cross_platform/communication_protocol.h"

#define DELAY_EXTINT 70


//I beliefe that a=1; is faster than a|=(1<<bit);
//Therefore I used that form, also the rest of the bits are not
//needed
//enables the relevant timer (bit 0 is counter enable (CEN))
#define x_timer_start() {TIM9->CR1=1;}
#define y_timer_start() {TIM10->CR1=1;}
#define z_timer_start() {TIM11->CR1=1;}
#define b_timer_start()	{TIM3->CR1=1;}
#define c_timer_start() {TIM4->CR1=1;}
#define init_timer_start() {TIM12->CR1=1;}
#define manual_timer_start() {TIM13->CR1=1;}
#define position_timer_start() {TIM3->CR1=1;}

#define x_timer_stop() {TIM9->CR1=0;motor_start_status&=~(1<<X_AXIS_MOVING);}
#define y_timer_stop() {TIM10->CR1=0;motor_start_status&=~(1<<Y_AXIS_MOVING);}
#define z_timer_stop() {TIM11->CR1=0;motor_start_status&=~(1<<Z_AXIS_MOVING);}
#define b_timer_stop() {TIM3->CR1=0;motor_start_status&=~(1<<B_AXIS_MOVING);}
#define c_timer_stop() {TIM4->CR1=0;motor_start_status&=~(1<<C_AXIS_MOVING);}
#define init_timer_stop() {TIM12->CR1=0;}
#define manual_timer_stop() {TIM13->CR1=0;}
#define position_timer_stop() {TIM3->CR1=0;}


//set direction macros
//look at the microcontrollers documentation for explanation what the BSRR register is

#define SET_X_AXIS_POSITIVE_DIRECTION() {MOTOR_DIRECTION_PORT->BSRR=(1<<0);}
#define SET_X_AXIS_NEGATIVE_DIRECTION() {MOTOR_DIRECTION_PORT->BSRR=(1<<16);}
//this has to be switched for the big machine...
#define SET_Y_AXIS_POSITIVE_DIRECTION() {MOTOR_DIRECTION_PORT->BSRR=(1<<17);}
#define SET_Y_AXIS_NEGATIVE_DIRECTION() {MOTOR_DIRECTION_PORT->BSRR=(1<<1);}
#define SET_Z_AXIS_POSITIVE_DIRECTION() {MOTOR_DIRECTION_PORT->BSRR=(1<<2);}
#define SET_Z_AXIS_NEGATIVE_DIRECTION() {MOTOR_DIRECTION_PORT->BSRR=(1<<18);}
#define SET_B_AXIS_POSITIVE_DIRECTION() {MOTOR_DIRECTION_PORT->BSRR=(1<<3);}
#define SET_B_AXIS_NEGATIVE_DIRECTION()	{MOTOR_DIRECTION_PORT->BSRR=(1<<19);}
#define SET_C_AXIS_POSITIVE_DIRECTION()	{MOTOR_DIRECTION_PORT->BSRR=(1<<4);}
#define SET_C_AXIS_NEGATIVE_DIRECTION()	{MOTOR_DIRECTION_PORT->BSRR=(1<<20);}



//enables the relevant bit in the compare enable register (bit 0 is capture compare
//1 output enable (CC1E))
//puts the certain pin on high (initial value))
#define x_compare_enable()      {TIM9->CCER=1;}
#define y_compare_enable()      {TIM10->CCER=1;}
#define z_compare_enable()      {TIM11->CCER=1;}
#define b_compare_enable()		{TIM3->CCER=1;}
#define c_compare_enable()		{TIM4->CCER=1;}
#define init_compare_enable()	{TIM12->CCER=1;}
#define init_compare_disable() 	{TIM12->CCER=0;}
#define manual_compare_enable() {TIM13->CCER=1;}
#define manual_compare_disable() {TIM13->CCER=0;}
#define position_timer_compare_enable() {TIM3->CCER=1;}
#define position_timer_compare_disable() {TIM3->CCER=0;}

//enables the relevant bit in the Timer INterrupt enable register DIER
#define x_interrupt_enable()  {TIM9->DIER|=TIM_DIER_CC1IE;}
#define y_interrupt_enable()  {TIM10->DIER|=TIM_DIER_CC1IE;}
#define z_interrupt_enable()  {TIM11->DIER|=TIM_DIER_CC1IE;}
#define b_interrupt_enable()  {TIM3->DIER|=TIM_DIER_CC1IE;}
#define c_interrupt_enable()  {TIM4->DIER|=TIM_DIER_CC1IE;}
#define init_interrupt_enable()  {TIM12->DIER|=TIM_DIER_CC1IE;}
#define init_interrupt_disable() {TIM12->DIER&=~TIM_DIER_CC1IE;}
#define manual_interrupt_enable() {TIM13->DIER|=TIM_DIER_CC1IE;}
#define manual_interrupt_disable() {TIM13->DIER&=~TIM_DIER_CC1IE;}
#define position_timer_interrupt_enable() {TIM3->DIER|=TIM_DIER_CC1IE;}
#define position_timer_interrupt_disable() {TIM3->DIER&=~TIM_DIER_CC1IE;}

//clears the Update interrupt flag in the Timer Status Register
//neccesary else the interrupt will immediately fire again //that has to be used if I delete the hal routine I guess
#define x_clear_interrupt()	{TIM9->SR=0;}
#define y_clear_interrupt() {TIM10->SR=0;}
#define z_clear_interrupt() {TIM11->SR=0;}
#define b_clear_interrupt() {TIM3->SR=0;}
#define c_clear_interrupt() {TIM4->SR=0;}
#define init_clear_interrupt() {TIM12->SR=0;}
#define manual_clear_interrupt() {TIM12->SR=0;}


#define FIFO_BUFFER_SIZE 3000
#define RESEARCH_DATA_BUFFER_SIZE 1000
#define CLOCK_FREQUENCY 168000000.0

//defines for flags
#define BUFFER_NOT_READY (1<<0)
#define PROGRAM_FINISHED_FLAG (1<<1)
#define B_MANUAL_MOVE (1<<2)
#define C_MANUAL_MOVE (1<<3)
#define CAN1_FIFO_OVERRUN ((uint32_t)(1<<10))
#define WRITE_CTR_UNDER_READ_CTR ((uint32_t)(1<<11))
#define BUFFER_NOT_EMPTY ((uint32_t)(1<<12))
#define BUFFER_FULL (1<<17)//that means buffer is completely filled before first move
#define FIRST_MOVE (1<<18) //that means first move has not yet started
#define FIRST_MOVE_IN_PROCESS (1<<19) //thats a flag wich prevents the buffer to be filled
								  //if the buffer is before the first move filled,
								  //the counter fifo_write_ctr goes to 0
								  //but the first move has not started yet wich means
								  //that the fifo_read_ctr is not yet at 0
								  //the consequenz would be that during the
								  //initial process of starting the first move the
								  //fifo_read_ctr would be set to 0, BUT during that time
								  //the buffer would be filled again from the fifo_write_ctr=0
								  //to prevent that there is this flag wich forbids to
								  // fill the buffer during the initial process of starting the
								  //first move
#define PROGRAM_RUNNING (1<<20)
#define X_MANUAL_MOVE (1<<21)
#define Y_MANUAL_MOVE (1<<22)
#define Z_MANUAL_MOVE (1<<23)
#define MACHINE_HOMED (1<<24)
#define BUFFER_FILLING_IN_PROGRESS (1<<25)
//#define FIRST_MOVE_NO_Z (1<<26)
#define Z_HOMED (1<<27)
#define X_HOMED (1<<28)
#define Y_HOMED (1<<29)
#define MEASURED_TOOL (1<<30)
#define DATA_SET_PENDING (1<<31)

//defines for commands
#define X_POSITION_REQUEST_FLAG (1<<0)
#define Y_POSITION_REQUEST_FLAG (1<<1)
#define Z_POSITION_REQUEST_FLAG (1<<2)
#define GO_TO_HOME (1<<3)		//thats the only way to get away from the endswitches
#define HOMING_CYCLE_FLAG (1<<4)
#define MOVE_X_POSITIVE (1<<5)
#define MOVE_X_NEGATIVE (1<<6)
#define MOVE_Y_POSITIVE (1<<7)
#define MOVE_Y_NEGATIVE (1<<8)
#define MOVE_Z_POSITIVE (1<<9)
#define MOVE_Z_NEGATIVE (1<<10)
#define MOVE_B_POSITIVE (1<<15)
#define MOVE_B_NEGATIVE (1<<16)
#define MOVE_C_POSITIVE (1<<17)
#define MOVE_C_NEGATIVE (1<<18)
//#define B_POSITION_REQUEST_FLAG (1<<19)
//#define C_POSITION_REQUEST_FLAG (1<<20)
#define START_PROGRAM (1<<11)
#define MEASURE_WCS_TOOL_FLAG (1<<12)
#define MEASURE_ACTUAL_TOOL_FLAG (1<<19)
#define SEND_DATA_REQUEST_PENDING (1<<13)
#define RESET_MICROCONTROLLER (1<<14)


//defines for direction plus or minus
#define POSITIVE 1
#define NEGATIVE 0

//thats for moving to 0 for function move_axis
#define MOVE_TO_ZERO 0
/*
//old version
//definitions for CAN receive FIFO mailbox identifier register
#define MC_DATA_PART_1 ((uint32_t)(10<<21))
#define MC_DATA_PART_2 ((uint32_t)(11<<21))
#define MC_DATA_PART_3 ((uint32_t)(12<<21))
#define MC_DATA_PART_4 ((uint32_t)(13<<21))


#define SEND_X_POSITION_DATA_IDENTIFIER ((uint32_t)(3<<21))
#define SEND_Y_POSITION_DATA_IDENTIFIER ((uint32_t)(4<<21))
#define SEND_Z_POSITION_DATA_IDENTIFIER ((uint32_t)(5<<21))

#define CAN_ID_MOVE_X_POS       ((uint32_t)(20<<21))
#define CAN_ID_MOVE_X_NEG       ((uint32_t)(21<<21))
#define CAN_ID_MOVE_Y_POS       ((uint32_t)(22<<21))
#define CAN_ID_MOVE_Y_NEG       ((uint32_t)(23<<21))
#define CAN_ID_MOVE_Z_POS       ((uint32_t)(24<<21))
#define CAN_ID_MOVE_Z_NEG       ((uint32_t)(25<<21))

#define CAN_ID_MOVE_SPEED_1     ((uint32_t)(26<<21))
#define CAN_ID_MOVE_SPEED_2     ((uint32_t)(27<<21))
#define CAN_ID_MOVE_SPEED_3     ((uint32_t)(28<<21))

#define CAN_ID_MOVE_INCREMENT_1 ((uint32_t)(29<<21))
#define CAN_ID_MOVE_INCREMENT_2 ((uint32_t)(30<<21))
#define CAN_ID_MOVE_INCREMENT_3 ((uint32_t)(31<<21))

//data is int32_t standpoint, +,- or m
#define CAN_ID_GET_Y_POSITION_ANSWER    ((uint32_t)(39<<21))
#define CAN_ID_GET_Z_POSITION_ANSWER    ((uint32_t)(40<<21))
#define CAN_ID_GO_TO_MACHINE_ZERO       ((uint32_t)(41<<21))

#define CAN_ID_X_POS_X_ABS				((uint32_t)(44<<21))
#define CAN_ID_Y_POS_Y_ABS				((uint32_t)(45<<21))
#define CAN_ID_Z_POS_Z_ABS				((uint32_t)(46<<21))
#define CAN_ID_GET_X_POSITION_ANSWER    ((uint32_t)(38<<21))      
#define CAN_ID_GCODE_LINE_NR			((uint32_t)(47<<21))
#define CAN_ID_HOMING_CYCLE				((uint32_t)(48<<21))
#define CAN_ID_MEASURE_TOOL				((uint32_t)(49<<21))
#define CAN_ID_PROGRAM_FINISHED			((uint32_t)(50<<21))



//definitions for CAN transmit FIFO mailbox identifier register
#define STOP_RECEIVING_DATA_IDENTIFIER ((uint32_t)(1<<21))
#define CONTINUE_RECEIVING_DATA_IDENTIFIER ((uint32_t)(2<<21))

#define CAN_ID_GET_X_POSITION_REQUEST   ((uint32_t)(32<<21)) //allways from toolcenter!! offset gets calculated in the computerprogram
#define CAN_ID_GET_Y_POSITION_REQUEST   ((uint32_t)(33<<21))
#define CAN_ID_GET_Z_POSITION_REQUEST   ((uint32_t)(34<<21))

#define CAN_ID_X_Y_POSITION             ((uint32_t)(42<<21))      //first 4 bytes x_standpoint int, second 4 bytes y_standpoint int
#define CAN_ID_Z_POSITION_GCODE_LINE_NUMBER ((uint32_t)(43<<21))  //first 4 bytes z_standpoint int, second 4 bytes gcode_line number (N-Number) int

*/

//new version
#define MC_DATA_PART_1_ID ((uint32_t)(MC_DATA_PART_1<<21))
#define MC_DATA_PART_2_ID ((uint32_t)(MC_DATA_PART_2<<21))
#define MC_DATA_PART_3_ID ((uint32_t)(MC_DATA_PART_3<<21))
#define MC_DATA_PART_4_ID ((uint32_t)(MC_DATA_PART_4<<21))


#define SEND_X_POSITION_DATA_ID ((uint32_t)(X_POSITION_ANSWER<<21))
#define SEND_Y_POSITION_DATA_ID ((uint32_t)(Y_POSITION_ANSWER<<21))
#define SEND_Z_POSITION_DATA_ID ((uint32_t)(Z_POSITION_ANSWER<<21))

#define CAN_ID_MOVE_X_POS       ((uint32_t)(MOVE_X_POS<<21))
#define CAN_ID_MOVE_X_NEG       ((uint32_t)(MOVE_X_NEG<<21))
#define CAN_ID_MOVE_Y_POS       ((uint32_t)(MOVE_Y_POS<<21))
#define CAN_ID_MOVE_Y_NEG       ((uint32_t)(MOVE_Y_NEG<<21))
#define CAN_ID_MOVE_Z_POS       ((uint32_t)(MOVE_Z_POS<<21))
#define CAN_ID_MOVE_Z_NEG       ((uint32_t)(MOVE_Z_NEG<<21))
#define CAN_ID_MOVE_B_POS		((uint32_t)(MOVE_B_POS<<21))
#define CAN_ID_MOVE_B_NEG		((uint32_t)(MOVE_B_NEG<<21))
#define CAN_ID_MOVE_C_POS		((uint32_t)(MOVE_C_POS<<21))
#define CAN_ID_MOVE_C_NEG		((uint32_t)(MOVE_C_NEG<<21))
#define CAN_ID_B_POSITION_REQUEST ((uint32_t)(B_POSITION_REQUEST<<21))
#define CAN_ID_C_POSITION_REQUEST ((uint32_t)(C_POSITION_REQUEST<<21))
#define CAN_ID_B_POSITION_ANSWER ((uint32_t)(B_POSITION_ANSWER<<21))
#define CAN_ID_C_POSITION_ANSWER ((uint32_t)(C_POSITION_ANSWER<<21))

#define CAN_ID_MOVE_SPEED_1     ((uint32_t)(STEP1<<21))
#define CAN_ID_MOVE_SPEED_2     ((uint32_t)(STEP50<<21))
#define CAN_ID_MOVE_SPEED_3     ((uint32_t)(STEP1000<<21))
/*
#define CAN_ID_MOVE_INCREMENT_1 ((uint32_t)(29<<21))
#define CAN_ID_MOVE_INCREMENT_2 ((uint32_t)(30<<21))
#define CAN_ID_MOVE_INCREMENT_3 ((uint32_t)(31<<21))
*/
//data is int32_t standpoint, +,- or m

#define CAN_ID_GET_X_POSITION_ANSWER    ((uint32_t)(X_POSITION_ANSWER<<21))
#define CAN_ID_GET_Y_POSITION_ANSWER    ((uint32_t)(Y_POSITION_ANSWER<<21))
#define CAN_ID_GET_Z_POSITION_ANSWER    ((uint32_t)(Z_POSITION_ANSWER<<21))
#define CAN_ID_GO_TO_MACHINE_ZERO       ((uint32_t)(GO_TO_MACHINE_ZERO<<21))

#define CAN_ID_X_POS_X_ABS				((uint32_t)(44<<21))
#define CAN_ID_Y_POS_Y_ABS				((uint32_t)(45<<21))
#define CAN_ID_Z_POS_Z_ABS				((uint32_t)(46<<21))
#define CAN_ID_GCODE_LINE_NR			((uint32_t)(47<<21))
#define CAN_ID_HOMING_CYCLE				((uint32_t)(HOMING_CYCLE<<21))
#define CAN_ID_MEASURE_WCS_TOOL			((uint32_t)(MEASURE_WCS_TOOL<<21))
#define MEASURE_WCS_TOOL_ANSWER_ID		((uint32_t)(MEASURE_WCS_TOOL_ANSWER<<21))
#define CAN_ID_MEASURE_ACTUAL_TOOL		((uint32_t)(MEASURE_ACTUAL_TOOL<<21))
#define MEASURE_ACTUAL_TOOL_ANSWER_ID	((uint32_t)(MEASURE_ACTUAL_TOOL_ANSWER<<21))
#define CAN_ID_PROGRAM_FINISHED			((uint32_t)(PROGRAM_FINISHED<<21))
#define CAN_ID_START_PROGRAM			((uint32_t)(START_PROGRAM_REQUEST<<21))
#define CAN_ID_STOP_PROGRAM				((uint32_t)(STOP_PROGRAM_REQUEST<<21))



//definitions for CAN transmit FIFO mailbox identifier register
//#define STOP_RECEIVING_DATA_IDENTIFIER ((uint32_t)(1<<21))
#define CONTINUE_RECEIVING_DATA_IDENTIFIER ((uint32_t)(CONTINUE_RECEIVING_DATA<<21))

#define CAN_ID_GET_X_POSITION_REQUEST   ((uint32_t)(X_POSITION_REQUEST<<21)) //allways from toolcenter!! offset gets calculated in the computerprogram
#define CAN_ID_GET_Y_POSITION_REQUEST   ((uint32_t)(Y_POSITION_REQUEST<<21))
#define CAN_ID_GET_Z_POSITION_REQUEST   ((uint32_t)(Z_POSITION_REQUEST<<21))

#define CAN_ID_MACHINE_HOME ((uint32_t)(MACHINE_HOME<<21))
/*
#define CAN_ID_X_Y_POSITION             ((uint32_t)(42<<21))      //first 4 bytes x_standpoint int, second 4 bytes y_standpoint int
#define CAN_ID_Z_POSITION_GCODE_LINE_NUMBER ((uint32_t)(43<<21))  //first 4 bytes z_standpoint int, second 4 bytes gcode_line number (N-Number) int
*/


//defines for motor_start
#define X_START (1<<0)
#define Y_START (1<<1)
#define Z_START (1<<2)
#define B_START (1<<3)
#define C_START (1<<4)




//defines for motor_start_status
#define X_AXIS_MOVING 0
#define Y_AXIS_MOVING 1
#define Z_AXIS_MOVING 2
#define B_AXIS_MOVING 3
#define C_AXIS_MOVING 4

//defines for motor directions every motor has its own int32_t for direction
#define X_AXIS_POSITIV motor_x_direction //if that variable is 1... than
#define Y_AXIS_POSITIV motor_y_direction
#define Z_AXIS_POSITIV motor_z_direction
#define B_AXIS_POSITIV motor_b_direction
#define C_AXIS_POSITIV motor_c_direction

#define MOTOR_DIRECTION_PORT GPIOE

//macro for motor_start_status
#define ALL_MOTORS_STOPPED !motor_start_status

//macro for checking wether no axis is in action
#define NO_ACTIVE_MOVE (!(flags_global_mc&(X_MANUAL_MOVE|Y_MANUAL_MOVE|Z_MANUAL_MOVE|B_MANUAL_MOVE|C_MANUAL_MOVE|PROGRAM_RUNNING)))



//macro for checking wether everything is ready for starting a program
#define MACHINE_READY_FOR_PROGRAM ((!(flags_global_mc&(X_MANUAL_MOVE|Y_MANUAL_MOVE|Z_MANUAL_MOVE)))&&(flags_global_mc&MACHINE_HOMED)&&((!x_standpoint)&&(!y_standpoint)&&(!z_standpoint)))





//defines for manual move
//for now random numbers to test
#define INCREMENT_1 1000
#define INCREMENT_2 100
#define INCREMENT_3 20

#define SPEED_1 2500
#define SPEED_2 15000
#define SPEED_3 40000
#endif
