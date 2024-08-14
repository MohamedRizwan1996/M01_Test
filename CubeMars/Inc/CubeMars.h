/*
 * CubeMars.h
 *
 *  Created on: Mar 11, 2024
 *      Author: Salah Souda @Micropolis
 *     Firmware Version V1.2
 */

#ifndef INC_CUBEMARS_H_
#define INC_CUBEMARS_H_

#include "main.h"
#include "can.h"
#include "usart.h"
#include "Status.h"
#include "stm32f4xx_hal.h"
#include <stdint.h>

/*____________________User define____________________*/
	#define Number_of_Motor			1
	#define motor_model				AK10_9
	#define CubMars_Firmware_V		1

#ifdef HAL_CAN_MODULE_ENABLED

	#define CUBEMARS_CAN			CAN1

#endif
#ifdef HAL_UART_MODULE_ENABLED

	#define CUBEMARS_USEUART		0		/*enable sending and receiving from UART*/
	#define UART_Timeout			1		/*ms*/
	#define UART_buffer_size		100		/*UART receiving buffer size*/

	#define CUBEMARS_1_UART			USART2
	#define CUBEMARS_2_UART			USART3
	#define CUBEMARS_3_UART			UART4
	#define CUBEMARS_4_UART			UART5

	#define CUBEMARS_1_huart		huart2
	#define CUBEMARS_2_huart		huart3
	#define CUBEMARS_3_huart		huart4
	#define CUBEMARS_4_huart		huart5

	#define CUBEMARS_1_DMA			DMA1_Stream5
	#define CUBEMARS_2_DMA			DMA1_Stream1
	#define CUBEMARS_3_DMA			DMA1_Stream2
	#define CUBEMARS_4_DMA			DMA1_Stream0

#endif

/*____________________end User define________________*/

#define AK10_9_POLE_PAIR		21		// = motor pole pairs
#define AK70_10_POLE_PAIR		21		// = motor pole pairs
#define AK60_60_POLE_PAIR		14		// = motor pole pairs
#define rpm						*rpm_to_erpm(motor_model)

typedef enum {
	not_set =0,
	AK70_10,
	AK60_6,
	AK10_9,
}CubeMars_model_t;

typedef enum {
	no_fault =0,
	motor_over_temperature,
	over_current,
	over_voltage,
	under_voltage,
	encoder_fault,
	MOSFET_over_temperature,
	motor_stall,
}error_code_t;

typedef enum {
	/*not included*/ UART_FW_VERSION = 0,
	/*not included*/ UART_JUMP_TO_BOOTLOADER,
	/*not included*/ UART_ERASE_NEW_APP,
	/*not included*/ UART_WRITE_NEW_APP_DATA,
	/*not included*/ UART_GET_VALUES, 			// Get motor operating parameters
					 UART_SET_DUTY, 			// Motor operates in duty cycle mode
					 UART_SET_CURRENT, 			// Motor operates in current loop mode
					 UART_SET_CURRENT_BRAKE, 	// Motor operates in current brake mode
					 UART_SET_RPM, 				// Motor operates in speed loop mode
					 UART_SET_POS, 				// Motor operates in position loop mode
					 UART_SET_HANDBRAKE, 		// Motor operates in hand-brake current loop mode
	/*not included*/ UART_SET_DETECT, 			// Motor real-time feedback current position command
	/*not included*/ UART_ROTOR_POSITION=22, 	// Motor feedback current position
					 UART_GET_VALUES_SETUP=50,	// Motor single or multiple parameter acquisition command
					 UART_SET_POS_SPD=91, 		// Motor operates in position-speed loop mode
					 UART_SET_POS_MULTI=92, 	// Set motor motion to single-turn mode
					 UART_SET_POS_SINGLE=93, 	// Set motor motion to multi-turn mode, range ±100 turns
					 UART_SET_POS_ORIGIN=95, 	// Set motor origin
					 UART_Disable = 0xFF,
} UART_PACKET_ID;

typedef enum {
	UART_GET_Default = 0,			/*request POS, SPD, Current, Temperature, error_code, and motor ID*/
	UART_GET_MOS_TEMPERATURE = 1,
	UART_GET_MOTOR_TEMPERATURE,
	UART_GET_OUTPUT_CURRENT,
	UART_GET_INPUT_CURRENT,
	UART_GET_Id_CURRENT,
	UART_GET_Iq_CURRENT,
	UART_GET_DUTY_CYCLE,
	UART_GET_SPEED,
	UART_GET_VOLTAGE,
	UART_GET_ERROR_FLAGE = 16,
	UART_GET_MOTOR_POSITION,
	UART_GET_MOTOR_ID,
	UART_REQUEST_DISABLE = 0xFF,
} UART_GET_PACKET;

typedef enum {
	CAN_PACKET_SET_DUTY = 0, 		// Duty Cycle Mode
	CAN_PACKET_SET_CURRENT, 		// Current Loop Mode
	CAN_PACKET_SET_CURRENT_BRAKE, 	// Current Brake Mode
	CAN_PACKET_SET_RPM, 			// Speed Mode
	CAN_PACKET_SET_POS, 			// Position Mode
	CAN_PACKET_SET_ORIGIN_HERE, 	// Set Origin Mode
	CAN_PACKET_SET_POS_SPD, 		// Position-Speed Loop Mode
	CAN_Disable = 0xFF,
} CAN_PACKET_ID;

typedef enum {
	Disable_MODE = 0,
	CURRENT_MODE, 		// Current Loop Mode
	DUTY_MODE, 				// Duty Cycle Mode
	CURRENT_BRAKE_MODE, 	// Current Brake Mode
	SPD_MODE, 			// Speed Mode
	POS_MODE, 			// Position Mode
	SET_ORIGIN_MODE, 	// Set Origin Mode
	POS_SPD_MODE, 		// Position-Speed Loop Mode
	HANDBRAKE_MODE,		// hand-brake current loop mode
} GENERAL_PACKET_ID;

typedef struct
{
	CubeMars_model_t 		model;
	uint8_t 				eid;

	CAN_PACKET_ID			CAN_mode;
	UART_PACKET_ID			UART_mode;
	UART_GET_PACKET			UART_request;

	GENERAL_PACKET_ID		General_mode;
	float 					duty_cycle;			/* set percent from 0 ~ 1.00 */
	float 					current;			/* set in A	*/
	float 					current_brake;		/* set in A	*/
	float					position;			/* set in degree*/
	float 					speed; 				/* set in rpm	*/
	float 					Accelera;			/* in POS_SPD mode only*/

	float 					hand_brake;

}CubeMars_control_register_t;

typedef struct
{
	/* read only */ float 			position;		/* degree */
	/* read only */ float 			speed;			/* rmp */
	/* read only */ float 			current;		/* A */
	/* read only */ uint8_t 		temp;			/* celsius */
	/* read only */ error_code_t	error;
	/* read only */ uint8_t			id;
}CubeMars_data_register_t;

typedef struct
{
	 CAN_TxHeaderTypeDef   MotorHeader;
	 uint8_t               MotorData[8];
	 uint32_t              MotorMailbox;
	 CAN_RxHeaderTypeDef   RxHeader;
	 uint8_t               RxData[8];
}CubeMars_CAN_message_t;

typedef struct
{
	uint8_t					data_len;
	uint8_t               	MotorData[18];
}CubeMars_UART_message_t;

/*____________________Static Variables Do not touch____________________*/
extern CubeMars_CAN_message_t 			MotorMessage;

#if CUBEMARS_USEUART
extern CubeMars_UART_message_t			UARTMessage;
extern uint8_t							UARTbuffer[4][UART_buffer_size];
extern uint8_t							STRAITUARTbuffer[4][UART_buffer_size];
extern uint8_t							index_input[4],index_prosses[4];
#endif
/*____________________User Control register____________________*/
extern CubeMars_control_register_t 		MotorControl[Number_of_Motor];			/* read/write register*/
extern CubeMars_data_register_t			Motor[Number_of_Motor];					/* read only  register*/
extern uint8_t							ID_check[Number_of_Motor];				/* read write  register*/
extern uint8_t							Motor_CAN_check;						/* read write  register*/
#if CUBEMARS_USEUART
extern CubeMars_data_register_t			UART_Motor[Number_of_Motor];			/* read only  register*/
#endif
/*____________________Static API's Do not touch____________________*/

void 		build_CAN_eid(CubeMars_CAN_message_t* __MotorMessage ,CubeMars_control_register_t* __MotorControl);
void 		build_data(uint8_t* buffer, int32_t data);
uint32_t 	rpm_to_erpm(CubeMars_model_t __motor_model);
void  		CubeMars_CAN_Message_parser(CAN_HandleTypeDef *hcan,CubeMars_data_register_t* __Motor);
#if CUBEMARS_USEUART
sys_status 	CubeMars_UART_Message_parser(CubeMars_data_register_t* __Motor,uint8_t __eid);
#endif

/*____________________User Easy API's____________________*/

#define 		set_motor_mode(__eid,__Md)				MotorControl[__eid-1].General_mode 	= __Md
#define 		set_motor_duty_cycle(__eid,__DC)		MotorControl[__eid-1].duty_cycle 	= __DC			/*work with CAN & UART*/
#define 		set_motor_position(__eid,__POS)			MotorControl[__eid-1].position 		= __POS			/*work with CAN & UART*/
#define 		set_motor_speed(__eid,__SPD)			MotorControl[__eid-1].speed 		= __SPD			/*work with CAN & UART*/
#define 		set_motor_accelera(__eid,__ACR)			MotorControl[__eid-1].Accelera 		= __ACR			/*work with CAN & UART SPD & POS mode only*/
#define 		set_motor_current(__eid,__C)			MotorControl[__eid-1].current 		= __C			/*work with CAN & UART*/
#define 		set_motor_brake(__eid,__CB)				MotorControl[__eid-1].current_brake = __CB			/*work with CAN & UART*/
#define 		set_motor_hand_brake(__eid,__HD)		MotorControl[__eid-1].hand_brake 	= __HD			/*work with UART only*/

#define			define_moror_id(__eid)					MotorControl[__eid-1].eid 			= __eid

#define			bulid_CubeMars_command(__eid)			CubeMars_bulid_command(&MotorControl[__eid-1])

/*____________________User API's____________________*/
				/*general mode API's*/
sys_status 		CubeMars_bulid_command(CubeMars_control_register_t* __MotorControl);
sys_status 		CubeMars_send_command(uint8_t UART_id);
				/*CAN mode API's*/
#ifdef HAL_CAN_MODULE_ENABLED
	void 		CAN_message_init(CubeMars_CAN_message_t* __MotorControl);
	sys_status 	CubeMars_Init_Motor(CAN_HandleTypeDef* hcan1);
	sys_status 	CubeMars_bulid_CAN_command(CubeMars_control_register_t* __MotorControl);
	sys_status 	send_message_over_CAN(CAN_HandleTypeDef* hcan);	/*CAN sending message rate 10ms*/
#endif
				/*UART mode API's*/
#ifdef HAL_UART_MODULE_ENABLED
	#if CUBEMARS_USEUART
	sys_status 	CubeMars_bulid_UART_command(CubeMars_control_register_t* __MotorControl);
	sys_status  CubeMars_bulid_UART_request(CubeMars_control_register_t* __MotorControl);
	sys_status 	send_message_over_UART(uint8_t UART_id);			/*UART sending message rate 100ms*/
	sys_status	CubeMars_UART_Messageing_Parser(uint8_t UART_id);	/*Call UART message Parser in while loop*/
	#endif
#endif

#endif /* INC_CUBEMARS_H_ */
