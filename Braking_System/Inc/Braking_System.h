/*
 * Braking_System.h
 *
 *  Created on: Apr 4, 2024
 *      Author: Shukri Dozom @Micropolis
 */

#ifndef INC_BRAKING_SYSTEM_H_
#define INC_BRAKING_SYSTEM_H_

#include "stdlib.h"
#include "math.h"
#include "string.h"
#include "can.h"
#include "CANBus.h"
#include "CubeMars.h"
#include "ErrorCode.h"


//Recovering variables and APIs:
extern uint8_t recoveryFlag;
extern uint8_t recoveringCounter;
void RecoverBrakeSystem();


#define CAN_MOTORS							hcan1
#define CAN_SLAVE							hcan2

#define BRAKE_CALIBRATION_DURATION_MS		1000
#define SET_ORIGIN_DURATION_MS				2000
#define RELEASE_DURATION_MS					2000

#define BRAKE_RELEASE_SPEED_RPM				600
#define BRAKE_RELEASE_ACC_RPMS				600

#define BRAKE_RELEASE_DEGREE				360

#define BRAKE_CURRENT_AMP					-0.3
#define BRAKE_CALIBRATION_CURRENT_AMP		-0.2
#define BRAKE_MAX_CURRENT_ABS_AMP			5


#define BRAKE_MESSAGE_DELAY_MS				10


// Comment or uncomment this macro for enabling/disabling the automated self-test mode of the brake system.
// When self-test mode is enabled, brake motors (based on Number_of_Motor macro from CubeMars.h ) will
// keep switching between the full-brake and the full-release statuses continuously, and the system
// will also keep considering the flags in order to determine the system-ready status.

//#define SELF_TEST_BRAKE_SYSTEM

//Positions based on brake level:
#define BRAKE_POSITION_LEVEL_0				360.0f //Fully-released
#define BRAKE_POSITION_LEVEL_1				200.0f
#define BRAKE_POSITION_LEVEL_2				110.0f
#define BRAKE_POSITION_LEVEL_3				100.0f
#define BRAKE_POSITION_LEVEL_4				80.0f
#define BRAKE_POSITION_LEVEL_5				70.0f
#define BRAKE_POSITION_LEVEL_6				60.0f
#define BRAKE_POSITION_LEVEL_7				50.0f
#define BRAKE_POSITION_LEVEL_8				45.0f
#define BRAKE_POSITION_LEVEL_9				40.0f
#define BRAKE_POSITION_LEVEL_10				0.0f  //Fully-braked


//Brake motors CAN IDs:
#define	MOTOR_CAN_ID_FRONT_RIGHT			1
#define	MOTOR_CAN_ID_FRONT_LEFT				2
#define	MOTOR_CAN_ID_REAR_LEFT				3
#define	MOTOR_CAN_ID_REAR_RIGHT				4

//Brake motors indexes:
#define MOTOR_INDEX_FRONT_RIGHT				MOTOR_CAN_ID_FRONT_RIGHT - 1
#define MOTOR_INDEX_FRONT_LEFT				MOTOR_CAN_ID_FRONT_LEFT  - 1
#define MOTOR_INDEX_REAR_LEFT				MOTOR_CAN_ID_REAR_LEFT   - 1
#define MOTOR_INDEX_REAR_RIGHT				MOTOR_CAN_ID_REAR_RIGHT  - 1

void SendBrakeValuesToMainBoard();
void SendMotorTemperatureToMainBoard();
void SendMotorCurrentToMainBoard();
void SendBrakeStatusToMainBoard();
sys_status CAN_TransmitToBrakeMotors();

void Braking_System_Loop();
void Braking_System_Loop_2();
void Braking_System_Config();
void CAN_Slave_Silent_Loopback_Test();


#ifdef SELF_TEST_BRAKE_SYSTEM

#define NUMBER_OF_TEMPERATURE_SAMPLES 10

extern uint8_t newBrakeValueFromDebug[Number_of_Motor];
extern uint8_t newBrakeValueFromDebugFlag;
extern float temperatureOfMotorBuffer[Number_of_Motor][NUMBER_OF_TEMPERATURE_SAMPLES];

void TestNewBrakeValuesFromDebug_Loop();

#endif


#endif /* INC_BRAKING_SYSTEM_H_ */
