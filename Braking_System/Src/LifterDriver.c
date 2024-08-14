/*
 * LifterDriver.c
 *
 *  Created on: May 16, 2024
 *      Author: Shukri Dozom @ Micropolis
 */

#include "LifterDriver.h"


//static LifterCommand_t lifterCommand = LIFTER_CMD_OFF;


void LifterDriverLoop(){}
//{
//	const float liftingSpeed = -100;
//	const float unliftingSpeed = 100;
//	const uint8_t lifterMotorCAN_ID = 25;
//	const uint32_t transmissionRateMs = 10;
//	static LifterCommand_t previousLifterCommand;
//	static CubeMars_control_register_t lifterMotor;
//
//	static uint32_t lastTick = 0;
//	static uint32_t lastTickBrakeCurrent = 0;
//	const uint32_t brakeCurrentTimeoutMs = 500;
//
//	lifterMotor.eid = lifterMotorCAN_ID;
//	uint8_t control0,control1;
//	control0 = HAL_GPIO_ReadPin(LifterControl0_GPIO_Port, LifterControl0_Pin);
//	control1 = HAL_GPIO_ReadPin(LifterControl1_GPIO_Port, LifterControl1_Pin);
//
//	if(control0 == 0 && control1 == 1)
//		lifterCommand = LIFTER_CMD_LIFT;
//	else if(control0 == 1 && control1 == 0)
//		lifterCommand = LIFTER_CMD_UNLIFT;
//	else
//		lifterCommand = LIFTER_CMD_OFF;
//
//	if(HAL_GetTick() - lastTick >= transmissionRateMs)
//	{
//		lastTick = HAL_GetTick();
//
//		switch(lifterCommand)
//		{
//
//		case LIFTER_CMD_LIFT:
//			lifterMotor.CAN_mode = CAN_PACKET_SET_RPM;
//			lifterMotor.speed = liftingSpeed;
//			break;
//		case LIFTER_CMD_UNLIFT:
//			lifterMotor.CAN_mode = CAN_PACKET_SET_RPM;
//			lifterMotor.speed = unliftingSpeed;
//			break;
//
//		case LIFTER_CMD_OFF:
//		default:
//
//		if(previousLifterCommand != LIFTER_CMD_OFF)
//			lastTickBrakeCurrent = HAL_GetTick();
//		if(HAL_GetTick() - lastTickBrakeCurrent >= brakeCurrentTimeoutMs)
//		{
//			lifterMotor.CAN_mode = 	CAN_Disable;
//		}
//		else
//		{
//			lifterMotor.CAN_mode = 	CAN_PACKET_SET_CURRENT_BRAKE;
//			lifterMotor.current_brake = 0.1;
//		}
//
//		break;
//		}
//
//		CubeMars_bulid_CAN_command(&lifterMotor);
//			if( sys_ok == send_message_over_CAN(&hcan1))
//				HAL_Delay(1);
//
//		previousLifterCommand = lifterCommand;
//
//
//	}
//}
