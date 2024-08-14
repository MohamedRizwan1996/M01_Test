/*
 * Lifter_M01.c
 *
 *  Created on: Aug 7, 2024
 *      Author: Micropolis
 */


#include "Lifter_M01.h"
#include "Braking_System.h"
#include "LimitSwitch.h"
#include "cmsis_os.h"

LifterCommand_t Lifter_Command = LIFTER_COMMAND_OFF;

LifterStatus_t Lifter_Status = LIFTER_STATUS_OFF;

uint8_t State_Change_Check[2] = {0};

static LifterCommand_t Previous_Lifter_Command;

static void Lifter_Brake_Current()
{
	static uint32_t Last_Tick_Current_Brake = 0;

	if(Previous_Lifter_Command != LIFTER_COMMAND_OFF)
		Last_Tick_Current_Brake = HAL_GetTick();

	if(HAL_GetTick() - Last_Tick_Current_Brake >= CURRENT_BRAKE_TIMEOUT)
	{
		MotorControl[0].CAN_mode = 	CAN_Disable;
		Lifter_Status = LIFTER_STATUS_CAN_DISABLE;
	}
	else
	{
		MotorControl[0].CAN_mode = 	CAN_PACKET_SET_CURRENT_BRAKE;
		MotorControl[0].current_brake = (float)(0.1);
		Lifter_Status = LIFTER_STATUS_BRAKING;
	}
}

void Lifter_M01_Loop()
{
	switch(Lifter_Command)
	{
	case LIFTER_COMMAND_LIFT:
		Lift_Process();
		break;

	case LIFTER_COMMAND_UNLIFT:
		Unlift_Process();
		break;

	case LIFTER_CONTINUOUS_RUN:
		Lifter_Continuous_Run();
		break;

	case LIFTER_COMMAND_OFF:
	default:
		Lifter_Brake_Current();
		break;
	}

	Previous_Lifter_Command = Lifter_Command;
}


void Lift_Process()
{
	if(!LIMIT_SWITCH(LIMIT_SWITCH_OPEN) || !LIMIT_SWITCH(LIMIT_SWITCH_OPEN_END))
	{
		Lifter_Command = LIFTER_COMMAND_OFF;
	}
	else
	{
		MotorControl[0].CAN_mode = CAN_PACKET_SET_RPM;
		MotorControl[0].speed = LIFT_SPEED;
		Lifter_Status = LIFTER_STATUS_LIFTING;
	}
}

void Unlift_Process()
{
	if(!LIMIT_SWITCH(LIMIT_SWITCH_CLOSE) || !LIMIT_SWITCH(LIMIT_SWITCH_CLOSE_END))
	{
		Lifter_Command = LIFTER_COMMAND_OFF;
	}
	else
	{
		MotorControl[0].CAN_mode = CAN_PACKET_SET_RPM;
		MotorControl[0].speed = UNLIFT_SPEED;
		Lifter_Status = LIFTER_STATUS_UNLIFTING;
	}
}

void Lifter_Continuous_Run()
{
	static LifterCommand_t State_Prev = LIFTER_COMMAND_OFF;
	static LifterCommand_t State_Next = LIFTER_COMMAND_OFF;
	static uint32_t State_Change_Brake_Timeout = 0;
	LifterCommand_t State_Current = LIFTER_COMMAND_OFF;

	State_Current = State_Next;

	/* Startup sequence */
	if(State_Prev == LIFTER_COMMAND_OFF && State_Current == LIFTER_COMMAND_OFF)
	{
		if(LIMIT_SWITCH(LIMIT_SWITCH_OPEN) && LIMIT_SWITCH(LIMIT_SWITCH_CLOSE) && LIMIT_SWITCH(LIMIT_SWITCH_OPEN_END) && LIMIT_SWITCH(LIMIT_SWITCH_CLOSE_END))
		{
			MotorControl[0].CAN_mode = CAN_PACKET_SET_RPM;
			MotorControl[0].speed = UNLIFT_SPEED;
			Lifter_Status = LIFTER_STATUS_UNLIFTING;
		}
		State_Prev = LIFTER_COMMAND_UNLIFT;
		State_Next = LIFTER_COMMAND_LIFT;
		State_Change_Check[0] = 0;
		State_Change_Check[1] = 0;
	}
	/* Lifting Process*/
	else if(State_Prev == LIFTER_COMMAND_UNLIFT && State_Current == LIFTER_COMMAND_LIFT)
	{
		if(!LIMIT_SWITCH(LIMIT_SWITCH_OPEN))
		{
			State_Prev = LIFTER_COMMAND_LIFT;
			State_Next = LIFTER_COMMAND_UNLIFT;
			State_Change_Check[1] = 0;
		}
		else if(State_Change_Check[0]==0)
		{
			Lifter_Brake_Current();
			State_Change_Check[0] = 1;
			State_Change_Brake_Timeout = HAL_GetTick();
		}
		else if((HAL_GetTick()-State_Change_Brake_Timeout) > 500)
		{
			MotorControl[0].CAN_mode = CAN_PACKET_SET_RPM;
			MotorControl[0].speed = LIFT_SPEED;
			Lifter_Status = LIFTER_STATUS_LIFTING;
		}
		else
			Lifter_Command = LIFTER_COMMAND_OFF;
	}
	/* Unlifting Process */
	else if(State_Prev == LIFTER_COMMAND_LIFT && State_Current == LIFTER_COMMAND_UNLIFT)
	{
		if(!LIMIT_SWITCH(LIMIT_SWITCH_CLOSE))
		{
			State_Prev = LIFTER_COMMAND_UNLIFT;
			State_Next = LIFTER_COMMAND_LIFT;
			State_Change_Check[0] = 0;
		}
		else if(State_Change_Check[1]==0)
		{
			Lifter_Brake_Current();
			State_Change_Check[1] = 1;
			State_Change_Brake_Timeout = HAL_GetTick();
		}
		else if((HAL_GetTick()-State_Change_Brake_Timeout) > 500)
		{
			MotorControl[0].CAN_mode = CAN_PACKET_SET_RPM;
			MotorControl[0].speed = UNLIFT_SPEED;
			Lifter_Status = LIFTER_STATUS_UNLIFTING;
		}
		else
			Lifter_Command = LIFTER_COMMAND_OFF;
	}
	/* Safety condition to Brake the motor and come out of continuous mode when unknown condition occurs */
	else
	{
		State_Prev = LIFTER_COMMAND_OFF;
		State_Next = LIFTER_COMMAND_OFF;
		Lifter_Command = LIFTER_COMMAND_OFF;
		return;
	}

	/* Brake the motor and come out of continuous mode when the safety final limit switches are activated */
	if( (!LIMIT_SWITCH(LIMIT_SWITCH_CLOSE_END)) || (!LIMIT_SWITCH(LIMIT_SWITCH_OPEN_END)) )
	{
		State_Change_Check[0] = 0;
		State_Change_Check[1] = 0;
		Lifter_Command = LIFTER_COMMAND_OFF;
	}
}
