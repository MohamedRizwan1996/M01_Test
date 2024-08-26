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
static LifterCommand_t Previous_Lifter_Command = LIFTER_COMMAND_OFF;

LifterStatus_t Lifter_Status = LIFTER_STATUS_OFF;

static uint8_t State_Change_Check[2] = {0};

LifterMode_t Lifter_Mode = LIFTER_MODE_LIMITSWITCH;
static LifterMode_t Previous_Lifter_Mode = LIFTER_MODE_INVALID;

Calib_State_t Calib_State = CALIB_NOT_STARTED;

float Lifter_Pos_Start = 0.0;
float Lifter_Pos_End = 0.0;
float Lifter_Pos_Diff = 0.0;

static void Lifter_Brake_Current()
{
//	static uint32_t Last_Tick_Current_Brake = 0;
//
//	if(Previous_Lifter_Command != LIFTER_COMMAND_OFF)
//		Last_Tick_Current_Brake = HAL_GetTick();
//
//	if(HAL_GetTick() - Last_Tick_Current_Brake >= CURRENT_BRAKE_TIMEOUT)
//	{
//		MotorControl[0].CAN_mode = 	CAN_Disable;
//		Lifter_Status = LIFTER_STATUS_CAN_DISABLE;
//	}
//	else
//	{
		MotorControl[0].CAN_mode = 	CAN_PACKET_SET_CURRENT_BRAKE;
		MotorControl[0].current_brake = (float)(0.3);
//		Lifter_Status = LIFTER_STATUS_BRAKING;
//	}
}



static void Lift_Process()
{
	if(!LIMIT_SWITCH(LIMIT_SWITCH_UP))
	{
		Lifter_Status = LIFTER_STATUS_LIFTED;
		if(Lifter_Command == LIFTER_COMMAND_POS_CALIB)
		{
			Calib_State = CALIB_LIFTED;
			Lifter_Pos_End = Lifting_Angle;
		}
		else	Lifter_Command = LIFTER_COMMAND_OFF;
	}
	else if(!LIMIT_SWITCH(LIMIT_SWITCH_UP_END))
	{
		Lifter_Status = LIFTER_STATUS_LIFTED;
		if(Lifter_Command == LIFTER_COMMAND_POS_CALIB)
		{
			Calib_State = CALIB_ERROR;
			Lifter_Pos_End = Lifting_Angle;
		}
		else 	Lifter_Command = LIFTER_COMMAND_OFF;
	}
	else
	{
		MotorControl[0].CAN_mode = CAN_PACKET_SET_RPM;
		MotorControl[0].speed = LIFT_SPEED;
		Lifter_Status = LIFTER_STATUS_LIFTING;
		if(Lifter_Command == LIFTER_COMMAND_POS_CALIB)
			Calib_State = CALIB_LIFTING;
	}
}

static void Unlift_Process()
{
	if(!LIMIT_SWITCH(LIMIT_SWITCH_DOWN))
	{
		Lifter_Status = LIFTER_STATUS_UNLIFTED;
		if(Lifter_Command == LIFTER_COMMAND_POS_CALIB)
		{
			Calib_State = CALIB_UNLIFTED;
			Lifter_Pos_Start = Lifting_Angle;
		}
		else	Lifter_Command = LIFTER_COMMAND_OFF;
	}
	else if (!LIMIT_SWITCH(LIMIT_SWITCH_DOWN_END))
	{
		Lifter_Status = LIFTER_STATUS_UNLIFTED;
		if(Lifter_Command == LIFTER_COMMAND_POS_CALIB)
		{
			Calib_State = CALIB_ERROR;
			Lifter_Pos_Start = Lifting_Angle;
		}
		else	Lifter_Command = LIFTER_COMMAND_OFF;
	}
	else
	{
		MotorControl[0].CAN_mode = CAN_PACKET_SET_RPM;
		MotorControl[0].speed = UNLIFT_SPEED;
		Lifter_Status = LIFTER_STATUS_UNLIFTING;
		if(Lifter_Command == LIFTER_COMMAND_POS_CALIB)
					Calib_State = CALIB_UNLIFTING;
	}
}

static LifterCommand_t State_Prev = LIFTER_COMMAND_OFF;
static LifterCommand_t State_Next = LIFTER_COMMAND_OFF;
static uint32_t State_Change_Brake_Timeout = 0;
static void Lifter_Continuous_Run()
{
	LifterCommand_t State_Current = LIFTER_COMMAND_OFF;

	State_Current = State_Next;

	/* Startup sequence */
	if(State_Prev == LIFTER_COMMAND_OFF && State_Current == LIFTER_COMMAND_OFF)
	{
		State_Next = LIFTER_COMMAND_UNLIFT;
		State_Change_Check[0] = 0;
		State_Change_Check[1] = 0;
	}
	/* UnLifting Process*/
	else if((State_Prev == LIFTER_COMMAND_OFF || State_Prev == LIFTER_COMMAND_LIFT) && State_Current == LIFTER_COMMAND_UNLIFT)
	{
		if(!LIMIT_SWITCH(LIMIT_SWITCH_DOWN))
		{
			State_Next = LIFTER_COMMAND_LIFT;
			State_Change_Check[1] = 0;
			Lifter_Status = LIFTER_STATUS_UNLIFTED;
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
			MotorControl[0].speed = UNLIFT_SPEED;
			Lifter_Status = LIFTER_STATUS_UNLIFTING;
		}
//		else
//			Lifter_Command = LIFTER_COMMAND_OFF;
	}
	/* Lifting Process */
	else if((State_Prev == LIFTER_COMMAND_OFF || State_Prev == LIFTER_COMMAND_UNLIFT) && State_Current == LIFTER_COMMAND_LIFT)
	{
		if(!LIMIT_SWITCH(LIMIT_SWITCH_UP))
		{
			State_Next = LIFTER_COMMAND_UNLIFT;
			State_Change_Check[0] = 0;
			Lifter_Status = LIFTER_STATUS_LIFTED;
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
			MotorControl[0].speed = LIFT_SPEED;
			Lifter_Status = LIFTER_STATUS_LIFTING;
		}
//		else
//			Lifter_Command = LIFTER_COMMAND_OFF;
	}
	/* Safety condition to Brake the motor and come out of continuous mode when unknown condition occurs */
	else
	{
		State_Change_Check[0] = 0;
		State_Change_Check[1] = 0;
		State_Next = LIFTER_COMMAND_OFF;
		Lifter_Command = LIFTER_COMMAND_OFF;
		return;
	}

	/* Brake the motor and come out of continuous mode when the safety final limit switches are activated */
	if( (!LIMIT_SWITCH(LIMIT_SWITCH_DOWN_END)) || (!LIMIT_SWITCH(LIMIT_SWITCH_UP_END)) )
	{
		State_Change_Check[0] = 0;
		State_Change_Check[1] = 0;
		State_Next = LIFTER_COMMAND_OFF;
		Lifter_Command = LIFTER_COMMAND_OFF;
	}

	State_Prev = State_Current;
}


static void Lifter_Position_Calibration()
{
	if(Previous_Lifter_Command != LIFTER_COMMAND_POS_CALIB)
	{
		Calib_State = CALIB_STARTED;
	}

	switch(Calib_State)
	{
	case CALIB_STARTED:
	case CALIB_LIFTING:
		Lift_Process();
		break;

	case CALIB_LIFTED:
	case CALIB_UNLIFTING:
		Unlift_Process();
		break;

	case CALIB_UNLIFTED:
		Calib_State = CALIB_FINISHED_SUCCESSFULLY;
		break;

	case CALIB_FINISHED_SUCCESSFULLY:
		Lifter_Pos_Diff = Lifter_Pos_End - Lifter_Pos_Start;
		Lifter_Command = LIFTER_COMMAND_OFF;
		break;

	case CALIB_ERROR:
	case CALIB_STATE_INVALID:
	default:
		Lifter_Command = LIFTER_COMMAND_OFF;
		break;
	}
}


void Lifter_M01_Loop()
{
	if(Previous_Lifter_Mode != Lifter_Mode)
	{
		Lifter_Command = LIFTER_COMMAND_OFF;
	}

	if( (Lifter_Mode == LIFTER_MODE_LIMITSWITCH) || (Lifter_Mode == LIFTER_MODE_POSITION) )
	{
		switch(Lifter_Command)
		{
		case LIFTER_COMMAND_LIFT:
			Lift_Process();
			break;

		case LIFTER_COMMAND_UNLIFT:
			Unlift_Process();
			break;

		case LIFTER_COMMAND_POS_CALIB:
			Lifter_Position_Calibration();
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
	else
	{
		Lifter_Brake_Current();
	}

	Previous_Lifter_Mode = Lifter_Mode;
}
