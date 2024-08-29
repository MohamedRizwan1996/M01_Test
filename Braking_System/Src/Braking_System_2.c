/*
 * Braking_System_2.c
 *
 *  Created on: Jul 9, 2024
 *      Author: Micropolis
 */
#include "Braking_System.h"
#include "Braking_System_2.h"
#include "LimitSwitch.h"
#include "cmsis_os.h"

#define SET_SPEED_BRAKE_2	300.0

uint8_t startup = 0;

uint8_t Continuous_Run = 0;

uint8_t brake_check[3] = {0};

BrakeCommand_t Brake_Command = BRAKE_COMMAND_OFF;
BrakeCommand_t Previous_Brake_Command = BRAKE_COMMAND_INVALID;

BrakeStatus_t Brake_Status = BRAKE_STATUS_OFF;

Brake_Calib_State_t Brake_Calib_State = BRAKE_CALIB_NOT_STARTED;

float Brake_Pos_Start = 0.0;
float Brake_Pos_End = 0.0;
float Brake_Pos_Diff = 0.0;

static void Stop_Brake_CubeMars()
{
	//	static uint32_t Last_Tick_Current_Brake = 0;
	//
	//	if(Previous_Brake_Command != BRAKE_COMMAND_OFF)
	//		Last_Tick_Current_Brake = HAL_GetTick();
	//
	//	if(HAL_GetTick() - Last_Tick_Current_Brake >= CURRENT_BRAKE_TIMEOUT)
	//	{
	//		MotorControl[0].CAN_mode = 	CAN_Disable;
	//		Brake_Status = BRAKE_STATUS_CAN_DISABLE;
	//	}
	//	else
	//	{
			MotorControl[0].CAN_mode = 	CAN_PACKET_SET_CURRENT_BRAKE;
			MotorControl[0].current_brake = (float)(0.3);
	//		Lifter_Status = LIFTER_STATUS_BRAKING;
}


static inline float BRAKE_SPEED_INCREMENT_BY_FACTOR(float* value, float factor)
{
	return (float)((*value)+factor);
}

static inline float BRAKE_SPEED_DECREMENT_BY_FACTOR(float* value, float factor)
{
	return (float)((*value)-factor);
}

static float Brake_Apply_Speed = BRAKE_SPEED_ZERO;
static void Brake_Apply_Process()
{
	float Brake_Motor_Position = 0.0;
	Brake_Motor_Position = Motor[0].position;
	if(!LIMIT_SWITCH(BRAKE_LS_END))
	{
		Brake_Status = BRAKE_STATUS_FULL_BRAKE;
		if(Brake_Command == BRAKE_COMMAND_CALIB)
		{
			Brake_Calib_State = BRAKE_CALIB_FULL_BRAKE;
			Brake_Pos_End = Motor[0].position;
		}
		else	Brake_Command = BRAKE_COMMAND_OFF;
		Brake_Apply_Speed = BRAKE_SPEED_ZERO;
		return;
	}
	else if(Brake_Command == BRAKE_COMMAND_APPLY)
	{
		if(Brake_Motor_Position < (0.8*Brake_Pos_Diff+Brake_Pos_Start))
			BRAKE_SPEED_DECREMENT_BY_FACTOR(&Brake_Apply_Speed, BRAKE_SPEED_CHANGE_VALUE);
		else if(Brake_Motor_Position > (0.8*Brake_Pos_Diff+Brake_Pos_Start) && Brake_Motor_Position < (0.95*Brake_Pos_Diff+Brake_Pos_Start))
			BRAKE_SPEED_INCREMENT_BY_FACTOR(&Brake_Apply_Speed, BRAKE_SPEED_CHANGE_VALUE);
		else if(Brake_Motor_Position >= (0.95*Brake_Pos_Diff+Brake_Pos_Start) && Brake_Motor_Position <= (1*Brake_Pos_Diff+Brake_Pos_Start))
			Brake_Apply_Speed = BRAKE_SPEED_ZERO;
	}
	if(Brake_Apply_Speed < BRAKE_APPLY_MAX_SPEED)
		Brake_Apply_Speed = BRAKE_APPLY_MAX_SPEED;
	else if(Brake_Apply_Speed >=0)
		Brake_Apply_Speed = BRAKE_SPEED_ZERO;

	MotorControl[0].CAN_mode = CAN_PACKET_SET_RPM;
	MotorControl[0].speed = Brake_Apply_Speed;
	Brake_Status = BRAKE_STATUS_BRAKING;
	if(Brake_Command == BRAKE_COMMAND_CALIB)
		Brake_Calib_State = BRAKE_CALIB_BRAKING;

}

static float Brake_Release_Speed = BRAKE_SPEED_ZERO;
static void Brake_Release_Process()
{
	float Brake_Motor_Position = 0.0;
	Brake_Motor_Position = Motor[0].position;

	if(!LIMIT_SWITCH(BRAKE_LS_START))
	{
		Brake_Status = BRAKE_STATUS_FULL_RELEASE;
		if(Brake_Command == BRAKE_COMMAND_CALIB)
		{
			Brake_Calib_State = BRAKE_CALIB_FULL_RELEASE;
			Brake_Pos_Start = Motor[0].position;;
		}
		else	Brake_Command = BRAKE_COMMAND_OFF;
		Brake_Release_Speed = BRAKE_SPEED_ZERO;
		return;

	}

	else if(Brake_Command == BRAKE_COMMAND_RELEASE)
	{
		if(Brake_Motor_Position > (0.2*Brake_Pos_Diff+Brake_Pos_Start))
			BRAKE_SPEED_INCREMENT_BY_FACTOR(&Brake_Release_Speed, BRAKE_SPEED_CHANGE_VALUE);
		else if(Brake_Motor_Position > (0.05*Brake_Pos_Diff+Brake_Pos_Start) && Brake_Motor_Position < (0.2*Brake_Pos_Diff+Brake_Pos_Start))
			BRAKE_SPEED_DECREMENT_BY_FACTOR(&Brake_Release_Speed, BRAKE_SPEED_CHANGE_VALUE);
		else if(Brake_Motor_Position >= Brake_Pos_Start && Brake_Motor_Position <= (0.05*Brake_Pos_Diff+Brake_Pos_Start))
			Brake_Release_Speed = BRAKE_SPEED_ZERO;
	}
	if(Brake_Release_Speed > BRAKE_RELEASE_MAX_SPEED)
		Brake_Release_Speed = BRAKE_RELEASE_MAX_SPEED;
	else if(Brake_Release_Speed <=0)
		Brake_Release_Speed = BRAKE_SPEED_ZERO;

	MotorControl[0].CAN_mode = CAN_PACKET_SET_RPM;
	MotorControl[0].speed = Brake_Release_Speed;
	Brake_Status = BRAKE_STATUS_RELEASING;
	if(Brake_Command == BRAKE_COMMAND_CALIB)
		Brake_Calib_State = BRAKE_CALIB_RELEASING;

}


static void Brake_Continuous_Run()
{
	Brake_Command = BRAKE_COMMAND_OFF;
//	static LifterCommand_t State_Prev = LIFTER_COMMAND_OFF;
//	static LifterCommand_t State_Next = LIFTER_COMMAND_OFF;
//	static uint32_t State_Change_Brake_Timeout = 0;
//	static void Lifter_Continuous_Run()
//	{
//		LifterCommand_t State_Current = LIFTER_COMMAND_OFF;
//
//		State_Current = State_Next;
//
//		/* Startup sequence */
//		if(State_Prev == LIFTER_COMMAND_OFF && State_Current == LIFTER_COMMAND_OFF)
//		{
//			State_Next = LIFTER_COMMAND_UNLIFT;
//			State_Change_Check[0] = 0;
//			State_Change_Check[1] = 0;
//		}
//		/* UnLifting Process*/
//		else if((State_Prev == LIFTER_COMMAND_OFF || State_Prev == LIFTER_COMMAND_LIFT) && State_Current == LIFTER_COMMAND_UNLIFT)
//		{
//			if(!LIMIT_SWITCH(LIMIT_SWITCH_DOWN))
//			{
//				State_Next = LIFTER_COMMAND_LIFT;
//				State_Change_Check[1] = 0;
//				Lifter_Status = LIFTER_STATUS_UNLIFTED;
//			}
//			else if(State_Change_Check[0]==0)
//			{
//				Lifter_Brake_Current();
//				State_Change_Check[0] = 1;
//				State_Change_Brake_Timeout = HAL_GetTick();
//			}
//			else if((HAL_GetTick()-State_Change_Brake_Timeout) > 500)
//			{
//				MotorControl[0].CAN_mode = CAN_PACKET_SET_RPM;
//				MotorControl[0].speed = UNLIFT_SPEED;
//				Lifter_Status = LIFTER_STATUS_UNLIFTING;
//			}
//	//		else
//	//			Lifter_Command = LIFTER_COMMAND_OFF;
//		}
//		/* Lifting Process */
//		else if((State_Prev == LIFTER_COMMAND_OFF || State_Prev == LIFTER_COMMAND_UNLIFT) && State_Current == LIFTER_COMMAND_LIFT)
//		{
//			if(!LIMIT_SWITCH(LIMIT_SWITCH_UP))
//			{
//				State_Next = LIFTER_COMMAND_UNLIFT;
//				State_Change_Check[0] = 0;
//				Lifter_Status = LIFTER_STATUS_LIFTED;
//			}
//			else if(State_Change_Check[1]==0)
//			{
//				Lifter_Brake_Current();
//				State_Change_Check[1] = 1;
//				State_Change_Brake_Timeout = HAL_GetTick();
//			}
//			else if((HAL_GetTick()-State_Change_Brake_Timeout) > 500)
//			{
//				MotorControl[0].CAN_mode = CAN_PACKET_SET_RPM;
//				MotorControl[0].speed = LIFT_SPEED;
//				Lifter_Status = LIFTER_STATUS_LIFTING;
//			}
//	//		else
//	//			Lifter_Command = LIFTER_COMMAND_OFF;
//		}
//		/* Safety condition to Brake the motor and come out of continuous mode when unknown condition occurs */
//		else
//		{
//			State_Change_Check[0] = 0;
//			State_Change_Check[1] = 0;
//			State_Next = LIFTER_COMMAND_OFF;
//			Lifter_Command = LIFTER_COMMAND_OFF;
//			return;
//		}
//
//		/* Brake the motor and come out of continuous mode when the safety final limit switches are activated */
//		if( (!LIMIT_SWITCH(LIMIT_SWITCH_DOWN_END)) || (!LIMIT_SWITCH(LIMIT_SWITCH_UP_END)) )
//		{
//			State_Change_Check[0] = 0;
//			State_Change_Check[1] = 0;
//			State_Next = LIFTER_COMMAND_OFF;
//			Lifter_Command = LIFTER_COMMAND_OFF;
//		}
//
//		State_Prev = State_Current;
//	}
}



static void Brake_Position_Calibration()
{
	if(Previous_Brake_Command != BRAKE_COMMAND_CALIB)
		{
			Brake_Calib_State = BRAKE_CALIB_STARTED;
		}

		switch(Brake_Calib_State)
		{
		case BRAKE_CALIB_STARTED:
		case BRAKE_CALIB_BRAKING:
			Brake_Apply_Process();
			break;

		case BRAKE_CALIB_FULL_BRAKE:
		case BRAKE_CALIB_RELEASING:
			Brake_Release_Process();
			break;

		case BRAKE_CALIB_FULL_RELEASE:
			Brake_Calib_State = BRAKE_CALIB_FINISHED_SUCCESSFULLY;
			break;

		case BRAKE_CALIB_FINISHED_SUCCESSFULLY:
			Brake_Pos_Diff = Brake_Pos_End - Brake_Pos_Start;

		case BRAKE_CALIB_ERROR:
		case BRAKE_CALIB_STATE_INVALID:
		default:
			Brake_Command = BRAKE_COMMAND_OFF;
			break;
		}
}


void Braking_System_Loop_M01()
{
	switch(Brake_Command)
			{
			case BRAKE_COMMAND_APPLY:
				Brake_Apply_Process();
				break;

			case BRAKE_COMMAND_RELEASE:
				Brake_Release_Process();
				break;

			case BRAKE_COMMAND_CALIB:
				Brake_Position_Calibration();
				break;

			case BRAKE_CONTINUOUS_RUN:
				Brake_Continuous_Run();
				break;

			case BRAKE_COMMAND_OFF:
			default:
				Stop_Brake_CubeMars();
				break;
			}

	Previous_Brake_Command = Brake_Command;
}


void Braking_System_Loop_2()
{
	if(Continuous_Run == 0)
	{
		startup=0;
		for(uint8_t i=0; i<3; i++)
			brake_check[i] = 0;
		MotorControl[0].CAN_mode = CAN_PACKET_SET_CURRENT_BRAKE;
		MotorControl[0].current_brake = (float)(0.3);
		MotorControl[0].speed = (float)(0.0);
	}
	else if(Continuous_Run == 1)
	{
		if(!LIMIT_SWITCH(LIMIT_SWITCH_OPEN_END) && !LIMIT_SWITCH(LIMIT_SWITCH_CLOSE_END))
		{
			Continuous_Run = 0;
		}
		else if(LIMIT_SWITCH(LIMIT_SWITCH_CLOSE_END) && !LIMIT_SWITCH(LIMIT_SWITCH_OPEN_END))
		{
			if(brake_check[0]==0)
			{
				MotorControl[0].CAN_mode = CAN_PACKET_SET_CURRENT_BRAKE;
				MotorControl[0].current_brake = (float)(0.3);
				osDelay(200);
				brake_check[0] = 1;
			}
//			else
			{
				MotorControl[0].CAN_mode = CAN_PACKET_SET_RPM;
				MotorControl[0].speed = (float)(SET_SPEED_BRAKE_2);
				startup = 1;
				brake_check[1] = 0;
			}
		}
		else if(LIMIT_SWITCH(LIMIT_SWITCH_OPEN_END) && !LIMIT_SWITCH(LIMIT_SWITCH_CLOSE_END))
		{
			if(brake_check[1]==0)
			{
				MotorControl[0].CAN_mode = CAN_PACKET_SET_CURRENT_BRAKE;
				MotorControl[0].current_brake = (float)(0.3);
				osDelay(200);
				brake_check[1] = 1;
			}
//			else
			{
				MotorControl[0].CAN_mode = CAN_PACKET_SET_RPM;
				MotorControl[0].speed = (float)(-SET_SPEED_BRAKE_2);
				startup=1;
				brake_check[0] = 0;
			}
		}
		else if(LIMIT_SWITCH(LIMIT_SWITCH_OPEN_END) && LIMIT_SWITCH(LIMIT_SWITCH_CLOSE_END) && startup==0)
		{
			if(brake_check[2]==0)
			{
				MotorControl[0].CAN_mode = CAN_PACKET_SET_CURRENT_BRAKE;
				MotorControl[0].current_brake = (float)(0.3);
				osDelay(100);
				brake_check[2] = 1;
			}
			else
			{
				MotorControl[0].CAN_mode = CAN_PACKET_SET_RPM;
				MotorControl[0].speed = (float)(-SET_SPEED_BRAKE_2);
				startup=1;
				brake_check[0] = 0;
				brake_check[1] = 0;
			}
		}

	}
	else if(Continuous_Run == 2)
	{
		if(!LIMIT_SWITCH(LIMIT_SWITCH_CLOSE_END))
		{
			Continuous_Run = 0;
		}
		else
		{
			MotorControl[0].CAN_mode = CAN_PACKET_SET_RPM;
			MotorControl[0].speed = (float)(SET_SPEED_BRAKE_2);
		}
	}
	else if(Continuous_Run == 3)
	{
		if(!LIMIT_SWITCH(LIMIT_SWITCH_OPEN_END))
		{
			Continuous_Run = 0;
		}
		else
		{
			MotorControl[0].CAN_mode = CAN_PACKET_SET_RPM;
			MotorControl[0].speed = (float)(-SET_SPEED_BRAKE_2);
		}
	}
	else
	{
		Continuous_Run = 0;
	}


}
