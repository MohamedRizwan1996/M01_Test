/*
 * Door_M01.c
 *
 *  Created on: Aug 7, 2024
 *      Author: Micropolis
 */


#include "Door_M01.h"
#include "Braking_System.h"
#include "LimitSwitch.h"
#include "cmsis_os.h"

DoorCommand_t Door_Command = DOOR_COMMAND_OFF;
DoorCommand_t Door_Left_Command = DOOR_COMMAND_OFF;
DoorCommand_t Door_Right_Command = DOOR_COMMAND_OFF;

static DoorCommand_t Previous_Door_Command = DOOR_COMMAND_OFF;
static DoorCommand_t Previous_Left_Door_Command = DOOR_COMMAND_OFF;
static DoorCommand_t Previous_Right_Door_Command = DOOR_COMMAND_OFF;

DoorStatus_t Door_Left_Status = DOOR_STATUS_OFF;
DoorStatus_t Door_Right_Status = DOOR_STATUS_OFF;


DoorMode_t Door_Mode = DOOR_MODE_LIMITSWITCH;

static DoorMode_t Previous_Door_Mode = DOOR_MODE_INVALID;

//Door_Calib_State_t Door_Left_Calib_State = CALIB_NOT_STARTED;
//Door_Calib_State_t Door_Right_Calib_State = CALIB_NOT_STARTED;

float Door_Left_Pos_Start = 0.0;
float Door_Left_Pos_End = 0.0;
float Door_Left_Pos_Diff = 0.0;

float Door_Right_Pos_Start = 0.0;
float Door_Right_Pos_End = 0.0;
float Door_Right_Pos_Diff = 0.0;


static void Door_Brake_Current(Side_t side)
{
//	static uint32_t Last_Tick_Current_Brake = 0;
//
//	if(Previous_Door_Command != DOOR_COMMAND_OFF)
//		Last_Tick_Current_Brake = HAL_GetTick();
//
//	if(HAL_GetTick() - Last_Tick_Current_Brake >= CURRENT_BRAKE_TIMEOUT)
//	{
//		MotorControl[0].CAN_mode = 	CAN_Disable;
//		Door_Status = DOOR_STATUS_CAN_DISABLE;
//	}
//	else
//	{
//		MotorControl[0].CAN_mode = 	CAN_PACKET_SET_CURRENT_BRAKE;
//		MotorControl[0].current_brake = (float)(0.3);
//		Door_Status = DOOR_STATUS_BRAKING;
//	}
	if(side == LEFT)
	{
		MotorControl[1].CAN_mode = 	CAN_PACKET_SET_CURRENT_BRAKE;
		MotorControl[1].current_brake = (float)(0.3);
	}
	else if(side == RIGHT)
	{
		MotorControl[0].CAN_mode = 	CAN_PACKET_SET_CURRENT_BRAKE;
		MotorControl[0].current_brake = (float)(0.3);
	}
	else
	{
		MotorControl[0].CAN_mode = 	CAN_PACKET_SET_CURRENT_BRAKE;
		MotorControl[0].current_brake = (float)(0.3);
		MotorControl[1].CAN_mode = 	CAN_PACKET_SET_CURRENT_BRAKE;
		MotorControl[1].current_brake = (float)(0.3);
	}
}

static inline float DOOR_SPEED_INCREMENT_BY_FACTOR(float* value, float factor)
{
	return (float)((*value)+factor);
}

static inline float DOOR_SPEED_DECREMENT_BY_FACTOR(float* value, float factor)
{
	return (float)((*value)-factor);
}

static float Door_Left_Close_Speed = DOOR_SPEED_ZERO;
static void Door_Left_Close_Process()
{
	if(!LIMIT_SWITCH(LS_DOOR_LEFT_CLOSE))
	{
		Door_Left_Status = DOOR_STATUS_CLOSED;
		if(Door_Left_Command == DOOR_COMMAND_POS_CALIB)
		{
//			Door_Left_Calib_State = CALIB_CLOSED;
//			Door_Left_Pos_End = Lifting_Angle;
		}
		else	Door_Left_Command = DOOR_COMMAND_OFF;
		Door_Left_Close_Speed = DOOR_SPEED_ZERO;
		return;
	}

	if(Door_Mode != DOOR_MODE_LIMITSWITCH)
	{
		if(Door_Left_Command == DOOR_COMMAND_CLOSE)
		{
			if(Lifting_Angle < (0.8*Door_Left_Pos_Diff+Door_Left_Pos_Start))
				DOOR_SPEED_DECREMENT_BY_FACTOR(&Door_Left_Close_Speed, DOOR_SPEED_CHANGE_VALUE);
			else if(Lifting_Angle > (0.8*Door_Left_Pos_Diff+Door_Left_Pos_Start) && Lifting_Angle < (0.95*Door_Left_Pos_Diff+Door_Left_Pos_Start))
				DOOR_SPEED_INCREMENT_BY_FACTOR(&Door_Left_Close_Speed, DOOR_SPEED_CHANGE_VALUE);
			else if(Lifting_Angle >= (0.95*Door_Left_Pos_Diff+Door_Left_Pos_Start) && Lifting_Angle <= (1*Door_Left_Pos_Diff+Door_Left_Pos_Start))
				Door_Left_Close_Speed = DOOR_SPEED_ZERO;
		}
		if(Door_Left_Close_Speed < (DOOR_CLOSE_MAX_SPEED*LEFT))
			Door_Left_Close_Speed = (DOOR_CLOSE_MAX_SPEED*LEFT);
		else if(Door_Left_Close_Speed >=0)
			Door_Left_Close_Speed = DOOR_SPEED_ZERO;
	}
	else
		Door_Left_Close_Speed = DOOR_CLOSE_MAX_SPEED*LEFT;

	MotorControl[1].CAN_mode = CAN_PACKET_SET_RPM;
	MotorControl[1].speed = Door_Left_Close_Speed;
	Door_Left_Status = DOOR_STATUS_CLOSING;

//	if(Door_Command == DOOR_COMMAND_POS_CALIB)
//		Door_Left_Calib_State = CALIB_CLOSING;
}

static float Door_Right_Close_Speed = DOOR_SPEED_ZERO;
static void Door_Right_Close_Process()
{
	if(!LIMIT_SWITCH(LS_DOOR_RIGHT_CLOSE))
	{
		Door_Right_Status = DOOR_STATUS_CLOSED;
		if(Door_Right_Command == DOOR_COMMAND_POS_CALIB)
		{
//			Door_Left_Calib_State = CALIB_CLOSED;
//			Door_Left_Pos_End = Lifting_Angle;
		}
		else	Door_Right_Command = DOOR_COMMAND_OFF;
		Door_Right_Close_Speed = DOOR_SPEED_ZERO;
		return;
	}

	if(Door_Mode != DOOR_MODE_LIMITSWITCH)
	{
		if(Door_Right_Command == DOOR_COMMAND_CLOSE)
		{
			if(Lifting_Angle < (0.8*Door_Right_Pos_Diff+Door_Right_Pos_Start))
				DOOR_SPEED_DECREMENT_BY_FACTOR(&Door_Right_Close_Speed, DOOR_SPEED_CHANGE_VALUE);
			else if(Lifting_Angle > (0.8*Door_Right_Pos_Diff+Door_Right_Pos_Start) && Lifting_Angle < (0.95*Door_Right_Pos_Diff+Door_Right_Pos_Start))
				DOOR_SPEED_INCREMENT_BY_FACTOR(&Door_Right_Close_Speed, DOOR_SPEED_CHANGE_VALUE);
			else if(Lifting_Angle >= (0.95*Door_Right_Pos_Diff+Door_Right_Pos_Start) && Lifting_Angle <= (1*Door_Right_Pos_Diff+Door_Right_Pos_Start))
				Door_Right_Close_Speed = DOOR_SPEED_ZERO;
		}
		if(Door_Right_Close_Speed < (DOOR_CLOSE_MAX_SPEED*RIGHT))
			Door_Right_Close_Speed = (DOOR_CLOSE_MAX_SPEED*RIGHT);
		else if(Door_Right_Close_Speed >=0)
			Door_Right_Close_Speed = DOOR_SPEED_ZERO;
	}
	else
		Door_Right_Close_Speed = DOOR_CLOSE_MAX_SPEED*RIGHT;

	MotorControl[0].CAN_mode = CAN_PACKET_SET_RPM;
	MotorControl[0].speed = Door_Right_Close_Speed;
	Door_Left_Status = DOOR_STATUS_CLOSING;

//	if(Door_Command == DOOR_COMMAND_POS_CALIB)
//		Door_Left_Calib_State = CALIB_CLOSING;
}

static float Door_Left_Open_Speed = DOOR_SPEED_ZERO;
static void Door_Left_Open_Process()
{
	if(!LIMIT_SWITCH(LS_DOOR_LEFT_OPEN))
	{
		Door_Left_Status = DOOR_STATUS_OPENED;
		if(Door_Left_Command == DOOR_COMMAND_POS_CALIB)
		{
//			Door_Left_Calib_State = CALIB_OPENED;
//			Door_Left_Pos_Start = Lifting_Angle;
		}
		else	Door_Left_Command = DOOR_COMMAND_OFF;
		Door_Left_Open_Speed = DOOR_SPEED_ZERO;
		return;
	}

	if(Door_Mode != DOOR_MODE_LIMITSWITCH)
	{
		if(Door_Left_Command == DOOR_COMMAND_OPEN)
		{
			if(Lifting_Angle > (0.2*Door_Left_Pos_Diff+Door_Left_Pos_Start))
				DOOR_SPEED_INCREMENT_BY_FACTOR(&Door_Left_Open_Speed, DOOR_SPEED_CHANGE_VALUE);
			else if(Lifting_Angle > (0.05*Door_Left_Pos_Diff+Door_Left_Pos_Start) && Lifting_Angle < (0.2*Door_Left_Pos_Diff+Door_Left_Pos_Start))
				DOOR_SPEED_DECREMENT_BY_FACTOR(&Door_Left_Open_Speed, DOOR_SPEED_CHANGE_VALUE);
			else if(Lifting_Angle >= Door_Left_Pos_Start && Lifting_Angle <= (0.05*Door_Left_Pos_Diff+Door_Left_Pos_Start))
				Door_Left_Open_Speed = DOOR_SPEED_ZERO;
		}
		if(Door_Left_Open_Speed > (DOOR_OPEN_MAX_SPEED*LEFT))
			Door_Left_Open_Speed = (DOOR_OPEN_MAX_SPEED*LEFT);
		else if(Door_Left_Open_Speed <=0)
			Door_Left_Open_Speed = DOOR_SPEED_ZERO;
	}
	else
		Door_Left_Open_Speed = DOOR_OPEN_MAX_SPEED*LEFT;

	MotorControl[1].CAN_mode = CAN_PACKET_SET_RPM;
	MotorControl[1].speed = Door_Left_Open_Speed;
	Door_Left_Status = DOOR_STATUS_OPENING;

//	if(Door_Command == DOOR_COMMAND_POS_CALIB)
//		Door_Left_Calib_State = CALIB_OPENING;

}

static float Door_Right_Open_Speed = DOOR_SPEED_ZERO;
static void Door_Right_Open_Process()
{
	if(!LIMIT_SWITCH(LS_DOOR_RIGHT_OPEN))
	{
		Door_Right_Status = DOOR_STATUS_OPENED;
		if(Door_Right_Command == DOOR_COMMAND_POS_CALIB)
		{
//			Door_Left_Calib_State = CALIB_OPENED;
//			Door_Left_Pos_Start = Lifting_Angle;
		}
		else	Door_Right_Command = DOOR_COMMAND_OFF;
		Door_Right_Open_Speed = DOOR_SPEED_ZERO;
		return;
	}

	if(Door_Mode != DOOR_MODE_LIMITSWITCH)
	{
		if(Door_Right_Command == DOOR_COMMAND_OPEN)
		{
			if(Lifting_Angle > (0.2*Door_Right_Pos_Diff+Door_Right_Pos_Start))
				DOOR_SPEED_INCREMENT_BY_FACTOR(&Door_Right_Open_Speed, DOOR_SPEED_CHANGE_VALUE);
			else if(Lifting_Angle > (0.05*Door_Right_Pos_Diff+Door_Right_Pos_Start) && Lifting_Angle < (0.2*Door_Right_Pos_Diff+Door_Right_Pos_Start))
				DOOR_SPEED_DECREMENT_BY_FACTOR(&Door_Right_Open_Speed, DOOR_SPEED_CHANGE_VALUE);
			else if(Lifting_Angle >= Door_Right_Pos_Start && Lifting_Angle <= (0.05*Door_Right_Pos_Diff+Door_Right_Pos_Start))
				Door_Right_Open_Speed = DOOR_SPEED_ZERO;
		}
		if(Door_Right_Open_Speed > (DOOR_OPEN_MAX_SPEED*RIGHT))
			Door_Right_Open_Speed = (DOOR_OPEN_MAX_SPEED*RIGHT);
		else if(Door_Right_Open_Speed <=0)
			Door_Right_Open_Speed = DOOR_SPEED_ZERO;
	}
	else
		Door_Right_Open_Speed = DOOR_OPEN_MAX_SPEED*RIGHT;

	MotorControl[0].CAN_mode = CAN_PACKET_SET_RPM;
	MotorControl[0].speed = Door_Right_Open_Speed;
	Door_Left_Status = DOOR_STATUS_OPENING;

//	if(Door_Command == DOOR_COMMAND_POS_CALIB)
//		Door_Left_Calib_State = CALIB_OPENING;

}

//static void Door_Position_Calibration(){}
//{
//	if(Previous_Door_Command != DOOR_COMMAND_POS_CALIB)
//	{
//		Calib_State = CALIB_STARTED;
//	}
//
//	switch(Calib_State)
//	{
//	case CALIB_STARTED:
//	case CALIB_LIFTING:
//		Lift_Process();
//		break;
//
//	case CALIB_LIFTED:
//	case CALIB_UNLIFTING:
//		Unlift_Process();
//		break;
//
//	case CALIB_UNLIFTED:
//		Calib_State = CALIB_FINISHED_SUCCESSFULLY;
//		break;
//
//	case CALIB_FINISHED_SUCCESSFULLY:
//		Door_Pos_Diff = Door_Pos_End - Door_Pos_Start;
//
//	case CALIB_ERROR:
//	case CALIB_STATE_INVALID:
//	default:
//		Door_Command = DOOR_COMMAND_OFF;
//		break;
//	}
//}


void Door_Left_M01_Loop()
{
	if(Previous_Door_Mode != Door_Mode)
	{
		Door_Left_Command = DOOR_COMMAND_OFF;
	}
	if( (Door_Mode == DOOR_MODE_LIMITSWITCH) || (Door_Mode == DOOR_MODE_POSITION) )
	{
		switch(Door_Left_Command)
		{
		case DOOR_COMMAND_CLOSE:
			Door_Left_Close_Process();
			break;

		case DOOR_COMMAND_OPEN:
			Door_Left_Open_Process();
			break;

//		case DOOR_COMMAND_POS_CALIB:
//			Door_Position_Calibration();
//			break;

		case DOOR_COMMAND_OFF:
		default:
			Door_Brake_Current(LEFT);
			break;
		}

		Previous_Left_Door_Command = Door_Left_Command;
	}
	else
	{
		Door_Brake_Current(LEFT);
	}

	Previous_Door_Mode = Door_Mode;
}


void Door_Right_M01_Loop()
{
	if(Previous_Door_Mode != Door_Mode)
	{
		Door_Right_Command = DOOR_COMMAND_OFF;
	}

	if( (Door_Mode == DOOR_MODE_LIMITSWITCH) || (Door_Mode == DOOR_MODE_POSITION) )
	{
		switch(Door_Right_Command)
		{
		case DOOR_COMMAND_CLOSE:
			Door_Right_Close_Process();
			break;

		case DOOR_COMMAND_OPEN:
			Door_Right_Open_Process();
			break;

//		case DOOR_COMMAND_POS_CALIB:
//			Door_Position_Calibration();
//			break;

		case DOOR_CONTINUOUS_RUN:
//			Door_Continuous_Run();
			break;

		case DOOR_COMMAND_OFF:
		default:
			Door_Brake_Current(RIGHT);
			break;
		}

		Previous_Right_Door_Command = Door_Right_Command;
	}
	else
	{
		Door_Brake_Current(RIGHT);
	}

	Previous_Door_Mode = Door_Mode;
}


void Door_State_Machine()
{
	static uint32_t Door_Close_TimeDelay = 0;
	static uint32_t Door_Open_TimeDelay = 0;
	if(Previous_Door_Mode != Door_Mode)
	{
		Door_Command = DOOR_COMMAND_OFF;
	}

	if( (Door_Mode == DOOR_MODE_LIMITSWITCH) || (Door_Mode == DOOR_MODE_POSITION) )
	{
		switch(Door_Command)
		{
		case DOOR_COMMAND_CLOSE:
			if(Door_Left_Command == DOOR_COMMAND_OFF && Door_Right_Command == DOOR_COMMAND_OFF)
				Door_Close_TimeDelay = HAL_GetTick();

			if(!LIMIT_SWITCH(LS_DOOR_LEFT_CLOSE))
				Door_Left_Command = DOOR_COMMAND_CLOSE;

			if((Door_Close_TimeDelay - HAL_GetTick()) > 2000)
				Door_Right_Command = DOOR_COMMAND_CLOSE;
			break;

		case DOOR_COMMAND_OPEN:
			if(Door_Left_Command == DOOR_COMMAND_OFF && Door_Right_Command == DOOR_COMMAND_OFF)
				Door_Open_TimeDelay = HAL_GetTick();

			if(!LIMIT_SWITCH(LS_DOOR_RIGHT_OPEN))
				Door_Right_Command = DOOR_COMMAND_OPEN;

			if((Door_Open_TimeDelay - HAL_GetTick()) > 2000)
				Door_Left_Command = DOOR_COMMAND_OPEN;
			break;

//		case DOOR_COMMAND_POS_CALIB:
//			Door_Position_Calibration();
//			break;

		case DOOR_COMMAND_OFF:
		default:
			Door_Brake_Current(BOTH_SIDE);
			break;
		}

		Previous_Door_Command = Door_Command;
	}
	else
	{
		Door_Brake_Current(BOTH_SIDE);
	}

	if(Door_Left_Command == DOOR_COMMAND_OFF && Door_Right_Command == DOOR_COMMAND_OFF)
		Door_Command = DOOR_COMMAND_OFF;

	Previous_Door_Mode = Door_Mode;
}


