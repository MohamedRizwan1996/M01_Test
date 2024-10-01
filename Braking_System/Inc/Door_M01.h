/*
 * Door_M01.h
 *
 *  Created on: Sep 9, 2024
 *      Author: Micropolis
 */

#ifndef INC_DOOR_M01_H_
#define INC_DOOR_M01_H_

#include "CubeMars.h"
#include "PositionSensor.h"

#define LS_DOOR_LEFT_OPEN		2
#define LS_DOOR_LEFT_CLOSE		4
#define LS_DOOR_RIGHT_OPEN		3
#define LS_DOOR_RIGHT_CLOSE		5

#define DOOR_CLOSE_MAX_SPEED	-80.0
#define DOOR_OPEN_MAX_SPEED		80.0

#define DOOR_SPEED_ZERO			0.0

#define DOOR_SPEED_CHANGE_VALUE		10.0

#define CURRENT_BRAKE_TIMEOUT		2000

typedef enum
{
	DOOR_MODE_LIMITSWITCH,
	DOOR_MODE_POSITION,
	DOOR_MODE_INVALID = 0xFF
}DoorMode_t;

typedef enum
{
	DOOR_COMMAND_OFF,
	DOOR_COMMAND_OPEN,
	DOOR_COMMAND_CLOSE,
	DOOR_COMMAND_POS_CALIB,
	DOOR_CONTINUOUS_RUN,
	DOOR_COMMAND_INVALID = 0xFF
}DoorCommand_t;

typedef enum
{
	DOOR_STATUS_OFF,
	DOOR_STATUS_OPENING,
	DOOR_STATUS_OPENED,
	DOOR_STATUS_CLOSING,
	DOOR_STATUS_CLOSED,
	DOOR_STATUS_BRAKING,
	DOOR_STATUS_CAN_DISABLE,
	DOOR_STATUS_LIMIT_ERROR,
	DOOR_STATUS_INVALID = 0xFF
}DoorStatus_t;

//typedef enum
//{
//	CALIB_NOT_STARTED,
//	CALIB_STARTED,
//	CALIB_OPENING,
//	CALIB_OPENED,
//	CALIB_CLOSING,
//	CALIB_CLOSED,
//	CALIB_FINISHED_SUCCESSFULLY,
//	CALIB_ERROR,
//	CALIB_STATE_INVALID = 0xFF
//}Door_Calib_State_t;


typedef enum
{
	LEFT = -1,
	RIGHT = 1,
	BOTH_SIDE =0
}Side_t;

void Door_Left_M01_Loop();
void Door_Right_M01_Loop();
void Door_TimeDelay_Process();
void Door_State_Machine();


#endif /* INC_DOOR_M01_H_ */
