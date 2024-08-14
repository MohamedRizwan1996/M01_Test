/*
 * Lifter_M01.h
 *
 *  Created on: Aug 7, 2024
 *      Author: Micropolis
 */

#ifndef INC_LIFTER_M01_H_
#define INC_LIFTER_M01_H_

#include "CubeMars.h"

#define LIFT_SPEED		-80.0
#define UNLIFT_SPEED	80.0

#define LIFTER_TASK_CYCLE_TIME		10
#define CURRENT_BRAKE_TIMEOUT		2000

typedef enum
{
	LIFTER_COMMAND_OFF,
	LIFTER_COMMAND_LIFT,
	LIFTER_COMMAND_UNLIFT,
	LIFTER_CONTINUOUS_RUN,
	LIFTER_COMMAND_INVALID = 0xFF
}LifterCommand_t;

typedef enum
{
	LIFTER_STATUS_OFF,
	LIFTER_STATUS_LIFTING,
	LIFTER_STATUS_UNLIFTING,
	LIFTER_STATUS_BRAKING,
	LIFTER_STATUS_CAN_DISABLE,
	LIFTER_STATUS_INVALID = 0xFF
}LifterStatus_t;


void Lifter_M01_Loop();

void Lift_Process();
void Unlift_Process();
void Lifter_Continuous_Run();

#endif /* INC_LIFTER_M01_H_ */
