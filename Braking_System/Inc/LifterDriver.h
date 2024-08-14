/*
 * LifterDriver.h
 *
 *  Created on: May 16, 2024
 *      Author: Shukri Dozom @ Micropolis
 */

#ifndef INC_LIFTERDRIVER_H_
#define INC_LIFTERDRIVER_H_

#include "CubeMars.h"

typedef enum
{
	LIFTER_CMD_OFF = 0,
	LIFTER_CMD_LIFT = 1,
	LIFTER_CMD_UNLIFT = 2
}LifterCommand_t;


void LifterDriverLoop();



#endif /* INC_LIFTERDRIVER_H_ */
