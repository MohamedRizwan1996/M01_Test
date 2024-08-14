/*
 * LimitSwitch.h
 *
 *  Created on: Mar 8, 2024
 *      Author: Salah Souda @Micropolis
 */

#ifndef INC_LIMITSWITCH_H_
#define INC_LIMITSWITCH_H_

#include "gpio.h"

#define LIMIT_SWITCH_CLOSE	3
#define LIMIT_SWITCH_OPEN	2

#define LIMIT_SWITCH_CLOSE_END	4
#define LIMIT_SWITCH_OPEN_END	5



#define MAX_DEBOUNCE_EVENT			10
#define LIMIT_SWITCH(_N)			Debounce[_N - 1]

extern uint8_t Debounce[MAX_DEBOUNCE_EVENT];

uint8_t Debounce_Handle(uint8_t __DebounceEvent,uint8_t __Condition);
void Update_LS(void);

#endif /* INC_LIMITSWITCH_H_ */
