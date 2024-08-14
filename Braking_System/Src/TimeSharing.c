/*
 * Soft_event.c
 *
 *  Created on: Apr 26, 2024
 *      Author: Micropolis
 */


#include "TimeSharing.h"

static uint8_t  	Event_H[MAX_GROUP][MAX_SOFT_EVENT] = {0};
static uint32_t		Start_Time[MAX_GROUP][MAX_SOFT_EVENT] = {0};
static uint8_t		Event_One_Time[MAX_GROUP][MAX_SOFT_EVENT] = {0};


uint8_t Time_Sharing_Handle(uint8_t __GROUP,uint8_t __Event_ID,uint8_t __Rate_delay)
{
	uint32_t 	_time = HAL_GetTick() - Start_Time[__GROUP][__Event_ID];

		if 		(_time > __Event_ID && Event_One_Time[__GROUP][__Event_ID] == 0)
		{
			Event_H[__GROUP][__Event_ID] = 1;Event_One_Time[__GROUP][__Event_ID] = 1;
			return 1;
		}

		else if (_time >= __Rate_delay)
		{
			Event_One_Time[__GROUP][__Event_ID] = 0;
			Start_Time[__GROUP][__Event_ID] = HAL_GetTick();
		}
		return 0;
}

void cycle_delay_ms(uint16_t __ms)
{
	uint32_t NOF_cycle = CORE_FREQ/1402;
	NOF_cycle *= __ms;
	for (uint32_t _Tdelay = 0; _Tdelay < NOF_cycle;_Tdelay++)
		__asm__("NOP");
}

void cycle_delay_us(uint16_t __us)
{
	uint32_t NOF_cycle = CORE_FREQ/1.402;
	NOF_cycle *= __us;
	for (uint32_t _Tdelay = 0; _Tdelay < NOF_cycle;_Tdelay++)
		__asm__("NOP");
}
