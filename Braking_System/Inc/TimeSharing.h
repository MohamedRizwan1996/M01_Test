/*
 * Soft_event.h
 *
 *  Created on: Apr 26, 2024
 *      Author: Micropolis
 */

#ifndef INC_TSH_OS_H_
#define INC_TSH_OS_H_
#include "stm32f4xx_hal.h"
#include "stdint.h"


#define MAX_SOFT_EVENT			5		/*maximum number of software event per group*/
#define MAX_GROUP				5

#define CORE_FREQ		18000000
/*
 * This function, is responsible for managing time-sharing events within the system.
 * It tracks the elapsed time since the start of each event and determines whether
 * the event should be triggered based on predefined criteria.
 * @ __GROUP		 Group identifier for grouping related tasks.
 * @ __Event_ID		 Identifier for the specific event or task within the group.
 * @ __Rate_delay 	 Time delay in milliseconds (ms) for event repetition, ranging from MAX_SOFT_EVENT to 255.
 * @ retval 		 Returns 1 if the event is triggered, otherwise returns 0.
 * */
uint8_t Time_Sharing_Handle(uint8_t __GROUP,uint8_t __Event_ID,uint8_t __Rate_delay);
void cycle_delay_ms(uint16_t __ms);
void cycle_delay_us(uint16_t __us);

#endif /* INC_TSH_OS_H_ */
