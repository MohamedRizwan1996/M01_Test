/*
 * LimitSwitch.C
 *
 *  Created on: Mar 8, 2024
 *      Author: Salah Souda @Micropolis
 */
#include "LimitSwitch.h"

uint8_t Debounce[MAX_DEBOUNCE_EVENT] = {0};

uint8_t Debounce_Handle(uint8_t __DebounceEvent,uint8_t __Condition)
{
	if (__Condition != Debounce[__DebounceEvent])
	{
		 Debounce[__DebounceEvent] = __Condition;
		 return 1;
	}
		return 0;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	switch(GPIO_Pin)
	{
		case IN_LS_1_Pin: Debounce_Handle( 0,(GPIO_PIN_RESET==HAL_GPIO_ReadPin(IN_LS_1_GPIO_Port, IN_LS_1_Pin)) );break;
		case IN_LS_2_Pin: Debounce_Handle( 1,(GPIO_PIN_RESET==HAL_GPIO_ReadPin(IN_LS_2_GPIO_Port, IN_LS_2_Pin)) );break;
		case IN_LS_3_Pin: Debounce_Handle( 2,(GPIO_PIN_RESET==HAL_GPIO_ReadPin(IN_LS_3_GPIO_Port, IN_LS_3_Pin)) );break;
		case IN_LS_4_Pin: Debounce_Handle( 3,(GPIO_PIN_RESET==HAL_GPIO_ReadPin(IN_LS_4_GPIO_Port, IN_LS_4_Pin)) );break;
		case IN_LS_5_Pin: Debounce_Handle( 4,(GPIO_PIN_RESET==HAL_GPIO_ReadPin(IN_LS_5_GPIO_Port, IN_LS_5_Pin)) );break;
		case IN_LS_6_Pin: Debounce_Handle( 5,(GPIO_PIN_RESET==HAL_GPIO_ReadPin(IN_LS_6_GPIO_Port, IN_LS_6_Pin)) );break;
		case IN_LS_7_Pin: Debounce_Handle( 6,(GPIO_PIN_RESET==HAL_GPIO_ReadPin(IN_LS_7_GPIO_Port, IN_LS_7_Pin)) );break;
		case IN_LS_8_Pin: Debounce_Handle( 7,(GPIO_PIN_RESET==HAL_GPIO_ReadPin(IN_LS_8_GPIO_Port, IN_LS_8_Pin)) );break;
		case IN_LS_9_Pin: Debounce_Handle( 8,(GPIO_PIN_RESET==HAL_GPIO_ReadPin(IN_LS_9_GPIO_Port, IN_LS_9_Pin)) );break;
		case IN_LS_10_Pin:Debounce_Handle( 9,(GPIO_PIN_RESET==HAL_GPIO_ReadPin(IN_LS_10_GPIO_Port, IN_LS_10_Pin)) );break;
		default: UNUSED(GPIO_Pin);
	}

}

void Update_LS(void)
{
	Debounce[0] = !HAL_GPIO_ReadPin(IN_LS_1_GPIO_Port, IN_LS_1_Pin);
	Debounce[1] = !HAL_GPIO_ReadPin(IN_LS_2_GPIO_Port, IN_LS_2_Pin);
	Debounce[2] = !HAL_GPIO_ReadPin(IN_LS_3_GPIO_Port, IN_LS_3_Pin);
	Debounce[3] = !HAL_GPIO_ReadPin(IN_LS_4_GPIO_Port, IN_LS_4_Pin);
	Debounce[4] = !HAL_GPIO_ReadPin(IN_LS_5_GPIO_Port, IN_LS_5_Pin);
	Debounce[5] = !HAL_GPIO_ReadPin(IN_LS_6_GPIO_Port, IN_LS_6_Pin);
	Debounce[6] = !HAL_GPIO_ReadPin(IN_LS_7_GPIO_Port, IN_LS_7_Pin);
	Debounce[7] = !HAL_GPIO_ReadPin(IN_LS_8_GPIO_Port, IN_LS_8_Pin);
	Debounce[8] = !HAL_GPIO_ReadPin(IN_LS_9_GPIO_Port, IN_LS_9_Pin);
	Debounce[9] = !HAL_GPIO_ReadPin(IN_LS_10_GPIO_Port, IN_LS_10_Pin);
}
