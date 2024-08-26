/*
 * PositionSensor.c
 *
 *  Created on: Mar 8, 2024
 *      Author: Salah Souda @Micropolis
 */
#include "PositionSensor.h"

static uint32_t ADC_buf[2];
static float 	ADC_value;
static float 	LastLifting_Angle;
float 	 		Lifting_Angle;
float 	 		Absolute_Angle;
float 	 		Lifting_Base;
float 	 		Lifting_Angular_Velocity;
uint16_t 		Angle_Sample_Time;
uint16_t 		Angular_Sample_Time;

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	float Temp_ADC = 0;
	if (hadc == &hadc2)
	{
		/* Conversion ADC to Float*/
		Temp_ADC = ADC_gain * ADC_buf[0];
		/* Apply Analog LP_Filter*/
		ADC_value = (Temp_ADC + w_LPF * ADC_value) / (float) (w_LPF + 1);

		if (ADC_value >= Full_POS)			Absolute_Angle = Half_rang;
		else if (ADC_value >= Zero_POS)		Absolute_Angle = POS_gain * (ADC_value - Zero_POS) - 90;
		else								Absolute_Angle = -Half_rang;

		Lifting_Angle = Absolute_Angle - Lifting_Base;

		Angle_Sample_Time = (uint16_t) HAL_GetTick();

		static uint32_t Tick_Start;
		uint32_t TS = uwTick - Tick_Start;

		if (TS > 0)
		{
			float Angular_Velocity;
			Angular_Velocity = (Lifting_Angle - LastLifting_Angle) * (1000 / (float) TS);
			LastLifting_Angle = Lifting_Angle;
			Lifting_Angular_Velocity = (Angular_Velocity + dw_LPF * Lifting_Angular_Velocity) / (float) (dw_LPF + 1);

			Angular_Sample_Time = (uint16_t) HAL_GetTick();
		}

		Tick_Start = uwTick;
	}
}

sys_status ADC_Start(void)
{
	uint8_t error_code = sys_ok;
	if (HAL_OK != HAL_ADC_Start_DMA(&hadc2,&ADC_buf[0], 2)) error_code |= 0x02;
	switch (error_code)
	{
		case 0: return sys_ok;
		case 1: return ADC1_error;
		case 2: return ADC2_error;
		case 3: return ADC_error;
	}
	return undefined_error;
}

void get_angular_velocity(uint32_t _TS)
{
	if (_TS == 0)	return;
	float Temp_AV;

	Temp_AV = (Lifting_Angle - LastLifting_Angle) / (float) _TS * 1000.0;
	Lifting_Angular_Velocity = (Temp_AV +w_LPF * Lifting_Angular_Velocity)/(float)(w_LPF + 1);
	LastLifting_Angle = Lifting_Angle;

}
