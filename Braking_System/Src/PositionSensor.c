/*
 * PositionSensor.c
 *
 *  Created on: Mar 8, 2024
 *      Author: Salah Souda @Micropolis
 */
#include "PositionSensor.h"

static uint32_t ADC_buf[4];
static float 	ADC_value[4];
static float 	lastSteering_Angle[4];
float 	 		Steering_Angle[4];
float 	 		Absolute_Angle[4];
float 	 		Steering_Base[4];
float 	 		Steering_angular_velocity[4];
uint16_t 		Angle_Sample_Time[4];
uint16_t 		angular_Sample_Time[4];

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	uint8_t STR,END;
	float Temp_ADC = 0;
//	if		(hadc == &hadc1)		{STR = 0;END = 2;}
//	else
		if (hadc == &hadc2)		{STR = 2;END = 4;}
	else 							{STR = 0;END = 0;}


	for (uint8_t NOC = STR;NOC < END;NOC++) {
	/* Conversion ADC to Float*/
		Temp_ADC = ADC_gain * ADC_buf[NOC];
	/* Apply Analog LP_Filter*/
		ADC_value[NOC] = (Temp_ADC +w_LPF * ADC_value[NOC])/(float)(w_LPF + 1);

		if 		(ADC_value[NOC] >= Full_POS)	Absolute_Angle[NOC] = Half_rang;
		else if (ADC_value[NOC] >= Zero_POS)	Absolute_Angle[NOC] = POS_gain * (ADC_value[NOC] - Zero_POS) - 90;
		else 									Absolute_Angle[NOC] = -Half_rang;
		Steering_Angle[NOC] = Absolute_Angle[NOC] - Steering_Base[NOC];
		Angle_Sample_Time[NOC] = (uint16_t) HAL_GetTick();
	}

	static uint32_t tick_start;
	uint32_t TS = uwTick - tick_start;

	if (TS > 0)
	{
		float angular_velocity;
		for (uint8_t NOC = 0;NOC < 4;NOC++)
		{
			angular_velocity = (Steering_Angle[NOC] - lastSteering_Angle[NOC])*(1000/(float) TS);
			lastSteering_Angle[NOC] =  Steering_Angle[NOC];
			Steering_angular_velocity[NOC] = (angular_velocity +dw_LPF * Steering_angular_velocity[NOC])/(float)(dw_LPF + 1);
			angular_Sample_Time[NOC] = (uint16_t) HAL_GetTick();
		}
	}

	tick_start = uwTick;
}

sys_status ADC_Start(void)
{
	uint8_t error_code = sys_ok;
//	if (HAL_OK != HAL_ADC_Start_DMA(&hadc1,&ADC_buf[0], 2)) error_code |= 0x01;
	if (HAL_OK != HAL_ADC_Start_DMA(&hadc2,&ADC_buf[2], 2)) error_code |= 0x02;
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
	for(uint8_t J = 0;J < 4;J++)
	{
		Temp_AV = (Steering_Angle[J] - lastSteering_Angle[J]) / (float) _TS * 1000.0;
		Steering_angular_velocity[J] = (Temp_AV +w_LPF * Steering_angular_velocity[J])/(float)(w_LPF + 1);
		lastSteering_Angle[J] = Steering_Angle[J];
	}

}
