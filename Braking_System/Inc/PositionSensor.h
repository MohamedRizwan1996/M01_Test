/*
 * PositionSensor.h
 *
 *  Created on: Mar 8, 2024
 *      Author: Salah Souda @Micropolis
 */

#ifndef INC_POSITIONSENSOR_H_
#define INC_POSITIONSENSOR_H_

#include "adc.h"
#include "dma.h"
#include "Status.h"
#include <stdint.h>

#define ADC_Vref			3.3
#define ADC_Resolution		4096
#define OpAmp_gain			0.6
#define ADC_gain			0.00134 //ADC_Vref / ADC_Resolution / OpAmp_gain
#define w_LPF				100
#define dw_LPF				100

#define LPF(_Y,_X,_taw)		_Y = (_X +_taw * _Y)/(float)(_taw + 1);


#define Zero_POS			0.5
#define Full_POS			4.5
#define Full_rang			180
#define Half_rang			90
#define POS_gain			45		// Full_rang / (Full_POS - Zero_POS)

#define micros()			uwTick *1000 + (SysTick->LOAD - SysTick->VAL) *1000 /SysTick->LOAD
#define get_micros()		u_tick = micros();

typedef uint32_t micro_type;
extern micro_type 	u_tick;

extern float 	 	Absolute_Angle[4];
extern float 	 	Steering_Base[4];
extern float 	 	Steering_Angle[4];
extern float 	 	Steering_angular_velocity[4];
extern uint16_t 	Angle_Sample_Time[4];
extern uint16_t 	angular_Sample_Time[4];

sys_status ADC_Start(void);
void get_angular_velocity(uint32_t _TS);

#endif /* INC_POSITIONSENSOR_H_ */
