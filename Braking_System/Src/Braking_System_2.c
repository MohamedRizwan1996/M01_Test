/*
 * Braking_System_2.c
 *
 *  Created on: Jul 9, 2024
 *      Author: Micropolis
 */
#include "Braking_System.h"
#include "LimitSwitch.h"
#include "cmsis_os.h"

#define SET_SPEED_BRAKE_2	300.0

uint8_t startup = 0;

uint8_t Continuous_Run = 0;

uint8_t brake_check[3] = {0};

void Braking_System_Loop_2()
{
//	MotorControl[0].CAN_mode = CAN_PACKET_SET_RPM;
	//MotorControl[0].speed = (float)(-100.0);;

//	MotorControl[0].CAN_mode = CAN_PACKET_SET_RPM;
//	MotorControl[0].current_brake = 0.2;

	if(Continuous_Run == 0)
	{
		startup=0;
		for(uint8_t i=0; i<3; i++)
			brake_check[i] = 0;
		MotorControl[0].CAN_mode = CAN_PACKET_SET_CURRENT_BRAKE;
		MotorControl[0].current_brake = (float)(0.1);
		MotorControl[0].speed = (float)(0.0);
	}
	else if(Continuous_Run == 1)
	{
//		if( (!LIMIT_SWITCH(LIMIT_SWITCH_CLOSE_END)) || (!LIMIT_SWITCH(LIMIT_SWITCH_OPEN_END)) )
		if(0)
		{
			MotorControl[0].CAN_mode = CAN_PACKET_SET_CURRENT_BRAKE;
			MotorControl[0].current_brake = (float)(0.1);
			for(uint8_t i=0; i<3; i++)
				brake_check[i] = 0;
		}
//		else if(!LIMIT_SWITCH(LIMIT_SWITCH_CLOSE) && LIMIT_SWITCH(LIMIT_SWITCH_OPEN))
		else if(!LIMIT_SWITCH(LIMIT_SWITCH_CLOSE_END) && LIMIT_SWITCH(LIMIT_SWITCH_OPEN_END))
		{
			if(brake_check[0]==0)
			{
				MotorControl[0].CAN_mode = CAN_PACKET_SET_CURRENT_BRAKE;
				MotorControl[0].current_brake = (float)(0.1);
				osDelay(200);
				brake_check[0] = 1;
			}
//			if( (!LIMIT_SWITCH(LIMIT_SWITCH_CLOSE_END)) || (!LIMIT_SWITCH(LIMIT_SWITCH_OPEN_END)) )
//			{
//				MotorControl[0].CAN_mode = CAN_PACKET_SET_CURRENT_BRAKE;
//				MotorControl[0].current_brake = (float)(0.1);
//				for(uint8_t i=0; i<3; i++)
//					brake_check[i] = 0;
//			}
//			else
			{
				MotorControl[0].CAN_mode = CAN_PACKET_SET_RPM;
				MotorControl[0].speed = (float)(SET_SPEED_BRAKE_2);
				startup = 1;
				brake_check[1] = 0;
	//			osDelay(500);
			}
		}
//		else if(!LIMIT_SWITCH(LIMIT_SWITCH_OPEN) && LIMIT_SWITCH(LIMIT_SWITCH_CLOSE))
		else if(!LIMIT_SWITCH(LIMIT_SWITCH_OPEN_END) && LIMIT_SWITCH(LIMIT_SWITCH_CLOSE_END))
		{
			if(brake_check[1]==0)
			{
				MotorControl[0].CAN_mode = CAN_PACKET_SET_CURRENT_BRAKE;
				MotorControl[0].current_brake = (float)(0.1);
				osDelay(200);
				brake_check[1] = 1;
			}
//			if( (!LIMIT_SWITCH(LIMIT_SWITCH_CLOSE_END)) || (!LIMIT_SWITCH(LIMIT_SWITCH_OPEN_END)) )
//			{
//				MotorControl[0].CAN_mode = CAN_PACKET_SET_CURRENT_BRAKE;
//				MotorControl[0].current_brake = (float)(0.1);
//				for(uint8_t i=0; i<3; i++)
//					brake_check[i] = 0;
//			}
//			else
			{
				MotorControl[0].CAN_mode = CAN_PACKET_SET_RPM;
				MotorControl[0].speed = (float)(-SET_SPEED_BRAKE_2);
				startup=1;
				brake_check[0] = 0;
	//			osDelay(500);
			}
		}
//		else if(LIMIT_SWITCH(LIMIT_SWITCH_OPEN) && LIMIT_SWITCH(LIMIT_SWITCH_CLOSE) && LIMIT_SWITCH(LIMIT_SWITCH_OPEN_END) && LIMIT_SWITCH(LIMIT_SWITCH_CLOSE_END)&& startup==0)
		else if(LIMIT_SWITCH(LIMIT_SWITCH_OPEN_END) && LIMIT_SWITCH(LIMIT_SWITCH_CLOSE_END)&& startup==0)
		{
			if(brake_check[2]==0)
			{
				MotorControl[0].CAN_mode = CAN_PACKET_SET_CURRENT_BRAKE;
				MotorControl[0].current_brake = (float)(0.1);
				osDelay(100);
				brake_check[2] = 1;
			}
//			if( (!LIMIT_SWITCH(LIMIT_SWITCH_CLOSE_END)) || (!LIMIT_SWITCH(LIMIT_SWITCH_OPEN_END)) )
//			{
//				MotorControl[0].CAN_mode = CAN_PACKET_SET_CURRENT_BRAKE;
//				MotorControl[0].current_brake = (float)(0.1);
//				for(uint8_t i=0; i<3; i++)
//					brake_check[i] = 0;
//			}
			else
			{
				MotorControl[0].CAN_mode = CAN_PACKET_SET_RPM;
				MotorControl[0].speed = (float)(-SET_SPEED_BRAKE_2);
				startup=1;
				brake_check[0] = 0;
				brake_check[1] = 0;
			}
		}

	}
	else if(Continuous_Run == 2)
	{
		if(!LIMIT_SWITCH(LIMIT_SWITCH_OPEN_END))
		{
			Continuous_Run = 0;
		}
		else
		{
			MotorControl[0].CAN_mode = CAN_PACKET_SET_RPM;
			MotorControl[0].speed = (float)(SET_SPEED_BRAKE_2);
		}
	}
	else if(Continuous_Run == 3)
	{
		if(!LIMIT_SWITCH(LIMIT_SWITCH_CLOSE_END))
		{
			Continuous_Run = 0;
		}
		else
		{
			MotorControl[0].CAN_mode = CAN_PACKET_SET_RPM;
			MotorControl[0].speed = (float)(-SET_SPEED_BRAKE_2);
		}
	}
	else
	{
		Continuous_Run = 0;
	}


}
