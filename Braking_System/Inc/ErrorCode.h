/*
 * ErrorCode.h
 *
 *  Created on: Apr 30, 2024
 *      Author: Micropolis
 */

#ifndef INC_ERRORCODE_H_
#define INC_ERRORCODE_H_

/*
 * 				 ______ ______ ______ ______ ______ ______ ______ ______
 * 				|__e7__|__e6__|__e5__|__e4__|__e3__|__e2__|__e1__|__e0__|
 *
 * 	@ e0	: motor 1	over temperature.
 * 	@ e1	: motor 2	over temperature.
 * 	@ e2	: motor 3	over temperature.
 * 	@ e3	: motor 4	over temperature.
 * 	@ e4	: motor 1	over current.
 * 	@ e5	: motor 2	over current.
 * 	@ e6	: motor 3	over current.
 * 	@ e7	: motor 4	over current.
 *
 * 				 ______ ______ ______ ______ ______ ______ ______ ______
 * 				|__e15_|__e14_|__e13_|__e12_|__e11_|__e10_|__e9_|__e8_|
 *
 *  @ e8	: motor 1 error (motor error).
 * 	@ e9	: motor 2 error (motor error).
 * 	@ e10	: motor 3 error (motor error).
 * 	@ e11	: motor 4 error (motor error).
 *  @ e12	: system calibration error.
 * 	@ e13	: system ready.
 * 	@ e14	: system in calibration mode.
 * 	@ e15	: motor init error.
 *
 *  			 ______ ______ ______ ______ ______ ______ ______ ______
 * 				|__e23_|__e22_|__e21_|__e20_|__e19_|__e18_|__e17_|__e16_|
 *
 *  @ e16	: motor ID 1 error or loss connection.
 * 	@ e17	: motor ID 2 error or loss connection.
 * 	@ e18	: motor ID 3 error or loss connection.
 * 	@ e19	: motor ID 4 error or loss connection.
 *  @ e20	: reserved.
 * 	@ e21	: reserved.
 * 	@ e22	: reserved.
 * 	@ e23	: Motors CAN error.
 *
 * */

typedef struct
{
	// motor over temperature:
	uint8_t Motor_FR_OverTemperature : 1;
	uint8_t Motor_FL_OverTemperature : 1;
	uint8_t Motor_RL_OverTemperature : 1;
	uint8_t Motor_RR_OverTemperature : 1;

	// motor over current:
	uint8_t Motor_FR_OverCurrent : 1;
	uint8_t Motor_FL_OverCurrent : 1;
	uint8_t Motor_RL_OverCurrent : 1;
	uint8_t Motor_RR_OverCurrent : 1;



	// motor errors:
	uint8_t Motor_FR_Error : 1;
	uint8_t Motor_FL_Error : 1;
	uint8_t Motor_RL_Error : 1;
	uint8_t Motor_RR_Error : 1;

	// reserved bits:
	uint8_t SystemCalibrationError : 1;

	// system ready:
	uint8_t SystemReady : 1;

	// system in calibration mode:
	uint8_t SystemInCalibrationMode : 1;

	// motors startup initialization error:
	uint8_t Motors_Init_Error : 1;



	// motor ID error or loss connection:
	uint8_t Motor_FR_Loss_Connection : 1;
	uint8_t Motor_FL_Loss_Connection : 1;
	uint8_t Motor_RL_Loss_Connection : 1;
	uint8_t Motor_RR_Loss_Connection : 1;

	// reserved bits:
	uint8_t reserved2 : 3;

	// motors CAN bus error:
	uint8_t MotorsCAN_Bus_Error : 1;

}BrakeSystemErrorCode_t;

#endif /* INC_ERRORCODE_H_ */
