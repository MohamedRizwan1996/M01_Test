/*
 * Satues.h
 *
 *  Created on: Mar 8, 2024
 *      Author: Salah Souda @Micropolis
 */

#ifndef INC_STATUS_H_
#define INC_STATUS_H_

typedef enum
{
	sys_ok = 0x00,

	ADC1_error,ADC2_error,ADC_error,
	Disabled,
	build_error,
	CAN_error,
	UART_error,
	message_error,
	CAN_UART_error,
	ID_error,
	undefined_error,
	busy,
	calibration_time_out,
	eeprom_error,
	not_calibrated,
	init_error,
}sys_status;

#endif /* INC_STATUS_H_ */
