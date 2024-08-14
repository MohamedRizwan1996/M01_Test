/*
 * CubeMars.c
 *
 *  Created on: Mar 11, 2024
 *      Author: Salah Souda @Micropolis
 *      Firmware Version V1.2
 */

#include "CubeMars.h"

CubeMars_CAN_message_t 			MotorMessage;
CubeMars_data_register_t		Motor[Number_of_Motor];
CubeMars_control_register_t 	MotorControl[Number_of_Motor];
uint8_t							ID_check[Number_of_Motor];
uint8_t							Motor_CAN_check;

#if CUBEMARS_USEUART
CubeMars_UART_message_t			UARTMessage;
CubeMars_data_register_t		UART_Motor[Number_of_Motor];
uint8_t							UARTbuffer[4][UART_buffer_size];
uint8_t							STRAITUARTbuffer[4][UART_buffer_size];
uint8_t							index_input[4],index_prosses[4] = {0};
#endif

void build_CAN_eid(CubeMars_CAN_message_t* __MotorMessage ,CubeMars_control_register_t* __MotorControl)
{
	__MotorMessage->MotorHeader.IDE 	= CAN_ID_EXT;
	__MotorMessage->MotorHeader.ExtId 	= (__MotorControl->CAN_mode << 8) | __MotorControl->eid;
	__MotorMessage->MotorHeader.RTR 	= CAN_RTR_DATA;
	switch (__MotorControl->CAN_mode)
	{
		case CAN_PACKET_SET_POS_SPD : 		__MotorMessage->MotorHeader.DLC = 8;break;
		case CAN_PACKET_SET_ORIGIN_HERE: 	__MotorMessage->MotorHeader.DLC = 1;break;
		default: 							__MotorMessage->MotorHeader.DLC = 4;break;
	}
}

void build_data(uint8_t* buffer, int32_t data)
{
	uint8_t* int8_data = (uint8_t*) &data;
	buffer[0] = int8_data[3];
	buffer[1] = int8_data[2];
	buffer[2] = int8_data[1];
	buffer[3] = int8_data[0];
}


void CAN_message_init(CubeMars_CAN_message_t* __MotorMessage)
{
	__MotorMessage->MotorHeader.IDE 	= CAN_ID_EXT;
	__MotorMessage->MotorHeader.ExtId 	= 0x00;
	__MotorMessage->MotorHeader.RTR 	= CAN_RTR_DATA;
	__MotorMessage->MotorHeader.DLC 	= 4;
}

sys_status CubeMars_Init_Motor(CAN_HandleTypeDef* hcan1)
{
	CubeMars_control_register_t 	Temp_Control;
	uint8_t 						test = 0;
	uint32_t 						Time_out = 0;

	uint8_t mask = 0;
	for(uint8_t i = 0; i < Number_of_Motor; i++)
	{
		mask <<= 1;
		mask |= 1;
	}

	Temp_Control.CAN_mode = CAN_PACKET_SET_CURRENT_BRAKE;
	Temp_Control.current_brake = 0;

	do
	{
		for (uint8_t tid = 0;tid < Number_of_Motor; tid++)
		{
			Temp_Control.eid = tid+1;
			CubeMars_bulid_CAN_command(&Temp_Control);
			send_message_over_CAN(hcan1);
			HAL_Delay(100);
			if (Motor[tid].id == Temp_Control.eid)	test |= (1<<tid);
		}
		Time_out++;
		if (Time_out >= 20) return init_error;
	}while(test != mask);

	return sys_ok;
}

/* Call in void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) */
__attribute__((always_inline))
void inline CubeMars_CAN_Message_parser(CAN_HandleTypeDef *hcan,CubeMars_data_register_t* __Motor)
{
	  if ( HAL_OK != HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &(MotorMessage.RxHeader), MotorMessage.RxData) )
	  {
	    Error_Handler();
	  }
#if(CubMars_Firmware_V == 1)
	  if ( (MotorMessage.RxHeader.ExtId & 0xFF00) == 0x2900)
	  {
		   uint8_t motor_ID = (uint8_t) MotorMessage.RxHeader.ExtId & 0x00FF;
		   int16_t temp_val;
		   if (motor_ID > 0 && motor_ID <= 4)
		   {
			   temp_val = (int16_t)(MotorMessage.RxData[1] | (MotorMessage.RxData[0] << 8));
			   __Motor[motor_ID-1].position 	=   temp_val / 10.0;
			   temp_val = (int16_t)(MotorMessage.RxData[3] | (MotorMessage.RxData[2] << 8));
			   __Motor[motor_ID-1].speed 		=   temp_val / (float) rpm_to_erpm(motor_model);
			   temp_val = (int16_t)(MotorMessage.RxData[5] | (MotorMessage.RxData[4] << 8));
			   __Motor[motor_ID-1].current 	=   temp_val/ 1000.0;
			   __Motor[motor_ID-1].temp  	=   MotorMessage.RxData[6];
			   __Motor[motor_ID-1].error  	=   MotorMessage.RxData[7];
			   __Motor[motor_ID-1].id		=   motor_ID;
			   ID_check[motor_ID-1]			= 1;
		   }
	  }

#else
	  if ( (MotorMessage.RxHeader.ExtId & 0x00FF) == 0x10)
	  {
		   uint8_t motor_ID = (uint8_t) MotorMessage.RxData[0];
		   int32_t temp_val;
		   if (motor_ID > 0 && motor_ID <= 4)
		   {
			   temp_val = (int32_t)(MotorMessage.RxData[4] | (MotorMessage.RxData[3] << 8) | (MotorMessage.RxData[2] << 16)  | (MotorMessage.RxData[1] << 24));
			   __Motor[motor_ID-1].position =   temp_val / 100.0;
			   __Motor[motor_ID-1].temp  	=   MotorMessage.RxData[5];
			   __Motor[motor_ID-1].error  	=   MotorMessage.RxData[6];
			   __Motor[motor_ID-1].id		=   motor_ID;
		   }
	  }
#endif
}

uint32_t rpm_to_erpm(CubeMars_model_t __motor_model)
{
	switch(__motor_model)
	{
	case AK70_10 : return AK70_10_POLE_PAIR;
	case AK60_6 : return AK70_10_POLE_PAIR;
	case AK10_9	: return AK10_9_POLE_PAIR;
	default : return 0;
	}
	return 0;
}

sys_status CubeMars_bulid_CAN_command(CubeMars_control_register_t* __MotorControl)
{

	build_CAN_eid(&MotorMessage,__MotorControl);
	int32_t __data = 0;

	switch(__MotorControl->CAN_mode)
	{
	case CAN_PACKET_SET_DUTY 			: __data = (int32_t) (__MotorControl->duty_cycle*100000.0);break;
	case CAN_PACKET_SET_CURRENT 		: __data = (int32_t) (__MotorControl->current*10000.0);break;
	case CAN_PACKET_SET_CURRENT_BRAKE 	: __data = (int32_t) (__MotorControl->current_brake*10000.0);break;
	case CAN_PACKET_SET_RPM 			: __data = (int32_t) (__MotorControl->speed*10.0 rpm);break;
	case CAN_PACKET_SET_POS_SPD			:
	case CAN_PACKET_SET_POS 			: __data = (int32_t) (__MotorControl->position*10000.0);break;
	case CAN_PACKET_SET_ORIGIN_HERE 	: __data = 1;break;
	case CAN_Disable : __data = 0; return Disabled;
	default : __data = 0; return build_error;
	}

	build_data(&MotorMessage.MotorData[0],__data);

	if (__MotorControl->CAN_mode == CAN_PACKET_SET_POS_SPD)
	{
		__data = ((int16_t) (__MotorControl->speed rpm) << 16) | ((int16_t) (__MotorControl->Accelera rpm));
		build_data(&MotorMessage.MotorData[4],__data);
	}
	return sys_ok;
}

#if CUBEMARS_USEUART
static const uint16_t crc16_tab[] =
{
	  0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7, 0x8108, 0x9129,
	  0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef, 0x1231, 0x0210, 0x3273, 0x2252,
	  0x52b5, 0x4294, 0x72f7, 0x62d6, 0x9339, 0x8318, 0xb37b, 0xa35a, 0xd3bd, 0xc39c,
	  0xf3ff, 0xe3de, 0x2462, 0x3443, 0x0420, 0x1401, 0x64e6, 0x74c7, 0x44a4, 0x5485,
	  0xa56a, 0xb54b, 0x8528, 0x9509, 0xe5ee, 0xf5cf, 0xc5ac, 0xd58d, 0x3653, 0x2672,
	  0x1611, 0x0630, 0x76d7, 0x66f6, 0x5695, 0x46b4, 0xb75b, 0xa77a, 0x9719, 0x8738,
	  0xf7df, 0xe7fe, 0xd79d, 0xc7bc, 0x48c4, 0x58e5, 0x6886, 0x78a7, 0x0840, 0x1861,
	  0x2802, 0x3823, 0xc9cc, 0xd9ed, 0xe98e, 0xf9af, 0x8948, 0x9969, 0xa90a, 0xb92b,
	  0x5af5, 0x4ad4, 0x7ab7, 0x6a96, 0x1a71, 0x0a50, 0x3a33, 0x2a12, 0xdbfd, 0xcbdc,
	  0xfbbf, 0xeb9e, 0x9b79, 0x8b58, 0xbb3b, 0xab1a, 0x6ca6, 0x7c87, 0x4ce4, 0x5cc5,
	  0x2c22, 0x3c03, 0x0c60, 0x1c41, 0xedae, 0xfd8f, 0xcdec, 0xddcd, 0xad2a, 0xbd0b,
	  0x8d68, 0x9d49, 0x7e97, 0x6eb6, 0x5ed5, 0x4ef4, 0x3e13, 0x2e32, 0x1e51, 0x0e70,
	  0xff9f, 0xefbe, 0xdfdd, 0xcffc, 0xbf1b, 0xaf3a, 0x9f59, 0x8f78, 0x9188, 0x81a9,
	  0xb1ca, 0xa1eb, 0xd10c, 0xc12d, 0xf14e, 0xe16f, 0x1080, 0x00a1, 0x30c2, 0x20e3,
	  0x5004, 0x4025, 0x7046, 0x6067, 0x83b9, 0x9398, 0xa3fb, 0xb3da, 0xc33d, 0xd31c,
	  0xe37f, 0xf35e, 0x02b1, 0x1290, 0x22f3, 0x32d2, 0x4235, 0x5214, 0x6277, 0x7256,
	  0xb5ea, 0xa5cb, 0x95a8, 0x8589, 0xf56e, 0xe54f, 0xd52c, 0xc50d, 0x34e2, 0x24c3,
	  0x14a0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405, 0xa7db, 0xb7fa, 0x8799, 0x97b8,
	  0xe75f, 0xf77e, 0xc71d, 0xd73c, 0x26d3, 0x36f2, 0x0691, 0x16b0, 0x6657, 0x7676,
	  0x4615, 0x5634, 0xd94c, 0xc96d, 0xf90e, 0xe92f, 0x99c8, 0x89e9, 0xb98a, 0xa9ab,
	  0x5844, 0x4865, 0x7806, 0x6827, 0x18c0, 0x08e1, 0x3882, 0x28a3, 0xcb7d, 0xdb5c,
	  0xeb3f, 0xfb1e, 0x8bf9, 0x9bd8, 0xabbb, 0xbb9a, 0x4a75, 0x5a54, 0x6a37, 0x7a16,
	  0x0af1, 0x1ad0, 0x2ab3, 0x3a92, 0xfd2e, 0xed0f, 0xdd6c, 0xcd4d, 0xbdaa, 0xad8b,
	  0x9de8, 0x8dc9, 0x7c26, 0x6c07, 0x5c64, 0x4c45, 0x3ca2, 0x2c83, 0x1ce0, 0x0cc1,
	  0xef1f, 0xff3e, 0xcf5d, 0xdf7c, 0xaf9b, 0xbfba, 0x8fd9, 0x9ff8, 0x6e17, 0x7e36,
	  0x4e55, 0x5e74, 0x2e93, 0x3eb2, 0x0ed1, 0x1ef0
};

static uint16_t crc16(uint8_t *buf, uint8_t len)
{
	uint16_t cksum = 0;
	for (uint8_t i = 0; i < len; i++) {
		cksum = crc16_tab[(((cksum >> 8) ^ *buf++) & 0xFF)] ^ (cksum << 8);
	}
return cksum;
}

static uint32_t   cast_data(uint8_t* databuffer,uint8_t data_size)
{
	switch (data_size)
	{
	case 1 : return (uint8_t) (databuffer[0]);
	case 2 : return (uint16_t) (databuffer[0] << 8) | (databuffer[1]);
	case 4 : return (uint32_t) (databuffer[0] << 24) | (databuffer[1] << 16) | (databuffer[2] << 8) | (databuffer[3]);
	default : return 0;
	}
	return 0;
}

static sys_status __CubeMars_UART_Message_parser(CubeMars_data_register_t* __Motor,uint8_t __eid)
{
	uint8_t 	M_len = 0;
	uint16_t 	M_crc = 0;

		/*Check for header*/
	__eid--;
	if (0x02 != STRAITUARTbuffer[__eid][0])					return message_error;
	/*get message length*/
	M_len = STRAITUARTbuffer[__eid][1];
	/*check message length*/
	if (M_len < 6)											return message_error;
		/*Check message type*/
	if (0x32 != STRAITUARTbuffer[__eid][2])					return message_error;
		/*Check for footer*/
	if (0x03 != STRAITUARTbuffer[__eid][M_len+4])			return message_error;
		/*get message CRC*/
	M_crc = (uint16_t) (STRAITUARTbuffer[__eid][M_len+2] << 8) | (STRAITUARTbuffer[__eid][M_len+3]);
		/*check message CRC*/
	uint16_t __CRC = crc16(&STRAITUARTbuffer[__eid][2],M_len);
	if (__CRC != M_crc)								   		return message_error;

	/*get data from message*/
	uint8_t  M_mark = 7;
	uint32_t Temp_data = 0;

	uint32_t M_ID = cast_data(&STRAITUARTbuffer[__eid][3],4);

	if ( M_ID & (1<<(UART_GET_MOS_TEMPERATURE-1)) )
		{Temp_data = (uint16_t) cast_data(&STRAITUARTbuffer[__eid][M_mark],2);M_mark+=2;}

	if ( M_ID & (1<<(UART_GET_MOTOR_TEMPERATURE-1)) )
		{ __Motor[__eid].temp = (uint8_t) ((uint16_t) cast_data(&STRAITUARTbuffer[__eid][M_mark],2)/8);M_mark+=2;}

	if ( M_ID & (1<<(UART_GET_OUTPUT_CURRENT-1)) )
		{__Motor[__eid].current = (float) ((int32_t) cast_data(&STRAITUARTbuffer[__eid][M_mark],4)/1000.0);M_mark+=4;}

	if ( M_ID & (1<<(UART_GET_INPUT_CURRENT-1)) )
		{Temp_data = (int32_t) cast_data(&STRAITUARTbuffer[__eid][M_mark],4);M_mark+=4;}

	if ( M_ID & (1<<(UART_GET_Id_CURRENT-1)) )
		{Temp_data = (int32_t) cast_data(&STRAITUARTbuffer[__eid][M_mark],4);M_mark+=4; }

	if ( M_ID & (1<<(UART_GET_Iq_CURRENT-1)) )
		{Temp_data = (int32_t) cast_data(&STRAITUARTbuffer[__eid][M_mark],4);M_mark+=4; }

	if ( M_ID & (1<<(UART_GET_DUTY_CYCLE-1)) )
		{Temp_data = (int16_t) cast_data(&STRAITUARTbuffer[__eid][M_mark],2);M_mark+=2; }

	if ( M_ID & (1<<(UART_GET_SPEED-1)) )
		{__Motor[__eid].speed = (float) ((int32_t) cast_data(&STRAITUARTbuffer[__eid][M_mark],4))/210.0;M_mark+=4;}

	if ( M_ID & (1<<(UART_GET_VOLTAGE-1)) )
		{Temp_data = (int16_t) cast_data(&STRAITUARTbuffer[__eid][M_mark],2);M_mark+=2;}

	if ( M_ID & (1<<(UART_GET_ERROR_FLAGE-1)) )
		{__Motor[__eid].error = (uint8_t) cast_data(&STRAITUARTbuffer[__eid][M_mark],1);M_mark+=1;}

	if ( M_ID & (1<<(UART_GET_MOTOR_POSITION-1)) )
		{__Motor[__eid].position = (float) ((int32_t) cast_data(&STRAITUARTbuffer[__eid][M_mark],4))/1000000.0; M_mark+=4;}

	if ( M_ID & (1<<(UART_GET_MOTOR_ID-1)) )
		{__Motor[__eid].id = (uint8_t) cast_data(&STRAITUARTbuffer[__eid][M_mark],1);M_mark+=1;}

	return sys_ok;
}


sys_status CubeMars_bulid_UART_request(CubeMars_control_register_t* __MotorControl)
{
	uint32_t __data = 0;
	if (UART_REQUEST_DISABLE == __MotorControl->UART_request) return Disabled;
	if (UART_GET_Default == __MotorControl->UART_request)
		__data = 0x0038086;
	else
		__data = (uint16_t) (1 << (__MotorControl->UART_request-1));
	UARTMessage.data_len = 5; 	/*data_length + 1*/
	UARTMessage.MotorData[0] = 0x02;		//message header
	UARTMessage.MotorData[1] = UARTMessage.data_len;
	UARTMessage.MotorData[2] = (uint8_t) UART_GET_VALUES_SETUP;
	UARTMessage.MotorData[3] = (uint8_t) (__data >> 24);
	UARTMessage.MotorData[4] = (uint8_t) (__data >> 16);
	UARTMessage.MotorData[5] = (uint8_t) (__data >> 8);
	UARTMessage.MotorData[6] = (uint8_t) (__data);
	uint16_t __CRC = crc16(&UARTMessage.MotorData[2],UARTMessage.data_len);
	UARTMessage.MotorData[7] = (uint8_t) (__CRC >>8 );
	UARTMessage.MotorData[8] = (uint8_t) (__CRC);
	UARTMessage.MotorData[9] = 0x03;	//message end of frame
	return sys_ok;
}

sys_status CubeMars_bulid_UART_command(CubeMars_control_register_t* __MotorControl)
{
		uint8_t __len = 5;
		int32_t __data = 0;
		uint16_t __CRC = 0;

		switch(__MotorControl->UART_mode)
		{
		case UART_SET_DUTY 			:  __data = (int32_t) (__MotorControl->duty_cycle*100000.0);	break;
		case UART_SET_CURRENT 		:  __data = (int32_t) (__MotorControl->current*10000.0);		break;
		case UART_SET_CURRENT_BRAKE :  __data = (int32_t) (__MotorControl->current_brake*10000.0);	break;
		case UART_SET_RPM 			:  __data = (int32_t) (__MotorControl->speed*10.0 rpm);			break;
		case UART_SET_POS 			:  __data = (int32_t) (__MotorControl->position*1000000.0);		break;
		case UART_SET_HANDBRAKE 	:  __data = (int32_t) (__MotorControl->hand_brake*1000.0);		break;
		case UART_SET_POS_SPD		:  __data = (int32_t) (__MotorControl->position*1000.0);__len = 13;break;
		case UART_SET_POS_ORIGIN 	:  __data = 1;break;
		case UART_SET_POS_MULTI		:  __data = 0;break;
		case UART_SET_POS_SINGLE	:  __data = 0;break;
		case UART_Disable			:  __data = 0;return Disabled;
		default : __data = 0; return build_error;
		}

		UARTMessage.data_len = __len; 	/*data_length + 1*/
		UARTMessage.MotorData[0] = 0x02;		//message header
		UARTMessage.MotorData[1] = UARTMessage.data_len;
		UARTMessage.MotorData[2] = (uint8_t) (__MotorControl->UART_mode);
		build_data(&UARTMessage.MotorData[3],__data);

		if (__MotorControl->UART_mode == UART_SET_POS_SPD)
		{
			//__MotorControl->UART_mode = UART_Disable;
			__data = (int32_t) (__MotorControl->speed*10.0 rpm);
			build_data(&UARTMessage.MotorData[7],__data);
			__data = (int32_t) (__MotorControl->Accelera*10.0 rpm);
			build_data(&UARTMessage.MotorData[11],__data);
		}
		__CRC = crc16(&UARTMessage.MotorData[2],UARTMessage.data_len);
		UARTMessage.MotorData[UARTMessage.data_len+2] = (uint8_t) (__CRC >>8 );
		UARTMessage.MotorData[UARTMessage.data_len+3] = (uint8_t) (__CRC);
		UARTMessage.MotorData[UARTMessage.data_len+4] = 0x03;	//message end of frame
		return sys_ok;
}
#endif

sys_status 	CubeMars_bulid_command(CubeMars_control_register_t* __MotorControl)
{
	__MotorControl->model = motor_model;
	switch(__MotorControl->General_mode)
	{
		case Disable_MODE		:  __MotorControl->CAN_mode = CAN_Disable;__MotorControl->UART_mode = UART_Disable; 				break;
		case DUTY_MODE 			:  __MotorControl->CAN_mode = CAN_PACKET_SET_DUTY;__MotorControl->UART_mode = UART_SET_DUTY; 		break;
		case CURRENT_MODE 		:  __MotorControl->CAN_mode = CAN_PACKET_SET_CURRENT;__MotorControl->UART_mode = UART_SET_CURRENT;	break;
		case CURRENT_BRAKE_MODE :  __MotorControl->CAN_mode = CAN_PACKET_SET_CURRENT_BRAKE;__MotorControl->UART_mode = UART_SET_CURRENT_BRAKE;	break;
		case HANDBRAKE_MODE 	:  __MotorControl->CAN_mode = CAN_Disable;__MotorControl->UART_mode = UART_SET_HANDBRAKE;			break;
		case POS_MODE 			:  __MotorControl->CAN_mode = CAN_PACKET_SET_POS;__MotorControl->UART_mode = UART_SET_POS;			break;
		case SPD_MODE 			:  __MotorControl->CAN_mode = CAN_PACKET_SET_RPM;__MotorControl->UART_mode = UART_SET_RPM;			break;
		case POS_SPD_MODE		:  __MotorControl->CAN_mode = CAN_PACKET_SET_POS_SPD;__MotorControl->UART_mode = UART_SET_POS_SPD; 	break;
		case SET_ORIGIN_MODE	:  __MotorControl->CAN_mode = CAN_PACKET_SET_ORIGIN_HERE;__MotorControl->UART_mode = UART_SET_POS_ORIGIN; 	break;
		default : return build_error;
	}
	uint8_t __ss = 0;
	if (sys_ok != CubeMars_bulid_CAN_command(__MotorControl))	__ss++;
#if CUBEMARS_USEUART
	if (sys_ok != CubeMars_bulid_UART_command(__MotorControl))	__ss++;
#endif
	if (__ss != 0)	return build_error;
	return sys_ok;
}

sys_status CubeMars_send_command(uint8_t UART_id)
{
	sys_status _status[2] = {sys_ok};
#ifdef HAL_CAN_MODULE_ENABLED
	CAN_HandleTypeDef* hcan;
	(CUBEMARS_CAN == CAN1) ? (hcan = &hcan1) : (hcan = &hcan2);
	_status[0] = send_message_over_CAN(hcan);
#endif
#ifdef HAL_UART_MODULE_ENABLED
#if CUBEMARS_USEUART
	_status[1] = send_message_over_UART(UART_id);
#endif
#endif

	if 		(sys_ok != _status[0] && sys_ok != _status[0])
		return CAN_UART_error;
	else if (sys_ok != _status[0])
		return CAN_error;
	else if (sys_ok != _status[2])
		return UART_error;
	return undefined_error;
}

#ifdef HAL_CAN_MODULE_ENABLED
sys_status send_message_over_CAN(CAN_HandleTypeDef* hcan)
{
		  if (HAL_OK != HAL_CAN_AddTxMessage(hcan, &MotorMessage.MotorHeader, MotorMessage.MotorData, &MotorMessage.MotorMailbox))
			  	  	  	  return CAN_error;
		  return sys_ok;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	if ( hcan->Instance == CUBEMARS_CAN) 	{
		Motor_CAN_check = 0;
		CubeMars_CAN_Message_parser(hcan,Motor);
	}
}
#endif

#ifdef HAL_UART_MODULE_ENABLED
#if CUBEMARS_USEUART

static UART_HandleTypeDef* get_huart(uint8_t _eid)
{
	switch (_eid)
	{
	case 1:	 	return &CUBEMARS_1_huart;
	case 2:		return &CUBEMARS_2_huart;
	case 3:		return &CUBEMARS_3_huart;
	case 4:		return &CUBEMARS_4_huart;
	default:	return NULL;
	}return NULL;
}

sys_status send_message_over_UART(uint8_t UART_id)
{
	UART_HandleTypeDef *huart = get_huart(UART_id);
	if (HAL_OK != HAL_UART_Transmit(huart, &UARTMessage.MotorData[0], UARTMessage.data_len+5, UART_Timeout))
					return UART_error;
	return sys_ok;
}

sys_status CubeMars_UART_Messageing_Parser(uint8_t UART_id)
{
	static uint8_t _M_Start[4] = {0};
	static uint8_t _M_End[4] = {0};
	static uint8_t _M_Index[4] = {0};
	static uint8_t _M_Len[4] = {0};
	uint8_t _eid = UART_id-1;
	switch (_eid)
	{
	case 0 : 	index_input[_eid] = UART_buffer_size - CUBEMARS_1_DMA->NDTR;break;
	case 1 :	index_input[_eid] = UART_buffer_size - CUBEMARS_2_DMA->NDTR;break;
	case 2 :	index_input[_eid] = UART_buffer_size - CUBEMARS_3_DMA->NDTR;break;
	case 3 :	index_input[_eid] = UART_buffer_size - CUBEMARS_4_DMA->NDTR;break;
	default :
	}

	while  (index_input[_eid] != index_prosses[_eid])
	{
		/*copy to STRAITUARTbuffer*/
		if(UARTbuffer[_eid][index_prosses[_eid]] == 0x02 && _M_Start[_eid] == 0)
					{_M_Start[_eid] = 1;_M_End[_eid] = 0; _M_Index[_eid] = 0;}

		if (_M_Start[_eid])			/*message started*/
		{
			STRAITUARTbuffer[_eid][_M_Index[_eid]] = UARTbuffer[_eid][index_prosses[_eid]];
			/*get data length*/
			if (_M_Index[_eid] == 1) 			_M_Len[_eid] = STRAITUARTbuffer[_eid][1]+4;
			/*message ended*/
			if(_M_Index[_eid] == _M_Len[_eid] && _M_Index[_eid] > 1)
												{_M_Start[_eid] = 0;_M_End[_eid] = 1;break;}
			_M_Index[_eid]++;
			if (_M_Index[_eid] >= UART_buffer_size) _M_Index[_eid]=0;
		}

		if (++index_prosses[_eid] >= UART_buffer_size) index_prosses[_eid] =0;
	}

	if (_M_End[_eid]) 	{
		if (sys_ok != __CubeMars_UART_Message_parser(UART_Motor,(_eid+1)) )	return message_error;
	}
	else if (_M_End[_eid] == 0 && _M_Start[_eid])
		return busy;

	return sys_ok;
}
#endif
#endif
