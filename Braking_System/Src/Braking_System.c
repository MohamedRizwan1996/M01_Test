/*
 * Braking_System.c
 *
 *  Created on: Apr 4, 2024
 *      Author: Shukri Dozom @Micropolis
 */

#include "Braking_System.h"

//Recovering variables and APIs:
uint8_t recoveryFlag = 0;
uint8_t recoveringCounter = 0;
void RecoverBrakeSystem();

static uint8_t motorsErrorCounter[Number_of_Motor];
static uint8_t motorsCurrentErrorCounter[Number_of_Motor];
static uint8_t motorsConnectionLossErrorCounter[Number_of_Motor];
const static uint8_t maxErrorsCounterValue = 3; //Every 10 --> 1 sec


BrakeSystemErrorCode_t data = {0};

static sys_status cubeMardInitMotors_ErrorFlag = 0;
static uint8_t InitializationFinishedFlag = 0;
static uint8_t SystemCalibrationErrorFlag = 0;
	   uint8_t TransmissionDataToMotorsErrorFlag = 0;

static void CAN_Slave_Startup();
static void CAN_Master_Startup();
static void Calibrate_Brakes();

static void bulidCommandsToBrakeMotors();
static void ApplyEmergencyBrake();
static uint8_t CheckMotorsMotionDuringCalibration();
static float Calculate_Required_Cunnent(float _Required_Brake_Force);

static void CAN_Slave_Startup()
{

	ConfigureCANFilter(_CAN2, 0x04);

	  if (HAL_CAN_ActivateNotification(&CAN_SLAVE, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)
		  Error_Handler();

	  HAL_CAN_Start(&CAN_SLAVE);
}

static void CAN_Master_Startup()
{
  CAN_FilterTypeDef canfilterconfig;

  canfilterconfig.FilterActivation = CAN_FILTER_ENABLE;
  canfilterconfig.FilterBank = 1;  // which filter bank to use from the assigned ones
  canfilterconfig.FilterFIFOAssignment = CAN_FILTER_FIFO0;
  canfilterconfig.FilterIdHigh = 0x00<<5;
  canfilterconfig.FilterIdLow = 0;
  canfilterconfig.FilterMaskIdHigh = 0x000<<5;
  canfilterconfig.FilterMaskIdLow = 0x0000;
  canfilterconfig.FilterMode = CAN_FILTERMODE_IDMASK;
  canfilterconfig.FilterScale = CAN_FILTERSCALE_32BIT;
  canfilterconfig.SlaveStartFilterBank = 5;  // how many filters to assign to the CAN1 (master can)

  HAL_CAN_ConfigFilter(&hcan1, &canfilterconfig);

  if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
	  {Error_Handler();}

	  HAL_CAN_Start(&hcan1);
}

static uint8_t Wait_until_position_reached(float _position,float _gap,uint16_t Timeout,uint8_t motor_number_start,uint8_t motor_number_end)
{
	uint8_t 	POS_statues;
	uint16_t	POS_timeout;
	uint16_t	Tick_timeout = HAL_GetTick();
	uint8_t		POS_Check = 0;
	if (motor_number_start != 0)
		motor_number_start--;
	do
	{
		POS_statues = 0;
		for(uint8_t i = motor_number_start;i< motor_number_end; i++)
		{
			POS_Check |= (1<<i);
			if(Motor[i].position <= _position + _gap && Motor[i].position >= _position - _gap)
			{
				POS_statues |= (1<<i);
			}
		}
		if (POS_statues == POS_Check)
			break;
		POS_timeout = HAL_GetTick() - Tick_timeout;
		if (POS_timeout >= Timeout)
		{
			SystemCalibrationErrorFlag = 1;
			return 1;
		}
	}while(1);
	return 0;
}

static void Calibrate_Brakes()
{
	uint8_t Calibration_check_statues = 0;
	/* Engaging All Brakes */
	for(uint8_t i = 0;i< Number_of_Motor; i++)
	{
		MotorControl[i].CAN_mode = CAN_PACKET_SET_CURRENT;
		MotorControl[i].current = BRAKE_CALIBRATION_CURRENT_AMP;
	}
	/* Delay Time to Ensure Brakes Are Fully Engaged */
	HAL_Delay(BRAKE_CALIBRATION_DURATION_MS);
	/* motor set origin */
	for(uint8_t i = 0;i< Number_of_Motor; i++)
	{
		MotorControl[i].current = 0;
		MotorControl[i].CAN_mode = CAN_PACKET_SET_ORIGIN_HERE;

		//Assigning static values for speed and acceleration(these values will be used when releasing the break)
		MotorControl[i].speed = BRAKE_RELEASE_SPEED_RPM;
		MotorControl[i].Accelera = BRAKE_RELEASE_ACC_RPMS;
	}

	/* wait until motor set origin */
	if( Wait_until_position_reached((float) 0,(float) 0,SET_ORIGIN_DURATION_MS,1,4))
		Calibration_check_statues++;

	if (CheckMotorsMotionDuringCalibration())
		Calibration_check_statues++;
	HAL_Delay(100);

	InitializationFinishedFlag = 1;
	if (Calibration_check_statues)
		SystemCalibrationErrorFlag = 1;
	else
		SystemCalibrationErrorFlag = 0;
}

void Braking_System_Config()
{

	MotorControl[MOTOR_INDEX_FRONT_RIGHT].eid = MOTOR_CAN_ID_FRONT_RIGHT;
//	MotorControl[MOTOR_INDEX_FRONT_LEFT].eid = MOTOR_CAN_ID_FRONT_LEFT;
//	MotorControl[MOTOR_INDEX_REAR_LEFT].eid = MOTOR_CAN_ID_REAR_LEFT;
//	MotorControl[MOTOR_INDEX_REAR_RIGHT].eid = MOTOR_CAN_ID_REAR_RIGHT;

	CAN_Master_Startup();

	if(sys_ok != CubeMars_Init_Motor(&hcan1))
		cubeMardInitMotors_ErrorFlag = 1;
	TransmissionDataToMotorsErrorFlag = 0;
	CAN_Slave_Startup();
	HAL_Delay(100);
	//Sending status to MRCU once before starting the calibration:
//	SendBrakeStatusToMainBoard();

//	Calibrate_Brakes();

//	if(data.SystemCalibrationError == 1)
//		recoveryFlag = 1;
}

void Braking_System_Loop()
{
	if(!data.SystemReady)
		recoveryFlag = 1;
	else
		recoveringCounter = 0;

	bulidCommandsToBrakeMotors();
}



void CAN_Slave_Silent_Loopback_Test()
{
	static uint8_t messageNumber = 0;

	CAN_TxHeaderTypeDef txHeader;
	uint8_t txData[5] = "TEST";
	uint32_t txMailbox;

	messageNumber++;
	txData[4] = messageNumber;

	txHeader.StdId = 0x124;
	txHeader.IDE = CAN_ID_STD;
	txHeader.DLC = 5;
	txHeader.RTR = CAN_RTR_DATA;

	if (HAL_OK != HAL_CAN_AddTxMessage(&CAN_SLAVE, &txHeader, txData, &txMailbox))
		Error_Handler();

	while(HAL_CAN_IsTxMessagePending(&CAN_SLAVE, txMailbox));

}

void SendBrakeValuesToMainBoard()
{
	static uint32_t lastTick = 0;
	if(HAL_GetTick() - lastTick > 10)
	{
		lastTick = HAL_GetTick();

		const uint32_t mainBoardNumber = 0x01;
		uint32_t messageNumber = 0x33;
		uint32_t messageStaus = 0;
		uint8_t data[4] = {0};

		CAN_TxHeaderTypeDef txHeader;
		uint32_t txMailbox;

		txHeader.StdId = (mainBoardNumber << 8) | (messageNumber << 2) | messageStaus;
		txHeader.IDE = CAN_ID_STD;
		txHeader.DLC = 4;
		txHeader.RTR = CAN_RTR_DATA;

		for(uint8_t i = 0; i < Number_of_Motor; i++)
		{
			float temp = Motor[i].position;
			if(temp < BRAKE_POSITION_LEVEL_9 - 3.0f)
				data[i] = 10;
			else if(temp >= BRAKE_POSITION_LEVEL_9 - 3.0f && temp < BRAKE_POSITION_LEVEL_8 - 3.0f)
				data[i] = 9;
			else if(temp >= BRAKE_POSITION_LEVEL_8 - 3.0f && temp < BRAKE_POSITION_LEVEL_7 - 3.0f)
				data[i] = 8;
			else if(temp >= BRAKE_POSITION_LEVEL_7 - 3.0f && temp < BRAKE_POSITION_LEVEL_6 - 3.0f)
				data[i] = 7;
			else if(temp >= BRAKE_POSITION_LEVEL_6 - 3.0f && temp < BRAKE_POSITION_LEVEL_5 - 3.0f)
				data[i] = 6;
			else if(temp >= BRAKE_POSITION_LEVEL_5 - 3.0f && temp < BRAKE_POSITION_LEVEL_4 - 3.0f)
				data[i] = 5;
			else if(temp >= BRAKE_POSITION_LEVEL_4 - 3.0f && temp < BRAKE_POSITION_LEVEL_4 - 3.0f)
				data[i] = 4;
			else if(temp >= BRAKE_POSITION_LEVEL_3 - 3.0f && temp < BRAKE_POSITION_LEVEL_2 - 3.0f)
				data[i] = 3;
			else if(temp >= BRAKE_POSITION_LEVEL_2 - 3.0f && temp < BRAKE_POSITION_LEVEL_1 - 3.0f)
				data[i] = 2;
			else if(temp >= BRAKE_POSITION_LEVEL_1 - 3.0f && temp < BRAKE_POSITION_LEVEL_0 - 3.0f)
				data[i] = 1;
			else if(temp >= BRAKE_POSITION_LEVEL_0 - 3.0f)
				data[i] = 0;
		}

		HAL_CAN_AddTxMessage(&CAN_SLAVE, &txHeader, data, &txMailbox);
		HAL_Delay(1);
	}
}

void SendMotorTemperatureToMainBoard()
{
	static uint32_t lastTick = 0;
	if(HAL_GetTick() - lastTick > 10)
	{
		lastTick = HAL_GetTick();

		const uint32_t mainBoardNumber = 0x01;
		uint32_t messageNumber = 0x36;
		uint32_t messageStaus = 0;
		uint8_t data[8] = {0};

		CAN_TxHeaderTypeDef txHeader;
		uint32_t txMailbox;

		txHeader.StdId = (mainBoardNumber << 8) | (messageNumber << 2) | messageStaus;
		txHeader.IDE = CAN_ID_STD;
		txHeader.DLC = 8;
		txHeader.RTR = CAN_RTR_DATA;

		for(uint8_t i = 0; i < Number_of_Motor; i++)
			CONVERTERu16tou8(&data[i * 2], float_encode(Motor[i].temp));

		HAL_CAN_AddTxMessage(&CAN_SLAVE, &txHeader, data, &txMailbox);
		HAL_Delay(1);
	}
}

static float Calculate_Required_Cunnent(float _Required_Brake_Force)
{
		/* torque(Nm) = F(KN).L(mm) / 2.pi.e
		 * L = 2(mm)
		 * e = 0.8 */
		float est_current = 0;
		float est_torque = 0.4*_Required_Brake_Force;
		if (est_torque < 5.5)
			est_current = 0.108*est_torque+0.045;
		else
			est_current = 0.185*(est_torque-5.5)+0.65;
		return est_current;
}

void SendMotorCurrentToMainBoard()
{
	static uint32_t lastTick = 0;
	if(HAL_GetTick() - lastTick > 10)
	{
		lastTick = HAL_GetTick();

		const uint32_t mainBoardNumber = 0x01;
		uint32_t messageNumber = 0x35;
		uint32_t messageStaus = 0;
		uint8_t data[8] = {0};

		CAN_TxHeaderTypeDef txHeader;
		uint32_t txMailbox;

		txHeader.StdId = (mainBoardNumber << 8) | (messageNumber << 2) | messageStaus;
		txHeader.IDE = CAN_ID_STD;
		txHeader.DLC = 8;
		txHeader.RTR = CAN_RTR_DATA;

		for(uint8_t i = 0; i < Number_of_Motor; i++)
			CONVERTERu16tou8(&data[i * 2], float_encode(Motor[i].current));

		HAL_CAN_AddTxMessage(&CAN_SLAVE, &txHeader, data, &txMailbox);
		HAL_Delay(1);
	}
}
uint8_t Motor_stall[4];
uint8_t stall_counter[4];
void SendBrakeStatusToMainBoard()
{
	static uint32_t lastTick = 0;
	if(HAL_GetTick() - lastTick > 100)
	{
		lastTick = HAL_GetTick();
		memset(&data, 0, 3);

		const uint32_t mainBoardNumber = 0x01;
		uint32_t messageNumber = 0x34;
		uint32_t messageStaus = 0;
		CAN_TxHeaderTypeDef txHeader;
		uint32_t txMailbox;

		txHeader.StdId = (mainBoardNumber << 8) | (messageNumber << 2) | messageStaus;
		txHeader.IDE = CAN_ID_STD;
		txHeader.DLC = 3;
		txHeader.RTR = CAN_RTR_DATA;

		// over temperature errors and motor errors:

#if Number_of_Motor >= 1
		if(Motor[MOTOR_INDEX_FRONT_RIGHT].error == motor_over_temperature
				|| Motor[MOTOR_INDEX_FRONT_RIGHT].error == MOSFET_over_temperature)
			data.Motor_FR_OverTemperature = 1;
		else if(Motor[MOTOR_INDEX_FRONT_RIGHT].error != no_fault)
			data.Motor_FR_Error = 1;
#endif

#if Number_of_Motor >= 2
		if(Motor[MOTOR_INDEX_FRONT_LEFT].error == motor_over_temperature
				|| Motor[MOTOR_INDEX_FRONT_LEFT].error == MOSFET_over_temperature)
			data.Motor_FL_OverTemperature = 1;
		else if(Motor[MOTOR_INDEX_FRONT_LEFT].error != no_fault)
			data.Motor_FL_Error = 1;
#endif

#if Number_of_Motor >= 3
		if(Motor[MOTOR_INDEX_REAR_LEFT].error == motor_over_temperature
				|| Motor[MOTOR_INDEX_REAR_LEFT].error == MOSFET_over_temperature)
			data.Motor_RL_OverTemperature = 1;
		else if(Motor[MOTOR_INDEX_REAR_LEFT].error != no_fault)
			data.Motor_RL_Error = 1;
#endif

#if Number_of_Motor >= 4
		if(Motor[MOTOR_INDEX_REAR_RIGHT].error == motor_over_temperature
				|| Motor[MOTOR_INDEX_REAR_RIGHT].error == MOSFET_over_temperature)
			data.Motor_RR_OverTemperature = 1;
		else if(Motor[MOTOR_INDEX_REAR_RIGHT].error != no_fault)
			data.Motor_RR_Error = 1;
#endif

		// over current errors:

#if Number_of_Motor >= 1
		if(fabs((double)Motor[MOTOR_INDEX_FRONT_RIGHT].current >= BRAKE_MAX_CURRENT_ABS_AMP))
			data.Motor_FR_OverCurrent = 1;
#endif

#if Number_of_Motor >= 2
		if(fabs((double)Motor[MOTOR_INDEX_FRONT_LEFT].current >= BRAKE_MAX_CURRENT_ABS_AMP))
			data.Motor_FL_OverCurrent = 1;
#endif

#if Number_of_Motor >= 3
		if(fabs((double)Motor[MOTOR_INDEX_REAR_LEFT].current >= BRAKE_MAX_CURRENT_ABS_AMP))
			data.Motor_RL_OverCurrent = 1;
#endif

#if Number_of_Motor >= 4
		if(fabs((double)Motor[MOTOR_INDEX_REAR_RIGHT].current >= BRAKE_MAX_CURRENT_ABS_AMP))
			data.Motor_RR_OverCurrent = 1;
#endif


		// system calibration error:
		if(SystemCalibrationErrorFlag == 1)
			data.SystemCalibrationError = 1;

		// system in calibration mode:
		if(InitializationFinishedFlag == 0)
			data.SystemInCalibrationMode = 1;

		// motor initialization error:
		if(cubeMardInitMotors_ErrorFlag == 1)
			data.Motors_Init_Error = 1;


		// motor IDs error or motor lost connection:
#if Number_of_Motor >= 1
		if(ID_check[MOTOR_INDEX_FRONT_RIGHT] != 1)
			data.Motor_FR_Loss_Connection = 1;
#endif

#if Number_of_Motor >= 2
		if(ID_check[MOTOR_INDEX_FRONT_LEFT] != 1)
			data.Motor_FL_Loss_Connection = 1;
#endif

#if Number_of_Motor >= 3
		if(ID_check[MOTOR_INDEX_REAR_LEFT] != 1)
			data.Motor_RL_Loss_Connection = 1;
#endif

#if Number_of_Motor >= 4
		if(ID_check[MOTOR_INDEX_REAR_RIGHT] != 1)
			data.Motor_RR_Loss_Connection = 1;
#endif

		// Resetting Id_check[]:
		for(uint8_t i = 0; i < Number_of_Motor; i++)
			ID_check[i] = 0;

		//CAN error:
		Motor_CAN_check++;
		if(Motor_CAN_check >= 100)
		{
			Motor_CAN_check = 100; //Set it to a maximum value to prevent overflow
			data.MotorsCAN_Bus_Error = 1;
		}


#if Number_of_Motor >= 1
		if(data.Motor_FR_Error == 1)
		{
			if(++motorsErrorCounter[MOTOR_INDEX_FRONT_RIGHT] >= maxErrorsCounterValue)
			{
				motorsErrorCounter[MOTOR_INDEX_FRONT_RIGHT] = maxErrorsCounterValue;
				data.SystemReady = 1;
			}
		}
		else
		{
			motorsErrorCounter[MOTOR_INDEX_FRONT_RIGHT] = 0;
		}
#endif

#if Number_of_Motor >= 2
		if(data.Motor_FL_Error == 1)
		{
			if(++motorsErrorCounter[MOTOR_INDEX_FRONT_LEFT] >= maxErrorsCounterValue)
			{
				motorsErrorCounter[MOTOR_INDEX_FRONT_LEFT] = maxErrorsCounterValue;
				data.SystemReady = 1;
			}
		}
		else
		{
			motorsErrorCounter[MOTOR_INDEX_FRONT_LEFT] = 0;
		}
#endif

#if Number_of_Motor >= 3
		if(data.Motor_RL_Error == 1)
		{
			if(++motorsErrorCounter[MOTOR_INDEX_REAR_LEFT] >= maxErrorsCounterValue)
			{
				motorsErrorCounter[MOTOR_INDEX_REAR_LEFT] = maxErrorsCounterValue;
				data.SystemReady = 1;
			}
		}
		else
		{
			motorsErrorCounter[MOTOR_INDEX_REAR_LEFT] = 0;
		}
#endif

#if Number_of_Motor >= 4
		if(data.Motor_RR_Error == 1)
		{
			if(++motorsErrorCounter[MOTOR_INDEX_REAR_RIGHT] >= maxErrorsCounterValue)
			{
				motorsErrorCounter[MOTOR_INDEX_REAR_RIGHT] = maxErrorsCounterValue;
				data.SystemReady = 1;
			}
		}
		else
		{
			motorsErrorCounter[MOTOR_INDEX_REAR_RIGHT] = 0;
		}
#endif

#if Number_of_Motor >= 1
		if(data.Motor_FR_OverCurrent == 1)
		{
			if(++motorsCurrentErrorCounter[MOTOR_INDEX_FRONT_RIGHT] >= maxErrorsCounterValue)
			{
				motorsCurrentErrorCounter[MOTOR_INDEX_FRONT_RIGHT] = maxErrorsCounterValue;
				data.SystemReady = 1;
			}
		}
		else
		{
			motorsCurrentErrorCounter[MOTOR_INDEX_FRONT_RIGHT] = 0;
		}
#endif

#if Number_of_Motor >= 2
		if(data.Motor_FL_OverCurrent == 1)
		{
			if(++motorsCurrentErrorCounter[MOTOR_INDEX_FRONT_LEFT] >= maxErrorsCounterValue)
			{
				motorsCurrentErrorCounter[MOTOR_INDEX_FRONT_LEFT] = maxErrorsCounterValue;
				data.SystemReady = 1;
			}
		}
		else
		{
			motorsCurrentErrorCounter[MOTOR_INDEX_FRONT_LEFT] = 0;
		}
#endif

#if Number_of_Motor >= 3
		if(data.Motor_RL_OverCurrent == 1)
		{
			if(++motorsCurrentErrorCounter[MOTOR_INDEX_REAR_LEFT] >= maxErrorsCounterValue)
			{
				motorsCurrentErrorCounter[MOTOR_INDEX_REAR_LEFT] = maxErrorsCounterValue;
				data.SystemReady = 1;
			}
		}
		else
		{
			motorsCurrentErrorCounter[MOTOR_INDEX_REAR_LEFT] = 0;
		}
#endif

#if Number_of_Motor >= 4
		if(data.Motor_RR_OverCurrent == 1)
		{
			if(++motorsCurrentErrorCounter[MOTOR_INDEX_REAR_RIGHT] >= maxErrorsCounterValue)
			{
				motorsCurrentErrorCounter[MOTOR_INDEX_REAR_RIGHT] = maxErrorsCounterValue;
				data.SystemReady = 1;
			}
		}
		else
		{
			motorsCurrentErrorCounter[MOTOR_INDEX_REAR_RIGHT] = 0;
		}
#endif

#if Number_of_Motor >= 1
		if(data.Motor_FR_Loss_Connection == 1)
		{
			if(++motorsConnectionLossErrorCounter[MOTOR_INDEX_FRONT_RIGHT] >= maxErrorsCounterValue)
			{
				motorsConnectionLossErrorCounter[MOTOR_INDEX_FRONT_RIGHT] = maxErrorsCounterValue;
				data.SystemReady = 1;
			}
		}
		else
		{
			motorsConnectionLossErrorCounter[MOTOR_INDEX_FRONT_RIGHT] = 0;
		}
#endif

#if Number_of_Motor >= 2
		if(data.Motor_FL_Loss_Connection == 1)
		{
			if(++motorsConnectionLossErrorCounter[MOTOR_INDEX_FRONT_LEFT] >= maxErrorsCounterValue)
			{
				motorsConnectionLossErrorCounter[MOTOR_INDEX_FRONT_LEFT] = maxErrorsCounterValue;
				data.SystemReady = 1;
			}
		}
		else
		{
			motorsConnectionLossErrorCounter[MOTOR_INDEX_FRONT_LEFT] = 0;
		}
#endif

#if Number_of_Motor >= 3
		if(data.Motor_RL_Loss_Connection == 1)
		{
			if(++motorsConnectionLossErrorCounter[MOTOR_INDEX_REAR_LEFT] >= maxErrorsCounterValue)
			{
				motorsConnectionLossErrorCounter[MOTOR_INDEX_REAR_LEFT] = maxErrorsCounterValue;
				data.SystemReady = 1;
			}
		}
		else
		{
			motorsConnectionLossErrorCounter[MOTOR_INDEX_REAR_LEFT] = 0;
		}
#endif

#if Number_of_Motor >= 4
		if(data.Motor_RR_Loss_Connection == 1)
		{
			if(++motorsConnectionLossErrorCounter[MOTOR_INDEX_REAR_RIGHT] >= maxErrorsCounterValue)
			{
				motorsConnectionLossErrorCounter[MOTOR_INDEX_REAR_RIGHT] = maxErrorsCounterValue;
				data.SystemReady = 1;
			}
		}
		else
		{
			motorsConnectionLossErrorCounter[MOTOR_INDEX_REAR_RIGHT] = 0;
		}
#endif

		/* check if motor in releasing mode && there are no speed */

#if Number_of_Motor >= 1
		if (MotorControl[MOTOR_INDEX_FRONT_RIGHT].CAN_mode ==  CAN_PACKET_SET_POS_SPD && MotorControl[MOTOR_INDEX_FRONT_RIGHT].position == BRAKE_RELEASE_DEGREE &&
			fabs(Motor[MOTOR_INDEX_FRONT_RIGHT].current) > 0.1 && Motor[MOTOR_INDEX_FRONT_RIGHT].speed == 0)
			stall_counter[MOTOR_INDEX_FRONT_RIGHT]++;
		else
			stall_counter[MOTOR_INDEX_FRONT_RIGHT] = 0;
		if (Motor_stall[MOTOR_INDEX_FRONT_RIGHT] == 1) data.Motor_FR_Error = 1;
		if (stall_counter[MOTOR_INDEX_FRONT_RIGHT] >= 5)
			Motor_stall[MOTOR_INDEX_FRONT_RIGHT] = 1;
#endif
#if Number_of_Motor >= 2
		if (MotorControl[MOTOR_INDEX_FRONT_LEFT].CAN_mode ==  CAN_PACKET_SET_POS_SPD && MotorControl[MOTOR_INDEX_FRONT_LEFT].position == BRAKE_RELEASE_DEGREE &&
			fabs(Motor[MOTOR_INDEX_FRONT_LEFT].current) > 0.1 && Motor[MOTOR_INDEX_FRONT_LEFT].speed == 0)
			stall_counter[MOTOR_INDEX_FRONT_LEFT]++;
		else
			stall_counter[MOTOR_INDEX_FRONT_LEFT] = 0;
		if (Motor_stall[MOTOR_INDEX_FRONT_LEFT] == 1)  data.Motor_FL_Error = 1;
		if (stall_counter[MOTOR_INDEX_FRONT_LEFT] >= 5)
			Motor_stall[MOTOR_INDEX_FRONT_LEFT] = 1;
#endif
#if Number_of_Motor >= 3
		if (MotorControl[MOTOR_INDEX_REAR_LEFT].CAN_mode ==  CAN_PACKET_SET_POS_SPD && MotorControl[MOTOR_INDEX_REAR_LEFT].position == BRAKE_RELEASE_DEGREE &&
			fabs(Motor[MOTOR_INDEX_REAR_LEFT].current) > 0.1 && Motor[MOTOR_INDEX_REAR_LEFT].speed == 0)
			stall_counter[MOTOR_INDEX_REAR_LEFT]++;
		else
			stall_counter[MOTOR_INDEX_REAR_LEFT] = 0;
		if (Motor_stall[MOTOR_INDEX_REAR_LEFT] == 1) data.Motor_RL_Error = 1;
		if (stall_counter[MOTOR_INDEX_REAR_LEFT] >= 5)
			Motor_stall[MOTOR_INDEX_REAR_LEFT] = 1;
#endif
#if Number_of_Motor == 4
		if (MotorControl[MOTOR_INDEX_REAR_RIGHT].CAN_mode ==  CAN_PACKET_SET_POS_SPD && MotorControl[MOTOR_INDEX_REAR_RIGHT].position == BRAKE_RELEASE_DEGREE &&
			fabs(Motor[MOTOR_INDEX_REAR_RIGHT].current) > 0.1 && Motor[MOTOR_INDEX_REAR_RIGHT].speed == 0)
			stall_counter[MOTOR_INDEX_REAR_RIGHT]++;
		else
			stall_counter[MOTOR_INDEX_REAR_RIGHT] = 0;
		if (Motor_stall[MOTOR_INDEX_REAR_RIGHT] == 1) data.Motor_RR_Error = 1;
		if (stall_counter[MOTOR_INDEX_REAR_RIGHT] >= 5)
			Motor_stall[MOTOR_INDEX_REAR_RIGHT] = 1;
#endif
		// system ready:
//		data.SystemReady |= data.Motor_FR_Error;
//		data.SystemReady |= data.Motor_FR_Loss_Connection;
		data.SystemReady |= data.Motor_FR_OverCurrent;
		data.SystemReady |= data.Motor_FR_OverTemperature;

//		data.SystemReady |= data.Motor_FL_Error;
//		data.SystemReady |= data.Motor_FL_Loss_Connection;
		data.SystemReady |= data.Motor_FL_OverCurrent;
		data.SystemReady |= data.Motor_FL_OverTemperature;

//		data.SystemReady |= data.Motor_RL_Error;
//		data.SystemReady |= data.Motor_RL_Loss_Connection;
		data.SystemReady |= data.Motor_RL_OverCurrent;
		data.SystemReady |= data.Motor_RL_OverTemperature;

//		data.SystemReady |= data.Motor_RR_Error;
//		data.SystemReady |= data.Motor_RR_Loss_Connection;
		data.SystemReady |= data.Motor_RR_OverCurrent;
		data.SystemReady |= data.Motor_RR_OverTemperature;

//		data.SystemReady |= data.MotorsCAN_Bus_Error;
		data.SystemReady |= data.SystemCalibrationError;
		data.SystemReady |= data.SystemInCalibrationMode;
		data.SystemReady |= data.Motors_Init_Error;

		//Checking if there is a problem sending data to CubeMars motors:
		data.SystemReady |= TransmissionDataToMotorsErrorFlag;

		data.SystemReady = ~data.SystemReady;

		//LED on: system ready, LED off: system is not ready
		HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, !data.SystemReady); // LED uses inverted logic (0: on)

		HAL_CAN_AddTxMessage(&CAN_SLAVE, &txHeader, (uint8_t*)&data, &txMailbox);
		HAL_Delay(1);
	}
}


static void bulidCommandsToBrakeMotors()
{

	const uint32_t brakeCommandsRateMs = 10;
	static uint32_t lastTick = 0;
	static float L_POS[Number_of_Motor] = {0};
	static float L_Current[Number_of_Motor] = {0};
	static uint32_t counter[Number_of_Motor] = {0};

	if(HAL_GetTick() - lastTick >= brakeCommandsRateMs)
	{
		for(uint8_t i = 0;i<Number_of_Motor;i++)
		{

		  float Current = MotorControl[i].current;
		  float POS = MotorControl[i].position;
	switch(MotorControl[i].CAN_mode)
		{
	case CAN_PACKET_SET_CURRENT:
			  if (L_Current[i] != Current)
			  {
				  counter[i]=0;
				  L_POS[i] = POS;
				  L_Current[i] = Current;
			  }
			  else
			  {
				  counter[i]++;

				  if(counter[i] > 10000)
					  counter[i] = 10000;
			  }

			  if (fabs(Motor[i].current) > 0.1 && fabs(Motor[i].speed) < 1 && counter[i] < 120)
			  {
				  MotorControl[i].current = 0;
			  }
			  else if (counter[i] >= 125)
			  {
				  MotorControl[i].current = 0;
			  }
			  break;
	case CAN_PACKET_SET_POS_SPD:
			  if (L_POS[i] != POS)
			  {
				  counter[i]=0;
				  L_POS[i] = POS;
				  L_Current[i] = Current;
			  }
			  else
			  {
				  counter[i]++;
				  if(counter[i] > 10000)
					  counter[i] = 10000;
			  }
			  if (Motor_stall[i] == 1)
			  {
				  MotorControl[i].current = 0;
				  MotorControl[i].CAN_mode = CAN_PACKET_SET_CURRENT;
			  }
			  break;

		  default:
			  break;
		 }
		}
	}
}



// This API tries to send data to CubeMars motors using CAN for a fixed
// number of trials, and returns the status of the transmission.
// Also it encapsulate the necessary delay needed between two
// adjacent messages in CAN
sys_status CAN_TransmitToBrakeMotors()
{
	uint8_t counter = 0;
	const uint8_t trials = 3;
	sys_status status = sys_ok;
	while(counter < trials)
	{
		status = send_message_over_CAN(&CAN_MOTORS);
		HAL_Delay(1);
		if(status != sys_ok)
			counter++;

		else return sys_ok;
	}
	return calibration_time_out;
}

static void ApplyEmergencyBrake()
{
	for(uint8_t i = 0;i < Number_of_Motor; i++)
	{
		MotorControl[i].current = BRAKE_CURRENT_AMP;
		MotorControl[i].CAN_mode = CAN_PACKET_SET_CURRENT;
	}
}

static uint8_t CheckMotorsMotionDuringCalibration()
{
	uint8_t Calibration_check_statues = 0;
	uint8_t numberOfFrontBrakeMotors;
	uint8_t numberOfRearBrakeMotors;

#if Number_of_Motor == 1
	numberOfFrontBrakeMotors = 1;
	numberOfRearBrakeMotors = 0;
#elif Number_of_Motor == 2
	numberOfFrontBrakeMotors = 2;
	numberOfRearBrakeMotors = 0;
#elif Number_of_Motor == 3
	numberOfFrontBrakeMotors = 2;
	numberOfRearBrakeMotors = 1;
#else
	numberOfFrontBrakeMotors = 2;
	numberOfRearBrakeMotors = 2;
#endif

	/* Testing the full-release state: (for Front motors) */
	for(uint8_t i = 0; i < numberOfFrontBrakeMotors; i++)
	{
		MotorControl[i].position = BRAKE_RELEASE_DEGREE;
		MotorControl[i].CAN_mode = CAN_PACKET_SET_POS_SPD;
	}
	/* wait until Release */
	if (Wait_until_position_reached((float) BRAKE_RELEASE_DEGREE,(float) 15,RELEASE_DURATION_MS,1,numberOfFrontBrakeMotors))
		Calibration_check_statues++;

	/*go back to zero position*/
	for(uint8_t i = 0;i< numberOfFrontBrakeMotors; i++)
	{
		MotorControl[i].CAN_mode = CAN_PACKET_SET_POS_SPD;
		MotorControl[i].position = 0;
	}
	/* back to zero time delay */
	if (Wait_until_position_reached((float) 0,(float) 15,BRAKE_CALIBRATION_DURATION_MS,1,numberOfFrontBrakeMotors))
		Calibration_check_statues++;
	/* switch to current mode */
	for(uint8_t i = 0;i< numberOfFrontBrakeMotors; i++)
	{
		MotorControl[i].current = 0;
		MotorControl[i].CAN_mode = CAN_PACKET_SET_CURRENT;
	}
	/* Testing the full-release state: (for Rear motors) */
	for(uint8_t i = 2; i < numberOfRearBrakeMotors + 2; i++)
	{
		MotorControl[i].position = BRAKE_RELEASE_DEGREE;
		MotorControl[i].CAN_mode = CAN_PACKET_SET_POS_SPD;
	}
	/* Release time Delay */
	if  (numberOfRearBrakeMotors !=0)
		if (Wait_until_position_reached((float) BRAKE_RELEASE_DEGREE,(float) 15,RELEASE_DURATION_MS,3,numberOfRearBrakeMotors+2))
			Calibration_check_statues++;

	/*go back to zero position*/
	for(uint8_t i = 2;i< numberOfRearBrakeMotors + 2; i++)
	{
		MotorControl[i].CAN_mode = CAN_PACKET_SET_POS_SPD;
		MotorControl[i].position = 0;
	}
	/* back to zero delay */
	if  (numberOfRearBrakeMotors !=0)
		if (Wait_until_position_reached((float) 0,(float) 15,BRAKE_CALIBRATION_DURATION_MS,3,numberOfRearBrakeMotors+2))
			Calibration_check_statues++;
	/* switch to current mode */
	for(uint8_t i = 2;i< numberOfRearBrakeMotors + 2; i++)
	{
		MotorControl[i].current = 0;
		MotorControl[i].CAN_mode = CAN_PACKET_SET_CURRENT;
	}
	return Calibration_check_statues;
}

void RecoverBrakeSystem()
{
	const uint8_t recoverMaxTrials = 3;

	ApplyEmergencyBrake();

	recoveryFlag = 0;

	if(recoveringCounter < recoverMaxTrials)
		recoveringCounter++;
	else
	{
		HAL_GPIO_WritePin(Led_GPIO_Port, Led_Pin, 1); // off
		HAL_GPIO_WritePin(Led2_GPIO_Port, Led2_Pin, 1); // on
		HAL_GPIO_WritePin(Led3_GPIO_Port, Led3_Pin, 1); // on
		for(uint8_t i = 0;i< Number_of_Motor; i++)
		{
			MotorControl[i].current = 0;
			MotorControl[i].CAN_mode = CAN_PACKET_SET_CURRENT;
		}
		HAL_Delay(500);
		//Error_Handler();
		while(1);
	}

	if(sys_ok != CubeMars_Init_Motor(&hcan1))
		cubeMardInitMotors_ErrorFlag = 1;
	HAL_Delay(1000);
	Calibrate_Brakes();
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
		ParserCANMessage(_CAN2);
	if(InitializationFinishedFlag)
	{

		uint8_t receivedData[4];
		receivedData[MOTOR_INDEX_FRONT_RIGHT] = (uint8_t)(Vehicle.BrakeCommand.BrakeTorqueFrontRight + 0.1f);
		receivedData[MOTOR_INDEX_FRONT_LEFT] = (uint8_t)(Vehicle.BrakeCommand.BrakeTorqueFrontLeft + 0.1f);
		receivedData[MOTOR_INDEX_REAR_LEFT] = (uint8_t)(Vehicle.BrakeCommand.BrakeTorqueRearLeft + 0.1f);
		receivedData[MOTOR_INDEX_REAR_RIGHT] = (uint8_t)(Vehicle.BrakeCommand.BrakeTorqueRearRight + 0.1f);

		static uint8_t last_receivedData[4];

		for(uint8_t i = 0; i < Number_of_Motor; i++)
		{
		/*over writ only if the data has been updated*/
			if (last_receivedData[i] != receivedData[i])
			{
				last_receivedData[i] = receivedData[i];
				if(receivedData[i] <= 0)
				{
					MotorControl[i].position = BRAKE_POSITION_LEVEL_0;
					MotorControl[i].current = 0;
					MotorControl[i].CAN_mode = CAN_PACKET_SET_POS_SPD;
				}
				else if(receivedData[i] == 1)
				{
					MotorControl[i].position = BRAKE_POSITION_LEVEL_1;
					MotorControl[i].current = 0;
					MotorControl[i].CAN_mode = CAN_PACKET_SET_POS_SPD;
				}
				else if(receivedData[i] == 2)
				{
					MotorControl[i].position = BRAKE_POSITION_LEVEL_2;
					MotorControl[i].current = 0;
					MotorControl[i].CAN_mode = CAN_PACKET_SET_POS_SPD;
				}
				else if(receivedData[i] == 3)
				{
					MotorControl[i].position = BRAKE_POSITION_LEVEL_3;
					MotorControl[i].current = 0;
					MotorControl[i].CAN_mode = CAN_PACKET_SET_POS_SPD;
				}
				else if(receivedData[i] == 4)
				{
					MotorControl[i].position = BRAKE_POSITION_LEVEL_4;
					MotorControl[i].current = 0;
					MotorControl[i].CAN_mode = CAN_PACKET_SET_POS_SPD;
				}
				else if(receivedData[i] == 5)
				{
					MotorControl[i].position = BRAKE_POSITION_LEVEL_5;
					MotorControl[i].current = 0;
					MotorControl[i].CAN_mode = CAN_PACKET_SET_POS_SPD;
				}
				else if(receivedData[i] == 6)
				{
					MotorControl[i].position = BRAKE_POSITION_LEVEL_6;
					MotorControl[i].current = 0;
					MotorControl[i].CAN_mode = CAN_PACKET_SET_POS_SPD;
				}
				else if(receivedData[i] == 7)
				{
					MotorControl[i].position = BRAKE_POSITION_LEVEL_7;
					MotorControl[i].current = 0;
					MotorControl[i].CAN_mode = CAN_PACKET_SET_POS_SPD;
				}
				else if(receivedData[i] == 8)
				{
					MotorControl[i].position = BRAKE_POSITION_LEVEL_8;
					MotorControl[i].current = 0;
					MotorControl[i].CAN_mode = CAN_PACKET_SET_POS_SPD;
				}
				else if(receivedData[i] == 9)
				{
					MotorControl[i].position = BRAKE_POSITION_LEVEL_9;
					MotorControl[i].current = 0;
					MotorControl[i].CAN_mode = CAN_PACKET_SET_POS_SPD;
				}
				else if(receivedData[i] >= 10)
				{
					MotorControl[i].position = BRAKE_POSITION_LEVEL_10;
					MotorControl[i].current = BRAKE_CURRENT_AMP;
					MotorControl[i].CAN_mode = CAN_PACKET_SET_CURRENT;
				}
			}
		}
	}
}




#ifdef SELF_TEST_BRAKE_SYSTEM

uint8_t newBrakeValueFromDebug[Number_of_Motor];
uint8_t newBrakeValueFromDebugFlag;
float temperatureOfMotorBuffer[Number_of_Motor][NUMBER_OF_TEMPERATURE_SAMPLES];

void TestNewBrakeValuesFromDebug_Loop()
{

{ //This scope is for implementing changing the applied braking value

  const uint32_t selfTestChangeBrakeValueRateMs = 2000;
  static uint32_t lastTick = 0;
  if(HAL_GetTick() - lastTick >= selfTestChangeBrakeValueRateMs)
  {
	  lastTick = HAL_GetTick();
	  for(uint8_t i = 0;i<Number_of_Motor; i++)
	  {
		  if(newBrakeValueFromDebug[i] == 0)
			  newBrakeValueFromDebug[i] = 10;
		  else
			  newBrakeValueFromDebug[i] = 0;
	  }
	  newBrakeValueFromDebugFlag = 1;

  }
}

{
	//This scope is for implementing monitoring the temperature of the motors
	 const uint32_t monitoringTemperatureRateMs = 5000;
	 static uint32_t lastTick = 0;
	 static uint8_t lastTemperatureIndex[Number_of_Motor];

	 if(HAL_GetTick() - lastTick >= monitoringTemperatureRateMs)
	 {
		 lastTick = HAL_GetTick();
		 for(uint8_t i = 0;i<Number_of_Motor; i++)
		 {
			 temperatureOfMotorBuffer[i][lastTemperatureIndex[i]] = Motor[i].temp;
			 lastTemperatureIndex[i]++;
			 if(lastTemperatureIndex[i] >= NUMBER_OF_TEMPERATURE_SAMPLES)
				 lastTemperatureIndex[i] = 0;
		 }

	 }
}


  if(newBrakeValueFromDebugFlag)
  {
	  newBrakeValueFromDebugFlag = 0;
		if(InitializationFinishedFlag)
		{

			for(uint8_t i = 0; i < Number_of_Motor; i++)
			{
				if(newBrakeValueFromDebug[i] <= 0)
				{
					MotorControl[i].position = BRAKE_POSITION_LEVEL_0;
					MotorControl[i].current = 0;
					MotorControl[i].CAN_mode = CAN_PACKET_SET_POS_SPD;
				}
				else if(newBrakeValueFromDebug[i] == 1)
				{
					MotorControl[i].position = BRAKE_POSITION_LEVEL_1;
					MotorControl[i].current = 0;
					MotorControl[i].CAN_mode = CAN_PACKET_SET_POS_SPD;
				}
				else if(newBrakeValueFromDebug[i] == 2)
				{
					MotorControl[i].position = BRAKE_POSITION_LEVEL_2;
					MotorControl[i].current = 0;
					MotorControl[i].CAN_mode = CAN_PACKET_SET_POS_SPD;
				}
				else if(newBrakeValueFromDebug[i] == 3)
				{
					MotorControl[i].position = BRAKE_POSITION_LEVEL_3;
					MotorControl[i].current = 0;
					MotorControl[i].CAN_mode = CAN_PACKET_SET_POS_SPD;
				}
				else if(newBrakeValueFromDebug[i] == 4)
				{
					MotorControl[i].position = BRAKE_POSITION_LEVEL_4;
					MotorControl[i].current = 0;
					MotorControl[i].CAN_mode = CAN_PACKET_SET_POS_SPD;
				}
				else if(newBrakeValueFromDebug[i] == 5)
				{
					MotorControl[i].position = BRAKE_POSITION_LEVEL_5;
					MotorControl[i].current = 0;
					MotorControl[i].CAN_mode = CAN_PACKET_SET_POS_SPD;
				}
				else if(newBrakeValueFromDebug[i] == 6)
				{
					MotorControl[i].position = BRAKE_POSITION_LEVEL_6;
					MotorControl[i].current = 0;
					MotorControl[i].CAN_mode = CAN_PACKET_SET_POS_SPD;
				}
				else if(newBrakeValueFromDebug[i] == 7)
				{
					MotorControl[i].position = BRAKE_POSITION_LEVEL_7;
					MotorControl[i].current = 0;
					MotorControl[i].CAN_mode = CAN_PACKET_SET_POS_SPD;
				}
				else if(newBrakeValueFromDebug[i] == 8)
				{
					MotorControl[i].position = BRAKE_POSITION_LEVEL_8;
					MotorControl[i].current = 0;
					MotorControl[i].CAN_mode = CAN_PACKET_SET_POS_SPD;
				}
				else if(newBrakeValueFromDebug[i] == 9)
				{
					MotorControl[i].position = BRAKE_POSITION_LEVEL_9;
					MotorControl[i].current = 0;
					MotorControl[i].CAN_mode = CAN_PACKET_SET_POS_SPD;
				}
				else if(newBrakeValueFromDebug[i] >= 10)
				{
					MotorControl[i].position = BRAKE_POSITION_LEVEL_10;
					MotorControl[i].current = BRAKE_CURRENT_AMP;
					MotorControl[i].CAN_mode = CAN_PACKET_SET_CURRENT;
				}
			}
		}
  }
}

#endif
