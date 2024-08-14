#include  "CANBus.h"

#include  "string.h"
#include  "stdint.h"
#include  "stdbool.h"
#include  "PrjTypedef.h"
#include  <math.h>

uint8_t CANAknowledgeFlag = 0;
uint8_t MessageNumberSent = 0x00;

Vehicle_Status Vehicle;

// Define your array of handles
void* get_can_handle(CANNumber number);

#if (CANTYPE == CAN)
#include "can.h"
CAN_HandleTypeDef* can_handles[] = {&hcan1, &hcan2};
#else
#include  "fdcan.h"
FDCAN_HandleTypeDef* fdcan_handles[] = {&hfdcan1, &hfdcan2};
#endif

STATUS_TypeDef SendCANMessage(CANNumber Can, CANMessage *Message) {

#if	(CANTYPE == CAN)
	CAN_HandleTypeDef *can=get_can_handle(Can);

	uint32_t TxMailbox;

		CAN_TxHeaderTypeDef TxHeader;
		TxHeader.StdId = Message->CANMessgeID;
		TxHeader.DLC = Message->CANMessageLength;
		TxHeader.ExtId =0;
		TxHeader.IDE = CAN_ID_STD;
		TxHeader.RTR = CAN_RTR_DATA;

		if (HAL_CAN_AddTxMessage(can, &TxHeader, Message->CANMessageData, &TxMailbox) == HAL_OK)
			return STATUS_OK;
		else
		   return STATUS_ERROR;

#elif	(CANTYPE == FDCAN)
		FDCAN_TxHeaderTypeDef TxHeader;

		TxHeader.IdType = FDCAN_STANDARD_ID;
		TxHeader.Identifier = Message->CANMessgeID;
		TxHeader.TxFrameType = FDCAN_DATA_FRAME;
		TxHeader.DataLength = Message->CANMessageLength;
		TxHeader.BitRateSwitch=FDCAN_BRS_OFF;
		TxHeader.FDFormat=FDCAN_CLASSIC_CAN;
		FDCAN_HandleTypeDef *fdcan=get_can_handle(Can);

	if (HAL_FDCAN_AddMessageToTxFifoQ(fdcan, &TxHeader, Message->CANMessageData) == HAL_OK)
		return STATUS_OK;
	else
		return STATUS_ERROR;
#endif
}


STATUS_TypeDef SendCANMessageWithRetries(CANNumber Can, CANMessage *Message, uint32_t timeout, uint8_t trials) {
	STATUS_TypeDef status;
	uint32_t tickstart = HAL_GetTick();
	uint8_t attempts = 0;
	uint8_t tempBoardNumber = 0x00;
	uint8_t tempStatus = 0x00;
	DecodeCANID(Message->CANMessgeID, &tempBoardNumber, &MessageNumberSent, &tempStatus);
	do {
		status = SendCANMessage(Can, Message);

		if (status == STATUS_OK) {
			return HAL_OK;
		} else {
			HAL_Delay(10);
			attempts++;
		}
	} while (((HAL_GetTick() - tickstart < timeout) && (attempts < trials)) || (CANAknowledgeFlag == 0));

	return STATUS_TIMEOUT;
}


STATUS_TypeDef SendAcknowledgmentMessage(CANNumber Can, uint8_t MessageNumber) {

	CANMessageStatus MessageState = Response;
	uint8_t Message[2] = { BOARDNUMBER, MessageNumber };

#if	(CANTYPE == CAN)
	CAN_HandleTypeDef *can=get_can_handle(Can);
	uint32_t TxMailbox;
	CAN_TxHeaderTypeDef TxHeader;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.StdId = EncodeCANID(GetSenderCANID(MessageNumber), MRCU_ACKNOWLEDGE, MessageState);
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.DLC = 0x02;
	if (HAL_CAN_AddTxMessage(can, &TxHeader, Message, &TxMailbox) == HAL_OK)
		return STATUS_OK;
	else
		return STATUS_ERROR;


#elif	(CANTYPE == FDCAN)
	FDCAN_TxHeaderTypeDef TxHeader;
	FDCAN_HandleTypeDef *fdcan = get_can_handle(Can);
	TxHeader.IdType = FDCAN_STANDARD_ID;
	TxHeader.Identifier = EncodeCANID(GetSenderCANID(MessageNumber), MRCU_ACKNOWLEDGE, MessageState);
	TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	TxHeader.DataLength = 0x02;
	if (HAL_FDCAN_AddMessageToTxFifoQ(fdcan, &TxHeader, Message) == HAL_OK)
		return STATUS_OK;
	else
		return STATUS_ERROR;
#endif
}

void ConfigureCANFilter(CANNumber Can, uint32_t IDBoard ) {
#if (CANTYPE == FDCAN)
	FDCAN_HandleTypeDef *fdcan = get_can_handle(Can);
	FDCAN_FilterTypeDef canFilterConfig;
	IDBoard = IDBoard << 8;

	canFilterConfig.IdType=FDCAN_STANDARD_ID;
	canFilterConfig.FilterConfig=CAN_FIFO_USE;
	canFilterConfig.FilterType=FDCAN_FILTER_MASK;
	canFilterConfig.FilterID1= 0x00 << 5;
	canFilterConfig.FilterID2=0x00 << 5;
	canFilterConfig.FilterIndex=18;

	if (HAL_FDCAN_ConfigFilter(fdcan, &canFilterConfig) != HAL_OK) {
		HAL_Delay(10);
	}
#elif	(CANTYPE == CAN)
	CAN_HandleTypeDef *can=get_can_handle(Can);

	CAN_FilterTypeDef canFilterConfig;
	IDBoard = IDBoard << 8;
	canFilterConfig.FilterActivation = CAN_FILTER_ENABLE;
	canFilterConfig.FilterBank = 18;
	canFilterConfig.FilterFIFOAssignment = CAN_FIFO_USE;
	canFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	canFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;

	canFilterConfig.FilterIdHigh = IDBoard << 5;
	canFilterConfig.FilterIdLow = 0x0000;
	canFilterConfig.FilterMaskIdHigh = 0x700 << 5;
	canFilterConfig.FilterMaskIdLow = 0x0000;
	canFilterConfig.SlaveStartFilterBank = FILTERSLAVENUMBER;

	if (HAL_CAN_ConfigFilter(can, &canFilterConfig) != HAL_OK) {
		// Handle error
	}

#endif
}

uint32_t EncodeCANID(Board boardNumber, uint8_t messageNumber, CANMessageStatus status) {

	boardNumber &= 0x07;
	messageNumber &= 0x3F;
	status &= 0x03;

	return (boardNumber << 8) | (messageNumber << 2) | status;
}
void DecodeCANID(uint16_t canID, Board *boardNumber, uint8_t *messageNumber, CANMessageStatus *status) {

	*boardNumber = (canID >> 8) & 0x07;
	*messageNumber = (canID >> 2) & 0x3F;
	*status = canID & 0x03;
}

/*
 compress float number (32Bit) to uint16_t (16Bit)
 @float _number must be in range (-8190 ~ +8190 )
 */
uint16_t float_encode(float _number) {
#define MAX_int13           0x1FFF

	uint16_t ret_16;
	uint8_t sign = 0;

	if (_number > 0)
		sign = 0;
	else if (_number < 0) {
		sign = 1;
		_number *= -1;
	}

	uint32_t exp = 1;
	uint8_t Dec_exp = 0;

	if (_number >= MAX_int13)
		return 0XFFFF;       //number can't be encode
	if (_number * 1000 < MAX_int13) {
		exp = 1000;
		Dec_exp = 3;
	} else if (_number * 100 < MAX_int13) {
		exp = 100;
		Dec_exp = 2;
	} else if (_number * 10 < MAX_int13) {
		exp = 10;
		Dec_exp = 1;
	} else {
		exp = 1;
		Dec_exp = 0;
	}

	ret_16 = ((uint16_t) (_number * exp) << 2);
	ret_16 |= Dec_exp;
	ret_16 |= (sign << 15);

	return ret_16;
}

/*
 uncompress uint16_t (16Bit) to float number (32Bit)
 @uint16_t _number must be float_encode form
 */
float float_decode(uint16_t _number) {

	float ret_float;
	int8_t sign = 0;
	if (_number == 0XFFFF)
		return (float) NAN;
	(_number & 0x8000) ? (sign = -1) : (sign = 1);      //get sign

	uint32_t exp = 1;
	uint8_t Dec_exp = (uint8_t) (_number & 0x0003);
	switch (Dec_exp) {
	case 0:
		exp = 1;
		break;
	case 1:
		exp = 10;
		break;
	case 2:
		exp = 100;
		break;
	case 3:
		exp = 1000;
		break;
	}

	ret_float = (float) ((_number & 0x7FFC) >> 2) * (float) sign / (float) exp;

	return ret_float;
}

uint8_t GetSenderCANID(uint8_t MessageNumber) {

	if (MessageNumber >= MRCU_ACKNOWLEDGE && MessageNumber <= MRCU_RESERVED_MESSAGE_8)
		return 1;
	else if (MessageNumber >= ORIN_SET_DRIVING_COMMAND && MessageNumber <= ORIN_RESERVED_MESSAGE_5)
		return 2;
	else if (MessageNumber >= STEERING_ANGLE_FRONT && MessageNumber <= STEERING_RESERVED_MESSAGE_7)
		return 3;
	else if (MessageNumber >= BRAKE_APPLIED_TORQUE_FRONT && MessageNumber <= BRAKE_RESERVED_MESSAGE_6)
		return 4;
	else if (MessageNumber >= PDU_BATTERY_STATE_OF_CHARGE && MessageNumber <= PDU_RESERVED_MESSAGE_1)
		return 5;
	else
		return 0;
}



STATUS_TypeDef ParserCANMessage(CANNumber Can) {
	uint8_t BoardNumber;
	uint8_t Status;
	uint8_t MessageNumber;

	uint8_t RxData[8];


#if	(CANTYPE == CAN)
	CAN_HandleTypeDef *can=get_can_handle(Can);

	CAN_RxHeaderTypeDef RxHeader;
	if (HAL_CAN_GetRxMessage(can, CAN_FIFO_USE, &RxHeader, RxData) == HAL_OK) {
		DecodeCANID(RxHeader.StdId, &BoardNumber, &MessageNumber, &Status);

		if (Status == 0x1 || Status == 0x02)
			SendAcknowledgmentMessage(Can, MessageNumber);

#elif	(CANTYPE == FDCAN)
		FDCAN_RxHeaderTypeDef RxHeader;
		FDCAN_HandleTypeDef *fdcan=get_can_handle(Can);
	if (HAL_FDCAN_GetRxMessage(fdcan, CAN_FIFO_USE, &RxHeader, RxData) == HAL_OK) {

		DecodeCANID(RxHeader.Identifier, &BoardNumber, &MessageNumber, &Status);

		if (Status == 0x1 || Status == 0x02)
			SendAcknowledgmentMessage(Can, MessageNumber);
#endif
		switch (MessageNumber) {
		case MRCU_ACKNOWLEDGE:
			if (BOARDNUMBER == RxData[0] && MessageNumberSent == RxData[1])
				CANAknowledgeFlag = 1;
			break;
		case MRCU_WHEELS_VELOCITY:
			Vehicle.WheelFrontRight.Driving.DrivingVelocity = float_decode(CAST(RxData[0]));
			Vehicle.WheelFrontLeft.Driving.DrivingVelocity =  float_decode(CAST(RxData[2]));
			Vehicle.WheelRearLeft.Driving.DrivingVelocity =  float_decode(CAST(RxData[4]));
			Vehicle.WheelRearRight.Driving.DrivingVelocity =   float_decode(CAST(RxData[6]));
			break;
		case MRCU_WHEELS_TORQUE:
			Vehicle.WheelFrontRight.Driving.DrivingTorque = float_decode(CAST(RxData[0]));
			Vehicle.WheelFrontLeft.Driving.DrivingTorque =  float_decode(CAST(RxData[2]));
			Vehicle.WheelRearLeft.Driving.DrivingTorque =  float_decode(CAST(RxData[4]));
			Vehicle.WheelRearRight.Driving.DrivingTorque =   float_decode(CAST(RxData[6]));
			break;
		case MRCU_IMU_ACCERLERATION:
			Vehicle.IMUSensor.Acceleration.Acceleration_X = float_decode(CAST(RxData[0]));
			Vehicle.IMUSensor.Acceleration.Acceleration_Y = float_decode(CAST(RxData[2]));
			Vehicle.IMUSensor.Acceleration.Acceleration_Z = float_decode(CAST(RxData[4]));
			break;
		case MRCU_IMU_VELOCITY:
			Vehicle.IMUSensor.Velocity.Velocity_X = float_decode(CAST(RxData[0]));
			Vehicle.IMUSensor.Velocity.Velocity_Y = float_decode(CAST(RxData[2]));
			Vehicle.IMUSensor.Velocity.Velocity_Z = float_decode(CAST(RxData[4]));
			break;
		case MRCU_IMU_ANGULAR_VELOCITY:
			Vehicle.IMUSensor.AngularVelocity.AngularVelocity_Roll = float_decode(CAST(RxData[0]));
			Vehicle.IMUSensor.AngularVelocity.AngularVelocity_Pitch = float_decode(CAST(RxData[2]));
			Vehicle.IMUSensor.AngularVelocity.AngularVelocity_Yaw = float_decode(CAST(RxData[4]));
			break;
		case MRCU_IMU_ANGLE:
			Vehicle.IMUSensor.Angle.Angle_Roll = float_decode(CAST(RxData[0]));
			Vehicle.IMUSensor.Angle.Angle_Pitch = float_decode(CAST(RxData[2]));
			Vehicle.IMUSensor.Angle.Angle_Yaw = float_decode(CAST(RxData[4]));
			break;

		case MRCU_STEERING_COMMAND:
			Vehicle.SteeringCommand.Angle = float_decode(CAST(RxData[0]));
			Vehicle.SteeringCommand.AngularVelocity = float_decode(CAST(RxData[2]));
			Vehicle.SteeringCommand.SteeringMode = RxData[4];
			break;
		case MRCU_BRAKING_COMMAND:
			Vehicle.BrakeCommand.BrakeTorqueFrontRight = float_decode(CAST(RxData[0]));
			Vehicle.BrakeCommand.BrakeTorqueFrontLeft = float_decode(CAST(RxData[2]));
			Vehicle.BrakeCommand.BrakeTorqueRearLeft = float_decode(CAST(RxData[4]));
			Vehicle.BrakeCommand.BrakeTorqueRearRight = float_decode(CAST(RxData[6]));
			break;

		case MRCU_STEERING_ANGLE:
			Vehicle.WheelFrontRight.Steering.SteeringAngle = float_decode(CAST(RxData[0]));
			Vehicle.WheelFrontLeft.Steering.SteeringAngle = float_decode(CAST(RxData[2]));
			Vehicle.WheelRearLeft.Steering.SteeringAngle = float_decode(CAST(RxData[4]));
			Vehicle.WheelRearRight.Steering.SteeringAngle = float_decode(CAST(RxData[6]));
			break;
		case MRCU_STEERING_ANGULAR_VELOCITY:
			Vehicle.WheelFrontRight.Steering.SteeringAngularVelocity = float_decode(CAST(RxData[0]));
			Vehicle.WheelFrontLeft.Steering.SteeringAngularVelocity = float_decode(CAST(RxData[2]));
			Vehicle.WheelRearLeft.Steering.SteeringAngularVelocity = float_decode(CAST(RxData[4]));
			Vehicle.WheelRearRight.Steering.SteeringAngularVelocity = float_decode(CAST(RxData[6]));
			break;
		case MRCU_BRAKE_VALUE:
			Vehicle.WheelFrontRight.Braking.BrakeValue = RxData[0];
			Vehicle.WheelFrontLeft.Braking.BrakeValue = RxData[1];
			Vehicle.WheelRearLeft.Braking.BrakeValue = RxData[2];
			Vehicle.WheelRearRight.Braking.BrakeValue = RxData[3];
			break;
		case MRCU_ULTRASONIC:
			Vehicle.UltraSonicSensors.FrontRight=RxData[0];
			Vehicle.UltraSonicSensors.FrontMiddleRight=RxData[1];
			Vehicle.UltraSonicSensors.FrontMiddleLeft=RxData[2];
			Vehicle.UltraSonicSensors.FrontLeft=RxData[3];
			Vehicle.UltraSonicSensors.RearRight=RxData[4];
			Vehicle.UltraSonicSensors.RearMiddleRight=RxData[5];
			Vehicle.UltraSonicSensors.RearMiddleLeft=RxData[6];
			Vehicle.UltraSonicSensors.RearLeft=RxData[7];
			break;
		case MRCU_STATE_OF_CHARGE:
			Vehicle.PDU.BatteryStateOfCharge.BatteryVoltage = float_decode(CAST(RxData[0]));
			Vehicle.PDU.BatteryStateOfCharge.BatteryCurrent = float_decode(CAST(RxData[2]));
			Vehicle.PDU.BatteryStateOfCharge.BatteryChargePercentage = RxData[4];
			Vehicle.PDU.BatteryStateOfCharge.BatteryStatus = RxData[5];
			break;
		case MRCU_POWER_STATUS_CAHNNEL_REQUEST:
			break;
		case MRCU_TEMP_HUMIDITY_FAN_REQUEST:
			break;
		case MRCU_RELAYS_H_BRIGDE_DIGITAL_OUTPUTS_COMMAND:
			break;
		case MRCU_DIGITAL_ANALOG_INPUT_REQUEST:
			break;

		case ORIN_SET_DRIVING_COMMAND:
			Vehicle.DesiredCarParameters.DesiredVelocity = float_decode(CAST(RxData[0]));
			Vehicle.DesiredCarParameters.DesiredAcceleration = float_decode(CAST(RxData[2]));
			Vehicle.DesiredCarParameters.DesiredSteeringAngle = float_decode(CAST(RxData[4]));
			Vehicle.DesiredCarParameters.DesiredSteeringAngularVelocity = float_decode(CAST(RxData[6]));
			break;
		case ORIN_SET_STEERING_MODE:
			switch(RxData[0]) {
			    case 0x00:
			        Vehicle.DesiredCarParameters.DesiredSteeringMode = Front_ackerman;
			        break;
			    case 0x01:
			        Vehicle.DesiredCarParameters.DesiredSteeringMode = Double_ackerman;
			        break;
			    case 0x02:
			        Vehicle.DesiredCarParameters.DesiredSteeringMode = Carb_steer;
			        break;
			    case 0x03:
			        Vehicle.DesiredCarParameters.DesiredSteeringMode = Front_parallel;
			        break;
			    case 0x04:
			        Vehicle.DesiredCarParameters.DesiredSteeringMode = Double_parallel;
			        break;
			    case 0x05:
			        Vehicle.DesiredCarParameters.DesiredSteeringMode = Goto_home;
			        break;
			    case 0x06:
			        Vehicle.DesiredCarParameters.DesiredSteeringMode = Calibration;
			        break;
			    default:
			        // Handle invalid value
			        break;
			}			break;

		case ORIN_SET_LIGHTS:
			break;

		case ORIN_SET_EMERGENCY_BRAKE:
			Vehicle.DesiredCarParameters.DesiredBrake = float_decode(CAST(RxData[0]));
			break;
		case ORIN_SET_AUTONOMOUS_MODE:
			Vehicle.AutonomousMode = (RxData[0] == 0x00) ? false : true;
			break;

		case STEERING_ANGLE_FRONT:
			Vehicle.WheelFrontRight.Steering.SteeringAngleSampleTime = CAST(RxData[0]);
			Vehicle.WheelFrontRight.Steering.SteeringAngle = float_decode(CAST(RxData[2]));
			Vehicle.WheelFrontLeft.Steering.SteeringAngleSampleTime = CAST(RxData[4]);
			Vehicle.WheelFrontLeft.Steering.SteeringAngle = float_decode(CAST(RxData[6]));
			break;

		case STEERING_ANGLE_REAR:
			Vehicle.WheelRearLeft.Steering.SteeringAngleSampleTime = CAST(RxData[0]);
			Vehicle.WheelRearLeft.Steering.SteeringAngle = float_decode(CAST(RxData[2]));
			Vehicle.WheelRearRight.Steering.SteeringAngleSampleTime = CAST(RxData[4]);
			Vehicle.WheelRearRight.Steering.SteeringAngle = float_decode(CAST(RxData[6]));
			break;

		case STEERING_ANGULAR_VELOCITY_FRONT:
			Vehicle.WheelFrontRight.Steering.SteeringAngularVelocitySampleTime = CAST(RxData[0]);
			Vehicle.WheelFrontRight.Steering.SteeringAngularVelocity = float_decode(CAST(RxData[2]));
			Vehicle.WheelFrontLeft.Steering.SteeringAngularVelocitySampleTime = CAST(RxData[4]);
			Vehicle.WheelFrontLeft.Steering.SteeringAngularVelocity = float_decode(CAST(RxData[6]));
			break;

		case STEERING_ANGULAR_VELOCITY_REAR:
			Vehicle.WheelRearLeft.Steering.SteeringAngularVelocitySampleTime = CAST(RxData[0]);
			Vehicle.WheelRearLeft.Steering.SteeringAngularVelocity = float_decode(CAST(RxData[2]));
			Vehicle.WheelRearRight.Steering.SteeringAngularVelocitySampleTime = CAST(RxData[4]);
			Vehicle.WheelRearRight.Steering.SteeringAngularVelocity = float_decode(CAST(RxData[6]));
			break;
		case STEERING_STATUS_ERROR:
			Vehicle.WheelFrontRight.Steering.WheelSteeringState.MotorOverTemperature=(RxData[0] & 0x01) ? true: false;
			Vehicle.WheelFrontLeft.Steering.WheelSteeringState.MotorOverTemperature=(RxData[0] & 0x02) ? true: false;
			Vehicle.WheelRearLeft.Steering.WheelSteeringState.MotorOverTemperature=(RxData[0] & 0x04) ? true: false;
			Vehicle.WheelRearRight.Steering.WheelSteeringState.MotorOverTemperature=(RxData[0] & 0x08) ? true: false;

			Vehicle.WheelFrontRight.Steering.WheelSteeringState.MotorOverCurrent=(RxData[0] & 0x10) ? true: false;
			Vehicle.WheelFrontLeft.Steering.WheelSteeringState.MotorOverCurrent=(RxData[0] & 0x20) ? true: false;
			Vehicle.WheelRearLeft.Steering.WheelSteeringState.MotorOverCurrent=(RxData[0] & 0x40) ? true: false;
			Vehicle.WheelRearRight.Steering.WheelSteeringState.MotorOverCurrent=(RxData[0] & 0x80) ? true: false;


			Vehicle.WheelFrontRight.Steering.WheelSteeringState.PositionSensorDemaged=(RxData[1] & 0x01) ? true: false;
			Vehicle.WheelFrontLeft.Steering.WheelSteeringState.PositionSensorDemaged=(RxData[1] & 0x02) ? true: false;
			Vehicle.WheelRearLeft.Steering.WheelSteeringState.PositionSensorDemaged=(RxData[1] & 0x04) ? true: false;
			Vehicle.WheelRearRight.Steering.WheelSteeringState.PositionSensorDemaged=(RxData[1] & 0x08) ? true: false;


			Vehicle.WheelFrontRight.Steering.WheelSteeringState.PositionSensorResponse=(RxData[1] & 0x10) ? true: false;
			Vehicle.WheelFrontLeft.Steering.WheelSteeringState.PositionSensorResponse=(RxData[1] & 0x20) ? true: false;
			Vehicle.WheelRearLeft.Steering.WheelSteeringState.PositionSensorResponse=(RxData[1] & 0x40) ? true: false;
			Vehicle.WheelRearRight.Steering.WheelSteeringState.PositionSensorResponse=(RxData[1] & 0x80) ? true: false;


			Vehicle.WheelFrontRight.Steering.WheelSteeringState.MotorError=	(RxData[2] & 0x01) ? true: false;
			Vehicle.WheelFrontLeft.Steering.WheelSteeringState.MotorError=(RxData[2] & 0x02) ? true: false;
			Vehicle.WheelRearLeft.Steering.WheelSteeringState.MotorError=(RxData[2] & 0x04) ? true: false;
			Vehicle.WheelRearRight.Steering.WheelSteeringState.MotorError=(RxData[2] & 0x08) ? true: false;

			Vehicle.SteeringSystemState.MotorCANBusConnection=(RxData[2] & 0x10) ? true: false;
			Vehicle.SteeringSystemState.BoardCANBusConnection=(RxData[2] & 0x20) ? true: false;
			Vehicle.SteeringSystemState.SteeringSystemError=(RxData[2] & 0x40) ? true: false;
			Vehicle.SteeringSystemState.MotorsInitError=(RxData[2] & 0x80) ? true: false;


			Vehicle.WheelFrontRight.Steering.WheelSteeringState.MotorConnectionLoss=	(RxData[3] & 0x01) ? true: false;
			Vehicle.WheelFrontLeft.Steering.WheelSteeringState.MotorConnectionLoss=(RxData[3] & 0x02) ? true: false;
			Vehicle.WheelRearLeft.Steering.WheelSteeringState.MotorConnectionLoss=(RxData[3] & 0x04) ? true: false;
			Vehicle.WheelRearRight.Steering.WheelSteeringState.MotorConnectionLoss=(RxData[3] & 0x08) ? true: false;

			Vehicle.SteeringSystemState.LimitswitchesStatus.value = ((uint16_t) RxData[5] << 8 )| RxData[4];

			Vehicle.SteeringSystemState.SteeringNeedCalibration = (RxData[5] & 0x10) ? true: false;
			Vehicle.SteeringSystemState.CalibrationTimout = (RxData[5] & 0x20) ? true: false;
			Vehicle.SteeringSystemState.CalibrationBusy = (RxData[5] & 0x40) ? true: false;
			Vehicle.SteeringSystemState.EEPROMError = (RxData[5] & 0x80) ? true: false;

			Vehicle.SteeringSystemState.SystemReady = (RxData[7] & 0x01) ? true: false;
			Vehicle.SteeringSystemState.SysteminCalibrationMode = (RxData[7] & 0x02) ? true: false;
			Vehicle.SteeringSystemState.SysteminOperatingMode = (RxData[7] & 0x04) ? true: false;
			break;


		case BRAKE_APPLIED_TORQUE_FRONT:
			Vehicle.WheelFrontRight.Braking.BrakeTorqueSampleTime = CAST(RxData[0]);
			Vehicle.WheelFrontRight.Braking.BrakeTorque = float_decode(CAST(RxData[2]));
			Vehicle.WheelFrontLeft.Braking.BrakeTorqueSampleTime = CAST(RxData[4]);
			Vehicle.WheelFrontLeft.Braking.BrakeTorque =float_decode(CAST(RxData[6]));
			break;

		case BRAKE_APPLIED_TORQUE_REAR:
			Vehicle.WheelRearLeft.Braking.BrakeTorqueSampleTime = CAST(RxData[0]);
			Vehicle.WheelRearLeft.Braking.BrakeTorque = float_decode(CAST(RxData[2]));
			Vehicle.WheelRearRight.Braking.BrakeTorqueSampleTime = CAST(RxData[4]);
			Vehicle.WheelRearRight.Braking.BrakeTorque = float_decode(CAST(RxData[6]));
			break;
		case BRAKE_BRAKE_VALUE:
			Vehicle.WheelFrontRight.Braking.BrakeValue = RxData[0];
			Vehicle.WheelFrontLeft.Braking.BrakeValue = RxData[1];
			Vehicle.WheelRearLeft.Braking.BrakeValue = RxData[2];
			Vehicle.WheelRearRight.Braking.BrakeValue = RxData[3];
			break;
		case BRAKE_MOTORS_SYSTEM_STATUS:
			Vehicle.WheelFrontRight.Braking.BrakingStatus.MotorOverTemperature = (RxData[0] & 0x01) ? true : false;
			Vehicle.WheelFrontLeft.Braking.BrakingStatus.MotorOverTemperature = (RxData[0] & 0x02) ? true : false;
			Vehicle.WheelRearLeft.Braking.BrakingStatus.MotorOverTemperature = (RxData[0] & 0x04) ? true : false;
			Vehicle.WheelRearRight.Braking.BrakingStatus.MotorOverTemperature = (RxData[0] & 0x08) ? true : false;

			Vehicle.WheelFrontRight.Braking.BrakingStatus.MotorOverCurrent = (RxData[0] & 0x10) ? true : false;
			Vehicle.WheelFrontLeft.Braking.BrakingStatus.MotorOverCurrent = (RxData[0] & 0x20) ? true : false;
			Vehicle.WheelRearLeft.Braking.BrakingStatus.MotorOverCurrent = (RxData[0] & 0x40) ? true : false;
			Vehicle.WheelRearRight.Braking.BrakingStatus.MotorOverCurrent = (RxData[0] & 0x80) ? true : false;

			Vehicle.WheelFrontRight.Braking.BrakingStatus.MotorError = (RxData[1] & 0x01) ? true : false;
			Vehicle.WheelFrontLeft.Braking.BrakingStatus.MotorError = (RxData[1] & 0x02) ? true : false;
			Vehicle.WheelRearLeft.Braking.BrakingStatus.MotorError = (RxData[1] & 0x04) ? true : false;
			Vehicle.WheelRearRight.Braking.BrakingStatus.MotorError = (RxData[1] & 0x08) ? true : false;

			Vehicle.BrakingSystemState.SystemReady =(RxData[1] & 0x20) ? true: false;
			Vehicle.BrakingSystemState.SysteminCalibrationMode =(RxData[1] & 0x40) ? true: false;
			Vehicle.BrakingSystemState.MotorInitERROR =(RxData[1] & 0x80) ? true: false;


			Vehicle.WheelFrontRight.Braking.BrakingStatus.MotorConnectionLoss = (RxData[2] & 0x01) ? true : false;
			Vehicle.WheelFrontLeft.Braking.BrakingStatus.MotorConnectionLoss = (RxData[2] & 0x02) ? true : false;
			Vehicle.WheelRearLeft.Braking.BrakingStatus.MotorConnectionLoss = (RxData[2] & 0x04) ? true : false;
			Vehicle.WheelRearRight.Braking.BrakingStatus.MotorConnectionLoss = (RxData[2] & 0x08) ? true : false;

			Vehicle.BrakingSystemState.MotorCANBusConnection =(RxData[2] & 0x80) ? true: false;


			break;
		case BRAKE_MOTORS_TEMPERATURE:
			Vehicle.WheelFrontRight.Braking.BrakeTemperature = float_decode(CAST(RxData[0]));
			Vehicle.WheelFrontLeft.Braking.BrakeTemperature = float_decode(CAST(RxData[2]));
			Vehicle.WheelRearLeft.Braking.BrakeTemperature = float_decode(CAST(RxData[4]));
			Vehicle.WheelRearRight.Braking.BrakeTemperature = float_decode(CAST(RxData[6]));
			break;
		case BRAKE_MOTORS_CURRENT:
			Vehicle.WheelFrontRight.Braking.BrakeCurrent = float_decode(CAST(RxData[0]));
			Vehicle.WheelFrontLeft.Braking.BrakeCurrent = float_decode(CAST(RxData[2]));
			Vehicle.WheelRearLeft.Braking.BrakeCurrent = float_decode(CAST(RxData[4]));
			Vehicle.WheelRearRight.Braking.BrakeCurrent = float_decode(CAST(RxData[6]));
			break;
		case PDU_BATTERY_STATE_OF_CHARGE:
			Vehicle.PDU.BatteryStateOfCharge.BatteryVoltage = float_decode(CAST(RxData[0]));
			Vehicle.PDU.BatteryStateOfCharge.BatteryCurrent =float_decode(CAST(RxData[2]));
			Vehicle.PDU.BatteryStateOfCharge.BatteryChargePercentage = RxData[4];
			Vehicle.PDU.BatteryStateOfCharge.BatteryStatus = RxData[5];
			break;
		case PDU_FAULT_MESSAGE:
			break;
		case PDU_POWER_STATUS_CAHNNEL_RESPONSE:
			break;
		case PDU_TEMP_HUMIDITY_FAN_RESPONSE:
			break;
		case PDU_DIGITAL_ANALOG_INPUT_RESPONSE:
			break;
		}

		return STATUS_OK;
	} else
		return STATUS_ERROR;
}

// Function to get the CAN handle based on CANNumber
void* get_can_handle(CANNumber number) {
	#if (CANTYPE == FDCAN)
	        return fdcan_handles[number];
	#else
	        return can_handles[number];
	#endif
}


