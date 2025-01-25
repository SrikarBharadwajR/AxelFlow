/*
 * AX-12A.c
 *
 *  Created on: Apr 23, 2024
 *      Author: Srikar Bharadwaj R
 */
#include "AxelFlow.h"

#define DEBUG_PRINT_STATUS

Instruction_Packet packet;

Servo AxelFlow_servo_init(uint8_t id, UART_HandleTypeDef *huartx,
bool jointMode)
{
	Servo servo;
	servo.id = id;
	servo.huartx = *huartx;
	servo.jointMode = jointMode;
	return servo;
}

bool checkChecksum(Status_Packet packet)
{
	Instruction_Packet ip;
	ip.Header_1 = packet.Header_1;
	ip.Header_2 = packet.Header_2;
	ip.Packet_ID = packet.Packet_ID;
	ip.Length = packet.Length;
	ip.Instruction = packet.Error;
	ip.Param = packet.Param;
	ip.Checksum = packet.Checksum;
	if (packet.Checksum == getChecksum(ip))
		return 1;
	else
		return 0;
}

uint8_t* degreesToData(float degrees)
{
	static uint8_t data[2];
	uint16_t degrees_Hex = (uint16_t) (degrees * 3.41f); // 1023/300
	data[0] = degrees_Hex;
	data[1] = degrees_Hex >> 8;
	if (degrees == 0.0)
	{
		data[0] = 0;
		data[1] = 0;
	}

	return data;
}
float dataToDegrees(uint8_t *Param)
{
	return ((uint16_t) Param[0] + (Param[1] << 8)) / 3.41;
}

uint16_t getCCWLimit(Servo servo)
{
	uint8_t param_array[2] = { EEPROM_CCW_ANGLE_LIMIT_L, ALL };
	packet.Packet_ID = servo.id;
	packet.Length = sizeof(param_array) + 2;
	packet.Instruction = COMMAND_READ_DATA;
	packet.Param = param_array;
	Status_Packet status = AxelFlow_fire(&servo.huartx, packet);
	if (status.Error == 0)
		return dataToDegrees(status.Param);
	else
		return -1;
}
Status_Packet setCCWLimit(float max_angle, Servo servo)
{
	uint8_t *data = degreesToData(max_angle);
	uint8_t param_array[3] = { EEPROM_CCW_ANGLE_LIMIT_L, data[0], data[1] };
	packet.Packet_ID = servo.id;
	packet.Length = sizeof(param_array) + 2;
	packet.Instruction = COMMAND_WRITE_DATA;
	packet.Param = param_array;
	return AxelFlow_fire(&servo.huartx, packet);
}
uint16_t getCWLimit(Servo servo)
{
	uint8_t param_array[2] = { EEPROM_CW_ANGLE_LIMIT_L, ALL };
	packet.Packet_ID = servo.id;
	packet.Length = sizeof(param_array) + 2;
	packet.Instruction = COMMAND_READ_DATA;
	packet.Param = param_array;
	Status_Packet status = AxelFlow_fire(&servo.huartx, packet);
	if (status.Error == 0)
		return dataToDegrees(status.Param);
	else
		return -1;
}
Status_Packet setCWLimit(float max_angle, Servo servo)
{
	uint8_t *data = degreesToData(max_angle);
	uint8_t param_array[3] = { EEPROM_CW_ANGLE_LIMIT_L, data[0], data[1] };
	packet.Packet_ID = servo.id;
	packet.Length = sizeof(param_array) + 2;
	packet.Instruction = COMMAND_WRITE_DATA;
	packet.Param = param_array;
	return AxelFlow_fire(&servo.huartx, packet);
}
Status_Packet setPosition(float angle, Servo servo)
{
	uint8_t *data = degreesToData(angle);
	uint8_t param_array[3] = { RAM_GOAL_POSITION_L, data[0], data[1] };
	packet.Packet_ID = servo.id;
	packet.Length = sizeof(param_array) + 2;
	packet.Instruction = COMMAND_WRITE_DATA;
	packet.Param = param_array;
	return AxelFlow_fire(&servo.huartx, packet);
}

Status_Packet setMaxTorque(float torque, Servo servo)	// torque 0 to 100%
{
	uint8_t *data = degreesToData(torque * 3); // 1023 / (100*3.41)
	uint8_t param_array[3] = { EEPROM_MAX_TORQUE_L, data[0], data[1] };
	packet.Packet_ID = servo.id;
	packet.Length = sizeof(param_array) + 2;
	packet.Instruction = COMMAND_WRITE_DATA;
	packet.Param = param_array;
	return AxelFlow_fire(&servo.huartx, packet);
}
float getMaxTorque(Servo servo)

{
	uint8_t param_array[2] = { EEPROM_MAX_TORQUE_L, ALL };
	packet.Packet_ID = servo.id;
	packet.Length = sizeof(param_array) + 2;
	packet.Instruction = COMMAND_READ_DATA;
	packet.Param = param_array;
	Status_Packet status = AxelFlow_fire(&servo.huartx, packet);
	if (status.Error == 0)
		return (dataToDegrees(status.Param) / 3.0);
	else
		return 0;
}

Status_Packet setRunningTorque(float torque, Servo servo)	// torque 0 to 100%
{
	uint8_t *data = degreesToData(torque * 3); // 1023 / (100*3.41)
	uint8_t param_array[3] = { RAM_TORQUE_LIMIT_L, data[0], data[1] };
	packet.Packet_ID = servo.id;
	packet.Length = sizeof(param_array) + 2;
	packet.Instruction = COMMAND_WRITE_DATA;
	packet.Param = param_array;
	return AxelFlow_fire(&servo.huartx, packet);
}

Status_Packet setSpeed(float speed, Servo servo)
{
	uint8_t *data;
	if (servo.jointMode)
	{
		data = (speed < 99) ? degreesToData(speed * 3) : 0; // 1023 / (100*3.41)
	}
	else // TODO; Add switch
	{
		speed *= CLOCKWISE_SWITCH;
		data = (speed >= 0 && speed <= 100) ?
				degreesToData(speed * 3.0) :
				((speed < 0 && speed >= -100) ?
						degreesToData((-speed + 100) * 3) : 0);
	}
	uint8_t param_array[3] = { RAM_GOAL_SPEED_L, data[0], data[1] };
	packet.Packet_ID = servo.id;
	packet.Length = sizeof(param_array) + 2;
	packet.Instruction = COMMAND_WRITE_DATA;
	packet.Param = param_array;
	return AxelFlow_fire(&servo.huartx, packet);
}
Status_Packet setSpeedinRPM(float rpm, Servo servo)
{
	uint8_t *data;

	if (servo.jointMode)
	{
		data = (rpm < 114) ? degreesToData(rpm * 2.632) : 0; // 1023 / (114*3.41)
	}
	else
	{
		data = NULL;
	}
	uint8_t param_array[3] = { RAM_GOAL_SPEED_L, data[0], data[1] };
	packet.Packet_ID = servo.id;
	packet.Length = sizeof(param_array) + 2;
	packet.Instruction = COMMAND_WRITE_DATA;
	return AxelFlow_fire(&servo.huartx, packet);

}

Status_Packet controlLED(bool state, Servo servo)
{
	uint8_t param_array[2] = { RAM_LED, state };
	packet.Packet_ID = servo.id;
	packet.Length = sizeof(param_array) + 2;
	packet.Instruction = COMMAND_WRITE_DATA;
	return AxelFlow_fire(&servo.huartx, packet);
}
uint8_t getLEDStatus(Servo servo)
{
	uint8_t param_array[1] = { RAM_LED };
	packet.Packet_ID = servo.id;
	packet.Length = sizeof(param_array) + 2;
	packet.Instruction = COMMAND_READ_DATA;
	packet.Param = param_array;
	Status_Packet status = AxelFlow_fire(&servo.huartx, packet);
	if (status.Error == 0)
		return status.Param[0];
	else
		return -1;
}

Status_Packet getPosition(Servo servo)
{
	uint8_t param_array[2] = { RAM_PRESENT_POSITION_L, ALL };
	packet.Packet_ID = servo.id;
	packet.Length = sizeof(param_array) + 2;
	packet.Instruction = COMMAND_READ_DATA;
	packet.Param = param_array;
	return AxelFlow_fire(&servo.huartx, packet);
}
float getPositionAngle(Servo servo)
{
	Status_Packet status = getPosition(servo);
	return dataToDegrees(status.Param);
}
Status_Packet getSpeed(Servo servo)

{
	uint8_t param_array[2] = { RAM_PRESENT_SPEED_L, ALL };
	packet.Packet_ID = servo.id;
	packet.Length = sizeof(param_array) + 2;
	packet.Instruction = COMMAND_READ_DATA;
	packet.Param = param_array;
	return AxelFlow_fire(&servo.huartx, packet);
}
float getSpeedRPM(Servo servo)
{
	Status_Packet status = getSpeed(servo);
	return dataToDegrees(status.Param);
}

Status_Packet reboot(Servo servo)
{
	uint8_t param_array[0] = { };
	packet.Packet_ID = servo.id;
	packet.Length = sizeof(param_array) + 2;
	packet.Instruction = COMMAND_REBOOT;
	packet.Param = param_array;
	return AxelFlow_fire(&servo.huartx, packet);
}

bool ping(Servo servo)
{
	uint8_t param_array[0] = { };
	packet.Packet_ID = servo.id;
	packet.Length = sizeof(param_array) + 2;
	packet.Instruction = COMMAND_PING;
	packet.Param = param_array;
	Status_Packet status = AxelFlow_fire(&servo.huartx, packet);
	if (status.Error
			== 0&& status.Header_1 == HEADER && status.Header_2 == HEADER)
		return true;
	else
		return false;
}
Status_Packet changeServoID(uint8_t future_ID, Servo servo)
{
	uint8_t param_array[2] = { EEPROM_ID, future_ID };
	packet.Packet_ID = servo.id;
	packet.Length = sizeof(param_array) + 2;
	packet.Instruction = COMMAND_WRITE_DATA;
	packet.Param = param_array;
	return AxelFlow_fire(&servo.huartx, packet);
}
uint8_t readID(Servo servo) //useless function
{
	uint8_t param_array[1] = { EEPROM_ID };
	packet.Packet_ID = servo.id;
	packet.Length = sizeof(param_array) + 2;
	packet.Instruction = COMMAND_READ_DATA;
	packet.Param = param_array;
	Status_Packet status = AxelFlow_fire(&servo.huartx, packet);
	if (status.Error == 0)
		return status.Param[0];
	else
		return -1;
}

BaudRate getBaudRate(int new_Baud_Rate)
{
	if (new_Baud_Rate <= 9600)
		return BAUD_9600;
	else if (new_Baud_Rate <= 19200)
		return BAUD_19200;
	else if (new_Baud_Rate <= 57600)
		return BAUD_57600;
	else if (new_Baud_Rate <= 115200)
		return BAUD_115200;
	else if (new_Baud_Rate <= 200000)
		return BAUD_200000;
	else if (new_Baud_Rate <= 250000)
		return BAUD_250000;
	else if (new_Baud_Rate <= 400000)
		return BAUD_400000;
	else if (new_Baud_Rate <= 500000)
		return BAUD_500000;
	else
		return BAUD_DEFAULT;
}
Status_Packet changeBaudRate(uint32_t new_Baud_Rate, Servo servo)
{
	uint8_t param_array[2] = { EEPROM_BAUD_RATE, getBaudRate(new_Baud_Rate) };
	packet.Packet_ID = servo.id;
	packet.Length = sizeof(param_array) + 2;
	packet.Instruction = COMMAND_WRITE_DATA;
	packet.Param = param_array;
	return AxelFlow_fire(&servo.huartx, packet);
}
uint32_t readBaudRate(Servo servo)
{
	uint8_t param_array[1] = { EEPROM_BAUD_RATE };
	packet.Packet_ID = servo.id;
	packet.Length = sizeof(param_array) + 2;
	packet.Instruction = COMMAND_READ_DATA;
	packet.Param = param_array;
	Status_Packet status = AxelFlow_fire(&servo.huartx, packet);
	switch ((uint8_t) *status.Param)
	{
	case BAUD_9600:
		return 9600;
	case BAUD_19200:
		return 19200;
	case BAUD_57600:
		return 57600;
	case BAUD_115200:
		return 115200;
	case BAUD_200000:
		return 200000;
	case BAUD_250000:
		return 250000;
	case BAUD_400000:
		return 400000;
	case BAUD_500000:
		return 500000;
	case BAUD_DEFAULT:
		return 1E6;
	default:
		return 0;
	}
}
Status_Packet forceSetPosition(uint16_t angle, Servo servo)
{
	uint16_t current_angle;
	Status_Packet status;
	do
	{
		status = setPosition(angle, servo);
		current_angle = (uint16_t) getPositionAngle(servo);
	} while (angle != current_angle && angle != current_angle + 1
			&& angle != current_angle - 1);
	return status;
}

uint8_t getFirmwareVersion(Servo servo)
{
	uint8_t param_array[1] = { EEPROM_VERSION };
	packet.Packet_ID = servo.id;
	packet.Length = sizeof(param_array) + 2;
	packet.Instruction = COMMAND_READ_DATA;
	packet.Param = param_array;
	Status_Packet status = AxelFlow_fire(&servo.huartx, packet);
	if (status.Error == 0)
		return status.Param[0];
	else
		return 0;
}
uint8_t scanID(Servo servo)
{
	for (uint8_t id = ID_MIN; id < ID_MAX; id++)
	{
		servo.id = id;
		if (ping(servo))
			return id;
	}
	return 0xFF;
}
uint32_t scanBaudRate(Servo servo) // TODO: Test
{
	uint32_t baudRates[9] = { 9600, 19200, 57600, 115200, 200000, 250000,
			400000, 500000, 1000000 };
	for (uint8_t baudID = 0; baudID < 9; baudID++)
	{
//		char ch[10];
//		sprintf(ch, "%lu\n\r", baudRates[baudID]);
//		AxelFlow_debug_println(ch);

		UART_HandleTypeDef servo1_UART_Handle = AxelFlow_UART_Init(USART1,
				getBaudRate(baudRates[baudID]));
		servo.huartx = servo1_UART_Handle;
		HAL_Delay(2);
		if (ping(servo))
			return baudRates[baudID];
	}
	return 0;
}
uint16_t getResponseDelayTime(Servo servo)
{
	uint8_t param_array[1] = { EEPROM_RETURN_DELAY_TIME };
	packet.Packet_ID = servo.id;
	packet.Length = sizeof(param_array) + 2;
	packet.Instruction = COMMAND_READ_DATA;
	packet.Param = param_array;
	Status_Packet status = AxelFlow_fire(&servo.huartx, packet);
	if (status.Error == 0)
		return status.Param[0] * 2;
	else
		return -1;
}
Status_Packet setResponseDelayTime(uint16_t response_time, Servo servo)
{
	uint8_t param_array[2] = { EEPROM_RETURN_DELAY_TIME, (response_time / 2) };
	packet.Packet_ID = servo.id;
	packet.Length = sizeof(param_array) + 2;
	packet.Instruction = COMMAND_WRITE_DATA;
	packet.Param = param_array;
	return AxelFlow_fire(&servo.huartx, packet);
}
uint8_t getTemperatureLimit(Servo servo)
{
	uint8_t param_array[1] = { EEPROM_LIMIT_TEMPERATURE };
	packet.Packet_ID = servo.id;
	packet.Length = sizeof(param_array) + 2;
	packet.Instruction = COMMAND_READ_DATA;
	packet.Param = param_array;
	Status_Packet status = AxelFlow_fire(&servo.huartx, packet);
	if (status.Error == 0)
		return status.Param[0];
	else
		return -1;
}
Status_Packet setMinVoltageLimit(float min_voltage, Servo servo)
{
	uint8_t param_array[2] = { EEPROM_LOW_LIMIT_VOLTAGE, (uint8_t) (min_voltage
			* 10) };
	packet.Packet_ID = servo.id;
	packet.Length = sizeof(param_array) + 2;
	packet.Instruction = COMMAND_WRITE_DATA;
	packet.Param = param_array;
	return AxelFlow_fire(&servo.huartx, packet);
}

uint8_t getMinVoltageLimit(Servo servo)
{
	uint8_t param_array[1] = { EEPROM_LOW_LIMIT_VOLTAGE };
	packet.Packet_ID = servo.id;
	packet.Length = sizeof(param_array) + 2;
	packet.Instruction = COMMAND_READ_DATA;
	packet.Param = param_array;
	Status_Packet status = AxelFlow_fire(&servo.huartx, packet);
	if (status.Error == 0)
		return (status.Param[0] / 10);
	else
		return -1;
}
Status_Packet setMaxVoltageLimit(float max_voltage, Servo servo)
{
	uint8_t param_array[2] = { EEPROM_HIGH_LIMIT_VOLTAGE, (uint8_t) (max_voltage
			* 10) };
	packet.Packet_ID = servo.id;
	packet.Length = sizeof(param_array) + 2;
	packet.Instruction = COMMAND_WRITE_DATA;
	packet.Param = param_array;
	return AxelFlow_fire(&servo.huartx, packet);
}

uint8_t getMaxVoltageLimit(Servo servo)
{
	uint8_t param_array[1] = { EEPROM_HIGH_LIMIT_VOLTAGE };
	packet.Packet_ID = servo.id;
	packet.Length = sizeof(param_array) + 2;
	packet.Instruction = COMMAND_READ_DATA;
	packet.Param = param_array;
	Status_Packet status = AxelFlow_fire(&servo.huartx, packet);
	if (status.Error == 0)
		return (status.Param[0] / 10);
	else
		return -1;
}

Status_Packet setTemperatureLimit(float max_temp, Servo servo)
{
	uint8_t param_array[2] = { EEPROM_LIMIT_TEMPERATURE, max_temp };
	packet.Packet_ID = servo.id;
	packet.Length = sizeof(param_array) + 2;
	packet.Instruction = COMMAND_WRITE_DATA;
	packet.Param = param_array;
	return AxelFlow_fire(&servo.huartx, packet);
}
uint8_t getStatusReturnLevel(Servo servo)
{
	uint8_t param_array[1] = { EEPROM_RETURN_LEVEL };
	packet.Packet_ID = servo.id;
	packet.Length = sizeof(param_array) + 2;
	packet.Instruction = COMMAND_READ_DATA;
	packet.Param = param_array;
	Status_Packet status = AxelFlow_fire(&servo.huartx, packet);
	if (status.Error == 0)
		return status.Param[0];
	else
		return -1;
}

Status_Packet setStatusReturnLevel(uint8_t status, Servo servo)
{
	uint8_t param_array[2] = { EEPROM_RETURN_LEVEL, status };
	packet.Packet_ID = servo.id;
	packet.Length = sizeof(param_array) + 2;
	packet.Instruction = COMMAND_WRITE_DATA;
	packet.Param = param_array;
	return AxelFlow_fire(&servo.huartx, packet);
}
uint8_t getAlarmLEDStatus(Servo servo)
{
	uint8_t param_array[1] = { EEPROM_ALARM_LED };
	packet.Packet_ID = servo.id;
	packet.Length = sizeof(param_array) + 2;
	packet.Instruction = COMMAND_READ_DATA;
	packet.Param = param_array;
	Status_Packet status = AxelFlow_fire(&servo.huartx, packet);
	if (status.Error == 0)
		return status.Param[0];
	else
		return -1;
}

Status_Packet setAlarmLEDStatus(uint8_t status, Servo servo)
{
	uint8_t param_array[2] = { EEPROM_ALARM_LED, status };
	packet.Packet_ID = servo.id;
	packet.Length = sizeof(param_array) + 2;
	packet.Instruction = COMMAND_WRITE_DATA;
	packet.Param = param_array;
	return AxelFlow_fire(&servo.huartx, packet);
}
uint8_t getShutdownStatus(Servo servo)
{
	uint8_t param_array[1] = { EEPROM_ALARM_SHUTDOWN };
	packet.Packet_ID = servo.id;
	packet.Length = sizeof(param_array) + 2;
	packet.Instruction = COMMAND_READ_DATA;
	packet.Param = param_array;
	Status_Packet status = AxelFlow_fire(&servo.huartx, packet);
	if (status.Error == 0)
		return status.Param[0];
	else
		return -1;
}
Status_Packet setShutdownStatus(uint8_t status, Servo servo)
{
	uint8_t param_array[2] = { EEPROM_ALARM_SHUTDOWN, status };
	packet.Packet_ID = servo.id;
	packet.Length = sizeof(param_array) + 2;
	packet.Instruction = COMMAND_WRITE_DATA;
	packet.Param = param_array;
	return AxelFlow_fire(&servo.huartx, packet);
}
uint8_t getTorqueEnableStatus(Servo servo)
{
	uint8_t param_array[1] = { RAM_TORQUE_ENABLE };
	packet.Packet_ID = servo.id;
	packet.Length = sizeof(param_array) + 2;
	packet.Instruction = COMMAND_READ_DATA;
	packet.Param = param_array;
	Status_Packet status = AxelFlow_fire(&servo.huartx, packet);
	if (status.Error == 0)
		return status.Param[0];
	else
		return -1;
}
Status_Packet setTorqueEnableStatus(bool enable, Servo servo)
{
	uint8_t param_array[2] = { RAM_TORQUE_ENABLE, enable };
	packet.Packet_ID = servo.id;
	packet.Length = sizeof(param_array) + 2;
	packet.Instruction = COMMAND_WRITE_DATA;
	packet.Param = param_array;
	return AxelFlow_fire(&servo.huartx, packet);
}
void setWheelMode(Servo servo)
{

	setCCWLimit(1, servo);
	HAL_Delay(10);
	setCWLimit(1, servo);

}
float getPresentLoad(Servo servo)
{
	uint8_t param_array[2] = { RAM_PRESENT_LOAD_L, ALL };
	packet.Packet_ID = servo.id;
	packet.Length = sizeof(param_array) + 2;
	packet.Instruction = COMMAND_READ_DATA;
	packet.Param = param_array;
	Status_Packet status = AxelFlow_fire(&servo.huartx, packet);
	if (status.Error == 0)
	{
		float load = dataToDegrees(status.Param) * 3.41;
		load = (load < 1023) ?
				load * (100.0 / 1023) : (load * (-100.0 / 1023) + 100.0);
		return load;
	}
	else
		return -1;
}

float getPresentTemperature(Servo servo)
{
	uint8_t param_array[1] = { RAM_PRESENT_TEMPERATURE };
	packet.Packet_ID = servo.id;
	packet.Length = sizeof(param_array) + 2;
	packet.Instruction = COMMAND_READ_DATA;
	packet.Param = param_array;
	Status_Packet status = AxelFlow_fire(&servo.huartx, packet);
	if (status.Error == 0)
	{
		return status.Param[0] * 1.0;
	}
	else
		return -1;
}

float getPresentVoltage(Servo servo)
{
	uint8_t param_array[1] = { RAM_PRESENT_VOLTAGE };
	packet.Packet_ID = servo.id;
	packet.Length = sizeof(param_array) + 2;
	packet.Instruction = COMMAND_READ_DATA;
	packet.Param = param_array;
	Status_Packet status = AxelFlow_fire(&servo.huartx, packet);
	if (status.Error == 0)
	{
		return status.Param[0] / 10.0;
	}
	else
		return -1;
}

uint8_t getRegistered(Servo servo)
{
	uint8_t param_array[1] = { RAM_REGISTER };
	packet.Packet_ID = servo.id;
	packet.Length = sizeof(param_array) + 2;
	packet.Instruction = COMMAND_READ_DATA;
	packet.Param = param_array;
	Status_Packet status = AxelFlow_fire(&servo.huartx, packet);
	if (status.Error == 0)
		return status.Param[0];
	else
		return -1;

}

uint8_t getMoving(Servo servo)
{
	uint8_t param_array[1] = { RAM_MOVING };
	packet.Packet_ID = servo.id;
	packet.Length = sizeof(param_array) + 2;
	packet.Instruction = COMMAND_READ_DATA;
	packet.Param = param_array;
	Status_Packet status = AxelFlow_fire(&servo.huartx, packet);
	if (status.Error == 0)
		return status.Param[0];
	else
		return -1;

}
uint8_t getLockStatus(Servo servo)
{
	uint8_t param_array[1] = { RAM_LOCK };
	packet.Packet_ID = servo.id;
	packet.Length = sizeof(param_array) + 2;
	packet.Instruction = COMMAND_READ_DATA;
	packet.Param = param_array;
	Status_Packet status = AxelFlow_fire(&servo.huartx, packet);
	if (status.Error == 0)
		return status.Param[0];
	else
		return -1;

}
Status_Packet setPunch(float punch, Servo servo)
{
	uint8_t *data = degreesToData(punch * 3.0);
	uint8_t param_array[3] = { RAM_PUNCH_L, data[0], data[1] };
	packet.Packet_ID = servo.id;
	packet.Length = sizeof(param_array) + 2;
	packet.Instruction = COMMAND_WRITE_DATA;
	packet.Param = param_array;
	return AxelFlow_fire(&servo.huartx, packet);
}

float getPunch(Servo servo)
{
	uint8_t param_array[2] = { RAM_PUNCH_L, RAM_PUNCH_H };
	packet.Packet_ID = servo.id;
	packet.Length = sizeof(param_array) + 2;
	packet.Instruction = COMMAND_READ_DATA;
	packet.Param = param_array;
	Status_Packet status = AxelFlow_fire(&servo.huartx, packet);
	if (status.Error == 0)
	{
		float data = dataToDegrees(status.Param) / 3.0;
		return data;
	}
	else
		return -1;

}

#ifdef DEBUG_PRINT_STATUS
void print_status(Status_Packet packet, bool just_Errors)
{
	if (!(just_Errors && packet.Error == 0x00))
	{
		if (packet.Header_1 == HEADER && packet.Header_2 == HEADER)
		{
			char temp[150] = { '\0' };
			sprintf(temp, "Header 1:  0x%02X", packet.Header_1);
			AxelFlow_debug_println(temp);
			sprintf(temp, "Header 2:  0x%02X", packet.Header_2);
			AxelFlow_debug_println(temp);
			sprintf(temp, "Packet ID: 0x%02X", packet.Packet_ID);
			AxelFlow_debug_println(temp);
			sprintf(temp, "Length:    0x%02X", packet.Length);
			AxelFlow_debug_println(temp);
			char error[100] = { '\0' };
			if (packet.Error & (1 << Input_Voltage_Error))
				strcat(error, "Input Voltage Error\t");
			if (packet.Error & (1 << Angle_Limit_Error))
				strcat(error, "Angle Limit Error\t");
			if (packet.Error & (1 << Overheating_Error))
				strcat(error, "Overheating Error\t");
			if (packet.Error & (1 << Range_Error))
				strcat(error, "Range Error\t");
			if (packet.Error & (1 << Checksum_Error))
				strcat(error, "Checksum Error\t");
			if (packet.Error & (1 << Overload_Error))
				strcat(error, "Overload Error\t");
			if (packet.Error & (1 << Instruction_Error))
				strcat(error, "Instruction Error\t");
			sprintf(temp, "Error:    %s(0x%02X)", error, packet.Error);
			AxelFlow_debug_println(temp);
			AxelFlow_debug_print("Param:\t");
			for (uint8_t i = 0; i < packet.Length - 2; i++)
			{
				sprintf(temp, "0x%02X\t", packet.Param[i]);
				AxelFlow_debug_print(temp);
			}
			AxelFlow_debug_println("");
			sprintf(temp, "Checksum:  0x%02X", packet.Checksum);
			AxelFlow_debug_println(temp);
			AxelFlow_debug_println("");
		}
		else
		{
			AxelFlow_debug_println("No Servo Connected");
		}
	}
}
#endif

