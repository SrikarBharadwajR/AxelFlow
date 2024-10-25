/*
 * AX-12A.c
 *
 *  Created on: Apr 23, 2024
 *      Author: Srikar Bharadwaj R
 */
#include "AxelFlow.h"

#define DEBUG_PRINT_STATUS

Instruction_Packet packet;

Servo AxelFlow_servo_init(uint8_t id, UART_HandleTypeDef *huartx)
{
	Servo servo;
	servo.id = id;
	servo.huartx = *huartx;
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
	uint8_t *data = (uint8_t*) malloc(2 * sizeof(uint8_t));
	uint16_t degrees_Hex = degrees * 3.41; // 1023/300
	data[0] = degrees_Hex;
	data[1] = degrees_Hex >> 8;
	return data;
}
float dataToDegrees(uint8_t *Param)
{
	return ((uint16_t) Param[0] + (Param[1] << 8)) / 3.41;
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

Status_Packet setJointSpeed(float speed, Servo servo)
{
	uint8_t *data = degreesToData(speed * 2.632); // 1023 / (114*3.41)
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
	if (status.Error == 0)
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
	uint8_t baud_Rate;
	if (new_Baud_Rate <= 9600) // Margin of Error = -0.160%
		baud_Rate = 0xCF;
	else if (new_Baud_Rate > 9600 && new_Baud_Rate <= 19200) // Margin of Error = -0.160%
		baud_Rate = 0x67;
	else if (new_Baud_Rate > 19200 && new_Baud_Rate <= 57600) // Margin of Error = 0.794%
		baud_Rate = 0x22;
	else if (new_Baud_Rate > 57600 && new_Baud_Rate <= 115200) // Margin of Error = -2.124%
		baud_Rate = 0x10;
	else if (new_Baud_Rate > 115200 && new_Baud_Rate <= 200000) // Margin of Error = 0.000%
		baud_Rate = 0x09;
	else if (new_Baud_Rate > 200000 && new_Baud_Rate <= 250000) // Margin of Error = 0.000%
		baud_Rate = 0x07;
	else if (new_Baud_Rate > 250000 && new_Baud_Rate <= 400000) // Margin of Error = 0.000%
		baud_Rate = 0x04;
	else if (new_Baud_Rate > 400000 && new_Baud_Rate <= 500000) // Margin of Error = 0.000%
		baud_Rate = 0x03;
	else
		baud_Rate = 0x01; // Margin of Error = 0.000% (default)

	uint8_t param_array[2] = { EEPROM_BAUD_RATE, baud_Rate };
	packet.Packet_ID = servo.id;
	packet.Length = sizeof(param_array) + 2;
	packet.Instruction = COMMAND_WRITE_DATA;
	packet.Param = param_array;
	return AxelFlow_fire(&servo.huartx, packet);
}
Status_Packet forceSetPosition(uint16_t angle, Servo servo)
{
	uint16_t current_angle;
	Status_Packet status;
	do
	{
		status = setPosition(angle, servo);
		current_angle = (uint16_t) getPositionAngle(servo);
//		char ch[10];
//		sprintf(ch,"%u\n\r",current_angle);
//		AxelFlow_debug_print(ch);
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

#ifdef DEBUG_PRINT_STATUS
void print_status(Status_Packet packet, bool just_Errors)
{
	if (packet.Header_1 == HEADER && packet.Header_2 == HEADER
			&& !(just_Errors && packet.Error == 0x00))
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
#endif

