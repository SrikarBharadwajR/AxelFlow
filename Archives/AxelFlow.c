/*
 * AX-12A.c
 *
 *  Created on: Apr 23, 2024
 *      Author: Srikar Bharadwaj R
 */
#include "AxelFlow.h"

//extern uint8_t rx_data[Rx_DATA_SIZE];
//extern volatile uint8_t received;
//
//Instruction_Packet packet;
//
//
//uint8_t getChecksum()
//{
//	uint8_t sum = packet.Packet_ID + packet.Length + packet.Instruction;
//
//	for (uint8_t i = 0; i < packet.Length - 2; i++)
//	{
//		sum += packet.Param[i];
//	}
//	return ~sum;
//}
//
//bool checkChecksum()
//{
//	if (packet.Checksum == getChecksum())
//		return 1;
//	else
//		return 0;
//}
//HAL_UART_StateTypeDef servoInit(UART_HandleTypeDef *huart)
//{
//	huartx = *huart;
//	return HAL_UART_GetState(huart);
//}
//bool transmitToServo()
//{
//	HAL_HalfDuplex_EnableTransmitter(&huartx);
//	char send_data[sizeof(packet)];
//	send_data[0] = HEADER;
//	send_data[1] = HEADER;
//	send_data[2] = packet.Packet_ID;
//	send_data[3] = packet.Length;
//	send_data[4] = packet.Instruction;
//	uint8_t i = 0;
//	for (i = 5; i < packet.Length + 3; i++)
//		send_data[i] = packet.Param[i - 5];
//	packet.Checksum = getChecksum(packet);
//	send_data[packet.Length + 3] = packet.Checksum;
//	if (HAL_UART_Transmit(&huartx, (uint8_t*) send_data, packet.Length + 4, 1)
//			== HAL_OK)
//		return 1;
//	else
//		return 0;
//}
//bool receiveFromServo(bool USBTransmit)
//{
//	HAL_HalfDuplex_EnableReceiver(&huartx);
//	uint32_t current_time = HAL_GetTick();
//	bool valid = 0;
//	while (HAL_GetTick() - current_time <= STATUS_PACKET_TIMEOUT)
//	{
//		if (received == 1)
//		{
//			uint8_t len_cnt = 0;
//			uint8_t start_flag = 0;
//			int8_t status_len = -1;
//			uint8_t j = 0, l = 0;
//			struct Instruction_Packet packet;
//			uint8_t *param_array = (uint8_t*) malloc(2 * STATUS_FRAME_BUFFER);
//
//			for (int i = 0; i < Rx_DATA_SIZE; i++)
//			{
//				if (rx_data[i] == 0xFF && !start_flag)
//				{
//					j = -2;
//					start_flag = 1;
//				}
//				if (start_flag)
//				{
//					++len_cnt;
//					sprintf(&usb_data[j += 2], "%02X", rx_data[i]);
//				}
//				if (len_cnt == 4)
//				{
//					status_len = rx_data[i];
//					packet.Packet_ID = rx_data[i - 1];
//					packet.Length = rx_data[i];
//					packet.Instruction = rx_data[i + 1];
//					l = i;
//				}
//				if (status_len > 0)
//				{
//					param_array[i-l] = rx_data[i+2];
//				}
//				if (status_len == 0)
//				{
////					param_array[len_cnt - 4] = ;
//					packet.Checksum = rx_data[i];
////					for (uint8_t k = len_cnt - 5	; k < STATUS_FRAME_BUFFER; k++)
////						param_array[k] = '\0';
//					break;
//				}
//				status_len--;
//			}
//			packet.Param = param_array;
//			valid = checkChecksum(packet);
////			char send_data[6];
////			sprintf(send_data, "%2X\n\r", packet.Param[1]);
////			CDC_Transmit_FS((uint8_t*) send_data, strlen(send_data));
//
//			if (USBTransmit && valid)
//			{
//				transmitToUSB();
//			}
//			received = 0;
//			return 1;
//			break;
//		}
////		if (valid)
////		{
////			if (USBTransmit)
////			{
////				transmitToUSB();
////			}
////			received = 0;
////			return 1;
////			break;
////		}
//	}
//	return 0;
//}
//
//void transmitToUSB()
//{
//	strcat(usb_data, "\n\r");
//	CDC_Transmit_FS((uint8_t*) usb_data, strlen(usb_data));
//	memset(usb_data, '\0', 30);
//}
//
//uint8_t* degreesToData(float degrees)
//{
//	uint8_t *data = (uint8_t*) malloc(2 * sizeof(uint8_t));
//	uint16_t degrees_Hex = degrees * 3.41; // 1023/300
//	data[0] = degrees_Hex;
//	data[1] = degrees_Hex >> 8;
//	return data;
//}
//
//void setCCWLimit(uint8_t id, float max_angle)
//{
//	struct Instruction_Packet packet;
//	uint8_t *data = degreesToData(max_angle);
//	uint8_t param_array[3] = { EEPROM_CCW_ANGLE_LIMIT_L, data[0], data[1] };
//	packet.Packet_ID = id;
//	packet.Length = sizeof(param_array) + 2;
//	packet.Instruction = COMMAND_WRITE_DATA;
//	packet.Param = param_array;
//	transmitToServo(packet);
//}
//void setCWLimit(uint8_t id, float max_angle)
//{
//	struct Instruction_Packet packet;
//	uint8_t *data = degreesToData(max_angle);
//	uint8_t param_array[3] = { EEPROM_CW_ANGLE_LIMIT_L, data[0], data[1] };
//	packet.Packet_ID = id;
//	packet.Length = sizeof(param_array) + 2;
//	packet.Instruction = COMMAND_WRITE_DATA;
//	packet.Param = param_array;
//	transmitToServo(packet);
//}
//void setPosition(uint8_t id, float angle)
//{
//	struct Instruction_Packet packet;
//	uint8_t *data = degreesToData(angle);
//	uint8_t param_array[3] = { RAM_GOAL_POSITION_L, data[0], data[1] };
//	packet.Packet_ID = id;
//	packet.Length = sizeof(param_array) + 2;
//	packet.Instruction = COMMAND_WRITE_DATA;
//	packet.Param = param_array;
//	transmitToServo(packet);
//}
//void setMaxTorque(uint8_t id, float torque)	// torque 0 to 100%
//{
//	struct Instruction_Packet packet;
//	uint8_t *data = degreesToData(torque * 3); // 1023 / (100*3.41)
//	uint8_t param_array[3] = { EEPROM_MAX_TORQUE_L, data[0], data[1] };
//	packet.Packet_ID = id;
//	packet.Length = sizeof(param_array) + 2;
//	packet.Instruction = COMMAND_WRITE_DATA;
//	packet.Param = param_array;
//	transmitToServo(packet);
//}
//void setRunningTorque(uint8_t id, float torque)	// torque 0 to 100%
//{
//	struct Instruction_Packet packet;
//	uint8_t *data = degreesToData(torque * 3); // 1023 / (100*3.41)
//	uint8_t param_array[3] = { RAM_TORQUE_LIMIT_L, data[0], data[1] };
//	packet.Packet_ID = id;
//	packet.Length = sizeof(param_array) + 2;
//	packet.Instruction = COMMAND_WRITE_DATA;
//	packet.Param = param_array;
//	transmitToServo(packet);
//}
//
//void reboot(uint8_t id)
//{
//	struct Instruction_Packet packet;
//	uint8_t param_array[0] = { };
//	packet.Packet_ID = id;
//	packet.Length = sizeof(param_array) + 2;
//	packet.Instruction = COMMAND_REBOOT;
//	packet.Param = param_array;
//	transmitToServo(packet);
//}
//void setJointSpeed(uint8_t id, float speed)
//{
//	struct Instruction_Packet packet;
//	uint8_t *data = degreesToData(speed * 2.632); // 1023 / (114*3.41)
//	uint8_t param_array[3] = { RAM_GOAL_SPEED_L, data[0], data[1] };
//	packet.Packet_ID = id;
//	packet.Length = sizeof(param_array) + 2;
//	packet.Instruction = COMMAND_WRITE_DATA;
//	packet.Param = param_array;
//	transmitToServo(packet);
//
//}
//void getPosition(uint8_t id)
//{
//	struct Instruction_Packet packet;
//	uint8_t param_array[2] = { RAM_PRESENT_POSITION_L, ALL };
//	packet.Packet_ID = id;
//	packet.Length = sizeof(param_array) + 2;
//	packet.Instruction = COMMAND_READ_DATA;
//	packet.Param = param_array;
//	transmitToServo(packet);
//}

/* Functions to be written:
 * IMPORTANT
 * Set torque
 *
 *
 *



 */

