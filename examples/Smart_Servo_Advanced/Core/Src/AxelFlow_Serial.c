#include "AxelFlow_Serial.h"
/*
 * Mainly takes the instruction packet as a structure, converts it into array,
 * and sends it to Serial Bus Servo via Half Duplex UART,
 * then receives the data as a array, then it converts the received data into structure
 * and returns it. This is so that the main code AxelFlow.c works only on structures, and 
 * to isolate the UART code for Microcontroller cross compabilities.
 */
// TODO Communication via Pointers
//#define DEBUG_PRINT_COMMUNICATION
uint8_t info_array[INSTRUCTION_FRAME_BUFFER];

void struct_to_arr(Instruction_Packet packet)
{
	info_array[0] = HEADER;
	info_array[1] = HEADER;
	info_array[2] = packet.Packet_ID;
	info_array[3] = packet.Length;
	info_array[4] = packet.Instruction;

	for (uint8_t i = 5; i < packet.Length + 3; i++)
		info_array[i] = packet.Param[i - 5];

	info_array[packet.Length + 3] = packet.Checksum;

	for (uint8_t i = packet.Length + 4; i <= INSTRUCTION_FRAME_BUFFER; i++)
	{
		info_array[i] = 0;
	}
}

Status_Packet arr_to_struct(uint8_t array[])
{
	Status_Packet packet;
	packet.Header_1 = array[0];
	packet.Header_2 = array[1];
	packet.Packet_ID = array[2];
	packet.Length = array[3];
	packet.Error = array[4];
	uint8_t prm[packet.Length - 2];
	for (uint8_t i = 5; i < packet.Length + 3; i++)
	{
		prm[i - 5] = array[i];
	}
	packet.Param = prm;
	packet.Checksum = array[packet.Length + 3];

	return packet;
}

Status_Packet AxelFlow_fire(UART_HandleTypeDef *huart, Instruction_Packet ip)
{
	HAL_StatusTypeDef err1, err2;
#ifndef DEBUG_PRINT_COMMUNICATION
	(void) err1, (void) err2; // silence warnings
#endif
	uint8_t Status_array[STATUS_FRAME_BUFFER];
	memset(Status_array, 0, STATUS_FRAME_BUFFER);
	struct_to_arr(ip);

	UART_HandleTypeDef huartx = *huart;

	err1 = HAL_UART_Transmit(&huartx, info_array, ip.Length + 4, HAL_MAX_DELAY);
	__HAL_UART_ENABLE_IT(&huartx, UART_IT_RXNE); // Enable receive interrupt after transmission
	err2 = HAL_UART_Receive(&huartx, Status_array, STATUS_FRAME_BUFFER,
	STATUS_PACKET_TIMEOUT);
	uint8_t Start_Index = 0;
	for (uint8_t i = 0; i < STATUS_FRAME_BUFFER - 1; i++)
	{
		if (Status_array[i] == 0xFF && Status_array[i + 1] == 0xFF
				&& Status_array[i + 2] != 0xFF)
		{
			Start_Index = i;
			break;
		}
	}
	uint8_t Status_array_filtered[Status_array[Start_Index + 3] + 4];

	for (uint8_t i = 0; i < sizeof(Status_array_filtered); i++)
	{
		Status_array_filtered[i] = Status_array[Start_Index + i];
	}
	Status_Packet packet = arr_to_struct(Status_array_filtered);
#ifdef DEBUG_PRINT_COMMUNICATION
	if (err1 != HAL_OK || (err2 != HAL_OK && err2 != HAL_TIMEOUT))
	{
		char temp[10];
		AxelFlow_debug_println("Communication Failed :(");
		sprintf(temp, "err1: %u", err1);
		AxelFlow_debug_println(temp);
		sprintf(temp, "err2: %u", err2);
		AxelFlow_debug_println(temp);
	}
#endif
	return packet;

	// TODO clean out received data.
}

// Take inputs such as uart handle, instruction packet
// returns success
