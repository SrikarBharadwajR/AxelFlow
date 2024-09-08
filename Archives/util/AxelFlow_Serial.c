#include "AxelFlow_Serial.h"
/*
 * Mainly takes the instruction packet as a structure, converts it into array,
 * and sends it to Serial Bus Servo via Half Duplex UART,
 * then receives the data as a array, then it converts the received data into structure
 * and returns it. This is so that the main code AxelFlow.c works only on structures, and 
 * to isolate the UART code for Microcontroller cross compabilities.
 */
// TODO Communication via Pointers

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

Status_Packet arr_to_struct(uint8_t array)
{
	Packet packet;
	packet.Header_1 = array[0];
	packet.Header_2 = array[1];
	packet.Packet_ID = array[2];
	packet.Length = array[3];
	packet.Info = array[4];

	for (uint8_t i = 5; i < packet.Length + 3; i++)
	{
		packet.param[i - 5] = array[i];
	}
	packet.checksum = array[packet.length + 3];

	return packet;
}

Status_Packet AxelFlow_fire(UART_HandleTypeDef *huart, Instruction_Packet ip)
{
	HAL_StatusTypeDef err1, err2;
	uint8_t Status_array[STATUS_FRAME_BUFFER];

	struct_to_arr(ip);

	UART_HandleTypeDef huartx = *huart;

	err1 = HAL_UART_Transmit(&huartx, info_array, sizeof(ip), HAL_MAX_DELAY);
	__HAL_UART_ENABLE_IT(&huartx, UART_IT_RXNE); // Enable receive interrupt after transmission
	err2 = HAL_UART_Receive(&huartx, Status_array, STATUS_FRAME_BUFFER,
	STATUS_PACKET_TIMEOUT);

	Status_Packet packet = arr_to_struct(Status_array);

	if (err1 == HAL_OK && err2 == HAL_OK)
	{
		return packet;
	}
	else
	{
		AxelFlow_debug_println("Communication Failed :(");
		return 0;
	}

	// TODO clean out received data.
}

// Take inputs such as uart handle, instruction packet
// returns success
