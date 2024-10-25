#ifndef AXELFLOW_SERIAL_H
#define AXELFLOW_SERIAL_H

//#include "AxelFlow.h"
#include "stm32f0xx_hal.h"
#include "AxelFlow_Debug.h"
//#include <stdio.h>
#define HEADER                          0xFF

#define STATUS_PACKET_TIMEOUT           50      // in millis()
#define STATUS_FRAME_BUFFER             10
#define INSTRUCTION_FRAME_BUFFER        15

typedef struct
{
	uint8_t Header_1;
	uint8_t Header_2;
	uint8_t Packet_ID;
	uint8_t Length;
	uint8_t Instruction;
	uint8_t *Param;
	uint8_t Checksum;
} Instruction_Packet;

typedef struct
{
	uint8_t Header_1;
	uint8_t Header_2;
	uint8_t Packet_ID;
	uint8_t Length;
	uint8_t Error;
	uint8_t *Param;
	uint8_t Checksum;
} Status_Packet;

UART_HandleTypeDef AxelFlow_UART_Init(USART_TypeDef* UART_ID, uint32_t baud_rate);
Status_Packet AxelFlow_fire(UART_HandleTypeDef *huart, Instruction_Packet ip);
Status_Packet arr_to_struct(uint8_t array[]);
void struct_to_arr(Instruction_Packet packet);
uint8_t getChecksum(Instruction_Packet packet);

#endif
