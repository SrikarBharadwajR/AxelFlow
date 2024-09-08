#ifndef AXELFLOW_SERIAL_H
#define AXELFLOW_SERIAL_H

#include "AxelFlow.h"
#include "stm32f0xx_hal.h"
#include "AxelFlow_Debug.h"
//typedef struct
//{
//	uint8_t Header_1;
//	uint8_t Header_2;
//	uint8_t Packet_ID;
//	uint8_t Length;
//	uint8_t Info;
//	uint8_t *Param;
//	uint8_t Checksum;
//} Packet;

Status_Packet AxelFlow_fire(UART_HandleTypeDef *huart,
		Instruction_Packet ip);
Status_Packet arr_to_struct(uint8_t array[]);
void struct_to_arr(Instruction_Packet packet);

#endif
