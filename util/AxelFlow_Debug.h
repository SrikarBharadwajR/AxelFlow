#ifndef INC_AXELFLOW_DEBUG_H_
#define INC_AXELFLOW_DEBUG_H_

#include "AxelFlow.h"
#include "stm32f0xx_hal.h"

void AxelFlow_debug_init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef AxelFlow_debug_print(char* str);
HAL_StatusTypeDef AxelFlow_debug_println(char* st);

#endif
