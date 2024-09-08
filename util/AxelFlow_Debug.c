#include "AxelFlow_Debug.h"

UART_HandleTypeDef huartx;

void AxelFlow_debug_init(UART_HandleTypeDef *huart)
{
	huartx = *huart;
}

HAL_StatusTypeDef AxelFlow_debug_print(char *str)
{
	return HAL_UART_Transmit(&huartx, (uint8_t*) str, strlen(str),
	HAL_MAX_DELAY);
}

HAL_StatusTypeDef AxelFlow_debug_println(char *st)
{
	char str[200];
	sprintf(str, "%s\n\r", st);
	return HAL_UART_Transmit(&huartx, (uint8_t*) str, strlen(str),
	HAL_MAX_DELAY);
}
