/*
 * TODO RM_Dynamixel_AX-12A.h
 *
 *  Created on: Apr 20, 2024
 *      Author: srikar
 */

#ifndef AXELFLOW_H
#define AXELFLOW_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "AxelFlow_Debug.h"
//#include <stm32f0xx_hal_uart.h>
//#include "stm32f0xx_hal.h"

//#########################################################################
//################ define - Dynamixel Hex code table ######################
// EEPROM AREA
#define EEPROM_MODEL_NUMBER_L           0x00
#define EEPROM_MODEL_NUMBER_H           0x01
#define EEPROM_VERSION                  0x02
#define EEPROM_ID                       0x03
#define EEPROM_BAUD_RATE                0x04
#define EEPROM_RETURN_DELAY_TIME        0x05
#define EEPROM_CW_ANGLE_LIMIT_L         0x06
#define EEPROM_CW_ANGLE_LIMIT_H         0x07
#define EEPROM_CCW_ANGLE_LIMIT_L        0x08
#define EEPROM_CCW_ANGLE_LIMIT_H        0x09
#define EEPROM_LIMIT_TEMPERATURE        0x0B
#define EEPROM_LOW_LIMIT_VOLTAGE        0x0C
#define EEPROM_HIGN_LIMIT_VOLTAGE       0x0D
#define EEPROM_MAX_TORQUE_L             0x0E
#define EEPROM_MAX_TORQUE_H             0x0F
#define EEPROM_RETURN_LEVEL             0x10
#define EEPROM_ALARM_LED                0x11
#define EEPROM_ALARM_SHUTDOWN           0x12
// RAM AREA
#define RAM_TORQUE_ENABLE               0x18
#define RAM_LED                         0x19
#define RAM_PROPORTIONAL_GAIN           0x1C
#define RAM_INTERGRAL_GAIN              0x1B
#define RAM_DERIVATIVE_GAIN             0x1A
#define RAM_GOAL_POSITION_L             0x1E
#define RAM_GOAL_POSITION_H             0x1F
#define RAM_GOAL_SPEED_L                0x20
#define RAM_GOAL_SPEED_H                0x21
#define RAM_TORQUE_LIMIT_L              0x22
#define RAM_TORQUE_LIMIT_H              0x23
#define RAM_PRESENT_POSITION_L          0x24
#define RAM_PRESENT_POSITION_H          0x25
#define RAM_PRESENT_SPEED_L             0x26
#define RAM_PRESENT_SPEED_H             0x27
#define RAM_PRESENT_LOAD_L              0x28
#define RAM_PRESENT_LOAD_H              0x29
#define RAM_PRESENT_VOLTAGE             0x2A
#define RAM_PRESENT_TEMPERATURE         0x2B
#define RAM_REGISTER                    0x2C
#define RAM_MOVING                      0x2E
#define RAM_LOCK                        0x2F
#define RAM_PUNCH_L                     0x30
#define RAM_PUNCH_H                     0x31

//#########################################################################
//################ Instruction commands Set ###############################
#define COMMAND_PING                    0x01
#define COMMAND_READ_DATA               0x02
#define COMMAND_WRITE_DATA              0x03
#define COMMAND_REG_WRITE_DATA          0x04
#define COMMAND_ACTION                  0x05
#define COMMAND_RESET                   0x06
#define COMMAND_SYNC_WRITE              0x83

//#########################################################################
//################ Instruction packet lengths #############################
// Packet length is number of parameters (N) + 2
#define READ_ONE_BYTE_LENGTH            0x01
#define READ_TWO_BYTE_LENGTH            0x02
#define RESET_LENGTH                    0x02
#define PING_LENGTH                     0x02
#define ACTION_LENGTH                   0x02
#define SET_ID_LENGTH                   0x04
#define SET_BD_LENGTH                   0x04
#define SET_RETURN_LEVEL_LENGTH         0x04
#define READ_TEMP_LENGTH                0x04
#define READ_POS_LENGTH                 0x04
#define READ_LOAD_LENGTH                0x04
#define READ_SPEED_LENGTH               0x04
#define READ_VOLT_LENGTH                0x04
#define READ_REGISTER_LENGTH            0x04
#define READ_MOVING_LENGTH              0x04
#define READ_LOCK_LENGTH                0x04
#define LED_LENGTH                      0x04
#define SET_HOLDING_TORQUE_LENGTH       0x04
#define SET_MAX_TORQUE_LENGTH           0x05
#define SET_ALARM_LENGTH                0x04
#define READ_LOAD_LENGTH                0x04
#define SET_RETURN_LENGTH               0x04
#define WHEEL_LENGTH                    0x05
#define SERVO_GOAL_LENGTH               0x07
#define SET_MODE_LENGTH                 0x07
#define SET_PUNCH_LENGTH                0x04
#define SET_PID_LENGTH                  0x06
#define SET_TEMP_LENGTH                 0x04
#define SET_VOLT_LENGTH                 0x05
#define SYNC_LOAD_LENGTH                0x0D
#define SYNC_DATA_LENGTH                0x02

//#########################################################################
//############################ Specials ###################################
#define PORT0                           0x00
#define PORT1                           0x01
#define PORT2                           0x02
#define PORT3                           0x03

#define OFF                             0x00
#define ON                              0x01

#define SERVO                           0x01
#define WHEEL                           0x00

#define LEFT                            0x00
#define RIGHT                           0x01

#define NONE                            0x00
#define READ                            0x01
#define ALL                             0x02

#define BROADCAST_ID                    0xFE

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

typedef enum
{
	Input_Voltage_Error = 0x01,
	Angle_Limit_Error,
	Overheating_Error,
	Range_Error,
	Checksum_Error,
	Overload_Error,
	Instruction_Error,
} Status_Error;
//typedef struct
//{
//	uint8_t Input_Voltage_Error = 0x00;
//	uint8_t Angle_Limit_Error = 0x01;
//	uint8_t Overheating_Error = 0x02;
//	uint8_t Range_Error = 0x03;
//	uint8_t Checksum_Error = 0x04;
//	uint8_t Overload_Error = 0x05;
//	uint8_t Instruction_Error = 0x06;
//} Status_Error;
void begin(long);
void end(void);

void setDirectionPin(unsigned char);
unsigned int reset(unsigned char);
unsigned int ping(unsigned char);

unsigned int setStatusPaketReturnDelay(unsigned char, unsigned char);
unsigned int setID(unsigned char, unsigned char);
unsigned int setBaudRate(unsigned char, long);
unsigned int setMaxTorque(unsigned char, int);
unsigned int setHoldingTorque(unsigned char, bool);
unsigned int setAlarmShutdown(unsigned char, unsigned char);
unsigned int setStatusPaket(unsigned char, unsigned char);
unsigned int setMode(unsigned char, bool, unsigned int, unsigned int);
unsigned int setPunch(unsigned char, unsigned int);
unsigned int setPID(unsigned char, unsigned char, unsigned char, unsigned char);
unsigned int setTemp(unsigned char, unsigned char);
unsigned int setVoltage(unsigned char, unsigned char, unsigned char);

unsigned int servo(unsigned char, unsigned int, unsigned int);
unsigned int servoPreload(unsigned char, unsigned int, unsigned int);
unsigned int wheel(unsigned char, bool, unsigned int);
void wheelSync(unsigned char, bool, unsigned int, unsigned char, bool,
		unsigned int, unsigned char, bool, unsigned int);
unsigned int wheelPreload(unsigned char, bool, unsigned int);

unsigned int action(unsigned char);

unsigned int readTemperature(unsigned char);
unsigned int readVoltage(unsigned char);
unsigned int readPosition(unsigned char);
unsigned int readLoad(unsigned char);
unsigned int readSpeed(unsigned char);

unsigned int checkRegister(unsigned char);
unsigned int checkMovement(unsigned char);
unsigned int checkLock(unsigned char);

unsigned int ledState(unsigned char, bool);

void transmitInstructionPacket(void);
unsigned int readStatusPacket(void);
void clearRXbuffer(void);
void print_status(Status_Packet packet);

#endif /* AXELFLOW_H */
