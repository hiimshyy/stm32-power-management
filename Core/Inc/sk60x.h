/*
 * sk60x.h
 *
 *  Created on: Jul 16, 2025
 *      Author: tiensy
 */




#ifndef __SK60X_H
#define __SK60X_H

#include "stm32f1xx_hal.h"
#include "debugger.h"
#include <stdint.h>
#include <stdbool.h>

#define START_ADDRESS 0x00
#define QUANTITY 0x13
#define SK60X_ADDR 0x01
#define READ_REGISTERS 0x03
#define WRITE_REGISTERS 0x06
#define REQUEST_FRAME_SIZE 8
#define RESPONSE_FRAME_SIZE 42
#define SK60X_MAX_VOLTAGE 3600
#define SK60X_MAX_CURRENT 500
#define SK60X_MIN_VOLTAGE 600
#define SK60X_MIN_CURRENT 0

extern UART_HandleTypeDef huart3;

typedef enum {
	SET_VOLTAGE = 0x01,
	SET_CURRENT = 0x02,
	SET_ON_OFF = 0x12,
} SK60X_Command;

typedef struct {
	uint16_t v_set;
	uint16_t i_set;
	uint16_t v_out;
	uint16_t i_out;
	uint16_t p_out;
	uint16_t v_in;
	uint16_t i_in;
	uint16_t lock;
	uint16_t temp;
	uint16_t h_use;
	uint16_t m_use;
	uint16_t s_use;
	uint16_t cvcc;
	uint16_t on_off;

} SK60X_Data;

extern SK60X_Data sk60x_data;

extern uint8_t _sk60_rx_buffer[RESPONSE_FRAME_SIZE];
extern uint8_t _sk60_tx_buffer[REQUEST_FRAME_SIZE];

bool SK60X_Read_Data();
bool SK60X_Fan_Control(bool enable);
bool SK60X_Set_On_Off(uint16_t  on_off);
bool SK60X_Set_Voltage(uint16_t voltage);
bool SK60X_Set_Current(uint16_t current);

#endif
