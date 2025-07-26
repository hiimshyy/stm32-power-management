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
#define RESPONSE_FRAME_SIZE 20
#define SK60X_MAX_VOLTAGE 36.0f
#define SK60X_MAX_CURRENT 5.0f
#define SK60X_MIN_VOLTAGE 6.0f
#define SK60X_MIN_CURRENT 0.0f

extern UART_HandleTypeDef huart3;

typedef enum {
	SET_VOLTAGE = 0x01,
	SET_CURRENT = 0x02,
	SET_ON_OFF = 0x12,
} SK60X_Command;

typedef struct {
    float v_set;
    float i_set;
    float v_out;
    float i_out;
    float p_out;
    float v_in;
    float i_in;
    float temp;
    uint8_t h_use;
    uint8_t m_use;
    uint8_t s_use;
    bool  status;
    bool  on_off;

} SK60X_Data;

extern SK60X_Data sk60x_data;

extern uint8_t _sk60_rx_buffer[RESPONSE_FRAME_SIZE];
extern uint8_t _sk60_tx_buffer[REQUEST_FRAME_SIZE];

bool SK60X_Read_Data();
bool SK60X_Fan_Control(bool enable);
void SK60X_Set_On_Off(bool on);
void SK60X_Set_Voltage(float voltage);
void SK60X_Set_Current(float current);

#endif
