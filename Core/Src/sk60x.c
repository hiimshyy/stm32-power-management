/*
 * sk60x.c
 *
 *  Created on: Jul 16, 2025
 *      Author: tiensy
 */




#include "sk60x.h"
#include <string.h>

SK60X_Data sk60x_data;
uint8_t _sk60_rx_buffer[RESPONSE_FRAME_SIZE];
uint8_t _sk60_tx_buffer[REQUEST_FRAME_SIZE];

static uint16_t Calculate_CRC(uint8_t *buf, uint8_t len)
{
    uint16_t crc = 0xFFFF;
    for (int pos = 0; pos < len; pos++) {
        crc ^= buf[pos];
        for (int i = 0; i < 8; i++) {
            if (crc & 1) crc = (crc >> 1) ^ 0xA001;
            else crc >>= 1;
        }
    }
    return crc;
}

static bool SK60X_Send_Command(uint8_t command, uint16_t value)
{
	memset(_sk60_tx_buffer, 0x00, REQUEST_FRAME_SIZE);
	_sk60_tx_buffer[0] = SK60X_ADDR;        // Device address
	_sk60_tx_buffer[1] = WRITE_REGISTERS;   // Function code
	_sk60_tx_buffer[3] = command;           // Command code
	_sk60_tx_buffer[5] = value;     		// Value low byte

	uint16_t crc = Calculate_CRC(_sk60_tx_buffer, 6);
	_sk60_tx_buffer[6] = crc & 0xFF;
	_sk60_tx_buffer[7] = (crc >> 8) & 0xFF;

	return (HAL_UART_Transmit(&huart3, _sk60_tx_buffer, REQUEST_FRAME_SIZE, 100) == HAL_OK);
}

bool SK60X_Read_Data()
{
	memset(_sk60_rx_buffer, 0x00, RESPONSE_FRAME_SIZE);
	memset(_sk60_tx_buffer, 0x00, REQUEST_FRAME_SIZE);
	_sk60_tx_buffer[0] = SK60X_ADDR;        // Device address
	_sk60_tx_buffer[1] = READ_REGISTERS;    // Function code: Read Holding Registers
	_sk60_tx_buffer[2] = START_ADDRESS >> 8;
	_sk60_tx_buffer[3] = START_ADDRESS & 0xFF;
	_sk60_tx_buffer[4] = (QUANTITY >> 8) & 0xFF; // Number of registers to read (high byte)
	_sk60_tx_buffer[5] = QUANTITY & 0xFF;        // Number of registers to read (low byte)


    uint16_t crc = Calculate_CRC(_sk60_tx_buffer, 6);
    _sk60_tx_buffer[6] = crc & 0xFF;
    _sk60_tx_buffer[7] = crc >> 8;
//    Debug_Printf("<SK60x> - Sending request:\n");
//    for (size_t i = 0; i < REQUEST_FRAME_SIZE; i++)
//    	Debug_Printf(" %02X", _sk60_tx_buffer[i]);
//    Debug_Printf("\n");

    if(HAL_UART_Transmit(&huart3, _sk60_tx_buffer, REQUEST_FRAME_SIZE, 100) != HAL_OK)
    	return false;
    if(HAL_UART_Receive(&huart3, _sk60_rx_buffer, sizeof(_sk60_rx_buffer), 500) != HAL_OK)
    	return false;
//    Debug_Printf("<SK60x> - Received:\n");
//    for (size_t i = 0; i < RESPONSE_FRAME_SIZE; i++)
//	{
//		Debug_Printf(" %02X", _sk60_rx_buffer[i]);
//	}
//    Debug_Printf("\n");

    sk60x_data.v_set = (_sk60_rx_buffer[4] << 8) | _sk60_rx_buffer[5];
    sk60x_data.i_set = (_sk60_rx_buffer[6] << 8) | _sk60_rx_buffer[7];
    sk60x_data.v_out = (_sk60_rx_buffer[8] << 8) | _sk60_rx_buffer[9];
    sk60x_data.i_out = (_sk60_rx_buffer[10] << 8) | _sk60_rx_buffer[11];
    sk60x_data.p_out = (_sk60_rx_buffer[12] << 8) | _sk60_rx_buffer[13];
    sk60x_data.v_in  = (_sk60_rx_buffer[14] << 8) | _sk60_rx_buffer[15];
    sk60x_data.i_in  = (_sk60_rx_buffer[16] << 8) | _sk60_rx_buffer[17];

    sk60x_data.h_use  = (_sk60_rx_buffer[24] << 8) | _sk60_rx_buffer[25];
    sk60x_data.m_use  = (_sk60_rx_buffer[26] << 8) | _sk60_rx_buffer[27];
    sk60x_data.s_use  = (_sk60_rx_buffer[28] << 8) | _sk60_rx_buffer[29];
    sk60x_data.temp   = (_sk60_rx_buffer[30] << 8) | _sk60_rx_buffer[31];
    sk60x_data.lock_v = (_sk60_rx_buffer[34] << 8) | _sk60_rx_buffer[35];
    sk60x_data.status = (_sk60_rx_buffer[38] << 8) | _sk60_rx_buffer[39];
    sk60x_data.on_off = (_sk60_rx_buffer[40] << 8) | _sk60_rx_buffer[41];

    return true;
}

bool SK60X_Check_Connection()
{
	memset(_sk60_tx_buffer, 0x00, REQUEST_FRAME_SIZE);
	_sk60_tx_buffer[0] = SK60X_ADDR;        // Device address
	_sk60_tx_buffer[1] = READ_REGISTERS;    // Function code: Read Holding Registers
	_sk60_tx_buffer[2] = START_ADDRESS >> 8;
	_sk60_tx_buffer[3] = START_ADDRESS & 0xFF;
	_sk60_tx_buffer[4] = (QUANTITY >> 8) & 0xFF; // Number of registers to read (high byte)
	_sk60_tx_buffer[5] = QUANTITY & 0xFF;        // Number of registers to read (low byte)

	uint16_t crc = Calculate_CRC(_sk60_tx_buffer, 6);
	_sk60_tx_buffer[6] = crc & 0xFF;
	_sk60_tx_buffer[7] = crc >> 8;

	if(HAL_UART_Transmit(&huart3, _sk60_tx_buffer, REQUEST_FRAME_SIZE, 100) != HAL_OK)
	{
		Debug_Printf("SK60X Connection Check failed to send request\n");
		return false; // Failed to send request
	}

	if(HAL_UART_Receive(&huart3, _sk60_rx_buffer, RESPONSE_FRAME_SIZE, 500) != HAL_OK)
	{
		Debug_Printf("SK60X Connection Check failed to receive response\n");
		return false; // Failed to receive response
	}

	return true; // Connection is valid if we received a response
}

void SK60X_Set_On_Off(bool on)
{
	uint16_t value = on ? 0x0001 : 0x0000; // Convert boolean to register value
	if(SK60X_Send_Command(SET_ON_OFF, value))
	{
		Debug_Printf("SK60X Set On/Off command sent successfully\n");
	}
	else
	{
		Debug_Printf("SK60X Set On/Off command failed\n");
		return; // Command failed
	}
}

void SK60X_Set_Voltage(float voltage)
{
	if (voltage < SK60X_MIN_VOLTAGE || voltage > SK60X_MAX_VOLTAGE) {
		Debug_Printf("Invalid voltage value: %f\n", voltage);
		return; // Invalid voltage value
	}

	uint16_t value = (uint16_t)(voltage * 100); // Convert to integer representation
	if(SK60X_Send_Command(SET_VOLTAGE, value))
	{
		Debug_Printf("SK60X Set Voltage command sent successfully\n");
	}
	else
	{
		Debug_Printf("SK60X Set Voltage command failed\n");
	}
}

void SK60X_Set_Current(float current)
{
	if (current < SK60X_MIN_CURRENT || current > SK60X_MAX_CURRENT) {
		Debug_Printf("Invalid current value: %f\n", current);
		return; // Invalid current value
	}

	uint16_t value = (uint16_t)(current * 100); // Convert to integer representation
	if(SK60X_Send_Command(SET_CURRENT, value))
	{
		Debug_Printf("SK60X Set Current command sent successfully\n");
	}
	else
	{
		Debug_Printf("SK60X Set Current command failed\n");
	}
}
