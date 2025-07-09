/*
 * daly_bms.c
 *
 *  Created on: Jul 9, 2025
 *      Author: tiensy
 */


#include "daly_bms.h"
#include <string.h>

bool DalyBMS_Request(UART_HandleTypeDef *huart, DalyBMS_Command command)
{
	// Prepare the transmit buffer
    memset(txBuffer, 0x00, FRAME_SIZE);

    uint8_t txChecksum = 0x00;    // transmit checksum buffer
    // Frame structure: [START_BYTE, HOST_ADDRESS, COMMAND, FRAME_LENGTH, DATA..., CHECKSUM]
    txBuffer[0] = START_BYTE;
    txBuffer[1] = HOST_ADDRESS;
    txBuffer[2] = command;
    txBuffer[3] = FRAME_LENGTH;

    // Add checksum for the first 4 bytes
    for (int i = 0; i < 11; i++)
	{
    	txChecksum += txBuffer[i];
	}
    txBuffer[12] = (uint8_t)(txChecksum & 0xFF); // checksum

    return (HAL_UART_Transmit(huart, txBuffer, FRAME_SIZE, 100) == HAL_OK);
}

bool DalyBMS_Recive(UART_HandleTypeDef *huart, unsigned int frameAmount)
{
	// Clear the buffer before receiving new data
	memset(rxFrameBuffer, 0x00, sizeof(rxFrameBuffer));
	memset(frameBuff, 0x00, sizeof(frameBuff));

	unsigned int byteCounter = 0;

	// Receive data from UART
	if (HAL_UART_Receive(huart, rxFrameBuffer, FRAME_SIZE*frameAmount, 100) == HAL_OK)
	{
		for(size_t i = 0; i < frameAmount; i++)
		{
			for(size_t j = 0; j < FRAME_SIZE; j++)
			{
				frameBuff[i][j] = rxFrameBuffer[byteCounter++];
			}

			uint8_t rxChecksum = 0x00;  // checksum for the current frame
			// Calculate checksum for the received frame
			for (int k = 0; k < FRAME_SIZE - 1; k++)
			{
				rxChecksum += frameBuff[i][k];
			}

			if (rxChecksum != frameBuff[i][FRAME_SIZE -1])
			{
				//debug_printf("crc error");
				return false;  // Checksum mismatch
			}
			if(rxChecksum == 0x00)
			{
				//debug_printf("no data");
				return false;  // Invalid frame
			}
			if(frameBuff[i][1] >= 0x20)
			{
				//debug_printf("bms sleep");
				return false;  // Invalid frame, address should be 0x40
			}
		}
	}

	return false;  // Invalid frame or receive error
}

bool DalyBMS_Parse(DalyBMS_Data *data, uint8_t *frame)
{
    if (frame[0] != 0xAA || frame[1] != 0x55)
        return false;

    uint8_t cmd = frame[6];
    switch (cmd)
    {
        case 0x03:  // Total voltage, current, SOC
            data->voltage_mv = (frame[8] << 8) | frame[9];
            data->current_ma = ((frame[10] << 8) | frame[11]) - 30000;  // offset
            data->soc_percent = frame[12];
            break;

        case 0x04:  // Cell voltages
            data->num_cells = frame[7];
            for (int i = 0; i < data->num_cells; i++)
            {
                data->cell_voltage_mv[i] = (frame[8 + 2*i] << 8) | frame[9 + 2*i];
            }
            break;

        case 0x05:  // Temperatures
            data->num_temp_sensors = frame[7];
            for (int i = 0; i < data->num_temp_sensors; i++)
            {
                uint16_t raw = (frame[8 + 2*i] << 8) | frame[9 + 2*i];
                data->temperature_c[i] = (float)(raw - 2731) / 10.0f;
            }
            break;

        case 0x06:  // Errors
            data->fault_flags = (frame[8] << 8) | frame[9];
            break;

        case 0xE1:  // MOSFET state
            data->charge_mos = frame[8] & 0x01;
            data->discharge_mos = frame[8] & 0x02;
            break;

        default:
            return false;
    }

    return true;
}


