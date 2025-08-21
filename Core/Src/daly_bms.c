/*
 * daly_bms.c
 *
 *  Created on: Jul 9, 2025
 *      Author: tiensy
 */


#include "daly_bms.h"
#include <string.h>

DalyBMS_Callback_t _bms_request_callback = NULL;

uint8_t _tx_buffer[FRAME_SIZE];
uint8_t _frame_buff[4][FRAME_SIZE];
uint8_t _rx_frame_buffer[FRAME_SIZE*4];
uint8_t _frame_count = 0;
uint8_t _error_counter = 0;
uint8_t _request_counter = 0;
bool 	_get_static_data = false;

DalyBMS_Data bms_data;


bool DalyBMS_Request(DalyBMS_Command command)
{
	// Prepare the transmit buffer
    memset(_tx_buffer, 0x00, FRAME_SIZE);

    uint8_t _tx_checksum = 0x00;    // transmit checksum buffer
    // Frame structure: [START_BYTE, HOST_ADDRESS, COMMAND, FRAME_LENGTH, DATA..., CHECKSUM]
    _tx_buffer[0] = START_BYTE;
    _tx_buffer[1] = HOST_ADDRESS;
    _tx_buffer[2] = command;
    _tx_buffer[3] = FRAME_LENGTH;

    // Add checksum for the first 4 bytes
    for (int i = 0; i < 11; i++) _tx_checksum += _tx_buffer[i];
    _tx_buffer[12] = (uint8_t)(_tx_checksum & 0xFF); // checksum

    // Debug start
//    Debug_Printf("<BMS> - Request:");
//    for (int i = 0; i < FRAME_SIZE; i++)
//	{
//		Debug_Printf(" %02X", _tx_buffer[i]);
//	}
//    Debug_Printf("\n");
    // Debug end

    return (HAL_UART_Transmit(&huart1, _tx_buffer, FRAME_SIZE, 100) == HAL_OK);
}

bool DalyBMS_Recive(uint8_t _frame_amount)
{
	// Clear the buffer before receiving new data
	memset(_rx_frame_buffer, 0x00, sizeof(_rx_frame_buffer));
	memset(_frame_buff, 0x00, sizeof(_frame_buff));

	size_t _total_bytes = _frame_amount * FRAME_SIZE;
	// Receive data into the buffer
	if (HAL_UART_Receive(&huart1, _rx_frame_buffer, _total_bytes, 200) != HAL_OK)
	{
		//Debug_Printf("BMS receive error\n");
		return false;  // Receive error
	}
//	else
//	{
//		Debug_Printf("<BMS> - Received:");
//		for (size_t i = 0; i < _total_bytes; i++)
//		{
//			Debug_Printf(" %02X", _rx_frame_buffer[i]);
//			if ((i + 1) % FRAME_SIZE == 0) Debug_Printf("\n");
//		}
//	}

	for (size_t i = 0; i < _frame_amount; i++)
	{
		memcpy(_frame_buff[i], &_rx_frame_buffer[i * FRAME_SIZE], FRAME_SIZE);

		uint8_t _rx_checksum = 0x00;
		for (size_t j = 0; j < FRAME_SIZE - 1; j++)	_rx_checksum += _frame_buff[i][j];

		if (_rx_checksum != _frame_buff[i][FRAME_SIZE - 1])
		{
			//Debug_Printf("BMS checksum error in frame %d\n", i);
			//Debug_Printf("Expected: %02X, Received: %02X\n", _rx_checksum, _frame_buff[i][FRAME_SIZE - 1]);
			return false;  // Checksum error
		}

		if (_rx_checksum == 0x00 && _frame_buff[i][0] == 0x20);	//Debug_Printf("BMS sleep or invalid frame\n");
	}

	return true;  // Invalid frame or receive error
}

bool DalyBMS_Get_Pack_Data()
{
	if (DalyBMS_Request(VOUT_IOUT_SOC))
	{
		if (DalyBMS_Recive(1))
		{
			if((((float)(((_frame_buff[0][8] << 8) | _frame_buff[0][9]) - 30000) / 10.0f) == -3000.f))
			{
				//Debug_Printf("Current out of range, retrying...\n");
				return false;  // Current out of range
			}
			else if((((float)(((_frame_buff[0][10] << 8) | _frame_buff[0][11])) / 10.0f) > 100.f))
			{
				//Debug_Printf("SOC out of range, retrying...\n");
				return false;  // SOC out of range
			}
			else
			{
				bms_data.voltage = ((_frame_buff[0][4] << 8) | _frame_buff[0][5]);
				bms_data.current = ((_frame_buff[0][8] << 8) | _frame_buff[0][9]) - 30000;
				bms_data.soc = ((_frame_buff[0][10] << 8) | _frame_buff[0][11]);
				return true; 
			}
		}
		else
		{
			//Debug_Printf("BMS receive pack data failed\n");
			return false;  // Receive error
		}
	}
	else
	{
		//Debug_Printf("BMS request pack data failed\n");
		return false;  // Request failed
	}
	return true;
}

bool DalyBMS_Get_Min_Max_Cell_Voltage()
{
	if(DalyBMS_Request(MIN_MAX_CELL_VOLTAGE))
	{
		if(DalyBMS_Recive(1))
		{
			bms_data.max_cell = ((_frame_buff[0][4] << 8) | _frame_buff[0][5]);
			bms_data.max_cell_index = _frame_buff[0][6];
			bms_data.min_cell = ((_frame_buff[0][7] << 8) | _frame_buff[0][8]);
			bms_data.min_cell_index = _frame_buff[0][9];
			bms_data.cell_diff = bms_data.max_cell - bms_data.min_cell;
		}
		else
		{
			//Debug_Printf("BMS receive min/max cell voltage failed\n");
			return false;  // Receive error
		}
	}
	else
	{
		//Debug_Printf("BMS request min/max cell voltage failed\n");
		return false;  // Request failed
	}

	return true;
}

bool DalyBMS_Get_Pack_Temperature()
{
	if (DalyBMS_Request(MIN_MAX_TEMPERATURE))
	{
		if (DalyBMS_Recive(1))
		{
//			Debug_Printf("BMS received min/max temperature successfully\n");
//			bms_data.max_temperature_c = (_frame_buff[0][4] - 40);
//			bms_data.min_temperature_c = (_frame_buff[0][6] - 40);
			bms_data.temperature_avr = ((_frame_buff[0][4] - 40) + (_frame_buff[0][6] - 40)) / 2;
			return true;
		}
		else
		{
			//Debug_Printf("BMS receive min/max temperature failed\n");
			return false;  // Receive error
		}
	}
	else
	{
		//Debug_Printf("BMS request min/max temperature failed\n");
		return false;  // Request failed
	}
	return true;
}

bool DalyBMS_Get_Charge_Discharge_Status()
{
	if (DalyBMS_Request(DISCHARGE_CHARGE_MOS_STATUS))
	{
		if (DalyBMS_Recive(1))
		{
//			Debug_Printf("BMS received charge/discharge MOS status successfully\n");
			switch (_frame_buff[0][4])
			{
				case 0:
					bms_data.charge_discharge_status = STATIONARY;
					break;
				case 1:
					bms_data.charge_discharge_status = CHARGE;
					break;
				case 2:
					bms_data.charge_discharge_status = DISCHARGE;
					break;
			}

			bms_data.charge_mos = _frame_buff[0][5];
			bms_data.discharge_mos = _frame_buff[0][6];
			bms_data.bms_life_cycle = _frame_buff[0][7];
			bms_data.residual_capacity_mAh = (((uint8_t)_frame_buff[0][8] << 0x18) | ((uint8_t)_frame_buff[0][9] << 0x10) | ((uint8_t)_frame_buff[0][10] << 0x08) | (uint8_t)_frame_buff[0][11]);
			return true;
		}
		else
		{
			//Debug_Printf("BMS receive charge/discharge MOS status failed\n");
			return false;  // Receive error
		}
	}
	else
	{
		//Debug_Printf("BMS request charge/discharge MOS status failed\n");
		return false;  // Request failed
	}

	return true;
}

bool DalyBMS_Get_Status_Info()
{
	if (DalyBMS_Request(STATUS_INFO))
	{
		if (DalyBMS_Recive(1))
		{
//			Debug_Printf("BMS received status info successfully\n");
			bms_data.num_cells = _frame_buff[0][4];
			bms_data.num_temp_sensors = _frame_buff[0][5];
			bms_data.charge_status = _frame_buff[0][6];
			bms_data.discharge_status = _frame_buff[0][7];
			bms_data.charge_discharge_cycle = ((uint16_t)_frame_buff[0][9] << 0x08) | (uint16_t)_frame_buff[0][10];
			return true;  // Request successful
		}
		else
		{
			//Debug_Printf("BMS receive status info failed\n");
			return false;  // Receive error
		}
	}
	else
	{
		//Debug_Printf("BMS request status info failed\n");
		return false;  // Request failed
	}
	return true;
}

bool DalyBMS_Get_Cell_Voltages()
{
	if (bms_data.num_cells < MIN_CELLS || bms_data.num_cells > MAX_CELLS)
	{
		//Debug_Printf("Invalid cell count: %d\n", bms_data.num_cells);
		return false;  // Invalid cell count
	}
	if (DalyBMS_Request(CELL_VOLTAGES))
	{
		uint8_t _cell_num = 0;
		uint8_t _frame_count = (bms_data.num_cells +2 )/3; // 1 frame for 1-3 cells, 2 frames for 4-6 cells, etc.
		if (DalyBMS_Recive(_frame_count))
		{
//			Debug_Printf("BMS received cell voltage successfully\n");

			for (uint8_t k = 0; k < _frame_count; k++)
			{
				for (uint8_t i = 0; i < 3; i++)
				{
					if (_cell_num >= bms_data.num_cells) break;

				  	uint16_t cell_mV = (_frame_buff[k][5 + 2*i] << 8) | _frame_buff[k][6 + 2*i];
				  	bms_data.cell_voltage_mv[_cell_num++] = cell_mV;
				}
			}
		}
		else
		{
			//Debug_Printf("BMS request cell voltage failed\n");
			return false;  // Request failed
		}
	}
	return true;
}

bool DalyBMS_Get_Cell_Temperatures()
{
	if (bms_data.num_temp_sensors < MIN_TEMP_SENSORS || bms_data.num_temp_sensors > MAX_TEMP_SENSORS)
	{
		//Debug_Printf("Invalid temperature sensor count: %d\n", bms_data.num_temp_sensors);
		return false;  // Invalid temperature sensor count
	}
	if (DalyBMS_Request(CELL_TEMPERATURE))
	{
		if (DalyBMS_Recive(1))
		{
//			Debug_Printf("BMS received cell temperatures successfully\n");

			for (uint8_t i = 0; i < bms_data.num_temp_sensors; i++)
			{
				bms_data.temperature_c[i] = (_frame_buff[0][5 + i] - 40);
			}
			return true;  // Request successful
		}
		else
		{
			//Debug_Printf("BMS receive cell temperatures failed\n");
			return false;  // Receive error
		}
	}
	else
	{
		//Debug_Printf("BMS request cell temperatures failed\n");
		return false;  // Request failed
	}

	return true;
}

bool DalyBMS_Get_Cell_Balance_State()
{
	if (bms_data.num_cells < MIN_CELLS || bms_data.num_cells > MAX_CELLS)
	{
		//Debug_Printf("Invalid cell count: %d\n", bms_data.num_cells);
		return false;  // Invalid cell count
	}

	if (DalyBMS_Request(CELL_BALANCE_STATE))
	{
		if (DalyBMS_Recive(1))
		{
//			Debug_Printf("BMS received cell balance state successfully\n");
			uint8_t _cell_bit = 0;
			uint8_t _balance_frame_count = 0;

			for (uint8_t i = 0; i < 6; i++)
			{
				uint8_t _byte = _frame_buff[0][4 + i];
				for (uint8_t j = 0; j < 8; j++)
				{
					if (_cell_bit >= bms_data.num_cells) break;
					uint8_t bit = (_byte >> j) & 0x01;
					bms_data.cell_balance_state[_cell_bit++] = bit;
					if (bit) _balance_frame_count++;
				}
			}
			bms_data.cell_balance_active = (_balance_frame_count > 0) ? true : false;
			return true;  // Request successful
		}
		else
		{
			//Debug_Printf("BMS receive cell balance state failed\n");
			return false;  // Receive error
		}
	}
	else
	{
		//Debug_Printf("BMS request cell balance state failed\n");
		return false;  // Request failed
	}

	return true;
}

bool DalyBMS_Get_Failure_Codes()
{
    if (DalyBMS_Request(FAILURE_CODES))
    {
        if (DalyBMS_Recive(1))
        {
            bms_data.fault_flags = 0; // clear trước

            // byte 0x00
            if ((_frame_buff[0][4] >> 1) & 0x01) bms_data.fault_flags |= BMS_ERR_CELL_VOLT_HIGH_LVL2;
            else if ((_frame_buff[0][4] >> 0) & 0x01) bms_data.fault_flags |= BMS_ERR_CELL_VOLT_HIGH_LVL1;
            if ((_frame_buff[0][4] >> 3) & 0x01) bms_data.fault_flags |= BMS_ERR_CELL_VOLT_LOW_LVL2;
            else if ((_frame_buff[0][4] >> 2) & 0x01) bms_data.fault_flags |= BMS_ERR_CELL_VOLT_LOW_LVL1;
            if ((_frame_buff[0][4] >> 5) & 0x01) bms_data.fault_flags |= BMS_ERR_SUM_VOLT_HIGH_LVL2;
            else if ((_frame_buff[0][4] >> 4) & 0x01) bms_data.fault_flags |= BMS_ERR_SUM_VOLT_HIGH_LVL1;
            if ((_frame_buff[0][4] >> 7) & 0x01) bms_data.fault_flags |= BMS_ERR_SUM_VOLT_LOW_LVL2;
            else if ((_frame_buff[0][4] >> 6) & 0x01) bms_data.fault_flags |= BMS_ERR_SUM_VOLT_LOW_LVL1;

            // byte 0x01
            if ((_frame_buff[0][5] >> 1) & 0x01) bms_data.fault_flags |= BMS_ERR_CHG_TEMP_HIGH_LVL2;
            else if ((_frame_buff[0][5] >> 0) & 0x01) bms_data.fault_flags |= BMS_ERR_CHG_TEMP_HIGH_LVL1;
            if ((_frame_buff[0][5] >> 3) & 0x01) bms_data.fault_flags |= BMS_ERR_CHG_TEMP_LOW_LVL2;
            else if ((_frame_buff[0][5] >> 2) & 0x01) bms_data.fault_flags |= BMS_ERR_CHG_TEMP_LOW_LVL1;
            if ((_frame_buff[0][5] >> 5) & 0x01) bms_data.fault_flags |= BMS_ERR_DISCHG_TEMP_HIGH_LVL2;
            else if ((_frame_buff[0][5] >> 4) & 0x01) bms_data.fault_flags |= BMS_ERR_DISCHG_TEMP_HIGH_LVL1;
            if ((_frame_buff[0][5] >> 7) & 0x01) bms_data.fault_flags |= BMS_ERR_DISCHG_TEMP_LOW_LVL2;
            else if ((_frame_buff[0][5] >> 6) & 0x01) bms_data.fault_flags |= BMS_ERR_DISCHG_TEMP_LOW_LVL1;

            // byte 0x02
            if ((_frame_buff[0][6] >> 1) & 0x01) bms_data.fault_flags |= BMS_ERR_CHG_OVERCURRENT_LVL2;
            else if ((_frame_buff[0][6] >> 0) & 0x01) bms_data.fault_flags |= BMS_ERR_CHG_OVERCURRENT_LVL1;
            if ((_frame_buff[0][6] >> 3) & 0x01) bms_data.fault_flags |= BMS_ERR_DISCHG_OVERCURRENT_LVL2;
            else if ((_frame_buff[0][6] >> 2) & 0x01) bms_data.fault_flags |= BMS_ERR_DISCHG_OVERCURRENT_LVL1;
            if ((_frame_buff[0][6] >> 5) & 0x01) bms_data.fault_flags |= BMS_ERR_SOC_HIGH_LVL2;
            else if ((_frame_buff[0][6] >> 4) & 0x01) bms_data.fault_flags |= BMS_ERR_SOC_HIGH_LVL1;
            if ((_frame_buff[0][6] >> 7) & 0x01) bms_data.fault_flags |= BMS_ERR_SOC_LOW_LVL2;
            else if ((_frame_buff[0][6] >> 6) & 0x01) bms_data.fault_flags |= BMS_ERR_SOC_LOW_LVL1;

            // byte 0x03
            if ((_frame_buff[0][7] >> 1) & 0x01) bms_data.fault_flags |= BMS_ERR_DIFF_VOLT_LVL2;
            else if ((_frame_buff[0][7] >> 0) & 0x01) bms_data.fault_flags |= BMS_ERR_DIFF_VOLT_LVL1;
            if ((_frame_buff[0][7] >> 3) & 0x01) bms_data.fault_flags |= BMS_ERR_DIFF_TEMP_LVL2;
            else if ((_frame_buff[0][7] >> 2) & 0x01) bms_data.fault_flags |= BMS_ERR_DIFF_TEMP_LVL1;

            // byte 0x04
            if ((_frame_buff[0][8] >> 0) & 0x01) bms_data.fault_flags |= BMS_ERR_CHG_MOS_TEMP_HIGH;
            if ((_frame_buff[0][8] >> 1) & 0x01) bms_data.fault_flags |= BMS_ERR_DISCHG_MOS_TEMP_HIGH;
            if ((_frame_buff[0][8] >> 2) & 0x01) bms_data.fault_flags |= BMS_ERR_CHG_MOS_TEMP_SENSOR;
            if ((_frame_buff[0][8] >> 3) & 0x01) bms_data.fault_flags |= BMS_ERR_DISCHG_MOS_TEMP_SENSOR;
            if ((_frame_buff[0][8] >> 4) & 0x01) bms_data.fault_flags |= BMS_ERR_CHG_MOS_ADHESION;
            if ((_frame_buff[0][8] >> 5) & 0x01) bms_data.fault_flags |= BMS_ERR_DISCHG_MOS_ADHESION;
            if ((_frame_buff[0][8] >> 6) & 0x01) bms_data.fault_flags |= BMS_ERR_CHG_MOS_OPEN_CIRCUIT;
            if ((_frame_buff[0][8] >> 7) & 0x01) bms_data.fault_flags |= BMS_ERR_DISCHG_MOS_OPEN_CIRCUIT;

            // byte 0x05
            if ((_frame_buff[0][9] >> 0) & 0x01) bms_data.fault_flags |= BMS_ERR_AFE_CHIP;
            if ((_frame_buff[0][9] >> 1) & 0x01) bms_data.fault_flags |= BMS_ERR_VOLT_COLLECT_DROPPED;
            if ((_frame_buff[0][9] >> 2) & 0x01) bms_data.fault_flags |= BMS_ERR_CELL_TEMP_SENSOR;
            if ((_frame_buff[0][9] >> 3) & 0x01) bms_data.fault_flags |= BMS_ERR_EEPROM;
            if ((_frame_buff[0][9] >> 4) & 0x01) bms_data.fault_flags |= BMS_ERR_RTC;
            if ((_frame_buff[0][9] >> 5) & 0x01) bms_data.fault_flags |= BMS_ERR_PRECHARGE_FAIL;
            if ((_frame_buff[0][9] >> 6) & 0x01) bms_data.fault_flags |= BMS_ERR_COMM_FAIL;
            if ((_frame_buff[0][9] >> 7) & 0x01) bms_data.fault_flags |= BMS_ERR_INTERNAL_COMM_FAIL;

            // byte 0x06
            if ((_frame_buff[0][10] >> 0) & 0x01) bms_data.fault_flags |= BMS_ERR_CURRENT_MODULE;
            if ((_frame_buff[0][10] >> 1) & 0x01) bms_data.fault_flags |= BMS_ERR_SUM_VOLT_DETECT;
            if ((_frame_buff[0][10] >> 2) & 0x01) bms_data.fault_flags |= BMS_ERR_SHORT_CIRCUIT;
            if ((_frame_buff[0][10] >> 3) & 0x01) bms_data.fault_flags |= BMS_ERR_LOW_VOLT_FORBID_CHG;

            return true;
        }
    }
    return false;
}


bool DalyBMS_Get_Voltage_Thresholds()
{
	if (DalyBMS_Request(CELL_THRESHOLDS))
	{
		if (DalyBMS_Recive(1))
		{
//			Debug_Printf("BMS received cell thresholds successfully\n");
			bms_data.max_cell_threshold_1 = (_frame_buff[0][4] << 8) | _frame_buff[0][5];
			bms_data.max_cell_threshold_2 = (_frame_buff[0][6] << 8) | _frame_buff[0][7];
			bms_data.min_cell_threshold_1 = (_frame_buff[0][8] << 8) | _frame_buff[0][9];
			bms_data.min_cell_threshold_2 = (_frame_buff[0][10] << 8) | _frame_buff[0][11];
			return true;  // Request successful
		}
		else
		{
			//Debug_Printf("BMS receive cell thresholds failed\n");
			return false;  // Receive error
		}
	}
	else
	{
		//Debug_Printf("BMS request cell thresholds failed\n");
		return false;  // Request failed
	}

	return true;
}

bool DalyBMS_Get_Pack_Thresholds()
{
	if (DalyBMS_Request(PACK_THRESHOLDS))
	{
		if (DalyBMS_Recive(1))
		{
//			Debug_Printf("BMS received pack thresholds successfully\n");
			bms_data.max_pack_threshold_1 = (_frame_buff[0][4] << 8) | _frame_buff[0][5];
			bms_data.max_pack_threshold_2 = (_frame_buff[0][6] << 8) | _frame_buff[0][7];
			bms_data.min_pack_threshold_1 = (_frame_buff[0][8] << 8) | _frame_buff[0][9];
			bms_data.min_pack_threshold_2 = (_frame_buff[0][10] << 8) | _frame_buff[0][11];
			return true;  // Request successful
		}
		else
		{
			//Debug_Printf("BMS receive pack thresholds failed\n");
			return false;  // Receive error
		}
	}
	else
	{
		//Debug_Printf("BMS request pack thresholds failed\n");
		return false;  // Request failed
	}

	return true;
}

bool DalyBMS_Get_Connection_Status()
{
	return bms_data.connection_status;
}

// 0xD9 0x80 First Byte 0x01=ON 0x00=OFF
bool DalyBMS_Set_Discharge_MOS(bool enable)
{
	if(enable)
	{
		_tx_buffer[4] = 0x01;  // Set command to enable discharge MOS
		DalyBMS_Request(DISCHRG_FET);
		_request_counter = 0;
	}
	else
	{
		_tx_buffer[4] = 0x00;  // Set command to disable discharge MOS
		DalyBMS_Request(DISCHRG_FET);
		_request_counter = 0;
	}

	if (!DalyBMS_Recive(1))
	{
		//Debug_Printf("BMS set discharge MOS failed\n");
		return false;
	}
	return true;
}

bool DalyBMS_Set_Charge_MOS(bool enable)
{
	if(enable)
	{
		_tx_buffer[4] = 0x01;  // Set command to enable charge MOS
		DalyBMS_Request(CHRG_FET);
		_request_counter = 0;
	}
	else
	{
		_tx_buffer[4] = 0x00;  // Set command to disable charge MOS
		DalyBMS_Request(CHRG_FET);
		_request_counter = 0;
	}

	if (!DalyBMS_Recive(1))
	{
		//debug_log
		//Debug_Printf("BMS set charge MOS failed\n");
		return false;
	}
	return true;
}

bool DalyBMS_Set_SoC(float soc)
{
	if (soc < 0 || soc > 100)
	{
		//Debug_Printf("Invalid SoC value: %f\n", soc);
		return false;  // Invalid SoC value
	}

	if (DalyBMS_Request(READ_SOC))
	{
		if (DalyBMS_Recive(1))
		{
//			Debug_Printf("BMS received SoC successfully\n");
			for (size_t i =5; i < 9; i++)
			{
				_tx_buffer[i] = _frame_buff[0][i];  // Copy time values from received frame
			}
		}
		else
		{
			//Debug_Printf("BMS request SoC failed\n");
			_tx_buffer[5] = 0x17;	// year
			_tx_buffer[6] = 0x01;	// month
			_tx_buffer[7] = 0x01;  	// day
			_tx_buffer[8] = 0x01;  	// hour
			_tx_buffer[9] = 0x01;  	// minute

			return false;
		}
	}

	uint16_t soc_value = (uint16_t)(soc * 10);  // Convert to integer representation
	_tx_buffer[10] = (uint8_t)(soc_value >> 8);  // High byte
	_tx_buffer[11] = (uint8_t)(soc_value & 0xFF);  // Low byte

	if (DalyBMS_Request(SET_SOC))
	{
		if (!DalyBMS_Recive(1))
		{
			//Debug_Printf("BMS set SoC failed\n");
			return false;  // Receive error
		}
	}


	return true;
}

bool DalyBMS_Reset()
{
	_request_counter = 0;
	DalyBMS_Request(BMS_RESET);

	if (!DalyBMS_Recive(1))
	{
		//debug_log
		//Debug_Printf("BMS reset failed\n");
		return false;  // Receive error
	}
	return true;
}

void DalyBMS_On_Request_Done(void)
{
//	Debug_Printf("<BMS> - Request done\n");
	//Debug_Printf("<BMS> - Voltage: %.2f V\n", bms_data.voltage_v);
	//Debug_Printf("<BMS> - Current: %.2f mA\n", bms_data.current_ma);
	//Debug_Printf("<BMS> - SoC: %.2f\n", bms_data.soc_percent);
	//Debug_Printf("<BMS> - Residual Capacity: %.2f mAh\n", bms_data.residual_capacity_mAh);
	//Debug_Printf("<BMS> - Life Cycle: %d\n", bms_data.bms_life_cycle);
	//Debug_Printf("<BMS> - Temperature: %.1f C\n", bms_data.temperature_average);
	//Debug_Printf("<BMS> - Charge/Discharge Status: %d\n", bms_data.charge_discharge_status);
	//Debug_Printf("<BMS> - Charge Status: %d\n", bms_data.charge_status);
	//Debug_Printf("<BMS> - Discharge Status: %d\n", bms_data.discharge_status);
	//Debug_Printf("<BMS> - Charge/Discharge Cycle: %d\n", bms_data.charge_discharge_cycle);
	//Debug_Printf("<BMS> - Charge MOS: %d\n", bms_data.charge_mos);
	//Debug_Printf("<BMS> - Discharge MOS: %d\n", bms_data.discharge_mos);
	//Debug_Printf("<BMS> - Cell Balance Active: %d\n", bms_data.cell_balance_active);
	//Debug_Printf("<BMS> - Cell Balance State (%d cells): ", bms_data.num_cells);
	// int max_cells = (bms_data.num_cells > MAX_CELLS) ? MAX_CELLS : bms_data.num_cells;
	// for (int i = 0; i < max_cells; i++) {
	// 	//Debug_Printf("%d ", bms_data.cell_balance_state[i]);
	// }
	//Debug_Printf("\n");
	//Debug_Printf("<BMS> - Max Cell Voltage: %.3f V (Cell %d)\n", bms_data.max_cell_v, bms_data.max_cell_voltage_num);
	//Debug_Printf("<BMS> - Min Cell Voltage: %.3f V (Cell %d)\n", bms_data.min_cell_v, bms_data.min_cell_voltage_num);
	//Debug_Printf("<BMS> - Cell Voltage Difference: %.3f V\n", bms_data.cell_diff);
	//Debug_Printf("<BMS> - Max Cell Threshold 1: %.3f V\n", bms_data.max_cell_threshold_1);
	//Debug_Printf("<BMS> - Max Cell Threshold 2: %.3f V\n", bms_data.max_cell_threshold_2);
	//Debug_Printf("<BMS> - Min Cell Threshold 1: %.3f V\n", bms_data.min_cell_threshold_1);
	//Debug_Printf("<BMS> - Min Cell Threshold 2: %.3f V\n", bms_data.min_cell_threshold_2);
	//Debug_Printf("<BMS> - Failure code: %s\n", bms_data.fault_flags);
}

void DalyBMS_Clear_Get()
{
    bms_data.charge_discharge_status = OFFLINE;
}

void DalyBMS_Set_Callback(DalyBMS_Callback_t callback) {
    _bms_request_callback = callback;
}
