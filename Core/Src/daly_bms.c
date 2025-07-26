/*
 * daly_bms.c
 *
 *  Created on: Jul 9, 2025
 *      Author: tiensy
 */


#include "daly_bms.h"
#include <string.h>

DalyBMS_Callback_t _bms_request_callback = NULL;

uint8_t _rx_buffer[FRAME_SIZE];
uint8_t _tx_buffer[FRAME_SIZE];
uint8_t _frame_buff[12][FRAME_SIZE];
uint8_t _rx_frame_buffer[FRAME_SIZE*12];
uint8_t _commandBuffer[5] = {0x10, 0x10, 0x10, 0x10, 0x10};
uint8_t _frame_index = 0;
uint8_t _frame_count = 0;
uint8_t _error_counter = 0;
uint8_t _request_count = 0;
uint8_t _request_counter = 0;
bool 	_get_static_data = false;

DalyBMS_Data bms_data;


bool DalyBMS_Request(DalyBMS_Command command)
{
	// Prepare the transmit buffer
    memset(_tx_buffer, 0x00, FRAME_SIZE);

    uint8_t txChecksum = 0x00;    // transmit checksum buffer
    // Frame structure: [START_BYTE, HOST_ADDRESS, COMMAND, FRAME_LENGTH, DATA..., CHECKSUM]
    _tx_buffer[0] = START_BYTE;
    _tx_buffer[1] = HOST_ADDRESS;
    _tx_buffer[2] = command;
    _tx_buffer[3] = FRAME_LENGTH;

    // Add checksum for the first 4 bytes
    for (int i = 0; i < 11; i++) txChecksum += _tx_buffer[i];
    _tx_buffer[12] = (uint8_t)(txChecksum & 0xFF); // checksum

    // Debug start
    Debug_Printf("BMS Send request:");
    for (int i = 0; i < FRAME_SIZE; i++)
	{
		Debug_Printf(" %02X", _tx_buffer[i]);
	}
    Debug_Printf("\n");
    // Debug end

    return (HAL_UART_Transmit(&huart1, _tx_buffer, FRAME_SIZE, 100) == HAL_OK);
}

bool DalyBMS_Recive(uint8_t _frame_amount)
{
	// Clear the buffer before receiving new data
//	memset(_rx_buffer, 0x00, FRAME_SIZE);
	memset(_rx_frame_buffer, 0x00, sizeof(_rx_frame_buffer));
	memset(_frame_buff, 0x00, sizeof(_frame_buff));

	size_t _total_bytes = _frame_amount * FRAME_SIZE;
	// Receive data into the buffer
	if (HAL_UART_Receive(&huart1, _rx_frame_buffer, _total_bytes, 200) != HAL_OK)
	{
		Debug_Printf("BMS receive error\n");
		return false;  // Receive error
	}
	else
	{
		Debug_Printf("Data received:");
		for (size_t i = 0; i < _total_bytes; i++)
		{
			Debug_Printf(" %02X", _rx_frame_buffer[i]);
			if ((i + 1) % FRAME_SIZE == 0) Debug_Printf("\n");
		}
	}

	for (size_t i = 0; i < _frame_amount; i++)
	{
		memcpy(_frame_buff[i], &_rx_frame_buffer[i * FRAME_SIZE], FRAME_SIZE);

		uint8_t _checksum = 0;
		for (size_t j = 0; j < FRAME_SIZE - 1; j++)	_checksum += _frame_buff[i][j];

		if (_checksum != _frame_buff[i][FRAME_SIZE - 1])
		{
			Debug_Printf("BMS checksum error in frame %d\n", i);
			Debug_Printf("Expected: %02X, Received: %02X\n", _checksum, _frame_buff[i][FRAME_SIZE - 1]);
			return false;  // Checksum error
		}

		if (_checksum == 0x00 && _frame_buff[i][0] == 0x20)	Debug_Printf("BMS sleep or invalid frame\n");
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
				Debug_Printf("Current out of range, retrying...\n");
				return false;  // Current out of range
			}
			else if((((float)(((_frame_buff[0][10] << 8) | _frame_buff[0][11])) / 10.0f) > 100.f))
			{
				Debug_Printf("SOC out of range, retrying...\n");
				return false;  // SOC out of range
			}
			else
			{
				bms_data.voltage_v = ((float)((_frame_buff[0][4] << 8) | _frame_buff[0][5])) / 10.0f;
				bms_data.current_ma = ((float)((_frame_buff[0][8] << 8) | _frame_buff[0][9]) - 30000) / 10.0f;
				bms_data.soc_percent = ((float)((_frame_buff[0][10] << 8) | _frame_buff[0][11]) / 10.0f);
//				Debug_Printf("<BMS> - BMS Voltage: %.2f V\n", bms_data.voltage_v);
//				Debug_Printf("<BMS> - BMS Current: %.2f mA\n", bms_data.current_ma);
//				Debug_Printf("<BMS> - BMS SoC: %.1f\n", bms_data.soc_percent);
//				return true;  // Request successful
			}
		}
		else
		{
			Debug_Printf("BMS receive pack data failed\n");
			return false;  // Receive error
		}
	}
	else
	{
		Debug_Printf("BMS request pack data failed\n");
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
			Debug_Printf("BMS received min/max cell voltage successfully\n");
			bms_data.max_cell_v = ((float)((_frame_buff[0][4] << 8) | _frame_buff[0][5])) / 1000.0f;
			bms_data.max_cell_voltage_num = _frame_buff[0][6];
			bms_data.min_cell_v = ((float)((_frame_buff[0][7] << 8) | _frame_buff[0][8])) / 1000.0f;
			bms_data.min_cell_voltage_num = _frame_buff[0][9];
			bms_data.cell_diff = bms_data.max_cell_v - bms_data.min_cell_v;
//			Debug_Printf("<BMS> - Max Cell Voltage: %.2f V (Cell %d)\n", bms_data.max_cell_v, bms_data.max_cell_voltage_num);
//			Debug_Printf("<BMS> - Min Cell Voltage: %.2f V (Cell %d)\n", bms_data.min_cell_v, bms_data.min_cell_voltage_num);
//			Debug_Printf("<BMS> - Voltage Difference: %.3f V\n", bms_data.cell_diff);
//			return true;  // Request successful
		}
		else
		{
			Debug_Printf("BMS receive min/max cell voltage failed\n");
			return false;  // Receive error
		}
	}
	else
	{
		Debug_Printf("BMS request min/max cell voltage failed\n");
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
			Debug_Printf("BMS received min/max temperature successfully\n");
//			bms_data.max_temperature_c = (_frame_buff[0][4] - 40);
//			bms_data.min_temperature_c = (_frame_buff[0][6] - 40);
			bms_data.temperature_average = ((_frame_buff[0][4] - 40) + (_frame_buff[0][6] - 40)) / 2;
//			Debug_Printf("<BMS> - Max Temperature: %.1f C\n", bms_data.max_temperature_c);
//			Debug_Printf("<BMS> - Min Temperature: %.1f C\n", bms_data.min_temperature_c);
//			Debug_Printf("<BMS> - Average Temperature: %.1f C\n", bms_data.temperature_average);
//			return true;  // Request successful
		}
		else
		{
			Debug_Printf("BMS receive min/max temperature failed\n");
			return false;  // Receive error
		}
	}
	else
	{
		Debug_Printf("BMS request min/max temperature failed\n");
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
			Debug_Printf("BMS received charge/discharge MOS status successfully\n");
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

			float tmpAh = (((uint32_t)_frame_buff[0][8] << 0x18) | ((uint32_t)_frame_buff[0][9] << 0x10) | ((uint32_t)_frame_buff[0][10] << 0x08) | (uint32_t)_frame_buff[0][11]);
			bms_data.residual_capacity_mAh = tmpAh;

			Debug_Printf("<BMS> - Charge/Discharge Status: %d\n", bms_data.charge_discharge_status);
			Debug_Printf("<BMS> - Charge Status: %d\n", bms_data.charge_mos);
			Debug_Printf("<BMS> - Discharge Status: %d\n", bms_data.discharge_mos);
			Debug_Printf("<BMS> - Life: %d\n", bms_data.bms_life_cycle);
			Debug_Printf("<BMS> - Residual Capacity: %.2f mAh\n", bms_data.residual_capacity_mAh);
//			return true;
		}
		else
		{
			Debug_Printf("BMS receive charge/discharge MOS status failed\n");
			return false;  // Receive error
		}
	}
	else
	{
		Debug_Printf("BMS request charge/discharge MOS status failed\n");
		return false;  // Request failed
	}

	return true;
}

bool DalyBMS_Get_Status_Info()
{
	Debug_Printf("BMS request status info\n");
	if (DalyBMS_Request(STATUS_INFO))
	{
		if (DalyBMS_Recive(1))
		{
			Debug_Printf("BMS received status info successfully\n");
			bms_data.num_cells = _frame_buff[0][4];
			bms_data.num_temp_sensors = _frame_buff[0][5];
			bms_data.charge_status = _frame_buff[0][6];
			bms_data.discharge_status = _frame_buff[0][7];
			bms_data.charge_discharge_cycle = ((uint16_t)_frame_buff[0][9] << 0x08) | (uint16_t)_frame_buff[0][10];
			Debug_Printf("<BMS> - Number of cell: %d\n", bms_data.num_cells);
			Debug_Printf("<BMS> - Number of temperature sensors: %d\n", bms_data.num_temp_sensors);
			Debug_Printf("<BMS> - Charge status: %d\n", bms_data.charge_status);
			Debug_Printf("<BMS> - Discharge status: %d\n", bms_data.discharge_status);
			Debug_Printf("<BMS> - Charge/Discharge Cycle: %d\n", bms_data.charge_discharge_cycle);
			return true;  // Request successful
		}
		else
		{
			Debug_Printf("BMS receive status info failed\n");
			return false;  // Receive error
		}
	}
	else
	{
		Debug_Printf("BMS request status info failed\n");
		return false;  // Request failed
	}
	return true;
}

bool DalyBMS_Get_Cell_Voltages()
{
	Debug_Printf("BMS request cell voltages\n");
	if (bms_data.num_cells < MIN_CELLS || bms_data.num_cells > MAX_CELLS)
	{
		Debug_Printf("Invalid cell count: %d\n", bms_data.num_cells);
		return false;  // Invalid cell count
	}
	if (DalyBMS_Request(CELL_VOLTAGES))
	{
		uint8_t _cell_num = 0;
		uint8_t _frame_count = (bms_data.num_cells +2 )/3; // 1 frame for 1-3 cells, 2 frames for 4-6 cells, etc.
		if (DalyBMS_Recive(_frame_count))
		{
			Debug_Printf("BMS received cell voltage successfully\n");


			for (uint8_t k = 0; k < _frame_count; k++)
			{
				for (uint8_t i = 0; i < 3; i++)
				{
					if (_cell_num >= bms_data.num_cells) break;

				  	uint16_t cell_mV = (_frame_buff[k][5 + 2*i] << 8) | _frame_buff[k][6 + 2*i];
				  	bms_data.cell_voltage_mv[_cell_num++] = cell_mV;
				  	Debug_Printf("<BMS> - Cell %d Voltage: %.3f V\n", _cell_num, ((float)cell_mV / 1000.0f));
				}
			}
		}
		else
		{
			Debug_Printf("BMS request cell voltage failed\n");
			return false;  // Request failed
		}
	}
	return true;
}

bool DalyBMS_Get_Cell_Temperatures()
{
	if (bms_data.num_temp_sensors < MIN_TEMP_SENSORS || bms_data.num_temp_sensors > MAX_TEMP_SENSORS)
	{
		Debug_Printf("Invalid temperature sensor count: %d\n", bms_data.num_temp_sensors);
		return false;  // Invalid temperature sensor count
	}
	if (DalyBMS_Request(CELL_TEMPERATURE))
	{
		if (DalyBMS_Recive(1))
		{
			Debug_Printf("BMS received cell temperatures successfully\n");
//			uint8_t _sensor_num = 0;
//			uint8_t _temp_frame_count = (bms_data.num_temp_sensors + 6) / 7;

			for (uint8_t i = 0; i < bms_data.num_temp_sensors; i++)
			{
//				if (_sensor_num >= bms_data.num_temp_sensors) break;
				bms_data.temperature_c[i] = (_frame_buff[0][5 + i] - 40);
//				Debug_Printf("<BMS> - Sensor %d Temperature: %d C\n", i + 1, bms_data.temperature_c[i]);
//				_sensor_num++;
			}
			return true;  // Request successful
		}
		else
		{
			Debug_Printf("BMS receive cell temperatures failed\n");
			return false;  // Receive error
		}
	}
	else
	{
		Debug_Printf("BMS request cell temperatures failed\n");
		return false;  // Request failed
	}

	return true;
}

bool DalyBMS_Get_Cell_Balance_State()
{
	if (bms_data.num_cells < MIN_CELLS || bms_data.num_cells > MAX_CELLS)
	{
		Debug_Printf("Invalid cell count: %d\n", bms_data.num_cells);
		return false;  // Invalid cell count
	}

	if (DalyBMS_Request(CELL_BALANCE_STATE))
	{
		if (DalyBMS_Recive(1))
		{
			Debug_Printf("BMS received cell balance state successfully\n");
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
//					Debug_Printf("<BMS> - Cell %d Balance State: %s\n", _cell_bit, bit ? "ON" : "OFF");
					if (bit) _balance_frame_count++;
				}
			}
			bms_data.cell_balance_active = (_balance_frame_count > 0) ? true : false;
//			Debug_Printf("<BMS> - Cell Balance Active: %s\n", bms_data.cell_balance_active ? "Yes" : "No");
			return true;  // Request successful
		}
		else
		{
			Debug_Printf("BMS receive cell balance state failed\n");
			return false;  // Receive error
		}
	}
	else
	{
		Debug_Printf("BMS request cell balance state failed\n");
		return false;  // Request failed
	}

	return true;
}

bool DalyBMS_Get_Failure_Codes()
{
	const char* faultMsgs[7][8] = {
		  		{"Cell volt high L1", "Cell volt high L2", "Cell volt low L1", "Cell volt low L2", "Sum volt high L1", "Sum volt high L2", "Sum volt low L1", "Sum volt low L2"},
		  		{"Charge temperature high L1", "Charge temperature high L2", "Charge temperature low L1", "Charge temperature low L2", "Discharge temperature high L1", "Discharge temperature high L2", "Discharge temperature low L1", "Discharge temperature low L2"},
		  		{"Charge over current L1", "Charge over current L2", "Discharge over current L1", "Discharge over current L2", "SoC high L1", "SoC high L2", "SoC low L1", "SoC low L2"},
		  		{"Difference volt L1", "Difference volt L2", "Difference temperature L1", "Difference temperature L2", "", "", "", ""},
		  		{"Charge MOS temperature high", "Discharge MOS temperature high", "Charge MOS temperature sensor err", "Discharge MOS temperature sensor err", "Charge MOS adhesion err", "Discharge MOS adhesion err", "Charge MOS open circuit", "Discharge MOS open circuit"},
		  		{"AFE collect chip err", "Voltage collect dropped", "Cell temperature sensor err", "EEPROM err", "RTC err", "Precharge fail", "Communication fail", "Internal communication fail"},
		  		{"Current module fault", "Sum volt detect fault", "Short circuit protect fault", "Low volt charge forbidden", "", "", "", ""}
		  	 };
	if (DalyBMS_Request(FAILURE_CODES))
	{
		if (DalyBMS_Recive(1))
		{
			Debug_Printf("BMS received failure codes successfully\n");
			Debug_Printf("<BMS> - Failure Codes:\n");

			for (uint8_t byteIdx = 0; byteIdx < 7; byteIdx++) {
				uint8_t byteVal = _frame_buff[0][4 + byteIdx];
				for (uint8_t bit = 0; bit < 8; bit++) {
			 	    if (byteVal & (1 << bit)) {
			 	    	const char* msg = faultMsgs[byteIdx][bit];
			 	    	if (msg[0] != '\0') Debug_Printf("- %s\n", msg);
			 	    }
				}
			}
			return true;
		}
		else
		{
			Debug_Printf("BMS receive failure codes failed\n");
			return false;  // Receive error
		}
	}
	else
	{
		Debug_Printf("BMS request failure codes failed\n");
		return false;  // Request failed
	}

	return true;
}

bool DalyBMS_Get_Voltage_Thresholds()
{
	if (DalyBMS_Request(CELL_THRESHOLDS))
	{
		if (DalyBMS_Recive(1))
		{
			Debug_Printf("BMS received cell thresholds successfully\n");
			bms_data.max_cell_threshold_1 = ((float)((_frame_buff[0][4] << 8) | _frame_buff[0][5]));
			bms_data.max_cell_threshold_2 = ((float)((_frame_buff[0][6] << 8) | _frame_buff[0][7]));
			bms_data.min_cell_threshold_1 = ((float)((_frame_buff[0][8] << 8) | _frame_buff[0][9]));
			bms_data.min_cell_threshold_2 = ((float)((_frame_buff[0][10] << 8) | _frame_buff[0][11]));
			Debug_Printf("<BMS> - Max Cell Threshold 1: %.2f V\n", bms_data.max_cell_threshold_1);
			Debug_Printf("<BMS> - Max Cell Threshold 2: %.2f V\n", bms_data.max_cell_threshold_2);
			Debug_Printf("<BMS> - Min Cell Threshold 1: %.2f V\n", bms_data.min_cell_threshold_1);
			Debug_Printf("<BMS> - Min Cell Threshold 2: %.2f V\n", bms_data.min_cell_threshold_2);
			return true;  // Request successful
		}
		else
		{
			Debug_Printf("BMS receive cell thresholds failed\n");
			return false;  // Receive error
		}
	}
	else
	{
		Debug_Printf("BMS request cell thresholds failed\n");
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
			Debug_Printf("BMS received pack thresholds successfully\n");
			bms_data.max_pack_threshold_1 = ((float)((_frame_buff[0][4] << 8) | _frame_buff[0][5]));
			bms_data.max_pack_threshold_2 = ((float)((_frame_buff[0][6] << 8) | _frame_buff[0][7]));
			bms_data.min_pack_threshold_1 = ((float)((_frame_buff[0][8] << 8) | _frame_buff[0][9]));
			bms_data.min_pack_threshold_2 = ((float)((_frame_buff[0][10] << 8) | _frame_buff[0][11]));
//			Debug_Printf("<BMS> - Max Pack Threshold 1: %.2f V\n", bms_data.max_pack_threshold_1);
//			Debug_Printf("<BMS> - Max Pack Threshold 2: %.2f V\n", bms_data.max_pack_threshold_2);
//			Debug_Printf("<BMS> - Min Pack Threshold 1: %.2f V\n", bms_data.min_pack_threshold_1);
//			Debug_Printf("<BMS> - Min Pack Threshold 2: %.2f V\n", bms_data.min_pack_threshold_2);
			return true;  // Request successful
		}
		else
		{
			Debug_Printf("BMS receive pack thresholds failed\n");
			return false;  // Receive error
		}
	}
	else
	{
		Debug_Printf("BMS request pack thresholds failed\n");
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
		//debug_log
		Debug_Printf("BMS set discharge MOS failed\n");
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
		Debug_Printf("BMS set charge MOS failed\n");
		return false;
	}
	return true;
}

bool DalyBMS_Set_SoC(float soc)
{
	if (soc < 0 || soc > 100)
	{
		Debug_Printf("Invalid SoC value: %f\n", soc);
		return false;  // Invalid SoC value
	}

	if (DalyBMS_Request(READ_SOC))
	{
		if (DalyBMS_Recive(1))
		{
			Debug_Printf("BMS received SoC successfully\n");
			for (size_t i =5; i < 9; i++)
			{
				_tx_buffer[i] = _frame_buff[0][i];  // Copy time values from received frame
			}
		}
		else
		{
			Debug_Printf("BMS request SoC failed\n");
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
			Debug_Printf("BMS set SoC failed\n");
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
		Debug_Printf("BMS reset failed\n");
		return false;  // Receive error
	}
	return true;
}

void DalyBMS_On_Request_Done(void)
{
	// send json data
	Debug_Printf("debug: on callback.\n");
//	char json_buffer[1024];
//	int offset = 0;

//	offset += snprintf(json_buffer + offset, sizeof(json_buffer) - offset,
//	        "{\"Voltage\":%.2f,"
//	        "\"Current\":%.2f,"
//	        "\"SoC\":%.2f,"
//			"\"Remaning_Ah\":%.2f,"
//			"\"Cycles\":%d,"
//			"\"BMS_Temp\":%.1f,"
//			"\"Cell_Temps\":%d,"
//	        "\"Status\":\"%s\"}",
//	        bms_data.voltage_mv,
//	        bms_data.current_ma,
//	        bms_data.soc_percent,
//			bms_data.residual_capacity_mAh,
//			bms_data.bms_life_cycle,
//			bms_data.temperature_average,
//			bms_data.temperature_c[0],
//	        bms_data.charge_discharge_status
//	    );
}

uint8_t DalyBMS_Calculate_Checksum(uint8_t *buffer, uint8_t length)
{
	if (length < 1) return -1;


	uint8_t checksum = 0;
	for (uint8_t i = 0; i < length - 1; i++)
	{
		checksum += buffer[i];
	}

	return checksum;
}

bool DalyBMS_Validate_Checksum()
{
	uint8_t checksum = 0;
	for (int i = 0; i < FRAME_SIZE - 1; i++)
	{
		checksum += _rx_buffer[i];
	}
	return (checksum == _rx_buffer[FRAME_SIZE - 1]);
}


void DalyBMS_Clear_Get()
{
    bms_data.charge_discharge_status = OFFLINE;
}

void DalyBMS_Set_Callback(DalyBMS_Callback_t callback) {
    _bms_request_callback = callback;
}
