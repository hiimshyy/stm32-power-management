/*
 * daly_bms.h
 *
 *  Created on: Jul 9, 2025
 *      Author: tiensy
 */

#ifndef DALY_BMS_H
#define DALY_BMS_H

#include "stm32f1xx_hal.h"
#include "debugger.h"
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#define FRAME_SIZE 13
#define MIN_CELLS 1
#define MAX_CELLS 16
#define MIN_TEMP_SENSORS 1
#define MAX_TEMP_SENSORS 4

#define START_BYTE 0xA5
#define HOST_ADDRESS 0x40
#define FRAME_LENGTH 0x08
#define MAX_ERROR 10
#define DELAY_TIME 150

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)

extern UART_HandleTypeDef huart1; // Define your UART handle here

typedef struct {
	// data form 0x59
	float  max_cell_threshold_1;
	float  min_cell_threshold_1;
	float  max_cell_threshold_2;
	float  min_cell_threshold_2;

	// data form 0x5A
	float  max_pack_threshold_1;
	float  min_pack_threshold_1;
	float  max_pack_threshold_2;
	float  min_pack_threshold_2;

	// data form 0x90
    float  voltage_v;
    float  current_ma;
    float  soc_percent;

    // data form 0x91
    float  	max_cell_v;
    float  	min_cell_v;
    uint8_t max_cell_voltage_num;
    uint8_t min_cell_voltage_num;
    uint8_t cell_diff;

    // data form 0x92
//    float max_temperature_c;
//    float min_temperature_c;
    float temperature_average;

    // data form 0x93
    uint8_t charge_discharge_status;
    bool    charge_mos;
    bool    discharge_mos;
    uint8_t bms_life_cycle;
    float 	residual_capacity_mAh;

    // data form 0x94
    uint8_t  num_cells;
    uint8_t  num_temp_sensors;
    bool	 charge_status;
    bool	 discharge_status;
//    bool	 dIO[8];
    uint8_t  charge_discharge_cycle;

    // data form 0x95
    float cell_voltage_mv[48];

    // data form 0x96
    uint8_t temperature_c[16];

    // data form 0x97
    bool cell_balance_state[48];
    bool cell_balance_active;

    // connection status
    bool connection_status;
	uint8_t fault_flags;
} DalyBMS_Data;

typedef enum {
	CELL_THRESHOLDS = 0x59,
	PACK_THRESHOLDS = 0x5A,
    VOUT_IOUT_SOC = 0x90,
    MIN_MAX_CELL_VOLTAGE = 0x91,
    MIN_MAX_TEMPERATURE = 0x92,
    DISCHARGE_CHARGE_MOS_STATUS = 0x93,
    STATUS_INFO = 0x94,
    CELL_VOLTAGES = 0x95,
    CELL_TEMPERATURE = 0x96,
    CELL_BALANCE_STATE = 0x97,
    FAILURE_CODES = 0x98,
    DISCHRG_FET = 0xD9,
    CHRG_FET = 0xDA,
    BMS_RESET = 0x00,
    READ_SOC = 0x61, //read the time and soc
    SET_SOC = 0x21, //set the time and soc
} DalyBMS_Command;

typedef enum {
	STATIONARY = 0x00,
	CHARGE = 0x01,
	DISCHARGE = 0x02,
	OFFLINE = 0x03,
} DalyBMS_Charge_Discharge_Status;

extern uint8_t _rx_buffer[FRAME_SIZE];
extern uint8_t _tx_buffer[FRAME_SIZE];
extern uint8_t _frame_buff[12][FRAME_SIZE];
extern uint8_t _rx_frame_buffer[FRAME_SIZE*12];

extern uint8_t _commandBuffer[5];

extern uint8_t _frame_index;
extern uint8_t _frame_count;
extern uint8_t _error_counter;
extern uint8_t _request_count;
extern uint8_t _request_counter;

extern bool	_get_static_data;

extern DalyBMS_Data bms_data;

bool DalyBMS_Request(DalyBMS_Command command);
bool DalyBMS_Recive(uint8_t _frame_amount);

bool DalyBMS_Get_Pack_Data();
bool DalyBMS_Get_Min_Max_Cell_Voltage();
bool DalyBMS_Get_Pack_Temperature();
bool DalyBMS_Get_Charge_Discharge_Status();
bool DalyBMS_Get_Status_Info();
bool DalyBMS_Get_Cell_Voltages();
bool DalyBMS_Get_Cell_Temperatures();
bool DalyBMS_Get_Cell_Balance_State();
bool DalyBMS_Get_Failure_Codes();
bool DalyBMS_Get_Voltage_Thresholds();
bool DalyBMS_Get_Pack_Thresholds();
bool DalyBMS_Get_Connection_Status();


bool DalyBMS_Set_Discharge_MOS(bool enable);
bool DalyBMS_Set_Charge_MOS(bool enable);
bool DalyBMS_Set_SoC(float soc);
bool DalyBMS_Reset();

uint8_t DalyBMS_Calculate_Checksum(uint8_t *buffer, uint8_t length);
bool DalyBMS_Validate_Checksum();
void DalyBMS_Clear_Get();
void DalyBMS_On_Request_Done();

typedef void (*DalyBMS_Callback_t)(void);
extern DalyBMS_Callback_t _bms_request_callback;
void DalyBMS_Set_Callback(DalyBMS_Callback_t callback);
#endif
