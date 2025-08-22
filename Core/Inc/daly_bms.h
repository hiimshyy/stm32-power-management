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
	uint16_t  max_cell_threshold_1;
	uint16_t  min_cell_threshold_1;
	uint16_t  max_cell_threshold_2;
	uint16_t  min_cell_threshold_2;

	// data form 0x5A
	uint16_t  max_pack_threshold_1;
	uint16_t  min_pack_threshold_1;
	uint16_t  max_pack_threshold_2;
	uint16_t  min_pack_threshold_2;

	// data form 0x90
    uint16_t  voltage;
    int16_t  current;
    uint16_t  soc;

    // data form 0x91
    uint16_t  	max_cell;
    uint16_t  	min_cell;
    uint8_t 	max_cell_index;
    uint8_t 	min_cell_index;
    uint8_t 	cell_diff;

    // data form 0x92
//    float max_temperature_c;
//    float min_temperature_c;
    uint8_t temperature_avr;

    // data form 0x93
    uint8_t charge_discharge_status;
    bool    charge_mos;
    bool    discharge_mos;
    uint8_t bms_life_cycle;
    uint16_t 	residual_capacity_mAh;

    // data form 0x94
    uint8_t  num_cells;
    uint8_t  num_temp_sensors;
    bool	 charge_status;
    bool	 discharge_status;
    uint8_t  charge_discharge_cycle;

    // data form 0x95
    uint16_t cell_voltage_mv[48];

    // data form 0x96
    uint8_t temperature_c[16];

    // data form 0x97
    bool cell_balance_state[48];
    bool cell_balance_active;

    // connection status
    bool connection_status;
    uint64_t fault_flags;
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

typedef enum {
    // Byte 0x00
    BMS_ERR_CELL_VOLT_HIGH_LVL2   = (1ULL << 0),
    BMS_ERR_CELL_VOLT_HIGH_LVL1   = (1ULL << 1),
    BMS_ERR_CELL_VOLT_LOW_LVL2    = (1ULL << 2),
    BMS_ERR_CELL_VOLT_LOW_LVL1    = (1ULL << 3),
    BMS_ERR_SUM_VOLT_HIGH_LVL2    = (1ULL << 4),
    BMS_ERR_SUM_VOLT_HIGH_LVL1    = (1ULL << 5),
    BMS_ERR_SUM_VOLT_LOW_LVL2     = (1ULL << 6),
    BMS_ERR_SUM_VOLT_LOW_LVL1     = (1ULL << 7),

    // Byte 0x01
    BMS_ERR_CHG_TEMP_HIGH_LVL2    = (1ULL << 8),
    BMS_ERR_CHG_TEMP_HIGH_LVL1    = (1ULL << 9),
    BMS_ERR_CHG_TEMP_LOW_LVL2     = (1ULL << 10),
    BMS_ERR_CHG_TEMP_LOW_LVL1     = (1ULL << 11),
    BMS_ERR_DISCHG_TEMP_HIGH_LVL2 = (1ULL << 12),
    BMS_ERR_DISCHG_TEMP_HIGH_LVL1 = (1ULL << 13),
    BMS_ERR_DISCHG_TEMP_LOW_LVL2  = (1ULL << 14),
    BMS_ERR_DISCHG_TEMP_LOW_LVL1  = (1ULL << 15),

    // Byte 0x02
    BMS_ERR_CHG_OVERCURRENT_LVL2  = (1ULL << 16),
    BMS_ERR_CHG_OVERCURRENT_LVL1  = (1ULL << 17),
    BMS_ERR_DISCHG_OVERCURRENT_LVL2 = (1ULL << 18),
    BMS_ERR_DISCHG_OVERCURRENT_LVL1 = (1ULL << 19),
    BMS_ERR_SOC_HIGH_LVL2         = (1ULL << 20),
    BMS_ERR_SOC_HIGH_LVL1         = (1ULL << 21),
    BMS_ERR_SOC_LOW_LVL2          = (1ULL << 22),
    BMS_ERR_SOC_LOW_LVL1          = (1ULL << 23),

    // Byte 0x03
    BMS_ERR_DIFF_VOLT_LVL2        = (1ULL << 24),
    BMS_ERR_DIFF_VOLT_LVL1        = (1ULL << 25),
    BMS_ERR_DIFF_TEMP_LVL2        = (1ULL << 26),
    BMS_ERR_DIFF_TEMP_LVL1        = (1ULL << 27),

    // Byte 0x04
    BMS_ERR_CHG_MOS_TEMP_HIGH     = (1ULL << 28),
    BMS_ERR_DISCHG_MOS_TEMP_HIGH  = (1ULL << 29),
    BMS_ERR_CHG_MOS_TEMP_SENSOR   = (1ULL << 30),
    BMS_ERR_DISCHG_MOS_TEMP_SENSOR= (1ULL << 31),
    BMS_ERR_CHG_MOS_ADHESION      = (1ULL << 32),
    BMS_ERR_DISCHG_MOS_ADHESION   = (1ULL << 33),
    BMS_ERR_CHG_MOS_OPEN_CIRCUIT  = (1ULL << 34),
    BMS_ERR_DISCHG_MOS_OPEN_CIRCUIT= (1ULL << 35),

    // Byte 0x05
    BMS_ERR_AFE_CHIP              = (1ULL << 36),
    BMS_ERR_VOLT_COLLECT_DROPPED  = (1ULL << 37),
    BMS_ERR_CELL_TEMP_SENSOR      = (1ULL << 38),
    BMS_ERR_EEPROM                = (1ULL << 39),
    BMS_ERR_RTC                   = (1ULL << 40),
    BMS_ERR_PRECHARGE_FAIL        = (1ULL << 41),
    BMS_ERR_COMM_FAIL             = (1ULL << 42),
    BMS_ERR_INTERNAL_COMM_FAIL    = (1ULL << 43),

    // Byte 0x06
    BMS_ERR_CURRENT_MODULE        = (1ULL << 44),
    BMS_ERR_SUM_VOLT_DETECT       = (1ULL << 45),
    BMS_ERR_SHORT_CIRCUIT         = (1ULL << 46),
    BMS_ERR_LOW_VOLT_FORBID_CHG   = (1ULL << 47)
} DalyBMS_ErrorFlags;

extern uint8_t _tx_buffer[FRAME_SIZE];
extern uint8_t _frame_buff[4][FRAME_SIZE];
extern uint8_t _rx_frame_buffer[FRAME_SIZE*4];

extern uint8_t _frame_count;
extern uint8_t _error_counter;
extern uint8_t _request_counter;
extern uint8_t _tx_checksum;
extern uint8_t _rx_checksum;
extern uint8_t _cell_num;
extern uint8_t _cell_bit;
extern uint8_t _balance_frame_count;

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

bool DalyBMS_Validate_Checksum();
void DalyBMS_Clear_Get();
void DalyBMS_On_Request_Done();

typedef void (*DalyBMS_Callback_t)(void);
extern DalyBMS_Callback_t _bms_request_callback;
void DalyBMS_Set_Callback(DalyBMS_Callback_t callback);
#endif
