#ifndef DALY_BMS_H
#define DALY_BMS_H

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define FRAME_SIZE 13
#define MIN_CELLS 1
#define MAX_CELLS 4
#define MIN_TEMP_SENSORS 1
#define MAX_TEMP_SENSORS 4

#define START_BYTE 0xA5
#define HOST_ADDRESS 0x40
#define FRAME_LENGTH 0x08
#define ERROR_COUNTER 10
#define DELAY_TIME 150

typedef struct {
    uint16_t voltage_mv;
    int16_t  current_ma;
    uint8_t  soc_percent;
    uint8_t  num_cells;
    uint16_t cell_voltage_mv[16];  // up to 16 cells
    uint8_t  num_temp_sensors;
    float    temperature_c[4];
    uint16_t fault_flags;
    bool     charge_mos;
    bool     discharge_mos;
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

uint8_t rxBuffer[FRAME_SIZE];
uint8_t txBuffer[FRAME_SIZE];
uint8_t frameBuff[12][FRAME_SIZE];
uint8_t rxFrameBuffer[FRAME_SIZE*12];

unsigned int frameIndex = 0;
unsigned int frameCount = 0;
unsigned int errorCounter = 0;
unsigned int requestcount = 0;
bool getStaticData = false;


bool DalyBMS_Request(UART_HandleTypeDef *huart, DalyBMS_Command command);
bool DalyBMS_Recive(UART_HandleTypeDef *huart, unsigned int frameAmount);
bool DalyBMS_Parse(DalyBMS_Data *data, uint8_t *frame);
bool Valiadte_Checksum();
#endif
