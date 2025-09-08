/**
  ******************************************************************************
  * @file           : modbus_rtu.h
  * @brief          : Header for modbus_rtu.c file.
  *                   Modbus RTU communication library for STM32
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */

#ifndef __MODBUS_RTU_H
#define __MODBUS_RTU_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

/* Exported defines ----------------------------------------------------------*/
#define MODBUS_MAX_FRAME_SIZE       256
#define MODBUS_CRC_SIZE            2
#define MODBUS_MIN_FRAME_SIZE      4
#define MODBUS_TIMEOUT_MS          100
#define MODBUS_TX_DELAY_MS         5

// Uncomment to enable debug output
#define DEBUG_MODBUS

// Function Codes
#define MODBUS_FC_READ_HOLDING_REGISTERS    0x03
#define MODBUS_FC_WRITE_SINGLE_REGISTER     0x06
#define MODBUS_FC_WRITE_MULTIPLE_REGISTERS  0x10

// Exception Codes
#define MODBUS_EXCEPTION_ILLEGAL_FUNCTION           0x01
#define MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS       0x02
#define MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE         0x03
#define MODBUS_EXCEPTION_SLAVE_DEVICE_FAILURE       0x04

// DalyBMS Status Registers
#define REG_BMS_VOLTAGE             0x0000
#define REG_BMS_CURRENT             0x0001
#define REG_BMS_SOC                 0x0002
#define REG_BMS_MAX_CELL_V          0x0003
#define REG_BMS_MIN_CELL_V          0x0004
#define REG_BMS_MAX_CELL_NUM        0x0005
#define REG_BMS_MIN_CELL_NUM        0x0006
#define REG_BMS_CELL_DIFF           0x0007
#define REG_BMS_TEMPERATURE         0x0008
#define REG_BMS_CONNECTION_STATUS   0x0009
#define REG_BMS_CHARGE_DISCHARGE_STATUS 0x000A
#define REG_BMS_CHARGE_MOS          0x000B
#define REG_BMS_DISCHARGE_MOS       0x000C
#define REG_BMS_LIFE_CYCLE          0x000D
#define REG_BMS_RESIDUAL_CAPACITY   0x000E
#define REG_BMS_NUM_CELLS           0x000F
#define REG_BMS_NUM_TEMP_SENSORS    0x0010
#define REG_BMS_CHARGE_STATUS       0x0011
#define REG_BMS_DISCHARGE_STATUS    0x0012
#define REG_BMS_CHARGE_DISCHARGE_CYCLE 0x0013
#define REG_BMS_CELL_VOLTAGE_START  0x0014   // 0x0014-0x0019 (6 cells)
#define REG_BMS_TEMPERATURE_START   0x001A   // 0x001A-0x001B (2 sensors)
#define REG_BMS_CELL_BALANCE_START  0x001C   // 0x001C-0x0021 (6 cells)
#define REG_BMS_CELL_BALANCE_ACTIVE 0x0022
#define REG_BMS_FAULT_FLAGS         0x0023
#define REG_BMS_MAX_CELL_THRESHOLD_1 0x0024
#define REG_BMS_MIN_CELL_THRESHOLD_1 0x0025
#define REG_BMS_MAX_CELL_THRESHOLD_2 0x0026
#define REG_BMS_MIN_CELL_THRESHOLD_2 0x0027
#define REG_BMS_MAX_PACK_THRESHOLD_1 0x0028
#define REG_BMS_MIN_PACK_THRESHOLD_1 0x0029
#define REG_BMS_MAX_PACK_THRESHOLD_2 0x002A
#define REG_BMS_MIN_PACK_THRESHOLD_2 0x002B

// SK60X Data Registers
#define REG_SK60X_V_SET             0x0030
#define REG_SK60X_I_SET             0x0031
#define REG_SK60X_V_OUT             0x0032
#define REG_SK60X_I_OUT             0x0033
#define REG_SK60X_P_OUT             0x0034
#define REG_SK60X_V_IN              0x0035
#define REG_SK60X_I_IN              0x0036
#define REG_SK60X_TEMP              0x0037
#define REG_SK60X_H_USE             0x0038
#define REG_SK60X_M_USE             0x0039
#define REG_SK60X_S_USE             0x003A
#define REG_SK60X_CVCC              0x003B
#define REG_SK60X_ON_OFF            0x003C
#define REG_SK60X_LOCK              0x003D
#define REG_SK60X_CHARGE_STATE      0x003E
#define REG_SK60X_CHARGE_REQUEST    0x003F

// INA219 Sensor Registers
#define REG_INA219_V_OUT_12V        0x0040
#define REG_INA219_I_OUT_12V        0x0041
#define REG_INA219_P_OUT_12V        0x0042
#define REG_INA219_V_OUT_5V         0x0043
#define REG_INA219_I_OUT_5V         0x0044
#define REG_INA219_P_OUT_5V         0x0045
#define REG_INA219_V_OUT_3V3        0x0046
#define REG_INA219_I_OUT_3V3        0x0047
#define REG_INA219_P_OUT_3V3        0x0048

// Relay Status Registers 
#define REG_RELAY_3V3_STATUS        0x0049
#define REG_RELAY_5V_STATUS         0x004A
#define REG_RELAY_12V_STATUS        0x004B
#define REG_RELAY_FAUL_STATUS       0x004C
#define REG_RELAY_CHG_STATUS        0x004D
#define REG_VOLTAGE_THRESHOLD       0x004E

// System Registers
#define REG_DEVICE_ID         0x0100  // Device ID (Modbus slave address)
#define REG_CONFIG_BAUDRATE   0x0101  // Config baudrate (1=9600, 2=19200, 3=38400,...)
#define REG_CONFIG_PARITY     0x0102  // Config parity (0=None, 1=Even, 2=Odd)
#define REG_CONFIG_STOP_BITS  0x0103  // Config stop bits (1=1, 2=2)
#define REG_MODULE_TYPE       0x0104  // Module type (0x0002 = Power Module)
#define REG_FIRMWARE_VERSION  0x0105  // Firmware version (e.g. 0x0101 = v1.01)
#define REG_HARDWARE_VERSION  0x0106  // Hardware version (e.g. 0x0101 = v1.01)
#define REG_SYSTEM_STATUS     0x0107  // System status (bit field)
#define REG_SYSTEM_ERROR      0x0108  // System error (global error code)
#define REG_RESET_ERROR_CMD   0x0109  // Reset error command (write 1 to reset all error flags)

/* Exported types ------------------------------------------------------------*/
typedef enum {
    MODBUS_OK = 0,
    MODBUS_ERROR_TIMEOUT,
    MODBUS_ERROR_CRC,
    MODBUS_ERROR_FRAME,
    MODBUS_ERROR_FUNCTION,
    MODBUS_ERROR_ADDRESS,
    MODBUS_ERROR_VALUE,
    MODBUS_ERROR_DEVICE_FAILURE
} ModbusStatus_t;

typedef enum {
    MODBUS_BAUD_9600 = 1,
    MODBUS_BAUD_19200 = 2,
    MODBUS_BAUD_38400 = 3,
    MODBUS_BAUD_57600 = 4,
    MODBUS_BAUD_115200 = 5
} ModbusBaudrate_t;

typedef enum {
    MODBUS_PARITY_NONE = 0,
    MODBUS_PARITY_EVEN = 1,
    MODBUS_PARITY_ODD = 2
} ModbusParity_t;

typedef struct {
    uint8_t slave_id;
    ModbusBaudrate_t baudrate_code;
    ModbusParity_t parity;
    uint8_t stop_bits;
    uint8_t fc_mask;
} ModbusConfig_t;

typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t slave_id;
    uint8_t rx_buffer[MODBUS_MAX_FRAME_SIZE];
    uint8_t tx_buffer[MODBUS_MAX_FRAME_SIZE];
    uint16_t rx_length;
    uint16_t tx_length;
    bool frame_received;
    uint32_t last_rx_time;
} ModbusRTU_t;

/* Exported variables --------------------------------------------------------*/
extern ModbusRTU_t modbus_rtu;

// External variables for relay control
extern bool relay_power_enabled;
extern float voltage_threshold;

/* Exported function prototypes ----------------------------------------------*/

// Initialization and Configuration
HAL_StatusTypeDef ModbusRTU_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef ModbusRTU_SetConfig(ModbusConfig_t *config);
HAL_StatusTypeDef ModbusRTU_ApplyConfig(void);

// Core Functions
void ModbusRTU_Process(void);
void ModbusRTU_RxCpltCallback(UART_HandleTypeDef *huart);
ModbusStatus_t ModbusRTU_SendResponse(uint8_t *data, uint16_t length);

// CRC Functions
uint16_t ModbusRTU_CalculateCRC(uint8_t *data, uint16_t length);
bool ModbusRTU_CheckCRC(uint8_t *frame, uint16_t length);

// Frame Processing Functions
ModbusStatus_t ModbusRTU_ProcessFrame(uint8_t *frame, uint16_t length);
ModbusStatus_t ModbusRTU_ReadHoldingRegisters(uint8_t *frame, uint16_t length);
ModbusStatus_t ModbusRTU_WriteSingleRegister(uint8_t *frame, uint16_t length);
ModbusStatus_t ModbusRTU_WriteMultipleRegisters(uint8_t *frame, uint16_t length);

// Register Access Functions
uint16_t ModbusRTU_ReadRegister(uint16_t address);
ModbusStatus_t ModbusRTU_WriteRegister(uint16_t address, uint16_t value);

// Exception Response
void ModbusRTU_SendException(uint8_t function_code, uint8_t exception_code);

// Utility Functions
void ModbusRTU_BaudrateFromCode(ModbusBaudrate_t code);
void ModbusRTU_UpdateDataFromSources(void);

#ifdef __cplusplus
}
#endif

#endif /* __MODBUS_RTU_H */
