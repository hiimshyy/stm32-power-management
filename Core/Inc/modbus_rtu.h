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
#include "modbus_regs.h"
#include "cmsis_os.h"

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

// External flag for Modbus processing status (for debugging only)
extern volatile bool modbus_processing;

/* Exported function prototypes ----------------------------------------------*/

// Initialization and Configuration
HAL_StatusTypeDef ModbusRTU_Init(UART_HandleTypeDef *huart);
HAL_StatusTypeDef ModbusRTU_SetConfig(ModbusConfig_t *config);
HAL_StatusTypeDef ModbusRTU_ApplyConfig(void);

// Core Functions
void ModbusRTU_Process(void);
void ModbusRTU_RxCpltCallback(UART_HandleTypeDef *huart);
void ModbusRTU_ErrorCallback(UART_HandleTypeDef *huart);
void ModbusRTU_Reset(void);
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
