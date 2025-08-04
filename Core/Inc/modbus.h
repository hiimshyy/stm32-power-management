/*
 * modbus.h
 *
 *  Created on: Aug 04, 2025
 *      Author: tiensy
 */

#ifndef INC_MODBUS_H_
#define INC_MODBUS_H_

#include "stm32f1xx_hal.h"
#include "mb.h"
#include "mbport.h"
#include "usbd_cdc_if.h"

#define MODBUS_REG_COUNT 256  // Số lượng Holding Register

// Modbus Configuration Registers (0x0100 - 0x0106)
#define REG_SLAVE_ID        0x0100  // Slave ID (1-247)
#define REG_BAUDRATE        0x0101  // Baudrate code (1=9600, 2=19200, 3=38400, 4=57600, 5=115200)
#define REG_PARITY          0x0102  // Parity (0=None, 1=Even, 2=Odd)
#define REG_STOP_BITS       0x0103  // Stop bits (1 or 2)
#define REG_FC_MASK         0x0104  // Function code mask (0x01=FC3, 0x02=FC6, 0x04=FC16)
#define REG_APPLY_CFG       0x0105  // Apply configuration (write 1 to apply)
#define REG_CONFIG_CRC      0x0106  // Configuration CRC16

// DalyBMS Status Registers (0x0000 - 0x002B)
#define REG_VOLTAGE        0x0000  // Pack voltage (V/10)
#define REG_CURRENT        0x0001  // Pack current (A/10)
#define REG_SOC            0x0002  // State of charge (%/10)
#define REG_MAX_CELL_V     0x0003  // Maximum cell voltage (V/10)
#define REG_MIN_CELL_V     0x0004  // Minimum cell voltage (V/10)
#define REG_MAX_CELL_NUM   0x0005  // Max voltage cell number
#define REG_MIN_CELL_NUM   0x0006  // Min voltage cell number
#define REG_CELL_DIFF      0x0007  // Cell voltage difference (mV/10)
#define REG_TEMPERATURE    0x0008  // Average temperature (°C)
#define REG_BMS_STATUS     0x0009  // BMS connection status
#define REG_CHG_DSG_STATUS 0x000A  // Charge/discharge status
#define REG_CHG_MOS        0x000B  // Charge MOSFET status
#define REG_DSG_MOS        0x000C  // Discharge MOSFET status
#define REG_BMS_CYCLES     0x000D  // BMS life cycles
#define REG_RESIDUAL_CAP   0x000E  // Residual capacity (mAh)
#define REG_NUM_CELLS      0x000F  // Number of cells
#define REG_NUM_TEMPS      0x0010  // Number of temperature sensors
#define REG_CHG_STAT       0x0011  // Charging status
#define REG_DSG_STAT       0x0012  // Discharging status
#define REG_CYCLE_COUNT    0x0013  // Charge-discharge cycles

// Cell voltages array (0x0014-0x0019)
#define REG_CELL_VOLTAGE   0x0014  // Start of cell voltages array [6]

// Temperature sensors array (0x001A-0x001B)
#define REG_TEMP_SENSORS   0x001A  // Start of temperature sensors array [2]

// Cell balance states (0x001C-0x0022)
#define REG_CELL_BALANCE   0x001C  // Start of cell balance states array [6]
#define REG_BAL_ACTIVE     0x0022  // Global balancing status

// Fault and threshold registers
#define REG_FAULT_FLAGS    0x0023  // Fault status flags
#define REG_MAX_CELL_TH1   0x0024  // Max cell voltage threshold 1
#define REG_MIN_CELL_TH1   0x0025  // Min cell voltage threshold 1
#define REG_MAX_CELL_TH2   0x0026  // Max cell voltage threshold 2
#define REG_MIN_CELL_TH2   0x0027  // Min cell voltage threshold 2
#define REG_MAX_PACK_TH1   0x0028  // Max pack voltage threshold 1
#define REG_MIN_PACK_TH1   0x0029  // Min pack voltage threshold 1
#define REG_MAX_PACK_TH2   0x002A  // Max pack voltage threshold 2
#define REG_MIN_PACK_TH2   0x002B  // Min pack voltage threshold 2

// SK60X Data Registers (0x0030 - 0x003E)
#define REG_SK60X_V_SET    0x0030  // Voltage setpoint (V/10)
#define REG_SK60X_I_SET    0x0031  // Current setpoint (A/10)
#define REG_SK60X_V_OUT    0x0032  // Output voltage (V/10)
#define REG_SK60X_I_OUT    0x0033  // Output current (A/10)
#define REG_SK60X_P_OUT    0x0034  // Output power (W/10)
#define REG_SK60X_V_IN     0x0035  // Input voltage (V/10)
#define REG_SK60X_I_IN     0x0036  // Input current (A/10)
#define REG_SK60X_TEMP     0x0037  // Temperature (°C)
#define REG_SK60X_HOURS    0x0038  // Hours of operation
#define REG_SK60X_MINS     0x0039  // Minutes of operation
#define REG_SK60X_SECS     0x003A  // Seconds of operation
#define REG_SK60X_STATUS   0x003B  // Operational status
#define REG_SK60X_ON_OFF   0x003C  // Output ON/OFF control
#define REG_SK60X_CHG_RLY  0x003D  // Charge relay control
#define REG_SK60X_CHG_ST   0x003E  // Charge state

// INA219 Sensor Registers (0x0040 - 0x0048)
#define REG_INA_V12        0x0040  // 12V rail voltage (V/10)
#define REG_INA_I12        0x0041  // 12V rail current (A/10)
#define REG_INA_P12        0x0042  // 12V rail power (W/10)
#define REG_INA_V5         0x0043  // 5V rail voltage (V/10)
#define REG_INA_I5         0x0044  // 5V rail current (A/10)
#define REG_INA_P5         0x0045  // 5V rail power (W/10)
#define REG_INA_V33        0x0046  // 3.3V rail voltage (V/10)
#define REG_INA_I33        0x0047  // 3.3V rail current (A/10)
#define REG_INA_P33        0x0048  // 3.3V rail power (W/10)

typedef enum {
    MB_REG_RO = 0,
    MB_REG_RW = 1,
    MB_REG_WO = 2
} ModbusAccess_t;

// Function prototypes
void Modbus_Init(void);
void Modbus_Access_Init(void);
void Modbus_UART2_Init(void);
void Modbus_CDC_Init(void);
void Modbus_Task(void *argument);
void Modbus_Apply_Config(void);
uint32_t baudrate_from_code(uint8_t code);
eMBParity parity_from_code(uint8_t code);

// Port function prototypes for UART2
BOOL xMBPortSerialPutByte_uart2(CHAR ucByte);
BOOL xMBPortSerialGetByte_uart2(CHAR *pucByte);
void vMBPortSerialEnable_uart2(BOOL xRxEnable, BOOL xTxEnable);

// Port function prototypes for CDC
BOOL xMBPortSerialPutByte_cdc(CHAR ucByte);
BOOL xMBPortSerialGetByte_cdc(CHAR *pucByte);
void vMBPortSerialEnable_cdc(BOOL xRxEnable, BOOL xTxEnable);

// Extern declarations
extern uint16_t modbus_registers[MODBUS_REG_COUNT];
extern SemaphoreHandle_t modbus_mutex;
eMBErrorCode eMBRegHoldingCB(UCHAR *pucRegBuffer, USHORT usAddress,
                             USHORT usNRegs, eMBRegisterMode eMode);



#endif /* INC_DEBUGGER_H_ */