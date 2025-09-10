/*
 * modbus_regs.h
 *
 *  Created on: Sep 9, 2025
 *      Author: tiensy
 */

#ifndef INC_MODBUS_REGS_H_
#define INC_MODBUS_REGS_H_

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



#endif /* INC_MODBUS_REGS_H_ */
