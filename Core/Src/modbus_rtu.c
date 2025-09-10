/**
  ******************************************************************************
  * @file           : modbus_rtu.c
  * @brief          : Modbus RTU communication library implementation
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

/* Includes ------------------------------------------------------------------*/
#include "modbus_rtu.h"
#include "main.h"
#include "daly_bms.h"
#include "sk60x.h"
#include "ina219.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ModbusRTU_t modbus_rtu;
ModbusConfig_t modbus_config = {
    .slave_id = 0x02,
    .baudrate_code = MODBUS_BAUD_115200,  // Match UART2 config
    .parity = MODBUS_PARITY_NONE,
    .stop_bits = 1,
    .fc_mask = 0x07  // Support FC 03, 06, 16
};

// Flag to indicate Modbus is processing
volatile bool modbus_processing = false;

// External variables from other modules
extern DalyBMS_Data bms_data;
extern SK60X_Data sk60x_data;
extern INA219_t ina_12v, ina_5v, ina_3v3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief Initialize Modbus RTU
 * @param huart: UART handle pointer
 * @param slave_id: Modbus slave ID
 * @retval HAL status
 */
HAL_StatusTypeDef ModbusRTU_Init(UART_HandleTypeDef *huart)
{
    if (huart == NULL) {
        return HAL_ERROR;
    }
    
    modbus_rtu.huart = huart;
    modbus_rtu.slave_id = modbus_config.slave_id;
    modbus_rtu.frame_received = false;
    modbus_rtu.rx_length = 0;
    modbus_rtu.tx_length = 0;
    modbus_rtu.last_rx_time = HAL_GetTick();
    
    memset(modbus_rtu.rx_buffer, 0, MODBUS_MAX_FRAME_SIZE);
    memset(modbus_rtu.tx_buffer, 0, MODBUS_MAX_FRAME_SIZE);
    
    // Ensure UART is ready before starting receive
    if (modbus_rtu.huart->RxState != HAL_UART_STATE_READY) {
        HAL_UART_Abort(modbus_rtu.huart);
    }
    
    // Start receiving data byte by byte
    return HAL_UART_Receive_IT(modbus_rtu.huart, &modbus_rtu.rx_buffer[0], 1);
}

/**
 * @brief Calculate CRC16 for Modbus RTU
 * @param data: Pointer to data buffer
 * @param length: Length of data
 * @retval CRC16 value
 */
uint16_t ModbusRTU_CalculateCRC(uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFF;
    
    for (uint16_t i = 0; i < length; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc = crc >> 1;
            }
        }
    }
    
    return crc;
}

/**
 * @brief Check CRC of received frame
 * @param frame: Pointer to frame buffer
 * @param length: Length of frame
 * @retval true if CRC is correct, false if wrong
 */
bool ModbusRTU_CheckCRC(uint8_t *frame, uint16_t length)
{
    if (length < MODBUS_MIN_FRAME_SIZE) {
        return false;
    }
    
    uint16_t calculated_crc = ModbusRTU_CalculateCRC(frame, length - 2);
    uint16_t received_crc = (frame[length - 1] << 8) | frame[length - 2];
    
    return (calculated_crc == received_crc);
}

/**
 * @brief Send response via UART
 * @param data: Pointer to data buffer
 * @param length: Length of data
 * @retval Modbus status
 */
ModbusStatus_t ModbusRTU_SendResponse(uint8_t *data, uint16_t length)
{
    // Calculate CRC and add to frame
    uint16_t crc = ModbusRTU_CalculateCRC(data, length);
    data[length] = crc & 0xFF;
    data[length + 1] = (crc >> 8) & 0xFF;
    length += 2;

    // Abort any ongoing receive to avoid conflicts
    HAL_UART_AbortReceive_IT(modbus_rtu.huart);

    // Small delay to ensure UART is ready
    HAL_Delay(1);

    // Send response with longer timeout for large frames
    uint32_t timeout = (length > 50) ? 200 : 100;
    if (HAL_UART_Transmit(modbus_rtu.huart, data, length, timeout) != HAL_OK) {
        HAL_GPIO_TogglePin(LED_FAULT_GPIO_Port, LED_FAULT_Pin);
        HAL_UART_Abort(modbus_rtu.huart);
        return MODBUS_ERROR_DEVICE_FAILURE;
    }
    
    // Clear receive buffer and restart receiving
    modbus_rtu.rx_length = 0;
    modbus_rtu.frame_received = false;
    memset(modbus_rtu.rx_buffer, 0, MODBUS_MAX_FRAME_SIZE);
    
    // Small delay before restarting receive
    HAL_Delay(1);
    
    // Restart receiving after transmission
    HAL_UART_Receive_IT(modbus_rtu.huart, &modbus_rtu.rx_buffer[0], 1);
    
    return MODBUS_OK;
}

/**
 * @brief Send exception response
 * @param function_code: Original function code
 * @param exception_code: Exception error code
 */
void ModbusRTU_SendException(uint8_t function_code, uint8_t exception_code)
{
    modbus_rtu.tx_buffer[0] = modbus_rtu.slave_id;
    modbus_rtu.tx_buffer[1] = function_code | 0x80;  // Set error bit
    modbus_rtu.tx_buffer[2] = exception_code;
    
    ModbusRTU_SendResponse(modbus_rtu.tx_buffer, 3);
    
}

/**
 * @brief Read register value by address
 * @param address: Register address
 * @retval Register value
 */
uint16_t ModbusRTU_ReadRegister(uint16_t address)
{
    float temp_float;
    
    switch (address) {
        // DalyBMS Status Registers
        case REG_BMS_VOLTAGE:
            return bms_data.voltage;
        case REG_BMS_CURRENT:
            return bms_data.current;
        case REG_BMS_SOC:
            return bms_data.soc;
        case REG_BMS_MAX_CELL_V:
            return bms_data.max_cell;
        case REG_BMS_MIN_CELL_V:
            return bms_data.min_cell;
        case REG_BMS_MAX_CELL_NUM:
            return bms_data.max_cell_index;
        case REG_BMS_MIN_CELL_NUM:
            return bms_data.min_cell_index;
        case REG_BMS_CELL_DIFF:
            return bms_data.cell_diff;
        case REG_BMS_TEMPERATURE:
            return bms_data.temperature_avr;
        case REG_BMS_CONNECTION_STATUS:
            return bms_data.connection_status ? 1 : 0;
        case REG_BMS_CHARGE_DISCHARGE_STATUS:
            return bms_data.charge_discharge_status;
        case REG_BMS_CHARGE_MOS:
            return bms_data.charge_mos ? 1 : 0;
        case REG_BMS_DISCHARGE_MOS:
            return bms_data.discharge_mos ? 1 : 0;
        case REG_BMS_LIFE_CYCLE:
            return bms_data.bms_life_cycle;
        case REG_BMS_RESIDUAL_CAPACITY:
            return bms_data.residual_capacity_mAh;
        case REG_BMS_NUM_CELLS:
            return bms_data.num_cells;
        case REG_BMS_NUM_TEMP_SENSORS:
            return bms_data.num_temp_sensors;
        case REG_BMS_CHARGE_STATUS:
            return bms_data.charge_status ? 1 : 0;
        case REG_BMS_DISCHARGE_STATUS:
            return bms_data.discharge_status ? 1 : 0;
        case REG_BMS_CHARGE_DISCHARGE_CYCLE:
            return bms_data.charge_discharge_cycle;
        case REG_BMS_CELL_BALANCE_ACTIVE:
            return bms_data.cell_balance_active ? 1 : 0;
        case REG_BMS_FAULT_FLAGS:
            return (uint16_t)(bms_data.fault_flags & 0xFFFF);  // Return lower 16 bits
            
        // Cell voltages (0x0014-0x0019)
        case REG_BMS_CELL_VOLTAGE_START ... (REG_BMS_CELL_VOLTAGE_START + 5):
            {
                uint8_t cell_index = address - REG_BMS_CELL_VOLTAGE_START;
                if (cell_index < bms_data.num_cells) {
                    return bms_data.cell_voltage_mv[cell_index];  // Already in mV
                }
                return 0;
            }
            
        // Temperature sensors (0x001A-0x001B)
        case REG_BMS_TEMPERATURE_START ... (REG_BMS_TEMPERATURE_START + 1):
            {
                uint8_t temp_index = address - REG_BMS_TEMPERATURE_START;
                if (temp_index < bms_data.num_temp_sensors) {
                    return bms_data.temperature_c[temp_index];
                }
                return 0;
            }
            
        // Cell balance state (0x001C-0x0021)
        case REG_BMS_CELL_BALANCE_START ... (REG_BMS_CELL_BALANCE_START + 5):
            {
                uint8_t cell_index = address - REG_BMS_CELL_BALANCE_START;
                if (cell_index < bms_data.num_cells) {
                    return bms_data.cell_balance_state[cell_index] ? 1 : 0;
                }
                return 0;
            }
            
        // BMS Thresholds
        case REG_BMS_MAX_CELL_THRESHOLD_1:
            return bms_data.max_cell_threshold_1;
        case REG_BMS_MIN_CELL_THRESHOLD_1:
            return bms_data.min_cell_threshold_1;
        case REG_BMS_MAX_CELL_THRESHOLD_2:
        	return bms_data.max_cell_threshold_2;
        case REG_BMS_MIN_CELL_THRESHOLD_2:
			return bms_data.min_cell_threshold_2;
        case REG_BMS_MAX_PACK_THRESHOLD_1:
            return bms_data.max_pack_threshold_1;
        case REG_BMS_MIN_PACK_THRESHOLD_1:
            return bms_data.min_pack_threshold_1;
        case REG_BMS_MAX_PACK_THRESHOLD_2:
        	return bms_data.max_pack_threshold_2;
        case REG_BMS_MIN_PACK_THRESHOLD_2:
        	return bms_data.min_pack_threshold_2;
            
        // SK60X Data Registers
        case REG_SK60X_V_SET:
			return sk60x_data.v_set;
        case REG_SK60X_I_SET:
            return sk60x_data.i_set;
        case REG_SK60X_V_OUT:
            return sk60x_data.v_out;
        case REG_SK60X_I_OUT:
            return sk60x_data.i_out;
        case REG_SK60X_P_OUT:
            return sk60x_data.p_out;
        case REG_SK60X_V_IN:
            return sk60x_data.v_in;
        case REG_SK60X_I_IN:
            return sk60x_data.i_in;
        case REG_SK60X_TEMP:
            return sk60x_data.temp;
        case REG_SK60X_H_USE:
            return sk60x_data.h_use;
        case REG_SK60X_M_USE:
            return sk60x_data.m_use;
        case REG_SK60X_S_USE:
            return sk60x_data.s_use;
        case REG_SK60X_CVCC:
            return sk60x_data.cvcc;
        case REG_SK60X_ON_OFF:
            return sk60x_data.on_off;
        case REG_SK60X_LOCK:
            return sk60x_data.lock;
        case REG_SK60X_CHARGE_STATE:
            return ChargeControl_GetChargeStateForModbus();
        case REG_SK60X_CHARGE_REQUEST:
            return charge_control.charge_request;
            
        // INA219 Sensor Registers
        case REG_INA219_V_OUT_12V:
            if (INA219_Read_Bus_Voltage(&ina_12v, &temp_float) == HAL_OK) {
                return (uint16_t)(temp_float * 10);
            }
            return 0;
        case REG_INA219_I_OUT_12V:
            if (INA219_Read_Current(&ina_12v, &temp_float) == HAL_OK) {
                return (uint16_t)(temp_float * 10);
            }
            return 0;
        case REG_INA219_P_OUT_12V:
            if (INA219_Read_Power(&ina_12v, &temp_float) == HAL_OK) {
                return (uint16_t)(temp_float * 10);
            }
            return 0;
        case REG_INA219_V_OUT_5V:
            if (INA219_Read_Bus_Voltage(&ina_5v, &temp_float) == HAL_OK) {
                return (uint16_t)(temp_float * 10);
            }
            return 0;
        case REG_INA219_I_OUT_5V:
            if (INA219_Read_Current(&ina_5v, &temp_float) == HAL_OK) {
                return (uint16_t)(temp_float * 10);
            }
            return 0;
        case REG_INA219_P_OUT_5V:
            if (INA219_Read_Power(&ina_5v, &temp_float) == HAL_OK) {
                return (uint16_t)(temp_float * 10);
            }
            return 0;
        case REG_INA219_V_OUT_3V3:
            if (INA219_Read_Bus_Voltage(&ina_3v3, &temp_float) == HAL_OK) {
                return (uint16_t)(temp_float * 10);
            }
            return 0;
        case REG_INA219_I_OUT_3V3:
            if (INA219_Read_Current(&ina_3v3, &temp_float) == HAL_OK) {
                return (uint16_t)(temp_float * 10);
            }
            return 0;
        case REG_INA219_P_OUT_3V3:
            if (INA219_Read_Power(&ina_3v3, &temp_float) == HAL_OK) {
                return (uint16_t)(temp_float * 10);
            }
            return 0;
            
        // Relay Status Registers (Read Only)
        case REG_RELAY_3V3_STATUS:
            return HAL_GPIO_ReadPin(RL_3V3_GPIO_Port, RL_3V3_Pin);
        case REG_RELAY_5V_STATUS:
            return HAL_GPIO_ReadPin(RL_5V_GPIO_Port, RL_5V_Pin);
        case REG_RELAY_12V_STATUS:
            return HAL_GPIO_ReadPin(RL_12V_GPIO_Port, RL_12V_Pin);
        case REG_RELAY_FAUL_STATUS:
        	return HAL_GPIO_ReadPin(FAUL_OUT_GPIO_Port, FAUL_OUT_Pin);
        case REG_RELAY_CHG_STATUS:
            return HAL_GPIO_ReadPin(RL_CHG_GPIO_Port, RL_CHG_Pin);
        case REG_VOLTAGE_THRESHOLD:
            return (uint16_t)(voltage_threshold * 100);  // Scale by 100 (13.5V -> 1350)
            
        // System Registers (0x00F0-0x00FF) - Auto Detect Support
        case REG_DEVICE_ID:
            return modbus_rtu.slave_id;
        case REG_CONFIG_BAUDRATE:
            return modbus_config.baudrate_code;
        case REG_CONFIG_PARITY:
            return modbus_config.parity;
        case REG_CONFIG_STOP_BITS:
            return modbus_config.stop_bits;
        case REG_MODULE_TYPE:
            return 0x0002;  // Power Module
        case REG_FIRMWARE_VERSION:
            return 0x0101;  // v1.01
        case REG_HARDWARE_VERSION:
        	return 0x0101;  // v1.01
        case REG_SYSTEM_STATUS:
            {
                uint16_t status = 0;
                // Bit 0: BMS Connection
                if (bms_data.connection_status) status |= 0x0001;
                // Bit 1: SK60X Status
                if (sk60x_data.on_off) status |= 0x0002;
                // Bit 2: Power relay enabled
                if (relay_power_enabled) status |= 0x0004;
                // Bit 3: Charge relay
                if (HAL_GPIO_ReadPin(RL_CHG_GPIO_Port, RL_CHG_Pin)) status |= 0x0008;
                // Bit 4-7: Reserved
                return status;
            }
        case REG_SYSTEM_ERROR:
            {
                uint16_t error = 0;
                // Bit 0: BMS Error
                if (!bms_data.connection_status) error |= 0x0001;
                // Bit 1: BMS Fault
                if (bms_data.fault_flags != 0) error |= 0x0002;
                // Bit 2: SK60X Error
                if (!sk60x_data.on_off) error |= 0x0004;
                // Bit 3-15: Reserved for future use
                return error;
            }
        case REG_RESET_ERROR_CMD:
            return 0;
            
        default:
            return 0;
    }
}

/**
 * @brief Write value to register
 * @param address: Register address
 * @param value: Value to write
 * @retval Modbus status
 */
ModbusStatus_t ModbusRTU_WriteRegister(uint16_t address, uint16_t value)
{
    switch (address) {            
        // SK60X Control Registers
        case REG_SK60X_V_SET:
            if (SK60X_Set_Voltage(value)) {
                return MODBUS_OK;
            }
            return MODBUS_ERROR_VALUE;
            
        case REG_SK60X_I_SET:
            if (SK60X_Set_Current(value)) {
                return MODBUS_OK;
            }
            return MODBUS_ERROR_VALUE;
            
        case REG_SK60X_ON_OFF:
            if (SK60X_Set_On_Off(value)) {
                return MODBUS_OK;
            }
            return MODBUS_ERROR_VALUE;
            
        case REG_SK60X_LOCK:
            if (SK60X_Set_Lock(value)) {
                return MODBUS_OK;
            }
            return MODBUS_ERROR_VALUE;
            
        case REG_SK60X_CHARGE_REQUEST:
            ChargeControl_HandleRequest(value);
            return MODBUS_OK;
            
        // Relay Control Configuration (writable)
        case REG_VOLTAGE_THRESHOLD:
            if (value >= 1200 && value <= 2000) {  // 12.0V to 20.0V range
                voltage_threshold = (float)value / 100.0f;
                return MODBUS_OK;
            }
            return MODBUS_ERROR_VALUE;
            
        // BMS Thresholds (writable)
        case REG_BMS_MAX_CELL_THRESHOLD_1:
        case REG_BMS_MIN_CELL_THRESHOLD_1:
        case REG_BMS_MAX_CELL_THRESHOLD_2:
        case REG_BMS_MIN_CELL_THRESHOLD_2:
        case REG_BMS_MAX_PACK_THRESHOLD_1:
        case REG_BMS_MIN_PACK_THRESHOLD_1:
        case REG_BMS_MAX_PACK_THRESHOLD_2:
        case REG_BMS_MIN_PACK_THRESHOLD_2:
            // TODO: Implement BMS threshold writing
            return MODBUS_OK;
            
        // System Registers (Writable ones)
        case REG_RESET_ERROR_CMD:
            if (value == 1) {
                // Reset all error flags
                // TODO: Implement actual error flag reset logic
                return MODBUS_OK;
            }
            return MODBUS_ERROR_VALUE;
            
        case REG_CONFIG_BAUDRATE:
            if (value >= 1 && value <= 5) {
                modbus_config.baudrate_code = (ModbusBaudrate_t)value;
                return MODBUS_OK;
            }
            return MODBUS_ERROR_VALUE;
            
        case REG_CONFIG_PARITY:
            if (value <= 2) {
                modbus_config.parity = (ModbusParity_t)value;
                return MODBUS_OK;
            }
            return MODBUS_ERROR_VALUE;
            
        case REG_CONFIG_STOP_BITS:
            if (value == 1 || value == 2) {
                modbus_config.stop_bits = (uint8_t)value;
                return MODBUS_OK;
            }
            return MODBUS_ERROR_VALUE;
            
        default:
            return MODBUS_ERROR_ADDRESS;
    }
}

/**
 * @brief Function Code 03 - Read Holding Registers
 * @param frame: Pointer to received frame
 * @param length: Frame length
 * @retval Modbus status
 */
ModbusStatus_t ModbusRTU_ReadHoldingRegisters(uint8_t *frame, uint16_t length)
{
    if (length != 8) {  // Slave ID + FC + Start Addr (2) + Quantity (2) + CRC (2)
        return MODBUS_ERROR_FRAME;
    }
    
    uint16_t start_address = (frame[2] << 8) | frame[3];
    uint16_t quantity = (frame[4] << 8) | frame[5];
    
    // Check valid register quantity
    if (quantity == 0 || quantity > 125) {
        ModbusRTU_SendException(MODBUS_FC_READ_HOLDING_REGISTERS, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
        return MODBUS_ERROR_VALUE;
    }
    
    // Create response
    modbus_rtu.tx_buffer[0] = modbus_rtu.slave_id;
    modbus_rtu.tx_buffer[1] = MODBUS_FC_READ_HOLDING_REGISTERS;
    modbus_rtu.tx_buffer[2] = quantity * 2;  // Byte count
    
    uint16_t response_index = 3;
    
    for (uint16_t i = 0; i < quantity; i++) {
        uint16_t reg_value = ModbusRTU_ReadRegister(start_address + i);
        modbus_rtu.tx_buffer[response_index++] = (reg_value >> 8) & 0xFF;  // High byte
        modbus_rtu.tx_buffer[response_index++] = reg_value & 0xFF;         // Low byte
    }
    
    // Add small delay for large responses to ensure stable transmission
    if (quantity > 50) {
        HAL_Delay(2);
    }
    
    return ModbusRTU_SendResponse(modbus_rtu.tx_buffer, response_index);
}

/**
 * @brief Function Code 06 - Write Single Register
 * @param frame: Pointer to received frame
 * @param length: Frame length
 * @retval Modbus status
 */
ModbusStatus_t ModbusRTU_WriteSingleRegister(uint8_t *frame, uint16_t length)
{
    if (length != 8) {  // Slave ID + FC + Addr (2) + Value (2) + CRC (2)
        return MODBUS_ERROR_FRAME;
    }
    
    uint16_t address = (frame[2] << 8) | frame[3];
    uint16_t value = (frame[4] << 8) | frame[5];
    
    ModbusStatus_t status = ModbusRTU_WriteRegister(address, value);
    
    if (status != MODBUS_OK) {
        uint8_t exception_code;
        switch (status) {
            case MODBUS_ERROR_ADDRESS:
                exception_code = MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
                break;
            case MODBUS_ERROR_VALUE:
                exception_code = MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;
                break;
            default:
                exception_code = MODBUS_EXCEPTION_SLAVE_DEVICE_FAILURE;
                break;
        }
        ModbusRTU_SendException(MODBUS_FC_WRITE_SINGLE_REGISTER, exception_code);
        return status;
    }
    
    // Echo request as response
    return ModbusRTU_SendResponse(frame, 6);
}

/**
 * @brief Function Code 16 - Write Multiple Registers
 * @param frame: Pointer to received frame
 * @param length: Frame length
 * @retval Modbus status
 */
ModbusStatus_t ModbusRTU_WriteMultipleRegisters(uint8_t *frame, uint16_t length)
{
    if (length < 9) {  // Minimum: Slave ID + FC + Start Addr (2) + Quantity (2) + Byte Count + Data (2) + CRC (2)
        return MODBUS_ERROR_FRAME;
    }
    
    uint16_t start_address = (frame[2] << 8) | frame[3];
    uint16_t quantity = (frame[4] << 8) | frame[5];
    uint8_t byte_count = frame[6];
    
    // Check validity
    if (quantity == 0 || quantity > 123 || byte_count != (quantity * 2)) {
        ModbusRTU_SendException(MODBUS_FC_WRITE_MULTIPLE_REGISTERS, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
        return MODBUS_ERROR_VALUE;
    }
    
    if (length != (9 + byte_count)) {
        return MODBUS_ERROR_FRAME;
    }
    
    // Write each register
    for (uint16_t i = 0; i < quantity; i++) {
        uint16_t reg_address = start_address + i;
        uint16_t reg_value = (frame[7 + i * 2] << 8) | frame[8 + i * 2];
        
        ModbusStatus_t status = ModbusRTU_WriteRegister(reg_address, reg_value);
        
        if (status != MODBUS_OK) {
            uint8_t exception_code;
            switch (status) {
                case MODBUS_ERROR_ADDRESS:
                    exception_code = MODBUS_EXCEPTION_ILLEGAL_DATA_ADDRESS;
                    break;
                case MODBUS_ERROR_VALUE:
                    exception_code = MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE;
                    break;
                default:
                    exception_code = MODBUS_EXCEPTION_SLAVE_DEVICE_FAILURE;
                    break;
            }
            ModbusRTU_SendException(MODBUS_FC_WRITE_MULTIPLE_REGISTERS, exception_code);
            return status;
        }
    }
    
    // Create response
    modbus_rtu.tx_buffer[0] = modbus_rtu.slave_id;
    modbus_rtu.tx_buffer[1] = MODBUS_FC_WRITE_MULTIPLE_REGISTERS;
    modbus_rtu.tx_buffer[2] = (start_address >> 8) & 0xFF;
    modbus_rtu.tx_buffer[3] = start_address & 0xFF;
    modbus_rtu.tx_buffer[4] = (quantity >> 8) & 0xFF;
    modbus_rtu.tx_buffer[5] = quantity & 0xFF;
    
    return ModbusRTU_SendResponse(modbus_rtu.tx_buffer, 6);
}

/**
 * @brief Process received Modbus RTU frame
 * @param frame: Pointer to frame buffer
 * @param length: Frame length
 * @retval Modbus status
 */
ModbusStatus_t ModbusRTU_ProcessFrame(uint8_t *frame, uint16_t length)
{
    // Set processing flag
    modbus_processing = true;
    
    // Check minimum length
    if (length < MODBUS_MIN_FRAME_SIZE) {
#ifdef DEBUG_MODBUS
        HAL_GPIO_TogglePin(LED_FAULT_GPIO_Port, LED_FAULT_Pin);  // Debug: Frame too short
#endif
        modbus_processing = false;
        return MODBUS_ERROR_FRAME;
    }
    
    // Check slave ID
    if (frame[0] != modbus_rtu.slave_id) {
        modbus_processing = false;
        return MODBUS_OK;  // Not our address, ignore
    }
    
    // Check CRC
    if (!ModbusRTU_CheckCRC(frame, length)) {
#ifdef DEBUG_MODBUS
        HAL_GPIO_TogglePin(LED_UART_GPIO_Port, LED_UART_Pin);  // Debug: CRC error
#endif
        modbus_processing = false;
        return MODBUS_ERROR_CRC;
    }
    
    uint8_t function_code = frame[1];
    
    // Check if function code is supported
    uint8_t fc_bit = 0;
    switch (function_code) {
        case MODBUS_FC_READ_HOLDING_REGISTERS:
            fc_bit = 0x01;
            break;
        case MODBUS_FC_WRITE_SINGLE_REGISTER:
            fc_bit = 0x02;
            break;
        case MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
            fc_bit = 0x04;
            break;
        default:
            ModbusRTU_SendException(function_code, MODBUS_EXCEPTION_ILLEGAL_FUNCTION);
            return MODBUS_ERROR_FUNCTION;
    }
    
    if (!(modbus_config.fc_mask & fc_bit)) {
        ModbusRTU_SendException(function_code, MODBUS_EXCEPTION_ILLEGAL_FUNCTION);
        return MODBUS_ERROR_FUNCTION;
    }
    
    // Process according to function code
    ModbusStatus_t result;
    switch (function_code) {
        case MODBUS_FC_READ_HOLDING_REGISTERS:
            result = ModbusRTU_ReadHoldingRegisters(frame, length);
            break;
            
        case MODBUS_FC_WRITE_SINGLE_REGISTER:
            result = ModbusRTU_WriteSingleRegister(frame, length);
            break;
            
        case MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
            result = ModbusRTU_WriteMultipleRegisters(frame, length);
            break;
            
        default:
            ModbusRTU_SendException(function_code, MODBUS_EXCEPTION_ILLEGAL_FUNCTION);
            result = MODBUS_ERROR_FUNCTION;
            break;
    }
    
#ifdef DEBUG_MODBUS
    if (result == MODBUS_OK) {
        HAL_GPIO_WritePin(LED_UART_GPIO_Port, LED_UART_Pin, GPIO_PIN_SET);  // Debug: Success
        HAL_Delay(10);
        HAL_GPIO_WritePin(LED_UART_GPIO_Port, LED_UART_Pin, GPIO_PIN_RESET);
    }
#endif
    
    // Clear processing flag
    modbus_processing = false;
    
    return result;
}

/**
 * @brief UART Rx Complete Callback
 * @param huart: UART handle pointer
 */
void ModbusRTU_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == modbus_rtu.huart) 
    {
        // Update last receive time
        modbus_rtu.last_rx_time = HAL_GetTick();

        if (modbus_rtu.rx_length < MODBUS_MAX_FRAME_SIZE - 1)
        {
            modbus_rtu.rx_buffer[modbus_rtu.rx_length++] = huart->Instance->DR;
            modbus_rtu.frame_received = true;

            // Check if we have received a complete frame
            if (modbus_rtu.rx_length >= 6)
            {
                uint8_t expectedLength = 0;
                if (modbus_rtu.rx_buffer[1] == 3 || modbus_rtu.rx_buffer[1] == 6)
                {
                    expectedLength = 8;
                }
                else if (modbus_rtu.rx_buffer[1] == 4)
                {
                    expectedLength = 8;
                }
                else if (modbus_rtu.rx_buffer[1] == 16)
                {
                    if (modbus_rtu.rx_length >= 7)
                    {
                        expectedLength = 9 + modbus_rtu.rx_buffer[6];
                    }
                }

                // Mark frame as ready for processing (don't process here to avoid conflicts)
                if (modbus_rtu.rx_length >= expectedLength)
                {
                    // Frame is complete, will be processed in ModbusRTU_Process()
                }
            }
        }
        else
        {
            // Buffer overflow, reset
            modbus_rtu.rx_length = 0;
            modbus_rtu.frame_received = false;
        }
        
        // Continue receiving next byte
        HAL_UART_Receive_IT(modbus_rtu.huart, &modbus_rtu.rx_buffer[modbus_rtu.rx_length], 1);
    }
}

/**
 * @brief UART Error Callback
 * @param huart: UART handle pointer
 */
void ModbusRTU_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart == modbus_rtu.huart)
    {
        HAL_GPIO_TogglePin(LED_FAULT_GPIO_Port, LED_FAULT_Pin);
        ModbusRTU_Reset();
    }
}

/**
 * @brief Reset Modbus RTU
 */
void ModbusRTU_Reset(void)
{
    modbus_rtu.rx_length = 0;
    modbus_rtu.frame_received = 0;
    modbus_rtu.last_rx_time = HAL_GetTick();
    HAL_UART_Abort(modbus_rtu.huart);
    HAL_UART_Receive_IT(modbus_rtu.huart, &modbus_rtu.rx_buffer[0], 1);
}

/**
 * @brief Main processing of Modbus RTU (called in main loop)
 */
void ModbusRTU_Process(void)
{
    // Process received frame if available
    if (modbus_rtu.frame_received && modbus_rtu.rx_length > 0) {
        // Check if frame is complete and valid
        if (modbus_rtu.rx_length >= MODBUS_MIN_FRAME_SIZE) {
            ModbusRTU_ProcessFrame(modbus_rtu.rx_buffer, modbus_rtu.rx_length);
        }
        
        // Clear frame after processing
        modbus_rtu.rx_length = 0;
        modbus_rtu.frame_received = false;
        memset(modbus_rtu.rx_buffer, 0, MODBUS_MAX_FRAME_SIZE);
    }
    
    // Handle timeout - reset if no activity for too long
    if (modbus_rtu.rx_length > 0 && (HAL_GetTick() - modbus_rtu.last_rx_time) > 50) {
        modbus_rtu.rx_length = 0;
        modbus_rtu.frame_received = false;
        memset(modbus_rtu.rx_buffer, 0, MODBUS_MAX_FRAME_SIZE);
    }
    
    // Ensure UART is receiving if not busy
    if (modbus_rtu.huart->RxState == HAL_UART_STATE_READY && modbus_rtu.rx_length == 0) {
        HAL_UART_Receive_IT(modbus_rtu.huart, &modbus_rtu.rx_buffer[0], 1);
    }
}

/**
 * @brief Apply new Modbus configuration
 * @retval HAL status
 */
HAL_StatusTypeDef ModbusRTU_ApplyConfig(void)
{
    // Update slave ID
    modbus_rtu.slave_id = modbus_config.slave_id;
    
    // TODO: Implement UART reconfiguration with new baudrate, parity, stop bits
    // This would require deinitializing and reinitializing UART with new parameters
    
    return HAL_OK;
}

/**
 * @brief Convert baudrate code to actual baudrate value
 * @param code: Baudrate code
 * @retval Baudrate value
 */
void ModbusRTU_BaudrateFromCode(ModbusBaudrate_t code)
{
//    uint32_t baudrate;
//    switch (code) {
//        case MODBUS_BAUD_9600:
//            baudrate = 9600;
//            break;
//        case MODBUS_BAUD_19200:
//            baudrate = 19200;
//            break;
//        case MODBUS_BAUD_38400:
//            baudrate = 38400;
//            break;
//        case MODBUS_BAUD_57600:
//            baudrate = 57600;
//            break;
//        case MODBUS_BAUD_115200:
//            baudrate = 115200;
//            break;
//        default:
//            baudrate = 9600;
//            break;
//    }

//    // turn off UART2 before change baudrate
//    CLEAR_BIT(huart2.Instance->CR1, USART_CR1_UE);
//
//    // Update BRR register
//    uint32_t pclk = HAL_RCC_GetPCLK1Freq();   // clock cho USART2 (APB1)
//    huart2.Instance->BRR = (pclk + (baudrate/2U)) / baudrate;
//
//    // Update baudrate
//    huart2.Init.BaudRate = baudrate;
//
//    // turn on UART2
//    SET_BIT(huart2.Instance->CR1, USART_CR1_UE);
}

/**
 * @brief Update data from external sources (BMS, SK60X, INA219)
 * Call this in main loop to update data from sources
 */
void ModbusRTU_UpdateDataFromSources(void)
{
    // Data has been automatically updated through separate tasks
    // This function can be extended to perform other synchronization tasks
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */
