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
    .slave_id = 0x01,
    .baudrate_code = MODBUS_BAUD_115200,  // Match UART2 config
    .parity = MODBUS_PARITY_NONE,
    .stop_bits = 1,
    .fc_mask = 0x07  // Support FC 03, 06, 16
};

// External variables từ các module khác
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
    
    // Bắt đầu nhận dữ liệu từng byte
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
 * @brief Check CRC của frame nhận được
 * @param frame: Pointer to frame buffer
 * @param length: Length of frame
 * @retval true nếu CRC đúng, false nếu sai
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
 * @brief Gửi response qua UART
 * @param data: Pointer to data buffer
 * @param length: Length of data
 * @retval Modbus status
 */
ModbusStatus_t ModbusRTU_SendResponse(uint8_t *data, uint16_t length)
{
    // Tính CRC và thêm vào frame
    uint16_t crc = ModbusRTU_CalculateCRC(data, length);
    data[length] = crc & 0xFF;
    data[length + 1] = (crc >> 8) & 0xFF;
    length += 2;
    // 🔥 Tạm dừng UART receive trước khi transmit
    HAL_UART_AbortReceive_IT(modbus_rtu.huart);

    // Gửi response
    if (HAL_UART_Transmit(modbus_rtu.huart, data, length, MODBUS_TIMEOUT_MS) != HAL_OK) {
        return MODBUS_ERROR_DEVICE_FAILURE;
    }
    
    // 🔥 QUAN TRỌNG: Reset hoàn toàn buffer RX sau khi transmit
    modbus_rtu.rx_length = 0;
    memset(modbus_rtu.rx_buffer, 0, MODBUS_MAX_FRAME_SIZE);
    
    // Restart UART receive
    if (modbus_rtu.huart->RxState == HAL_UART_STATE_READY) {
        HAL_UART_Receive_IT(modbus_rtu.huart, &modbus_rtu.rx_buffer[0], 1);
    }
    
    return MODBUS_OK;
}

/**
 * @brief Gửi exception response
 * @param function_code: Function code gốc
 * @param exception_code: Mã lỗi exception
 */
void ModbusRTU_SendException(uint8_t function_code, uint8_t exception_code)
{
    modbus_rtu.tx_buffer[0] = modbus_rtu.slave_id;
    modbus_rtu.tx_buffer[1] = function_code | 0x80;  // Set bit lỗi
    modbus_rtu.tx_buffer[2] = exception_code;
    
    ModbusRTU_SendResponse(modbus_rtu.tx_buffer, 3);
    
    // ModbusRTU_SendResponse đã restart UART receive interrupt
}

/**
 * @brief Đọc giá trị register theo địa chỉ
 * @param address: Địa chỉ register
 * @retval Giá trị register
 */
uint16_t ModbusRTU_ReadRegister(uint16_t address)
{
    float temp_float;
    
    switch (address) {
        // Modbus Configuration Registers
        case REG_SLAVE_ID:
            return modbus_config.slave_id;
        case REG_BAUDRATE_CODE:
            return modbus_config.baudrate_code;
        case REG_PARITY:
            return modbus_config.parity;
        case REG_STOP_BITS:
            return modbus_config.stop_bits;
        case REG_FC_MASK:
            return modbus_config.fc_mask;
        case REG_CONFIG_CRC:
            return 0x0000;  // TODO: Implement config CRC
            
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
        case REG_SK60X_STATUS:
            return sk60x_data.status;
        case REG_SK60X_ON_OFF:
            return sk60x_data.on_off;
        case REG_SK60X_CHARGE_RELAY:
            return HAL_GPIO_ReadPin(RL_CHG_GPIO_Port, RL_CHG_Pin);
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
        case REG_VOLTAGE_THRESHOLD:
            return (uint16_t)(voltage_threshold * 100);  // Scale by 100 (13.5V -> 1350)
            
        default:
            return 0;
    }
}

/**
 * @brief Ghi giá trị vào register
 * @param address: Địa chỉ register
 * @param value: Giá trị cần ghi
 * @retval Modbus status
 */
ModbusStatus_t ModbusRTU_WriteRegister(uint16_t address, uint16_t value)
{
    switch (address) {
        // Modbus Configuration Registers
        case REG_SLAVE_ID:
            if (value >= 1 && value <= 247) {
                modbus_config.slave_id = (uint8_t)value;
                return MODBUS_OK;
            }
            return MODBUS_ERROR_VALUE;
            
        case REG_BAUDRATE_CODE:
            if (value >= 1 && value <= 5) {
                modbus_config.baudrate_code = (ModbusBaudrate_t)value;
                return MODBUS_OK;
            }
            return MODBUS_ERROR_VALUE;
            
        case REG_PARITY:
            if (value <= 2) {
                modbus_config.parity = (ModbusParity_t)value;
                return MODBUS_OK;
            }
            return MODBUS_ERROR_VALUE;
            
        case REG_STOP_BITS:
            if (value == 1 || value == 2) {
                modbus_config.stop_bits = (uint8_t)value;
                return MODBUS_OK;
            }
            return MODBUS_ERROR_VALUE;
            
        case REG_FC_MASK:
            if (value <= 0x07) {
                modbus_config.fc_mask = (uint8_t)value;
                return MODBUS_OK;
            }
            return MODBUS_ERROR_VALUE;
            
        case REG_APPLY_CONFIG:
            if (value == 1) {
                return ModbusRTU_ApplyConfig();
            }
            return MODBUS_ERROR_VALUE;
            
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
            
        case REG_SK60X_CHARGE_RELAY:
            HAL_GPIO_WritePin(RL_CHG_GPIO_Port, RL_CHG_Pin, value ? GPIO_PIN_SET : GPIO_PIN_RESET);
            return MODBUS_OK;
        case REG_SK60X_CHARGE_REQUEST:
            ChargeControl_HandleRequest(value);
            return MODBUS_OK;
            
        // Relay Control Configuration (writable)
        case REG_VOLTAGE_THRESHOLD:
            if (value >= 1000 && value <= 2000) {  // 10.0V to 20.0V range
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
    
    // Kiểm tra số lượng register hợp lệ
    if (quantity == 0 || quantity > 125) {
        ModbusRTU_SendException(MODBUS_FC_READ_HOLDING_REGISTERS, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
        return MODBUS_ERROR_VALUE;
    }
    
    // Tạo response
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
    
    // Echo lại request như response
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
    
    // Kiểm tra tính hợp lệ
    if (quantity == 0 || quantity > 123 || byte_count != (quantity * 2)) {
        ModbusRTU_SendException(MODBUS_FC_WRITE_MULTIPLE_REGISTERS, MODBUS_EXCEPTION_ILLEGAL_DATA_VALUE);
        return MODBUS_ERROR_VALUE;
    }
    
    if (length != (9 + byte_count)) {
        return MODBUS_ERROR_FRAME;
    }
    
    // Ghi từng register
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
    
    // Tạo response
    modbus_rtu.tx_buffer[0] = modbus_rtu.slave_id;
    modbus_rtu.tx_buffer[1] = MODBUS_FC_WRITE_MULTIPLE_REGISTERS;
    modbus_rtu.tx_buffer[2] = (start_address >> 8) & 0xFF;
    modbus_rtu.tx_buffer[3] = start_address & 0xFF;
    modbus_rtu.tx_buffer[4] = (quantity >> 8) & 0xFF;
    modbus_rtu.tx_buffer[5] = quantity & 0xFF;
    
    return ModbusRTU_SendResponse(modbus_rtu.tx_buffer, 6);
}

/**
 * @brief Xử lý frame Modbus RTU nhận được
 * @param frame: Pointer to frame buffer
 * @param length: Frame length
 * @retval Modbus status
 */
ModbusStatus_t ModbusRTU_ProcessFrame(uint8_t *frame, uint16_t length)
{
    // Kiểm tra độ dài tối thiểu
    if (length < MODBUS_MIN_FRAME_SIZE) {
        return MODBUS_ERROR_FRAME;
    }
    
    // Kiểm tra slave ID
    if (frame[0] != modbus_rtu.slave_id) {
        return MODBUS_OK;  // Không phải địa chỉ của mình, bỏ qua
    }
    
    // Kiểm tra CRC
    if (!ModbusRTU_CheckCRC(frame, length)) {
        return MODBUS_ERROR_CRC;
    }
    
    uint8_t function_code = frame[1];
    
    // Kiểm tra function code có được hỗ trợ không
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
    
    // Xử lý theo function code
    switch (function_code) {
        case MODBUS_FC_READ_HOLDING_REGISTERS:
            return ModbusRTU_ReadHoldingRegisters(frame, length);
            
        case MODBUS_FC_WRITE_SINGLE_REGISTER:
            return ModbusRTU_WriteSingleRegister(frame, length);
            
        case MODBUS_FC_WRITE_MULTIPLE_REGISTERS:
            return ModbusRTU_WriteMultipleRegisters(frame, length);
            
        default:
            ModbusRTU_SendException(function_code, MODBUS_EXCEPTION_ILLEGAL_FUNCTION);
            return MODBUS_ERROR_FUNCTION;
    }
}

/**
 * @brief UART Rx Complete Callback
 * @param huart: UART handle pointer
 */
void ModbusRTU_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != modbus_rtu.huart) return;
    
    uint32_t current_time = HAL_GetTick();
    
    // Kiểm tra frame timeout (3.5 character times)
    if (modbus_rtu.rx_length > 0 && (current_time - modbus_rtu.last_rx_time) > 4) {
        modbus_rtu.rx_length = 0; // Reset buffer
    }
    
    modbus_rtu.last_rx_time = current_time;
    
    // 🔥 QUAN TRỌNG: Kiểm tra buffer trước khi tăng index
    if (modbus_rtu.rx_length < MODBUS_MAX_FRAME_SIZE - 1) {
        modbus_rtu.rx_length++;

        // Schedule next byte reception
        if (modbus_rtu.huart->RxState == HAL_UART_STATE_READY) {
            HAL_UART_Receive_IT(modbus_rtu.huart,
                              &modbus_rtu.rx_buffer[modbus_rtu.rx_length],
                              1);
        }
    } else {
        // Buffer overflow - reset
        modbus_rtu.rx_length = 0;
        if (modbus_rtu.huart->RxState == HAL_UART_STATE_READY) {
            HAL_UART_Receive_IT(modbus_rtu.huart, &modbus_rtu.rx_buffer[0], 1);
        }
    }
}

/**
 * @brief Xử lý chính của Modbus RTU (gọi trong main loop)
 */
void ModbusRTU_Process(void)
{
    uint32_t current_time = HAL_GetTick();
    
    // Kiểm tra frame timeout (10ms - an toàn hơn)
    if (modbus_rtu.rx_length > 0 && (current_time - modbus_rtu.last_rx_time) > 10) {
        // Chỉ xử lý nếu frame có độ dài hợp lệ
        if (modbus_rtu.rx_length >= MODBUS_MIN_FRAME_SIZE) {
            ModbusRTU_ProcessFrame(modbus_rtu.rx_buffer, modbus_rtu.rx_length);
        }
        
        modbus_rtu.rx_length = 0;
        memset(modbus_rtu.rx_buffer, 0, MODBUS_MAX_FRAME_SIZE);
    }
    
    if (modbus_rtu.huart->RxState == HAL_UART_STATE_READY && modbus_rtu.rx_length == 0) {
        HAL_UART_Receive_IT(modbus_rtu.huart, &modbus_rtu.rx_buffer[0], 1);
    }
}

/**
 * @brief Áp dụng cấu hình Modbus mới
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
uint32_t ModbusRTU_BaudrateFromCode(ModbusBaudrate_t code)
{
    switch (code) {
        case MODBUS_BAUD_9600:   return 9600;
        case MODBUS_BAUD_19200:  return 19200;
        case MODBUS_BAUD_38400:  return 38400;
        case MODBUS_BAUD_57600:  return 57600;
        case MODBUS_BAUD_115200: return 115200;
        default:                 return 9600;
    }
}

/**
 * @brief Update data from external sources (BMS, SK60X, INA219)
 * Call này trong main loop để cập nhật dữ liệu từ các nguồn
 */
void ModbusRTU_UpdateDataFromSources(void)
{
    // Dữ liệu đã được cập nhật tự động qua các task riêng biệt
    // Function này có thể được mở rộng để thực hiện các tác vụ đồng bộ khác
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */
