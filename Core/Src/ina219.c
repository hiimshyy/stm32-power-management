/*
 * ina219.c
 *
 *  Created on: Aug 19, 2025
 *      Author: tiensy
 */

#include "ina219.h"
#include <string.h>

/**
 * @brief write 16 bit to a register of INA219
 * @param ina: pointer to INA219_t structure
 * @param reg: register address need to write
 * @param value: 16-bit value to write
 * @return HAL_StatusTypeDef: status of the operation
 */
static HAL_StatusTypeDef INA219_WriteRegister(INA219_t *ina, uint8_t reg, uint16_t value) {
    uint8_t data[3];
    data[0] = reg;
    data[1] = (value >> 8) & 0xFF;
    data[2] = value & 0xFF;
    return HAL_I2C_Master_Transmit(ina->hi2c, ina->address, data, 3, 100);
}

/**
 * @brief read 16 bit from a register of INA219
 * @param ina: pointer to INA219_t structure
 * @param reg: register address need to read
 * @param value: pointer to store the read value
 * @return HAL_StatusTypeDef: status of the operation
 */
static HAL_StatusTypeDef INA219_ReadRegister(INA219_t *ina, uint8_t reg, uint16_t *value) {
    uint8_t data[2];
    HAL_StatusTypeDef ret = HAL_I2C_Master_Transmit(ina->hi2c, ina->address, &reg, 1, 100);
    if (ret != HAL_OK) return ret;
    ret = HAL_I2C_Master_Receive(ina->hi2c, ina->address, data, 2, 100);
    if (ret != HAL_OK) return ret;
    *value = (data[0] << 8) | data[1];
    return HAL_OK;
}

/**
 * @brief Initialize INA219 sensor
 * @param ina: pointer to INA219_t structure
 * @param hi2c: pointer to I2C_HandleTypeDef structure
 * @param address: I2C address of the INA219 sensor
 * @param shunt_resistor: value of the shunt resistor in Ohms
 * @param max_expected_current: maximum expected current in Amperes
 */
void INA219_Init(INA219_t *ina, I2C_HandleTypeDef *hi2c, uint8_t address, float shunt_resistor, float max_expected_current) {
    ina->hi2c = hi2c;
    ina->address = address;
    ina->shunt_resistor = shunt_resistor;
    // Calculate current_lsb (base on datasheet)
    // current_lsb = max_expected_current / 32767
    ina->current_lsb = max_expected_current / 32767.0f;
    // Calibration value = 0.04096 / (current_lsb * shunt_resistor)
    ina->calibration_value = (uint16_t)(0.04096f / (ina->current_lsb * shunt_resistor));
    // Power LSB = 20 * current_lsb
    ina->power_lsb = 20.0f * ina->current_lsb;
    // Write calibration value to register
    INA219_WriteRegister(ina, INA219_REG_CALIBRATION, ina->calibration_value);
    // Default config: 32V, 2A, 12-bit ADC, continuous
    INA219_WriteRegister(ina, INA219_REG_CONFIG, 0x399F);
}

/**
 * @brief Read bus voltage from INA219
 * @param ina: pointer to INA219_t structure
 * @param voltage: pointer to store the bus voltage in Volts
 * @return HAL_StatusTypeDef: status of the operation
 */
HAL_StatusTypeDef INA219_Read_Bus_Voltage(INA219_t *ina, float *voltage) {
    uint16_t value;
    HAL_StatusTypeDef ret = INA219_ReadRegister(ina, INA219_REG_BUSVOLTAGE, &value);
    if (ret != HAL_OK) return ret;
    // 1 bit = 4mV, 13 bit available
    *voltage = ((value >> 3) * 4.0f) / 1000.0f;
    return HAL_OK;
}

/**
 * @brief Read shunt voltage from INA219
 * @param ina: pointer to INA219_t structure
 * @param voltage: pointer to store the shunt voltage in Volts
 * @return HAL_StatusTypeDef: status of the operation
 */
HAL_StatusTypeDef INA219_Read_Shunt_Voltage(INA219_t *ina, float *voltage) {
    uint16_t value;
    HAL_StatusTypeDef ret = INA219_ReadRegister(ina, INA219_REG_SHUNTVOLTAGE, &value);
    if (ret != HAL_OK) return ret;
    int16_t sval = (int16_t)value;
    // 1 bit = 10uV
    *voltage = sval * 0.00001f;
    return HAL_OK;
}

/**
 * @brief Read current from INA219
 * @param ina: pointer to INA219_t structure
 * @param current: pointer to store the current in Amperes
 * @return HAL_StatusTypeDef: status of the operation
 */
HAL_StatusTypeDef INA219_Read_Current(INA219_t *ina, float *current) {
    INA219_WriteRegister(ina, INA219_REG_CALIBRATION, ina->calibration_value);
    uint16_t value;
    HAL_StatusTypeDef ret = INA219_ReadRegister(ina, INA219_REG_CURRENT, &value);
    if (ret != HAL_OK) return ret;
    int16_t sval = (int16_t)value;
    *current = sval * ina->current_lsb;
    return HAL_OK;
}

/**
 * @brief Read power from INA219
 * @param ina: pointer to INA219_t structure
 * @param power: pointer to store the power in Watts
 * @return HAL_StatusTypeDef: status of the operation
 */
HAL_StatusTypeDef INA219_Read_Power(INA219_t *ina, float *power) {
    INA219_WriteRegister(ina, INA219_REG_CALIBRATION, ina->calibration_value);
    uint16_t value;
    HAL_StatusTypeDef ret = INA219_ReadRegister(ina, INA219_REG_POWER, &value);
    if (ret != HAL_OK) return ret;
    *power = value * ina->power_lsb;
    return HAL_OK;
}
