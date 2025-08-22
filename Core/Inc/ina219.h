/*
 * ina219.h
 *
 *  Created on: Aug 19, 2025
 *      Author: tiensy
 */

#ifndef INC_INA219_H_
#define INC_INA219_H_

#include "stm32f1xx_hal.h"
#include <stdint.h>

// Địa chỉ I2C của các cảm biến INA219
#define INA219_ADDR_12V   (0x40 << 1) // STM32 HAL dùng 8-bit address
#define INA219_ADDR_5V    (0x41 << 1)
#define INA219_ADDR_3V3   (0x44 << 1)

// Định nghĩa các thanh ghi của INA219
#define INA219_REG_CONFIG         0x00
#define INA219_REG_SHUNTVOLTAGE   0x01
#define INA219_REG_BUSVOLTAGE     0x02
#define INA219_REG_POWER          0x03
#define INA219_REG_CURRENT        0x04
#define INA219_REG_CALIBRATION    0x05

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t address;
    float current_lsb;
    float power_lsb;
    uint16_t calibration_value;
    float shunt_resistor; // Ohm
} INA219_t;

// API
void INA219_Init(INA219_t *ina, I2C_HandleTypeDef *hi2c, uint8_t address, float shunt_resistor, float max_expected_current);
HAL_StatusTypeDef INA219_Read_Bus_Voltage(INA219_t *ina, float *voltage);
HAL_StatusTypeDef INA219_Read_Shunt_Voltage(INA219_t *ina, float *voltage);
HAL_StatusTypeDef INA219_Read_Current(INA219_t *ina, float *current);
HAL_StatusTypeDef INA219_Read_Power(INA219_t *ina, float *power);

#endif /* INC_INA219_H_ */
