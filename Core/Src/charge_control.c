/*
 * charge_control.c
 *
 *  Created on: Jan 2025
 *      Author: STM32 Charge Control System
 */

#include "charge_control.h"

/* Private variables --------------------------------------------------------*/
ChargeControl_t charge_control = {0};

/* External variables -------------------------------------------------------*/
extern SK60X_Data sk60x_data;

/* Private function prototypes ----------------------------------------------*/

/* Exported functions -------------------------------------------------------*/

/**
 * @brief Khởi tạo hệ thống charge control
 * @retval HAL status
 */
HAL_StatusTypeDef ChargeControl_Init(void)
{
    // Khởi tạo charge control structure
    charge_control.charge_request = false;
    charge_control.charge_relay_enabled = false;
    charge_control.current_state = CHARGE_STATE_IDLE;
    charge_control.sk60x_conditions_met = false;
    charge_control.last_check_time = HAL_GetTick();
    
    // Đảm bảo relay sạc bị tắt ban đầu
    ChargeControl_SetChargeRelay(false);
    
    return HAL_OK;
}

/**
 * @brief Xử lý yêu cầu sạc từ Modbus register 0x003F
 * @param request: Yêu cầu sạc (true/false)
 */
void ChargeControl_HandleRequest(bool request)
{
//	bool current_request = (request != 0); // Chuyển đổi từ uint16_t sang bool
    if (request != charge_control.charge_request) {
        charge_control.charge_request = request;
        
        if (!request) {
            // Nếu request bị tắt, ngay lập tức chuyển về IDLE và tắt relay
            charge_control.current_state = CHARGE_STATE_IDLE;
            ChargeControl_SetChargeRelay(false);
        }
    }
}

/**
 * @brief Kiểm tra điều kiện SK60X
 * @retval true nếu điều kiện thỏa mãn
 */
bool ChargeControl_CheckSK60XConditions(void)
{
    // Điều kiện: sk60x_data.v_in >= 24V && sk60x_data.v_out == sk60x_data.v_set
    bool v_in_ok = (sk60x_data.v_in >= CHARGE_VOLTAGE_THRESHOLD);
    bool v_out_ok = (sk60x_data.v_out == sk60x_data.v_set);
    
    bool conditions_met = v_in_ok && v_out_ok;
    
    return conditions_met;
}

/**
 * @brief Điều khiển relay sạc
 * @param enable: true để bật, false để tắt
 */
void ChargeControl_SetChargeRelay(bool enable)
{
    if (enable != charge_control.charge_relay_enabled) {
        charge_control.charge_relay_enabled = enable;
        
        // Điều khiển GPIO relay (RL_CHG_Pin đã được định nghĩa trong main.h)
        HAL_GPIO_WritePin(GPIOB, RL_CHG_Pin, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
}

/**
 * @brief Lấy trạng thái relay sạc hiện tại
 * @retval true nếu relay đang bật
 */
bool ChargeControl_GetChargeRelayStatus(void)
{
    return charge_control.charge_relay_enabled;
}

/**
 * @brief Lấy trạng thái sạc hiện tại cho Modbus
 * @retval Charge state (0, 1, 2)
 */
uint8_t ChargeControl_GetChargeStateForModbus(void)
{
    // Trả về charge state dựa trên trạng thái hiện tại
    if (charge_control.charge_request) {
        return (uint8_t)charge_control.current_state;
    } else {
        return 0; // Không có request thì luôn trả về 0
    }
}

/**
 * @brief Xử lý logic charge control chính
 * @retval Current charge state
 */
ChargeState_t ChargeControl_Process(void)
{
    uint32_t current_time = HAL_GetTick();
    
    // Chỉ xử lý mỗi 100ms để tránh quá tải
    if (current_time - charge_control.last_check_time < 100) {
        return charge_control.current_state;
    }
    charge_control.last_check_time = current_time;
    
    // State machine cho charge control
    switch (charge_control.current_state) {
        case CHARGE_STATE_IDLE:
            if (charge_control.charge_request) {
                // Đọc dữ liệu từ SK60X
                SK60X_Read_Data();
                
                // Chuyển sang trạng thái waiting
                charge_control.current_state = CHARGE_STATE_WAITING;
            }
            break;
            
        case CHARGE_STATE_WAITING:
            if (!charge_control.charge_request) {
                // Nếu request bị hủy, quay về IDLE
                charge_control.current_state = CHARGE_STATE_IDLE;
                ChargeControl_SetChargeRelay(false);
            } else {
                // Đọc dữ liệu từ SK60X
                SK60X_Read_Data();
                
                // Kiểm tra điều kiện SK60X
                charge_control.sk60x_conditions_met = ChargeControl_CheckSK60XConditions();
                
                if (charge_control.sk60x_conditions_met) {
                    ChargeControl_SetChargeRelay(true);
                    charge_control.current_state = CHARGE_STATE_CHARGING;
                }
            }
            break;
            
        case CHARGE_STATE_CHARGING:
            if (!charge_control.charge_request) {
                // Request bị hủy
                charge_control.current_state = CHARGE_STATE_IDLE;
                ChargeControl_SetChargeRelay(false);
            } else {
                // Đọc dữ liệu từ SK60X
                SK60X_Read_Data();
                
                // Kiểm tra điều kiện SK60X có còn thỏa mãn không
                charge_control.sk60x_conditions_met = ChargeControl_CheckSK60XConditions();
                
                if (!charge_control.sk60x_conditions_met) {
                    ChargeControl_SetChargeRelay(false);
                    charge_control.current_state = CHARGE_STATE_WAITING;
                }
            }
            break;
    }
    
    return charge_control.current_state;
}
