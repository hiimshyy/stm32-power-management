/*
 * charge_control.h
 *
 *  Created on: Jan 2025
 *      Author: STM32 Charge Control System
 */

#ifndef INC_CHARGE_CONTROL_H_
#define INC_CHARGE_CONTROL_H_

#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>
#include "sk60x.h"
#include "main.h"

/* Defines ------------------------------------------------------------------*/
#define CHARGE_VOLTAGE_THRESHOLD    24.0f   // Ngưỡng điện áp 24V cho sk60x_data.v_in

/* Exported types -----------------------------------------------------------*/
typedef enum {
    CHARGE_STATE_IDLE = 0,          // Không được phép sạc
    CHARGE_STATE_WAITING = 1,       // Đang chờ điều kiện sạc
    CHARGE_STATE_CHARGING = 2       // Đang sạc
} ChargeState_t;

typedef struct {
    bool charge_request;            // Yêu cầu sạc từ thanh ghi 0x003F
    bool charge_relay_enabled;      // Trạng thái relay sạc
    ChargeState_t current_state;    // Trạng thái sạc hiện tại
    bool sk60x_conditions_met;      // Điều kiện SK60X đã thỏa mãn
    uint32_t last_check_time;       // Thời gian kiểm tra cuối cùng
} ChargeControl_t;

/* Exported variables -------------------------------------------------------*/
extern ChargeControl_t charge_control;

/* Exported function prototypes ---------------------------------------------*/

/**
 * @brief Khởi tạo hệ thống charge control
 * @retval HAL status
 */
HAL_StatusTypeDef ChargeControl_Init(void);

/**
 * @brief Xử lý yêu cầu sạc từ Modbus register 0x003F
 * @param request: Yêu cầu sạc (true/false)
 */
void ChargeControl_HandleRequest(bool request);

/**
 * @brief Xử lý logic charge control chính
 * @retval Current charge state
 */
ChargeState_t ChargeControl_Process(void);

/**
 * @brief Kiểm tra điều kiện SK60X
 * @retval true nếu điều kiện thỏa mãn
 */
bool ChargeControl_CheckSK60XConditions(void);

/**
 * @brief Điều khiển relay sạc
 * @param enable: true để bật, false để tắt
 */
void ChargeControl_SetChargeRelay(bool enable);

/**
 * @brief Lấy trạng thái relay sạc hiện tại
 * @retval true nếu relay đang bật
 */
bool ChargeControl_GetChargeRelayStatus(void);

/**
 * @brief Lấy trạng thái sạc hiện tại cho Modbus
 * @retval Charge state (0, 1, 2)
 */
uint8_t ChargeControl_GetChargeStateForModbus(void);


#endif /* INC_CHARGE_CONTROL_H_ */
