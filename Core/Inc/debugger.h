/*
 * debugger.h
 *
 *  Created on: Jul 10, 2025
 *      Author: tiensy
 */

#ifndef INC_DEBUGGER_H_
#define INC_DEBUGGER_H_

#include "stm32f1xx_hal.h"

extern UART_HandleTypeDef huart2; // Define your UART handle here

void Debug_Printf(const char *format, ...);




#endif /* INC_DEBUGGER_H_ */
