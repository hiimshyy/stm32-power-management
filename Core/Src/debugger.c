/*
 * debugger.c
 *
 *  Created on: Jul 10, 2025
 *      Author: tiensy
 */
#include "debugger.h"
#include <stdarg.h>
#include <stdio.h>

void Debug_Printf(const char *format, ...)
{
    char buffer[128];
    va_list args;
    va_start(args, format);
    int len = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (len < 0) {
        return;
    }

    if (len >= sizeof(buffer)) {
        buffer[sizeof(buffer) - 1] = '\0';
        len = sizeof(buffer) - 1;
    }

    HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, 100);
}

