//#ifndef INC_DEBUGGER_H_
//#define INC_DEBUGGER_H_
//
//#include "stm32f1xx_hal.h"
//#include "cmsis_os.h"
//#include <stdio.h>
//
//// ===== Debug modes =====
//typedef enum {
//    DEBUG_NONE = 0,
//    DEBUG_USB    // Chỉ sử dụng USB CDC để tránh xung đột với Modbus RTU
//} DebugMode_t;
//
//#if 1
//#include "usbd_cdc_if.h"
//#include "usbd_def.h"
//#include "usbd_core.h"
//#endif
//
//// API
//void Debug_Init(void);
//void Debug_SetMode(DebugMode_t mode);
//DebugMode_t Debug_GetMode(void);
//void Debug_Printf(const char *format, ...);
//void Debug_USB_Process(void);
//void Debug_GetStatus(void);  // Function để kiểm tra trạng thái debug
//
//#endif /* INC_DEBUGGER_H_ */
