//#include "debugger.h"
//#include <stdarg.h>
//#include <string.h>
//
//static DebugMode_t currentMode = DEBUG_NONE;
//static osMutexId debugMutexHandle;
//
//// ================= USB RING BUFFER =================
//#define USB_RING_BUFFER_SIZE 512
//static uint8_t usb_ring_buffer[USB_RING_BUFFER_SIZE];
//static volatile uint16_t usb_head = 0;
//static volatile uint16_t usb_tail = 0;
//
//static int USB_RingBuffer_IsEmpty(void) {
//    return (usb_head == usb_tail);
//}
//static int USB_RingBuffer_IsFull(void) {
//    return ((usb_head + 1) % USB_RING_BUFFER_SIZE == usb_tail);
//}
//static void USB_RingBuffer_Push(uint8_t c) {
//    if (!USB_RingBuffer_IsFull()) {
//        usb_ring_buffer[usb_head] = c;
//        usb_head = (usb_head + 1) % USB_RING_BUFFER_SIZE;
//    }
//}
//static int USB_RingBuffer_Pop(uint8_t *c) {
//    if (!USB_RingBuffer_IsEmpty()) {
//        *c = usb_ring_buffer[usb_tail];
//        usb_tail = (usb_tail + 1) % USB_RING_BUFFER_SIZE;
//        return 1;
//    }
//    return 0;
//}
//
//// ================= Debug API ========================
//void Debug_Init(void) {
//    osMutexDef(debugMutex);
//    debugMutexHandle = osMutexCreate(osMutex(debugMutex));
//}
//
//void Debug_SetMode(DebugMode_t mode) {
//    currentMode = mode;
//}
//
//DebugMode_t Debug_GetMode(void) {
//    return currentMode;
//}
//
//void Debug_Printf(const char *format, ...)
//{
//    if (currentMode == DEBUG_NONE) return;
//
//    // Chỉ sử dụng USB CDC cho debug - kiểm tra USB sẵn sàng
//    if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) {
//        return; // USB chưa ready, không gửi
//    }
//
//    char buffer[128];
//    va_list args;
//    va_start(args, format);
//    int len = vsnprintf(buffer, sizeof(buffer), format, args);
//    va_end(args);
//
//    if (len < 0) return;
//    if (len >= sizeof(buffer)) {
//        buffer[sizeof(buffer) - 1] = '\0';
//        len = sizeof(buffer) - 1;
//    }
//
//    if (osMutexWait(debugMutexHandle, osWaitForever) == osOK) {
//        for (int i = 0; i < len; i++) {
//            USB_RingBuffer_Push(buffer[i]);
//        }
//        osMutexRelease(debugMutexHandle);
//    }
//}
//
//// ================= USB Process ========================
//void Debug_USB_Process(void) {
//    static uint8_t tx_buf[64];
//    static uint8_t tx_len = 0;
//    static uint32_t last_attempt = 0;
//    static uint8_t retry_count = 0;
//
//    // Chỉ xử lý khi USB sẵn sàng và có dữ liệu cần gửi
//    if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
//
//        uint32_t now = HAL_GetTick();
//
//        // Chỉ thử gửi mỗi 10ms để tránh flood
//        if (now - last_attempt < 10) {
//            return;
//        }
//        last_attempt = now;
//
//        if (tx_len > 0) {
//            // Thử gửi buffer hiện tại
//            USBD_StatusTypeDef status = CDC_Transmit_FS(tx_buf, tx_len);
//            if (status == USBD_OK) {
//                tx_len = 0;
//                retry_count = 0;
//            } else if (status == USBD_BUSY) {
//                retry_count++;
//                if (retry_count > 20) {  // 20 lần retry (~200ms)
//                    tx_len = 0;  // Reset để tránh block
//                    retry_count = 0;
//                }
//            } else {
//                tx_len = 0;
//                retry_count = 0;
//            }
//        }
//
//        // Nếu không có buffer đang chờ, lấy dữ liệu mới từ ring buffer
//        if (tx_len == 0 && !USB_RingBuffer_IsEmpty()) {
//            uint8_t c;
//            tx_len = 0;
//            // Lấy tối đa 64 bytes từ ring buffer
//            while (tx_len < sizeof(tx_buf) && USB_RingBuffer_Pop(&c)) {
//                tx_buf[tx_len++] = c;
//            }
//
//            // Nếu có dữ liệu thì thử gửi ngay
//            if (tx_len > 0) {
//                USBD_StatusTypeDef status = CDC_Transmit_FS(tx_buf, tx_len);
//                if (status == USBD_OK) {
//                    tx_len = 0;
//                    retry_count = 0;
//                    HAL_GPIO_WritePin(GPIOC, LED_FAULT_Pin, GPIO_PIN_RESET);
//                } else if (status != USBD_BUSY) {
//                    tx_len = 0; // Reset nếu có lỗi khác
//                }
//            }
//        }
//    }
//}
//
//// ================= Debug Status ========================
//void Debug_GetStatus(void) {
//    // Gửi trạng thái debug qua USB CDC (bỏ qua kiểm tra trạng thái để force gửi)
//    char status_msg[256];
//    int len = snprintf(status_msg, sizeof(status_msg),
//        "=== DEBUG STATUS ===\r\n"
//        "Debug Mode: %d\r\n"
//        "USB State: %d\r\n"
//        "Ring Buffer Head: %d\r\n"
//        "Ring Buffer Tail: %d\r\n"
//        "Ring Buffer Empty: %s\r\n"
//        "====================\r\n",
//        currentMode,
//        hUsbDeviceFS.dev_state,
//        usb_head,
//        usb_tail,
//        USB_RingBuffer_IsEmpty() ? "YES" : "NO"
//    );
//
//    // Force gửi trực tiếp qua CDC mà không qua ring buffer
//    if (hUsbDeviceFS.dev_state == USBD_STATE_CONFIGURED) {
//        CDC_Transmit_FS((uint8_t*)status_msg, len);
//    }
//}
