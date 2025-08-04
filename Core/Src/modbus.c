// modbus.c
#include "modbus.h"

// Biến dùng chung
uint16_t modbus_registers[MODBUS_REG_COUNT] = {0};
ModbusAccess_t modbus_access[MODBUS_REG_COUNT];
SemaphoreHandle_t modbus_mutex;

// Modbus context riêng biệt cho từng giao tiếp
static eMBHandle uart2_mb_handle;
static eMBHandle cdc_mb_handle;

// UART handle (CubeMX đã tạo sẵn)
extern UART_HandleTypeDef huart2;

// ======================== Modbus Initialization ======================
void Modbus_Init(void)
{
    // Khởi tạo các thanh ghi Modbus với giá trị mặc định
    modbus_registers[REG_SLAVE_ID] = 1;  // Slave ID mặc định
    modbus_registers[REG_BAUDRATE] = 1;  // Mặc định là 9600
    modbus_registers[REG_PARITY] = 0;    // Không parity
    modbus_registers[REG_STOP_BITS] = 1; // 1 stop bit
    modbus_registers[REG_FC_MASK] = 0xFF; // Hỗ trợ tất cả các function code
    modbus_registers[REG_APPLY_CFG] = 0; // Không áp dụng cấu hình ngay
    modbus_registers[REG_CONFIG_CRC] = 0xFFFF; // CRC mặc định
}

void Modbus_Access_Init(void)
{
    // Set default access as Read Only
    for (int i = 0; i < MODBUS_REG_COUNT; i++)
    {
        modbus_access[i] = MB_REG_RO;
    }

    // Modbus Configuration registers (0x0100-0x0106) - Read/Write except apply_config
    for (int i = REG_SLAVE_ID; i <= REG_CONFIG_CRC; i++)
    {
        modbus_access[i] = MB_REG_RW;
    }
    modbus_access[REG_APPLY_CFG] = MB_REG_WO; // Apply config is write-only

    // DalyBMS Status registers (0x0000-0x0023) - Read Only
    for (int i = 0x0000; i <= REG_FAULT_FLAGS; i++)
    {
        modbus_access[i] = MB_REG_RO;
    }

    // BMS Threshold registers (0x0024-0x002B) - Read/Write
    for (int i = REG_MAX_CELL_TH1; i <= REG_MIN_PACK_TH2; i++)
    {
        modbus_access[i] = MB_REG_RW;
    }

    // SK60X Data registers (0x0030-0x003E)
    // Most are Read Only except for control registers
    for (int i = REG_SK60X_V_OUT; i <= REG_SK60X_STATUS; i++)
    {
        modbus_access[i] = MB_REG_RO;
    }
    modbus_access[REG_SK60X_V_SET] = MB_REG_RW;     // Voltage setpoint
    modbus_access[REG_SK60X_I_SET] = MB_REG_RW;     // Current setpoint
    modbus_access[REG_SK60X_ON_OFF] = MB_REG_RW;    // Output control
    modbus_access[REG_SK60X_CHG_RLY] = MB_REG_RW;   // Charge relay control

    // INA219 Sensor registers (0x0040-0x0048) - All Read Only
    for (int i = REG_INA_V12; i <= REG_INA_P33; i++)
    {
        modbus_access[i] = MB_REG_RO;
    }
}

// ======================== UART2 ========================
void Modbus_UART2_Init(void)
{
    eMBErrorCode status = eMBInitExt(&uart2_mb_handle, MB_RTU, modbus_registers[REG_SLAVE_ID], 2,
                                     baudrate_from_code(modbus_registers[REG_BAUDRATE]),
                                     parity_from_code(modbus_registers[REG_PARITY]));
    if (status == MB_ENOERR) eMBEnable(&uart2_mb_handle);
}

// ======================== USB CDC ======================
void Modbus_CDC_Init(void)
{
    eMBErrorCode status = eMBInitExt(&cdc_mb_handle, MB_RTU, modbus_registers[REG_SLAVE_ID], 0,
                                     baudrate_from_code(modbus_registers[REG_BAUDRATE]),
                                     parity_from_code(modbus_registers[REG_PARITY]));
    if (status == MB_ENOERR) eMBEnable(&cdc_mb_handle);
}

// ======================== Modbus Task ==============================
void Modbus_Task(void *argument)
{
    modbus_mutex = xSemaphoreCreateMutex();
    Modbus_UART2_Init();
    Modbus_CDC_Init();

    for (;;)
    {
        eMBPoll(&uart2_mb_handle);  // RS485 Modbus
        eMBPoll(&cdc_mb_handle);    // USB CDC Modbus
        Modbus_Apply_Config();      // Kiểm tra apply_config
        osDelay(2);
    }
}

// =================== Callback đọc/ghi Holding Register =============
static BOOL validate_register_value(USHORT usAddress, uint16_t value)
{
    switch(usAddress)
    {
        case REG_SLAVE_ID:
            return (value >= 1 && value <= 247);
        
        case REG_BAUDRATE:
            return (value >= 1 && value <= 5);
            
        case REG_PARITY:
            return (value <= 2);
            
        case REG_STOP_BITS:
            return (value == 1 || value == 2);
            
        case REG_FC_MASK:
            return (value <= 0x07);  // Chỉ cho phép FC3, FC6, FC16
            
        case REG_APPLY_CFG:
            return (value <= 1);

        // Kiểm tra các threshold
        case REG_MAX_CELL_TH1:
        case REG_MAX_CELL_TH2:
            return (value >= 30 && value <= 42);  // 3.0V - 4.2V
            
        case REG_MIN_CELL_TH1:
        case REG_MIN_CELL_TH2:
            return (value >= 25 && value <= 35);  // 2.5V - 3.5V
            
        case REG_MAX_PACK_TH1:
        case REG_MAX_PACK_TH2:
            return (value >= 180 && value <= 252);  // 18.0V - 25.2V (6 cells)
            
        case REG_MIN_PACK_TH1:
        case REG_MIN_PACK_TH2:
            return (value >= 150 && value <= 210);  // 15.0V - 21.0V

        // SK60X control registers
        case REG_SK60X_V_SET:
            return (value >= 0 && value <= 600);  // 0-60.0V
            
        case REG_SK60X_I_SET:
            return (value >= 0 && value <= 100);  // 0-10.0A
            
        case REG_SK60X_ON_OFF:
        case REG_SK60X_CHG_RLY:
            return (value <= 1);
            
        default:
            return TRUE;  // Cho phép các giá trị khác
    }
}

eMBErrorCode eMBRegHoldingCB(UCHAR *pucRegBuffer, USHORT usAddress,
                             USHORT usNRegs, eMBRegisterMode eMode)
{
    // Kiểm tra địa chỉ hợp lệ
    if (usAddress < 0x0000 || (usAddress + usNRegs) > MODBUS_REG_COUNT)
        return MB_ENOREG;

    if (xSemaphoreTake(modbus_mutex, 5) == pdTRUE)
    {
        USHORT i;
        if (eMode == MB_REG_READ)
        {
            // Kiểm tra quyền đọc
            for (i = 0; i < usNRegs; i++)
            {
                if (modbus_access[usAddress + i] == MB_REG_WO)
                {
                    xSemaphoreGive(modbus_mutex);
                    return MB_EINVAL;  // Không cho phép đọc register write-only
                }
                uint16_t val = modbus_registers[usAddress + i];
                pucRegBuffer[2 * i]     = val >> 8;
                pucRegBuffer[2 * i + 1] = val & 0xFF;
            }
        }
        else if (eMode == MB_REG_WRITE)
        {
            // Kiểm tra quyền ghi
            for (i = 0; i < usNRegs; i++)
            {
                if (modbus_access[usAddress + i] == MB_REG_RO)
                {
                    xSemaphoreGive(modbus_mutex);
                    return MB_EINVAL;  // Không cho phép ghi register read-only
                }
                
                uint16_t newValue = (pucRegBuffer[2 * i] << 8) | pucRegBuffer[2 * i + 1];
                
                // Validate giá trị trước khi ghi
                if (!validate_register_value(usAddress + i, newValue))
                {
                    xSemaphoreGive(modbus_mutex);
                    return MB_EINVAL;
                }
                
                modbus_registers[usAddress + i] = newValue;
            }
        }
        xSemaphoreGive(modbus_mutex);
        return MB_ENOERR;
    }
    return MB_EBUSY;
}

// ==================== Cấu hình lại Modbus nếu có ===================
void Modbus_Apply_Config(void)
{
    static uint16_t last_crc = 0xFFFF;

    if (modbus_registers[0x0105] == 1)
    {
        // Tính CRC mới và so với config_crc (nếu cần)
        // uint16_t crc = CRC16((uint8_t*)&modbus_registers[0x0100], 5*2);

        // Ghi EEPROM nếu cần
        // eeprom_write_block(0x0100, &modbus_registers[0x0100], 6);

        NVIC_SystemReset();  // Reset để áp dụng cấu hình mới
    }
}

uint32_t baudrate_from_code(uint8_t code)
{
    switch (code)
    {
    case 1: return 9600;
    case 2: return 19200;
    case 3: return 38400;
    case 4: return 57600;
    case 5: return 115200;
    default: return 9600;
    }
}

eMBParity parity_from_code(uint8_t code)
{
    switch (code)
    {
    case 0: return MB_PAR_NONE;
    case 1: return MB_PAR_EVEN;
    case 2: return MB_PAR_ODD;
    default: return MB_PAR_NONE;
    }
}

// ======================== PortSerial UART2 ==========================
BOOL xMBPortSerialPutByte_uart2(CHAR ucByte)
{
    return (HAL_UART_Transmit(&huart2, (uint8_t *)&ucByte, 1, 10) == HAL_OK);
}

BOOL xMBPortSerialGetByte_uart2(CHAR *pucByte)
{
    return (HAL_UART_Receive(&huart2, (uint8_t *)pucByte, 1, 10) == HAL_OK);
}

void vMBPortSerialEnable_uart2(BOOL xRxEnable, BOOL xTxEnable)
{
    if (xRxEnable)
    {
        HAL_UART_Receive_IT(&huart2, (uint8_t *)&ucUART2RxByte, 1);  // bạn cần khai báo ucUART2RxByte bên ngoài
    }
    if (xTxEnable)
    {
        // không làm gì ở đây nếu bạn truyền blocking
    }
}

// ======================== PortSerial CDC ============================
BOOL xMBPortSerialPutByte_cdc(CHAR ucByte)
{
    return (CDC_Transmit_FS((uint8_t *)&ucByte, 1) == USBD_OK);
}

BOOL xMBPortSerialGetByte_cdc(CHAR *pucByte)
{
    return CDC_ReadByte(pucByte);  // bạn cần cài FIFO hoặc ringbuffer đọc CDC
}

void vMBPortSerialEnable_cdc(BOOL xRxEnable, BOOL xTxEnable)
{
    // Tùy thuộc bạn có interrupt hay polling USB CDC
    // Nếu dùng FIFO, không cần làm gì tại đây
}
