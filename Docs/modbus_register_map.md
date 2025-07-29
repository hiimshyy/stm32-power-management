# 📘 Modbus RTU Register Mapping - STM32 Power Management

Hệ thống STM32 giao tiếp với Radxa qua Modbus RTU để cung cấp dữ liệu điện áp, dòng điện, trạng thái sạc, và điều khiển nguồn. Dữ liệu được lưu trữ trong vùng nhớ **Holding Registers (16-bit)**.

## ⚙️ Modbus Configuration

- **Slave ID:** `0x01`
- **Baudrate:** 9600 (hoặc cấu hình của bạn)
- **Parity:** None
- **Stop bit:** 1
- **Function Codes:** 
  - `0x03` – Read Holding Registers
  - `0x06` – Write Single Register
  - `0x10` – Write Multiple Registers

---

## 🧭 Register Table

| Addr (Hex) | Addr (Dec) | Name                        | Unit   | Description                                           | Data Type | Access     |
|------------|-------------|-----------------------------|--------|-------------------------------------------------------|-----------|------------|
| `0x00`     | 0           | Battery Voltage             | mV     | Voltage of battery pack (from DalyBMS)                | `uint16`  | Read Only  |
| `0x01`     | 1           | Battery Current             | mA     | Current of battery pack                               | `int16`   | Read Only  |
| `0x02`     | 2           | Battery SoC                 | %      | State of Charge of battery                            | `uint16`  | Read Only  |
| `0x03`     | 3           | Charging Voltage            | mV     | Charging voltage (from SK60x)                         | `uint16`  | Read Only  |
| `0x04`     | 4           | Charging State              | 0/1    | Charging ON/OFF (1 = charging)                        | `uint16`  | Read Only  |
| `0x05`     | 5           | BMS Warning Code            | bitmask| Warnings from DalyBMS (e.g., overheat, overcurrent)   | `uint16`  | Read Only  |

---

## 🔌 Control Registers

| Addr (Hex) | Addr (Dec) | Name                        | Unit | Description                        | Data Type | Access    |
|------------|-------------|-----------------------------|------|------------------------------------|-----------|-----------|
| `0x10`     | 16          | Charging Control            | 0/1  | 1 = Enable charging, 0 = Disable   | `uint16`  | Read/Write|
| `0x11`     | 17          | Relay 12V Control           | 0/1  | Control 12V output relay           | `uint16`  | Read/Write|
| `0x12`     | 18          | Relay 5V Control            | 0/1  | Control 5V output relay            | `uint16`  | Read/Write|
| `0x13`     | 19          | Relay 3.3V Control          | 0/1  | Control 3.3V output relay          | `uint16`  | Read/Write|

---

## 📟 INA219 Sensor Values

| Addr (Hex) | Addr (Dec) | Name                        | Unit | Description                        | Data Type | Access    |
|------------|-------------|-----------------------------|------|------------------------------------|-----------|-----------|
| `0x20`     | 32          | 12V Voltage Output          | mV   | Voltage of 12V output (INA219)     | `uint16`  | Read Only |
| `0x21`     | 33          | 12V Current Output          | mA   | Current of 12V output              | `uint16`  | Read Only |
| `0x22`     | 34          | 5V Voltage Output           | mV   | Voltage of 5V output               | `uint16`  | Read Only |
| `0x23`     | 35          | 5V Current Output           | mA   | Current of 5V output               | `uint16`  | Read Only |
| `0x24`     | 36          | 3.3V Voltage Output         | mV   | Voltage of 3.3V output             | `uint16`  | Read Only |
| `0x25`     | 37          | 3.3V Current Output         | mA   | Current of 3.3V output             | `uint16`  | Read Only |

---

## 📝 Notes

- All registers are **16-bit wide**.
- Values larger than `uint16_t` should be split across two registers (not currently used).
- Update frequency of data from EEPROM is recommended to be at least every 1s.

---

## 🔁 Example Queries

| Action                 | Function Code | Start Addr | Quantity | Notes                      |
|------------------------|----------------|------------|----------|----------------------------|
| Read Battery Voltage   | `0x03`         | `0x00`     | 1        | Read DalyBMS voltage       |
| Write Relay 12V ON     | `0x06`         | `0x11`     | 1        | Payload: `0x0001`          |
| Read All INA219 Data   | `0x03`         | `0x20`     | 6        | Read all 3 channels (V, I) |

---

