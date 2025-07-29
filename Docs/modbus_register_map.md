# 📘 Modbus RTU Register Mapping - STM32 Power Management

This document defines the Modbus RTU register map used for communication between STM32 and an external master device (e.g. Radxa) via RS485. The values are cached from BMS and SK60X modules and stored in EEPROM (AT24C256).

## ⚙️ Modbus Configuration

| Parameter      | Value         |
|----------------|---------------|
| Slave ID       | `0x01`        |
| Baudrate       | `9600 bps`    |
| Data bits      | `8`           |
| Parity         | `None`        |
| Stop bits      | `1`           |
| Supported FCs  | `0x03`, `0x06`, `0x10` |

---

## 📑 Register Map
### 0x0000 – DalyBMS Status

| Address  | Name | Unit | Description | Data Type | Access |
|---------|------|------|-------------|-----------|--------|
| `0x0000-1` | voltage_v | V | Voltage of battery pack | `float` | Read Only |
| `0x0002-3` | current_a | A | Current of battery pack | `float` | Read Only |
| `0x0004-5` | soc_percent | % | State of Charge | `float` | Read Only |
| `0x0006-7` | max_cell_v | V | Maximum voltage among cells | `float` | Read Only |
| `0x0008-9` | min_cell_v | V | Minimum voltage among cells | `float` | Read Only |
| `0x000A` | max_cell_voltage_num | | Index of max voltage cell | `uint8` | Read Only |
| `0x000B` | min_cell_voltage_num | | Index of min voltage cell | `uint8` | Read Only |
| `0x000C` | cell_diff | mV | Voltage difference between cells | `uint8` | Read Only |
| `0x000D-E` | temperature | °C | Average temperature | `float` | Read Only |
| `0x000F` | connection_status | 0/1 | BMS Connection Status (0/1 = ) | `bool` | Read Only |
| `0x0010` | charge_discharge_status| bitmask | Charge/discharge status flags               | uint8     | Read Only  |
| `0x0011` | charge_mos || Charge MOSFET status | `bool` | Read Only |
| `0x0012` | discharge_mos || Discharge MOSFET status | `bool` | Read Only |
| `0x0013` | bms_life_cycle | count | Number of BMS power cycles | `uint8` | Read Only |
| `0x0014–15` | residual_capacity_mAh | mAh | Remaining battery capacity | float | Read Only |
| `0x0016` | num_cells | count | Number of battery cells | `uint8` | Read Only  |
| `0x0017` | num_temp_sensors | count | Number of temperature sensors | `uint8 ` | Read Only |
| `0x0018` | charge_status || Charging in progress (1 = Yes) | `bool` | Read Only |
| `0x0019` | discharge_status || Discharging in progress (1 = Yes) | `bool` | Read Only |
| `0x001A` | charge_discharge_cycle | count | Number of full charge-discharge cycles | `uint8` | Read Only  |
| `0x001B–20` | cell_voltage_mv[6] | mV | Voltage of each cell (6 cells) | `uint8[6]` | Read Only |
| `0x0021–22` | temperature_c[2] | °C | Temperature sensor values | `uint8[2]` | Read Only |
| `0x0023–28` | cell_balance_state[6] | bit | Per-cell balancing status | `bool[6]` | Read Only |
| `0x0029` | cell_balance_active | | Global balancing status | `bool` | Read Only |
| `0x002A` | fault_flags | bitmask | Fault status flags | `uint8` | Read Only |

---

### 0x0030 - SK60X Data
| Address | Name | Unit | Description | Data Type | Access |
|---------|------|------|-------------|-----------|--------|
| `0x0030–31`| v_set | V | Voltage setpoint | `float` | Read/Write |
| `0x0032-33`| i_set | A | Current setpoint | `float` | Read/Write |
| `0x0034–35`| v_out | V | Output voltage | `float` | Read Only |
| `0x0036–37`| i_out | A | Output current | `float` | Read Only |
| `0x0038–39`| p_out | W | Output power   | `float` | Read Only |
| `0x003A–3B`| v_in  | V | Input voltage  | `float` | Read Only |
| `0x003C–3D`| i_in  | A | Input current  | `float` | Read Only |
| `0x003E–3F`| temp | °C | Temperature of SK60X | `float` | Read Only |
| `0x0040` | h_use | h | Time used – hours | `uint8` | Read Only |
| `0x0041` | m_use | m | Time used – minutes | `uint8` | Read Only |
| `0x0042` | s_use | s | Time used – seconds | `uint8` | Read Only |
| `0x0043` | status | | Operational status | `bool` | Read Only |
| `0x0044` | on_off | | Output ON/OFF state | `bool` | Read/Write |
| `0x0045` | charge_relay | | 1 = Enable , 0 = Disable | `bool` | Read/Write |
| `0x0046` | charge_state | | 0 = NOT, 1 = PREPARE, 2 = READY | `bool` | Read/Write |



### 0x005 - INA219 Sensor Values 

| Address | Name | Unit | Description | Data Type | Access    |
|---------|------|------|-------------|-----------|-----------|
| `0x0050` | 12V Voltage Output | mV | Voltage of 12V output | `uint8` | Read Only |
| `0x0051` | 12V Current Output | mA | Current of 12V output | `uint8` | Read Only |
| `0x0052` | 5V Voltage Output | mV | Voltage of 5V output | `uint8` | Read Only |
| `0x0053` | 5V Current Output | mA | Current of 5V output | `uint16`  | Read Only |
| `0x0054` | 3.3V Voltage Output | mV | Voltage of 3.3V output | `uint8`  | Read Only |
| `0x0055` | 3.3V Current Output | mA | Current of 3.3V output  | `uint8`  | Read Only |

---

## 📝 Notes

- **Data Type:**
  - `float`: 2 registers (4 bytes), IEEE 754 format.
  - `uint8`: 1 register (1 byte, upper or lower byte used).
  - `bool`: 1 bit, stored in 1 register.

- **Access:**
  - `Read Only`: Chỉ đọc, không ghi được qua Modbus RTU.
  - `Read/Write`: Cho phép đọc và ghi qua Modbus RTU.

- **Note:** Tất cả địa chỉ được viết ở định dạng Hex (`0x`), theo chuẩn của Modbus.


---

## 🔁 Example Queries

| Action                 | Function Code | Start Addr | Quantity | Notes                      |
|------------------------|----------------|------------|----------|----------------------------|
| Read Battery Voltage   | `0x03`         | `0x00`     | 1        | Read DalyBMS voltage       |
| Write Relay 12V ON     | `0x06`         | `0x11`     | 1        | Payload: `0x0001`          |
| Read All INA219 Data   | `0x03`         | `0x20`     | 6        | Read all 3 channels (V, I) |

---

