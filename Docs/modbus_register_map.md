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

| Address  | Name | Unit | Description | Data Type | Scaling | Access |
|----------|------|------|-------------|-----------|---------|--------|
| `0x0000` | voltage_v | V | Voltage of battery pack | `uint8` | /10.0 | Read Only |
| `0x0001` | current_a | A | Current of battery pack | `uint8` | /10.0 | Read Only |
| `0x0002` | soc_percent | % | State of Charge | `uint8` | /10.0 | Read Only |
| `0x0003` | max_cell_v | V | Maximum voltage among cells | `uint8` | /10.0 | Read Only |
| `0x0004` | min_cell_v | V | Minimum voltage among cells | `uint8` | /10.0 | Read Only |
| `0x0005` | max_cell_voltage_num |-| Index of max voltage cell | `uint8` | /10.0 | Read Only |
| `0x0006` | min_cell_voltage_num |-| Index of min voltage cell | `uint8` | /10.0 | Read Only |
| `0x0007` | cell_diff | mV | Voltage difference between cells | `uint8` | /10.0 | Read Only |
| `0x0008` | temperature | °C | Average temperature | `uint8` | - | Read Only |
| `0x0009` | connection_status |-| BMS Connection Status (0/1 = NG/OK) | `bool` |-| Read Only |
| `0x000A` | charge_discharge_status|-| Charge/discharge status flags | `uint8` |-| Read Only  |
| `0x000B` | charge_mos |-| Charge MOSFET status | `bool` |-| Read Only |
| `0x000C` | discharge_mos |-| Discharge MOSFET status | `bool` |-| Read Only |
| `0x000D` | bms_life_cycle | count | Number of BMS power cycles | `uint8` |-| Read Only |
| `0x000E` | residual_capacity_mAh | mAh | Remaining battery capacity | `uint8` |-| Read Only |
| `0x000F` | num_cells | count | Number of battery cells | `uint8` || Read Only  |
| `0x0010` | num_temp_sensors | count | Number of temperature sensors | `uint8 ` |-| Read Only |
| `0x0011` | charge_status |-| Charging in progress (1 = Yes) | `bool` |-| Read Only |
| `0x0012` | discharge_status |-| Discharging in progress (1 = Yes) | `bool` |-| Read Only |
| `0x0013` | charge_discharge_cycle | count | Number of full charge-discharge cycles | `uint8` |-| Read Only  |
| `0x0014–19` | cell_voltage_mv[6] | mV | Voltage of each cell (6 cells) | `uint8[6]` |-| Read Only |
| `0x001A–1B` | temperature_c[2] | °C | Temperature sensor values | `uint8[2]` |-| Read Only |
| `0x001C–21` | cell_balance_state[6] | bit | Per-cell balancing status | `bool[6]` |-| Read Only |
| `0x0022` | cell_balance_active |-| Global balancing status | `bool` |-| Read Only |
| `0x0023` | fault_flags | bitmask | Fault status flags | `uint8` |-| Read Only |
| `0x0024` | max_cell_threshold_1  | V | Max cell voltage threshold 1 | `uint8` | /10.0 | Read/Write |
| `0x0025` | min_cell_threshold_1  | V | Min cell voltage threshold 1 | `uint8` | /10.0 | Read/Write |
| `0x0026` | max_cell_threshold_2  | V | Max cell voltage threshold 2 | `uint8` | /10.0 | Read/Write |
| `0x0027` | min_cell_threshold_2  | V | Min cell voltage threshold 2 | `uint8` | /10.0 | Read/Write |
| `0x0028` | max_pack_threshold_1  | V | Max pack voltage threshold 1 | `uint8` | /10.0 | Read/Write |
| `0x0029` | min_pack_threshold_1  | V | Min pack voltage threshold 1 | `uint8` | /10.0 | Read/Write |
| `0x002A` | max_pack_threshold_2  | V | Max pack voltage threshold 2 | `uint8` | /10.0 | Read/Write |
| `0x002B` | min_pack_threshold_2  | V | Min pack voltage threshold 2 | `uint8` | /10.0 | Read/Write |

---

### 0x0030 - SK60X Data
| Address | Name | Unit | Description | Data Type | Scaling | Access |
|---------|------|------|-------------|-----------|---------|--------|
| `0x0030`| v_set | V | Voltage setpoint | `uint8` | /10.0 | Read/Write |
| `0x0031`| i_set | A | Current setpoint | `uint8` | /10.0 | Read/Write |
| `0x0032`| v_out | V | Output voltage | `uint8` | /10.0 | Read Only |
| `0x0033`| i_out | A | Output current | `uint8` | /10.0 | Read Only |
| `0x0034`| p_out | W | Output power   | `uint8` | /10.0 | Read Only |
| `0x0035`| v_in  | V | Input voltage  | `uint8` | /10.0 | Read Only |
| `0x0036`| i_in  | A | Input current  | `uint8` | /10.0 | Read Only |
| `0x0037`| temp | °C | Temperature of SK60X | `uint8` | - | Read Only |
| `0x0038` | h_use | h | Time used – hours | `uint8` | - | Read Only |
| `0x0039` | m_use | m | Time used – minutes | `uint8` | - | Read Only |
| `0x003A` | s_use | s | Time used – seconds | `uint8` | - | Read Only |
| `0x003B` | status |-| Operational status | `bool` | - | Read Only |
| `0x003C` | on_off |-| Output ON/OFF state | `bool` | - | Read/Write |
| `0x003D` | charge_relay |-| 1 = Enable , 0 = Disable | `bool` | - | Read/Write |
| `0x003E` | charge_state |-| 0 = NOT, 1 = PREPARE, 2 = READY | `bool` | - | Read Only |



### 0x004 - INA219 Sensor Values 

| Address | Name | Unit | Description | Data Type | Scaling | Access    |
|---------|------|------|-------------|-----------|---------|-----------|
| `0x0040` | v_out_12V | V | Voltage of 12V output | `uint8` | /10.0 | Read Only |
| `0x0041` | i_out_12V | A | Current of 12V output | `uint8` | /10.0 | Read Only |
| `0x0041` | p_out_12v | W | Power of 12V output | `uint8` | /10.0 | Read Only |
| `0x0042` | v_out_5V | V | Voltage of 5V output | `uint8` | /10.0 | Read Only |
| `0x0043` | i_out_5v | A | Current of 5V output | `uint8`  | /10.0 | Read Only |
| `0x0041` | p_out_5V | W | Power of 5V output | `uint8` | /10.0 | Read Only |
| `0x0044` | v_out_3V3 | V | Voltage of 3.3V output | `uint8`  | /10.0 | Read Only |
| `0x0045` | i_out_3V3 | A | Current of 3.3V output  | `uint8`  | /10.0 | Read Only |
| `0x0041` | p_out_3V3 | W | Power of 3.3V output | `uint8` | /10.0 | Read Only |

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
|------------------------|---------------|------------|----------|----------------------------|
| Read Battery Voltage   | `0x03`        | `0x0000`   | 1        | Read DalyBMS voltage       |
| Write Relay 12V ON     | `0x06`        | `0x0011`   | 1        | Payload: `0x0001`          |
| Read All INA219 Data   | `0x03`        | `0x0040`   | 6        | Read all 3 channels (V, I) |

---

