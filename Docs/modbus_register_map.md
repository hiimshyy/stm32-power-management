# 📘 Modbus RTU Register Mapping - STM32 Power Management

This document defines the Modbus RTU register map used for communication between STM32 and an external master device (e.g. Radxa) via RS485. The values are cached from BMS and SK60X modules and stored in EEPROM (AT24C256).
  ## ⚙️ System Configuration & Information

  | Address   | Name | Description | Data Type | Values / Notes | Default | Access |
  |-----------|------|-------------|-----------|----------------|---------|--------|
  | `0x0100`  | `slave_id` | Modbus slave address | `uint8` | `1–247` | `0x01` | Read/Write |
  | `0x0101`  | `baudrate_code`| UART baudrate setting (encoded) | `uint8` | `1=9600`, `2=19200`, `3=38400`, `4=57600`, `5=115200` | `1` | Read/Write |
  | `0x0102`  | `parity`| UART parity | `uint8` | `0=None`, `1=Even`, `2=Odd` | `0` | Read/Write |
  | `0x0103`  | `stop_bits` | UART stop bits | `uint8` | `1` or `2` | `1` | Read/Write |
  | `0x0104`  | `module_type` | Type of module | `uint8` | `2 = Power module` | `2` | Read Only |
  | `0x0105`  | `fw_version` | Firmware version (encoded) | `uint16` | e.g. `0x0101 = v1.01` | - | Read only |
  | `0x0106`  | `hw_version` | Hardware version (encoded) | `uint16` | e.g. `0x0101 = v1.01` | - | Read only |
  | `0x0107`  | `system_stt` | System status flags | `uint16` | `0=Normal`,`1=Warning`,`2=Charging` | `0` | Read only |
  | `0x0108`  | `system_err` | System error flags | `int16` | `0:COMM_ERR`,`1:OVER_VOLT`,`2=UNDER_VOLT`,`3=OVER_CURRENT`,`4:OVER_TEMP` | - | Read only |
  | `0x0109`  | `reset_err` | Reset error counters/flags | `uint16` | `0=IDLE`,`1=RESET` | `0` | Read/Write |

  ---

## 📑 Register Map
### 0x0000 – DalyBMS Status

| Address  | Name | Unit | Description | Data Type | Scaling | Access |
|----------|------|------|-------------|-----------|---------|--------|
| `0x0000` | voltage_v | V | Voltage of battery pack | `uint16` | /10.0 | Read Only |
| `0x0001` | current_a | A | Current of battery pack | `uint16` | /10.0 | Read Only |
| `0x0002` | soc_percent | % | State of Charge | `uint16` | /10.0 | Read Only |
| `0x0003` | max_cell_v | mV | Maximum voltage among cells | `uint16` | - | Read Only |
| `0x0004` | min_cell_v | mV | Minimum voltage among cells | `uint16` | - | Read Only |
| `0x0005` | max_cell_index |-| Index of max voltage cell | `uint8` | - | Read Only |
| `0x0006` | min_cell_index |-| Index of min voltage cell | `uint8` | - | Read Only |
| `0x0007` | cell_diff | mV | Voltage difference between cells | `uint8` | - | Read Only |
| `0x0008` | temperature | °C | Average temperature | `uint8` | - | Read Only |
| `0x0009` | connection_status |-| BMS Connection Status (0/1 = NG/OK) | `bool` |-| Read Only |
| `0x000A` | charge_discharge_status|-| Charge/discharge status flags | `uint8` |-| Read Only  |
| `0x000B` | charge_mos |-| Charge MOSFET status | `bool` |-| Read Only |
| `0x000C` | discharge_mos |-| Discharge MOSFET status | `bool` |-| Read Only |
| `0x000D` | bms_life_cycle | count | Number of BMS power cycles | `uint8` |-| Read Only |
| `0x000E` | residual_capacity_mAh | mAh | Remaining battery capacity | `uint8` |-| Read Only |
| `0x000F` | num_cells | count | Number of battery cells | `uint8` | - | Read Only  |
| `0x0010` | num_temp_sensors | count | Number of temperature sensors | `uint8 ` |-| Read Only |
| `0x0011` | charge_status |-| Charging in progress (1 = Yes) | `bool` |-| Read Only |
| `0x0012` | discharge_status |-| Discharging in progress (1 = Yes) | `bool` |-| Read Only |
| `0x0013` | charge_discharge_cycle | count | Number of full charge-discharge cycles | `uint8` |-| Read Only  |
| `0x0014–19` | cell_voltage_mv[6] | mV | Voltage of each cell (6 cells) | `uint8[6]` |-| Read Only |
| `0x001A–1B` | temperature_c[2] | °C | Temperature sensor values | `uint8[2]` |-| Read Only |
| `0x001C–21` | cell_balance_state[6] | bit | Per-cell balancing status | `bool[6]` |-| Read Only |
| `0x0022` | cell_balance_active |-| Global balancing status | `bool` |-| Read Only |
| `0x0023` | fault_flags | bitmask | Fault status flags | `uint8` |-| Read Only |
| `0x0024` | max_cell_threshold_1  | mV | Max cell voltage threshold 1 | `uint16` | - | Read/Write |
| `0x0025` | min_cell_threshold_1  | mV | Min cell voltage threshold 1 | `uint16` | - | Read/Write |
| `0x0026` | max_cell_threshold_2  | mV | Max cell voltage threshold 2 | `uint16` | - | Read/Write |
| `0x0027` | min_cell_threshold_2  | mV | Min cell voltage threshold 2 | `uint16` | - | Read/Write |
| `0x0028` | max_pack_threshold_1  | mV | Max pack voltage threshold 1 | `uint16` | - | Read/Write |
| `0x0029` | min_pack_threshold_1  | mV | Min pack voltage threshold 1 | `uint16` | - | Read/Write |
| `0x002A` | max_pack_threshold_2  | mV | Max pack voltage threshold 2 | `uint16` | - | Read/Write |
| `0x002B` | min_pack_threshold_2  | mV | Min pack voltage threshold 2 | `uint16` | - | Read/Write |

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
| `0x003B` | cvcc |-| CVCC Operational status | `bool` | - | Read Only |
| `0x003C` | on_off |-| Output ON/OFF state | `bool` | - | Read/Write |
| `0x003D` | lock |-| Disable all button | `bool` | - | Read/Write |
| `0x003E` | charge_state |-| 0 = IDLE, 1 = WAITING, 2 = CHARGE | `uint16` | - | Read Only |
| `0x003F` | charge_request |-| 0 = NOT, 1 = REQUEST| `bool` | - | Read/Write |



### 0x0040 - INA219 Sensor Values 

| Address | Name | Unit | Description | Data Type | Scaling | Access    |
|---------|------|------|-------------|-----------|---------|-----------|
| `0x0040` | v_out_12V | V | Voltage of 12V output | `uint8` | /10.0 | Read Only |
| `0x0041` | i_out_12V | A | Current of 12V output | `uint8` | /10.0 | Read Only |
| `0x0042` | p_out_12V | W | Power of 12V output | `uint8` | /10.0 | Read Only |
| `0x0043` | v_out_5V | V | Voltage of 5V output | `uint8` | /10.0 | Read Only |
| `0x0044` | i_out_5v | A | Current of 5V output | `uint8`  | /10.0 | Read Only |
| `0x0045` | p_out_5V | W | Power of 5V output | `uint8` | /10.0 | Read Only |
| `0x0046` | v_out_3V3 | V | Voltage of 3.3V output | `uint8`  | /10.0 | Read Only |
| `0x0047` | i_out_3V3 | A | Current of 3.3V output  | `uint8`  | /10.0 | Read Only |
| `0x0048` | p_out_3V3 | W | Power of 3.3V output | `uint8` | /10.0 | Read Only |

### 0x0049 - Relay state
| Address | Name | Unit | Description | Data Type | Scaling | Access    |
|---------|------|------|-------------|-----------|---------|-----------|
| `0x0049` | rl_3V3 | - | State of 3V3 relay | `uint8` | - | Read Only |
| `0x004A` | rl_5V | - | State of 5V relay | `uint8` | - | Read Only |
| `0x004B` | rl_12V | - | State of 12V relay | `uint8`  | - | Read Only |
| `0x004C` | rl_faul | - | State of Faul relay | `uint8` | - | Read Only |
| `0x004D` | rl_charge | - | State of Charge relay | `uint8` | - | Read Only |
| `0x004F` | use_v_ths | - | Setting voltage usage threshold | `uint16`  | /100.0 | Read/Write |

---

## 📝 Notes

- **Data Types:**
  - `uint8`: 1 register (16-bit) where only 1 byte is used (typically lower byte).
  - `uint16`: 1 full 16-bit register.
  - `bool`: 1-bit flag stored inside a full 16-bit register.
  - `array[N]`: Consecutive registers with N elements.

- **Scaling:**
  - Some values are scaled (e.g., `/10.0`), meaning the raw register value must be divided by 10 to get the actual unit.
  - For example, `voltage_v = 168 → 16.8V`.

- **Access Types:**
  - `Read Only`: Value is readable via Modbus RTU but cannot be written.
  - `Read/Write`: Value is both readable and writable.
  - `Write Only`: Value is writable but not readable.

- **Address Format:**
  - All register addresses are written in hexadecimal (`0x` prefix) and represent **Modbus register offsets**, not absolute addresses (e.g., Modbus register 40001 → `0x0000`).

---

## 🔁 Example Queries

These examples demonstrate reading/writing registers using Modbus RTU protocol. CRC values are included for 9600 baud, 8N1, and Slave ID `0x01`.

> Use tools like **ModbusTool**, **ModScan**, or **pymodbus** for testing.

---
### 📘 1. Read battery voltage from DalyBMS

- **Register**: `0x0000`  
- **Function**: `0x03` (Read Holding Register)

**Request:** 01 03 00 00 00 01 84 0A

**Response (value = 168):** 01 03 02 00 A8 B8 44

---
### 🔌 2. Turn SK60X ON (`on_off = 1` at `0x003C`)

**Request:** 01 06 00 3C 00 01 D9 E2

**Response:** 01 06 00 3C 00 01 D9 E2

---

### 📊 3. Read INA219 3 rails (9 registers from `0x0040`)

**Request:** 01 03 00 40 00 09 45 D1

**Response:** 01 03 12 00 C8 00 64 00 96 ... XX XX
> (example values: voltage = 12.0V, current = 6.4A, power = 9.6W, etc.)

---
### 🔐 4. Change Modbus Slave ID to `0x02` (write `0x02` to `0x0100`)

**Request:** 01 06 01 00 00 02 28 0B

---

### ⚙️ 5. Apply configuration (`0x0105 ← 0x0001`)

**Request:** 01 06 01 05 00 01 18 C3

---

### 🧰 6. Write multiple config registers at once

Write 5 registers: `slave_id`, `baudrate`, `parity`, `stop_bits`, `fc_mask`

| Value     | Meaning        |
|-----------|----------------|
| `0x02`    | Slave ID       |
| `0x05`    | Baudrate = 115200 |
| `0x00`    | Parity = None  |
| `0x01`    | Stop bits = 1  |
| `0x07`    | Allow FC 3,6,16 |

**Request:** 01 10 01 00 00 05 0A 00 02 00 05 00 00 00 01 00 07 6B B9

---

### 🔍 7. Read System Information (Module Type, Name, Version)

Read 4 registers from `0x00F0`: Device ID, Firmware Version, System Status, System Error

**Request:** 01 03 00 F0 00 04 04 3F

**Response:** 01 03 08 00 01 01 01 00 0F 00 00 XX XX

> Example values: Device ID = 1, Firmware = v1.01, Status = 0x000F (all OK), Error = 0x0000 (no errors)

---

### 📋 8. Read Module Identification (Auto-detect support)

Read module type and name: `0x00F7-0x00F9` (3 registers)

**Request:** 01 03 00 F7 00 03 F4 3C

**Response:** 01 03 06 00 02 44 57 50 57 XX XX

> Values: Module Type = 0x0002 (Power Module), Name = "DWPW" (DalyBms Power)

---


