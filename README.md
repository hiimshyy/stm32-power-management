# 🔋 STM32 Power Management System

## 📌 Project Overview
This is a smart power management module using STM32 as the central controller, capable of monitoring and controlling power outputs (12V, 5V, 3.3V), and communicating with:
- BMS module
- Charger
- Current/Voltage sensors
- AT24C256 EEPROM for logging
- Modbus RTU

---

## 📐 System Architecture

![System Architecture](./Docs/power_manage_flow.png)

---
## 🧰 Main Features

- ✅ Read voltage, current, SoC 
- ✅ Read charging voltage 
- ✅ Read current and voltage used by the system
- ✅ Read temperature from sensors
- ✅ Log data to EEPROM 
- ✅ Control relays for charging and power outputs
- ✅ Provide Modbus RTU interface

---
## 🔌 Modbus RTU Interface

- See register mapping in [`Docs/modbus_register_map.md`](./Docs/modbus_register_map.md)

---