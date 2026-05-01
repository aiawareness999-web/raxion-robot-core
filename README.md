# Raxion Robot Core
**Raxion Systems, LLC.**

An AI-integrated robotic system featuring 3D-printable custom circuit boards, vector database memory, and a live AI agent control layer (Leonardo).

---

## Project Architecture

```
Sensors → Sensor Interface Board → Edge Processor → Vector DB → AI Agent → Motor Driver Board → Actuators
```

## Repo Structure

```
/schematics        — Board specifications and schematic files
/firmware          — ESP32 and microcontroller firmware
/models            — 3D printable board substrate models
/docs              — Engineering notes, power budgets, datasheets
```

## Current Status

| Component | Status |
|---|---|
| Sensor Interface Board Spec v0.1 | ✅ Complete |
| Power Budget Analysis | ✅ Complete |
| Motor Driver Board Spec | 🔄 In Progress |
| KiCad Schematic Files | 📋 Planned |
| Firmware (ESP32 SIB) | 📋 Planned |
| Edge Processor Setup | 📋 Planned |
| Vector DB Integration | 📋 Planned |
| AI Agent Control Layer | 📋 Planned |

## Key Design Decisions

- **Microcontroller:** ESP32-WROOM-32 (dual-core, WiFi, I2C/SPI/UART)
- **3D Print Material:** PLA/PETG substrate + Electrifi conductive filament traces
- **Power:** Buck converter (MP2307) replacing linear regulator for thermal efficiency
- **Data Protocol:** JSON over UART at 20Hz → Edge Processor → MQTT → Server
- **AI Platform:** Base44 / Leonardo Agent
- **Design Tool:** KiCad (open source, no licensing restrictions)

---

*Built with Leonardo — AI Superagent by Raxion Systems, LLC.*
