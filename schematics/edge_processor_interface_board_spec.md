# Edge Processor Interface Board — Schematic Specification v0.1
**Project:** AI Robotic System — Leonardo Control Architecture  
**Author:** Leonardo (AI Superagent) for Aaron Edgley / Raxion Systems, LLC.  
**Date:** 2026-05-04  
**Status:** Draft v0.1

---

## 1. Board Purpose
The Edge Processor Interface Board (EPIB) is the central nervous system of the robot stack. It sits between the Sensor Interface Board (SIB), the Motor Driver Board (MDB), and the Edge Processor (Raspberry Pi 4 or Jetson Nano). It manages all data routing, protocol translation, power distribution to the other boards, and the upstream connection to the AI Agent (Leonardo) via WiFi or Ethernet. Every byte of sensor data and every motor command passes through this board.

---

## 2. Position in the Full Stack

```
┌─────────────────────────────────────────────────────┐
│               AI Agent — Leonardo                    │
│         (Base44 Cloud / Vector DB Layer)             │
└───────────────────────┬─────────────────────────────┘
                        │ WiFi / Ethernet (MQTT / WebSocket)
┌───────────────────────▼─────────────────────────────┐
│         Edge Processor Interface Board (EPIB)        │  ← THIS BOARD
│   [Raspberry Pi 4 / Jetson Nano seated on board]    │
│                                                      │
│   UART ←→ SIB (Sensor Interface Board)              │
│   UART ←→ MDB (Motor Driver Board)                  │
│   GPIO ←→ Status / fault lines                      │
│   Power distribution to SIB + MDB                   │
└───────────────────────────────────────────────────────┘
         ↓ UART                        ↑ UART
┌────────────────┐            ┌────────────────────┐
│  Sensor Board  │            │  Motor Driver Board │
│     (SIB)      │            │       (MDB)         │
└────────────────┘            └────────────────────┘
```

---

## 3. Design Constraints
- Physically seats a Raspberry Pi 4 (or Jetson Nano) via 40-pin GPIO header
- Must route power cleanly to Pi + both peripheral boards
- 3D printable substrate (PLA/PETG + Electrifi conductive filament)
- All signal routing through clearly labeled headers — no hidden trace magic
- Target form factor: ~150mm x 120mm (accommodates Pi 4 footprint)
- Open-source design (KiCad), no proprietary IP

---

## 4. Core Components

### 4.1 Edge Processor — Raspberry Pi 4 (4GB) or Jetson Nano
- Seated via standard 40-pin GPIO header (2x20 pin, 2.54mm pitch)
- Runs Linux OS (Raspberry Pi OS or Ubuntu)
- Handles: UART data routing, MQTT broker, AI agent communication, local inference
- USB ports exposed via passthrough headers for camera, peripherals
- Ethernet port exposed via passthrough for wired AI server connection
- **Why Pi 4:** Quad-core ARM, 4GB RAM, USB 3.0, Gigabit Ethernet, proven robotics platform
- **Why Jetson Nano alt:** If local AI inference is needed on-robot (CV, object detection)

### 4.2 40-Pin GPIO Breakout Header
- Full 2x20 pin header matching Raspberry Pi GPIO pinout
- Pi seats directly onto this header (or via short ribbon cable for serviceability)
- All GPIO, I2C, SPI, UART, and power pins broken out to labeled pads
- 3.3V and 5V power pins carry Pi's output to board power bus

### 4.3 UART Multiplexer — 74HC4052 (Dual 4-channel analog mux)
- Pi has limited hardware UART ports (UART0 + UART1)
- 74HC4052 expands to handle both SIB and MDB UART channels
- Select pins (S0, S1) driven by Pi GPIO → switches active UART channel
- Allows Pi to address SIB on one tick, MDB on the next — 50Hz cycle
- Alternatively: use USB-to-UART bridges (CH340G) for each board — simpler, more reliable
  - CH340G #1 → SIB UART
  - CH340G #2 → MDB UART
  - Both appear as /dev/ttyUSB0 and /dev/ttyUSB1 on Pi
- **Recommendation:** CH340G approach — avoids mux timing complexity

### 4.4 USB-to-UART Bridges — CH340G (x2)
- One per peripheral board (SIB, MDB)
- USB Mini-B or USB-C input from Pi USB port
- UART TX/RX output to board header
- Auto-detected by Linux as serial device
- 3.3V logic level output — matches ESP32 on SIB and MDB
- Built-in crystal oscillator — no external clock needed
- Decoupling: 100nF ceramic on VCC pin

### 4.5 Power Input & Distribution

#### Main Input
- XT60 connector — main battery input (2S–4S LiPo, 7.4V–16.8V)
- Polyfuse: 10A resettable fuse on main input
- Reverse polarity protection: P-channel MOSFET on VIN

#### 5V Rail — Raspberry Pi Power
- MP2307 buck converter #1: Vin → 5V @ 3A
- Feedback resistors: R1=39kΩ, R2=22kΩ → Vout ≈ 5.05V
  - Verified: `0.925 × (1 + 39/22) = 0.925 × 2.77 = 2.56V`
  - **Correction:** Use standard formula for MP2307: Vout = 0.925(1 + R1/R2)
  - For 5V: R1/R2 = (5/0.925) - 1 = 4.405 → R1=100kΩ, R2=22.1kΩ (use 22kΩ)
  - Verified: `0.925 × (1 + 100/22) = 0.925 × 5.545 = 5.13V` ✓ (acceptable)
- Pi powered via GPIO pin 2 (5V) and pin 6 (GND) — bypasses micro USB
- Bulk cap: 470µF electrolytic on 5V output rail

#### 3.3V Rail — Logic / Signals
- MP2307 buck converter #2: Vin → 3.3V @ 1A (for board logic, not Pi — Pi generates its own 3.3V internally)
- Feedback resistors: same as SIB — R1=39kΩ, R2=15kΩ → 3.33V ✓
- Powers CH340G bridges, mux ICs, status LEDs, level shifters

#### SIB Power Output Header
- 5V @ 1A to SIB board via 2-pin JST
- SIB's MP2307 steps it down to 3.3V locally
- Fused: 1A polyfuse on output

#### MDB Power Output Header
- Direct battery voltage (7.4V–12V) to MDB motor rail via XT30 connector
- 3A polyfuse on output
- MDB handles its own regulation internally

### 4.6 I2C Expansion — TCA9548A (8-channel I2C Multiplexer)
- Allows Pi to address up to 8 separate I2C buses
- Prevents address conflicts between identical sensors on SIB and MDB
- I2C address: 0x70 (configurable via A0/A1/A2 pins)
- Each channel individually enable/disable via I2C command
- SDA/SCL lines broken out to labeled headers for future expansion
- **Why:** As the robot grows, more I2C devices will be added — this future-proofs the design

### 4.7 Real-Time Clock — DS3231 (I2C RTC)
- Maintains accurate time even when Pi is off or rebooting
- I2C interface (0x68)
- Battery backup: CR2032 coin cell holder on board
- **Why:** Timestamps on sensor data and AI decisions need to be accurate — Pi's system clock drifts without NTP, and NTP requires internet. RTC gives us reliable local time always.

### 4.8 MicroSD Card Slot (secondary)
- For local data logging when WiFi/server connection is unavailable
- SPI interface → Pi GPIO
- Logs sensor JSON packets and AI decision records
- Formatted as FAT32 — Pi reads/writes natively

### 4.9 WiFi / Ethernet Selection
- Raspberry Pi 4 has both onboard — no extra hardware needed
- Ethernet: RJ45 passthrough connector routed to board edge — for wired server connection
- WiFi antenna: Pi's onboard antenna — ensure no metal above Pi on enclosure
- MQTT broker runs on Pi — publishes sensor data, subscribes to AI control commands
- **MQTT Topics:**
  - `raxzion/robot/sensors` — SIB data upstream
  - `raxzion/robot/control` — AI commands downstream to MDB
  - `raxzion/robot/status` — board health, faults, battery level

### 4.10 Battery Monitor — INA219 (I2C)
- Monitors main battery voltage and current draw of entire system
- I2C address: 0x40
- Shunt resistor: 0.05Ω, 2W rated (handles full system current)
- Reports to Pi → AI Agent for battery-aware decision making
- When battery drops below threshold, AI can command reduced speed / return to base

### 4.11 Level Shifter — TXS0108E
- For any 5V ↔ 3.3V signal crossings between Pi GPIO and peripheral boards
- Pi GPIO is 3.3V — most safe, but level shifter adds protection layer
- VCCA = 3.3V (Pi side), VCCB = 5V (peripheral side where needed)

### 4.12 Fan / Thermal Management Header
- 2-pin header for 5V cooling fan (for Pi 4 — runs hot under load)
- PWM-controlled via Pi GPIO — fan speed proportional to CPU temp
- Thermistor header: NTC 10kΩ thermistor for board ambient temp monitoring

### 4.13 UART Debug Header
- 4-pin header: VCC, GND, TX, RX
- Connected to Pi UART0 (GPIO 14/15)
- For SSH-less debugging and OS recovery

### 4.14 Status LEDs
- Green: 5V Pi power rail OK
- Blue: 3.3V logic rail OK
- Yellow: MQTT connected / data flowing
- Red: System fault / battery low
- All 330Ω current limiting resistors
- `I = (3.3V - 2.0V) / 330Ω = 3.9mA` ✓

---

## 5. Pin Allocation (Raspberry Pi 4 GPIO)

| Pi GPIO | Function                    | Connected To                  |
|---------|-----------------------------|-------------------------------|
| 2 (5V)  | Pi power input              | 5V rail from MP2307           |
| 6 (GND) | Pi ground                   | Common ground                 |
| 3 (SDA) | I2C SDA                     | TCA9548A, DS3231, INA219      |
| 5 (SCL) | I2C SCL                     | TCA9548A, DS3231, INA219      |
| 8 (TX)  | UART TX                     | Debug header                  |
| 10 (RX) | UART RX                     | Debug header                  |
| 11      | Fan PWM control             | Fan header                    |
| 12      | Status LED Yellow           | MQTT indicator                |
| 13      | Status LED Red              | Fault indicator               |
| 15      | MUX select S0               | 74HC4052 (if mux used)        |
| 16      | MUX select S1               | 74HC4052 (if mux used)        |
| USB 2.0 | CH340G #1 (SIB)             | /dev/ttyUSB0                  |
| USB 2.0 | CH340G #2 (MDB)             | /dev/ttyUSB1                  |
| ETH     | Wired server connection     | RJ45 passthrough              |

---

## 6. Power Budget

| Component | Voltage | Current (typical) | Power |
|---|---|---|---|
| Raspberry Pi 4 (active, WiFi) | 5V | 600mA | 3.0W |
| CH340G x2 | 3.3V | 20mA | 66mW |
| TCA9548A I2C mux | 3.3V | 1mA | 3.3mW |
| DS3231 RTC | 3.3V | 0.5mA | 1.65mW |
| INA219 battery monitor | 3.3V | 1mA | 3.3mW |
| TXS0108E level shifter | 3.3V | 1mA | 3.3mW |
| Status LEDs x4 | 3.3V | 12mA | 40mW |
| Fan (active cooling) | 5V | 100mA | 500mW |
| SIB board (via header) | 5V | 415mA | 2.07W |
| MDB logic (via header) | 12V | 95mA | 1.14W |
| **5V rail total** | 5V | ~1.12A | ~5.6W |
| **3.3V rail total** | 3.3V | ~36mA | ~120mW |

**5V MP2307 heat:** `(1 - 0.93) × 5.6W = ~0.39W` — acceptable for PLA ✓  
**Recommended battery:** 3S LiPo (11.1V, 5000mAh) — powers EPIB + SIB + MDB comfortably

---

## 7. Software Stack (runs on Pi)

```
┌─────────────────────────────────┐
│     Leonardo AI Agent (Cloud)   │
│     Base44 / Vector DB          │
└───────────┬─────────────────────┘
            │ MQTT over WiFi/Ethernet
┌───────────▼─────────────────────┐
│     MQTT Broker (Mosquitto)     │  ← runs on Pi
├─────────────────────────────────┤
│     Data Router (Python)        │  ← reads /dev/ttyUSB0 (SIB)
│                                 │      writes /dev/ttyUSB1 (MDB)
│                                 │      publishes to MQTT topics
├─────────────────────────────────┤
│     Battery Monitor Daemon      │  ← reads INA219, triggers alerts
├─────────────────────────────────┤
│     RTC Sync Service            │  ← timestamps all packets from DS3231
└─────────────────────────────────┘
```

---

## 8. 3D Printing Considerations

### Trace Widths
- 5V Pi power traces: 8mm wide minimum, doubled where possible (carries ~1A+)
- Battery input traces: 10mm wide (carries up to 5A+)
- Logic signal traces: 2mm wide
- Ground plane: flood fill approach — wide interconnected ground traces on bottom layer

### Physical Layout Zones
- Zone A (top-left): Pi seating area — no traces under Pi GPIO header
- Zone B (top-right): Power input, polyfuse, MOSFETs, buck converters
- Zone C (bottom-left): SIB / MDB communication headers, CH340G bridges
- Zone D (bottom-right): I2C expansion, RTC, battery monitor, status LEDs

### Mechanical
- M2.5 standoff holes at Pi mounting positions (56mm x 49mm pattern)
- Board edge cutouts for Pi USB, Ethernet, and HDMI passthrough
- Fan mount holes: 30mm x 30mm pattern above Pi CPU area

---

## 9. Next Steps
- [ ] Convert to KiCad schematic
- [ ] Design 3D printable board substrate with Pi mounting cutouts
- [ ] Write Python data router script (SIB UART → MQTT → MDB UART)
- [ ] Configure Mosquitto MQTT broker on Pi
- [ ] Set up Leonardo ↔ MQTT bridge on Base44
- [ ] Define AI decision loop: receive sensor data → process → send motor commands
- [ ] Design enclosure / robot chassis that houses all 3 boards

---

*End of Edge Processor Interface Board Spec v0.1*
