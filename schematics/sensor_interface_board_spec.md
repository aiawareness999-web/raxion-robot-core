# Sensor Interface Board — Schematic Specification v0.2
**Project:** AI Robotic System — Leonardo Control Architecture  
**Author:** Leonardo (AI Superagent) for Aaron Edgley / Raxion Systems, LLC.  
**Date:** 2026-05-01  
**Status:** Draft v0.2 — Buck Converter Update

### Changelog
- v0.2: Replaced AMS1117-3.3 linear regulator with MP2307 buck converter — thermal risk mitigation for 3D printed PLA substrate

---

## 1. Board Purpose
The Sensor Interface Board (SIB) is the outermost layer of the robot's intelligence stack. It collects raw physical-world data from all attached sensors, digitizes and packages it, then forwards structured data packets to the Edge Processor for transmission to the AI/vector DB layer.

---

## 2. Design Constraints
- Must be manufacturable via conductive-filament 3D printing (e.g., Proto-pasta, Electrifi, or similar)
- Operating voltage: 3.3V logic, 5V power input (USB or LiPo regulated)
- Target form factor: ~100mm x 80mm (adjustable)
- All components must be through-hole or large SMD (hand-solderable onto 3D printed substrate)
- Open-source design (KiCad), no proprietary IP

---

## 3. Core Components

### 3.1 Microcontroller — ESP32-WROOM-32
- Dual-core 240MHz Xtensa LX6
- Built-in WiFi + Bluetooth
- 34 GPIO pins
- Built-in ADC (12-bit, up to 18 channels)
- Supports I2C, SPI, UART, PWM
- 3.3V logic
- **Why:** Powerful, well-documented, open ecosystem, cheap, widely available

### 3.2 External ADC — ADS1115 (16-bit, 4-channel)
- I2C interface (address configurable 0x48–0x4B)
- Programmable gain amplifier (PGA) built in
- 860 samples/sec max
- **Why:** ESP32 ADC has noise issues; ADS1115 gives clean, high-resolution analog reads

### 3.3 I2C Bus
- SDA / SCL lines with 4.7kΩ pull-up resistors to 3.3V
- Supports up to 127 devices on the bus
- Sensor breakouts (IMU, ADS1115, temp sensors) all share this bus

### 3.4 SPI Bus
- MOSI / MISO / SCK / CS lines
- Reserved for high-speed sensors (e.g., camera modules, high-rate IMUs)

### 3.5 IMU — MPU-6050 (Accelerometer + Gyroscope)
- 6-axis motion sensing
- I2C interface
- Interrupt pin connected to ESP32 GPIO for data-ready signaling
- **Why:** Core orientation/motion data for robot balance and movement awareness

### 3.6 Ultrasonic Sensor Header — HC-SR04 Compatible
- 4-pin header: VCC, TRIG, ECHO, GND
- TRIG → ESP32 GPIO output
- ECHO → ESP32 GPIO input (with 1kΩ / 2kΩ voltage divider — HC-SR04 outputs 5V, ESP32 is 3.3V tolerant only)
- **Note:** Voltage divider is critical — protects ESP32 from 5V signal

### 3.7 Camera Interface Header
- For OV2640 or ESP32-CAM compatible module
- 8-bit parallel data bus or UART/SPI depending on module
- Dedicated 3.3V power rail with 100µF bulk cap + 100nF decoupling

### 3.8 Voltage Regulator — MP2307 Buck Converter ⚡ (UPDATED from v0.1)
**Replaces:** AMS1117-3.3 linear regulator

#### Why the change:
- AMS1117 dissipates: `(5V - 3.3V) × 0.415A = 0.705W` as heat
- PLA substrate softens at ~60°C — linear regulator heat poses real warping risk
- MP2307 switching efficiency: ~93% vs ~66% for AMS1117
- Heat dissipated by MP2307 at same load: `(1 - 0.93) × 1.45W = ~0.10W` — 7x less heat

#### MP2307 Specifications:
- Input voltage: 4.75V – 23V
- Output voltage: Adjustable (set to 3.3V via feedback resistors)
- Output current: Up to 3A continuous
- Switching frequency: 340kHz
- Efficiency: up to 93%
- Package: SOP-8 (solderable onto printed pads)

#### Feedback Resistor Values (for 3.3V output):
Using formula: `Vout = 0.925 × (1 + R1/R2)`
- R1 = 39kΩ
- R2 = 15.4kΩ (use 15kΩ standard value — output ≈ 3.32V, acceptable)
- Verified: `0.925 × (1 + 39/15) = 0.925 × 3.6 = 3.33V ✓`

#### Supporting Components:
- Input cap: 100µF electrolytic + 100nF ceramic
- Output cap: 100µF electrolytic + 100nF ceramic
- Inductor: 10µH, rated >1A (e.g., Bourns SRR1260)
- Schottky diode: 1N5819 (fast recovery, low forward voltage drop)
- Bootstrap cap: 100nF ceramic on BST pin

### 3.9 Power Input
- USB Micro-B connector (5V input) OR 2-pin JST connector (from LiPo + boost converter)
- Reverse polarity protection: P-channel MOSFET on VIN
- Power LED indicator: green LED + 330Ω resistor on 3.3V rail
- LED current: `I = (3.3V - 2.0V) / 330Ω = 3.9mA` ✓ (safe for LED and GPIO)

### 3.10 Decoupling Capacitors
- 100nF ceramic capacitor on every IC power pin, placed as close as possible
- 10µF electrolytic on main 3.3V rail

### 3.11 Level Shifter — TXS0108E (8-channel bidirectional)
- For any 5V sensor interfacing with 3.3V ESP32 GPIOs
- OE pin tied to 3.3V (always enabled)
- VCCA = 3.3V, VCCB = 5V

### 3.12 UART Debug Header
- 4-pin header: VCC, GND, TX, RX
- Connected to ESP32 UART0
- Used for firmware flashing and serial debugging

### 3.13 Status LEDs
- Red: Power fault / error indicator → GPIO controlled
- Blue: Data transmitting indicator → GPIO controlled
- Both with 330Ω current limiting resistors
- Current per LED: `(3.3V - 2.0V) / 330Ω = 3.9mA` ✓

---

## 4. Pin Allocation (ESP32)

| GPIO | Function           | Connected To          |
|------|--------------------|-----------------------|
| 21   | I2C SDA            | MPU-6050, ADS1115     |
| 22   | I2C SCL            | MPU-6050, ADS1115     |
| 23   | SPI MOSI           | Camera / high-speed   |
| 19   | SPI MISO           | Camera / high-speed   |
| 18   | SPI SCK            | Camera / high-speed   |
| 5    | SPI CS             | Camera / high-speed   |
| 13   | TRIG (Ultrasonic)  | HC-SR04               |
| 14   | ECHO (Ultrasonic)  | HC-SR04 (via divider) |
| 4    | IMU Interrupt      | MPU-6050 INT pin      |
| 2    | Status LED Blue    | LED + resistor        |
| 15   | Status LED Red     | LED + resistor        |
| 1    | UART TX            | Debug header          |
| 3    | UART RX            | Debug header          |
| 34   | Analog IN 1        | Spare analog (ADC1)   |
| 35   | Analog IN 2        | Spare analog (ADC1)   |

---

## 5. Power Budget (Updated v0.2)

| Component | Voltage | Current | Power |
|---|---|---|---|
| ESP32-WROOM-32 (WiFi active) | 3.3V | 240mA | 792mW |
| MPU-6050 IMU | 3.3V | 3.9mA | 12.9mW |
| ADS1115 ADC | 3.3V | 0.15mA | 0.5mW |
| HC-SR04 Ultrasonic | 5V | 15mA | 75mW |
| OV2640 Camera | 3.3V | 60mA | 198mW |
| TXS0108E Level Shifter | 3.3V | 1mA | 3.3mW |
| Status LEDs x2 | 3.3V | 6mA | 20mW |
| MP2307 (quiescent) | — | 1mA | 3.3mW |
| **TOTAL (peak)** | | **~327mA @ 3.3V** | **~1.45W** |

**Regulator heat (MP2307):** `~0.10W` — safe for PLA substrate ✓  
**Regulator heat (old AMS1117):** `~0.705W` — PLA warp risk ✗

---

## 6. 3D Printing Considerations

### Substrate
- Material: PLA or PETG base layer (structural)
- Conductive traces: Electrifi conductive filament (volume resistivity ~0.006 Ω·cm)
- Trace width minimum: 2mm for signal lines, 4mm for power lines

### Trace Resistance Compensation
- Conductive filament: ~100–1000x higher resistance than copper
- Compensate by: wider traces, shorter runs, parallel trace doubling for power rails
- MP2307 power input/output traces: minimum 6mm wide, doubled if possible

### Component Mounting
- Through-hole components: pressed into printed holes and soldered
- SMD components: placed on flat pads printed into substrate surface
- Conductive epoxy available as alternative to solder where heat may warp PLA

### Layer Stack
1. Base structural layer (PLA/PETG) — 3–4 perimeters thick
2. Conductive trace layer (Electrifi) — single pass, recessed channels
3. Optional insulating top layer (PLA) with component windows cut out

---

## 7. Data Output Format
JSON payload via UART to Edge Processor at 20Hz (configurable):

```json
{
  "timestamp": 1714348800000,
  "accel": {"x": 0.12, "y": -0.03, "z": 9.81},
  "gyro": {"x": 0.002, "y": -0.001, "z": 0.000},
  "ultrasonic_cm": 34.5,
  "analog": [512, 308, 0, 0],
  "camera_frame_ready": true,
  "board_temp_c": 28.4
}
```

---

## 8. Next Steps
- [ ] Convert this spec into a KiCad schematic file
- [ ] Define 3D printable PCB footprints for each component
- [ ] Source MP2307 module (verify SOP-8 package availability)
- [ ] Design Motor Driver Board (SIB companion)
- [ ] Define communication protocol: SIB → Edge Processor → Server
- [ ] Write ESP32 firmware skeleton (sensor polling + JSON serialization)

---

*End of Sensor Interface Board Spec v0.2*
