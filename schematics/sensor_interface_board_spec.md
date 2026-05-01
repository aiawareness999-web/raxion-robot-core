# Sensor Interface Board — Schematic Specification v0.1
**Project:** AI Robotic System — Leonardo Control Architecture  
**Author:** Leonardo (AI Superagent) for Aaron Edgley  
**Date:** 2026-04-28  
**Status:** Draft v0.1

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
- Built-in WiFi + Bluetooth (useful for wireless sensor data forwarding)
- 34 GPIO pins
- Built-in ADC (12-bit, up to 18 channels)
- Supports I2C, SPI, UART, PWM
- 3.3V logic
- **Why:** Powerful, well-documented, open ecosystem, cheap, widely available

### 3.2 External ADC — ADS1115 (16-bit, 4-channel)
- I2C interface (address configurable 0x48–0x4B)
- Programmable gain amplifier (PGA) built in
- 860 samples/sec max
- **Why:** ESP32 ADC has noise issues; ADS1115 gives clean, high-resolution analog reads for sensors like flex sensors, pressure sensors, IR analog outputs

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
- **Note:** Voltage divider is critical here — protects ESP32

### 3.7 Camera Interface Header
- For OV2640 or ESP32-CAM compatible module
- 8-bit parallel data bus or UART/SPI depending on module
- Dedicated 3.3V power rail with 100µF bulk cap + 100nF decoupling

### 3.8 Voltage Regulator — AMS1117-3.3
- Input: 5V (USB or regulated LiPo)
- Output: 3.3V @ up to 1A
- Bulk cap: 10µF electrolytic on input and output
- Decoupling: 100nF ceramic on input and output

### 3.9 Power Input
- USB Micro-B connector (5V input) OR 2-pin JST connector (from LiPo + boost converter)
- Reverse polarity protection: P-channel MOSFET or Schottky diode on VIN
- Power LED indicator: green LED + 330Ω resistor on 3.3V rail

### 3.10 Decoupling Capacitors
- 100nF ceramic capacitor on every IC power pin (VCC to GND), placed as close as possible
- 10µF electrolytic on main 3.3V rail

### 3.11 Crystal Oscillator
- ESP32-WROOM-32 module includes internal oscillator — external crystal not required
- ADS1115 uses internal oscillator — no external clock needed

### 3.12 Level Shifter — TXS0108E (8-channel bidirectional)
- For any 5V sensor interfacing with 3.3V ESP32 GPIOs
- OE pin tied to 3.3V (always enabled)
- VCCA = 3.3V, VCCB = 5V

### 3.13 UART Debug Header
- 4-pin header: VCC, GND, TX, RX
- Connected to ESP32 UART0
- Used for firmware flashing and serial debugging

### 3.14 Status LEDs
- Red: Power fault / error indicator → GPIO controlled
- Blue: Data transmitting indicator → GPIO controlled
- Both with 330Ω current limiting resistors

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

## 5. 3D Printing Considerations

### Substrate
- Material: PLA or PETG base layer (structural)
- Conductive traces: Electrifi conductive filament (volume resistivity ~0.006 Ω·cm) or Proto-pasta
- Trace width minimum: 2mm for signal lines, 4mm for power lines (to handle resistance of conductive filament)

### Trace Resistance Compensation
- Conductive filament has ~100–1000x higher resistance than copper
- Compensate by: wider traces, shorter runs, parallel trace doubling for power lines
- Add resistor correction values into schematic annotations

### Component Mounting
- Through-hole components preferred — can be pressed into printed holes and soldered
- SMD components: place on flat pads printed into the substrate surface
- Use conductive epoxy as alternative to solder where heat may warp substrate

### Layer Stack
1. Base structural layer (PLA/PETG) — 3–4 perimeters thick
2. Conductive trace layer (Electrifi) — single pass, recessed channels
3. Optional insulating top layer (PLA) with component windows cut out

---

## 6. Data Output Format
The SIB packages sensor readings into a structured JSON payload sent via UART to the Edge Processor every 50ms (20Hz default, configurable):

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

## 7. Next Steps
- [ ] Convert this spec into a KiCad schematic file
- [ ] Define 3D printable PCB footprints for each component
- [ ] Simulate power consumption budget
- [ ] Design Motor Driver Board (SIB companion — receives control signals)
- [ ] Define communication protocol between SIB → Edge Processor → Server

---

*End of Sensor Interface Board Spec v0.1*
