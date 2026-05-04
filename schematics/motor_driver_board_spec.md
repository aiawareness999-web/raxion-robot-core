# Motor Driver Board — Schematic Specification v0.1
**Project:** AI Robotic System — Leonardo Control Architecture  
**Author:** Leonardo (AI Superagent) for Aaron Edgley / Raxion Systems, LLC.  
**Date:** 2026-05-04  
**Status:** Draft v0.1

---

## 1. Board Purpose
The Motor Driver Board (MDB) is the output-side companion to the Sensor Interface Board. It receives control signals from the Edge Processor (which relays commands from the AI Agent / Leonardo), and physically drives motors, servos, and actuators on the robot body. Where the SIB listens to the world, the MDB acts on it.

---

## 2. Design Constraints
- Must be manufacturable via conductive-filament 3D printing (Electrifi / Proto-pasta on PLA/PETG substrate)
- Operating voltage: 3.3V logic, 6V–12V motor power input (separate from logic supply)
- Target form factor: ~120mm x 90mm
- Logic and motor power rails must be electrically isolated — prevents motor noise corrupting signal lines
- All components must be through-hole or large SMD (hand-solderable onto 3D printed substrate)
- Open-source design (KiCad), no proprietary IP

---

## 3. System Role in the Full Stack

```
AI Agent (Leonardo)
       ↓
  Vector DB / Decision Layer
       ↓
  Edge Processor (Raspberry Pi / Jetson)
       ↓  [UART / I2C control signals]
  Motor Driver Board  ← (THIS BOARD)
       ↓
  Motors / Servos / Actuators
```

---

## 4. Core Components

### 4.1 Microcontroller — ESP32-WROOM-32
- Same MCU family as SIB for firmware consistency
- Receives control packets from Edge Processor via UART
- Generates PWM signals for motor speed control
- Drives servo control signals (50Hz PWM, 1–2ms pulse width)
- 3.3V logic
- Built-in WiFi allows OTA (over-the-air) firmware updates and optional direct AI agent commands

### 4.2 Motor Driver IC — L298N (Dual H-Bridge)
- Controls up to 2 DC motors bidirectionally (forward/reverse)
- Input voltage: 5V–46V motor supply
- Output current: 2A per channel (4A peak)
- Logic input: 3.3V or 5V compatible
- Built-in protection diodes
- Enable pins allow PWM speed control
- **Why:** Robust, well-documented, through-hole friendly, handles the motor loads we need

### 4.3 Additional Motor Driver — DRV8833 (for smaller motors/servos)
- Dual H-bridge, 1.5A per channel
- 2.7V–10.8V motor supply
- Low RDS(on) — more efficient than L298N for smaller loads
- I2C or PWM control
- Used for precision small motors, gripper actuators, or wheel encoders
- **Why:** Complements L298N — handles lighter loads more efficiently

### 4.4 Servo Control Headers (x4)
- Standard 3-pin servo headers: GND / VCC (5V) / Signal
- Signal lines driven by ESP32 PWM GPIO outputs
- 50Hz PWM frequency, 1ms–2ms pulse width = 0°–180° servo range
- 5V servo power from dedicated filtered rail (not logic 3.3V)
- 100µF bulk cap on servo power rail — absorbs current spikes from servo movement

### 4.5 Power Supply — Dual Rail Design

#### Rail 1: Logic Power (3.3V)
- MP2307 buck converter (same as SIB — consistent design language)
- Input: 6V–12V battery
- Output: 3.3V @ up to 3A
- Powers ESP32, signal lines, ICs
- Feedback resistors: R1=39kΩ, R2=15kΩ → Vout ≈ 3.33V (proven from SIB v0.2)

#### Rail 2: Motor Power (6V–12V)
- Direct from battery input — no regulation (motors are voltage-tolerant)
- Bulk capacitor: 1000µF electrolytic — absorbs motor back-EMF spikes
- Schottky diode: 1N5822 on motor supply input — reverse polarity protection
- Completely separated from logic ground until single-point ground join at input connector

### 4.6 Motor Power Input
- XT30 or XT60 connector (robot battery standard — handles high current cleanly)
- 2-pin JST for logic-only bench power option
- Reverse polarity protection on both inputs
- Input fuse: 5A polyfuse (resettable) on motor rail — protects traces and components

### 4.7 Flyback / Freewheeling Diodes
- 1N4007 diodes across each motor terminal pair
- Absorb voltage spikes generated when motor coils collapse (back-EMF)
- Critical for protecting the L298N and ESP32 from voltage transients
- 8 diodes total (4 per H-bridge channel, 2 channels on L298N)

### 4.8 Current Sensing — INA219 (I2C Power Monitor)
- Measures actual current draw per motor channel
- I2C interface → ESP32 → Edge Processor → AI Agent
- Allows Leonardo to detect motor stall, overload, or unexpected resistance
- Shunt resistor: 0.1Ω, 1W rated
- **Why:** Gives the AI real-time feedback on mechanical load — enables intelligent motor protection

### 4.9 Encoder Inputs (x2)
- 2x 2-pin headers for quadrature encoder signals (A/B channels)
- Pull-up resistors: 10kΩ to 3.3V on each encoder line
- Connected to ESP32 GPIO interrupt pins for precise position/speed tracking
- Encoder data sent back to Edge Processor for closed-loop control

### 4.10 Level Shifter — TXS0108E
- For any 5V signal lines interfacing with 3.3V ESP32 GPIOs
- Specifically: L298N enable/direction lines if driven at 5V
- VCCA = 3.3V, VCCB = 5V

### 4.11 Decoupling Capacitors
- 100nF ceramic on every IC power pin (VCC to GND), as close as possible
- 10µF electrolytic on 3.3V logic rail
- 1000µF electrolytic on motor power rail (absorbs back-EMF and inrush)

### 4.12 UART Communication Header
- 4-pin header: VCC, GND, TX, RX
- Connected to ESP32 UART0
- Receives JSON control packets from Edge Processor
- Also used for firmware flashing and debugging

### 4.13 Emergency Stop (E-Stop) Input
- Dedicated GPIO pin + physical header for external E-Stop button
- Active LOW — pulling to GND immediately disables all motor outputs
- Hardware AND software E-Stop: GPIO interrupt disables PWM + software flag set
- Pull-up resistor: 10kΩ to 3.3V
- **Why:** Safety-critical for any robot — must be able to kill all motion instantly

### 4.14 Status LEDs
- Green: Logic power OK → always on when 3.3V rail is live
- Yellow: Motor power active → on when motor rail is live
- Red: Fault / E-Stop triggered → GPIO controlled
- Blue: Data receiving (control packets incoming) → GPIO controlled
- All with 330Ω current limiting resistors
- LED current: `(3.3V - 2.0V) / 330Ω = 3.9mA` ✓

---

## 5. Pin Allocation (ESP32)

| GPIO | Function                  | Connected To               |
|------|---------------------------|----------------------------|
| 21   | I2C SDA                   | INA219 current sensor      |
| 22   | I2C SCL                   | INA219 current sensor      |
| 25   | Motor A PWM (speed)       | L298N ENA                  |
| 26   | Motor A Direction 1       | L298N IN1                  |
| 27   | Motor A Direction 2       | L298N IN2                  |
| 32   | Motor B PWM (speed)       | L298N ENB                  |
| 33   | Motor B Direction 1       | L298N IN3                  |
| 14   | Motor B Direction 2       | L298N IN4                  |
| 16   | Servo 1 PWM               | Servo header 1             |
| 17   | Servo 2 PWM               | Servo header 2             |
| 18   | Servo 3 PWM               | Servo header 3             |
| 19   | Servo 4 PWM               | Servo header 4             |
| 34   | Encoder A — Channel A     | Encoder 1 interrupt        |
| 35   | Encoder A — Channel B     | Encoder 1 interrupt        |
| 36   | Encoder B — Channel A     | Encoder 2 interrupt        |
| 39   | Encoder B — Channel B     | Encoder 2 interrupt        |
| 23   | E-Stop Input              | E-Stop header (active LOW) |
| 2    | LED Blue (data RX)        | LED + resistor             |
| 4    | LED Red (fault)           | LED + resistor             |
| 1    | UART TX                   | Edge Processor             |
| 3    | UART RX                   | Edge Processor             |

---

## 6. Power Budget

| Component | Voltage | Current (typical) | Power |
|---|---|---|---|
| ESP32-WROOM-32 (active) | 3.3V | 80mA | 264mW |
| L298N (quiescent) | 5V | 36mA | 180mW |
| DRV8833 (quiescent) | 3.3V | 1mA | 3.3mW |
| INA219 | 3.3V | 1mA | 3.3mW |
| TXS0108E | 3.3V | 1mA | 3.3mW |
| Status LEDs x4 | 3.3V | 12mA | 40mW |
| 2x DC Motors (mid load) | 6–12V | 500mA each | 6–12W |
| 4x Servos (active) | 5V | 150mA each | 3W |
| **Logic rail total** | 3.3V | ~95mA | ~314mW |
| **Motor rail total (typical)** | 6–12V | ~1.6A | ~10–19W |

**Logic regulator heat (MP2307):** `~0.10W` — safe for PLA ✓  
**Motor rail:** Direct battery — no regulation heat generated ✓  
**Recommended battery:** 3S LiPo (11.1V, 2200mAh+) — provides both rails cleanly

---

## 7. Control Packet Format
The MDB receives JSON control packets from the Edge Processor via UART at up to 50Hz:

```json
{
  "timestamp": 1714348800000,
  "motor_a": {"speed": 75, "direction": "forward"},
  "motor_b": {"speed": 60, "direction": "reverse"},
  "servo": [90, 45, 120, 0],
  "estop": false
}
```

Response packet sent back to Edge Processor:
```json
{
  "timestamp": 1714348800000,
  "motor_a_current_ma": 423,
  "motor_b_current_ma": 388,
  "encoder_a_ticks": 1042,
  "encoder_b_ticks": 998,
  "fault": false,
  "estop_active": false
}
```

---

## 8. 3D Printing Considerations

### Power Trace Widths (critical on conductive filament)
- Motor power traces: minimum 8mm wide, doubled (parallel) where possible
- Logic power traces: 4mm wide (same as SIB)
- Signal traces: 2mm wide
- Motor traces carry up to 2A — wider is always safer with conductive filament

### Thermal Zones
- L298N generates heat under load: mount on a small aluminum heatsink pad embedded in the print
- Separate motor and logic component zones physically on the board — reduces noise coupling
- MP2307 buck converter: same low-heat profile as SIB (~0.10W) ✓

### Layer Stack (same as SIB)
1. Base structural layer (PLA/PETG) — 3–4 perimeters
2. Conductive trace layer (Electrifi) — recessed channels, wider for power
3. Optional insulating top layer with component windows

### Ground Plane Strategy
- Single ground join point at power input connector
- Motor ground and logic ground tied only at that one point
- Prevents motor switching noise from entering logic ground — critical for signal integrity

---

## 9. Safety Notes
- Always connect E-Stop header before powering motor rail
- Never connect/disconnect motors while board is powered
- Motor back-EMF can spike to 2–3x supply voltage — flyback diodes are non-optional
- PLA substrate: keep motor rail traces away from L298N body — add thermal break gap in print

---

## 10. Next Steps
- [ ] Convert to KiCad schematic file
- [ ] Define 3D printable footprints for L298N, DRV8833, XT60 connector
- [ ] Write ESP32 firmware (PWM generation + UART packet parsing + INA219 reads)
- [ ] Define SIB ↔ MDB communication protocol via Edge Processor
- [ ] Design Edge Processor interface board (board 3)
- [ ] Simulate H-bridge switching with SPICE netlist

---

*End of Motor Driver Board Spec v0.1*
