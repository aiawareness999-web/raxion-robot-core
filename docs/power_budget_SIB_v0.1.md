# Sensor Interface Board — Power Budget
**Date:** 2026-05-01  
**Board:** Sensor Interface Board (SIB) v0.1

---

## Component Power Draw (3.3V rail unless noted)

| Component | Voltage | Current (typical) | Power |
|---|---|---|---|
| ESP32-WROOM-32 (active, WiFi on) | 3.3V | 240mA | 792mW |
| ESP32-WROOM-32 (active, WiFi off) | 3.3V | 80mA | 264mW |
| MPU-6050 (IMU) | 3.3V | 3.9mA | 12.9mW |
| ADS1115 (ADC) | 3.3V | 0.15mA | 0.5mW |
| HC-SR04 Ultrasonic | 5V | 15mA | 75mW |
| OV2640 Camera Module | 3.3V | 60mA | 198mW |
| TXS0108E Level Shifter | 3.3V | 1mA | 3.3mW |
| Status LEDs x2 (330Ω @ 3.3V) | 3.3V | 6mA | 20mW |
| Regulator (quiescent) | — | 5mA | 16.5mW |

---

## Totals

- **Peak load (WiFi + camera active):** ~415mA @ 3.3V = ~1.37W + 75mW (5V) = **~1.45W total**
- **Idle load (WiFi off, no camera):** ~110mA @ 3.3V = **~363mW**

---

## ⚠️ Critical Design Note — Regulator Heat

Using AMS1117-3.3 (linear regulator):
- Heat dissipated = (5V - 3.3V) × 0.415A = **0.705W**
- PLA softens at ~60°C — this is a thermal risk on a 3D printed substrate

**Recommendation:** Replace AMS1117-3.3 with **MP2307 buck converter**
- Efficiency: ~93% vs ~66% for linear
- Heat output dramatically reduced
- Same footprint availability in through-hole friendly modules

---

*Calculated using P = I × E and P = I² × R (Ohm's Law fundamentals)*
