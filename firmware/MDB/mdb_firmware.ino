/**
 * ============================================================
 * Motor Driver Board (MDB) — Firmware Skeleton v0.1
 * Project: AI Robotic System — Leonardo Control Architecture
 * Author:  Leonardo (AI Superagent) for Aaron Edgley / Raxzion Systems, LLC.
 * Date:    2026-06-25
 * MCU:     ESP32-WROOM-32
 * ============================================================
 *
 * OVERVIEW:
 * This firmware runs on the ESP32 aboard the Motor Driver Board.
 * It receives JSON control packets from the Edge Processor (EPIB)
 * via UART, drives DC motors and servos, reads encoder ticks and
 * INA219 current feedback, then sends a status packet back upstream.
 *
 * DATA FLOW:
 *   EPIB / Pi ──UART──► ESP32 ──► L298N ──► DC Motors
 *                              ──► DRV8833 ──► Small Motors
 *                              ──► Servo PWM ──► Servos x4
 *   Encoders ──► ESP32 ──► Status JSON ──UART──► EPIB / Pi
 *   INA219   ──I2C──► ESP32 ──► Status JSON
 *
 * INCOMING CONTROL PACKET (from Pi @ up to 50Hz):
 * {
 *   "timestamp": 1718000000000,
 *   "motor_a": {"speed": 75, "direction": "forward"},
 *   "motor_b": {"speed": 60, "direction": "reverse"},
 *   "servo": [90, 45, 120, 0],
 *   "estop": false
 * }
 *
 * OUTGOING STATUS PACKET (to Pi @ 20Hz):
 * {
 *   "timestamp": 1718000000000,
 *   "motor_a_current_ma": 423,
 *   "motor_b_current_ma": 388,
 *   "encoder_a_ticks": 1042,
 *   "encoder_b_ticks": 998,
 *   "fault": false,
 *   "estop_active": false
 * }
 *
 * DEPENDENCIES (install via Arduino Library Manager):
 *   - ArduinoJson v6+
 *   - Adafruit INA219
 *   - ESP32Servo
 * ============================================================
 */

// ─── Libraries ───────────────────────────────────────────────
#include <Arduino.h>
#include <Wire.h>
#include <ArduinoJson.h>         // ArduinoJson v6+
#include <Adafruit_INA219.h>     // Adafruit INA219 current sensor
#include <ESP32Servo.h>          // ESP32-compatible servo library

// ─── Pin Definitions ─────────────────────────────────────────
// I2C
#define PIN_I2C_SDA         21
#define PIN_I2C_SCL         22

// L298N — Motor A
#define PIN_MOTOR_A_PWM     25   // ENA — speed control
#define PIN_MOTOR_A_IN1     26   // IN1 — direction
#define PIN_MOTOR_A_IN2     27   // IN2 — direction

// L298N — Motor B
#define PIN_MOTOR_B_PWM     32   // ENB — speed control
#define PIN_MOTOR_B_IN1     33   // IN3 — direction
#define PIN_MOTOR_B_IN2     14   // IN4 — direction

// Servos (50Hz PWM)
#define PIN_SERVO_1         16
#define PIN_SERVO_2         17
#define PIN_SERVO_3         18
#define PIN_SERVO_4         19

// Encoders (interrupt-capable pins)
#define PIN_ENC_A_CH_A      34   // Encoder A — Channel A
#define PIN_ENC_A_CH_B      35   // Encoder A — Channel B
#define PIN_ENC_B_CH_A      36   // Encoder B — Channel A
#define PIN_ENC_B_CH_B      39   // Encoder B — Channel B

// E-Stop (active LOW)
#define PIN_ESTOP           23

// Status LEDs
#define PIN_LED_GREEN        0   // Logic power OK (always on when live)
#define PIN_LED_YELLOW      12   // Motor power active
#define PIN_LED_RED          4   // Fault / E-Stop
#define PIN_LED_BLUE         2   // Data RX heartbeat

// UART (to/from EPIB)
#define PIN_UART_TX          1
#define PIN_UART_RX          3

// ─── Configuration ───────────────────────────────────────────
#define UART_BAUD           115200
#define I2C_FREQ            400000
#define LOOP_HZ             20
#define LOOP_INTERVAL_MS    (1000 / LOOP_HZ)   // 50ms
#define PWM_FREQ            20000              // 20kHz PWM — inaudible
#define PWM_RESOLUTION      8                  // 8-bit: 0–255
#define SERVO_MIN_US        500                // Minimum pulse width (µs)
#define SERVO_MAX_US        2500               // Maximum pulse width (µs)
#define ESTOP_DEBOUNCE_MS   20
#define WATCHDOG_TIMEOUT_MS 500                // Stop motors if no cmd in 500ms

// ─── PWM Channels (ESP32 LEDC) ───────────────────────────────
#define PWM_CH_MOTOR_A      0
#define PWM_CH_MOTOR_B      1

// ─── Objects ─────────────────────────────────────────────────
Adafruit_INA219 ina219_a(0x40);   // Motor A current sensor
Adafruit_INA219 ina219_b(0x41);   // Motor B current sensor (alt address)
Servo servo1, servo2, servo3, servo4;

// ─── State Variables ─────────────────────────────────────────
bool ina219_ok       = false;
bool estop_active    = false;
bool fault           = false;
volatile long enc_a_ticks = 0;
volatile long enc_b_ticks = 0;
unsigned long last_loop_ms    = 0;
unsigned long last_cmd_ms     = 0;
uint32_t      packet_count    = 0;

// ─── Forward Declarations ────────────────────────────────────
void init_i2c();
void init_motors();
void init_servos();
void init_encoders();
void init_leds();
void init_estop();
bool init_current_sensors();
void process_control_packet(const JsonDocument& doc);
void set_motor(uint8_t pwm_ch, uint8_t in1, uint8_t in2,
               int speed, const char* direction);
void stop_motor(uint8_t pwm_ch, uint8_t in1, uint8_t in2);
void stop_all_motors();
void set_servos(const JsonArray& angles);
void trigger_estop();
void clear_estop();
void transmit_status(float current_a_ma, float current_b_ma);
void blink_led(uint8_t pin, uint16_t ms);
void check_watchdog();

// ─── Encoder ISRs ────────────────────────────────────────────
void IRAM_ATTR isr_enc_a() { enc_a_ticks++; }
void IRAM_ATTR isr_enc_b() { enc_b_ticks++; }

// ─── E-Stop ISR ──────────────────────────────────────────────
void IRAM_ATTR isr_estop() {
  static unsigned long last_trigger = 0;
  unsigned long now = millis();
  if (now - last_trigger > ESTOP_DEBOUNCE_MS) {
    estop_active = true;
    last_trigger = now;
  }
}

// ════════════════════════════════════════════════════════════
// SETUP
// ════════════════════════════════════════════════════════════
void setup() {
  Serial.begin(UART_BAUD);
  delay(100);
  Serial.println("[MDB] Motor Driver Board v0.1 booting...");

  init_leds();
  init_i2c();
  init_motors();
  init_servos();
  init_encoders();
  init_estop();
  ina219_ok = init_current_sensors();

  // Signal boot complete
  digitalWrite(PIN_LED_GREEN, HIGH);
  digitalWrite(PIN_LED_YELLOW, HIGH);

  last_cmd_ms = millis();   // Init watchdog timer
  Serial.println("[MDB] Boot complete. Awaiting control packets.");
}

// ════════════════════════════════════════════════════════════
// MAIN LOOP — runs at 20Hz
// ════════════════════════════════════════════════════════════
void loop() {
  unsigned long now = millis();
  if (now - last_loop_ms < LOOP_INTERVAL_MS) return;
  last_loop_ms = now;

  // ── Check E-Stop ─────────────────────────────────────────
  if (estop_active) {
    stop_all_motors();
    digitalWrite(PIN_LED_RED, HIGH);
    fault = true;
  }

  // ── Watchdog — stop motors if no command received ─────────
  check_watchdog();

  // ── Read incoming UART packet ─────────────────────────────
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      StaticJsonDocument<512> doc;
      DeserializationError err = deserializeJson(doc, line);
      if (err) {
        Serial.println("[MDB] JSON parse error: " + String(err.c_str()));
      } else {
        // Check for software E-Stop command
        if (doc.containsKey("estop") && doc["estop"].as<bool>()) {
          trigger_estop();
        } else if (estop_active && doc.containsKey("estop_clear")) {
          clear_estop();
        } else if (!estop_active) {
          process_control_packet(doc);
          last_cmd_ms = millis();
          blink_led(PIN_LED_BLUE, 5);
        }
      }
    }
  }

  // ── Read current sensors ──────────────────────────────────
  float current_a_ma = 0.0;
  float current_b_ma = 0.0;
  if (ina219_ok) {
    current_a_ma = ina219_a.getCurrent_mA();
    current_b_ma = ina219_b.getCurrent_mA();
  }

  // ── Transmit status packet upstream ──────────────────────
  transmit_status(current_a_ma, current_b_ma);
  packet_count++;
}

// ════════════════════════════════════════════════════════════
// INITIALIZATION
// ════════════════════════════════════════════════════════════

void init_i2c() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(I2C_FREQ);
  Serial.println("[MDB] I2C initialized at 400kHz.");
}

void init_motors() {
  // Configure LEDC PWM channels
  ledcSetup(PWM_CH_MOTOR_A, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(PWM_CH_MOTOR_B, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(PIN_MOTOR_A_PWM, PWM_CH_MOTOR_A);
  ledcAttachPin(PIN_MOTOR_B_PWM, PWM_CH_MOTOR_B);

  // Direction pins
  pinMode(PIN_MOTOR_A_IN1, OUTPUT);
  pinMode(PIN_MOTOR_A_IN2, OUTPUT);
  pinMode(PIN_MOTOR_B_IN1, OUTPUT);
  pinMode(PIN_MOTOR_B_IN2, OUTPUT);

  // Start stopped
  stop_all_motors();
  Serial.println("[MDB] Motors initialized (stopped).");
}

void init_servos() {
  servo1.attach(PIN_SERVO_1, SERVO_MIN_US, SERVO_MAX_US);
  servo2.attach(PIN_SERVO_2, SERVO_MIN_US, SERVO_MAX_US);
  servo3.attach(PIN_SERVO_3, SERVO_MIN_US, SERVO_MAX_US);
  servo4.attach(PIN_SERVO_4, SERVO_MIN_US, SERVO_MAX_US);

  // Center all servos on boot
  servo1.write(90); servo2.write(90);
  servo3.write(90); servo4.write(90);
  Serial.println("[MDB] Servos initialized (centered at 90°).");
}

void init_encoders() {
  pinMode(PIN_ENC_A_CH_A, INPUT_PULLUP);
  pinMode(PIN_ENC_A_CH_B, INPUT_PULLUP);
  pinMode(PIN_ENC_B_CH_A, INPUT_PULLUP);
  pinMode(PIN_ENC_B_CH_B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(PIN_ENC_A_CH_A), isr_enc_a, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_ENC_B_CH_A), isr_enc_b, RISING);
  Serial.println("[MDB] Encoders initialized with interrupts.");
}

void init_leds() {
  pinMode(PIN_LED_GREEN,  OUTPUT);
  pinMode(PIN_LED_YELLOW, OUTPUT);
  pinMode(PIN_LED_RED,    OUTPUT);
  pinMode(PIN_LED_BLUE,   OUTPUT);
  digitalWrite(PIN_LED_GREEN,  LOW);
  digitalWrite(PIN_LED_YELLOW, LOW);
  digitalWrite(PIN_LED_RED,    LOW);
  digitalWrite(PIN_LED_BLUE,   LOW);
}

void init_estop() {
  pinMode(PIN_ESTOP, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_ESTOP), isr_estop, FALLING);
  Serial.println("[MDB] E-Stop initialized (active LOW, interrupt-driven).");
}

bool init_current_sensors() {
  bool ok_a = ina219_a.begin();
  bool ok_b = ina219_b.begin();
  if (!ok_a) Serial.println("[MDB] WARNING: INA219 (Motor A) not found.");
  if (!ok_b) Serial.println("[MDB] WARNING: INA219 (Motor B) not found.");
  if (ok_a && ok_b) Serial.println("[MDB] INA219 current sensors initialized.");
  return (ok_a || ok_b);   // Partial OK — at least one sensor present
}

// ════════════════════════════════════════════════════════════
// CONTROL PACKET PROCESSING
// ════════════════════════════════════════════════════════════

void process_control_packet(const JsonDocument& doc) {
  // Motor A
  if (doc.containsKey("motor_a")) {
    int speed        = doc["motor_a"]["speed"]     | 0;
    const char* dir  = doc["motor_a"]["direction"] | "stop";
    set_motor(PWM_CH_MOTOR_A, PIN_MOTOR_A_IN1, PIN_MOTOR_A_IN2, speed, dir);
  }

  // Motor B
  if (doc.containsKey("motor_b")) {
    int speed        = doc["motor_b"]["speed"]     | 0;
    const char* dir  = doc["motor_b"]["direction"] | "stop";
    set_motor(PWM_CH_MOTOR_B, PIN_MOTOR_B_IN1, PIN_MOTOR_B_IN2, speed, dir);
  }

  // Servos — array of 4 angles [0–180]
  if (doc.containsKey("servo")) {
    JsonArrayConst angles = doc["servo"].as<JsonArrayConst>();
    if (angles.size() >= 4) {
      int a1 = constrain(angles[0].as<int>(), 0, 180);
      int a2 = constrain(angles[1].as<int>(), 0, 180);
      int a3 = constrain(angles[2].as<int>(), 0, 180);
      int a4 = constrain(angles[3].as<int>(), 0, 180);
      servo1.write(a1);
      servo2.write(a2);
      servo3.write(a3);
      servo4.write(a4);
    }
  }
}

// ════════════════════════════════════════════════════════════
// MOTOR CONTROL
// ════════════════════════════════════════════════════════════

void set_motor(uint8_t pwm_ch, uint8_t in1, uint8_t in2,
               int speed, const char* direction) {
  // Clamp speed 0–100 → map to 0–255 PWM
  speed = constrain(speed, 0, 100);
  uint8_t pwm_val = map(speed, 0, 100, 0, 255);

  if (strcmp(direction, "forward") == 0) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (strcmp(direction, "reverse") == 0) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    // "stop" or unknown
    stop_motor(pwm_ch, in1, in2);
    return;
  }

  ledcWrite(pwm_ch, pwm_val);
}

void stop_motor(uint8_t pwm_ch, uint8_t in1, uint8_t in2) {
  ledcWrite(pwm_ch, 0);
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
}

void stop_all_motors() {
  stop_motor(PWM_CH_MOTOR_A, PIN_MOTOR_A_IN1, PIN_MOTOR_A_IN2);
  stop_motor(PWM_CH_MOTOR_B, PIN_MOTOR_B_IN1, PIN_MOTOR_B_IN2);
}

// ════════════════════════════════════════════════════════════
// E-STOP
// ════════════════════════════════════════════════════════════

void trigger_estop() {
  estop_active = true;
  stop_all_motors();
  digitalWrite(PIN_LED_RED, HIGH);
  fault = true;
  Serial.println("[MDB] E-STOP TRIGGERED — all motors halted.");
}

void clear_estop() {
  estop_active = false;
  fault = false;
  digitalWrite(PIN_LED_RED, LOW);
  Serial.println("[MDB] E-Stop cleared. Ready for commands.");
}

// ════════════════════════════════════════════════════════════
// WATCHDOG
// ════════════════════════════════════════════════════════════

void check_watchdog() {
  if (!estop_active && (millis() - last_cmd_ms > WATCHDOG_TIMEOUT_MS)) {
    stop_all_motors();  // Silent stop — no fault, just safety
  }
}

// ════════════════════════════════════════════════════════════
// STATUS PACKET TRANSMISSION
// ════════════════════════════════════════════════════════════

void transmit_status(float current_a_ma, float current_b_ma) {
  StaticJsonDocument<256> doc;

  doc["timestamp"]        = (unsigned long long)millis();
  doc["packet_id"]        = packet_count;
  doc["motor_a_current_ma"] = serialized(String(current_a_ma, 1));
  doc["motor_b_current_ma"] = serialized(String(current_b_ma, 1));

  // Snapshot encoder ticks atomically
  noInterrupts();
  long snap_a = enc_a_ticks;
  long snap_b = enc_b_ticks;
  interrupts();

  doc["encoder_a_ticks"]  = snap_a;
  doc["encoder_b_ticks"]  = snap_b;
  doc["fault"]            = fault;
  doc["estop_active"]     = estop_active;

  serializeJson(doc, Serial);
  Serial.println();
}

// ════════════════════════════════════════════════════════════
// UTILITY
// ════════════════════════════════════════════════════════════

void blink_led(uint8_t pin, uint16_t ms) {
  digitalWrite(pin, HIGH);
  delay(ms);
  digitalWrite(pin, LOW);
}
