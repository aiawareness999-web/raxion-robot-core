/**
 * ============================================================
 * Sensor Interface Board (SIB) — Firmware Skeleton v0.1
 * Project: AI Robotic System — Leonardo Control Architecture
 * Author:  Leonardo (AI Superagent) for Aaron Edgley / Raxion Systems, LLC.
 * Date:    2026-06-16
 * MCU:     ESP32-WROOM-32
 * ============================================================
 *
 * OVERVIEW:
 * This firmware runs on the ESP32 aboard the Sensor Interface Board.
 * It polls all onboard sensors at 20Hz, packages readings into a
 * JSON payload, and transmits via UART to the Edge Processor (EPIB).
 *
 * DATA FLOW:
 *   MPU-6050 (IMU) ──┐
 *   ADS1115 (ADC)  ──┤
 *   HC-SR04 (USBL) ──┼──► ESP32 ──► JSON @ UART0 ──► EPIB / Pi
 *   OV2640 (CAM)   ──┤
 *   Analog GPIOs   ──┘
 *
 * UART OUTPUT FORMAT (20Hz):
 * {"timestamp":1718000000000,"accel":{"x":0.00,"y":0.00,"z":9.81},
 *  "gyro":{"x":0.000,"y":0.000,"z":0.000},"ultrasonic_cm":0.0,
 *  "analog":[0,0,0,0],"camera_frame_ready":false,"board_temp_c":0.0}
 * ============================================================
 */

// ─── Libraries ───────────────────────────────────────────────
#include <Arduino.h>
#include <Wire.h>
#include <ArduinoJson.h>       // ArduinoJson v6+ (install via Library Manager)
#include <Adafruit_MPU6050.h>  // Adafruit MPU6050 library
#include <Adafruit_ADS1X15.h>  // Adafruit ADS1115 library
#include <WiFi.h>              // ESP32 built-in WiFi

// ─── Pin Definitions ─────────────────────────────────────────
#define PIN_I2C_SDA       21
#define PIN_I2C_SCL       22
#define PIN_TRIG          13    // HC-SR04 trigger
#define PIN_ECHO          14    // HC-SR04 echo (via 1kΩ/2kΩ divider)
#define PIN_IMU_INT        4    // MPU-6050 interrupt
#define PIN_LED_BLUE       2    // Status LED — data transmitting
#define PIN_LED_RED       15    // Status LED — fault
#define PIN_ANALOG_1      34    // Spare analog input 1 (ADC1)
#define PIN_ANALOG_2      35    // Spare analog input 2 (ADC1)
#define PIN_UART_TX        1    // UART0 TX → EPIB
#define PIN_UART_RX        3    // UART0 RX ← EPIB

// ─── Configuration ───────────────────────────────────────────
#define LOOP_HZ           20    // Target sensor polling rate (Hz)
#define LOOP_INTERVAL_MS  (1000 / LOOP_HZ)  // 50ms per cycle
#define UART_BAUD         115200
#define I2C_FREQ          400000  // 400kHz fast mode I2C
#define ULTRASONIC_TIMEOUT_US 25000  // ~4.25m max range

// ─── WiFi Config (optional — disable if not needed) ──────────
// #define WIFI_ENABLED
#ifdef WIFI_ENABLED
  const char* WIFI_SSID = "YOUR_SSID";
  const char* WIFI_PASS = "YOUR_PASSWORD";
#endif

// ─── Sensor Objects ──────────────────────────────────────────
Adafruit_MPU6050 mpu;
Adafruit_ADS1115 ads;

// ─── State Variables ─────────────────────────────────────────
bool imu_ok    = false;
bool adc_ok    = false;
bool cam_ready = false;
unsigned long last_loop_ms = 0;
uint32_t packet_count = 0;

// ─── Forward Declarations ────────────────────────────────────
void     init_i2c();
void     init_imu();
void     init_adc();
void     init_ultrasonic();
void     init_leds();
bool     read_imu(float &ax, float &ay, float &az,
                  float &gx, float &gy, float &gz, float &temp_c);
float    read_ultrasonic_cm();
int16_t* read_adc(int16_t results[4]);
void     transmit_packet(float ax, float ay, float az,
                         float gx, float gy, float gz,
                         float ultrasonic_cm, int16_t adc[4],
                         bool cam_rdy, float board_temp_c);
void     blink_led(uint8_t pin, uint16_t ms);
void     set_fault(bool fault);

// ════════════════════════════════════════════════════════════
// SETUP
// ════════════════════════════════════════════════════════════
void setup() {
  // UART — primary comms to EPIB
  Serial.begin(UART_BAUD);
  delay(100);
  Serial.println("[SIB] Sensor Interface Board v0.1 booting...");

  // GPIO init
  init_leds();
  init_ultrasonic();

  // I2C bus
  init_i2c();

  // Sensors
  init_imu();
  init_adc();

  // Optional WiFi
  #ifdef WIFI_ENABLED
    Serial.print("[SIB] Connecting WiFi...");
    WiFi.begin(WIFI_SSID, WIFI_PASS);
    uint8_t attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
      delay(500); Serial.print("."); attempts++;
    }
    if (WiFi.status() == WL_CONNECTED) {
      Serial.println("\n[SIB] WiFi connected: " + WiFi.localIP().toString());
    } else {
      Serial.println("\n[SIB] WiFi failed — running UART-only mode.");
    }
  #endif

  Serial.println("[SIB] Boot complete. Starting sensor loop at 20Hz.");
  digitalWrite(PIN_LED_BLUE, HIGH);  // Signal boot complete
}

// ════════════════════════════════════════════════════════════
// MAIN LOOP — runs at 20Hz
// ════════════════════════════════════════════════════════════
void loop() {
  unsigned long now = millis();
  if (now - last_loop_ms < LOOP_INTERVAL_MS) return;
  last_loop_ms = now;

  // ── Read IMU ─────────────────────────────────────────────
  float ax = 0, ay = 0, az = 0;
  float gx = 0, gy = 0, gz = 0;
  float board_temp_c = 0;
  bool imu_read_ok = read_imu(ax, ay, az, gx, gy, gz, board_temp_c);

  // ── Read Ultrasonic ──────────────────────────────────────
  float distance_cm = read_ultrasonic_cm();

  // ── Read ADC channels ────────────────────────────────────
  int16_t adc_results[4] = {0, 0, 0, 0};
  read_adc(adc_results);

  // ── Camera frame flag (stub — expand when camera integrated)
  cam_ready = false;  // TODO: set true when OV2640 frame is ready

  // ── Fault check ──────────────────────────────────────────
  bool fault = (!imu_ok || !adc_ok || !imu_read_ok);
  set_fault(fault);

  // ── Transmit JSON packet to EPIB ─────────────────────────
  transmit_packet(ax, ay, az, gx, gy, gz,
                  distance_cm, adc_results,
                  cam_ready, board_temp_c);

  // ── Heartbeat LED ────────────────────────────────────────
  blink_led(PIN_LED_BLUE, 5);  // Quick flash on each packet

  packet_count++;
}

// ════════════════════════════════════════════════════════════
// INITIALIZATION FUNCTIONS
// ════════════════════════════════════════════════════════════

void init_i2c() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(I2C_FREQ);
  Serial.println("[SIB] I2C initialized at 400kHz.");
}

void init_imu() {
  if (!mpu.begin()) {
    Serial.println("[SIB] ERROR: MPU-6050 not found. Check wiring.");
    imu_ok = false;
    set_fault(true);
    return;
  }
  // Configure ranges
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  imu_ok = true;
  Serial.println("[SIB] MPU-6050 initialized. Range: ±8G / ±500°/s.");
}

void init_adc() {
  if (!ads.begin()) {
    Serial.println("[SIB] ERROR: ADS1115 not found. Check wiring.");
    adc_ok = false;
    return;
  }
  ads.setGain(GAIN_ONE);  // ±4.096V range, 0.125mV resolution
  adc_ok = true;
  Serial.println("[SIB] ADS1115 initialized. Gain: ±4.096V.");
}

void init_ultrasonic() {
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);
  Serial.println("[SIB] HC-SR04 ultrasonic initialized.");
}

void init_leds() {
  pinMode(PIN_LED_BLUE, OUTPUT);
  pinMode(PIN_LED_RED,  OUTPUT);
  digitalWrite(PIN_LED_BLUE, LOW);
  digitalWrite(PIN_LED_RED,  LOW);
}

// ════════════════════════════════════════════════════════════
// SENSOR READ FUNCTIONS
// ════════════════════════════════════════════════════════════

bool read_imu(float &ax, float &ay, float &az,
              float &gx, float &gy, float &gz, float &temp_c) {
  if (!imu_ok) return false;

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  ax = a.acceleration.x;
  ay = a.acceleration.y;
  az = a.acceleration.z;
  gx = g.gyro.x;
  gy = g.gyro.y;
  gz = g.gyro.z;
  temp_c = temp.temperature;

  return true;
}

float read_ultrasonic_cm() {
  // Send 10µs trigger pulse
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);

  // Measure echo pulse duration
  long duration_us = pulseIn(PIN_ECHO, HIGH, ULTRASONIC_TIMEOUT_US);

  if (duration_us == 0) return -1.0;  // Timeout — no object detected

  // Distance = (duration × speed of sound) / 2
  // Speed of sound at ~20°C = 343 m/s = 0.0343 cm/µs
  float distance_cm = (duration_us * 0.0343) / 2.0;
  return distance_cm;
}

int16_t* read_adc(int16_t results[4]) {
  if (!adc_ok) return results;  // Return zeros if ADC failed

  // Read all 4 channels (single-ended)
  results[0] = ads.readADC_SingleEnded(0);
  results[1] = ads.readADC_SingleEnded(1);
  results[2] = ads.readADC_SingleEnded(2);
  results[3] = ads.readADC_SingleEnded(3);

  return results;
}

// ════════════════════════════════════════════════════════════
// PACKET TRANSMISSION
// ════════════════════════════════════════════════════════════

void transmit_packet(float ax, float ay, float az,
                     float gx, float gy, float gz,
                     float ultrasonic_cm, int16_t adc[4],
                     bool cam_rdy, float board_temp_c) {

  // Build JSON document (StaticJsonDocument = stack allocated, faster)
  StaticJsonDocument<512> doc;

  doc["timestamp"]          = (unsigned long long)millis();
  doc["packet_id"]          = packet_count;

  JsonObject accel          = doc.createNestedObject("accel");
  accel["x"]                = serialized(String(ax, 3));
  accel["y"]                = serialized(String(ay, 3));
  accel["z"]                = serialized(String(az, 3));

  JsonObject gyro           = doc.createNestedObject("gyro");
  gyro["x"]                 = serialized(String(gx, 4));
  gyro["y"]                 = serialized(String(gy, 4));
  gyro["z"]                 = serialized(String(gz, 4));

  doc["ultrasonic_cm"]      = serialized(String(ultrasonic_cm, 1));

  JsonArray analog          = doc.createNestedArray("analog");
  analog.add(adc[0]);
  analog.add(adc[1]);
  analog.add(adc[2]);
  analog.add(adc[3]);

  doc["camera_frame_ready"] = cam_rdy;
  doc["board_temp_c"]       = serialized(String(board_temp_c, 1));

  // Serialize to UART — single line, newline terminated
  serializeJson(doc, Serial);
  Serial.println();  // \n as packet delimiter for Pi parser
}

// ════════════════════════════════════════════════════════════
// UTILITY FUNCTIONS
// ════════════════════════════════════════════════════════════

void blink_led(uint8_t pin, uint16_t ms) {
  digitalWrite(pin, HIGH);
  delay(ms);
  digitalWrite(pin, LOW);
}

void set_fault(bool fault) {
  digitalWrite(PIN_LED_RED, fault ? HIGH : LOW);
}

