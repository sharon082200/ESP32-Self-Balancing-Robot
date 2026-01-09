/*
 * Project: ESP32 Self-Balancing Robot
 * Author: SHARON YERUKHIMOVICH
 * Description: Two-wheeled inverted pendulum robot using PID control and Sensor Fusion (MPU6050).
 * Features Bluetooth control and battery monitoring.
 */

#include <Wire.h>
#include "BluetoothSerial.h"

BluetoothSerial SerialBT;

// --- Pin Definitions ---
const int ENA = 32;
const int IN1 = 5;
const int IN2 = 25;
const int IN3 = 26;
const int IN4 = 27;
const int ENB = 14;
const int PIN_BATTERY = 35; // Changed name for clarity

const int ENC_L_A = 4;
const int ENC_L_B = 13;
const int ENC_R_A = 18;
const int ENC_R_B = 19;

const int LED_PIN = 2;
const int PIN_BUZZER = 15;

// --- PID Constants ---
float k1 = 41.0f;  // Kp - Proportional
float k2 = 2.5f;   // Kd - Derivative
const float kI = 1.2f; // Ki - Integral

float kx = 0.0f;   // Position P
float kv = 0.01f;  // Position D

// --- State Variables ---
float target_angle = 3.7f; // Mechanical balance point offset
float angle_pitch = 0.0f;
float gyro_rate = 0.0f;
float gyro_bias = 0.0f;
float error_i = 0.0f;

volatile long countLeft = 0;
volatile long countRight = 0;

// --- Control Variables ---
float manual_move_offset = 0.0f;
float manual_turn_offset = 0.0f;
float speed_move_val = 2.5f;
float speed_turn_val = 50.0f;

// --- Function Prototypes ---
void calibrateGyro();
void setMotor(int pwmR, int pwmL);
void IRAM_ATTR readLeft();
void IRAM_ATTR readRight();
void beep(int duration_ms);
bool readSensor(float dt);
void checkBluetooth();
void sendTelemetry();

// Helper: Clamp float values
static inline float clampf(float x, float a, float b) {
  if (x < a) return a;
  if (x > b) return b;
  return x;
}

void setup() {
  Serial.begin(115200);

  // Initialize Bluetooth
  if (!SerialBT.begin("BalancingRobot")) {
    Serial.println("Bluetooth Init Failed!");
    while (1); // Halt execution
  } else {
    Serial.println("Bluetooth Started Successfully!");
  }

  // Initialize I2C
  Wire.begin();
  Wire.setClock(400000);

  // Pin Modes
  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(PIN_BATTERY, INPUT);

  pinMode(ENC_L_A, INPUT_PULLUP);
  pinMode(ENC_L_B, INPUT_PULLUP);
  pinMode(ENC_R_A, INPUT_PULLUP);
  pinMode(ENC_R_B, INPUT_PULLUP);

  // Encoder Interrupts
  attachInterrupt(digitalPinToInterrupt(ENC_L_A), readLeft, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_R_A), readRight, RISING);

  pinMode(LED_PIN, OUTPUT);
  pinMode(PIN_BUZZER, OUTPUT);

  // Startup sequence
  digitalWrite(LED_PIN, HIGH);
  beep(100);
  digitalWrite(LED_PIN, LOW);

  // Wake up MPU6050
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);

  calibrateGyro();

  // Reset encoders before starting loop
  noInterrupts();
  countLeft = 0;
  countRight = 0;
  interrupts();
}

void loop() {
  const uint32_t loop_time_us = 5000; // 5ms loop = 200Hz
  static uint32_t last_time = 0;

  uint32_t now = micros();

  if (last_time == 0) {
    last_time = now;
    return;
  }

  if (now - last_time < loop_time_us) return;

  float dt = (now - last_time) * 1e-6f;
  last_time = now;

  dt = clampf(dt, 0.001f, 0.01f);

  // Read IMU
  if (!readSensor(dt)) return;

  // Handle Inputs & Telemetry
  checkBluetooth();
  
  static int telemetry_counter = 0;
  if (telemetry_counter++ > 40) { // Send every ~200ms
    sendTelemetry();
    telemetry_counter = 0;
  }

  // Encoder Processing
  long L, R;
  noInterrupts();
  L = countLeft;
  R = countRight;
  interrupts();

  float position = 0.5f * (L + R);
  static float prev_position = 0.0f;
  float speed_raw = (position - prev_position) / dt;
  prev_position = position;

  // Filter speed
  speed_raw = clampf(speed_raw, -3000.0f, 3000.0f);
  static float speed_f = 0.0f;
  const float alpha = 0.4f;
  speed_f = (1.0f - alpha) * speed_f + alpha * speed_raw;
  float speed = speed_f;

  // --- PID Calculation ---
  float position_correction = (kx * position) + (kv * speed);
  position_correction = clampf(position_correction, -6.0f, 6.0f);

  float target_angle_dynamic = target_angle + position_correction + manual_move_offset;
  float error = angle_pitch - target_angle_dynamic;

  // Integral term (Anti-windup logic)
  if (fabs(angle_pitch) < 25.0f && fabs(error) < 6.0f) {
    error_i += error * dt;
  }
  error_i = clampf(error_i, -100.0f, 100.0f);
  float I_term = kI * error_i;
  I_term = clampf(I_term, -60.0f, 60.0f);

  // Gyro filtering
  static float gyro_f = 0.0f;
  const float g_alpha = 0.35f;
  gyro_f = (1.0f - g_alpha) * gyro_f + g_alpha * gyro_rate;

  // Final Output Calculation
  float motorV = ((k1 * error) + (k2 * gyro_f) + I_term);

  // Safety Cutoff (Fall detection)
  if (fabs(angle_pitch) > 35.0f) { // Increased slightly to prevent false trips
    setMotor(0, 0);
    noInterrupts();
    countLeft = 0;
    countRight = 0;
    interrupts();
    prev_position = 0.0f;
    error_i = 0.0f;
    return;
  }

  int pwm = constrain((int)motorV, -250, 250);
  int pwmL = pwm + manual_turn_offset;
  int pwmR = pwm - manual_turn_offset;

  pwmL = constrain(pwmL, -255, 250);
  pwmR = constrain(pwmR, -255, 250);

  setMotor(pwmL, pwmR);

  // LED Status
  if (fabs(error) < 5.0f) digitalWrite(LED_PIN, HIGH);
  else digitalWrite(LED_PIN, LOW);

  // Battery Monitoring
  float sens_v = analogRead(PIN_BATTERY);
  float current_v = sens_v / 4095.0f * 3.3f * 4.3f;

  if (current_v < 5.6f && current_v > 3.0f) { // Added > 3.0f to ignore USB power
    unsigned long m = millis();
    digitalWrite(LED_PIN, HIGH);
    // Beep pattern
    if ((m % 1000) < 100) {
      tone(PIN_BUZZER, 2000);
      digitalWrite(LED_PIN, LOW);
    }
    else if ((m % 1000) > 200 && (m % 1000) < 300) {
      tone(PIN_BUZZER, 2000);
      digitalWrite(LED_PIN, HIGH);
    }
    else {
      noTone(PIN_BUZZER);
      digitalWrite(LED_PIN, LOW);
    }
  }
}

void setMotor(int pwmL, int pwmR) {
  const int offset = 29; // Min PWM to overcome friction
  const int dead_zone = 6;

  if (abs(pwmL) > dead_zone) {
    int s = (pwmL > 0) ? 1 : -1;
    pwmL = s * (abs(pwmL) + offset);
  }

  if (abs(pwmR) > dead_zone) {
    int s = (pwmR > 0) ? 1 : -1;
    pwmR = s * (abs(pwmR) + offset);
  }

  // Left Motor Logic
  if (pwmL >= 0) {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  } else {
    digitalWrite(IN2, LOW);
    digitalWrite(IN1, HIGH);
    pwmL = -pwmL;
  }

  // Right Motor Logic
  if (pwmR >= 0) {
    digitalWrite(IN4, LOW);
    digitalWrite(IN3, HIGH);
  } else {
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    pwmR = -pwmR;
  }

  // Final Safety Clamp
  if (pwmL > 255) pwmL = 255;
  if (pwmR > 255) pwmR = 255;

  analogWrite(ENA, pwmL);
  analogWrite(ENB, pwmR);
}

bool readSensor(float dt) {
  Wire.beginTransmission(0x68);
  Wire.write(0x3B);
  if (Wire.endTransmission(false) != 0) return false;

  int n = Wire.requestFrom(0x68, 14, true);
  if (n != 14 || Wire.available() < 14) return false;

  int16_t Acx = Wire.read() << 8 | Wire.read();
  int16_t Acy = Wire.read() << 8 | Wire.read();
  int16_t Acz = Wire.read() << 8 | Wire.read();

  Wire.read(); // Temperature high
  Wire.read(); // Temperature low

  int16_t GyroX = Wire.read() << 8 | Wire.read();
  int16_t GyroY = Wire.read() << 8 | Wire.read();
  int16_t GyroZ = Wire.read() << 8 | Wire.read();

  float angle = atan2((float)Acx, (float)Acz) * 180.0f / PI;
  gyro_rate = -((float)GyroY) / 131.0f - gyro_bias;

  // Complementary Filter
  angle_pitch = 0.98f * (angle_pitch + gyro_rate * dt) + 0.02f * angle;

  return true;
}

void calibrateGyro() {
  long sum = 0;
  const int N = 1200;
  int valid = 0;
  for (int i = 0; i < N; i++) {
    Wire.beginTransmission(0x68);
    Wire.write(0x45); // GyroY High
    if (Wire.endTransmission(false) != 0) continue;

    int n = Wire.requestFrom(0x68, 2, true);
    if (n != 2 || Wire.available() < 2) continue;

    int16_t Gy = Wire.read() << 8 | Wire.read();
    sum += Gy;
    valid++;
    delay(2);
  }
  if (valid < 200) gyro_bias = 0.0f;
  else gyro_bias = (sum / (float)valid) / 131.0f;
  
  beep(100);
}

void beep(int duration_ms) {
  unsigned long end = millis() + duration_ms;
  while (millis() < end) {
    digitalWrite(PIN_BUZZER, HIGH);
    delayMicroseconds(250);
    digitalWrite(PIN_BUZZER, LOW);
    delayMicroseconds(250);
  }
}

void IRAM_ATTR readLeft() {
  if (digitalRead(ENC_L_B) > 0) {
    countLeft--;
  } else {
    countLeft++;
  }
}

void IRAM_ATTR readRight() {
  if (digitalRead(ENC_R_B) > 0) {
    countRight++;
  } else {
    countRight--;
  }
}

void checkBluetooth() {
  if (SerialBT.available()) {
    char cmd = SerialBT.read();
    
    // Speed Modes
    if (cmd == '1') {
      speed_move_val = 1.5f;
      speed_turn_val = 30.0f;
    } else if (cmd == '2') {
      speed_move_val = 2.5f;
      speed_turn_val = 50.0f;
    } else if (cmd == '3') {
      speed_move_val = 4.0f;
      speed_turn_val = 70.0f;
    }

    // Direction Control
    if (cmd == 'F') { manual_move_offset = -speed_move_val; }      // Forward
    else if (cmd == 'B') { manual_move_offset = speed_move_val; }  // Backward
    else if (cmd == 'L') { manual_turn_offset = speed_turn_val; }  // Left
    else if (cmd == 'R') { manual_turn_offset = -speed_turn_val; } // Right
    else if (cmd == 'S') { // Stop
      manual_move_offset = 0.0f;
      manual_turn_offset = 0.0f;
    }
  }
}

void sendTelemetry() {
  float sens_v = analogRead(PIN_BATTERY);
  float current_v = sens_v / 4095.0f * 3.3f * 4.3f;

  if (SerialBT.hasClient()) {
    SerialBT.print("Bat: ");
    SerialBT.print(current_v);
    SerialBT.print("V | Angle: ");
    SerialBT.println(angle_pitch);
  }
}