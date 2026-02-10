#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

// ===== HELMET SENSOR (0x4B) =====
#define HELMET_PS0_PIN 6
#define HELMET_PS1_PIN 7
#define HELMET_RST_PIN 4
Adafruit_BNO08x bno_helmet = Adafruit_BNO08x(HELMET_RST_PIN);

// ===== CAR REFERENCE SENSOR (0x4A) =====
#define CAR_PS0_PIN 8
#define CAR_PS1_PIN 9
#define CAR_RST_PIN 5
Adafruit_BNO08x bno_car = Adafruit_BNO08x(CAR_RST_PIN);

sh2_SensorValue_t sensorValue_helmet;
sh2_SensorValue_t sensorValue_car;

// Optional: INT pin
constexpr int PIN_INT = 2;

void setup() {
  Serial.begin(115200);
  while (!Serial) {} // USB-Seriell warten

  Serial.println(F("==== DUAL BNO08x Head Tracking System ===="));
  Serial.println(F("Helmet sensor: 0x4B | Car sensor: 0x4A"));
  Serial.println();

  // ===== SETUP HELMET SENSOR (0x4B) =====
  pinMode(HELMET_PS0_PIN, OUTPUT);
  pinMode(HELMET_PS1_PIN, OUTPUT);
  pinMode(HELMET_RST_PIN, OUTPUT);
  digitalWrite(HELMET_PS0_PIN, LOW);
  digitalWrite(HELMET_PS1_PIN, LOW);

  // ===== SETUP CAR SENSOR (0x4A) =====
  pinMode(CAR_PS0_PIN, OUTPUT);
  pinMode(CAR_PS1_PIN, OUTPUT);
  pinMode(CAR_RST_PIN, OUTPUT);
  digitalWrite(CAR_PS0_PIN, LOW);
  digitalWrite(CAR_PS1_PIN, LOW);

  // Reset both sensors
  digitalWrite(HELMET_RST_PIN, LOW);
  digitalWrite(CAR_RST_PIN, LOW);
  delay(20);
  digitalWrite(HELMET_RST_PIN, HIGH);
  digitalWrite(CAR_RST_PIN, HIGH);
  delay(200);

  // I2C initialisieren
  Wire.begin();
  Wire.setClock(100000);  // 100 kHz
  pinMode(PIN_INT, INPUT_PULLUP);

  // I2C Bus scannen
  Serial.println(F("Scanning I2C bus..."));
  byte found = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    byte error = Wire.endTransmission();
    if (error == 0) {
      Serial.print(F("  Device at 0x"));
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
      found++;
    }
  }
  Serial.print(F("Found "));
  Serial.print(found);
  Serial.println(F(" device(s)"));
  Serial.println();

  // ===== INIT HELMET SENSOR (0x4B) =====
  Serial.println(F("Initializing HELMET sensor (0x4B)..."));
  if (!bno_helmet.begin_I2C(0x4B)) {
    Serial.println(F("FEHLER: Helmet sensor nicht gefunden!"));
    while (1) { delay(10); }
  }
  Serial.println(F("  Helmet sensor gefunden!"));
  bno_helmet.hardwareReset();
  delay(300);
  if (!bno_helmet.enableReport(SH2_ROTATION_VECTOR, 5000)) {
    Serial.println(F("  FEHLER: Helmet report enable failed!"));
    while(1) { delay(10); }
  }
  Serial.println(F("  Helmet sensor bereit!"));

  // ===== INIT CAR REFERENCE SENSOR (0x4A) =====
  Serial.println(F("Initializing CAR sensor (0x4A)..."));
  if (!bno_car.begin_I2C(0x4A)) {
    Serial.println(F("FEHLER: Car sensor nicht gefunden!"));
    Serial.println(F("Stelle sicher dass SA0 auf GND liegt!"));
    while (1) { delay(10); }
  }
  Serial.println(F("  Car sensor gefunden!"));
  bno_car.hardwareReset();
  delay(300);
  if (!bno_car.enableReport(SH2_ROTATION_VECTOR, 5000)) {
    Serial.println(F("  FEHLER: Car report enable failed!"));
    while(1) { delay(10); }
  }
  Serial.println(F("  Car sensor bereit!"));

  Serial.println();
  Serial.println(F("==== System Ready! ===="));
  Serial.println(F("Format: RelYaw RelPitch RelRoll | Helmet | Car"));
  Serial.println();
}

// Helper: Convert quaternion to Euler angles
void quatToEuler(float qw, float qx, float qy, float qz, float &yaw, float &pitch, float &roll) {
  float ysqr = qy * qy;

  // Roll (x-axis rotation)
  float t0 = 2.0f * (qw * qx + qy * qz);
  float t1 = 1.0f - 2.0f * (qx * qx + ysqr);
  roll = atan2f(t0, t1);

  // Pitch (y-axis rotation)
  float t2 = 2.0f * (qw * qy - qz * qx);
  t2 = constrain(t2, -1.0f, 1.0f);
  pitch = asinf(t2);

  // Yaw (z-axis rotation)
  float t3 = 2.0f * (qw * qz + qx * qy);
  float t4 = 1.0f - 2.0f * (ysqr + qz * qz);
  yaw = atan2f(t3, t4);
}

// Helper: Normalize angle to -180..+180
float normalizeAngle(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

void loop() {
  static float helmet_yaw = 0, helmet_pitch = 0, helmet_roll = 0;
  static float car_yaw = 0, car_pitch = 0, car_roll = 0;
  static bool helmet_updated = false;
  static bool car_updated = false;

  // Read HELMET sensor
  if (bno_helmet.getSensorEvent(&sensorValue_helmet)) {
    if (sensorValue_helmet.sensorId == SH2_ROTATION_VECTOR) {
      float qw = sensorValue_helmet.un.rotationVector.real;
      float qx = sensorValue_helmet.un.rotationVector.i;
      float qy = sensorValue_helmet.un.rotationVector.j;
      float qz = sensorValue_helmet.un.rotationVector.k;
      quatToEuler(qw, qx, qy, qz, helmet_yaw, helmet_pitch, helmet_roll);
      helmet_yaw *= 180.0f / PI;
      helmet_pitch *= 180.0f / PI;
      helmet_roll *= 180.0f / PI;
      helmet_updated = true;
    }
  }

  // Read CAR sensor
  if (bno_car.getSensorEvent(&sensorValue_car)) {
    if (sensorValue_car.sensorId == SH2_ROTATION_VECTOR) {
      float qw = sensorValue_car.un.rotationVector.real;
      float qx = sensorValue_car.un.rotationVector.i;
      float qy = sensorValue_car.un.rotationVector.j;
      float qz = sensorValue_car.un.rotationVector.k;
      quatToEuler(qw, qx, qy, qz, car_yaw, car_pitch, car_roll);
      car_yaw *= 180.0f / PI;
      car_pitch *= 180.0f / PI;
      car_roll *= 180.0f / PI;
      car_updated = true;
    }
  }

  // When both sensors have new data, calculate and print relative orientation
  if (helmet_updated && car_updated) {
    // RELATIVE orientation: Helmet relative to Car
    float rel_yaw = normalizeAngle(helmet_yaw - car_yaw);
    float rel_pitch = normalizeAngle(helmet_pitch - car_pitch);
    float rel_roll = normalizeAngle(helmet_roll - car_roll);

    // Print relative orientation (for searchlight control)
    Serial.print(F("REL=> Y:"));
    Serial.print(rel_yaw, 1);
    Serial.print(F(" P:"));
    Serial.print(rel_pitch, 1);
    Serial.print(F(" R:"));
    Serial.print(rel_roll, 1);

    // Print absolute orientations (for debugging)
    Serial.print(F(" | Helmet=> Y:"));
    Serial.print(helmet_yaw, 1);
    Serial.print(F(" P:"));
    Serial.print(helmet_pitch, 1);
    Serial.print(F(" R:"));
    Serial.print(helmet_roll, 1);

    Serial.print(F(" | Car=> Y:"));
    Serial.print(car_yaw, 1);
    Serial.print(F(" P:"));
    Serial.print(car_pitch, 1);
    Serial.print(F(" R:"));
    Serial.println(car_roll, 1);

    helmet_updated = false;
    car_updated = false;
  }
}
