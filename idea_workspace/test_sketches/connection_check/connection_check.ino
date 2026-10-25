#include <Wire.h>
#include "ICM_20948.h"

// ============================================================
// R2 CONNECTION CHECK — NO BALANCE CONTROL
//
// Intended pin map (based on the proposed reassignment):
//   Right Hall A/B/C : D0, D1, D7
//   Left  Hall A/B/C : D8, D9, D10
//   IMU I2C          : SDA D2, SCL D3
//   Right motor DIR  : D4
//   Left motor PWM   : D5
//   Right motor PWM  : D6
//   Left motor DIR   : D12
//
// Motors are only driven after an explicit serial command.
// Each motor command automatically expires after 2 seconds.
// This sketch does NOT balance the robot.
// ============================================================

// Motor pins
const uint8_t LEFT_PWM_PIN  = 5;
const uint8_t RIGHT_PWM_PIN = 6;
const uint8_t LEFT_DIR_PIN  = 12;
const uint8_t RIGHT_DIR_PIN = 4;

// Hall inputs — provisional A/B/C labels; we only display raw states.
const uint8_t RIGHT_HA_PIN = 8;
const uint8_t RIGHT_HB_PIN = 9;
const uint8_t RIGHT_HC_PIN = 10;
const uint8_t LEFT_HA_PIN  = 0;
const uint8_t LEFT_HB_PIN  = 1;
const uint8_t LEFT_HC_PIN  = 7;

const int MAX_PWM = 51;                  // Keep existing conservative limit
const int PWM_STEP = 1;
const unsigned long MOTOR_UPDATE_MS = 10;
const unsigned long COMMAND_HOLD_MS = 2000;
const unsigned long TELEMETRY_INTERVAL_MS = 100;

ICM_20948_I2C imu;
#define AD0_VAL 1
const int CALIBRATION_SAMPLES = 500;
const float ALPHA = 0.98f;

float gyroYBias = 0.0f;
float filteredPitch = 0.0f;
float pitchRate = 0.0f;
float accelPitch = 0.0f;
unsigned long lastIMUmicros = 0;
unsigned long lastMotorUpdate = 0;
unsigned long lastTelemetryTime = 0;
unsigned long commandExpiryTime = 0;

int targetLeft = 0, targetRight = 0;
int currentLeft = 0, currentRight = 0;

// ------------------------------------------------------------
// Setup
// ------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(1000);

  // Explicitly initialize ALL six Hall pins, as requested.
  pinMode(RIGHT_HA_PIN, INPUT);
  pinMode(RIGHT_HB_PIN, INPUT);
  pinMode(RIGHT_HC_PIN, INPUT);
  pinMode(LEFT_HA_PIN, INPUT);
  pinMode(LEFT_HB_PIN, INPUT);
  pinMode(LEFT_HC_PIN, INPUT);

  // Explicit motor output setup and safe stopped state.
  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
  analogWrite(LEFT_PWM_PIN, 0);
  analogWrite(RIGHT_PWM_PIN, 0);
  digitalWrite(LEFT_DIR_PIN, LOW);
  digitalWrite(RIGHT_DIR_PIN, LOW);

  Serial.println(F("R2 connection check; balance control is DISABLED."));
  Serial.println(F("Motor commands are limited to +/-100 input, mapped to +/-51 PWM."));
  Serial.println(F("Commands: M <left> <right> | L <value> | R <value> | STOP | HELP"));
  Serial.println(F("Example: M 25 25   (about PWM 12, auto-stop after 2 seconds)"));
  Serial.println(F("Keep wheels off the ground for motor checks."));

  Wire.begin();
  Wire.setClock(400000);
  Serial.println(F("Starting ICM-20948..."));
  while (imu.begin(Wire, AD0_VAL) != ICM_20948_Stat_Ok) {
    Serial.println(F("IMU initialization failed; check wiring/address. Retrying..."));
    delay(500);
  }
  Serial.println(F("IMU initialized."));

  calibrateGyro();

  while (!imu.dataReady()) delay(1);
  imu.getAGMT();
  filteredPitch = atan2(-imu.accX(), imu.accZ()) * 180.0f / PI;

  lastIMUmicros = micros();
  lastMotorUpdate = millis();
  lastTelemetryTime = millis();

  Serial.println(F("READY"));
  Serial.println(F("CSV: ms,pitch_deg,pitchRate_dps,accelPitch_deg,leftPWM,rightPWM,R_HA,R_HB,R_HC,L_HA,L_HB,L_HC"));
}

// ------------------------------------------------------------
// Main loop
// ------------------------------------------------------------
void loop() {
  readIMU();
  processSerialCommands();
  updateMotors();
  printTelemetry();
}

// ------------------------------------------------------------
// IMU: same pitch convention/filter as the existing known-good sketch
// Positive pitch = nose/front rises (tilted backward).
// ------------------------------------------------------------
void readIMU() {
  if (!imu.dataReady()) return;
  imu.getAGMT();

  unsigned long nowUs = micros();
  float dt = (nowUs - lastIMUmicros) / 1000000.0f;
  lastIMUmicros = nowUs;
  if (dt <= 0.0f || dt > 0.1f) return;

  accelPitch = atan2(-imu.accX(), imu.accZ()) * 180.0f / PI;
  pitchRate = -(imu.gyrY() - gyroYBias);
  float gyroPitch = filteredPitch + pitchRate * dt;
  filteredPitch = ALPHA * gyroPitch + (1.0f - ALPHA) * accelPitch;
}

void calibrateGyro() {
  Serial.println(F("Gyro calibration: keep robot completely still."));
  Serial.println(F("It need not be upright."));
  float sum = 0.0f;
  int collected = 0;
  while (collected < CALIBRATION_SAMPLES) {
    if (imu.dataReady()) {
      imu.getAGMT();
      sum += imu.gyrY();
      collected++;
    }
    delay(2);
  }
  gyroYBias = sum / CALIBRATION_SAMPLES;
  Serial.print(F("Gyro Y bias: "));
  Serial.println(gyroYBias, 4);
}

// ------------------------------------------------------------
// Simple text commands. Values use the same -100..+100 interface
// as the Python packet test and are mapped to -51..+51 PWM.
// Each command expires after 2 seconds as a safety measure.
// ------------------------------------------------------------
void processSerialCommands() {
  static char line[64];
  static uint8_t used = 0;

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      line[used] = '\0';
      handleCommand(line);
      used = 0;
    } else if (used < sizeof(line) - 1) {
      line[used++] = c;
    } else {
      used = 0;
      Serial.println(F("ERR: command too long"));
    }
  }
}

int commandToPWM(int value) {
  value = constrain(value, -100, 100);
  int pwm = map(abs(value), 0, 100, 0, MAX_PWM);
  return (value < 0) ? -pwm : pwm;
}

void handleCommand(char *line) {
  char cmd[12] = {0};
  int a = 0, b = 0;
  int n = sscanf(line, "%11s %d %d", cmd, &a, &b);

  if (!strcmp(cmd, "HELP")) {
    Serial.println(F("M <left> <right>; L <value>; R <value>; STOP"));
    Serial.println(F("Values -100..100; command auto-stops after 2 seconds."));
    return;
  }
  if (!strcmp(cmd, "STOP")) {
    targetLeft = targetRight = 0;
    commandExpiryTime = 0;
    Serial.println(F("STOP requested"));
    return;
  }
  if (!strcmp(cmd, "M") && n == 3) {
    targetLeft = commandToPWM(a);
    targetRight = commandToPWM(b);
  } else if (!strcmp(cmd, "L") && n >= 2) {
    targetLeft = commandToPWM(a);
  } else if (!strcmp(cmd, "R") && n >= 2) {
    targetRight = commandToPWM(a);
  } else {
    Serial.println(F("ERR: use M <left> <right>, L <value>, R <value>, STOP, or HELP"));
    return;
  }

  commandExpiryTime = millis() + COMMAND_HOLD_MS;
  Serial.print(F("Target PWM L/R: "));
  Serial.print(targetLeft);
  Serial.print('/');
  Serial.println(targetRight);
}

void updateMotors() {
  unsigned long now = millis();
  if ((long)(now - lastMotorUpdate) < (long)MOTOR_UPDATE_MS) return;
  lastMotorUpdate = now;

  if (commandExpiryTime != 0 && (long)(now - commandExpiryTime) >= 0) {
    targetLeft = targetRight = 0;
    commandExpiryTime = 0;
    Serial.println(F("Motor command expired; ramping to zero."));
  }

  if (currentLeft < targetLeft) currentLeft = min(currentLeft + PWM_STEP, targetLeft);
  else if (currentLeft > targetLeft) currentLeft = max(currentLeft - PWM_STEP, targetLeft);
  if (currentRight < targetRight) currentRight = min(currentRight + PWM_STEP, targetRight);
  else if (currentRight > targetRight) currentRight = max(currentRight - PWM_STEP, targetRight);

  setMotorLeft(currentLeft);
  setMotorRight(currentRight);
}

void setMotorLeft(int command) {
  command = constrain(command, -MAX_PWM, MAX_PWM);
  if (command >= 0) {
    digitalWrite(LEFT_DIR_PIN, HIGH);
    analogWrite(LEFT_PWM_PIN, command);
  } else {
    digitalWrite(LEFT_DIR_PIN, LOW);
    analogWrite(LEFT_PWM_PIN, -command);
  }
}

void setMotorRight(int command) {
  command = constrain(command, -MAX_PWM, MAX_PWM);
  if (command >= 0) {
    digitalWrite(RIGHT_DIR_PIN, LOW);
    analogWrite(RIGHT_PWM_PIN, command);
  } else {
    digitalWrite(RIGHT_DIR_PIN, HIGH);
    analogWrite(RIGHT_PWM_PIN, -command);
  }
}

void printTelemetry() {
  unsigned long now = millis();
  if (now - lastTelemetryTime < TELEMETRY_INTERVAL_MS) return;
  lastTelemetryTime = now;

  Serial.print(now); Serial.print(',');
  Serial.print(filteredPitch, 3); Serial.print(',');
  Serial.print(pitchRate, 3); Serial.print(',');
  Serial.print(accelPitch, 3); Serial.print(',');
  Serial.print(currentLeft); Serial.print(',');
  Serial.print(currentRight); Serial.print(',');
  Serial.print(digitalRead(RIGHT_HA_PIN)); Serial.print(',');
  Serial.print(digitalRead(RIGHT_HB_PIN)); Serial.print(',');
  Serial.print(digitalRead(RIGHT_HC_PIN)); Serial.print(',');
  Serial.print(digitalRead(LEFT_HA_PIN)); Serial.print(',');
  Serial.print(digitalRead(LEFT_HB_PIN)); Serial.print(',');
  Serial.println(digitalRead(LEFT_HC_PIN));

}
