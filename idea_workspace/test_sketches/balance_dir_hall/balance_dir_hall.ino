/*
  R2 balance controller with Hall-derived wheel velocity telemetry
  Board: Arduino Leonardo / ATmega32U4

  IMPORTANT:
  - Balance control law, gains, limits, ramp, deadband, IMU filter,
    calibration and safety thresholds are preserved from the prior
    r2_balance_with_encoders.ino baseline.
  - Wheel velocity is telemetry ONLY. It is not used by the controller.
  - Hall decoding follows the validated period-preferred estimator.

  Current wiring:
    IMU I2C: SDA D2, SCL D3
    Right direction D4; left direction D12
    Left PWM D5; right PWM D6
    Left Hall A/B/C: D0, D1, D7
    Right Hall A/B/C: D8, D9, D10
    Left SC D18; right SC D19 (not used for velocity)

  Hall calibration: 90 transitions/revolution on each wheel
  Wheel radius: 0.075 m
*/

#include <Arduino.h>
#include <Wire.h>
#include "ICM_20948.h"

ICM_20948_I2C imu;
#define AD0_VAL 1

// -------------------- IMU / balance baseline --------------------
const float ALPHA = 0.98f;
const int CALIBRATION_SAMPLES = 500;
float gyroYBias = 0.0f;
float filteredPitch = 0.0f;

const uint8_t LEFT_PWM_PIN  = 5;
const uint8_t RIGHT_PWM_PIN = 6;
const uint8_t LEFT_DIR_PIN  = 12;
const uint8_t RIGHT_DIR_PIN = 4;

const int MAX_PWM = 51;
const int BALANCE_MAX = 40;
const float MAX_PITCH_RATE = 120.0f;
const int PWM_STEP = 5;
const int MOTOR_DEADBAND = 8;

int currentLeftMotor = 0;
int currentRightMotor = 0;

float Kp = 2.0f;
float Kd = 0.30f;
float Ki = 0.0f; // No integral term; retained from baseline.

const float STARTUP_ANGLE = 5.0f;
const float ABORT_ANGLE = 45.0f;
bool balanceEnabled = false;
bool balanceFault = false;
int faultCode = 0;

// -------------------- Hall inputs / velocity telemetry --------------------
const uint8_t LEFT_HA_PIN  = 0;
const uint8_t LEFT_HB_PIN  = 1;
const uint8_t LEFT_HC_PIN  = 7;
const uint8_t RIGHT_HA_PIN = 8;
const uint8_t RIGHT_HB_PIN = 9;
const uint8_t RIGHT_HC_PIN = 10;

const float HALL_TRANSITIONS_PER_REV = 90.0f;
const float WHEEL_RADIUS_M = 0.075f;
const uint32_t TRANSITION_STALE_US = 350000UL;
const float RPM_FILTER_TAU_SECONDS = 0.30f;
const unsigned long VELOCITY_UPDATE_MS = 50UL;

volatile uint8_t leftState = 0;
volatile uint8_t rightState = 0;
volatile long leftSignedTransitions = 0;
volatile long rightSignedTransitions = 0;
volatile unsigned long leftValidTransitions = 0;
volatile unsigned long rightValidTransitions = 0;
volatile unsigned long leftInvalidTransitions = 0;
volatile unsigned long rightInvalidTransitions = 0;
volatile unsigned long leftLastTransitionUs = 0;
volatile unsigned long rightLastTransitionUs = 0;
volatile unsigned long leftPeriodUs = 0;
volatile unsigned long rightPeriodUs = 0;
volatile int8_t leftLastDirection = 0;
volatile int8_t rightLastDirection = 0;

// The decoder learns each motor's six-state ring from its first observed
// direction. As in the validated test, signed direction is relative to that
// initial learned direction; verify physical forward sign in the telemetry.
struct StateDecoder {
  uint8_t ring[6];
  uint8_t ringCount;
  uint8_t previous;
  bool initialized;
  int8_t lastDirection;
  long signedCount;
  unsigned long validCount;
  unsigned long invalidCount;
  unsigned long lastUs;
  unsigned long periodUs;

  void begin(uint8_t initialState) {
    ringCount = 1;
    ring[0] = initialState;
    previous = initialState;
    initialized = true;
    lastDirection = 0;
    signedCount = 0;
    validCount = 0;
    invalidCount = 0;
    lastUs = 0;
    periodUs = 0;
  }

  int indexOf(uint8_t state) {
    for (uint8_t i = 0; i < ringCount; ++i) {
      if (ring[i] == state) return i;
    }
    return -1;
  }

  void update(uint8_t state, unsigned long nowUs) {
    if (!initialized) { begin(state); return; }
    if (state == previous) return;

    uint8_t changed = previous ^ state;
    if (changed == 0 || (changed & (changed - 1)) != 0) {
      ++invalidCount;
      previous = state;
      return;
    }

    int oldIdx = indexOf(previous);
    int newIdx = indexOf(state);

    if (ringCount < 6 && oldIdx == (int)ringCount - 1 && newIdx < 0) {
      ring[ringCount++] = state;
      ++validCount;
      if (lastDirection == 0) lastDirection = +1;
      signedCount += lastDirection;
      if (lastUs != 0) periodUs = nowUs - lastUs;
      lastUs = nowUs;
    } else if (oldIdx >= 0 && newIdx >= 0) {
      int forward = (newIdx - oldIdx + 6) % 6;
      int direction = 0;
      if (forward == 1) direction = +1;
      else if (forward == 5) direction = -1;

      if (direction == 0) {
        ++invalidCount;
      } else {
        ++validCount;
        lastDirection = direction;
        signedCount += direction;
        if (lastUs != 0) periodUs = nowUs - lastUs;
        lastUs = nowUs;
      }
    } else {
      ++invalidCount;
      ringCount = 1;
      ring[0] = state;
      lastDirection = 0;
    }
    previous = state;
  }
};

StateDecoder leftDecoder;
StateDecoder rightDecoder;

long previousLeftCount = 0;
long previousRightCount = 0;
unsigned long previousVelocitySampleMs = 0;
float leftRpmCount = 0.0f, rightRpmCount = 0.0f;
float leftRpmPeriod = 0.0f, rightRpmPeriod = 0.0f;
float leftRpmSelected = 0.0f, rightRpmSelected = 0.0f;
float leftRpmFiltered = 0.0f, rightRpmFiltered = 0.0f;
float leftVelocityFiltered = 0.0f, rightVelocityFiltered = 0.0f;
uint8_t leftVelocityMode = 0, rightVelocityMode = 0; // 0 stale, 1 period, 2 count fallback
unsigned long leftTransitionAgeMs = 0, rightTransitionAgeMs = 0;

// -------------------- Telemetry / timing / gyro stats --------------------
unsigned long lastTelemetryTime = 0;
const unsigned long TELEMETRY_INTERVAL = 50UL;
unsigned long lastIMUTime = 0;
float gyroMin = 999999.0f;
float gyroMax = -999999.0f;
float gyroSum = 0.0f;
unsigned long gyroSampleCount = 0;

void resetGyroStats() {
  gyroMin = 999999.0f;
  gyroMax = -999999.0f;
  gyroSum = 0.0f;
  gyroSampleCount = 0;
}

void updateGyroStats(float value) {
  if (value < gyroMin) gyroMin = value;
  if (value > gyroMax) gyroMax = value;
  gyroSum += value;
  ++gyroSampleCount;
}

uint8_t readLeftState() {
  return (digitalRead(LEFT_HA_PIN) << 2) |
         (digitalRead(LEFT_HB_PIN) << 1) |
          digitalRead(LEFT_HC_PIN);
}

uint8_t readRightState() {
  return (digitalRead(RIGHT_HA_PIN) << 2) |
         (digitalRead(RIGHT_HB_PIN) << 1) |
          digitalRead(RIGHT_HC_PIN);
}

void leftHallISR() {
  uint8_t state = readLeftState();
  leftDecoder.update(state, micros());
  leftState = state;
  leftSignedTransitions = leftDecoder.signedCount;
  leftValidTransitions = leftDecoder.validCount;
  leftInvalidTransitions = leftDecoder.invalidCount;
  leftPeriodUs = leftDecoder.periodUs;
  leftLastTransitionUs = leftDecoder.lastUs;
  leftLastDirection = leftDecoder.lastDirection;
}

ISR(PCINT0_vect) {
  uint8_t state = readRightState();
  rightDecoder.update(state, micros());
  rightState = state;
  rightSignedTransitions = rightDecoder.signedCount;
  rightValidTransitions = rightDecoder.validCount;
  rightInvalidTransitions = rightDecoder.invalidCount;
  rightPeriodUs = rightDecoder.periodUs;
  rightLastTransitionUs = rightDecoder.lastUs;
  rightLastDirection = rightDecoder.lastDirection;
}

void updateWheelVelocityTelemetry(unsigned long nowMs) {
  if (nowMs - previousVelocitySampleMs < VELOCITY_UPDATE_MS) return;

  unsigned long sampleDtMs = nowMs - previousVelocitySampleMs;
  unsigned long nowUs = micros();
  long lc, rc;
  unsigned long lp, rp, lLast, rLast;
  int8_t lDir, rDir;

  noInterrupts();
  lc = leftSignedTransitions;
  rc = rightSignedTransitions;
  lp = leftPeriodUs;
  rp = rightPeriodUs;
  lLast = leftLastTransitionUs;
  rLast = rightLastTransitionUs;
  lDir = leftLastDirection;
  rDir = rightLastDirection;
  interrupts();

  long deltaLeft = lc - previousLeftCount;
  long deltaRight = rc - previousRightCount;
  float dt = sampleDtMs / 1000.0f;
  leftRpmCount = ((float)deltaLeft / HALL_TRANSITIONS_PER_REV) * (60.0f / dt);
  rightRpmCount = ((float)deltaRight / HALL_TRANSITIONS_PER_REV) * (60.0f / dt);

  bool leftFresh = (lLast != 0 && (uint32_t)(nowUs - lLast) <= TRANSITION_STALE_US);
  bool rightFresh = (rLast != 0 && (uint32_t)(nowUs - rLast) <= TRANSITION_STALE_US);
  leftTransitionAgeMs = (lLast == 0) ? 0xFFFFFFFFUL : (nowUs - lLast) / 1000UL;
  rightTransitionAgeMs = (rLast == 0) ? 0xFFFFFFFFUL : (nowUs - rLast) / 1000UL;

  leftRpmPeriod = (lp > 0 && lDir != 0)
      ? (float)lDir * 60000000.0f / (HALL_TRANSITIONS_PER_REV * (float)lp) : 0.0f;
  rightRpmPeriod = (rp > 0 && rDir != 0)
      ? (float)rDir * 60000000.0f / (HALL_TRANSITIONS_PER_REV * (float)rp) : 0.0f;

  if (!leftFresh) { leftRpmSelected = 0.0f; leftVelocityMode = 0; }
  else if (lp > 0 && lDir != 0) { leftRpmSelected = leftRpmPeriod; leftVelocityMode = 1; }
  else if (deltaLeft != 0) { leftRpmSelected = leftRpmCount; leftVelocityMode = 2; }
  else { leftRpmSelected = 0.0f; leftVelocityMode = 0; }

  if (!rightFresh) { rightRpmSelected = 0.0f; rightVelocityMode = 0; }
  else if (rp > 0 && rDir != 0) { rightRpmSelected = rightRpmPeriod; rightVelocityMode = 1; }
  else if (deltaRight != 0) { rightRpmSelected = rightRpmCount; rightVelocityMode = 2; }
  else { rightRpmSelected = 0.0f; rightVelocityMode = 0; }

  // Time-constant low-pass filter (tau 0.30 s). Telemetry only.
  float alpha = dt / (RPM_FILTER_TAU_SECONDS + dt);
  leftRpmFiltered += alpha * (leftRpmSelected - leftRpmFiltered);
  rightRpmFiltered += alpha * (rightRpmSelected - rightRpmFiltered);
  const float circumference = 2.0f * PI * WHEEL_RADIUS_M;
  leftVelocityFiltered = leftRpmFiltered * circumference / 60.0f;
  rightVelocityFiltered = rightRpmFiltered * circumference / 60.0f;

  previousLeftCount = lc;
  previousRightCount = rc;
  previousVelocitySampleMs = nowMs;
}

// -------------------- Motor control (unchanged baseline) --------------------
void setMotorLeft(int command) {
  command = constrain(command, -MAX_PWM, MAX_PWM);
  if (command > 0) digitalWrite(LEFT_DIR_PIN, HIGH);
  else if (command < 0) digitalWrite(LEFT_DIR_PIN, LOW);
  analogWrite(LEFT_PWM_PIN, abs(command));
}

void setMotorRight(int command) {
  command = constrain(command, -MAX_PWM, MAX_PWM);
  if (command > 0) digitalWrite(RIGHT_DIR_PIN, LOW);
  else if (command < 0) digitalWrite(RIGHT_DIR_PIN, HIGH);
  analogWrite(RIGHT_PWM_PIN, abs(command));
}

void stopMotors() {
  currentLeftMotor = 0;
  currentRightMotor = 0;
  analogWrite(LEFT_PWM_PIN, 0);
  analogWrite(RIGHT_PWM_PIN, 0);
}

int rampMotor(int current, int target) {
  if (current < target) {
    current += PWM_STEP;
    if (current > target) current = target;
  } else if (current > target) {
    current -= PWM_STEP;
    if (current < target) current = target;
  }
  return current;
}

float applyMotorDeadband(float output) {
  if (output > 0.0f) output += MOTOR_DEADBAND;
  else if (output < 0.0f) output -= MOTOR_DEADBAND;
  return output;
}

void calibrateGyro() {
  Serial.println();
  Serial.println(F("================================"));
  Serial.println(F("GYRO CALIBRATION"));
  Serial.println(F("Keep robot completely stationary"));
  Serial.println(F("================================"));
  delay(1000);
  float sum = 0.0f;
  for (int i = 0; i < CALIBRATION_SAMPLES; ++i) {
    while (!imu.dataReady()) delay(1);
    imu.getAGMT();
    sum += imu.gyrY();
    delay(10);
  }
  gyroYBias = sum / CALIBRATION_SAMPLES;
  Serial.print(F("Gyro Y bias: "));
  Serial.println(gyroYBias, 4);
  Serial.println(F("Calibration complete."));
}

bool updateIMU(float &pitch, float &pitchRate, float &rawGyroY, float &accelPitch) {
  if (!imu.dataReady()) return false;
  imu.getAGMT();
  float ax = imu.accX();
  float az = imu.accZ();
  accelPitch = atan2(-ax, az) * 180.0f / PI;
  rawGyroY = imu.gyrY();
  updateGyroStats(rawGyroY);
  pitchRate = -(rawGyroY - gyroYBias);

  if (fabs(pitchRate) >= MAX_PITCH_RATE) {
    balanceEnabled = false;
    balanceFault = true;
    faultCode = 2;
    stopMotors();
    Serial.println();
    Serial.println(F("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"));
    Serial.println(F("BALANCE FAULT: PITCH RATE LIMIT EXCEEDED"));
    Serial.print(F("Raw gyro Y: ")); Serial.println(rawGyroY, 2);
    Serial.print(F("Pitch rate: ")); Serial.println(pitchRate, 2);
    Serial.println(F("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"));
    return false;
  }

  unsigned long now = micros();
  float dt = (now - lastIMUTime) / 1000000.0f;
  lastIMUTime = now;
  if (dt <= 0.0f || dt > 0.1f) return false;
  float gyroPitch = filteredPitch + pitchRate * dt;
  filteredPitch = ALPHA * gyroPitch + (1.0f - ALPHA) * accelPitch;
  pitch = filteredPitch;
  return true;
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);

  pinMode(LEFT_HA_PIN, INPUT);
  pinMode(LEFT_HB_PIN, INPUT);
  pinMode(LEFT_HC_PIN, INPUT);
  pinMode(RIGHT_HA_PIN, INPUT);
  pinMode(RIGHT_HB_PIN, INPUT);
  pinMode(RIGHT_HC_PIN, INPUT);
  stopMotors();

  delay(50);
  uint8_t ls = readLeftState();
  uint8_t rs = readRightState();
  leftDecoder.begin(ls);
  rightDecoder.begin(rs);
  leftState = ls;
  rightState = rs;

  attachInterrupt(digitalPinToInterrupt(LEFT_HA_PIN), leftHallISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_HB_PIN), leftHallISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_HC_PIN), leftHallISR, CHANGE);

  // Leonardo D8/D9/D10 = PB4/PB5/PB6; use Port-B pin-change interrupt.
  PCIFR |= _BV(PCIF0);
  PCMSK0 |= _BV(PCINT4) | _BV(PCINT5) | _BV(PCINT6);
  PCICR |= _BV(PCIE0);

  Wire.begin();
  Wire.setClock(400000);
  Serial.println(F("Initializing ICM-20948..."));
  while (imu.begin(Wire, AD0_VAL) != ICM_20948_Stat_Ok) {
    Serial.println(F("IMU initialization failed."));
    delay(1000);
  }
  Serial.println(F("IMU initialized."));
  calibrateGyro();
  lastIMUTime = micros();
  resetGyroStats();
  previousVelocitySampleMs = millis();
  lastTelemetryTime = millis();

  Serial.println(F("BALANCE CONTROLLER READY"));
  Serial.println(F("Hold robot upright. Balance auto-enables within +/-5 degrees."));
  Serial.println(F("Wheel velocity is telemetry ONLY; not used in control."));
  Serial.println(F("time_ms,pitch_deg,pitch_rate_dps,raw_gyro_y,gyro_min,gyro_max,gyro_avg,gyro_range,accel_pitch_deg,controller_output,target_motor,left_motor,right_motor,balance_enabled,balance_fault,fault_code,left_signed_transitions,right_signed_transitions,left_valid,left_invalid,right_valid,right_invalid,left_period_us,right_period_us,left_rpm_period,right_rpm_period,left_rpm_filtered,right_rpm_filtered,left_velocity_filtered_mps,right_velocity_filtered_mps,left_velocity_mode,right_velocity_mode,left_transition_age_ms,right_transition_age_ms"));
}

void loop() {
  float pitch = 0.0f;
  float pitchRate = 0.0f;
  float rawGyroY = 0.0f;
  float accelPitch = 0.0f;
  if (!updateIMU(pitch, pitchRate, rawGyroY, accelPitch)) return;

  if (!balanceEnabled && !balanceFault && fabs(pitch) <= STARTUP_ANGLE) {
    balanceEnabled = true;
    Serial.println(F("BALANCE ENABLED"));
  }

  if (balanceEnabled && fabs(pitch) >= ABORT_ANGLE) {
    balanceFault = true;
    balanceEnabled = false;
    faultCode = 1;
    stopMotors();
    Serial.println();
    Serial.println(F("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"));
    Serial.println(F("BALANCE FAULT: ANGLE LIMIT EXCEEDED"));
    Serial.print(F("Pitch: ")); Serial.println(pitch, 2);
    Serial.println(F("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"));
  }

  // UNCHANGED balance controller: wheel velocity is deliberately not used.
  float controllerOutput = 0.0f;
  int targetMotor = 0;
  if (balanceEnabled && !balanceFault) {
    controllerOutput = (Kp * pitch) - (Kd * pitchRate);
    controllerOutput = applyMotorDeadband(controllerOutput);
    controllerOutput = constrain(controllerOutput, -BALANCE_MAX, BALANCE_MAX);
    targetMotor = (int)controllerOutput;
  } else {
    targetMotor = 0;
  }

  currentLeftMotor = rampMotor(currentLeftMotor, targetMotor);
  currentRightMotor = rampMotor(currentRightMotor, targetMotor);
  setMotorLeft(currentLeftMotor);
  setMotorRight(currentRightMotor);

  unsigned long nowMs = millis();
  updateWheelVelocityTelemetry(nowMs);

  if (nowMs - lastTelemetryTime >= TELEMETRY_INTERVAL) {
    lastTelemetryTime = nowMs;
    float gyroAvg = 0.0f;
    float gyroRange = 0.0f;
    if (gyroSampleCount > 0) {
      gyroAvg = gyroSum / (float)gyroSampleCount;
      gyroRange = gyroMax - gyroMin;
    }

    long lc, rc;
    unsigned long lv, li, rv, ri, lp, rp;
    noInterrupts();
    lc = leftSignedTransitions;
    rc = rightSignedTransitions;
    lv = leftValidTransitions;
    li = leftInvalidTransitions;
    rv = rightValidTransitions;
    ri = rightInvalidTransitions;
    lp = leftPeriodUs;
    rp = rightPeriodUs;
    interrupts();

    Serial.print(nowMs); Serial.print(',');
    Serial.print(pitch, 3); Serial.print(',');
    Serial.print(pitchRate, 3); Serial.print(',');
    Serial.print(rawGyroY, 3); Serial.print(',');
    Serial.print(gyroMin, 3); Serial.print(',');
    Serial.print(gyroMax, 3); Serial.print(',');
    Serial.print(gyroAvg, 3); Serial.print(',');
    Serial.print(gyroRange, 3); Serial.print(',');
    Serial.print(accelPitch, 3); Serial.print(',');
    Serial.print(controllerOutput, 3); Serial.print(',');
    Serial.print(targetMotor); Serial.print(',');
    Serial.print(currentLeftMotor); Serial.print(',');
    Serial.print(currentRightMotor); Serial.print(',');
    Serial.print(balanceEnabled ? 1 : 0); Serial.print(',');
    Serial.print(balanceFault ? 1 : 0); Serial.print(',');
    Serial.print(faultCode); Serial.print(',');
    Serial.print(lc); Serial.print(',');
    Serial.print(rc); Serial.print(',');
    Serial.print(lv); Serial.print(',');
    Serial.print(li); Serial.print(',');
    Serial.print(rv); Serial.print(',');
    Serial.print(ri); Serial.print(',');
    Serial.print(lp); Serial.print(',');
    Serial.print(rp); Serial.print(',');
    Serial.print(leftRpmPeriod, 3); Serial.print(',');
    Serial.print(rightRpmPeriod, 3); Serial.print(',');
    Serial.print(leftRpmFiltered, 3); Serial.print(',');
    Serial.print(rightRpmFiltered, 3); Serial.print(',');
    Serial.print(leftVelocityFiltered, 4); Serial.print(',');
    Serial.print(rightVelocityFiltered, 4); Serial.print(',');
    Serial.print(leftVelocityMode); Serial.print(',');
    Serial.print(rightVelocityMode); Serial.print(',');
    Serial.print(leftTransitionAgeMs); Serial.print(',');
    Serial.println(rightTransitionAgeMs);
    resetGyroStats();
  }
}
