#include <Wire.h>
#include "ICM_20948.h"

// ============================================================
// MANUAL MOTOR TEST + WHEEL ENCODER LOGGER
//
// NO BALANCE CONTROL
//
// This sketch is intentionally based on the known-good simple
// motor-control sketch. It accepts the same motor packets but
// does NOT attempt to balance the robot.
//
// Wheel encoders:
//   D0 = right wheel SC
//   D7 = left wheel SC
//
// SC changes state 90 times per wheel revolution.
// Both rising and falling edges are counted.
// ============================================================

// ============================================================
// MOTOR PINS
// ============================================================

const int LEFT_PWM_PIN  = 9;
const int RIGHT_PWM_PIN = 10;

const int LEFT_DIR_PIN  = 5;
const int RIGHT_DIR_PIN = 4;

// ============================================================
// WHEEL ENCODERS
// ============================================================

const int RIGHT_HALL_PIN = 0;  // D0
const int LEFT_HALL_PIN  = 7;  // D7

const uint32_t HALL_TRANSITIONS_PER_REV = 90;
const uint32_t HALL_TIMEOUT_US = 1000000UL;

// ISR state
volatile uint32_t rightLastTransitionUs = 0;
volatile uint32_t leftLastTransitionUs = 0;

volatile uint32_t rightPeriodUs = 0;
volatile uint32_t leftPeriodUs = 0;

volatile uint32_t rightTransitionCount = 0;
volatile uint32_t leftTransitionCount = 0;

volatile bool rightHallInitialized = false;
volatile bool leftHallInitialized = false;

// Position zero is established after startup.
uint32_t rightStartCount = 0;
uint32_t leftStartCount = 0;

// Calculated measurements
float rightRPM = 0.0;
float leftRPM = 0.0;

// Signed only by commanded motor direction.
// This is diagnostic and does NOT affect motor control.
float rightSignedRPM = 0.0;
float leftSignedRPM = 0.0;

// ============================================================
// MOTOR LIMITS / SAFETY
// ============================================================

const int MAX_PWM = 51;
const int PWM_STEP = 1;
const unsigned long MOTOR_UPDATE_MS = 10;
const unsigned long SERIAL_TIMEOUT_MS = 250;

// ============================================================
// IMU
// ============================================================
//
// We keep the IMU running and report pitch/pitchRate so that
// this test can later be compared directly with balance tests.
// The IMU is NOT used to control the motors.
//

ICM_20948_I2C imu;

#define AD0_VAL 1

const float ALPHA = 0.98;
const int CALIBRATION_SAMPLES = 500;

float gyroYBias = 0.0;
float filteredPitch = 0.0;

unsigned long lastIMUmicros = 0;
unsigned long lastMotorUpdate = 0;
unsigned long lastCommandTime = 0;

// ============================================================
// MOTOR COMMAND STATE
// ============================================================

int currentLeft = 0;
int currentRight = 0;

int targetLeft = 0;
int targetRight = 0;

const unsigned long TELEMETRY_INTERVAL_MS = 50;
unsigned long lastTelemetryTime = 0;

// ============================================================
// ENCODER INTERRUPTS
// ============================================================

void rightHallISR()
{
  uint32_t now = micros();

  if (rightHallInitialized)
  {
    uint32_t period = now - rightLastTransitionUs;

    if (period > 0)
      rightPeriodUs = period;
  }

  rightLastTransitionUs = now;
  rightTransitionCount++;
  rightHallInitialized = true;
}


void leftHallISR()
{
  uint32_t now = micros();

  if (leftHallInitialized)
  {
    uint32_t period = now - leftLastTransitionUs;

    if (period > 0)
      leftPeriodUs = period;
  }

  leftLastTransitionUs = now;
  leftTransitionCount++;
  leftHallInitialized = true;
}


// ============================================================
// WHEEL MEASUREMENTS
// ============================================================

void updateWheelMeasurements()
{
  uint32_t nowUs = micros();

  uint32_t rPeriod;
  uint32_t lPeriod;
  uint32_t rLast;
  uint32_t lLast;

  noInterrupts();

  rPeriod = rightPeriodUs;
  lPeriod = leftPeriodUs;
  rLast = rightLastTransitionUs;
  lLast = leftLastTransitionUs;

  interrupts();

  if (rightHallInitialized &&
      rPeriod > 0 &&
      (uint32_t)(nowUs - rLast) <= HALL_TIMEOUT_US)
  {
    rightRPM =
      60000000.0 /
      ((float)rPeriod * HALL_TRANSITIONS_PER_REV);
  }
  else
  {
    rightRPM = 0.0;
  }

  if (leftHallInitialized &&
      lPeriod > 0 &&
      (uint32_t)(nowUs - lLast) <= HALL_TIMEOUT_US)
  {
    leftRPM =
      60000000.0 /
      ((float)lPeriod * HALL_TRANSITIONS_PER_REV);
  }
  else
  {
    leftRPM = 0.0;
  }

  // Provisional signed RPM for diagnostics only.
  if (currentRight > 0)
    rightSignedRPM = rightRPM;
  else if (currentRight < 0)
    rightSignedRPM = -rightRPM;
  else
    rightSignedRPM = 0.0;

  if (currentLeft > 0)
    leftSignedRPM = leftRPM;
  else if (currentLeft < 0)
    leftSignedRPM = -leftRPM;
  else
    leftSignedRPM = 0.0;
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);
  delay(1000);

  // Motors
  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);

  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);

  analogWrite(LEFT_PWM_PIN, 0);
  analogWrite(RIGHT_PWM_PIN, 0);

  // Hall inputs
  pinMode(RIGHT_HALL_PIN, INPUT);
  pinMode(LEFT_HALL_PIN, INPUT);

  // Establish initial interrupt timing before enabling interrupts.
  uint32_t nowUs = micros();

  rightLastTransitionUs = nowUs;
  leftLastTransitionUs = nowUs;

  attachInterrupt(
    digitalPinToInterrupt(RIGHT_HALL_PIN),
    rightHallISR,
    CHANGE
  );

  attachInterrupt(
    digitalPinToInterrupt(LEFT_HALL_PIN),
    leftHallISR,
    CHANGE
  );

  // IMU
  Wire.begin();
  Wire.setClock(400000);

  Serial.println();
  Serial.println("Starting ICM-20948...");

  while (imu.begin(Wire, AD0_VAL) != ICM_20948_Stat_Ok)
  {
    Serial.println("IMU initialization failed.");
    delay(500);
  }

  Serial.println("IMU initialized.");
  Serial.println();

  calibrateGyro();

  while (!imu.dataReady())
  {
    delay(1);
  }

  imu.getAGMT();

  float ax = imu.accX();
  float az = imu.accZ();

  filteredPitch =
    atan2(-ax, az) * 180.0 / PI;

  noInterrupts();
  rightStartCount = rightTransitionCount;
  leftStartCount = leftTransitionCount;
  interrupts();

  Serial.println();
  Serial.print("Gyro Y bias: ");
  Serial.print(gyroYBias, 4);
  Serial.println(" deg/s");

  Serial.print("Initial pitch: ");
  Serial.print(filteredPitch, 2);
  Serial.println(" deg");

  Serial.println();
  Serial.println("READY - BALANCE CONTROL DISABLED");
  Serial.println();

  Serial.println(
    "time_ms,"
    "pitch,"
    "pitchRate,"
    "accelPitch,"
    "currentLeft,"
    "currentRight,"
    "rightRPM,"
    "leftRPM,"
    "rightSignedRPM,"
    "leftSignedRPM,"
    "rightTransitionCount,"
    "leftTransitionCount,"
    "rightPositionTransitions,"
    "leftPositionTransitions,"
    "rightPeriodUs,"
    "leftPeriodUs"
  );

  lastIMUmicros = micros();
  lastMotorUpdate = millis();
  lastCommandTime = millis();
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
  readIMU();
  processSerial();
  updateMotors();
  updateWheelMeasurements();
}


// ============================================================
// IMU
// ============================================================

void readIMU()
{
  if (!imu.dataReady())
    return;

  imu.getAGMT();

  unsigned long now = micros();

  float dt =
    (now - lastIMUmicros) / 1000000.0;

  lastIMUmicros = now;

  if (dt <= 0.0 || dt > 0.1)
    return;

  float ax = imu.accX();
  float az = imu.accZ();

  float accelPitch =
    atan2(-ax, az) * 180.0 / PI;

  float gyroY = imu.gyrY();

  float pitchRate =
    -(gyroY - gyroYBias);

  float gyroPitch =
    filteredPitch + pitchRate * dt;

  filteredPitch =
    ALPHA * gyroPitch +
    (1.0 - ALPHA) * accelPitch;

  if (millis() - lastTelemetryTime >= TELEMETRY_INTERVAL_MS)
  {
    lastTelemetryTime = millis();

    uint32_t rCount;
    uint32_t lCount;
    uint32_t rPeriod;
    uint32_t lPeriod;

    noInterrupts();

    rCount = rightTransitionCount;
    lCount = leftTransitionCount;
    rPeriod = rightPeriodUs;
    lPeriod = leftPeriodUs;

    interrupts();

    Serial.print(millis());
    Serial.print(",");
    Serial.print(filteredPitch, 3);
    Serial.print(",");
    Serial.print(pitchRate, 3);
    Serial.print(",");
    Serial.print(accelPitch, 3);
    Serial.print(",");
    Serial.print(currentLeft);
    Serial.print(",");
    Serial.print(currentRight);
    Serial.print(",");
    Serial.print(rightRPM, 3);
    Serial.print(",");
    Serial.print(leftRPM, 3);
    Serial.print(",");
    Serial.print(rightSignedRPM, 3);
    Serial.print(",");
    Serial.print(leftSignedRPM, 3);
    Serial.print(",");
    Serial.print(rCount);
    Serial.print(",");
    Serial.print(lCount);
    Serial.print(",");
    Serial.print(rCount - rightStartCount);
    Serial.print(",");
    Serial.print(lCount - leftStartCount);
    Serial.print(",");
    Serial.print(rPeriod);
    Serial.print(",");
    Serial.println(lPeriod);
  }
}


// ============================================================
// MOTOR CONTROL
// ============================================================

void updateMotors()
{
  unsigned long now = millis();

  if (now - lastMotorUpdate < MOTOR_UPDATE_MS)
    return;

  lastMotorUpdate = now;

  // Serial watchdog
  if (now - lastCommandTime > SERIAL_TIMEOUT_MS)
  {
    targetLeft = 0;
    targetRight = 0;
  }

  // Gradually move current command toward target.
  if (currentLeft < targetLeft)
    currentLeft += PWM_STEP;
  else if (currentLeft > targetLeft)
    currentLeft -= PWM_STEP;

  if (currentRight < targetRight)
    currentRight += PWM_STEP;
  else if (currentRight > targetRight)
    currentRight -= PWM_STEP;

  setMotorLeft(currentLeft);
  setMotorRight(currentRight);
}


// ============================================================
// LEFT MOTOR
// ============================================================

void setMotorLeft(int command)
{
  command = constrain(command, -MAX_PWM, MAX_PWM);

  if (command >= 0)
  {
    digitalWrite(LEFT_DIR_PIN, HIGH);
    analogWrite(LEFT_PWM_PIN, command);
  }
  else
  {
    digitalWrite(LEFT_DIR_PIN, LOW);
    analogWrite(LEFT_PWM_PIN, -command);
  }
}


// ============================================================
// RIGHT MOTOR
// ============================================================

void setMotorRight(int command)
{
  command = constrain(command, -MAX_PWM, MAX_PWM);

  if (command >= 0)
  {
    digitalWrite(RIGHT_DIR_PIN, LOW);
    analogWrite(RIGHT_PWM_PIN, command);
  }
  else
  {
    digitalWrite(RIGHT_DIR_PIN, HIGH);
    analogWrite(RIGHT_PWM_PIN, -command);
  }
}


// ============================================================
// SERIAL COMMAND PROCESSING
//
// Packet:
//
// AA 55 LEN CMD LEFT RIGHT CRC
//
// CMD = 0x01
// LEFT / RIGHT = signed int8, -100 to +100
// ============================================================

void processSerial()
{
  static uint8_t buffer[16];
  static int index = 0;

  while (Serial.available())
  {
    uint8_t byteReceived = Serial.read();

    if (index == 0)
    {
      if (byteReceived == 0xAA)
        buffer[index++] = byteReceived;

      continue;
    }

    if (index == 1)
    {
      if (byteReceived == 0x55)
        buffer[index++] = byteReceived;
      else
        index = 0;

      continue;
    }

    buffer[index++] = byteReceived;

    if (index >= 7)
    {
      uint8_t len = buffer[2];

      if (len != 4)
      {
        index = 0;
        continue;
      }

      uint8_t crcReceived = buffer[6];
      uint8_t crcCalculated = crc8(buffer, 6);

      if (crcReceived == crcCalculated &&
          buffer[3] == 0x01)
      {
        int left = (int8_t)buffer[4];
        int right = (int8_t)buffer[5];

        targetLeft =
          constrain(
            map(abs(left), 0, 100, 0, MAX_PWM),
            0,
            MAX_PWM
          );

        targetRight =
          constrain(
            map(abs(right), 0, 100, 0, MAX_PWM),
            0,
            MAX_PWM
          );

        if (left < 0)
          targetLeft = -targetLeft;

        if (right < 0)
          targetRight = -targetRight;

        lastCommandTime = millis();
      }

      index = 0;
    }
  }
}


// ============================================================
// CRC-8
// ============================================================

uint8_t crc8(uint8_t *data, int length)
{
  uint8_t crc = 0x00;

  for (int i = 0; i < length; i++)
  {
    crc ^= data[i];

    for (int j = 0; j < 8; j++)
    {
      if (crc & 0x80)
        crc = (crc << 1) ^ 0x07;
      else
        crc <<= 1;
    }
  }

  return crc;
}


// ============================================================
// GYRO CALIBRATION
// ============================================================

void calibrateGyro()
{
  Serial.println("GYRO CALIBRATION");
  Serial.println("-----------------");
  Serial.println("Keep the robot COMPLETELY STILL.");
  Serial.println("The robot does NOT need to be upright.");
  Serial.println("Calibration will begin in 2 seconds.");

  delay(2000);

  float sum = 0.0;
  int samples = 0;

  Serial.println("Calibrating...");

  while (samples < CALIBRATION_SAMPLES)
  {
    if (imu.dataReady())
    {
      imu.getAGMT();

      sum += imu.gyrY();

      samples++;
    }
  }

  gyroYBias = sum / CALIBRATION_SAMPLES;

  Serial.println("Calibration complete.");
}
