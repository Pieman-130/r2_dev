
#include <Wire.h>
#include "ICM_20948.h"

// ============================================================
// MOTOR PINS
// ============================================================

const int LEFT_PWM_PIN  = 9;
const int RIGHT_PWM_PIN = 10;

const int LEFT_DIR_PIN  = 5;
const int RIGHT_DIR_PIN = 4;

// ============================================================
// MOTOR LIMITS / SAFETY
// ============================================================

const int MAX_PWM = 51;       // ~20% maximum
const int PWM_STEP = 1;      // Maximum PWM change per update
const unsigned long MOTOR_UPDATE_MS = 10;
const unsigned long SERIAL_TIMEOUT_MS = 250;

// ============================================================
// IMU
// ============================================================

ICM_20948_I2C imu;

#define AD0_VAL 1

const float ALPHA = 0.98;
const int CALIBRATION_SAMPLES = 500;

float gyroYBias = 0.0;
float filteredPitch = 0.0;

unsigned long lastIMUmicros = 0;
unsigned long lastMotorUpdate = 0;
unsigned long lastCommandTime = 0;

// Current and requested motor commands
int currentLeft = 0;
int currentRight = 0;

int targetLeft = 0;
int targetRight = 0;

const unsigned long TELEMETRY_INTERVAL_MS = 50;
unsigned long lastTelemetryTime = 0;

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

  // Gyro calibration
  calibrateGyro();

  // Get initial accelerometer angle
  while (!imu.dataReady())
  {
    delay(1);
  }

  imu.getAGMT();

  float ax = imu.accX();
  float az = imu.accZ();

  filteredPitch = atan2(-ax, az) * 180.0 / PI;

  Serial.println();
  Serial.print("Gyro Y bias: ");
  Serial.print(gyroYBias, 4);
  Serial.println(" deg/s");

  Serial.print("Initial pitch: ");
  Serial.print(filteredPitch, 2);
  Serial.println(" deg");

  Serial.println();
  Serial.println("READY");
  Serial.println();
  Serial.println("time_ms,pitch,pitchRate,accelPitch,left,right");

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
}

// ============================================================
// IMU
// ============================================================

void readIMU()
{
  if (!imu.dataReady())
  {
    return;
  }

  imu.getAGMT();

  unsigned long now = micros();

  float dt = (now - lastIMUmicros) / 1000000.0;
  lastIMUmicros = now;

  if (dt <= 0.0 || dt > 0.1)
  {
    return;
  }

  float ax = imu.accX();
  float az = imu.accZ();

  // Our established convention:
  //
  // Positive pitch = robot tilted backward
  // Negative pitch = robot tilted forward

  float accelPitch =
      atan2(-ax, az) * 180.0 / PI;

  float gyroY = imu.gyrY();

  // Established gyro sign convention:
  // positive physical pitch rate corresponds to
  // negative raw gyro Y

  float pitchRate =
      -(gyroY - gyroYBias);

  float gyroPitch =
      filteredPitch + pitchRate * dt;

  filteredPitch =
      ALPHA * gyroPitch +
      (1.0 - ALPHA) * accelPitch;

  // Telemetry

if (millis() - lastTelemetryTime >= TELEMETRY_INTERVAL_MS)
{
  lastTelemetryTime = millis();

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
  Serial.println(currentRight);
}

}

// ============================================================
// MOTOR CONTROL
// ============================================================

void updateMotors()
{
  unsigned long now = millis();

  if (now - lastMotorUpdate < MOTOR_UPDATE_MS)
  {
    return;
  }

  lastMotorUpdate = now;

  // Serial watchdog
  if (now - lastCommandTime > SERIAL_TIMEOUT_MS)
  {
    targetLeft = 0;
    targetRight = 0;
  }

  // Gradually move current command toward target
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
// ============================================================
//
// Packet:
//
// AA 55 LEN CMD LEFT RIGHT CRC
//
// CMD = 0x01
// LEFT / RIGHT = signed int8, -100 to +100
// CRC-8 polynomial = 0x07
//

void processSerial()
{
  static uint8_t buffer[16];
  static int index = 0;

  while (Serial.available())
  {
    uint8_t byteReceived = Serial.read();

    // Look for first preamble byte
    if (index == 0)
    {
      if (byteReceived == 0xAA)
      {
        buffer[index++] = byteReceived;
      }

      continue;
    }

    // Look for second preamble byte
    if (index == 1)
    {
      if (byteReceived == 0x55)
      {
        buffer[index++] = byteReceived;
      }
      else
      {
        index = 0;
      }

      continue;
    }

    buffer[index++] = byteReceived;

    // We expect a 7-byte packet:
    //
    // 0 AA
    // 1 55
    // 2 LEN
    // 3 CMD
    // 4 LEFT
    // 5 RIGHT
    // 6 CRC

    if (index >= 7)
    {
      uint8_t len = buffer[2];

      if (len != 4)
      {
        index = 0;
        continue;
      }

      uint8_t crcReceived = buffer[6];

      uint8_t crcCalculated =
          crc8(buffer, 6);

    if (crcReceived == crcCalculated &&
    buffer[3] == 0x01)
{
    int left  = (int8_t)buffer[4];
    int right = (int8_t)buffer[5];

    Serial.print("CMD RECEIVED: ");
    Serial.print(left);
    Serial.print(",");
    Serial.println(right);

    targetLeft =
        constrain(map(abs(left), 0, 100, 0, MAX_PWM), 0, MAX_PWM);

    targetRight =
        constrain(map(abs(right), 0, 100, 0, MAX_PWM), 0, MAX_PWM);

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

  gyroYBias =
      sum / CALIBRATION_SAMPLES;

  Serial.println("Calibration complete.");
}