#include <Wire.h>
#include "ICM_20948.h"

// ============================================================
// IMU
// ============================================================

ICM_20948_I2C imu;

#define AD0_VAL 1

const float ALPHA = 0.98;
const int CALIBRATION_SAMPLES = 500;

float gyroYBias = 0.0;
float filteredPitch = 0.0;

unsigned long lastMicros = 0;


// ============================================================
// BALANCE CONTROL TEST
// ============================================================

// This is intentionally just a test gain.
// It is NOT a tuned balance gain.

const float Kp = 2.0;

// Maximum hypothetical motor command.
const int MAX_COMMAND = 51;


// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("Balance Control Direction Test");
  Serial.println("--------------------------------");
  Serial.println();

  Wire.begin();
  Wire.setClock(400000);

  // ----------------------------------------------------------
  // Initialize IMU
  // ----------------------------------------------------------

  while (imu.begin(Wire, AD0_VAL) != ICM_20948_Stat_Ok)
  {
    Serial.println("IMU initialization failed.");
    delay(500);
  }

  Serial.println("IMU initialized.");
  Serial.println();

  // ----------------------------------------------------------
  // Calibrate gyro
  // ----------------------------------------------------------

  calibrateGyro();

  // ----------------------------------------------------------
  // Establish initial pitch
  // ----------------------------------------------------------

  while (!imu.dataReady())
  {
    delay(1);
  }

  imu.getAGMT();

  float ax = imu.accX();
  float az = imu.accZ();

  filteredPitch =
    atan2(-ax, az) * 180.0 / PI;

  Serial.print("Initial pitch: ");
  Serial.print(filteredPitch, 2);
  Serial.println(" degrees");

  Serial.println();
  Serial.println("Motors are DISABLED.");
  Serial.println();
  Serial.println("Expected behavior:");
  Serial.println("  Forward tilt  (negative pitch) -> positive command");
  Serial.println("  Backward tilt (positive pitch) -> negative command");
  Serial.println();
  Serial.println("time_ms,pitch,pitchRate,accelPitch,command");

  lastMicros = micros();
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
  if (!imu.dataReady())
  {
    return;
  }

  imu.getAGMT();

  // ----------------------------------------------------------
  // Calculate dt
  // ----------------------------------------------------------

  unsigned long now = micros();

  float dt =
    (now - lastMicros) / 1000000.0;

  lastMicros = now;

  if (dt <= 0.0 || dt > 0.1)
  {
    return;
  }

  // ----------------------------------------------------------
  // Accelerometer pitch
  // ----------------------------------------------------------

  float ax = imu.accX();
  float az = imu.accZ();

  float accelPitch =
    atan2(-ax, az) * 180.0 / PI;

  // ----------------------------------------------------------
  // Gyroscope pitch rate
  // ----------------------------------------------------------

  float gyroY = imu.gyrY();

  float pitchRate =
    -(gyroY - gyroYBias);

  // ----------------------------------------------------------
  // Complementary filter
  // ----------------------------------------------------------

  float gyroPitch =
    filteredPitch + pitchRate * dt;

  filteredPitch =
    ALPHA * gyroPitch +
    (1.0 - ALPHA) * accelPitch;


  // ----------------------------------------------------------
  // Calculate hypothetical balance command
  // ----------------------------------------------------------

  // Negative pitch = falling forward
  // Therefore command should be positive (forward).

  float balanceCommand =
    -Kp * filteredPitch;

  balanceCommand =
    constrain(
      balanceCommand,
      -MAX_COMMAND,
      MAX_COMMAND
    );


  // ----------------------------------------------------------
  // Telemetry
  // ----------------------------------------------------------

  Serial.print(millis());
  Serial.print(",");
  Serial.print(filteredPitch, 3);
  Serial.print(",");
  Serial.print(pitchRate, 3);
  Serial.print(",");
  Serial.print(accelPitch, 3);
  Serial.print(",");
  Serial.println(balanceCommand, 3);
}


// ============================================================
// GYRO CALIBRATION
// ============================================================

void calibrateGyro()
{
  Serial.println("GYRO CALIBRATION");
  Serial.println("-----------------");
  Serial.println("Keep the robot COMPLETELY STILL.");
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