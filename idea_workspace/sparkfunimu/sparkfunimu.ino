
#include <Wire.h>
#include "ICM_20948.h"

ICM_20948_I2C imu;

#define AD0_VAL 1

// --------------------------------------------------
// Filter settings
// --------------------------------------------------

const float ALPHA = 0.98;
const int CALIBRATION_SAMPLES = 500;

// --------------------------------------------------
// State
// --------------------------------------------------

float gyroYBias = 0.0;
float filteredPitch = 0.0;

float previousPitchRate = 0.0;

unsigned long lastMicros = 0;
unsigned long recordingStartMs = 0;

// --------------------------------------------------
// Setup
// --------------------------------------------------

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("Starting ICM-20948...");

  Wire.begin();
  Wire.setClock(400000);

  while (imu.begin(Wire, AD0_VAL) != ICM_20948_Stat_Ok)
  {
    Serial.println("IMU initialization failed.");
    delay(500);
  }

  Serial.println("IMU initialized.");
  Serial.println();

  // -----------------------------------------------
  // Calibrate gyro
  // -----------------------------------------------

  calibrateGyro();

  // -----------------------------------------------
  // Wait for first valid sample
  // -----------------------------------------------

  while (!imu.dataReady())
  {
    delay(1);
  }

  imu.getAGMT();

  float ax = imu.accX();
  float az = imu.accZ();

  // Establish initial pitch from accelerometer
  filteredPitch = atan2(-ax, az) * 180.0 / PI;

  Serial.println();
  Serial.print("Gyro Y bias: ");
  Serial.print(gyroYBias, 4);
  Serial.println(" deg/s");

  Serial.print("Initial pitch: ");
  Serial.print(filteredPitch, 3);
  Serial.println(" deg");

  Serial.println();
  Serial.println("==========================================");
  Serial.println("READY FOR DYNAMIC TEST");
  Serial.println("==========================================");
  Serial.println();
  Serial.println("Place robot upright and stable.");
  Serial.println("Then press any key in Serial Monitor");
  Serial.println("to begin recording.");
  Serial.println();

  // -----------------------------------------------
  // Wait for user to start test
  // -----------------------------------------------

  while (!Serial.available())
  {
    // Keep waiting
  }

  // Clear input
  while (Serial.available())
  {
    Serial.read();
  }

  // -----------------------------------------------
  // Re-zero pitch immediately before test
  // -----------------------------------------------

  while (!imu.dataReady())
  {
    delay(1);
  }

  imu.getAGMT();

  ax = imu.accX();
  az = imu.accZ();

  filteredPitch = atan2(-ax, az) * 180.0 / PI;

  previousPitchRate = 0.0;

  lastMicros = micros();
  recordingStartMs = millis();

  Serial.println();
  Serial.println("RECORDING STARTED");
  Serial.println("Push robot forward, then release.");
  Serial.println("Let it oscillate until it settles.");
  Serial.println();
  Serial.println("time_ms,pitch,pitchRate,angularAccel,accelPitch");
}

// --------------------------------------------------
// Main loop
// --------------------------------------------------

void loop()
{
  if (!imu.dataReady())
  {
    return;
  }

  imu.getAGMT();

  unsigned long nowMicros = micros();

  float dt = (nowMicros - lastMicros) / 1000000.0;

  lastMicros = nowMicros;

  // Ignore invalid timing intervals
  if (dt <= 0.0 || dt > 0.1)
  {
    return;
  }

  // -----------------------------------------------
  // Accelerometer
  // -----------------------------------------------

  float ax = imu.accX();
  float az = imu.accZ();

  float accelPitch =
      atan2(-ax, az) * 180.0 / PI;

  // -----------------------------------------------
  // Gyroscope
  // -----------------------------------------------

  float gyroY = imu.gyrY();

  // Positive pitch = robot tilting backward
  // Negative pitch = robot tilting forward
  //
  // Physical gyro sign is opposite our pitch convention.

  float pitchRate =
      -(gyroY - gyroYBias);

  // -----------------------------------------------
  // Complementary filter
  // -----------------------------------------------

  float gyroPitch =
      filteredPitch + pitchRate * dt;

  filteredPitch =
      ALPHA * gyroPitch +
      (1.0 - ALPHA) * accelPitch;

  // -----------------------------------------------
  // Angular acceleration
  // -----------------------------------------------

  float angularAccel =
      (pitchRate - previousPitchRate) / dt;

  previousPitchRate = pitchRate;

  // -----------------------------------------------
  // Telemetry
  // -----------------------------------------------

  unsigned long elapsedMs =
      millis() - recordingStartMs;

  Serial.print(elapsedMs);
  Serial.print(",");

  Serial.print(filteredPitch, 3);
  Serial.print(",");

  Serial.print(pitchRate, 3);
  Serial.print(",");

  Serial.print(angularAccel, 3);
  Serial.print(",");

  Serial.println(accelPitch, 3);
}

// --------------------------------------------------
// Gyro calibration
// --------------------------------------------------

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