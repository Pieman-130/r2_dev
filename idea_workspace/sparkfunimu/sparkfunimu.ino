
#include <Wire.h>
#include "ICM_20948.h"

ICM_20948_I2C imu;

#define AD0_VAL 1

// --------------------------------------------------
// Filter parameters
// --------------------------------------------------

const float ALPHA = 0.98;

// Number of gyro samples used for startup calibration
const int GYRO_CALIBRATION_SAMPLES = 500;


// --------------------------------------------------
// Gyro calibration
// --------------------------------------------------

float gyroYBias = 0.0;


// --------------------------------------------------
// Filter state
// --------------------------------------------------

float filteredPitch = 0.0;

unsigned long previousTime = 0;

unsigned long guardTime = millis();

// --------------------------------------------------
// Setup
// --------------------------------------------------

void setup()
{
  Serial.begin(115200);

  // Give USB serial time to initialize
  delay(1000);

  Wire.begin();
  Wire.setClock(400000);

  Serial.println();
  Serial.println("ICM-20948 Pitch Filter");
  Serial.println("======================");

  // Initialize IMU
  imu.begin(Wire, AD0_VAL);

  if (imu.status != ICM_20948_Stat_Ok)
  {
    Serial.print("IMU initialization failed: ");
    Serial.println(imu.statusString());

    while (1)
    {
      delay(1000);
    }
  }

  Serial.println("IMU initialized.");
  Serial.println();

  // ------------------------------------------------
  // Gyro calibration
  // ------------------------------------------------

  Serial.println("Keep robot completely still.");
  Serial.println("Calibrating gyro...");

  float gyroSum = 0.0;

  int samples = 0;

  while (samples < GYRO_CALIBRATION_SAMPLES)
  {
    if (imu.dataReady())
    {
      imu.getAGMT();

      gyroSum += imu.gyrY();

      samples++;

      delay(2);
    }
  }

  gyroYBias = gyroSum / GYRO_CALIBRATION_SAMPLES;

  Serial.print("Gyro Y bias: ");
  Serial.print(gyroYBias, 4);
  Serial.println(" deg/s");

  Serial.println();

  // ------------------------------------------------
  // Initialize pitch from accelerometer
  // ------------------------------------------------

  while (!imu.dataReady())
  {
    delay(1);
  }

  imu.getAGMT();

  float ax = imu.accX();
  float az = imu.accZ();

  filteredPitch =
    atan2(ax, az) * 180.0 / PI;

  previousTime = micros();

  Serial.println("Filter running.");
  Serial.println();
  Serial.println("AccelPitch   GyroRate   FilteredPitch");
  Serial.println("-------------------------------------");

 
}


// --------------------------------------------------
// Main loop
// --------------------------------------------------

void loop()
{
  if (!imu.dataReady())
    return;

  imu.getAGMT();

  // ------------------------------------------------
  // Calculate time step
  // ------------------------------------------------

  unsigned long currentTime = micros();

  float dt =
    (currentTime - previousTime) / 1000000.0;

  previousTime = currentTime;

  // Protect against unreasonable time steps
  if (dt <= 0.0 || dt > 0.1)
    return;


  // ------------------------------------------------
  // Accelerometer pitch
  // ------------------------------------------------

  float ax = imu.accX();
  float az = imu.accZ();

  float accelPitch =
    atan2(ax, az) * 180.0 / PI;


  // ------------------------------------------------
  // Gyroscope pitch rate
  //
  // Positive = rotating toward forward pitch
  // ------------------------------------------------

  float gyroRate =
    -(imu.gyrY() - gyroYBias);


  // ------------------------------------------------
  // Integrate gyro
  // ------------------------------------------------

  float gyroPitch =
    filteredPitch + gyroRate * dt;


  // ------------------------------------------------
  // Complementary filter
  // ------------------------------------------------

  filteredPitch =
    ALPHA * gyroPitch +
    (1.0 - ALPHA) * accelPitch;


  // ------------------------------------------------
  // Output
  // ------------------------------------------------
  if (millis() - guardTime > 500){
  Serial.print(accelPitch, 2);
  Serial.print("       ");

  Serial.print(gyroRate, 2);
  Serial.print("       ");

  Serial.println(filteredPitch, 2);
  guardTime = millis();
  }
}