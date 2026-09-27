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


// ============================================================
// MOTOR PINS
// ============================================================

const int LEFT_PWM_PIN  = 9;
const int RIGHT_PWM_PIN = 10;

const int LEFT_DIR_PIN  = 5;
const int RIGHT_DIR_PIN = 4;


// ============================================================
// MOTOR LIMITS
// ============================================================

const int MAX_PWM = 51;

const int BALANCE_MAX = 40;

const float MAX_PITCH_RATE = 120.0;

const int PWM_STEP = 5;

const int MOTOR_DEADBAND = 8;

int currentLeftMotor = 0;
int currentRightMotor = 0;


// ============================================================
// BALANCE CONTROLLER
// ============================================================

float Kp = 2.0;
float Kd = 0.30;
float Ki = 0.0;


// ============================================================
// BALANCE SAFETY
// ============================================================

const float STARTUP_ANGLE = 5.0;
const float ABORT_ANGLE = 45.0;

bool balanceEnabled = false;
bool balanceFault = false;

int faultCode = 0;


// ============================================================
// TELEMETRY
// ============================================================

unsigned long lastTelemetryTime = 0;
const unsigned long TELEMETRY_INTERVAL = 50;


// ============================================================
// IMU TIMING
// ============================================================

unsigned long lastIMUTime = 0;


// ============================================================
// GYRO STATISTICS
//
// These are calculated over each 50 ms telemetry window.
// ============================================================

float gyroMin = 999999.0;
float gyroMax = -999999.0;
float gyroSum = 0.0;
unsigned long gyroSampleCount = 0;


// ============================================================
// RESET GYRO STATISTICS
// ============================================================

void resetGyroStats()
{
  gyroMin = 999999.0;
  gyroMax = -999999.0;
  gyroSum = 0.0;
  gyroSampleCount = 0;
}


// ============================================================
// UPDATE GYRO STATISTICS
// ============================================================

void updateGyroStats(float gyroValue)
{
  if (gyroValue < gyroMin)
    gyroMin = gyroValue;

  if (gyroValue > gyroMax)
    gyroMax = gyroValue;

  gyroSum += gyroValue;

  gyroSampleCount++;
}


// ============================================================
// MOTOR CONTROL
// ============================================================

void setMotorLeft(int command)
{
  command = constrain(command, -MAX_PWM, MAX_PWM);

  if (command > 0)
  {
    digitalWrite(LEFT_DIR_PIN, HIGH);
  }
  else if (command < 0)
  {
    digitalWrite(LEFT_DIR_PIN, LOW);
  }

  analogWrite(LEFT_PWM_PIN, abs(command));
}


void setMotorRight(int command)
{
  command = constrain(command, -MAX_PWM, MAX_PWM);

  if (command > 0)
  {
    digitalWrite(RIGHT_DIR_PIN, LOW);
  }
  else if (command < 0)
  {
    digitalWrite(RIGHT_DIR_PIN, HIGH);
  }

  analogWrite(RIGHT_PWM_PIN, abs(command));
}


// ============================================================
// STOP MOTORS
// ============================================================

void stopMotors()
{
  currentLeftMotor = 0;
  currentRightMotor = 0;

  analogWrite(LEFT_PWM_PIN, 0);
  analogWrite(RIGHT_PWM_PIN, 0);
}


// ============================================================
// RAMP MOTOR COMMAND
// ============================================================

int rampMotor(int current, int target)
{
  if (current < target)
  {
    current += PWM_STEP;

    if (current > target)
      current = target;
  }
  else if (current > target)
  {
    current -= PWM_STEP;

    if (current < target)
      current = target;
  }

  return current;
}


// ============================================================
// MOTOR DEADBAND
// ============================================================

float applyMotorDeadband(float output)
{
  if (output > 0.0)
  {
    output += MOTOR_DEADBAND;
  }
  else if (output < 0.0)
  {
    output -= MOTOR_DEADBAND;
  }

  return output;
}


// ============================================================
// GYRO CALIBRATION
// ============================================================

void calibrateGyro()
{
  Serial.println();
  Serial.println("================================");
  Serial.println("GYRO CALIBRATION");
  Serial.println("Keep robot completely stationary");
  Serial.println("================================");

  delay(1000);

  float sum = 0.0;

  for (int i = 0; i < CALIBRATION_SAMPLES; i++)
  {
    while (!imu.dataReady())
    {
      delay(1);
    }

    imu.getAGMT();

    sum += imu.gyrY();

    delay(10);
  }

  gyroYBias = sum / CALIBRATION_SAMPLES;

  Serial.print("Gyro Y bias: ");
  Serial.println(gyroYBias, 4);

  Serial.println("Calibration complete.");
  Serial.println();
}


// ============================================================
// UPDATE IMU
// ============================================================

bool updateIMU(
  float &pitch,
  float &pitchRate,
  float &rawGyroY,
  float &accelPitch
)
{
  if (!imu.dataReady())
    return false;

  imu.getAGMT();

  // ----------------------------------------------------------
  // Accelerometer pitch
  // ----------------------------------------------------------

  float ax = imu.accX();
  float az = imu.accZ();

  accelPitch = atan2(-ax, az) * 180.0 / PI;

  // ----------------------------------------------------------
  // Raw gyro Y
  // ----------------------------------------------------------

  rawGyroY = imu.gyrY();

  // Add EVERY IMU gyro sample to our statistics
  updateGyroStats(rawGyroY);

  // ----------------------------------------------------------
  // Corrected pitch rate
  // ----------------------------------------------------------

  pitchRate = -(rawGyroY - gyroYBias);

  // ----------------------------------------------------------
  // Pitch-rate safety cutoff
  // ----------------------------------------------------------

  if (abs(pitchRate) >= MAX_PITCH_RATE)
  {
    balanceEnabled = false;
    balanceFault = true;
    faultCode = 2;

    stopMotors();

    Serial.println();
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    Serial.println("BALANCE FAULT");
    Serial.println("PITCH RATE LIMIT EXCEEDED");
    Serial.print("Raw gyro Y: ");
    Serial.println(rawGyroY, 2);
    Serial.print("Pitch rate: ");
    Serial.println(pitchRate, 2);
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    Serial.println();

    return false;
  }

  // ----------------------------------------------------------
  // Calculate dt
  // ----------------------------------------------------------

  unsigned long now = micros();

  float dt = (now - lastIMUTime) / 1000000.0;

  lastIMUTime = now;

  if (dt <= 0.0 || dt > 0.1)
    return false;

  // ----------------------------------------------------------
  // Complementary filter
  // ----------------------------------------------------------

  float gyroPitch = filteredPitch + pitchRate * dt;

  filteredPitch =
      ALPHA * gyroPitch +
      (1.0 - ALPHA) * accelPitch;

  pitch = filteredPitch;

  return true;
}


// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);

  delay(1000);

  // ----------------------------------------------------------
  // Motors
  // ----------------------------------------------------------

  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);

  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);

  stopMotors();

  // ----------------------------------------------------------
  // I2C
  // ----------------------------------------------------------

  Wire.begin();
  Wire.setClock(400000);

  // ----------------------------------------------------------
  // IMU
  // ----------------------------------------------------------

  Serial.println("Initializing ICM-20948...");

  while (imu.begin(Wire, AD0_VAL) != ICM_20948_Stat_Ok)
  {
    Serial.println("IMU initialization failed.");
    delay(1000);
  }

  Serial.println("IMU initialized.");

  // ----------------------------------------------------------
  // Gyro calibration
  // ----------------------------------------------------------

  calibrateGyro();

  // ----------------------------------------------------------
  // Timing
  // ----------------------------------------------------------

  lastIMUTime = micros();

  // Start a fresh gyro statistics window
  resetGyroStats();

  Serial.println("================================");
  Serial.println("BALANCE CONTROLLER READY");
  Serial.println("================================");
  Serial.println();

  Serial.println("Hold robot upright.");
  Serial.println("Balance will automatically enable");
  Serial.println("when pitch is within +/-5 degrees.");
  Serial.println();

  // ----------------------------------------------------------
  // CSV HEADER
  // ----------------------------------------------------------

 Serial.println(
  "time_ms,"
  "pitch,"
  "pitchRate,"
  "rawGyroY,"
  "gyroMin,"
  "gyroMax,"
  "gyroAvg,"
  "gyroRange,"
  "accelPitch,"
  "controllerOutput,"
  "targetMotor,"
  "currentLeftMotor,"
  "currentRightMotor,"
  "balanceEnabled,"
  "balanceFault,"
  "faultCode"
);
}


// ============================================================
// LOOP
// ============================================================

void loop()
{
  float pitch = 0.0;
  float pitchRate = 0.0;
  float rawGyroY = 0.0;
  float accelPitch = 0.0;

  // ----------------------------------------------------------
  // Update IMU
  // ----------------------------------------------------------

  if (!updateIMU(
        pitch,
        pitchRate,
        rawGyroY,
        accelPitch))
  {
    return;
  }

  // ----------------------------------------------------------
  // Enable balance
  // ----------------------------------------------------------

  if (!balanceEnabled &&
      !balanceFault &&
      abs(pitch) <= STARTUP_ANGLE)
  {
    balanceEnabled = true;

    Serial.println("BALANCE ENABLED");
  }

  // ----------------------------------------------------------
  // Angle fault
  // ----------------------------------------------------------

  if (balanceEnabled &&
      abs(pitch) >= ABORT_ANGLE)
  {
    balanceFault = true;
    balanceEnabled = false;

    faultCode = 1;

    stopMotors();

    Serial.println();
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    Serial.println("BALANCE FAULT");
    Serial.println("ANGLE LIMIT EXCEEDED");
    Serial.print("Pitch: ");
    Serial.println(pitch, 2);
    Serial.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    Serial.println();
  }

  // ----------------------------------------------------------
  // Controller
  // ----------------------------------------------------------

  float controllerOutput = 0.0;
  int targetMotor = 0;

  if (balanceEnabled && !balanceFault)
  {
    controllerOutput =
        (Kp * pitch) -
        (Kd * pitchRate);

    controllerOutput =
        applyMotorDeadband(controllerOutput);

    controllerOutput =
        constrain(
          controllerOutput,
          -BALANCE_MAX,
          BALANCE_MAX
        );

    targetMotor = (int)controllerOutput;
  }
  else
  {
    targetMotor = 0;
  }

  // ----------------------------------------------------------
  // Ramp motor command
  // ----------------------------------------------------------

  currentLeftMotor =
      rampMotor(currentLeftMotor, targetMotor);

  currentRightMotor =
      rampMotor(currentRightMotor, targetMotor);

  // ----------------------------------------------------------
  // Apply motor command
  // ----------------------------------------------------------

  setMotorLeft(currentLeftMotor);
  setMotorRight(currentRightMotor);

  // ----------------------------------------------------------
  // Telemetry
  // ----------------------------------------------------------

  unsigned long now = millis();

  if (now - lastTelemetryTime >= TELEMETRY_INTERVAL)
  {
    lastTelemetryTime = now;

    // --------------------------------------------------------
    // Calculate gyro statistics for this window
    // --------------------------------------------------------

    float gyroAvg = 0.0;
    float gyroRange = 0.0;

    if (gyroSampleCount > 0)
    {
      gyroAvg =
          gyroSum / (float)gyroSampleCount;

      gyroRange =
          gyroMax - gyroMin;
    }

    // --------------------------------------------------------
    // Output CSV
    // --------------------------------------------------------

Serial.print(now);
Serial.print(",");

Serial.print(pitch, 3);
Serial.print(",");

Serial.print(pitchRate, 3);
Serial.print(",");

Serial.print(rawGyroY, 3);
Serial.print(",");

Serial.print(gyroMin, 3);
Serial.print(",");

Serial.print(gyroMax, 3);
Serial.print(",");

Serial.print(gyroAvg, 3);
Serial.print(",");

Serial.print(gyroRange, 3);
Serial.print(",");

Serial.print(accelPitch, 3);
Serial.print(",");

Serial.print(controllerOutput, 3);
Serial.print(",");

Serial.print(targetMotor);
Serial.print(",");

Serial.print(currentLeftMotor);
Serial.print(",");

Serial.print(currentRightMotor);
Serial.print(",");

Serial.print(balanceEnabled ? 1 : 0);
Serial.print(",");

Serial.print(balanceFault ? 1 : 0);
Serial.print(",");

Serial.println(faultCode);
    // --------------------------------------------------------
    // Start a new statistics window
    // --------------------------------------------------------

    resetGyroStats();
  }
}