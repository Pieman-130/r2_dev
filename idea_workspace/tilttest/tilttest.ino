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
// MOTOR PINS
// ============================================================

const int LEFT_PWM_PIN  = 9;
const int RIGHT_PWM_PIN = 10;

const int LEFT_DIR_PIN  = 5;
const int RIGHT_DIR_PIN = 4;


// ============================================================
// BALANCE CONTROLLER
// ============================================================

// Proportional gain
const float Kp = 1.0;

// Gyro damping gain
const float Kd = 0.15;

// Small region around upright where the controller
// does not command the motors.
const float PITCH_DEADBAND = 1.0;

// Conservative maximum motor command.
const int MAX_PWM = 20;


// ============================================================
// MOTOR SLEW LIMITING
// ============================================================

const int PWM_STEP = 1;

int currentLeftPWM = 0;
int currentRightPWM = 0;

int targetLeftPWM = 0;
int targetRightPWM = 0;

unsigned long lastMotorUpdate = 0;

const unsigned long MOTOR_UPDATE_INTERVAL = 10;


// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("PD BALANCE TEST");
  Serial.println("---------------");
  Serial.println();

  // ----------------------------------------------------------
  // Motor setup
  // ----------------------------------------------------------

  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);

  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);

  analogWrite(LEFT_PWM_PIN, 0);
  analogWrite(RIGHT_PWM_PIN, 0);

  // ----------------------------------------------------------
  // IMU setup
  // ----------------------------------------------------------

  Wire.begin();
  Wire.setClock(400000);

  while (imu.begin(Wire, AD0_VAL) != ICM_20948_Stat_Ok)
  {
    Serial.println("IMU initialization failed.");
    delay(500);
  }

  Serial.println("IMU initialized.");
  Serial.println();

  // ----------------------------------------------------------
  // Gyro calibration
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
  Serial.println("PD controller active.");
  Serial.println("Motors limited to 20 PWM.");
  Serial.println("Keep robot restrained in fixture.");
  Serial.println();

  Serial.println(
    "time_ms,pitch,pitchRate,accelPitch,command"
  );

  lastMicros = micros();
  lastMotorUpdate = millis();
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
  // ----------------------------------------------------------
  // Wait for fresh IMU data
  // ----------------------------------------------------------

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


  // ==========================================================
  // PD BALANCE CONTROLLER
  // ==========================================================

  float balanceCommand = 0.0;

  if (filteredPitch > PITCH_DEADBAND ||
      filteredPitch < -PITCH_DEADBAND)
  {
    balanceCommand =
      Kp * filteredPitch +
      Kd * pitchRate;
  }

  balanceCommand =
    constrain(
      balanceCommand,
      -MAX_PWM,
      MAX_PWM
    );

  targetLeftPWM = (int)balanceCommand;
  targetRightPWM = (int)balanceCommand;


  // ----------------------------------------------------------
  // Update motors
  // ----------------------------------------------------------

  if (millis() - lastMotorUpdate >= MOTOR_UPDATE_INTERVAL)
  {
    lastMotorUpdate = millis();

    currentLeftPWM =
      approach(
        currentLeftPWM,
        targetLeftPWM,
        PWM_STEP
      );

    currentRightPWM =
      approach(
        currentRightPWM,
        targetRightPWM,
        PWM_STEP
      );

    setMotorLeft(currentLeftPWM);
    setMotorRight(currentRightPWM);
  }


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
// LEFT MOTOR
// ============================================================

void setMotorLeft(int pwm)
{
  pwm = constrain(pwm, -MAX_PWM, MAX_PWM);

  if (pwm > 0)
  {
    // Positive = robot forward
    digitalWrite(LEFT_DIR_PIN, HIGH);
    analogWrite(LEFT_PWM_PIN, pwm);
  }
  else if (pwm < 0)
  {
    // Negative = robot reverse
    digitalWrite(LEFT_DIR_PIN, LOW);
    analogWrite(LEFT_PWM_PIN, -pwm);
  }
  else
  {
    analogWrite(LEFT_PWM_PIN, 0);
  }
}


// ============================================================
// RIGHT MOTOR
// ============================================================

void setMotorRight(int pwm)
{
  pwm = constrain(pwm, -MAX_PWM, MAX_PWM);

  if (pwm > 0)
  {
    // Positive = robot forward
    digitalWrite(RIGHT_DIR_PIN, LOW);
    analogWrite(RIGHT_PWM_PIN, pwm);
  }
  else if (pwm < 0)
  {
    // Negative = robot reverse
    digitalWrite(RIGHT_DIR_PIN, HIGH);
    analogWrite(RIGHT_PWM_PIN, -pwm);
  }
  else
  {
    analogWrite(RIGHT_PWM_PIN, 0);
  }
}


// ============================================================
// SLEW RATE LIMITER
// ============================================================

int approach(int current, int target, int amount)
{
  if (current < target)
  {
    current += amount;

    if (current > target)
    {
      current = target;
    }
  }
  else if (current > target)
  {
    current -= amount;

    if (current < target)
    {
      current = target;
    }
  }

  return current;
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