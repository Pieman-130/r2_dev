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
// MOTOR CONTROL
// ============================================================

const int MAX_PWM = 51;       // Approximately 20% maximum
const int PWM_STEP = 1;       // Maximum PWM change per update

int currentLeftPWM = 0;
int currentRightPWM = 0;

int targetLeftPWM = 0;
int targetRightPWM = 0;

unsigned long lastMotorUpdate = 0;
unsigned long lastCommandTime = 0;

const unsigned long MOTOR_UPDATE_INTERVAL = 10;
const unsigned long COMMAND_TIMEOUT = 250;


// ============================================================
// SETUP
// ============================================================

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println();
  Serial.println("Starting robot controller...");
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

  filteredPitch = atan2(-ax, az) * 180.0 / PI;

  Serial.println();
  Serial.print("Gyro Y bias: ");
  Serial.print(gyroYBias, 4);
  Serial.println(" deg/s");

  Serial.print("Initial accelerometer pitch: ");
  Serial.print(filteredPitch, 2);
  Serial.println(" deg");

  Serial.println();
  Serial.println("Controller ready.");
  Serial.println();
  Serial.println("Commands:");
  Serial.println("  F = forward");
  Serial.println("  B = reverse");
  Serial.println("  L = left turn");
  Serial.println("  R = right turn");
  Serial.println("  S = stop");
  Serial.println();

  lastMicros = micros();
  lastMotorUpdate = millis();
  lastCommandTime = millis();
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop()
{
  // ----------------------------------------------------------
  // Read motor commands from serial
  // ----------------------------------------------------------

  while (Serial.available() > 0)
  {
    char command = Serial.read();

    if (command == 'F' || command == 'f')
    {
      targetLeftPWM = MAX_PWM;
      targetRightPWM = MAX_PWM;
      lastCommandTime = millis();
    }

    else if (command == 'B' || command == 'b')
    {
      targetLeftPWM = -MAX_PWM;
      targetRightPWM = -MAX_PWM;
      lastCommandTime = millis();
    }

    else if (command == 'L' || command == 'l')
    {
      targetLeftPWM = -MAX_PWM;
      targetRightPWM = MAX_PWM;
      lastCommandTime = millis();
    }

    else if (command == 'R' || command == 'r')
    {
      targetLeftPWM = MAX_PWM;
      targetRightPWM = -MAX_PWM;
      lastCommandTime = millis();
    }

    else if (command == 'S' || command == 's')
    {
      targetLeftPWM = 0;
      targetRightPWM = 0;
      lastCommandTime = millis();
    }
  }


  // ----------------------------------------------------------
  // Serial watchdog
  // ----------------------------------------------------------

  if (millis() - lastCommandTime > COMMAND_TIMEOUT)
  {
    targetLeftPWM = 0;
    targetRightPWM = 0;
  }


  // ----------------------------------------------------------
  // Update motors
  // ----------------------------------------------------------

  if (millis() - lastMotorUpdate >= MOTOR_UPDATE_INTERVAL)
  {
    lastMotorUpdate = millis();

    currentLeftPWM = approach(
      currentLeftPWM,
      targetLeftPWM,
      PWM_STEP
    );

    currentRightPWM = approach(
      currentRightPWM,
      targetRightPWM,
      PWM_STEP
    );

    setMotorLeft(currentLeftPWM);
    setMotorRight(currentRightPWM);
  }


  // ----------------------------------------------------------
  // Update IMU
  // ----------------------------------------------------------

  if (!imu.dataReady())
  {
    return;
  }

  imu.getAGMT();

  unsigned long now = micros();

  float dt = (now - lastMicros) / 1000000.0;

  lastMicros = now;

  if (dt <= 0.0 || dt > 0.1)
  {
    return;
  }

  float ax = imu.accX();
  float az = imu.accZ();

  // Positive pitch = robot tilted backward
  // Negative pitch = robot tilted forward

  float accelPitch =
    atan2(-ax, az) * 180.0 / PI;

  float gyroY = imu.gyrY();

  // Positive pitch rate = rotating toward backward tilt
  float pitchRate =
    -(gyroY - gyroYBias);

  float gyroPitch =
    filteredPitch + pitchRate * dt;

  filteredPitch =
    ALPHA * gyroPitch +
    (1.0 - ALPHA) * accelPitch;


  // ----------------------------------------------------------
  // Telemetry
  // ----------------------------------------------------------

  Serial.print(millis());
  Serial.print(",");
  Serial.print(filteredPitch, 3);
  Serial.print(",");
  Serial.print(pitchRate, 3);
  Serial.print(",");
  Serial.println(accelPitch, 3);
}


// ============================================================
// MOTOR FUNCTIONS
// ============================================================

void setMotorLeft(int pwm)
{
  pwm = constrain(pwm, -MAX_PWM, MAX_PWM);

  if (pwm > 0)
  {
    // LEFT motor forward
    digitalWrite(LEFT_DIR_PIN, HIGH);
    analogWrite(LEFT_PWM_PIN, pwm);
  }

  else if (pwm < 0)
  {
    // LEFT motor reverse
    digitalWrite(LEFT_DIR_PIN, LOW);
    analogWrite(LEFT_PWM_PIN, -pwm);
  }

  else
  {
    analogWrite(LEFT_PWM_PIN, 0);
  }
}


void setMotorRight(int pwm)
{
  pwm = constrain(pwm, -MAX_PWM, MAX_PWM);

  if (pwm > 0)
  {
    // RIGHT motor forward
    digitalWrite(RIGHT_DIR_PIN, LOW);
    analogWrite(RIGHT_PWM_PIN, pwm);
  }

  else if (pwm < 0)
  {
    // RIGHT motor reverse
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

  gyroYBias = sum / CALIBRATION_SAMPLES;

  Serial.println("Calibration complete.");
}