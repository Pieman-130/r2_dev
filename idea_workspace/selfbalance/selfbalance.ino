#include <Wire.h>
#include <ICM_20948.h>   // SparkFun ICM-20948 Arduino library

// ---------------- Motor pins ----------------
#define LEFTMOTOR 9
#define RIGHTMOTOR 10
#define LEFTDIR 5
#define RIGHTDIR 4

// ---------------- Serial protocol ----------------
const byte PREAMBLE = 0xAA;
const unsigned long COMM_FAILSAFE_TIMEOUT = 1000; // ms
unsigned long lastCommandTime = 0;

// Drive bias decoded from incoming packets, layered on top of the
// balance controller's output rather than replacing it.
volatile int leftBias = 0;   // signed, roughly -255..255
volatile int rightBias = 0;

// ---------------- IMU ----------------
ICM_20948_I2C myICM;
float pitchAngle = 0.0;         // fused pitch estimate, degrees
const float COMP_ALPHA = 0.98;  // complementary filter weight

// ---------------- Balance PID (tuning-safe starting values) ----------------
float targetAngle = 0.0;   // set by calibrateIMU() at startup
float Kp = 2.0;
float Ki = 0.0;
float Kd = 0.0;
float integralError = 0;
float lastError = 0;
unsigned long lastPIDTime = 0;
const unsigned long PID_INTERVAL_MS = 5;

const float FALL_ANGLE = 45.0;
const float ERROR_DEADBAND = 0.5;        // degrees
const float MAX_OUTPUT_STEP = 15.0;      // max PWM change per loop iteration
const float EXPECTED_G = 1.0;            // 1.0 if accX/Y/Z are in g's; ~9.81 if m/s^2
const float ACCEL_TOLERANCE = 0.15;

float lastPidOutput = 0;  // for slew-rate limiting

void setup() {
  Serial.begin(115200);  // raised from 9600 — see notes
  pinMode(LEFTMOTOR, OUTPUT);
  pinMode(RIGHTMOTOR, OUTPUT);
  pinMode(LEFTDIR, OUTPUT);
  pinMode(RIGHTDIR, OUTPUT);
  stopMotors();

  Wire.begin();
  Wire.setClock(400000);

  bool ok = false;
  while (!ok) {
    myICM.begin(Wire, 1);  // 1 = AD0 pulled high; use 0 if AD0 is low
    if (myICM.status == ICM_20948_Stat_Ok) {
      ok = true;
    } else {
      Serial.println("IMU init failed, retrying...");
      delay(500);
    }
  }

  calibrateIMU();
  lastPIDTime = millis();
}

void loop() {
  readSerialCommands();
  updateBalance();

  // Comms lost: don't cut motor power (the robot would faceplant) —
  // just zero the drive bias so it holds position and keeps balancing.
  if (millis() - lastCommandTime > COMM_FAILSAFE_TIMEOUT) {
    leftBias = 0;
    rightBias = 0;
  }
}

// ---------------- Serial ----------------
void readSerialCommands() {
  if (Serial.available() >= 6) {
    byte preamble   = Serial.read();
    byte leftDir    = Serial.read();
    byte leftSpeed  = Serial.read();
    byte rightDir   = Serial.read();
    byte rightSpeed = Serial.read();
    byte checksum   = Serial.read();
    byte calc = (preamble + leftDir + leftSpeed + rightDir + rightSpeed) & 0xFF;

    if (preamble == PREAMBLE && checksum == calc) {
      leftBias  = leftDir  ? (int)leftSpeed  : -(int)leftSpeed;
      rightBias = rightDir ? (int)rightSpeed : -(int)rightSpeed;
      lastCommandTime = millis();
    } else {
      Serial.println("Checksum error");
      flushSerial();
    }
  }
}

void flushSerial() {
  while (Serial.available()) Serial.read();
}

// ---------------- IMU / balance ----------------
float computeAccelPitch() {
  // Axis choice here depends entirely on how the IMU is mounted on the
  // chassis — confirm empirically (see notes).
  float ax = myICM.accX();
  float az = myICM.accZ();
  return atan2(ax, az) * 180.0 / PI;
}

void calibrateIMU() {
  Serial.println("Calibrating - hold the robot still at its balance point...");
  float sum = 0;
  const int N = 200;
  for (int i = 0; i < N; i++) {
    if (myICM.dataReady()) {
      myICM.getAGMT();
      sum += computeAccelPitch();
    }
    delay(5);
  }
  targetAngle = sum / N;
  pitchAngle = targetAngle;
  Serial.print("Balance point: ");
  Serial.println(targetAngle);
}

void updateBalance() {
  unsigned long now = millis();
  if (now - lastPIDTime < PID_INTERVAL_MS) return;
  float dt = (now - lastPIDTime) / 1000.0;
  lastPIDTime = now;

  if (!myICM.dataReady()) return;
  myICM.getAGMT();
  if (myICM.status != ICM_20948_Stat_Ok) return;  // skip bad I2C reads entirely

  float ax = myICM.accX();
  float ay = myICM.accY();
  float az = myICM.accZ();

  // Only trust the accelerometer's tilt estimate when it looks like pure
  // gravity — reject it during vibration/bumps and fall back to the
  // last known pitch (the gyro term below still updates normally).
  float accelPitch = pitchAngle;
  float accelMagnitude = sqrt(ax*ax + ay*ay + az*az);
  if (fabs(accelMagnitude - EXPECTED_G) < ACCEL_TOLERANCE) {
    accelPitch = atan2(ax, az) * 180.0 / PI;
  }

  float gyroRate = -myICM.gyrY();  // negated to match accelPitch's sign convention

  pitchAngle = COMP_ALPHA * (pitchAngle + gyroRate * dt)
             + (1.0 - COMP_ALPHA) * accelPitch;

  float error = pitchAngle - targetAngle;
  if (fabs(error) < ERROR_DEADBAND) error = 0;  // ignore jitter near vertical

  if (fabs(error) > FALL_ANGLE) {
    stopMotors();
    integralError = 0;
    lastPidOutput = 0;
    return;
  }

  integralError += error * dt;
  integralError = constrain(integralError, -50, 50);
  float derivative = (error - lastError) / dt;
  lastError = error;

  float pidOutput = Kp * error + Ki * integralError + Kd * derivative;
  pidOutput = -pidOutput;  // corrects for the inverted wheel response you found

  // Slew-rate limit: cap how much the output can jump in one loop iteration
  pidOutput = constrain(pidOutput, lastPidOutput - MAX_OUTPUT_STEP, lastPidOutput + MAX_OUTPUT_STEP);
  lastPidOutput = pidOutput;

  int leftCmd  = (int)pidOutput + leftBias;
  int rightCmd = (int)pidOutput + rightBias;

  driveMotor(LEFTMOTOR, LEFTDIR, leftCmd, false);
  driveMotor(RIGHTMOTOR, RIGHTDIR, rightCmd, true);
}

void driveMotor(int pwmPin, int dirPin, int value, bool reversedDir) {
  bool forward = value >= 0;
  bool dirHigh = reversedDir ? !forward : forward;
  digitalWrite(dirPin, dirHigh ? HIGH : LOW);
  analogWrite(pwmPin, constrain(abs(value), 0, 255));
}

void stopMotors() {
  analogWrite(LEFTMOTOR, 0);
  analogWrite(RIGHTMOTOR, 0);
}