#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <AccelStepper.h>
#include <MPU6050.h>

#define ENABLE_TEXT_DEBUG true
#define DEBUG_INTERVAL 200  // ms

#define STEP_PIN 13
#define DIR_PIN  12

#define PREAMBLE  0xAA
#define POSTAMBLE 0x55

#define MAX_PITCH  40.0
#define MIN_PITCH -35.0

#define SERIAL_TIMEOUT 10000  // ms before returning to 0 pitch

// ----- Objects -----
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
MPU6050 mpu;
AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// ----- Variables -----
float targetPitch = 0.0;
float currentPitch = 0.0;
unsigned long lastCommandTime = 0;

const float stepsPerRevolution = 200.0;  // Adjust if needed
const float stepsPerDegree = stepsPerRevolution / 360.0;
const float Kp = 1.0;  // Proportional gain (tune this)

unsigned long lastDebugPrint = 0;

// -------------------- CRC --------------------
uint8_t computeCRC(uint8_t *data, uint8_t length) {
  uint8_t crc = 0;
  for (int i = 0; i < length; i++) {
    crc ^= data[i];
  }
  return crc;
}

// --------------------Debug Human Readable ----------
void sendDebugText(float pitch, float distance) {
  if (!ENABLE_TEXT_DEBUG) return;

  if (millis() - lastDebugPrint >= DEBUG_INTERVAL) {
    lastDebugPrint = millis();

    Serial.print("DBG: Pitch=");
    Serial.print(pitch, 3);
    Serial.print(" deg | Distance=");
    Serial.print(distance, 1);
    Serial.println(" mm");
  }
}

// -------------------- MPU Pitch --------------------
float getPitch() {
  int16_t ax, ay, az;
  mpu.getAcceleration(&ax, &ay, &az);

  float accelX = ax / 16384.0;
  float accelY = ay / 16384.0;
  float accelZ = az / 16384.0;

  float pitch = atan2(accelY, sqrt(accelX * accelX + accelZ * accelZ)) * 180.0 / PI;
  return pitch;
}


// -------------------- Setup --------------------
void setup() {
  Serial.begin(115200);
  Wire.begin();

  // MPU6050
  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 not connected!");
    while (1);
  }

  // VL53L0X
  if (!lox.begin()) {
    Serial.println("VL53L0X not found!");
    while (1);
  }

  // Stepper
  stepper.setMaxSpeed(1000);
  stepper.setAcceleration(1000);

  lastCommandTime = millis();
}


// -------------------- Read Serial Command --------------------
void readSerialCommand() {
  if (Serial.available() >= 7) {
    if (Serial.read() == PREAMBLE) {

      uint8_t buffer[4];
      for (int i = 0; i < 4; i++) {
        buffer[i] = Serial.read();
      }

      uint8_t receivedCRC = Serial.read();
      uint8_t post = Serial.read();

      if (post != POSTAMBLE) return;

      if (computeCRC(buffer, 4) == receivedCRC) {
        float incomingPitch;
        memcpy(&targetPitch, buffer, 4);
        
        // Clamp to allowed range
        if (incomingPitch > MAX_PITCH) incomingPitch = MAX_PITCH;
        if (incomingPitch < MIN_PITCH) incomingPitch = MIN_PITCH;
        
        lastCommandTime = millis();
      }
    }
  }
}


// -------------------- Send Telemetry --------------------
void sendTelemetry(float pitch, float distance) {
  uint8_t payload[8];
  memcpy(payload, &pitch, 4);
  memcpy(payload + 4, &distance, 4);

  uint8_t crc = computeCRC(payload, 8);

  Serial.write(PREAMBLE);
  Serial.write(payload, 8);
  Serial.write(crc);
  Serial.write(POSTAMBLE);
}

// -------------------- Control Loop --------------------
void loop() {

  readSerialCommand();

  // Timeout → return to 0 pitch
  if (millis() - lastCommandTime > SERIAL_TIMEOUT) {
    targetPitch = 0.0;
  }

  // Read sensors
  currentPitch = getPitch();

  // HARD SAFETY: Prevent movement beyond physical limits
  if (currentPitch > MAX_PITCH) {
    targetPitch = MAX_PITCH;
  }
  if (currentPitch < MIN_PITCH) {
    targetPitch = MIN_PITCH;
  }

  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  float distance = measure.RangeMilliMeter;

  // ----- Control -----
  long targetPosition = targetPitch * stepsPerDegree;
  stepper.moveTo(targetPosition);
  stepper.run();

  // Send telemetry
  sendTelemetry(currentPitch, distance);
  sendDebugText(currentPitch, distance);


  delay(50);
}
