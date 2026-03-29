#include <Wire.h>
#include <VL53L0X.h>

// --- Pin & Address Constants ---
const int MPU_addr1 = 0x68;
const int VL53_addr  = 0x29;  // default VL53L0X address
const int head       = 9;     // head servo pin D9

// --- Servo Limits ---
const int up_limit   = 220;   // highest position
const int down_limit = 130;   // lowest position
const int straight   = 154;   // center / 0 deg pitch / looking straight forward

// --- Timing ---
const unsigned long REPORT_INTERVAL  = 200;   // ms between telemetry packets
const unsigned long SERVO_TIMEOUT_MS = 20000; // 20s auto-return to center

// --- Servo Speed Control ---
int SERVO_STEP_DELAY = 15;  // ms between each step (higher = slower)
const int SERVO_STEP_SIZE  = 1;   // servo units per step (higher = faster)
int targetServoVal = straight;    // where we want to get to

// --- Packet Protocol ---
// Command packet (host → Arduino): [0xAA, 0x01, angle_byte, checksum]
// Telemetry packet (Arduino → host): [0xBB, dist_hi, dist_lo, pitch_hi, pitch_lo, checksum]
//   distance in mm (uint16), pitch in 10ths of degrees (int16)

const byte CMD_HEADER    = 0xAA;
const byte CMD_TYPE_SERVO = 0x01;
const byte CMD_TYPE_SPEED = 0x02;
const byte TEL_HEADER    = 0xBB;

float pitch = 0.0;
int   currentServoVal = straight;
unsigned long lastCmdTime    = 0;
unsigned long lastReportTime = 0;

VL53L0X distSensor;

// -------------------------------------------------------
void setup() {
  Wire.begin();

  // Wake MPU6050
  Wire.beginTransmission(MPU_addr1);
  Wire.write(0x6B);
  Wire.write(0);
  byte err = Wire.endTransmission(true);

  // Init VL53L0X
  distSensor.init();
  distSensor.setTimeout(500);
  distSensor.startContinuous();

  pinMode(head, OUTPUT);
  analogWrite(head, straight);

  Serial.begin(9600);
}

// -------------------------------------------------------
float get_pitch() {
  Wire.beginTransmission(MPU_addr1);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr1, 6, true);

  float xa = (int16_t)(Wire.read() << 8 | Wire.read());
  float ya = (int16_t)(Wire.read() << 8 | Wire.read());
  float za = (int16_t)(Wire.read() << 8 | Wire.read());

  return atan2(ya, za) * 180.0 / PI;
}

// -------------------------------------------------------
byte calcChecksum(byte *buf, int len) {
  byte cs = 0;
  for (int i = 0; i < len; i++) cs ^= buf[i];
  return cs;
}

// -------------------------------------------------------
void sendTelemetry(uint16_t distMM, float pitchDeg) {
  int16_t pitchTenths = (int16_t)(pitchDeg * 10.0);

  byte pkt[6];
  pkt[0] = TEL_HEADER;
  pkt[1] = (distMM >> 8) & 0xFF;
  pkt[2] = distMM & 0xFF;
  pkt[3] = (pitchTenths >> 8) & 0xFF;
  pkt[4] = pitchTenths & 0xFF;
  pkt[5] = calcChecksum(pkt, 5);   // XOR of bytes 0-4

  Serial.write(pkt, 6);
}

// -------------------------------------------------------
void updateServo() {
  if (currentServoVal == targetServoVal) return;  // already at target

  unsigned long now = millis();
  static unsigned long lastStepTime = 0;

  if (now - lastStepTime < SERVO_STEP_DELAY) return;  // not time to step yet
  lastStepTime = now;

  if (currentServoVal < targetServoVal) {
    currentServoVal = min(currentServoVal + SERVO_STEP_SIZE, targetServoVal);
  } else {
    currentServoVal = max(currentServoVal - SERVO_STEP_SIZE, targetServoVal);
  }

  analogWrite(head, currentServoVal);
}


void processIncoming() {
  // Expect: [0xAA, 0x01/0x02, angle_byte, checksum]
  static byte buf[4];
  static int  idx = 0;

  while (Serial.available()) {
    byte b = Serial.read();

    if (idx == 0) {
      if (b == CMD_HEADER) buf[idx++] = b;
      // else discard
    } else {
      buf[idx++] = b;
      if (idx == 4) {
        idx = 0;
        byte expected = calcChecksum(buf, 3);  // XOR of bytes 0-2
        if (buf[3] == expected) {
          if (buf[1] == CMD_TYPE_SERVO) {
            // Clamp to safe range
            int angle = constrain(buf[2], down_limit, up_limit);
            targetServoVal = angle;
            lastCmdTime     = millis();
          } else if (buf[1] == CMD_TYPE_SPEED) {
            SERVO_STEP_DELAY = buf[2];
          }
        // bad checksum / unknown type → silently drop
      }
    }
  }
  }}

// -------------------------------------------------------
void loop() {
  unsigned long now = millis();

  // Handle incoming serial commands
  processIncoming();
  updateServo();

  // Auto-return to center after timeout
  if (lastCmdTime > 0 && (now - lastCmdTime >= SERVO_TIMEOUT_MS)) {
    //analogWrite(head, straight);
    targetServoVal = straight;
    lastCmdTime = 0;
  }

  // Send telemetry at regular interval
  if (now - lastReportTime >= REPORT_INTERVAL) {
    lastReportTime = now;
    pitch = get_pitch();
    uint16_t distMM = distSensor.readRangeContinuousMillimeters();
    sendTelemetry(distMM, pitch);
  }
}