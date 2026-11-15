
/*
  R2 Hall encoder physical direction verification — V1
  Board: Arduino Leonardo / ATmega32U4

  Uses the Hall decoder and pin assignments from the supplied balance_dir_hall.ino.
  Balance controller and IMU are NOT initialized or used.

  IMPORTANT:
  - Securely raise and support R2 with both wheels clear of the ground.
  - First test is BOTH_FWD so each decoder learns its initial direction
    while both motors receive the same positive command.
  - Each command runs for 1500 ms and then stops automatically.
  - PWM is fixed at 8, based on the prior unloaded motor tests.
  - A 2500 ms command watchdog also stops the motors.
  - Commands: BOTH_FWD, LEFT_FWD, RIGHT_FWD, BOTH_REV,
              LEFT_REV, RIGHT_REV, STOP, STATUS

  Motor pins/polarity from supplied sketch:
    Left PWM D5, direction D12; positive = HIGH
    Right PWM D6, direction D4; positive = LOW

  Hall pins:
    Left A/B/C: D0, D1, D7
    Right A/B/C: D8, D9, D10

  Decoder signed counts are relative to the first learned direction.
  The first BOTH_FWD test establishes the reference, provided both wheels
  physically rotate in the robot-forward direction.
*/

#include <Arduino.h>

const uint8_t LEFT_PWM_PIN  = 5;
const uint8_t RIGHT_PWM_PIN = 6;
const uint8_t LEFT_DIR_PIN  = 12;
const uint8_t RIGHT_DIR_PIN = 4;

const uint8_t LEFT_HA_PIN = 0;
const uint8_t LEFT_HB_PIN = 1;
const uint8_t LEFT_HC_PIN = 7;
const uint8_t RIGHT_HA_PIN = 8;
const uint8_t RIGHT_HB_PIN = 9;
const uint8_t RIGHT_HC_PIN = 10;

const uint8_t TEST_PWM = 8;
const float HALL_TRANSITIONS_PER_REV = 90.0f;
const float WHEEL_RADIUS_M = 0.075f;
const unsigned long TEST_DURATION_MS = 1500UL;
const unsigned long WATCHDOG_MS = 2500UL;
const unsigned long REPORT_INTERVAL_MS = 250UL;

enum TestMode : uint8_t {
  IDLE,
  BOTH_FORWARD,
  LEFT_FORWARD,
  RIGHT_FORWARD,
  BOTH_REVERSE,
  LEFT_REVERSE,
  RIGHT_REVERSE
};

struct StateDecoder {
  uint8_t ring[6];
  uint8_t ringCount;
  uint8_t previous;
  bool initialized;
  int8_t lastDirection;
  long signedCount;
  unsigned long validCount;
  unsigned long invalidCount;

  void begin(uint8_t initialState) {
    ringCount = 1;
    ring[0] = initialState;
    previous = initialState;
    initialized = true;
    lastDirection = 0;
    signedCount = 0;
    validCount = 0;
    invalidCount = 0;
  }

  int indexOf(uint8_t state) {
    for (uint8_t i = 0; i < ringCount; ++i) {
      if (ring[i] == state) return i;
    }
    return -1;
  }

  void update(uint8_t state) {
    if (!initialized) { begin(state); return; }
    if (state == previous) return;

    uint8_t changed = previous ^ state;
    if (changed == 0 || (changed & (changed - 1)) != 0) {
      ++invalidCount;
      previous = state;
      return;
    }

    int oldIdx = indexOf(previous);
    int newIdx = indexOf(state);

    if (ringCount < 6 && oldIdx == (int)ringCount - 1 && newIdx < 0) {
      ring[ringCount++] = state;
      ++validCount;
      if (lastDirection == 0) lastDirection = +1;
      signedCount += lastDirection;
    } else if (oldIdx >= 0 && newIdx >= 0) {
      int forward = (newIdx - oldIdx + 6) % 6;
      int direction = 0;
      if (forward == 1) direction = +1;
      else if (forward == 5) direction = -1;

      if (direction == 0) {
        ++invalidCount;
      } else {
        ++validCount;
        lastDirection = direction;
        signedCount += direction;
      }
    } else {
      ++invalidCount;
      ringCount = 1;
      ring[0] = state;
      lastDirection = 0;
    }
    previous = state;
  }
};

StateDecoder leftDecoder;
StateDecoder rightDecoder;

volatile uint8_t leftState = 0;
volatile uint8_t rightState = 0;

uint8_t readLeftState() {
  return (digitalRead(LEFT_HA_PIN) << 2) |
         (digitalRead(LEFT_HB_PIN) << 1) |
          digitalRead(LEFT_HC_PIN);
}

uint8_t readRightState() {
  return (digitalRead(RIGHT_HA_PIN) << 2) |
         (digitalRead(RIGHT_HB_PIN) << 1) |
          digitalRead(RIGHT_HC_PIN);
}

void leftHallISR() {
  uint8_t state = readLeftState();
  leftDecoder.update(state);
  leftState = state;
}

ISR(PCINT0_vect) {
  uint8_t state = readRightState();
  rightDecoder.update(state);
  rightState = state;
}

void stopMotors() {
  analogWrite(LEFT_PWM_PIN, 0);
  analogWrite(RIGHT_PWM_PIN, 0);
}

void setLeft(int8_t direction, uint8_t pwm) {
  // Match supplied balance sketch: positive = HIGH.
  digitalWrite(LEFT_DIR_PIN, direction >= 0 ? HIGH : LOW);
  analogWrite(LEFT_PWM_PIN, pwm);
}

void setRight(int8_t direction, uint8_t pwm) {
  // Match supplied balance sketch: positive = LOW.
  digitalWrite(RIGHT_DIR_PIN, direction >= 0 ? LOW : HIGH);
  analogWrite(RIGHT_PWM_PIN, pwm);
}

TestMode activeMode = IDLE;
unsigned long testStartMs = 0;
unsigned long lastReportMs = 0;
long startLeftCount = 0;
long startRightCount = 0;
unsigned long startLeftValid = 0;
unsigned long startRightValid = 0;
unsigned long startLeftInvalid = 0;
unsigned long startRightInvalid = 0;

char inputLine[32];
uint8_t inputLength = 0;


const char *modeName(TestMode mode) {
  switch (mode) {
    case BOTH_FORWARD: return "BOTH_FWD";
    case LEFT_FORWARD: return "LEFT_FWD";
    case RIGHT_FORWARD: return "RIGHT_FWD";
    case BOTH_REVERSE: return "BOTH_REV";
    case LEFT_REVERSE: return "LEFT_REV";
    case RIGHT_REVERSE: return "RIGHT_REV";
    default: return "IDLE";
  }
}

void snapshotCounts(long &lc, long &rc,
                    unsigned long &lv, unsigned long &rv,
                    unsigned long &li, unsigned long &ri) {
  noInterrupts();
  lc = leftDecoder.signedCount;
  rc = rightDecoder.signedCount;
  lv = leftDecoder.validCount;
  rv = rightDecoder.validCount;
  li = leftDecoder.invalidCount;
  ri = rightDecoder.invalidCount;
  interrupts();
}

void printStatus(const char *tag) {
  long lc, rc;
  unsigned long lv, rv, li, ri;
  snapshotCounts(lc, rc, lv, rv, li, ri);
  Serial.print(tag);
  Serial.print(',');
  Serial.print(millis());
  Serial.print(',');
  Serial.print(lc);
  Serial.print(',');
  Serial.print(rc);
  Serial.print(',');
  Serial.print(lv);
  Serial.print(',');
  Serial.print(rv);
  Serial.print(',');
  Serial.print(li);
  Serial.print(',');
  Serial.println(ri);
}

void finishTest(const char *reason) {
  stopMotors();

  long lc, rc;
  unsigned long lv, rv, li, ri;
  snapshotCounts(lc, rc, lv, rv, li, ri);

  Serial.print(F("RESULT,"));
  Serial.print(modeName(activeMode));
  Serial.print(',');
  Serial.print(reason);
  Serial.print(F(",pwm="));
  Serial.print(TEST_PWM);
  Serial.print(F(",left_delta="));
  Serial.print(lc - startLeftCount);
  Serial.print(F(",right_delta="));
  Serial.print(rc - startRightCount);
  Serial.print(F(",left_valid_delta="));
  Serial.print(lv - startLeftValid);
  Serial.print(F(",right_valid_delta="));
  Serial.print(rv - startRightValid);
  Serial.print(F(",left_invalid_delta="));
  Serial.print(li - startLeftInvalid);
  Serial.print(F(",right_invalid_delta="));
  Serial.print(ri - startRightInvalid);

  float elapsedSeconds = (millis() - testStartMs) / 1000.0f;
  if (elapsedSeconds < 0.001f) elapsedSeconds = 0.001f;
  long leftDelta = lc - startLeftCount;
  long rightDelta = rc - startRightCount;
  float leftRpm = ((float)leftDelta / HALL_TRANSITIONS_PER_REV) * (60.0f / elapsedSeconds);
  float rightRpm = ((float)rightDelta / HALL_TRANSITIONS_PER_REV) * (60.0f / elapsedSeconds);
  float wheelCircumference = 2.0f * PI * WHEEL_RADIUS_M;
  float leftVelocity = leftRpm * wheelCircumference / 60.0f;
  float rightVelocity = rightRpm * wheelCircumference / 60.0f;

  Serial.print(F(",elapsed_s="));
  Serial.print(elapsedSeconds, 3);
  Serial.print(F(",left_rpm="));
  Serial.print(leftRpm, 2);
  Serial.print(F(",right_rpm="));
  Serial.print(rightRpm, 2);
  Serial.print(F(",left_mps="));
  Serial.print(leftVelocity, 4);
  Serial.print(F(",right_mps="));
  Serial.println(rightVelocity, 4);

  activeMode = IDLE;
  printStatus("STATUS");
}

void beginTest(TestMode mode) {
  if (activeMode != IDLE) {
    Serial.println(F("BUSY, send STOP first"));
    return;
  }

  long lc, rc;
  unsigned long lv, rv, li, ri;
  snapshotCounts(lc, rc, lv, rv, li, ri);
  startLeftCount = lc;
  startRightCount = rc;
  startLeftValid = lv;
  startRightValid = rv;
  startLeftInvalid = li;
  startRightInvalid = ri;

  activeMode = mode;
  testStartMs = millis();
  lastReportMs = testStartMs;

  stopMotors();

  if (mode == BOTH_FORWARD || mode == LEFT_FORWARD)
    setLeft(+1, TEST_PWM);
  else if (mode == BOTH_REVERSE || mode == LEFT_REVERSE)
    setLeft(-1, TEST_PWM);

  if (mode == BOTH_FORWARD || mode == RIGHT_FORWARD)
    setRight(+1, TEST_PWM);
  else if (mode == BOTH_REVERSE || mode == RIGHT_REVERSE)
    setRight(-1, TEST_PWM);

  Serial.print(F("START,"));
  Serial.print(modeName(mode));
  Serial.print(F(",pwm="));
  Serial.print(TEST_PWM);
  Serial.print(F(",duration_ms="));
  Serial.println(TEST_DURATION_MS);
}

void handleCommand(char *cmd) {
  if (!strcmp(cmd, "STOP")) {
    stopMotors();
    if (activeMode != IDLE) finishTest("MANUAL_STOP");
    else Serial.println(F("STOPPED"));
    return;
  }

  if (!strcmp(cmd, "STATUS")) {
    printStatus("STATUS");
    return;
  }

  if (!strcmp(cmd, "BOTH_FWD")) { beginTest(BOTH_FORWARD); return; }
  if (!strcmp(cmd, "LEFT_FWD")) { beginTest(LEFT_FORWARD); return; }
  if (!strcmp(cmd, "RIGHT_FWD")) { beginTest(RIGHT_FORWARD); return; }
  if (!strcmp(cmd, "BOTH_REV")) { beginTest(BOTH_REVERSE); return; }
  if (!strcmp(cmd, "LEFT_REV")) { beginTest(LEFT_REVERSE); return; }
  if (!strcmp(cmd, "RIGHT_REV")) { beginTest(RIGHT_REVERSE); return; }

Serial.print(F("Unknown command received: ["));
Serial.print(cmd);
Serial.println(F("]"));

Serial.print(F("Command length: "));
Serial.println(strlen(cmd));
}

void readSerialCommands() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      inputLine[inputLength] = '\0';
      if (inputLength > 0) handleCommand(inputLine);
      inputLength = 0;
    } else if (inputLength < sizeof(inputLine) - 1) {
      inputLine[inputLength++] = c;
    } else {
      inputLength = 0;
      Serial.println(F("ERR,line too long"));
    }
  }
}

void setup() {
  Serial.begin(115200);

unsigned long serialStart = millis();
while (!Serial && millis() - serialStart < 5000) {
  // Wait up to 5 seconds for the Leonardo USB serial connection.
}

  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);

  pinMode(LEFT_HA_PIN, INPUT);
  pinMode(LEFT_HB_PIN, INPUT);
  pinMode(LEFT_HC_PIN, INPUT);
  pinMode(RIGHT_HA_PIN, INPUT);
  pinMode(RIGHT_HB_PIN, INPUT);
  pinMode(RIGHT_HC_PIN, INPUT);

  stopMotors();

  delay(50);
  uint8_t ls = readLeftState();
  uint8_t rs = readRightState();
  leftDecoder.begin(ls);
  rightDecoder.begin(rs);
  leftState = ls;
  rightState = rs;

  attachInterrupt(digitalPinToInterrupt(LEFT_HA_PIN), leftHallISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_HB_PIN), leftHallISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_HC_PIN), leftHallISR, CHANGE);

  // Leonardo D8/D9/D10 = PB4/PB5/PB6.
  PCIFR |= _BV(PCIF0);
  PCMSK0 |= _BV(PCINT4) | _BV(PCINT5) | _BV(PCINT6);
  PCICR |= _BV(PCIE0);

  Serial.println(F("R2_HALL_DIRECTION_TEST_V1"));
  Serial.println(F("Balance controller and IMU are disabled."));
  Serial.println(F("Secure robot raised; keep clear of wheels."));
  Serial.println(F("Run BOTH_FWD first; observe both wheel directions."));
  Serial.println(F("Each test uses PWM 8 for 1500 ms, then auto-stops."));
  Serial.println(F("CSV: type,time_ms,left_signed,right_signed,left_valid,right_valid,left_invalid,right_invalid"));
  Serial.println(F("Commands: BOTH_FWD, LEFT_FWD, RIGHT_FWD, BOTH_REV, LEFT_REV, RIGHT_REV, STOP, STATUS"));
}

void loop() {
  readSerialCommands();

  if (activeMode != IDLE) {
    unsigned long now = millis();

    if (now - testStartMs >= TEST_DURATION_MS) {
      finishTest("AUTO_STOP");
    } else if (now - testStartMs >= WATCHDOG_MS) {
      finishTest("WATCHDOG");
    } else if (now - lastReportMs >= REPORT_INTERVAL_MS) {
      lastReportMs = now;
      printStatus("SAMPLE");
    }
  }
}
