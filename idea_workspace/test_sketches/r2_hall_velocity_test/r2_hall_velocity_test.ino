/*
  R2 Hall velocity estimator validation
  Leonardo / ATmega32U4

  Current wiring:
    Left  Hall A/B/C = D0, D1, D7
    Right Hall A/B/C = D8, D9, D10
    Left SC  = D18
    Right SC = D19
    IMU I2C = D2/D3
    Right direction = D4
    Left PWM = D5
    Right PWM = D6
    Left direction = D12

  This sketch does NOT balance the robot.
  Hall inputs are decoded as a six-state sequence. The A/B/C names are
  provisional; the decoder learns the observed sequence and determines
  signed direction relative to the first observed direction. If a wheel is
  manually turned in the opposite direction, its signed count should reverse.

  Important: left/right sign conventions may need flipping after validation
  so that positive means robot-forward for both wheels.
*/

#include <Arduino.h>

const uint8_t LEFT_HA_PIN  = 0;
const uint8_t LEFT_HB_PIN  = 1;
const uint8_t LEFT_HC_PIN  = 7;

const uint8_t RIGHT_HA_PIN = 8;
const uint8_t RIGHT_HB_PIN = 9;
const uint8_t RIGHT_HC_PIN = 10;

const uint8_t LEFT_SC_PIN  = 18;
const uint8_t RIGHT_SC_PIN = 19;

const uint8_t RIGHT_DIR_PIN = 4;
const uint8_t LEFT_PWM_PIN  = 5;
const uint8_t RIGHT_PWM_PIN = 6;
const uint8_t LEFT_DIR_PIN  = 12;

const int MAX_PWM = 51;
const unsigned long SERIAL_TIMEOUT_MS = 2000;
const unsigned long TELEMETRY_MS = 100;
// Use reciprocal transition timing at low speed; use interval counts at higher speed.
const unsigned long TRANSITION_STALE_US = 350000UL; // declare stopped after 350 ms without an edge

// Calibration from 10 manual wheel revolutions on each wheel.
const float HALL_TRANSITIONS_PER_REV = 90.0f;
const float WHEEL_RADIUS_M = 0.075f;
// Time-constant-based low-pass filter. At 100 ms, tau=0.30 s is
// approximately equivalent to the previous alpha=0.25 filter.
const float RPM_FILTER_TAU_SECONDS = 0.30f;

volatile uint8_t leftState = 0;
volatile uint8_t rightState = 0;
volatile long leftSignedTransitions = 0;
volatile long rightSignedTransitions = 0;
volatile unsigned long leftValidTransitions = 0;
volatile unsigned long rightValidTransitions = 0;
volatile unsigned long leftInvalidTransitions = 0;
volatile unsigned long rightInvalidTransitions = 0;
volatile unsigned long leftLastTransitionUs = 0;
volatile unsigned long rightLastTransitionUs = 0;
volatile unsigned long leftPeriodUs = 0;
volatile unsigned long rightPeriodUs = 0;
volatile int8_t leftLastDirection = 0;
volatile int8_t rightLastDirection = 0;

// SC transitions are counted with pin-change/external interrupts where possible.
// D18/D19 do not support attachInterrupt on Leonardo, so sample level in loop.
uint8_t leftSCPrevious = LOW;
uint8_t rightSCPrevious = LOW;
unsigned long leftSCTransitions = 0;
unsigned long rightSCTransitions = 0;

int targetLeftPWM = 0;
int targetRightPWM = 0;
int currentLeftPWM = 0;
int currentRightPWM = 0;
// Last percentages received from the host (before mapping to controller PWM).
int requestedLeftPercent = 0;
int requestedRightPercent = 0;
unsigned long lastCommandTime = 0;
unsigned long lastTelemetryTime = 0;
long previousLeftCount = 0;
long previousRightCount = 0;
unsigned long previousVelocitySampleMs = 0;
float leftRpmRaw = 0.0f;
float rightRpmRaw = 0.0f;
float leftRpmFiltered = 0.0f;
float rightRpmFiltered = 0.0f;
float leftVelocityRaw = 0.0f;
float rightVelocityRaw = 0.0f;
float leftVelocityFiltered = 0.0f;
float rightVelocityFiltered = 0.0f;
float leftRpmPeriod = 0.0f;
float rightRpmPeriod = 0.0f;
float leftRpmHybrid = 0.0f;
float rightRpmHybrid = 0.0f;
uint8_t leftVelocityMode = 0;  // 0=stopped/stale, 1=period, 2=count-window
uint8_t rightVelocityMode = 0;
unsigned long leftTransitionAgeMs = 0;
unsigned long rightTransitionAgeMs = 0;

char commandLine[64];
uint8_t commandUsed = 0;

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

// Return 0 for no state change, +1/-1 for one adjacent step in the observed
// six-state ring, and 2 for an invalid/skipped transition. The actual ring
// order is discovered from observed one-bit state changes.
struct StateDecoder {
  uint8_t ring[6];
  uint8_t ringCount;
  uint8_t previous;
  bool initialized;
  int8_t lastDirection;
  long signedCount;
  unsigned long validCount;
  unsigned long invalidCount;
  unsigned long lastUs;
  unsigned long periodUs;

  void begin(uint8_t initialState) {
    ringCount = 1;
    ring[0] = initialState;
    previous = initialState;
    initialized = true;
    lastDirection = 0;
    signedCount = 0;
    validCount = 0;
    invalidCount = 0;
    lastUs = 0;
    periodUs = 0;
  }

  int indexOf(uint8_t state) {
    for (uint8_t i = 0; i < ringCount; ++i) {
      if (ring[i] == state) return i;
    }
    return -1;
  }

  // State change is processed in chronological order.
  void update(uint8_t state, unsigned long nowUs) {
    if (!initialized) {
      begin(state);
      return;
    }
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
      // Learn the next adjacent state in the currently observed direction.
      ring[ringCount++] = state;
      ++validCount;
      if (lastDirection == 0) lastDirection = +1;
      signedCount += lastDirection;
      if (lastUs != 0) periodUs = nowUs - lastUs;
      lastUs = nowUs;
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
        if (lastUs != 0) periodUs = nowUs - lastUs;
        lastUs = nowUs;
      }
    } else {
      // A previously unseen state appeared out of the learned order. Reset
      // learning around the new observed state instead of inventing direction.
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

void applyLeftMotor(int pwm) {
  pwm = constrain(pwm, -MAX_PWM, MAX_PWM);
  digitalWrite(LEFT_DIR_PIN, pwm >= 0 ? HIGH : LOW);
  analogWrite(LEFT_PWM_PIN, abs(pwm));
}
void applyRightMotor(int pwm) {
  pwm = constrain(pwm, -MAX_PWM, MAX_PWM);
  digitalWrite(RIGHT_DIR_PIN, pwm >= 0 ? LOW : HIGH);
  analogWrite(RIGHT_PWM_PIN, abs(pwm));
}

void updateMotors() {
  unsigned long now = millis();
  if (now - lastCommandTime > SERIAL_TIMEOUT_MS) {
    targetLeftPWM = 0;
    targetRightPWM = 0;
  }

  if (currentLeftPWM < targetLeftPWM) currentLeftPWM++;
  else if (currentLeftPWM > targetLeftPWM) currentLeftPWM--;

  if (currentRightPWM < targetRightPWM) currentRightPWM++;
  else if (currentRightPWM > targetRightPWM) currentRightPWM--;

  applyLeftMotor(currentLeftPWM);
  applyRightMotor(currentRightPWM);
}

void handleCommand(char *line) {
  if (strcmp(line, "STOP") == 0 || strcmp(line, "stop") == 0) {
    targetLeftPWM = 0;
    targetRightPWM = 0;
    requestedLeftPercent = 0;
    requestedRightPercent = 0;
    lastCommandTime = millis();
    Serial.println(F("OK STOP"));
    return;
  }
  if (line[0] == 'M' || line[0] == 'm') {
    int left, right;
    if (sscanf(line + 1, "%d %d", &left, &right) == 2) {
      left = constrain(left, -100, 100);
      right = constrain(right, -100, 100);
      requestedLeftPercent = left;
      requestedRightPercent = right;
      targetLeftPWM = map(abs(left), 0, 100, 0, MAX_PWM);
      targetRightPWM = map(abs(right), 0, 100, 0, MAX_PWM);
      if (left < 0) targetLeftPWM = -targetLeftPWM;
      if (right < 0) targetRightPWM = -targetRightPWM;
      lastCommandTime = millis();
      Serial.print(F("OK M "));
      Serial.print(left);
      Serial.print(' ');
      Serial.println(right);
      return;
    }
  }
  Serial.println(F("ERR command; use M <left -100..100> <right -100..100> or STOP"));
}

void processSerialCommands() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      commandLine[commandUsed] = '\0';
      handleCommand(commandLine);
      commandUsed = 0;
    } else if (commandUsed < sizeof(commandLine) - 1) {
      commandLine[commandUsed++] = c;
    } else {
      commandUsed = 0;
      Serial.println(F("ERR: command too long"));
    }
  }
}

void setup() {
  Serial.begin(115200);

  // Explicitly configure every Hall and SC input.
  pinMode(LEFT_HA_PIN, INPUT);
  pinMode(LEFT_HB_PIN, INPUT);
  pinMode(LEFT_HC_PIN, INPUT);
  pinMode(RIGHT_HA_PIN, INPUT);
  pinMode(RIGHT_HB_PIN, INPUT);
  pinMode(RIGHT_HC_PIN, INPUT);
  pinMode(LEFT_SC_PIN, INPUT);
  pinMode(RIGHT_SC_PIN, INPUT);

  pinMode(RIGHT_DIR_PIN, OUTPUT);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);

  analogWrite(LEFT_PWM_PIN, 0);
  analogWrite(RIGHT_PWM_PIN, 0);
  digitalWrite(LEFT_DIR_PIN, LOW);
  digitalWrite(RIGHT_DIR_PIN, LOW);

  delay(300);
  uint8_t ls = readLeftState();
  uint8_t rs = readRightState();
  leftDecoder.begin(ls);
  rightDecoder.begin(rs);
  leftState = ls;
  rightState = rs;
  leftSCPrevious = digitalRead(LEFT_SC_PIN);
  rightSCPrevious = digitalRead(RIGHT_SC_PIN);
  lastCommandTime = millis();
  previousVelocitySampleMs = millis();
  previousLeftCount = leftSignedTransitions;
  previousRightCount = rightSignedTransitions;

  // Attach interrupts for left Hall signals on D0, D1, D7.
  attachInterrupt(digitalPinToInterrupt(LEFT_HA_PIN), leftHallISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_HB_PIN), leftHallISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_HC_PIN), leftHallISR, CHANGE);

  // Right Hall pins D8/D9/D10 share Port B. PCINT0_vect is not supported by
  // Arduino's generic attachInterrupt; use the port-change ISR below.
  PCICR |= _BV(PCIE0);
  PCMSK0 |= _BV(PCINT4) | _BV(PCINT5) | _BV(PCINT6);
  rightState = readRightState();

  Serial.println(F("R2 Hall velocity estimator validation ready; no balance controller."));
  Serial.println(F("Commands: M <left> <right> (-100..100), STOP"));
  Serial.println(F("Telemetry: 36 fields; period-preferred hybrid RPM/velocity estimates"));
}

void leftHallISR() {
  uint8_t state = readLeftState();
  leftDecoder.update(state, micros());
  leftState = state;
  leftSignedTransitions = leftDecoder.signedCount;
  leftValidTransitions = leftDecoder.validCount;
  leftInvalidTransitions = leftDecoder.invalidCount;
  leftPeriodUs = leftDecoder.periodUs;
  leftLastTransitionUs = leftDecoder.lastUs;
  leftLastDirection = leftDecoder.lastDirection;
}

ISR(PCINT0_vect) {
  uint8_t state = readRightState();
  rightDecoder.update(state, micros());
  rightState = state;
  rightSignedTransitions = rightDecoder.signedCount;
  rightValidTransitions = rightDecoder.validCount;
  rightInvalidTransitions = rightDecoder.invalidCount;
  rightPeriodUs = rightDecoder.periodUs;
  rightLastTransitionUs = rightDecoder.lastUs;
  rightLastDirection = rightDecoder.lastDirection;
}

void loop() {
  processSerialCommands();
  updateMotors();

  unsigned long nowMs = millis();
  if (nowMs - lastTelemetryTime >= TELEMETRY_MS) {
    lastTelemetryTime = nowMs;

    uint8_t ls = readLeftState();
    uint8_t rs = readRightState();

    // Sample SC levels. This is only a comparison counter; D18/D19 cannot use
    // attachInterrupt on a Leonardo, so fast transitions may be missed.
    uint8_t lsc = digitalRead(LEFT_SC_PIN);
    uint8_t rsc = digitalRead(RIGHT_SC_PIN);
    if (lsc != leftSCPrevious) ++leftSCTransitions;
    if (rsc != rightSCPrevious) ++rightSCTransitions;
    leftSCPrevious = lsc;
    rightSCPrevious = rsc;

    noInterrupts();
    long lc = leftSignedTransitions;
    long rc = rightSignedTransitions;
    unsigned long lv = leftValidTransitions;
    unsigned long rv = rightValidTransitions;
    unsigned long li = leftInvalidTransitions;
    unsigned long ri = rightInvalidTransitions;
    unsigned long lp = leftPeriodUs;
    unsigned long rp = rightPeriodUs;
    interrupts();

    // Hybrid velocity estimator (telemetry only):
    // 1) Prefer reciprocal transition-period RPM whenever the latest edge is
    //    fresh and a valid interval between two edges is available.
    // 2) Fall back to count-window RPM only when no valid period is available
    //    and at least one transition occurred in this sample.
    // 3) Report zero when the last edge is stale or no usable estimate exists.
    // This avoids switching to the noisier count estimate merely because the
    // wheel crossed an arbitrary transitions-per-window threshold.
    unsigned long sampleDtMs = nowMs - previousVelocitySampleMs;
    unsigned long nowUs = micros();
    unsigned long leftLastUs, rightLastUs, leftPeriod, rightPeriod;
    int8_t leftDir, rightDir;
    noInterrupts();
    leftLastUs = leftLastTransitionUs;
    rightLastUs = rightLastTransitionUs;
    leftPeriod = leftPeriodUs;
    rightPeriod = rightPeriodUs;
    leftDir = leftLastDirection;
    rightDir = rightLastDirection;
    interrupts();

    long deltaLeft = lc - previousLeftCount;
    long deltaRight = rc - previousRightCount;
    leftTransitionAgeMs = (leftLastUs == 0) ? 0xFFFFFFFFUL : (nowUs - leftLastUs) / 1000UL;
    rightTransitionAgeMs = (rightLastUs == 0) ? 0xFFFFFFFFUL : (nowUs - rightLastUs) / 1000UL;

    if (sampleDtMs > 0) {
      const float dtSeconds = sampleDtMs / 1000.0f;
      leftRpmRaw = ((float)deltaLeft / HALL_TRANSITIONS_PER_REV) * (60.0f / dtSeconds);
      rightRpmRaw = ((float)deltaRight / HALL_TRANSITIONS_PER_REV) * (60.0f / dtSeconds);

      const bool leftFresh = (leftLastUs != 0 && (nowUs - leftLastUs) <= TRANSITION_STALE_US);
      const bool rightFresh = (rightLastUs != 0 && (nowUs - rightLastUs) <= TRANSITION_STALE_US);

      // Diagnostic period estimates are available whenever a valid period exists.
      leftRpmPeriod = (leftPeriod > 0 && leftDir != 0)
          ? (float)leftDir * 60000000.0f / (HALL_TRANSITIONS_PER_REV * (float)leftPeriod) : 0.0f;
      rightRpmPeriod = (rightPeriod > 0 && rightDir != 0)
          ? (float)rightDir * 60000000.0f / (HALL_TRANSITIONS_PER_REV * (float)rightPeriod) : 0.0f;

      // Prefer period timing. Count-window is strictly a fallback if the
      // period has not yet been established (e.g. the first edge after start).
      if (!leftFresh) {
        leftRpmHybrid = 0.0f;
        leftVelocityMode = 0;
      } else if (leftPeriod > 0 && leftDir != 0) {
        leftRpmHybrid = leftRpmPeriod;
        leftVelocityMode = 1;
      } else if (deltaLeft != 0) {
        leftRpmHybrid = leftRpmRaw;
        leftVelocityMode = 2;
      } else {
        leftRpmHybrid = 0.0f;
        leftVelocityMode = 0;
      }

      if (!rightFresh) {
        rightRpmHybrid = 0.0f;
        rightVelocityMode = 0;
      } else if (rightPeriod > 0 && rightDir != 0) {
        rightRpmHybrid = rightRpmPeriod;
        rightVelocityMode = 1;
      } else if (deltaRight != 0) {
        rightRpmHybrid = rightRpmRaw;
        rightVelocityMode = 2;
      } else {
        rightRpmHybrid = 0.0f;
        rightVelocityMode = 0;
      }

      // Time-based first-order low-pass: alpha = dt / (tau + dt).
      // This keeps the filter's time response consistent if telemetry cadence changes.
      const float alpha = dtSeconds / (RPM_FILTER_TAU_SECONDS + dtSeconds);
      leftRpmFiltered += alpha * (leftRpmHybrid - leftRpmFiltered);
      rightRpmFiltered += alpha * (rightRpmHybrid - rightRpmFiltered);
      const float circumference = 2.0f * PI * WHEEL_RADIUS_M;
      leftVelocityRaw = leftRpmHybrid * circumference / 60.0f;
      rightVelocityRaw = rightRpmHybrid * circumference / 60.0f;
      leftVelocityFiltered = leftRpmFiltered * circumference / 60.0f;
      rightVelocityFiltered = rightRpmFiltered * circumference / 60.0f;
      previousLeftCount = lc;
      previousRightCount = rc;
      previousVelocitySampleMs = nowMs;
    }

    Serial.print(nowMs); Serial.print(',');
    Serial.print(ls); Serial.print(',');
    Serial.print(rs); Serial.print(',');
    Serial.print(lc); Serial.print(',');
    Serial.print(rc); Serial.print(',');
    Serial.print(lv); Serial.print(',');
    Serial.print(rv); Serial.print(',');
    Serial.print(li); Serial.print(',');
    Serial.print(ri); Serial.print(',');
    Serial.print(lp); Serial.print(',');
    Serial.print(rp); Serial.print(',');
    Serial.print(leftSCTransitions); Serial.print(',');
    Serial.print(rightSCTransitions); Serial.print(',');
    Serial.print(requestedLeftPercent); Serial.print(',');
    Serial.print(requestedRightPercent); Serial.print(',');
    Serial.print(targetLeftPWM); Serial.print(',');
    Serial.print(targetRightPWM); Serial.print(',');
    Serial.print(currentLeftPWM); Serial.print(',');
    Serial.print(currentRightPWM); Serial.print(',');
    Serial.print((millis() - lastCommandTime > SERIAL_TIMEOUT_MS) ? 1 : 0); Serial.print(',');
    Serial.print(leftRpmRaw, 3); Serial.print(',');
    Serial.print(rightRpmRaw, 3); Serial.print(',');
    Serial.print(leftRpmPeriod, 3); Serial.print(',');
    Serial.print(rightRpmPeriod, 3); Serial.print(',');
    Serial.print(leftRpmHybrid, 3); Serial.print(',');
    Serial.print(rightRpmHybrid, 3); Serial.print(',');
    Serial.print(leftRpmFiltered, 3); Serial.print(',');
    Serial.print(rightRpmFiltered, 3); Serial.print(',');
    Serial.print(leftVelocityRaw, 4); Serial.print(',');
    Serial.print(rightVelocityRaw, 4); Serial.print(',');
    Serial.print(leftVelocityFiltered, 4); Serial.print(',');
    Serial.print(rightVelocityFiltered, 4); Serial.print(',');
    Serial.print(leftVelocityMode); Serial.print(',');
    Serial.print(rightVelocityMode); Serial.print(',');
    Serial.print(leftTransitionAgeMs); Serial.print(',');
    Serial.println(rightTransitionAgeMs);
  }
}
