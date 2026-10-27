/*
  R2 Hall decoder validation
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

  // Attach interrupts for left Hall signals on D0, D1, D7.
  attachInterrupt(digitalPinToInterrupt(LEFT_HA_PIN), leftHallISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_HB_PIN), leftHallISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_HC_PIN), leftHallISR, CHANGE);

  // Right Hall pins D8/D9/D10 share Port B. PCINT0_vect is not supported by
  // Arduino's generic attachInterrupt; use the port-change ISR below.
  PCICR |= _BV(PCIE0);
  PCMSK0 |= _BV(PCINT4) | _BV(PCINT5) | _BV(PCINT6);
  rightState = readRightState();

  Serial.println(F("R2 Hall decoder validation ready; no balance controller."));
  Serial.println(F("Commands: M <left> <right> (-100..100), STOP"));
  Serial.println(F("Telemetry fields: ms,Lstate,Rstate,Lsigned,Rsigned,Lvalid,Rvalid,Linvalid,Rinvalid,LperiodUs,RperiodUs,Lsc,Rsc,requestedL_pct,requestedR_pct,targetL_pwm,targetR_pwm,appliedL_pwm,appliedR_pwm,command_timed_out"));
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
}

ISR(PCINT0_vect) {
  uint8_t state = readRightState();
  rightDecoder.update(state, micros());
  rightState = state;
  rightSignedTransitions = rightDecoder.signedCount;
  rightValidTransitions = rightDecoder.validCount;
  rightInvalidTransitions = rightDecoder.invalidCount;
  rightPeriodUs = rightDecoder.periodUs;
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
    Serial.println((millis() - lastCommandTime > SERIAL_TIMEOUT_MS) ? 1 : 0);
  }
}
