/*
  R2 motor breakaway / low-PWM characterization test
  Board: Arduino Leonardo / ATmega32U4

  PURPOSE:
    Characterize each motor independently with the chassis securely raised.
    This sketch does NOT run the IMU or balance controller.

  Wiring copied from R2 V4:
    Left PWM D5, Left DIR D12 (positive = HIGH)
    Right PWM D6, Right DIR D4 (positive = LOW)
    Left Hall A/B/C D0/D1/D7
    Right Hall A/B/C D8/D9/D10

  Serial protocol at 115200 baud:
    L <signed_pwm>   e.g. L 8 or L -10
    R <signed_pwm>   e.g. R 8 or R -10
    STOP             stop both motors
    STATUS           print current commands and Hall counts

  Safety limits:
    - Only one motor may be commanded at a time.
    - Absolute PWM limited to 30.
    - A direction reversal requires STOP (zero) first.
    - 2-second command watchdog stops motors if no command is received.
    - Type STOP at any time for immediate stop.

  Hall counts are raw 3-bit state transitions, not decoded direction counts.
*/
#include <Arduino.h>

const uint8_t LEFT_PWM_PIN = 5;
const uint8_t RIGHT_PWM_PIN = 6;
const uint8_t LEFT_DIR_PIN = 12;
const uint8_t RIGHT_DIR_PIN = 4;
const uint8_t LEFT_HA_PIN = 0, LEFT_HB_PIN = 1, LEFT_HC_PIN = 7;
const uint8_t RIGHT_HA_PIN = 8, RIGHT_HB_PIN = 9, RIGHT_HC_PIN = 10;
const int MAX_TEST_PWM = 30;
const unsigned long WATCHDOG_MS = 2000;
const unsigned long TELEMETRY_MS = 100;

volatile uint32_t leftHallEdges = 0;
volatile uint32_t rightHallEdges = 0;
volatile uint8_t lastLeftState = 0;
volatile uint8_t lastRightState = 0;
int leftCommand = 0;
int rightCommand = 0;
int lastLeftDirection = 0;
int lastRightDirection = 0;
unsigned long lastCommandMs = 0;
unsigned long lastTelemetryMs = 0;
char inputLine[32];
uint8_t inputPos = 0;

uint8_t readLeftState() {
  return (digitalRead(LEFT_HA_PIN) << 2) |
         (digitalRead(LEFT_HB_PIN) << 1) | digitalRead(LEFT_HC_PIN);
}
uint8_t readRightState() {
  return (digitalRead(RIGHT_HA_PIN) << 2) |
         (digitalRead(RIGHT_HB_PIN) << 1) | digitalRead(RIGHT_HC_PIN);
}
void leftHallISR() {
  uint8_t s = readLeftState();
  if (s != lastLeftState) { ++leftHallEdges; lastLeftState = s; }
}
ISR(PCINT0_vect) {
  uint8_t s = readRightState();
  if (s != lastRightState) { ++rightHallEdges; lastRightState = s; }
}

void stopMotors() {
  analogWrite(LEFT_PWM_PIN, 0);
  analogWrite(RIGHT_PWM_PIN, 0);
  leftCommand = 0;
  rightCommand = 0;
}
void setLeft(int cmd) {
  cmd = constrain(cmd, -MAX_TEST_PWM, MAX_TEST_PWM);
  if (cmd > 0) digitalWrite(LEFT_DIR_PIN, HIGH);
  else if (cmd < 0) digitalWrite(LEFT_DIR_PIN, LOW);
  analogWrite(LEFT_PWM_PIN, abs(cmd));
  leftCommand = cmd;
}
void setRight(int cmd) {
  cmd = constrain(cmd, -MAX_TEST_PWM, MAX_TEST_PWM);
  if (cmd > 0) digitalWrite(RIGHT_DIR_PIN, LOW);
  else if (cmd < 0) digitalWrite(RIGHT_DIR_PIN, HIGH);
  analogWrite(RIGHT_PWM_PIN, abs(cmd));
  rightCommand = cmd;
}
void reportStatus() {
  uint32_t l, r;
  noInterrupts(); l = leftHallEdges; r = rightHallEdges; interrupts();
  Serial.print(F("S,")); Serial.print(leftCommand); Serial.print(',');
  Serial.print(rightCommand); Serial.print(','); Serial.print(l); Serial.print(','); Serial.println(r);
}
void processCommand(char *line) {
  while (*line == ' ' || *line == '\t') ++line;
  if (strcasecmp(line, "STOP") == 0) {
    stopMotors(); lastLeftDirection = 0; lastRightDirection = 0;
    lastCommandMs = millis(); Serial.println(F("OK,STOP")); reportStatus(); return;
  }
  if (strcasecmp(line, "STATUS") == 0) { reportStatus(); return; }
  char motor = 0; int value = 0; char extra = 0;
  if (sscanf(line, " %c %d %c", &motor, &value, &extra) != 2) {
    Serial.println(F("ERR,Use L <pwm>, R <pwm>, STOP, or STATUS")); return;
  }
  motor = toupper(motor);
  if (motor != 'L' && motor != 'R') { Serial.println(F("ERR,Motor must be L or R")); return; }
  if (value < -MAX_TEST_PWM || value > MAX_TEST_PWM) {
    Serial.println(F("ERR,PWM outside +/-30 limit")); return;
  }
  if (value != 0 && motor == 'L' && rightCommand != 0) {
    Serial.println(F("ERR,STOP right motor before testing left")); return;
  }
  if (value != 0 && motor == 'R' && leftCommand != 0) {
    Serial.println(F("ERR,STOP left motor before testing right")); return;
  }
  int dir = (value > 0) ? 1 : ((value < 0) ? -1 : 0);
  if (motor == 'L' && dir && lastLeftDirection && dir != lastLeftDirection) {
    Serial.println(F("ERR,Send STOP before reversing left motor")); return;
  }
  if (motor == 'R' && dir && lastRightDirection && dir != lastRightDirection) {
    Serial.println(F("ERR,Send STOP before reversing right motor")); return;
  }
  if (motor == 'L') { setLeft(value); if (dir) lastLeftDirection = dir; }
  else { setRight(value); if (dir) lastRightDirection = dir; }
  lastCommandMs = millis();
  Serial.print(F("OK,")); Serial.print(motor); Serial.print(','); Serial.println(value);
}
void setup() {
  Serial.begin(115200);
  pinMode(LEFT_PWM_PIN, OUTPUT); pinMode(RIGHT_PWM_PIN, OUTPUT);
  pinMode(LEFT_DIR_PIN, OUTPUT); pinMode(RIGHT_DIR_PIN, OUTPUT);
  pinMode(LEFT_HA_PIN, INPUT); pinMode(LEFT_HB_PIN, INPUT); pinMode(LEFT_HC_PIN, INPUT);
  pinMode(RIGHT_HA_PIN, INPUT); pinMode(RIGHT_HB_PIN, INPUT); pinMode(RIGHT_HC_PIN, INPUT);
  stopMotors();
  delay(20);
  lastLeftState = readLeftState(); lastRightState = readRightState();
  attachInterrupt(digitalPinToInterrupt(LEFT_HA_PIN), leftHallISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_HB_PIN), leftHallISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_HC_PIN), leftHallISR, CHANGE);
  PCIFR |= _BV(PCIF0);
  PCMSK0 |= _BV(PCINT4) | _BV(PCINT5) | _BV(PCINT6);
  PCICR |= _BV(PCIE0);
  lastCommandMs = millis(); lastTelemetryMs = millis();
  Serial.println(F("R2_MOTOR_BREAKAWAY_V1"));
  Serial.println(F("Commands: L <+/-0..30>, R <+/-0..30>, STOP, STATUS"));
  Serial.println(F("Only one motor at a time; STOP before reversing; 2s watchdog."));
  Serial.println(F("T,time_ms,left_cmd,right_cmd,left_edges,right_edges"));
}
void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r' || c == '\n') {
      if (inputPos) { inputLine[inputPos] = '\0'; processCommand(inputLine); inputPos = 0; }
    } else if (inputPos < sizeof(inputLine) - 1) inputLine[inputPos++] = c;
    else { inputPos = 0; Serial.println(F("ERR,command too long")); }
  }
  if ((leftCommand != 0 || rightCommand != 0) && millis() - lastCommandMs > WATCHDOG_MS) {
    stopMotors(); lastLeftDirection = 0; lastRightDirection = 0;
    Serial.println(F("WATCHDOG,STOPPED"));
  }
  if (millis() - lastTelemetryMs >= TELEMETRY_MS) {
    lastTelemetryMs = millis();
    uint32_t l, r;
    noInterrupts(); l = leftHallEdges; r = rightHallEdges; interrupts();
    Serial.print(F("T,")); Serial.print(millis()); Serial.print(',');
    Serial.print(leftCommand); Serial.print(','); Serial.print(rightCommand); Serial.print(',');
    Serial.print(l); Serial.print(','); Serial.println(r);
  }
}
