#include <Arduino.h>
#include <avr/interrupt.h>

// ============================================================
// R2 HALL + SC VALIDATION
// No balance controller. Intended for supported robot / wheels
// off the ground during powered tests.
//
// Pin map:
//   Left Hall  A/B/C : D0, D1, D7 (external interrupts)
//   Right Hall A/B/C : D8, D9, D10 (Port B pin-change interrupt)
//   Left SC          : D18 / A0 (polled reference only)
//   Right SC         : D19 / A1 (polled reference only)
//   Left PWM / DIR   : D5 / D12
//   Right PWM / DIR  : D6 / D4
//
// Hall A/B/C labels remain provisional. This sketch counts raw
// state transitions and invalid multi-bit transitions; it does
// NOT yet infer forward/reverse direction.
// ============================================================

const uint8_t LEFT_HA_PIN  = 0;
const uint8_t LEFT_HB_PIN  = 1;
const uint8_t LEFT_HC_PIN  = 7;
const uint8_t RIGHT_HA_PIN = 8;
const uint8_t RIGHT_HB_PIN = 9;
const uint8_t RIGHT_HC_PIN = 10;

const uint8_t LEFT_SC_PIN  = 18;  // A0; polled, not attachInterrupt-capable
const uint8_t RIGHT_SC_PIN = 19;  // A1; polled, not attachInterrupt-capable

const uint8_t LEFT_PWM_PIN  = 5;
const uint8_t RIGHT_PWM_PIN = 6;
const uint8_t LEFT_DIR_PIN  = 12;
const uint8_t RIGHT_DIR_PIN = 4;

const int MAX_PWM = 51;
const int PWM_STEP = 1;
const unsigned long MOTOR_UPDATE_MS = 10;
const unsigned long COMMAND_HOLD_MS = 2000;
const unsigned long TELEMETRY_INTERVAL_MS = 100;

volatile uint8_t leftHallState = 0;
volatile uint8_t rightHallState = 0;
volatile uint32_t leftHallTransitions = 0;
volatile uint32_t rightHallTransitions = 0;
volatile uint32_t leftHallInvalid = 0;
volatile uint32_t rightHallInvalid = 0;

// SC is polled continuously in loop(). Counts are therefore a
// comparison signal, not a guaranteed lossless reference at high RPM.
uint8_t leftSCPrevious = LOW;
uint8_t rightSCPrevious = LOW;
uint32_t leftSCTransitions = 0;
uint32_t rightSCTransitions = 0;

int targetLeft = 0;
int targetRight = 0;
int currentLeft = 0;
int currentRight = 0;
unsigned long commandExpiryTime = 0;
unsigned long lastMotorUpdate = 0;
unsigned long lastTelemetryTime = 0;

static inline uint8_t readLeftHallState() {
  // bit 0 = HA, bit 1 = HB, bit 2 = HC
  return (digitalRead(LEFT_HA_PIN) ? 1 : 0)
       | (digitalRead(LEFT_HB_PIN) ? 2 : 0)
       | (digitalRead(LEFT_HC_PIN) ? 4 : 0);
}

static inline uint8_t readRightHallStateFast() {
  // Leonardo: D8=PB4, D9=PB5, D10=PB6.
  // Read all three simultaneously from PINB.
  uint8_t p = PINB;
  return ((p & _BV(PB4)) ? 1 : 0)
       | ((p & _BV(PB5)) ? 2 : 0)
       | ((p & _BV(PB6)) ? 4 : 0);
}

static inline uint8_t bitCount3(uint8_t x) {
  x &= 0x07;
  return (x & 1) + ((x >> 1) & 1) + ((x >> 2) & 1);
}

static inline void updateLeftHallFromISR() {
  uint8_t s = readLeftHallState();
  uint8_t old = leftHallState;
  if (s != old) {
    uint8_t changed = (s ^ old) & 0x07;
    if (bitCount3(changed) == 1) leftHallTransitions++;
    else leftHallInvalid++;
    leftHallState = s;
  }
}

static inline void updateRightHallFromISR() {
  uint8_t s = readRightHallStateFast();
  uint8_t old = rightHallState;
  if (s != old) {
    uint8_t changed = (s ^ old) & 0x07;
    if (bitCount3(changed) == 1) rightHallTransitions++;
    else rightHallInvalid++;
    rightHallState = s;
  }
}

void leftHallA_ISR() { updateLeftHallFromISR(); }
void leftHallB_ISR() { updateLeftHallFromISR(); }
void leftHallC_ISR() { updateLeftHallFromISR(); }

// D8/D9/D10 are PB4/PB5/PB6 and are captured together here.
ISR(PCINT0_vect) {
  updateRightHallFromISR();
}

void setup() {
  Serial.begin(115200);
  unsigned long serialWaitStart = millis();
  while (!Serial && millis() - serialWaitStart < 2500) { }

  // Explicitly configure all Hall inputs.
  pinMode(LEFT_HA_PIN, INPUT);
  pinMode(LEFT_HB_PIN, INPUT);
  pinMode(LEFT_HC_PIN, INPUT);
  pinMode(RIGHT_HA_PIN, INPUT);
  pinMode(RIGHT_HB_PIN, INPUT);
  pinMode(RIGHT_HC_PIN, INPUT);

  // Explicitly configure SC inputs too.
  pinMode(LEFT_SC_PIN, INPUT);
  pinMode(RIGHT_SC_PIN, INPUT);
  leftSCPrevious = digitalRead(LEFT_SC_PIN);
  rightSCPrevious = digitalRead(RIGHT_SC_PIN);

  // Motor outputs: initialize stopped before accepting commands.
  pinMode(LEFT_PWM_PIN, OUTPUT);
  pinMode(RIGHT_PWM_PIN, OUTPUT);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
  analogWrite(LEFT_PWM_PIN, 0);
  analogWrite(RIGHT_PWM_PIN, 0);
  digitalWrite(LEFT_DIR_PIN, LOW);
  digitalWrite(RIGHT_DIR_PIN, LOW);

  // Establish starting Hall states before enabling interrupts.
  leftHallState = readLeftHallState();
  rightHallState = readRightHallStateFast();

  attachInterrupt(digitalPinToInterrupt(LEFT_HA_PIN), leftHallA_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_HB_PIN), leftHallB_ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_HC_PIN), leftHallC_ISR, CHANGE);

  // Enable Port B pin-change interrupt for PB4/PB5/PB6 = D8/D9/D10.
  cli();
  PCMSK0 |= _BV(PCINT4) | _BV(PCINT5) | _BV(PCINT6);
  PCIFR |= _BV(PCIF0);  // Clear pending Port B change flag.
  PCICR |= _BV(PCIE0);
  sei();

  lastMotorUpdate = millis();
  lastTelemetryTime = millis();

  Serial.println(F("R2 Hall validation; balance control DISABLED."));
  Serial.println(F("Hall state bits: A=bit0, B=bit1, C=bit2; labels provisional."));
  Serial.println(F("Commands: M <left> <right>, L <value>, R <value>, STOP, HELP"));
  Serial.println(F("Motor values -100..100 map to PWM -51..51; each command expires after 2 s."));
  Serial.println(F("Use wheels off ground for motor tests; begin at low commands."));
  Serial.println(F("SC on D18/D19 is polled; SC counts may miss edges at high speed."));
  Serial.println(F("CSV:ms,leftPWM,rightPWM,Lstate,Rstate,LhallTransitions,RhallTransitions,Linvalid,Rinvalid,LscTransitions,RscTransitions,LscLevel,RscLevel"));
}

void loop() {
  sampleSCInputs();
  processSerialCommands();
  updateMotors();
  printTelemetry();
}

void sampleSCInputs() {
  uint8_t l = digitalRead(LEFT_SC_PIN);
  uint8_t r = digitalRead(RIGHT_SC_PIN);
  if (l != leftSCPrevious) {
    leftSCTransitions++;
    leftSCPrevious = l;
  }
  if (r != rightSCPrevious) {
    rightSCTransitions++;
    rightSCPrevious = r;
  }
}

int commandToPWM(int value) {
  value = constrain(value, -100, 100);
  int pwm = map(abs(value), 0, 100, 0, MAX_PWM);
  return value < 0 ? -pwm : pwm;
}

void processSerialCommands() {
  static char line[64];
  static uint8_t used = 0;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      line[used] = '\0';
      handleCommand(line);
      used = 0;
    } else if (used < sizeof(line) - 1) {
      line[used++] = c;
    } else {
      used = 0;
      Serial.println(F("ERR: command too long"));
    }
  }
}

void handleCommand(char *line) {
  char cmd[12] = {0};
  int a = 0, b = 0;
  int n = sscanf(line, "%11s %d %d", cmd, &a, &b);

  if (!strcmp(cmd, "HELP")) {
    Serial.println(F("M <left> <right>; L <value>; R <value>; STOP"));
    return;
  }
  if (!strcmp(cmd, "STOP")) {
    targetLeft = targetRight = 0;
    commandExpiryTime = 0;
    Serial.println(F("STOP requested; ramping to zero."));
    return;
  }
  if (!strcmp(cmd, "M") && n == 3) {
    targetLeft = commandToPWM(a);
    targetRight = commandToPWM(b);
  } else if (!strcmp(cmd, "L") && n >= 2) {
    targetLeft = commandToPWM(a);
  } else if (!strcmp(cmd, "R") && n >= 2) {
    targetRight = commandToPWM(a);
  } else {
    Serial.println(F("ERR: use M <left> <right>, L <value>, R <value>, STOP, or HELP"));
    return;
  }
  commandExpiryTime = millis() + COMMAND_HOLD_MS;
  Serial.print(F("Target PWM L/R: "));
  Serial.print(targetLeft);
  Serial.print('/');
  Serial.println(targetRight);
}

void updateMotors() {
  unsigned long now = millis();
  if (now - lastMotorUpdate < MOTOR_UPDATE_MS) return;
  lastMotorUpdate = now;

  if (commandExpiryTime != 0 && (long)(now - commandExpiryTime) >= 0) {
    targetLeft = targetRight = 0;
    commandExpiryTime = 0;
    Serial.println(F("Motor command expired; ramping to zero."));
  }

  if (currentLeft < targetLeft) currentLeft = min(currentLeft + PWM_STEP, targetLeft);
  else if (currentLeft > targetLeft) currentLeft = max(currentLeft - PWM_STEP, targetLeft);
  if (currentRight < targetRight) currentRight = min(currentRight + PWM_STEP, targetRight);
  else if (currentRight > targetRight) currentRight = max(currentRight - PWM_STEP, targetRight);

  setMotorLeft(currentLeft);
  setMotorRight(currentRight);
}

void setMotorLeft(int command) {
  command = constrain(command, -MAX_PWM, MAX_PWM);
  if (command >= 0) {
    digitalWrite(LEFT_DIR_PIN, HIGH);
    analogWrite(LEFT_PWM_PIN, command);
  } else {
    digitalWrite(LEFT_DIR_PIN, LOW);
    analogWrite(LEFT_PWM_PIN, -command);
  }
}

void setMotorRight(int command) {
  command = constrain(command, -MAX_PWM, MAX_PWM);
  if (command >= 0) {
    digitalWrite(RIGHT_DIR_PIN, LOW);
    analogWrite(RIGHT_PWM_PIN, command);
  } else {
    digitalWrite(RIGHT_DIR_PIN, HIGH);
    analogWrite(RIGHT_PWM_PIN, -command);
  }
}

void printTelemetry() {
  unsigned long now = millis();
  if (now - lastTelemetryTime < TELEMETRY_INTERVAL_MS) return;
  lastTelemetryTime = now;

  uint8_t ls, rs;
  uint32_t lht, rht, li, ri;
  noInterrupts();
  ls = leftHallState;
  rs = rightHallState;
  lht = leftHallTransitions;
  rht = rightHallTransitions;
  li = leftHallInvalid;
  ri = rightHallInvalid;
  interrupts();

  Serial.print(now); Serial.print(',');
  Serial.print(currentLeft); Serial.print(',');
  Serial.print(currentRight); Serial.print(',');
  Serial.print(ls); Serial.print(',');
  Serial.print(rs); Serial.print(',');
  Serial.print(lht); Serial.print(',');
  Serial.print(rht); Serial.print(',');
  Serial.print(li); Serial.print(',');
  Serial.print(ri); Serial.print(',');
  Serial.print(leftSCTransitions); Serial.print(',');
  Serial.print(rightSCTransitions); Serial.print(',');
  Serial.print(leftSCPrevious); Serial.print(',');
  Serial.println(rightSCPrevious);
}
