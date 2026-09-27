#include <Arduino.h>
#include <stdint.h>
#include <string.h>

// ============================================================
// PIN ASSIGNMENTS
// ============================================================

const int RCliff = A4;
const int FCliff = A5;

const int RtTrigPin = 13;
const int RtEchoPin = 12;

const int LtTrigPin = 11;
const int LtEchoPin = 10;

const int RrTrigPin = 9;
const int RrEchoPin = 8;

const int RtHall = 7;   // PD7 / PCINT23
const int LtHall = 6;   // PD6 / PCINT22


// ============================================================
// EXISTING ENVIRONMENTAL PACKET
// ============================================================

const byte PREAMBLE[]  = {0xAA, 0x55};
const byte POSTAMBLE[] = {0x55, 0xAA};

int32_t sensor[7];


// ============================================================
// HALL SENSOR SYSTEM
//
// 90 transitions = 1 wheel revolution
//
// We measure:
//   - transition-to-transition period
//   - HIGH pulse width
//   - cumulative transition count
// ============================================================

const uint8_t TRANSITIONS_PER_REV = 90;
const uint32_t HALL_TIMEOUT_US = 1000000UL;

// Right wheel
volatile uint32_t rightLastTransitionUs = 0;
volatile uint32_t rightPeriodUs = 0;
volatile uint32_t rightHighStartUs = 0;
volatile uint32_t rightHighWidthUs = 0;
volatile uint32_t rightTransitionCount = 0;
volatile bool rightHallInitialized = false;

// Left wheel
volatile uint32_t leftLastTransitionUs = 0;
volatile uint32_t leftPeriodUs = 0;
volatile uint32_t leftHighStartUs = 0;
volatile uint32_t leftHighWidthUs = 0;
volatile uint32_t leftTransitionCount = 0;
volatile bool leftHallInitialized = false;


// ============================================================
// RPM FILTER
// ============================================================

const uint8_t RPM_FILTER_SIZE = 5;

float rightRpmHistory[RPM_FILTER_SIZE] = {0};
float leftRpmHistory[RPM_FILTER_SIZE] = {0};

uint8_t rightRpmIndex = 0;
uint8_t leftRpmIndex = 0;

float rightFilteredRPM = 0.0f;
float leftFilteredRPM = 0.0f;


// ============================================================
// HC-SR04 SYSTEM
//
// Three sensors are measured sequentially to prevent
// ultrasonic cross-talk.
//
// D12 = right echo
// D10 = left echo
// D8  = rear echo
// ============================================================

struct UltrasonicSensor {

  uint8_t trigPin;
  uint8_t echoPin;

  volatile uint32_t echoStartUs;
  volatile uint32_t echoDurationUs;

  volatile bool echoActive;
  volatile bool measurementReady;

  uint32_t triggerTimeUs;
  bool waitingForEcho;
};


UltrasonicSensor ultrasonic[3] = {

  {RtTrigPin, RtEchoPin,
   0, 0,
   false, false,
   0, false},

  {LtTrigPin, LtEchoPin,
   0, 0,
   false, false,
   0, false},

  {RrTrigPin, RrEchoPin,
   0, 0,
   false, false,
   0, false}
};


const uint32_t ULTRASONIC_TIMEOUT_US = 30000UL;

// Small quiet period between sensors to reduce cross-talk.
const uint32_t ULTRASONIC_GAP_US = 2000UL;

uint8_t ultrasonicIndex = 0;
uint32_t lastUltrasonicCompleteUs = 0;


// ============================================================
// PACKET TIMING
// ============================================================

const uint32_t ENV_PACKET_INTERVAL_MS = 100;  // 10 Hz
const uint32_t WHEEL_PACKET_INTERVAL_MS = 20; // 50 Hz

uint32_t lastEnvPacketMs = 0;
uint32_t lastWheelPacketMs = 0;


// ============================================================
// PIN-CHANGE ISR STATE
//
// IMPORTANT:
// These are initialized in setup() BEFORE interrupts are
// enabled. This prevents startup pin states from appearing
// as fake transitions.
// ============================================================

volatile uint8_t previousPortD = 0;
volatile uint8_t previousPortB = 0;


// ============================================================
// PORTD PIN-CHANGE INTERRUPT
//
// D6 = left Hall
// D7 = right Hall
// ============================================================

ISR(PCINT2_vect) {

  uint8_t currentPortD = PIND;

  uint8_t changed =
      currentPortD ^ previousPortD;

  uint32_t now = micros();


  // ----------------------------------------------------------
  // LEFT HALL - D6 / PD6
  // ----------------------------------------------------------

  if (changed & _BV(PD6)) {

    bool high =
        currentPortD & _BV(PD6);

    if (high) {

      // Rising edge

      if (leftHallInitialized) {

        uint32_t period =
            now - leftLastTransitionUs;

        if (period > 0) {
          leftPeriodUs = period;
        }
      }

      leftHighStartUs = now;
      leftLastTransitionUs = now;

      leftTransitionCount++;

      leftHallInitialized = true;

    } else {

      // Falling edge

      if (leftHighStartUs != 0) {

        leftHighWidthUs =
            now - leftHighStartUs;
      }
    }
  }


  // ----------------------------------------------------------
  // RIGHT HALL - D7 / PD7
  // ----------------------------------------------------------

  if (changed & _BV(PD7)) {

    bool high =
        currentPortD & _BV(PD7);

    if (high) {

      // Rising edge

      if (rightHallInitialized) {

        uint32_t period =
            now - rightLastTransitionUs;

        if (period > 0) {
          rightPeriodUs = period;
        }
      }

      rightHighStartUs = now;
      rightLastTransitionUs = now;

      rightTransitionCount++;

      rightHallInitialized = true;

    } else {

      // Falling edge

      if (rightHighStartUs != 0) {

        rightHighWidthUs =
            now - rightHighStartUs;
      }
    }
  }


  previousPortD = currentPortD;
}


// ============================================================
// PORTB PIN-CHANGE INTERRUPT
//
// D8  = rear ultrasonic echo
// D10 = left ultrasonic echo
// D12 = right ultrasonic echo
// ============================================================

ISR(PCINT0_vect) {

  uint8_t currentPortB = PINB;

  uint8_t changed =
      currentPortB ^ previousPortB;

  uint32_t now = micros();


  // ----------------------------------------------------------
  // RIGHT ECHO - D12 / PB4
  // ----------------------------------------------------------

  if (changed & _BV(PB4)) {

    bool high =
        currentPortB & _BV(PB4);

    if (high) {

      ultrasonic[0].echoStartUs = now;
      ultrasonic[0].echoActive = true;

    } else if (ultrasonic[0].echoActive) {

      ultrasonic[0].echoDurationUs =
          now - ultrasonic[0].echoStartUs;

      ultrasonic[0].echoActive = false;
      ultrasonic[0].measurementReady = true;
    }
  }


  // ----------------------------------------------------------
  // LEFT ECHO - D10 / PB2
  // ----------------------------------------------------------

  if (changed & _BV(PB2)) {

    bool high =
        currentPortB & _BV(PB2);

    if (high) {

      ultrasonic[1].echoStartUs = now;
      ultrasonic[1].echoActive = true;

    } else if (ultrasonic[1].echoActive) {

      ultrasonic[1].echoDurationUs =
          now - ultrasonic[1].echoStartUs;

      ultrasonic[1].echoActive = false;
      ultrasonic[1].measurementReady = true;
    }
  }


  // ----------------------------------------------------------
  // REAR ECHO - D8 / PB0
  // ----------------------------------------------------------

  if (changed & _BV(PB0)) {

    bool high =
        currentPortB & _BV(PB0);

    if (high) {

      ultrasonic[2].echoStartUs = now;
      ultrasonic[2].echoActive = true;

    } else if (ultrasonic[2].echoActive) {

      ultrasonic[2].echoDurationUs =
          now - ultrasonic[2].echoStartUs;

      ultrasonic[2].echoActive = false;
      ultrasonic[2].measurementReady = true;
    }
  }


  previousPortB = currentPortB;
}


// ============================================================
// TRIGGER HC-SR04
// ============================================================

void triggerUltrasonic(uint8_t index) {

  UltrasonicSensor &us =
      ultrasonic[index];

  digitalWrite(us.trigPin, LOW);
  delayMicroseconds(2);

  digitalWrite(us.trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(us.trigPin, LOW);

  us.triggerTimeUs = micros();
  us.waitingForEcho = true;
}


// ============================================================
// SERVICE HC-SR04 STATE MACHINE
// ============================================================

void serviceUltrasonic() {

  uint32_t now = micros();

  UltrasonicSensor &us =
      ultrasonic[ultrasonicIndex];


  // ----------------------------------------------------------
  // Currently waiting for echo
  // ----------------------------------------------------------

  if (us.waitingForEcho) {

    bool ready;

    noInterrupts();

    ready =
        us.measurementReady;

    interrupts();


    if (ready) {

      us.waitingForEcho = false;

      lastUltrasonicCompleteUs =
          now;

      ultrasonicIndex++;

      if (ultrasonicIndex >= 3) {
        ultrasonicIndex = 0;
      }

      return;
    }


    // --------------------------------------------------------
    // Timeout
    // --------------------------------------------------------

    if ((uint32_t)
        (now - us.triggerTimeUs)
        >= ULTRASONIC_TIMEOUT_US) {

      noInterrupts();

      us.echoActive = false;
      us.measurementReady = false;
      us.echoDurationUs = 0;

      interrupts();

      us.waitingForEcho = false;

      ultrasonicIndex++;

      if (ultrasonicIndex >= 3) {
        ultrasonicIndex = 0;
      }

      lastUltrasonicCompleteUs =
          now;
    }

    return;
  }


  // ----------------------------------------------------------
  // Idle - wait before next trigger
  // ----------------------------------------------------------

  if ((uint32_t)
      (now - lastUltrasonicCompleteUs)
      >= ULTRASONIC_GAP_US) {

    noInterrupts();

    us.measurementReady = false;
    us.echoActive = false;

    interrupts();

    triggerUltrasonic(
        ultrasonicIndex);
  }
}


// ============================================================
// UPDATE ULTRASONIC VALUES
//
// sensor[0] = right echo duration
// sensor[1] = left echo duration
// sensor[2] = rear echo duration
//
// Values remain raw microseconds, just like the original
// environmental packet.
// ============================================================

void updateUltrasonicValues() {

  for (uint8_t i = 0; i < 3; i++) {

    uint32_t duration;

    noInterrupts();

    duration =
        ultrasonic[i].echoDurationUs;

    interrupts();

    sensor[i] =
        (int32_t)duration;
  }
}


// ============================================================
// UPDATE CLIFF SENSORS
// ============================================================

void updateCliffValues() {

  sensor[3] =
      analogRead(FCliff);

  sensor[4] =
      analogRead(RCliff);
}


// ============================================================
// UPDATE HALL VALUES FOR EXISTING ENVIRONMENT PACKET
//
// sensor[5] = right HIGH pulse width
// sensor[6] = left HIGH pulse width
//
// These are retained because we now have freedom to change
// the implementation without needing to preserve the old
// pulseIn() implementation itself.
// ============================================================

void updateHallEnvironmentalValues() {

  uint32_t rightHigh;
  uint32_t leftHigh;

  noInterrupts();

  rightHigh =
      rightHighWidthUs;

  leftHigh =
      leftHighWidthUs;

  interrupts();

  sensor[5] =
      (int32_t)rightHigh;

  sensor[6] =
      (int32_t)leftHigh;
}


// ============================================================
// RPM CALCULATION
// ============================================================

float rpmFromPeriod(uint32_t periodUs) {

  if (periodUs == 0) {
    return 0.0f;
  }

  return
      60000000.0f /
      ((float)periodUs *
       TRANSITIONS_PER_REV);
}


// ============================================================
// MOVING AVERAGE
// ============================================================

float updateRpmFilter(
    float *history,
    uint8_t &index,
    float newValue) {

  history[index] =
      newValue;

  index++;

  if (index >= RPM_FILTER_SIZE) {
    index = 0;
  }

  float sum = 0.0f;

  for (uint8_t i = 0;
       i < RPM_FILTER_SIZE;
       i++) {

    sum += history[i];
  }

  return
      sum /
      RPM_FILTER_SIZE;
}


// ============================================================
// UPDATE WHEEL RPM
// ============================================================

void updateWheelRPM() {

  uint32_t rightPeriod;
  uint32_t leftPeriod;

  uint32_t rightLast;
  uint32_t leftLast;

  noInterrupts();

  rightPeriod =
      rightPeriodUs;

  leftPeriod =
      leftPeriodUs;

  rightLast =
      rightLastTransitionUs;

  leftLast =
      leftLastTransitionUs;

  interrupts();

  uint32_t now =
      micros();


  // ----------------------------------------------------------
  // RIGHT
  // ----------------------------------------------------------

  if (rightLast == 0 ||
      (uint32_t)
      (now - rightLast)
      > HALL_TIMEOUT_US) {

    rightFilteredRPM =
        updateRpmFilter(
            rightRpmHistory,
            rightRpmIndex,
            0.0f);

  } else {

    float rpm =
        rpmFromPeriod(
            rightPeriod);

    rightFilteredRPM =
        updateRpmFilter(
            rightRpmHistory,
            rightRpmIndex,
            rpm);
  }


  // ----------------------------------------------------------
  // LEFT
  // ----------------------------------------------------------

  if (leftLast == 0 ||
      (uint32_t)
      (now - leftLast)
      > HALL_TIMEOUT_US) {

    leftFilteredRPM =
        updateRpmFilter(
            leftRpmHistory,
            leftRpmIndex,
            0.0f);

  } else {

    float rpm =
        rpmFromPeriod(
            leftPeriod);

    leftFilteredRPM =
        updateRpmFilter(
            leftRpmHistory,
            leftRpmIndex,
            rpm);
  }
}


// ============================================================
// SEND EXISTING ENVIRONMENTAL PACKET
// ============================================================

void sendEnvironmentalPacket() {

  byte *p =
      (byte *)sensor;

  byte checksum = 0;

  for (size_t i = 0;
       i < sizeof(sensor);
       i++) {

    checksum ^= p[i];
  }


  byte packet[33];

  packet[0] =
      PREAMBLE[0];

  packet[1] =
      PREAMBLE[1];

  memcpy(
      &packet[2],
      p,
      sizeof(sensor));

  packet[30] =
      checksum;

  packet[31] =
      POSTAMBLE[0];

  packet[32] =
      POSTAMBLE[1];

  Serial.write(
      packet,
      sizeof(packet));
}


// ============================================================
// SEND WHEEL PACKET
//
// AA 56 LEN TYPE
//
// Payload:
//   timestamp_ms
//   right_period_us
//   left_period_us
//   right_transition_count
//   left_transition_count
//   right_rpm_x100
//   left_rpm_x100
//   right_age_ms
//   left_age_ms
//
// checksum
// 55 AA
//
// Total = 43 bytes
// ============================================================

void sendWheelPacket() {

  const uint8_t TYPE =
      0x01;

  const uint8_t PAYLOAD_LENGTH =
      36;

  byte packet[43];

  uint32_t timestampMs =
      millis();

  uint32_t rightPeriod;
  uint32_t leftPeriod;

  uint32_t rightCount;
  uint32_t leftCount;

  uint32_t rightLast;
  uint32_t leftLast;

  noInterrupts();

  rightPeriod =
      rightPeriodUs;

  leftPeriod =
      leftPeriodUs;

  rightCount =
      rightTransitionCount;

  leftCount =
      leftTransitionCount;

  rightLast =
      rightLastTransitionUs;

  leftLast =
      leftLastTransitionUs;

  interrupts();


  uint32_t nowUs =
      micros();

  uint32_t rightAgeMs;
  uint32_t leftAgeMs;


  if (rightLast == 0) {

    rightAgeMs =
        0xFFFFFFFFUL;

  } else {

    rightAgeMs =
        (nowUs - rightLast)
        / 1000UL;
  }


  if (leftLast == 0) {

    leftAgeMs =
        0xFFFFFFFFUL;

  } else {

    leftAgeMs =
        (nowUs - leftLast)
        / 1000UL;
  }


  uint32_t rightRpmX100 =
      (uint32_t)
      (rightFilteredRPM * 100.0f);

  uint32_t leftRpmX100 =
      (uint32_t)
      (leftFilteredRPM * 100.0f);


  // ----------------------------------------------------------
  // Build payload
  // ----------------------------------------------------------

  byte payload[PAYLOAD_LENGTH];

  uint8_t pos = 0;


  memcpy(
      &payload[pos],
      &timestampMs,
      4);

  pos += 4;


  memcpy(
      &payload[pos],
      &rightPeriod,
      4);

  pos += 4;


  memcpy(
      &payload[pos],
      &leftPeriod,
      4);

  pos += 4;


  memcpy(
      &payload[pos],
      &rightCount,
      4);

  pos += 4;


  memcpy(
      &payload[pos],
      &leftCount,
      4);

  pos += 4;


  memcpy(
      &payload[pos],
      &rightRpmX100,
      4);

  pos += 4;


  memcpy(
      &payload[pos],
      &leftRpmX100,
      4);

  pos += 4;


  memcpy(
      &payload[pos],
      &rightAgeMs,
      4);

  pos += 4;


  memcpy(
      &payload[pos],
      &leftAgeMs,
      4);

  pos += 4;


  // ----------------------------------------------------------
  // Header
  // ----------------------------------------------------------

  packet[0] =
      0xAA;

  packet[1] =
      0x56;

  packet[2] =
      PAYLOAD_LENGTH;

  packet[3] =
      TYPE;


  memcpy(
      &packet[4],
      payload,
      PAYLOAD_LENGTH);


  // ----------------------------------------------------------
  // Checksum
  // ----------------------------------------------------------

  byte checksum = 0;

  for (uint8_t i = 0;
       i < PAYLOAD_LENGTH;
       i++) {

    checksum ^=
        payload[i];
  }


  packet[40] =
      checksum;

  packet[41] =
      0x55;

  packet[42] =
      0xAA;


  Serial.write(
      packet,
      sizeof(packet));
}


// ============================================================
// SETUP
// ============================================================

void setup() {

  // ----------------------------------------------------------
  // Pin modes
  // ----------------------------------------------------------

  pinMode(LtTrigPin, OUTPUT);
  pinMode(LtEchoPin, INPUT);

  pinMode(RtTrigPin, OUTPUT);
  pinMode(RtEchoPin, INPUT);

  pinMode(RrTrigPin, OUTPUT);
  pinMode(RrEchoPin, INPUT);

  pinMode(RtHall, INPUT);
  pinMode(LtHall, INPUT);


  digitalWrite(
      LtTrigPin,
      LOW);

  digitalWrite(
      RtTrigPin,
      LOW);

  digitalWrite(
      RrTrigPin,
      LOW);


  // ----------------------------------------------------------
  // Capture the ACTUAL pin states before enabling interrupts.
  // ----------------------------------------------------------

  previousPortD =
      PIND;

  previousPortB =
      PINB;


  // ----------------------------------------------------------
  // Enable pin-change interrupts
  // ----------------------------------------------------------

  noInterrupts();

  // PORTB:
  // D8  = PCINT0
  // D10 = PCINT2
  // D12 = PCINT4

  PCICR |=
      _BV(PCIE0);

  PCMSK0 |=
      _BV(PCINT0);

  PCMSK0 |=
      _BV(PCINT2);

  PCMSK0 |=
      _BV(PCINT4);


  // PORTD:
  // D6 = PCINT22
  // D7 = PCINT23

  PCICR |=
      _BV(PCIE2);

  PCMSK2 |=
      _BV(PCINT22);

  PCMSK2 |=
      _BV(PCINT23);


  interrupts();


  Serial.begin(115200);

  lastEnvPacketMs =
      millis();

  lastWheelPacketMs =
      millis();

  lastUltrasonicCompleteUs =
      micros();
}


// ============================================================
// MAIN LOOP
// ============================================================

void loop() {

  uint32_t nowMs =
      millis();


  // ----------------------------------------------------------
  // Continuously service HC-SR04 state machine
  // ----------------------------------------------------------

  serviceUltrasonic();


  // ----------------------------------------------------------
  // Continuously update wheel RPM
  // ----------------------------------------------------------

  updateWheelRPM();


  // ----------------------------------------------------------
  // ENVIRONMENTAL PACKET
  // ----------------------------------------------------------

  if ((uint32_t)
      (nowMs - lastEnvPacketMs)
      >= ENV_PACKET_INTERVAL_MS) {

    lastEnvPacketMs =
        nowMs;

    updateUltrasonicValues();
    updateCliffValues();
    updateHallEnvironmentalValues();

    sendEnvironmentalPacket();
  }


  // ----------------------------------------------------------
  // WHEEL PACKET
  // ----------------------------------------------------------

  if ((uint32_t)
      (nowMs - lastWheelPacketMs)
      >= WHEEL_PACKET_INTERVAL_MS) {

    lastWheelPacketMs =
        nowMs;

    sendWheelPacket();
  }
}