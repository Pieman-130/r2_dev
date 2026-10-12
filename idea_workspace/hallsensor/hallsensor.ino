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

// Previous GPIO snapshots used by the existing HC-SR04
// pin-change interrupt handlers. These are unrelated to the
// wheel encoders, which have been moved to the Leonardo.
volatile uint8_t previousPortB = 0;
volatile uint8_t previousPortD = 0;

// ============================================================
// EXISTING ENVIRONMENTAL PACKET
// ============================================================

const byte PREAMBLE[]  = {0xAA, 0x55};
const byte POSTAMBLE[] = {0x55, 0xAA};

int32_t sensor[7];


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

uint32_t lastEnvPacketMs = 0;


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
// RESERVED ENVIRONMENT PACKET FIELDS
//
// The wheel encoders have been moved to the Leonardo.
// We intentionally keep the original 7 x int32 packet:
//
//   sensor[0] = right ultrasonic
//   sensor[1] = left ultrasonic
//   sensor[2] = rear ultrasonic
//   sensor[3] = front cliff IR
//   sensor[4] = rear cliff IR
//   sensor[5] = reserved
//   sensor[6] = reserved
//
// Reserved fields are transmitted as zero.
// ============================================================

void updateReservedValues() {

  sensor[5] = 0;
  sensor[6] = 0;
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
  // Enable pin-change interrupts for ultrasonic echo inputs.
  // Wheel encoder interrupts are now handled by the Leonardo.
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

  interrupts();


  Serial.begin(115200);

  lastEnvPacketMs =
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
  // ENVIRONMENTAL PACKET
  // ----------------------------------------------------------

  if ((uint32_t)
      (nowMs - lastEnvPacketMs)
      >= ENV_PACKET_INTERVAL_MS) {

    lastEnvPacketMs =
        nowMs;

    updateUltrasonicValues();
    updateCliffValues();
    updateReservedValues();

    sendEnvironmentalPacket();
  }


}
