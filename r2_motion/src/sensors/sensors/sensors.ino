const byte PREAMBLE[]  = {0xAA, 0x55};
const byte POSTAMBLE[] = {0x55, 0xAA};

const int RCliff = A4;
const int FCliff = A5;
const int RtTrigPin = 13;
const int RtEchoPin = 12;
const int LtTrigPin = 11;
const int LtEchoPin = 10;
const int RrTrigPin = 9;
const int RrEchoPin = 8;
const int RtHall = 7;
const int LtHall = 6;

int32_t sensor[7];

// Ultrasonic distance sensors
int32_t distanceSensor(int trig_pin, int echo_pin) {
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);
  int32_t dur = pulseIn(echo_pin, HIGH, 30000);  // timeout to avoid blocking forever
  return dur;
}

// IR Cliff sensors
int32_t cliff(int pin) {
  return analogRead(pin);
}

// Wheel speed hall sensors
int32_t speedSensor(int hall) {
  return pulseIn(hall, HIGH, 100000); // timeout 100ms
}

void setup() {
  pinMode(LtTrigPin, OUTPUT);
  pinMode(LtEchoPin, INPUT);
  pinMode(RtTrigPin, OUTPUT);
  pinMode(RtEchoPin, INPUT);
  pinMode(RrTrigPin, OUTPUT);
  pinMode(RrEchoPin, INPUT);
  pinMode(RtHall, INPUT);
  pinMode(LtHall, INPUT);

  Serial.begin(115200);
}

void loop() {
  // Collect sensor data
  sensor[0] = distanceSensor(LtTrigPin, LtEchoPin);
  sensor[1] = distanceSensor(RtTrigPin, RtEchoPin);
  sensor[2] = distanceSensor(RrTrigPin, RrEchoPin);
  sensor[3] = cliff(FCliff);
  sensor[4] = cliff(RCliff);
  sensor[5] = speedSensor(RtHall);
  sensor[6] = speedSensor(LtHall);

  // Convert sensor array to bytes
  byte* p = (byte*)sensor;

  // Compute checksum (XOR of all data bytes)
  byte checksum = 0;
  for (size_t i = 0; i < sizeof(sensor); i++) {
    checksum ^= p[i];
  }

  // Construct packet
  byte packet[33];
  packet[0] = PREAMBLE[0];
  packet[1] = PREAMBLE[1];
  memcpy(&packet[2], p, sizeof(sensor)); // copy 28 bytes
  packet[30] = checksum;
  packet[31] = POSTAMBLE[0];
  packet[32] = POSTAMBLE[1];

  // Send packet
  Serial.write(packet, sizeof(packet));

  delay(100);  // small delay between packets
}
