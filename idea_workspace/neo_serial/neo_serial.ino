#include <Adafruit_NeoPixel.h>

#define LED_PIN    0
#define LED_COUNT  16

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(115200);
  strip.begin();
  strip.show();
  Serial.println("Arduino ready (command + checksum)");
}

void loop() {
  // We know the maximum possible packet size (update command)
  const int MAX_PACKET = 1 + 1 + (LED_COUNT * 3) + 1;  // 51 bytes

  if (Serial.available() >= MAX_PACKET) {
    // Wait for start header
    if (Serial.read() == 0xAA) {
      uint8_t cmd = Serial.read();

      if (cmd == 0x01) {  // === LED FRAME UPDATE ===
        uint8_t buffer[LED_COUNT * 3];
        int bytesRead = Serial.readBytes((char*)buffer, LED_COUNT * 3);
        uint8_t checksum = Serial.read();

        // Compute checksum
        uint8_t calc_sum = cmd;  // include command byte
        for (int i = 0; i < LED_COUNT * 3; i++) {
          calc_sum += buffer[i];
        }
        calc_sum &= 0xFF;

        if (calc_sum == checksum && bytesRead == LED_COUNT * 3) {
          // ✅ Valid packet, update pixels
          for (int i = 0; i < LED_COUNT; i++) {
            uint8_t r = buffer[i * 3];
            uint8_t g = buffer[i * 3 + 1];
            uint8_t b = buffer[i * 3 + 2];
            strip.setPixelColor(i, strip.Color(r, g, b));
          }
          strip.show();
          Serial.println("Frame OK");
        } else {
          Serial.println("Checksum error");
        }

      } else if (cmd == 0x02) {  // === CLEAR STRIP ===
        strip.clear();
        strip.show();
        uint8_t checksum = Serial.read(); // still need to read checksum byte
        Serial.println("Strip cleared");

      } else if (cmd == 0x03) {  // === SET BRIGHTNESS ===
        uint8_t brightness = Serial.read(); // one byte payload
        uint8_t checksum = Serial.read();

        uint8_t calc_sum = (cmd + brightness) & 0xFF;
        if (calc_sum == checksum) {
          strip.setBrightness(brightness);
          strip.show();
          Serial.print("Brightness set to ");
          Serial.println(brightness);
        } else {
          Serial.println("Brightness checksum error");
        }

      } else {
        Serial.println("Unknown command");
      }
    }
     // ✅ Clear buffer after handling one command
    while (Serial.available() > 0) {
      Serial.read();
    }
  }
}