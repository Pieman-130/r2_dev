#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64

// OLED reset pin. Most I2C OLEDs don't have one.
#define OLED_RESET -1

// Common I2C address is 0x3C
#define SCREEN_ADDRESS 0x3C

Adafruit_SSD1306 display(
  SCREEN_WIDTH,
  SCREEN_HEIGHT,
  &Wire,
  OLED_RESET
);

void setup() {
  Serial.begin(115200);

  // Initialize OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println("OLED initialization failed!");
    while (true);
  }

  Serial.println("OLED initialized!");

  // Clear display
  display.clearDisplay();

  // Text settings
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  display.println("OLED TEST");
  display.println();
  display.println("128 x 64");
  display.println("I2C working!");

  // Actually send the buffer to the OLED
  display.display();

  delay(2000);
}

void loop() {
  // Draw a simple animation
  display.clearDisplay();

  display.setTextSize(2);
  display.setCursor(10, 5);
  display.println("HELLO!");

  display.setTextSize(1);
  display.setCursor(20, 35);
  display.println("OLED is working");

  // Draw a rectangle around the screen
  display.drawRect(0, 0, 128, 64, SSD1306_WHITE);

  display.display();

  delay(500);

  // Invert the display
  display.invertDisplay(true);
  delay(500);

  display.invertDisplay(false);
}