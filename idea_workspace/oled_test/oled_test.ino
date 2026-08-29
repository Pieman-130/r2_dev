#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1   // Set to -1 if sharing Arduino reset pin
#define SCREEN_ADDRESS 0x3C // Common addr; try 0x3D if this doesn't work

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

void setup() {
  Serial.begin(115200);

  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (true); // halt
  }

  display.clearDisplay();

  // Text test
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("SSD1306 OLED Test");
  display.println("128x64");
  display.display();
  delay(2000);

  // Shape test
  display.clearDisplay();
  display.drawRect(0, 0, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
  display.fillCircle(SCREEN_WIDTH / 2, SCREEN_HEIGHT / 2, 15, SSD1306_WHITE);
  display.display();
  delay(2000);
}

void loop() {
  // Scrolling text demo
  static int x = SCREEN_WIDTH;
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(x, 24);
  display.print("Hello!");
  display.display();

  x -= 2;
  if (x < -80) x = SCREEN_WIDTH;

  delay(30);
}