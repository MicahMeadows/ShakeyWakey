#include <Arduino.h>
#include "AiEsp32RotaryEncoder.h"
#include "Arduino.h"
#include "RTClib.h"
#include <GxEPD2_BW.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeMonoBold18pt7b.h>


// Encoder 1 pins
#define ROTARY1_A_PIN 32
#define ROTARY1_B_PIN 34
#define ROTARY1_BUTTON_PIN 25
#define ROTARY1_VCC_PIN -1
#define ROTARY1_STEPS 4

// Encoder 2 pins (change to your pins)
#define ROTARY2_A_PIN 33
#define ROTARY2_B_PIN 35
#define ROTARY2_BUTTON_PIN 26
#define ROTARY2_VCC_PIN -1
#define ROTARY2_STEPS 4

GxEPD2_BW<GxEPD2_420_GYE042A87, GxEPD2_420_GYE042A87::HEIGHT> display(GxEPD2_420_GYE042A87(/*CS=5*/ SS, /*DC=*/ 17, /*RST=*/ 16, /*BUSY=*/ 4)); // 400x300

RTC_DS3231 rtc;

volatile bool updateRtcFromEncoder1 = false;
volatile bool updateRtcFromEncoder2 = false;

// Create encoder objects
AiEsp32RotaryEncoder rotaryEncoder1(ROTARY1_A_PIN, ROTARY1_B_PIN, ROTARY1_BUTTON_PIN, ROTARY1_VCC_PIN, ROTARY1_STEPS);
AiEsp32RotaryEncoder rotaryEncoder2(ROTARY2_A_PIN, ROTARY2_B_PIN, ROTARY2_BUTTON_PIN, ROTARY2_VCC_PIN, ROTARY2_STEPS);

void IRAM_ATTR readEncoderISR1()
{
  rotaryEncoder1.readEncoder_ISR();
  updateRtcFromEncoder1 = true;
}

void IRAM_ATTR readEncoderISR2()
{
  rotaryEncoder2.readEncoder_ISR();
  updateRtcFromEncoder2 = true;
}

void rotary_onButtonClick1()
{
  static unsigned long lastTimePressed = 0;
  if (millis() - lastTimePressed < 500) return;
  lastTimePressed = millis();
  Serial.print("Encoder 1 button pressed at ");
  Serial.print(millis());
  Serial.println(" ms");
}

void rotary_onButtonClick2()
{
  static unsigned long lastTimePressed = 0;
  if (millis() - lastTimePressed < 500) return;
  lastTimePressed = millis();
  Serial.print("Encoder 2 button pressed at ");
  Serial.print(millis());
  Serial.println(" ms");
}

int lastEncoder1Val = 0;
int lastEncoder2Val = 0;

void rotary_loop()
{
  // Encoder 1: Hours
  if (rotaryEncoder1.encoderChanged())
  {
    int newVal = rotaryEncoder1.readEncoder();
    int delta = newVal - lastEncoder1Val;
    lastEncoder1Val = newVal;

    DateTime now = rtc.now();
    int newHour = (now.hour() + delta) % 24;
    if (newHour < 0) newHour += 24;
    rtc.adjust(DateTime(now.year(), now.month(), now.day(), newHour, now.minute(), now.second()));
    Serial.printf("RTC hour updated to: %02d\n", newHour);
  }

  if (rotaryEncoder1.isEncoderButtonClicked())
    rotary_onButtonClick1();

  // Encoder 2: Minutes
  if (rotaryEncoder2.encoderChanged())
  {
    int newVal = rotaryEncoder2.readEncoder();
    int delta = newVal - lastEncoder2Val;
    lastEncoder2Val = newVal;

    DateTime now = rtc.now();
    int newMinute = (now.minute() + delta) % 60;
    if (newMinute < 0) newMinute += 60;
    rtc.adjust(DateTime(now.year(), now.month(), now.day(), now.hour(), newMinute, now.second()));
    Serial.printf("RTC minute updated to: %02d\n", newMinute);
  }

  if (rotaryEncoder2.isEncoderButtonClicked())
    rotary_onButtonClick2();
}


void setup()
{
  Serial.begin(115200);

  rtc.begin();

  if (rtc.lostPower()) {
    Serial.println("RTC not running!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  // Initialize Encoder 1
  rotaryEncoder1.begin();
  rotaryEncoder1.setup(readEncoderISR1);
  rotaryEncoder1.setBoundaries(0, 23, true);
  rotaryEncoder1.setAcceleration(0);

  // Initialize Encoder 2
  rotaryEncoder2.begin();
  rotaryEncoder2.setup(readEncoderISR2);
  rotaryEncoder2.setBoundaries(0, 59, true);
  rotaryEncoder2.setAcceleration(0);

  display.init(115200, true, 2, false);
  display.setRotation(0);
  display.setTextColor(GxEPD_BLACK);
  display.setFont(&FreeMonoBold18pt7b);
  display.setTextSize(1);
  display.setFullWindow();
  display.firstPage();
  do {
    display.fillScreen(GxEPD_WHITE);
  } while (display.nextPage());
}

unsigned long lastUpdate = 0;

void loop()
{
  rotary_loop();
  delay(50);  // For encoder responsiveness

  unsigned long currentMillis = millis();

  if (currentMillis - lastUpdate >= 1000) {
    lastUpdate = currentMillis;

    // Get current RTC time
    DateTime now = rtc.now();
    int hours = now.hour();
    int minutes = now.minute();

    // Format HH:MM
    char timeStr[6];
    snprintf(timeStr, sizeof(timeStr), "%02d:%02d", hours, minutes);

    // Calculate text bounds
    int16_t x1, y1;
    uint16_t w, h;
    display.setFont(&FreeMonoBold18pt7b);
    display.getTextBounds(timeStr, 0, 0, &x1, &y1, &w, &h);

    // Center text on 400x300 screen
    int16_t x = (400 - w) / 2;
    int16_t y = (300 + h) / 2;

    // Define a partial window with padding
    int16_t pad = 10;
    int16_t pw = w + pad * 2;
    int16_t ph = h + pad * 2;
    int16_t px = x - pad;
    int16_t py = y - h - pad;

    // Draw to e-ink screen
    display.setPartialWindow(px, py, pw, ph);
    display.firstPage();
    do {
      display.fillRect(px, py, pw, ph, GxEPD_WHITE);
      display.setCursor(x, y);
      display.print(timeStr);
    } while (display.nextPage());

    // Optional: Serial print
    Serial.println(timeStr);
  }
}
