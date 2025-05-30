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

// Create encoder objects
AiEsp32RotaryEncoder rotaryEncoder1(ROTARY1_A_PIN, ROTARY1_B_PIN, ROTARY1_BUTTON_PIN, ROTARY1_VCC_PIN, ROTARY1_STEPS);
AiEsp32RotaryEncoder rotaryEncoder2(ROTARY2_A_PIN, ROTARY2_B_PIN, ROTARY2_BUTTON_PIN, ROTARY2_VCC_PIN, ROTARY2_STEPS);

void IRAM_ATTR readEncoderISR1()
{
  rotaryEncoder1.readEncoder_ISR();
}

void IRAM_ATTR readEncoderISR2()
{
  rotaryEncoder2.readEncoder_ISR();
}

int encoder1Val = 0;
int encoder2Val = 0;
bool setClockMode = true;

DateTime clockTime(__DATE__, __TIME__);

void updateTime(int dHours, int dMinutes)
{
  DateTime now = rtc.now();
  int newHour = (now.hour() + dHours) % 24;
  if (newHour < 0) newHour += 24;

  int newMinute = (now.minute() + dMinutes) % 60;
  if (newMinute < 0) newMinute += 60;

  rtc.adjust(DateTime(now.year(), now.month(), now.day(), newHour, newMinute, now.second()));
  Serial.printf("RTC updated to: %02d:%02d\n", newHour, newMinute);
  clockTime = rtc.now();
}

void checkEncoders()
{
  if (rotaryEncoder1.encoderChanged()) 
  {
    const long oldVal = encoder1Val;
    encoder1Val = rotaryEncoder1.readEncoder();

    const long delta = encoder1Val - oldVal;
    printf("Encoder 1 value: %d | delta: %d \n", encoder1Val, delta);

    updateTime(delta, 0);
  }

  if (rotaryEncoder2.encoderChanged()) 
  {
    const long oldVal = encoder2Val;
    encoder2Val = rotaryEncoder2.readEncoder();

    const long delta = encoder2Val - oldVal;
    printf("Encoder 2 value: %d | delta: %d \n", encoder2Val, delta);

    updateTime(0, delta);
  }
}


void setup()
{
  Serial.begin(115200);

  rtc.begin();

  clockTime = rtc.now();

  if (rtc.lostPower()) {
    Serial.println("RTC not running!");
    rtc.adjust(clockTime);
  }


  // Initialize Encoder 1
  rotaryEncoder1.begin();
  rotaryEncoder1.setEncoderValue(0);
  rotaryEncoder1.setup(readEncoderISR1);
  rotaryEncoder1.setAcceleration(0);

  // Initialize Encoder 2
  rotaryEncoder2.begin();
  rotaryEncoder2.setEncoderValue(0);
  rotaryEncoder2.setup(readEncoderISR2);
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

void renderScreen()
{
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
}

void loop()
{
  checkEncoders();
  delay(50);  // For encoder responsiveness

  unsigned long currentMillis = millis();

  if (currentMillis - lastUpdate >= 1000) {
    lastUpdate = currentMillis;

    renderScreen();
  }
}
