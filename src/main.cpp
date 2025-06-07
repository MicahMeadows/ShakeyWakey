#include "Arduino.h"
#include <stdlib.h>
#include <Wire.h>
#include "AiEsp32RotaryEncoder.h"
#include "RTClib.h"
#include <GxEPD2_BW.h>
#include <Fonts/FreeMonoBold18pt7b.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeMono9pt7b.h>
#include "images.h"

#define RTC_INT_PIN 26
// volatile bool clockNeedsUpdate = true;
volatile bool shouldCheckAlarm = false;

#define BUZZER_PIN 15

#define SHAKE_PIN 13
#define SHAKE_TIMEOUT 800
#define SHAKE_THRESHOLD 2200

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

#define DISPLAY_HEIGHT 300
#define DISPLAY_WIDTH 400
#define BUFFER_SIZE (DISPLAY_WIDTH * DISPLAY_HEIGHT / 8)

enum HandlingMode {
  MODE_SET_CLOCK,
  MODE_SET_ALARM,
  MODE_DRAWING,
};

enum AlarmMode {
  ALARM_UNSET,
  ALARM_SET,
  ALARM_ACTIVE,
  ALARM_SNOOZED,
};

struct EncoderEvent {
    bool isEncoderA;
    int delta;
};

AlarmMode alarmMode = ALARM_UNSET;
DateTime currentAlarm = DateTime(0, 0, 0, 0, 0, 0);
int snoozeAmount = 10; // Snooze amount in minutes
int snoozeCount = 0;


uint8_t drawingBuffer[BUFFER_SIZE] = {0};
int minX = 0;
int maxX = DISPLAY_WIDTH - 1;
int minY = 0;
int maxY = DISPLAY_HEIGHT - 1;

uint16_t mouseX = DISPLAY_WIDTH / 2;
uint16_t mouseY = DISPLAY_HEIGHT / 2;

TaskHandle_t BeepTaskHandle = NULL;

void triggerAlarm(void *parameter) {
  alarmMode = AlarmMode::ALARM_ACTIVE;
  while (alarmMode == AlarmMode::ALARM_ACTIVE)
  {
    tone(BUZZER_PIN, 1500, 1000);
    vTaskDelay(pdMS_TO_TICKS(2000));

    if (alarmMode != AlarmMode::ALARM_ACTIVE)
    {
      break;
    }

    tone(BUZZER_PIN, 1500, 1000);
    vTaskDelay(pdMS_TO_TICKS(2000));

    if (alarmMode != AlarmMode::ALARM_ACTIVE)
    {
      break;
    }

    tone(BUZZER_PIN, 1500, 1000);
    vTaskDelay(pdMS_TO_TICKS(2000));
  }

  BeepTaskHandle = NULL;
  vTaskDelete(NULL);
}

unsigned long nextUpdate = 0;

void markDirty(int delay)
{
  if (nextUpdate == -1)
  {
    nextUpdate = millis() + delay;
  }
}

int clampInt(int x, int low, int high) {
  if (x < low) return low;
  if (x > high) return high;
  return x;
}

void addDrawingBoundingPoint(int x, int y)
{
  // if all changes are already rendered then we should set the initial bounding box to the first drawing point
  minX = min(minX, x);
  maxX = max(maxX, x);
  minY = min(minY, y);
  maxY = max(maxY, y);
  
  markDirty(100);
}

#define ENC_BUFFER_SIZE 64

class EncoderRingBuffer {
public:
  EncoderEvent buffer[ENC_BUFFER_SIZE];
  size_t head = 0;
  size_t tail = 0;
  bool isFull = false;

  bool push(const EncoderEvent& event) {
    if (isFull) return false;

    buffer[head] = event;
    head = (head + 1) % ENC_BUFFER_SIZE;
    isFull = (head == tail);
    return true;
  }

  bool pop(EncoderEvent& event) {
    if (isEmpty()) return false;

    event = buffer[tail];
    tail = (tail + 1) % ENC_BUFFER_SIZE;
    isFull = false;
    return true;
  }

  bool peek(EncoderEvent &event) const {
    if (isEmpty()) return false;
    event = buffer[tail];
    return true;
  }
  

  bool isEmpty() const {
    return (!isFull && head == tail);
  }

  size_t size() const {
    if (isFull) return ENC_BUFFER_SIZE;
    if (head >= tail) return head - tail;
    return ENC_BUFFER_SIZE - tail + head;
  }

  void clear() {
    head = tail = 0;
    isFull = false;
  }
};

EncoderRingBuffer eventBuffer;

void onEncoderTurn(bool isA, int delta) {
  EncoderEvent e = { isA, delta };
  eventBuffer.push(e);
}

// HandlingMode handlingMode = HandlingMode::MODE_DRAWING;
HandlingMode handlingMode = HandlingMode::MODE_DRAWING;

void drawBitmapToBuffer(uint8_t *buffer, int bufWidth, int bufHeight, const uint8_t *bitmap, int x, int y, int w, int h)
{
  for (int row = 0; row < h; row++)
  {
    for (int col = 0; col < w; col++)
    {
      int byteIndex = (row * ((w + 7) / 8)) + (col / 8);
      int bitIndex = 7 - (col % 8); // MSB first
      bool pixel = (bitmap[byteIndex] >> bitIndex) & 0x01;

      int targetX = x + col;
      int targetY = y + row;

      if (targetX >= 0 && targetX < bufWidth && targetY >= 0 && targetY < bufHeight)
      {
        int bufByteIndex = (targetY * bufWidth + targetX) / 8;
        int bufBitIndex = 7 - (targetX % 8);
        uint8_t bitMask = (1 << bufBitIndex);

        if (pixel)
        {
          // Bitmap pixel is black -> OR in the bit
          buffer[bufByteIndex] |= bitMask;
        }
        else
        {
          // Bitmap pixel is white -> do nothing (OR with 0 leaves as-is)
        }
      }
    }
  }
  addDrawingBoundingPoint(x, y);
  addDrawingBoundingPoint(x + w - 1, y + h - 1);
}



void clearFrameBuffer(uint8_t *buffer, int bufWidth, int bufHeight)
{
  memset(buffer, 0xFF, (bufWidth * bufHeight) / 8); // Clear the buffer to white
}

GxEPD2_BW<GxEPD2_420_GYE042A87, GxEPD2_420_GYE042A87::HEIGHT> display(GxEPD2_420_GYE042A87(/*CS=5*/ SS, /*DC=*/17, /*RST=*/16, /*BUSY=*/4)); // 400x300

RTC_DS3231 rtc;

// Create encoder objects
AiEsp32RotaryEncoder rotaryEncoder1(ROTARY1_A_PIN, ROTARY1_B_PIN, ROTARY1_BUTTON_PIN, ROTARY1_VCC_PIN, ROTARY1_STEPS);
AiEsp32RotaryEncoder rotaryEncoder2(ROTARY2_A_PIN, ROTARY2_B_PIN, ROTARY2_BUTTON_PIN, ROTARY2_VCC_PIN, ROTARY2_STEPS);

void IRAM_ATTR readEncoderISR1()
{
  long oldValue = rotaryEncoder1.readEncoder();
  rotaryEncoder1.readEncoder_ISR();
  if (rotaryEncoder1.encoderChanged()) {
    long newValue = rotaryEncoder1.readEncoder();
    long delta = newValue - oldValue;
    onEncoderTurn(true, delta);
  }
}

void IRAM_ATTR readEncoderISR2()
{
  long oldValue = rotaryEncoder2.readEncoder();
  rotaryEncoder2.readEncoder_ISR();
  if (rotaryEncoder2.encoderChanged()) {
    long newValue = rotaryEncoder2.readEncoder();
    long delta = newValue - oldValue;
    onEncoderTurn(false, delta);
  }
}

bool setClockMode = true;

DateTime clockTime(__DATE__, __TIME__);

void updateTime(int dHours, int dMinutes)
{
  DateTime now = rtc.now();
  int newHour = (now.hour() + dHours) % 24;
  if (newHour < 0)
    newHour += 24;

  int newMinute = (now.minute() + dMinutes) % 60;
  if (newMinute < 0)
    newMinute += 60;

  rtc.adjust(DateTime(now.year(), now.month(), now.day(), newHour, newMinute, now.second()));
  Serial.printf("RTC updated to: %02d:%02d\n", newHour, newMinute);
  clockTime = rtc.now();
  // clockNeedsUpdate = true;
  markDirty(0);
}

void updateAlarm(int dHours, int dMinutes)
{
  currentAlarm = currentAlarm + TimeSpan(0, dHours, dMinutes, 0);

  alarmMode = AlarmMode::ALARM_SET;

  Serial.printf("Alarm updated to: %02d:%02d\n", currentAlarm.hour(), currentAlarm.minute());
  markDirty(0);
}

void formatTime12Hour(int hour24, int minute, char* outBuffer, size_t bufferSize) {
  // Constrain inputs
  hour24 = constrain(hour24, 0, 23);
  minute = constrain(minute, 0, 59);

  // Convert to 12-hour format
  int hour12;
  if (hour24 == 0) {
    hour12 = 12;
  } else if (hour24 > 12) {
    hour12 = hour24 - 12;
  } else {
    hour12 = hour24;
  }

  // Write to buffer, ensure null-termination
  snprintf(outBuffer, bufferSize, "%d:%02d", hour12, minute);
}

void drawPixel(int x, int y) {
  if (x < 0 || x >= DISPLAY_WIDTH || y < 0 || y >= DISPLAY_HEIGHT) return;
  int byteIndex = (y * DISPLAY_WIDTH + x) / 8;
  int bitIndex = 7 - (x % 8);
  drawingBuffer[byteIndex] |= (1 << bitIndex);
  addDrawingBoundingPoint(x, y);
}

void clearPixel(int x, int y) {
  if (x < 0 || x >= DISPLAY_WIDTH || y < 0 || y >= DISPLAY_HEIGHT) return;
  int byteIndex = (y * DISPLAY_WIDTH + x) / 8;
  int bitIndex = 7 - (x % 8);
  drawingBuffer[byteIndex] &= ~(1 << bitIndex); // Clear the bit
  addDrawingBoundingPoint(x, y);
}

void drawLine(int x0, int y0, int x1, int y1) {
  int dx = abs(x1 - x0);
  int dy = abs(y1 - y0);
  int sx = x0 < x1 ? 1 : -1;
  int sy = y0 < y1 ? 1 : -1;
  int err = dx - dy;

  while (true) {
    drawPixel(x0, y0);
    if (x0 == x1 && y0 == y1) break;
    int e2 = 2 * err;
    if (e2 > -dy) {
      err -= dy;
      x0 += sx;
    }
    if (e2 < dx) {
      err += dx;
      y0 += sy;
    }
  }

  clearPixel(mouseX, mouseY);
}

void checkEncoders()
{
  while (!eventBuffer.isEmpty())
  {
    if (handlingMode == HandlingMode::MODE_DRAWING)
    {
      EncoderEvent event;
      bool success = eventBuffer.pop(event);
      if (success)
      {
        int oldMouseX = mouseX;
        int oldMouseY = mouseY;

        if (event.isEncoderA) {
          mouseX = clampInt(mouseX + event.delta, 0, DISPLAY_WIDTH - 1);
        } else {
          mouseY = clampInt(mouseY - event.delta, 0, DISPLAY_HEIGHT - 1);
        }

        if (mouseX != oldMouseX || mouseY != oldMouseY)
        {
          drawLine(oldMouseX, oldMouseY, mouseX, mouseY);
        }
      }
    }
    else if (handlingMode == HandlingMode::MODE_SET_CLOCK)
    {
      EncoderEvent event;
      bool success = eventBuffer.pop(event);
      if (success)
      {
        if (event.isEncoderA) {
          updateTime(event.delta, 0);
        } else {
          updateTime(0, event.delta);
        }
      }
    }
    else if (handlingMode == HandlingMode::MODE_SET_ALARM)
    {
      EncoderEvent event;
      bool success = eventBuffer.pop(event);
      if (success)
      {
        if (event.isEncoderA) {
          updateAlarm(event.delta, 0);
        } else {
          updateAlarm(0, event.delta);
        }
      }
    }
  }
}


void IRAM_ATTR onAlarm() {
  shouldCheckAlarm = true;
}

volatile int shakeCount = 0;
unsigned long shakeStartTime = 0;

void IRAM_ATTR shakeISR() {
  shakeCount++;
}

void setup()
{

  Serial.begin(115200);
  Wire.begin();

  pinMode(BUZZER_PIN, OUTPUT);

  pinMode(SHAKE_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(SHAKE_PIN), shakeISR, FALLING);


  rtc.begin();

  rtc.disableAlarm(1);
  rtc.disableAlarm(2);
  rtc.clearAlarm(1);
  rtc.clearAlarm(2);

  rtc.writeSqwPinMode(DS3231_OFF); // Disable SQW if not used

  rtc.setAlarm2(
    DateTime(0, 0, 0, 0, 0, 0),
    DS3231_A2_PerMinute
  );

  pinMode(RTC_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RTC_INT_PIN), onAlarm, FALLING);

  clockTime = rtc.now();
  currentAlarm = rtc.now();
  updateAlarm(0, 1);

  if (rtc.lostPower())
  {
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
  display.setTextColor(GxEPD_BLACK);
  display.setFont(&FreeMonoBold18pt7b);
  display.setTextSize(1);

  drawBitmapToBuffer(drawingBuffer, DISPLAY_WIDTH, DISPLAY_HEIGHT, giraffe_image_bytes, 0, 0, 400, 300);
}

// unsigned long lastUpdate = 0;

void renderScreen()
{

  display.setPartialWindow(minX, minY, maxX - minX + 1, maxY - minY + 1);

  display.firstPage();
  do {
    display.drawBitmap(minX, minY, drawingBuffer, 400, 300, GxEPD_BLACK, GxEPD_WHITE);

    DateTime now = rtc.now();
    int hours = now.hour();
    int minutes = now.minute();
    char timeStr[6];
    formatTime12Hour(hours, minutes, timeStr, sizeof(timeStr));

    display.setFont(&FreeMonoBold18pt7b);
    display.setCursor(40, 58); // 40 is X, 58 is Y baseline
    display.print(timeStr);

    // Measure width of time string to find where to place "AM"/"PM"
    int16_t x1, y1;
    uint16_t w, h;
    display.getTextBounds(timeStr, 40, 58, &x1, &y1, &w, &h);

    // Use a smaller font for AM/PM
    display.setFont(&FreeMonoBold9pt7b); // or another small font
    display.setCursor(40 + w + 8, 50); // Position AM/PM right of time
    display.print((hours < 12) ? "AM" : "PM");

    // Draw alarm time below in non-bold font
    char alarmStr[16];
    DateTime currentAlarmPlusSnooze = currentAlarm + TimeSpan(0, 0, snoozeAmount * snoozeCount, 0);
    // int alarmHours = currentAlarm.hour();
    // int alarmMinutes = currentAlarm.minute();
    int alarmHours = currentAlarmPlusSnooze.hour();
    int alarmMinutes = currentAlarmPlusSnooze.minute();
    char alarmTime[6];
    formatTime12Hour(alarmHours, alarmMinutes, alarmTime, sizeof(alarmTime));

    snprintf(alarmStr, sizeof(alarmStr), "%s %s", alarmTime, (alarmHours < 12) ? "AM" : "PM");

    display.setFont(&FreeMono9pt7b); // Non-bold font
    display.setCursor(40, 75); // Slightly below current time
    display.print(alarmStr);

    display.setCursor(300, 20);
    if (handlingMode == HandlingMode::MODE_SET_CLOCK)
    {
      display.print("Set Clock");
    }
    else if (handlingMode == HandlingMode::MODE_SET_ALARM)
    {
      display.print("Set Alarm");
    }

  } while (display.nextPage());
}

#include <stdlib.h>  // Required for random()

void handleShake()
{
  // print num shakes:
  // Serial.printf("Shake detected! Count: %d\n", shakeCount);
  if (handlingMode == HandlingMode::MODE_DRAWING)
  {
    int totalPixels = DISPLAY_WIDTH * DISPLAY_HEIGHT;
    int pixelsToClear = (int)(totalPixels * 2.5);

    for (int i = 0; i < pixelsToClear; i++)
    {
      int x = random(0, DISPLAY_WIDTH);
      int y = random(0, DISPLAY_HEIGHT);
      clearPixel(x, y);
    }

    markDirty(0);
  }

  if (alarmMode == AlarmMode::ALARM_ACTIVE)
  {
    Serial.println("Alarm active. Shake turns it off!");
    alarmMode = AlarmMode::ALARM_SNOOZED;
    snoozeCount += 1;
    markDirty(0);
  }
}


void checkShake()
{
  if (shakeCount > 0)
  {
    if (shakeStartTime == 0)
    {
      shakeStartTime = millis();
    }

    if (millis() - shakeStartTime > SHAKE_TIMEOUT)
    {
      if (shakeCount >= SHAKE_THRESHOLD)
      {
        handleShake();
      }
      // else
      // {
      //   Serial.println("Shake not strong enough.");
      // }

      shakeCount = 0;
      shakeStartTime = 0;
    }
  }
}

void checkAlarm()
{
  if (shouldCheckAlarm)
  {
    markDirty(0);
    rtc.clearAlarm(2);
    Serial.println("Checking alarm...");
    if (alarmMode == AlarmMode::ALARM_SET || alarmMode == AlarmMode::ALARM_SNOOZED)
    {
      DateTime now = rtc.now();
      DateTime currentAlarmPlusSnooze = currentAlarm + TimeSpan(0, 0, snoozeAmount * snoozeCount, 0);
      int nowMinutes = now.hour() * 60 + now.minute();
      int currentAlarmMinutes = currentAlarmPlusSnooze.hour() * 60 + currentAlarmPlusSnooze.minute();

      int difference = nowMinutes - currentAlarmMinutes;
      if (difference >= 0 && difference < 2)
      {
        Serial.println("Alarm time reached! Should beep!");
        xTaskCreatePinnedToCore(
          triggerAlarm,          // Task function
          "BeepTask",       // Name of the task
          2048,             // Stack size (in bytes)
          NULL,             // Task input parameter
          1,                // Priority of the task
          &BeepTaskHandle,  // Task handle
          0                 // Core where the task should run (0 for core 0)
        );
      }
    }
  }
  shouldCheckAlarm = false;
}

void loop()
{
  checkAlarm();

  checkShake();

  checkEncoders();

  unsigned long currentMillis = millis();

  if (nextUpdate != -1 && currentMillis >= nextUpdate)
  {
    renderScreen();
    nextUpdate = -1;
  }
}