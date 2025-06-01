#include "Arduino.h"
#include <Wire.h>
#include "AiEsp32RotaryEncoder.h"
#include "RTClib.h"
#include <GxEPD2_BW.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeMonoBold18pt7b.h>
#include "images.h"

struct EncoderEvent {
    bool isEncoderA;
    int delta;
};

#define RTC_INT_PIN 26

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

uint8_t drawingBuffer[BUFFER_SIZE] = {0};
int minX = 0;
int maxX = DISPLAY_WIDTH - 1;
int minY = 0;
int maxY = DISPLAY_HEIGHT - 1;
bool changesRendered = true;

uint16_t mouseX = DISPLAY_WIDTH / 2;
uint16_t mouseY = DISPLAY_HEIGHT / 2;

void addDrawingBoundingPoint(int x, int y)
{
  minX = min(minX, x);
  maxX = max(maxX, x);
  minY = min(minY, y);
  maxY = max(maxY, y);
  changesRendered = false;
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

int clampInt(int x, int low, int high) {
  if (x < low) return low;
  if (x > high) return high;
  return x;
}

enum HandlingMode {
  MODE_SET_CLOCK,
  MODE_DRAWING,
};

HandlingMode currentMode = MODE_DRAWING;

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

int encoder1Val = 0;
int encoder2Val = 0;
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
    if (currentMode == HandlingMode::MODE_DRAWING)
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
  }
}

volatile bool alarmTriggered = true;

void onAlarm() {
  alarmTriggered = true;
}

void setup()
{

  Serial.begin(115200);
  Wire.begin();

  uint32_t psramSize = ESP.getPsramSize();
  Serial.printf("PSRAM Size: %u bytes (%.2f KB)\n", psramSize, psramSize / 1024.0);

  if (psramSize == 0) {
    Serial.println("No PSRAM detected.");
  } else {
    Serial.println("PSRAM detected and ready.");
  }


  rtc.begin();

  rtc.clearAlarm(1);
  rtc.disableAlarm(1);
  rtc.writeSqwPinMode(DS3231_OFF); // Disable SQW if not used

  rtc.setAlarm1(
    rtc.now(),
    DS3231_A1_Second
  );

  rtc.clearAlarm(1);

  pinMode(RTC_INT_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RTC_INT_PIN), onAlarm, FALLING);



  clockTime = rtc.now();

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

unsigned long lastUpdate = 0;

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
    snprintf(timeStr, sizeof(timeStr), "%02d:%02d", hours, minutes);
    display.setFont(&FreeMonoBold18pt7b);
    display.setCursor(40, 40 + 18); // Adjust Y for baseline
    display.print(timeStr);

  } while (display.nextPage());

  changesRendered = true;
}


void loop()
{
  checkEncoders();

  unsigned long currentMillis = millis();

  bool updateFromDrawing = !changesRendered && currentMillis - lastUpdate >= 100;
  bool updateFromClock = alarmTriggered;
  if (updateFromDrawing || updateFromClock)
  {
    lastUpdate = currentMillis;

    renderScreen();
  }
}