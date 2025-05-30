#include "Arduino.h"
#include "AiEsp32RotaryEncoder.h"
#include "RTClib.h"
#include <GxEPD2_BW.h>
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeMonoBold18pt7b.h>
#include "line_art/face.h"
#include "line_art/giraffe.h"

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

uint16_t mouseX = DISPLAY_WIDTH / 2;
uint16_t mouseY = DISPLAY_HEIGHT / 2;

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
  if (newHour < 0)
    newHour += 24;

  int newMinute = (now.minute() + dMinutes) % 60;
  if (newMinute < 0)
    newMinute += 60;

  rtc.adjust(DateTime(now.year(), now.month(), now.day(), newHour, newMinute, now.second()));
  Serial.printf("RTC updated to: %02d:%02d\n", newHour, newMinute);
  clockTime = rtc.now();
}

void drawSimplePixel(int x, int y) {
  if (x < 0 || x >= DISPLAY_WIDTH || y < 0 || y >= DISPLAY_HEIGHT) return;
  int byteIndex = (y * DISPLAY_WIDTH + x) / 8;
  int bitIndex = 7 - (x % 8);
  drawingBuffer[byteIndex] |= (1 << bitIndex);
}

void drawPixel(int x, int y) {
  if (x < 0 || x >= DISPLAY_WIDTH || y < 0 || y >= DISPLAY_HEIGHT) return;
  int byteIndex = (y * DISPLAY_WIDTH + x) / 8;
  int bitIndex = 7 - (x % 8);
  drawingBuffer[byteIndex] |= (1 << bitIndex);
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
}




void addDrawing(int dX, int dY)
{
  // Draw a pixel at the current mouse position
  int x = clampInt(mouseX + dX, 0, DISPLAY_WIDTH - 1);
  int y = clampInt(mouseY + dY, 0, DISPLAY_HEIGHT - 1);

  int byteIndex = (y * DISPLAY_WIDTH + x) / 8;
  int bitIndex = 7 - (x % 8); // MSB first
  drawingBuffer[byteIndex] |= (1 << bitIndex);
}

void checkEncoders()
{
  if (rotaryEncoder1.encoderChanged())
  {
    const long oldVal = encoder1Val;
    encoder1Val = rotaryEncoder1.readEncoder();
    const long delta = encoder1Val - oldVal;

    Serial.printf("Encoder 1 value: %d | delta: %d \n", encoder1Val, delta);

    if (currentMode == HandlingMode::MODE_DRAWING)
    {
      int oldX = mouseX;
      mouseX = clampInt(mouseX + delta, 0, DISPLAY_WIDTH - 1);
      drawLine(oldX, mouseY, mouseX, mouseY);
    }

    updateTime(delta, 0);
  }

  if (rotaryEncoder2.encoderChanged())
  {
    const long oldVal = encoder2Val;
    encoder2Val = rotaryEncoder2.readEncoder();
    const long delta = encoder2Val - oldVal;

    Serial.printf("Encoder 2 value: %d | delta: %d \n", encoder2Val, delta);

    if (currentMode == HandlingMode::MODE_DRAWING)
    {
      int oldY = mouseY;
      mouseY = clampInt(mouseY + delta, 0, DISPLAY_HEIGHT - 1);
      drawLine(mouseX, oldY, mouseX, mouseY);
    }

    updateTime(0, delta);
  }
}


void setup()
{
  Serial.begin(115200);

  rtc.begin();

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
  display.setRotation(0);
  display.setTextColor(GxEPD_BLACK);
  display.setFont(&FreeMonoBold18pt7b);
  display.setTextSize(1);
  display.setFullWindow();
  display.firstPage();
  do
  {
    display.fillScreen(GxEPD_WHITE);
  } while (display.nextPage());
}

unsigned long lastUpdate = 0;

// void renderScreen()
// {
//   // Get current RTC time
//   DateTime now = rtc.now();
//   int hours = now.hour();
//   int minutes = now.minute();

//   // Format HH:MM
//   char timeStr[6];
//   snprintf(timeStr, sizeof(timeStr), "%02d:%02d", hours, minutes);

//   // ---- RENDER IMAGE TOP-RIGHT ----
//   const int imgX = 400 - 300; // assuming 200x200 image
//   const int imgY = 0;
//   const int imgW = 300;
//   const int imgH = 300;

//   display.setPartialWindow(imgX, imgY, imgW, imgH);
//   display.firstPage();
//   do
//   {
//     display.drawBitmap(imgX, imgY, epd_bitmap_lineart_removebg_preview, imgW, imgH, GxEPD_BLACK, GxEPD_WHITE);
//   } while (display.nextPage());

//   // ---- RENDER CLOCK BOTTOM-LEFT ----
//   display.setFont(&FreeMonoBold18pt7b);
//   int16_t x1, y1;
//   uint16_t w, h;
//   display.getTextBounds(timeStr, 0, 0, &x1, &y1, &w, &h);

//   const int16_t pad = 10;
//   const int clockX = pad;
//   const int clockY = 300 - h - pad;
//   const int clockW = w + pad * 2;
//   const int clockH = h + pad * 2;

//   display.setPartialWindow(clockX, clockY, clockW, clockH);
//   display.firstPage();
//   do
//   {
//     display.fillRect(clockX, clockY, clockW, clockH, GxEPD_WHITE);
//     display.setCursor(clockX + pad, clockY + h); // adjust Y for baseline
//     display.print(timeStr);
//   } while (display.nextPage());
// }

void renderScreen()
{
  drawBitmapToBuffer(drawingBuffer, DISPLAY_WIDTH, DISPLAY_HEIGHT, face_image_bytes, 0, 0, 300, 300);

  display.firstPage();
  display.setPartialWindow(0, 0, DISPLAY_WIDTH, DISPLAY_HEIGHT);
  display.drawBitmap(0, 0, drawingBuffer, 400, 300, GxEPD_BLACK, GxEPD_WHITE);
  display.nextPage();

}

void loop()
{
  checkEncoders();
  delay(50); // For encoder responsiveness

  unsigned long currentMillis = millis();

  if (currentMillis - lastUpdate >= 1000)
  {
    lastUpdate = currentMillis;

    renderScreen();
  }
}