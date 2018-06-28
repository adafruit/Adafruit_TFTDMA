// Example for the Adafruit_TFTDMA TFT_framebuffer object.
// This is a huge memory hog because the entire screen is buffered in RAM,
// but it's easiest to comprehend & use.

#include <Adafruit_TFTDMA.h>
#include "wiring_private.h"
#include "graphics.h"

#define TC     2       // Timer/counter index
#define RESET -1       // Reset pin (-1 if not used)
#define CS    -1       // Chip-select pin (-1 if not used -- tie to GND)
#define CD    A2       // Command/data pin
#define RD    A0       // Read-strobe pin
#define WR    A3       // Write-strobe pin (CCL-modified output from timer)
#define D0     0       // Data bit 0 pin (MUST be on PORT byte boundary)
#define PERIPH PIO_CCL // Peripheral type of WR pin (PIO_CCL, PIO_TIMER, etc.)

TFT_framebuffer tft(TC, RESET, CS, CD, RD, WR, D0, PERIPH);

uint32_t startTime, frame = 0;

#define YBOUNCE -2.5

float ballx=20, bally=123, ballvx=0.53, ballvy=YBOUNCE;
int   balloldx = ballx, balloldy = bally;
float ballframe = 3;
uint16_t *buf;

// Ball is 113x98
// background is (170,170,170)
// 0xAA
// 1010 1101 0111 0101  0xAD75
// 1010 1000 0001 0101  0xA815
// shadow is (102,102,102) 0x66
// grid shadow is (102,0,102)
// 0101 0010 1000 1010 0x5285
// 0110 0000 0000 1100 0x600C

#define BGCOLOR    0xAD75
#define GRIDCOLOR  0xA815
#define BGSHADOW   0x5285
#define GRIDSHADOW 0x600C

void blit2(
  uint8_t *src, // Source bitmap data
  int      sw,  // Source bitmap width
  int      sh,  // Source bitmap height
  int      sx1, // Left coord in source bitmap
  int      sy1, // Top coord in source bitmap
  int      dx1, // Left coord on screen
  int      dy1, // Top coord on screen
  int      w,   // Width of section to copy
  int      h,   // Height of section to copy
  int      color1,
  int      color2) {

  int       x, y;
  uint16_t *dest;

  src += sy1 * sw + sx1; // Offset to 1st pixel
  dest = &buf[dy1 * TFTWIDTH + dx1];

  color1 = (color1 << 8) | (color1 >> 8);
  color2 = (color2 << 8) | (color2 >> 8);

  for(y=0; y<h; y++) {
    for(x=0; x<w; x++) {
      if(src[x]) dest[x] = color2;
      else       dest[x] = color1;
    }
    src  += sw;
    dest += TFTWIDTH;
  }
}

void blit3(
  uint8_t *src, // Source bitmap data
  int      sw,  // Source bitmap width
  int      sh,  // Source bitmap height
  int      sx1, // Left coord in source bitmap
  int      sy1, // Top coord in source bitmap
  int      dx1, // Left coord on screen
  int      dy1, // Top coord on screen
  int      w,   // Width of section to copy
  int      h,   // Height of section to copy
  int      color1,
  int      color2) {

  int       x, y;
  uint16_t *dest;
  uint8_t   c;

  src += sy1 * sw + sx1; // Offset to 1st pixel
  dest = &buf[dy1 * TFTWIDTH + dx1];

  color1 = (color1 << 8) | (color1 >> 8);
  color2 = (color2 << 8) | (color2 >> 8);

  for(y=0; y<h; y++) {
    for(x=0; x<w; x++) {
      c = src[x];
      if(c == 255) dest[x] = color1;
      else if(c)   dest[x] = color2;
    }
    src  += sw;
    dest += TFTWIDTH;
  }
}

void shade(
  uint8_t *src, // Source bitmap data
  int      sw,  // Source bitmap width
  int      sh,  // Source bitmap height
  int      sx1, // Left coord in source bitmap
  int      sy1, // Top coord in source bitmap
  int      dx1, // Left coord on screen
  int      dy1, // Top coord on screen
  int      w,   // Width of section to copy
  int      h,   // Height of section to copy
  int      color1,
  int      color2) {

  int       x, y;
  uint16_t *dest;
  uint16_t  cmp = (BGCOLOR << 8) | (BGCOLOR >> 8);

  src += sy1 * sw + sx1; // Offset to 1st pixel
  dest = &buf[dy1 * TFTWIDTH + dx1];

  color1 = (color1 << 8) | (color1 >> 8);
  color2 = (color2 << 8) | (color2 >> 8);

  for(y=0; y<h; y++) {
    for(x=0; x<w; x++) {
      if(src[x]) {
        if(dest[x] == cmp) dest[x] = color1;
        else               dest[x] = color2;
      }
    }
    src  += sw;
    dest += TFTWIDTH;
  }
}

void setup() {
  Serial.begin(9600);
//  while(!Serial);

  // DEBUG: for monitoring non-CCL-inverted PWM output:
  //pinMode(MOSI, OUTPUT);
  //pinPeripheral(MOSI, PIO_TIMER); // TC2/WO[0]

  if(tft.begin()) {
    Serial.println("Init fail");
    pinMode(LED_BUILTIN, OUTPUT);
    for(bool i=true;;i=!i) {
      digitalWrite(LED_BUILTIN, i);
      delay(100);
    }
  }
  Serial.println("Init OK!");

  buf = tft.getBuffer();

  tft.update(); // Clear display

  tft.waitForUpdate();
  tft.fillScreen(BGCOLOR);
  blit2((uint8_t *)bg, BGWIDTH, BGHEIGHT,
    0, 0, 12, 12,
    BGWIDTH, BGHEIGHT, BGCOLOR, GRIDCOLOR);
  tft.update();

  startTime = millis();
}

void loop() {
  int i;

  // Prior DMA transfer must complete before modifying framebuffer.
  // We can do stuff in the meantime, such as game logic, or in this
  // case we calculate the next ball position (but don't draw it):
  balloldx = (int)ballx;
  ballx   += ballvx;
  if((ballx <= 13) || (ballx >= 176)) ballvx *= -1;

  balloldy = (int)bally;
  ballvy  += 0.031;
  bally   += ballvy;
  if(bally >= 123) {
    bally  = 123;
    ballvy = YBOUNCE;
  }

  // Determine bounds of prior and new positions
  int minx = (int)ballx;
  if(balloldx < minx)         minx = balloldx;
  int miny = (int)bally - 2; // -2 for shadow Y offset
  if((balloldy -   2) < miny) miny = balloldy - 2;
  int maxx = (int)ballx + 135; // +135 for ball + shadow width
  if((balloldx + 135) > maxx) maxx = balloldx + 135;
  int maxy = (int)bally + 97; // +97 for ball height
  if((balloldy +  97) > maxy) maxy = balloldy + 97;

  ballframe += ballvx * 0.3;
  if(ballframe < 0)        ballframe += 14;
  else if(ballframe >= 14) ballframe -= 14;

  // This function blocks until it's safe to modify the framebuffer...
  tft.waitForUpdate();

  tft.sully(minx, miny);
  tft.sully(maxx, maxy);

  // Erase old ball (redraw background)
  blit2((uint8_t *)bg, BGWIDTH, BGHEIGHT,
    minx - 12, miny - 12, minx, miny,
    maxx - minx + 1, maxy - miny + 1, BGCOLOR, GRIDCOLOR);

  // Bug in graphics file - frame 0 & 1 are swapped
  int b = (int)ballframe;
  if(b == 0) b = 1;
  else if(b == 1) b = 0;

  // Draw new ball
  blit3((uint8_t *)ball, BALLWIDTH, BALLHEIGHT,
    0, (int)b * 98, (int)ballx, (int)bally, BALLWIDTH, 98,
    0xFFFF, 0xF800);

  shade((uint8_t *)shadow, SHADOWWIDTH, SHADOWHEIGHT,
    0, 0, (int)ballx + 62, (int)bally - 2, SHADOWWIDTH, SHADOWHEIGHT,
    BGSHADOW, GRIDSHADOW);

  // Start new screen update in the background
  tft.update();

  // Show approximate frame rate
  if(!(++frame & 255)) { // Every 256 frames...
    uint32_t elapsed = (millis() - startTime) / 1000; // Seconds
    if(elapsed) {
      Serial.print(frame / elapsed);
      Serial.println(" fps");
    }
  }
}

