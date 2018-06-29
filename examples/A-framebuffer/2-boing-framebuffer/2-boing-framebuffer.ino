// 'Boing' ball example for TFT_framebuffer.  This is a little more involved
// than the boxes demo, bypassing the basic drawing primitives and modifying
// the framebuffer directly.

#include <Adafruit_TFTDMA.h>
#include "graphics.h" // Background and ball data

// TFT hardware config:
#define TC     2       // Timer/counter index
#define RESET A4       // Reset pin (-1 if not used)
#define CS    -1       // Chip-select pin (-1 if not used -- tie to GND)
#define CD    A2       // Command/data pin
#define RD    A0       // Read-strobe pin
#define WR    A3       // Write-strobe pin (CCL-modified output from timer)
#define D0     0       // Data bit 0 pin (MUST be on PORT byte boundary)
#define PERIPH PIO_CCL // Peripheral type of WR pin (PIO_CCL, PIO_TIMER, etc.)

TFT_framebuffer tft(TC, RESET, CS, CD, RD, WR, D0, PERIPH);

// Each of the colors used in this code is defined twice: 'normal' order if
// using the ILI9341 16-bit parallel interface, and byte-swapped otherwise.
// In 8-bit mode, the ILI9341 expects colors most-significant-byte first
// (big-endian), but the SAMD is little-endian and DMA transfers are served
// straight from RAM.  The drawing functions in Adafruit_TFTDMA will perform
// a byte swap as needed, but this demo is modifying the RAM framebuffer
// directly and must provide its own swap.
#if TFT_INTERFACE == TFT_INTERFACE_16
  #define BGCOLOR    0xAD75
  #define GRIDCOLOR  0xA815
  #define BGSHADOW   0x5285
  #define GRIDSHADOW 0x600C
  #define RED        0xF800
  #define WHITE      0xFFFF
#else
  #define BGCOLOR    0x75AD
  #define GRIDCOLOR  0x15A8
  #define BGSHADOW   0x8552
  #define GRIDSHADOW 0x0C60
  #define RED        0x00F8
  #define WHITE      0xFFFF
#endif

uint16_t *framebuffer; // Pointer to TFT_framebuffer's pixel data

#define YBOUNCE -2.5   // Upward velocity on ball bounce
#define YBOTTOM 123    // Ball Y coord at bottom

// Ball coordinates are stored floating-point because screen refresh
// is so quick, whole-pixel movements are just too fast!
float ballx     = 20.0,  bally    = 123.0,   // Current ball position
      ballvx    = 0.55,  ballvy   = YBOUNCE, // Ball velocity
      ballframe = 3;                         // Ball animation frame #
int   balloldx  = ballx, balloldy = bally;   // Prior ball position

uint16_t palette[16]; // Color table for ball rotation effect

uint32_t startTime, frame = 0; // For frames-per-second estimate

void setup() {
  Serial.begin(9600);
  //while(!Serial);

  // Initialize TFT hardware.  If this fails, it's usually a problem
  // with the constructor, like an invalid timer number or data pin.
  if(tft.begin()) {
    Serial.println("Init fail");
    pinMode(LED_BUILTIN, OUTPUT);
    for(bool i=true;;i=!i) {
      digitalWrite(LED_BUILTIN, i);
      delay(100);
    }
  }
  Serial.println("Init OK!");

  // Save address of RAM framebuffer for later use.
  framebuffer = tft.getBuffer();

  // Draw initial framebuffer contents:
  drawBackground(0, 0, TFTWIDTH - 1, TFTHEIGHT - 1);

  startTime = millis();
}

// This function draws (or redraws) a section of the framebuffer using the
// background bitmap.  It is VERY specific to this program and NOT a
// general-purpose bitmap renderer!  Bitmap data matches the screen
// dimensions exactly, there is no accommodation for offsets, clipping, etc.
void drawBackground(int16_t x1, int16_t y1, int16_t x2, int16_t y2) {
  const uint8_t *srcPtr;
  uint16_t      *dstPtr;
  uint16_t       bitmap, mask, x, w = x2 - x1 + 1;
  for(; y1 <= y2; y1++) {              // For each scanline...
    srcPtr = &background[y1][x1 / 8];  // Initial pointer into source bitmap
    bitmap = *srcPtr++;                // Read first byte
    mask   = 0x80 >> (x1 & 7);         // Get bitmask for initial column
    dstPtr = &framebuffer[y1 * TFTWIDTH + x1]; // Initial -> into framebuffer
    for(x=0; x<w; x++) {               // For each column...
      *dstPtr++ = (bitmap & mask) ? GRIDCOLOR : BGCOLOR; // Set pixel
      if(!(mask >>= 1)) {              // Advance bitmask 1 pixel. Past bit 0?
        mask   = 0x80;                 // Reset bitmask to bit 7
        bitmap = *srcPtr++;            // Read next byte from bitmap
      }
    }
  }
}

// This function renders the ball atop the current framebuffer contents.
// The previous ball must first be erased using drawBackground()...there is
// nothing clever or optimal here, it's a brute-force approach to keep the
// code relatively simple.  Like the prior function, this is very specific
// to this one program and not a general-purpose blitter.
void drawBall(int16_t x1, int16_t y1) {
  const uint8_t *srcPtr = &ball[0][0];
  uint16_t      *dstPtr = &framebuffer[y1 * TFTWIDTH + x1];
  uint8_t        x, y, c;
  uint16_t foo;

  // Ball data is 2 pixels per byte -- even pixels are the 4 high bits
  // of each byte, odd pixels are the low 4 bits.  These are run through
  // a color palette to determine the 16-bit framebuffer color.
  for(y=0; y<BALLHEIGHT; y++) {    // For each scanline...
    for(x=0; x<BALLWIDTH/2; x++) { // For each pair of pixels...
      // Pixel is skipped (transparent) if value is 0.
      // Value 1 is in shadow, other values use palette lookup.
      if(c = (srcPtr[x] >> 4)) {   // Even pixel, opaque?
        *dstPtr = (c > 1) ? palette[c] :                // Palette
          (*dstPtr == BGCOLOR) ? BGSHADOW : GRIDSHADOW; // Shadow
      }
      dstPtr++;
      if(c = (srcPtr[x] & 0xF)) {  // Odd pixel, opaque?
        *dstPtr = (c > 1) ? palette[c] :                // Palette
          (*dstPtr == BGCOLOR) ? BGSHADOW : GRIDSHADOW; // Shadow
      }
      dstPtr++;
    }
    srcPtr += BALLWIDTH / 2;
    dstPtr += TFTWIDTH - BALLWIDTH;
  }
}

void loop() {

  // Prior DMA transfer must complete before modifying framebuffer.
  // We can do stuff in the meantime, such as calculate the next ball
  // position (but don't draw it yet):
  balloldx = (int16_t)ballx; // Save prior position
  balloldy = (int16_t)bally;
  ballx   += ballvx;         // Update position
  bally   += ballvy;
  ballvy  += 0.031;          // Update Y velocity
  if((ballx <= 15) || (ballx >= TFTWIDTH - BALLWIDTH))
    ballvx *= -1;            // Left/right bounce
  if(bally >= YBOTTOM) {     // Hit ground?
    bally  = YBOTTOM;        // Clip and
    ballvy = YBOUNCE;        // bounce up
  }

  // Determine screen area to update.  This is the bounds of the ball's
  // prior and current positions, so the old ball is fully erased and new
  // ball is fully drawn.
  uint16_t minx, miny, maxx, maxy;
  if(frame == 0) {   // If first frame...
    minx = miny = 0; // Update the whole screen
    maxx = TFTWIDTH  - 1;
    maxy = TFTHEIGHT - 1;
  } else { // Determine bounds of prior and new positions
    minx = ballx;
    if(balloldx < minx)                    minx = balloldx;
    miny = bally;
    if(balloldy < miny)                    miny = balloldy;
    maxx = ballx + BALLWIDTH  - 1;
    if((balloldx + BALLWIDTH  - 1) > maxx) maxx = balloldx + BALLWIDTH  - 1;
    maxy = bally + BALLHEIGHT - 1;
    if((balloldy + BALLHEIGHT - 1) > maxy) maxy = balloldy + BALLHEIGHT - 1;
  }

  // Ball animation frame # is incremented opposite the ball's X velocity
  ballframe -= ballvx * 0.3;
  if(ballframe < 0)        ballframe += 14; // Constrain from 0 to 13
  else if(ballframe >= 14) ballframe -= 14;

  // This function blocks until it's safe to modify the framebuffer:
  tft.waitForUpdate();

  // Set 7 palette entries to white, 7 to red, based on frame number.
  // This makes the ball spin
  for(uint8_t i=0; i<14; i++) {
    palette[i+2] = ((((int)ballframe + i) % 14) < 7) ? WHITE : RED;
    // Palette entries 0 and 1 aren't used (clear and shadow, respectively)
  }

  // Erase old ball (redraw background):
  drawBackground(balloldx, balloldy,
    balloldx + BALLWIDTH - 1, balloldy + BALLHEIGHT - 1);

  // Draw new ball:
  drawBall(ballx, bally);

  // Mark the area of the framebuffer we've modified so
  // TFT_framebuffer is aware and refreshes the correct region:
  tft.sully(minx, miny);
  tft.sully(maxx, maxy);

  // Transfer the changed area to the screen. This returns almost
  // immediately and the data transfer proceeds in the background:
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

