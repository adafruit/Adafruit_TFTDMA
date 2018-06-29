// 'Wobble tiles' example using the TFT_scanline object, demonstrating how
// it might be used with precomputed raster data in program space or RAM;
// there is NO RENDERING taking place in this code, it just sets up pointers
// for a series of DMA transfers.  This is probably the hardest mode to use
// as it requires considerable planning, but for certain situations can be
// fast and RAM-efficient (about 12K required).

#include <Adafruit_TFTDMA.h>
#include "tile.h" // Graphics data is here

// TFT hardware config:
#define TC     2       // Timer/counter index
#define RESET A4       // Reset pin (-1 if not used)
#define CS    -1       // Chip-select pin (-1 if not used -- tie to GND)
#define CD    A2       // Command/data pin
#define RD    A0       // Read-strobe pin
#define WR    A3       // Write-strobe pin (CCL-modified output from timer)
#define D0     0       // Data bit 0 pin (MUST be on PORT byte boundary)
#define PERIPH PIO_CCL // Peripheral type of WR pin (PIO_CCL, PIO_TIMER, etc.)

TFT_scanline tft(TC, RESET, CS, CD, RD, WR, D0, PERIPH);

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

  startTime = millis();
}

// Scanline counter used for callback function below.  A "good and proper
// computer science" program would probably use the user data pointer to
// the callback, but nobody's looking, I'm just gonna use a global...
uint8_t lineNum;

// This callback function is invoked once per scanline, always top-to-bottom
// and in-order.  Its job is to generate one scanline of graphics, through
// rendering and/or by setting up "spans" -- pointers to and lengths of
// graphics data in RAM or flash.  A single-scanline working buffer
// (TFTWIDTH pixels wide) is provided if needed by the application, as is
// a pointer to any optional user data provided to tft.update() (this
// example is just using a global variable for simplicity).
// The idea is that one scanline is being set up while the prior scanline
// is being transmitted via DMA.  Optimally the times for setup and transmit
// will be equal (about 4,000 CPU cycles/line using 8-bit parallel interface,
// 2,000 cycles/line using 16-bit), but reality will always be a little
// different, and that's OK...non-optimal doesn't necessarily mean pessimal.
// It's almost a "racing the beam" approach, except A) it's an LCD, there is
// no beam, and B) it's not racing, because the line-to-line time isn't
// constant, it's determined by the efficiency/complexity of the callback,
// though there is a lower limit dictated by the DMA transfer speed.  Match
// that and either the transfer or render is like "free" time.
void myCallback(uint16_t *linebuf, void *data) {
  // Figure initial X offset into tile graphics for this scanline.
  // The 'wobble' is a sine wave factoring in the current line and frame
  // with some primes for scaling factors so things are less repetitious.
  uint32_t x = (uint32_t)((sin((float)(lineNum + frame)/41.0)+1.0) * 32.0 +
                     lineNum / 3 + frame / 19);
  int      y = lineNum + frame / 8; // Initial Y offset
  if(y & 32) x += 32; // Alternate every 32 lines for checkerboard effect

  x &= 63; // Graphics tile is 64 pixels wide
  y &= 31; // and 32 pixels tall

  int spanWidth       = 64 - x;   // Width of first span, in pixels
  int pixelsRemaining = TFTWIDTH; // Number of pixels remaining on scanline
  while(pixelsRemaining) {
    // Add a span, originating at tile pixel (x,y), spanWidth pixels wide
    // (address uses [y][x] because of how 2D arrays are expressed in C).
    tft.addSpan((uint16_t *)&tile[y][x], spanWidth);
    pixelsRemaining -= spanWidth; // Subtract span width from count
    x                = 0;         // Subsequent spans always start from 0
    spanWidth        = 64;        // and are 64 pixels wide, unless...
    // Crop the last span to the right edge of the update area.
    // Cumulative span width MUST match the update width, or there will
    // be...trouble.  Library does NOT crop this, expects well-behaved
    // code, because scanline callback time needs to be minimized.
    if(pixelsRemaining < 64) spanWidth = pixelsRemaining;
  }

  lineNum++; // Increment counter for next scanline
}

void loop() {

  // Reset scanline counter and update full screen:
  lineNum = 0;
  tft.update(0, 0, TFTWIDTH-1, TFTHEIGHT-1, myCallback, NULL);

  // Show approximate frame rate
  if(!(++frame & 255)) { // Every 256 frames...
    uint32_t elapsed = (millis() - startTime) / 1000; // Seconds
    if(elapsed) {
      Serial.print(frame / elapsed);
      Serial.println(" fps");
    }
  }
}

