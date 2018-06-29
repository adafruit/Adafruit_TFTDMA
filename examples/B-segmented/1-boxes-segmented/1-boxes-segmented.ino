// This is the 'bouncing boxes' example again, this time using the
// TFT_segmented object.  This is much more RAM-friendly than
// TFT_framebuffer (using only about 20K in this example, vs 160K),
// but the tradeoff is that it requires more computation -- screen
// contents are not preserved between frames and must be redrawn
// each time (usually in multiple passes).  It's slightly slower.
// MOST Adafruit_TFTDMA PROGRAMS WILL PROBABLY USE TFT_segmented!

#include <Adafruit_TFTDMA.h>

// TFT hardware config:
#define TC     2       // Timer/counter index
#define RESET A4       // Reset pin (-1 if not used)
#define CS    -1       // Chip-select pin (-1 if not used -- tie to GND)
#define CD    A2       // Command/data pin
#define RD    A0       // Read-strobe pin
#define WR    A3       // Write-strobe pin (CCL-modified output from timer)
#define D0     0       // Data bit 0 pin (MUST be on PORT byte boundary)
#define PERIPH PIO_CCL // Peripheral type of WR pin (PIO_CCL, PIO_TIMER, etc.)

TFT_segmented tft(TC, RESET, CS, CD, RD, WR, D0, PERIPH);

// Same box structure from the boxes-framebuffer example:
struct {
  int16_t  oldx, oldy; // Prior position
  int16_t  x, y;       // Current position
  int8_t   vx, vy;     // Current velocity (+1 or -1)
} box[3];
// And their corresponding colors:
const uint16_t color[3] = { 0xF800, 0x07E0, 0x001F };

// When the screen (or a section thereof) is to be updated, this area is
// divided into horizontal bands we'll call "segments," each some number
// of scanlines as determined by the amount of RAM you can commit to.
// The minimum segment buffer size is one full scanline (TFTWIDTH), but
// you must allocate TWICE this because there are always two segments in
// use -- one is being transmitted while the next is being rendered.
// In this example, we'll provide for a 16-scanline segment buffer:
#define SEGMENTLINES 16
uint16_t segmentBuf[2][TFTWIDTH * SEGMENTLINES];
// If the area of the screen being updated is less than TFTWIDTH, the
// number of scanlines in each segment is automatically adjusted up to
// fill as much of the buffer as possible (fewer, larger segments),
// though there are ways to prevent this if it's not desired. Also,
// pixels are 16 bits, so the statement above allocates 2 * 320 * 16 * 2
// bytes, or 20K.

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

  // Initialize boxes to random positions & directions
  for(int i=0; i<3; i++) {
    box[i].x  = box[i].oldx = random(10, TFTWIDTH-110);
    box[i].y  = box[i].oldy = random(10, TFTHEIGHT-110);
    box[i].vx = random(2) ? 1 : -1;
    box[i].vy = random(2) ? 1 : -1;
  }

  startTime = millis();
}

// A user-provided callback function is invoked each time a segment is to
// be rendered.  The segment must be FULLY rendered by this function --
// because there is no framebuffer, there is no prior state whatsoever to
// modify.  The buffer isn't even cleared, it's 100% up to the callback.
// The callback is passed an address into the segment buffer (which will
// alternate on each call as one segment is to be rendered as the prior
// segment is transmitted), the number of scanlines in the segment buffer,
// and an optional pointer to your own application-specific data (notice
// for instance that the segment width is not passed in, nor a scanline
// number or such -- it is entirely to the application to decide how this
// information (if needed), is stored, whether in a structure or in global
// state variables).  The segment callback will ALWAYS be invoked in-order
// and top-to-bottom, so simple incremental counters could be used.
// In this box example, we're not using any of the parameters.
int16_t mySegmentCallback(uint16_t *dest, uint16_t lines, void *data) {

  // The same minimal graphics primitives (point, rect, fill screen) as
  // TFT_framebuffer are available, but there's an interesting twist:
  // these are automatically clipped to the segment boundaries.  You can
  // call the same set of graphics operations on each segment and they'll
  // piece together into a contiguous screen update.

  // Erase old boxes by just clearing the screen
  // (this also clears the whole segment buffer):
  tft.fillScreen(0);

  // Draw new boxes:
  for(int i=0; i<3; i++) {
    tft.fillRect(box[i].x, box[i].y, 100, 100, color[i]);
  }

  // The segment callback has the option of rendering fewer scanlines
  // than the value passed in.  This is esoteric but might be applicable
  // to some situations like a tile renderer that's optimized to start on
  // a tile boundary and render one full row.  But in most cases, if
  // rendering the full segment, you'll return the same value passed in:
  return lines;
}

void loop() {
  int i;

  // Unlike TFT_framebuffer, TFT_segmented does NOT provide a 'dirty
  // rectangle' for changed pixel bounds, because there is no global
  // framebuffer memory.  In this example we DO want to make the screen
  // updates efficient though, updating only the minimum rectangle needed
  // each time, so we track our own bounding rectangle:
  int16_t minx, miny, maxx, maxy;

  // Some "game logic" can run while the last segment is transmitted.
  // No more rendering is taking place at this time, so it's OK to
  // update the box positions:
  for(i=0; i<3; i++) {
    box[i].oldx = box[i].x;  // Save old position
    box[i].oldy = box[i].y;
    box[i].x   += box[i].vx; // Update position
    box[i].y   += box[i].vy;
    if((box[i].x == 0) || (box[i].x == TFTWIDTH-100))
      box[i].vx *= -1;       // Bounce off edges
    if((box[i].y == 0) || (box[i].y == TFTHEIGHT-100))
      box[i].vy *= -1;
  }

  // This function blocks until it's safe to start a new transfer:
  tft.waitForUpdate();

  if(frame == 0) {
    // Force full-screen redraw on first frame
    minx = miny = 0;
    maxx = TFTWIDTH  - 1;
    maxy = TFTHEIGHT - 1;
  } else {
    // Calculate minimum bounding rect of boxes on prior and current
    // frames (so we both erase the old ones and draw new ones):
    minx = miny = 0x7FFF;
    maxx = maxy = -1;
    for(i=0; i<3; i++) {
      if(box[i].x    < minx) minx = box[i].x;
      if(box[i].oldx < minx) minx = box[i].oldx;
      if(box[i].y    < miny) miny = box[i].y;
      if(box[i].oldy < miny) miny = box[i].oldy;
      if((box[i].x    + 99) > maxx) maxx = box[i].x    + 99;
      if((box[i].oldx + 99) > maxx) maxx = box[i].oldx + 99;
      if((box[i].y    + 99) > maxy) maxy = box[i].y    + 99;
      if((box[i].oldy + 99) > maxy) maxy = box[i].oldy + 99;
    }
  }

  // Erasing and box-drawing all takes place in the background.
  // No drawing operations here.

  // Process and transfer changed area to the screen.  This function blocks
  // for a while until the last segment is rendered, then returns when last
  // segment is being transmitted.  We pass in the coordinates of the rect
  // to update, the base address of our segment buffer (sufficient for TWO
  // segments), the size of ONE segment in bytes (65535 max), our callback
  // function, and in this case a NULL pointer because we're not making use
  // of the user data pointer in our callback.
  tft.update(minx, miny, maxx, maxy,
    segmentBuf[0], sizeof segmentBuf[0], mySegmentCallback, NULL);
  // Segment buffer is passed in (by design, it's not set in the constructor
  // or begin()) so it could be allocated on the heap if desired, using that
  // RAM only during screen updates rather than permanently.  You might have
  // other functions (an SD image loader or such) that would really like to
  // have that memory.  This is a simple example though so it's just global.

  // Show approximate frame rate
  if(!(++frame & 255)) { // Every 256 frames...
    uint32_t elapsed = (millis() - startTime) / 1000; // Seconds
    if(elapsed) {
      Serial.print(frame / elapsed);
      Serial.println(" fps");
    }
  }
}

