// Example for the Adafruit_TFTDMA TFT_segmented object.

#include <Adafruit_TFTDMA.h>

#define TC     2       // Timer/counter index
#define RESET -1       // Reset pin (-1 if not used)
#define CS    -1       // Chip-select pin (-1 if not used -- tie to GND)
#define CD    A2       // Command/data pin
#define RD    A0       // Read-strobe pin
#define WR    A3       // Write-strobe pin (CCL-modified output from timer)
#define D0     0       // Data bit 0 pin (MUST be on PORT byte boundary)
#define PERIPH PIO_CCL // Peripheral type of WR pin (PIO_CCL, PIO_TIMER, etc.)

TFT_segmented tft(TC, RESET, CS, CD, RD, WR, D0, PERIPH);

// Some boxes...
struct {
  uint16_t color;      // 16-bit color
  int16_t  oldx, oldy; // Prior position
  int16_t  x, y;       // Current position
  int8_t   vx, vy;     // Current velocity (+1 or -1)  
} box[3] = {
  { 0xF800 }, // Red box
  { 0x07E0 }, // Green box
  { 0x001F }  // Blue box
};

uint32_t startTime, frame = 0;

int16_t minx, miny, maxx, maxy;

void setup() {
  Serial.begin(9600);
  while(!Serial);

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

  // Initialize boxes to random positions & directions
  for(int i=0; i<3; i++) {
    box[i].x  = box[i].oldx = random(10, TFTWIDTH-110);
    box[i].y  = box[i].oldy = random(10, TFTHEIGHT-110);
    box[i].vx = random(2) ? 1 : -1;
    box[i].vy = random(2) ? 1 : -1;
  }

  startTime = millis();
}

#define SEGMENTLINES 16
uint16_t segmentBuf[2][TFTWIDTH * SEGMENTLINES];

void mySegmentCallback(uint16_t *dest, uint16_t lines) {
  // Clear screen (erase old boxes)
  tft.fillScreen(0);

  // Draw new boxes
  for(int i=0; i<3; i++) {
    tft.fillRect(box[i].x, box[i].y, 100, 100, box[i].color);
  }
}

void loop() {
  int i;

  // Don't modify the box positions until callback is done using them
  tft.waitForUpdate();

  for(i=0; i<3; i++) {
    box[i].oldx = box[i].x;
    box[i].x   += box[i].vx;
    if((box[i].x == 0) || (box[i].x == TFTWIDTH-100))
      box[i].vx *= -1;
    box[i].oldy = box[i].y;
    box[i].y   += box[i].vy;
    if((box[i].y == 0) || (box[i].y == TFTHEIGHT-100))
      box[i].vy *= -1;
  }

  if(frame == 0) {
    // Force full-screen redraw on first frame
    minx = miny = 0;
    maxx = TFTWIDTH  - 1;
    maxy = TFTHEIGHT - 1;
  } else {
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

  // Start new screen update in the background
  tft.update(minx, miny, maxx, maxy,
    segmentBuf[0], sizeof segmentBuf[0], mySegmentCallback);

  // Show approximate frame rate
  if(!(++frame & 255)) { // Every 256 frames...
    uint32_t elapsed = (millis() - startTime) / 1000; // Seconds
    if(elapsed) {
      Serial.print(frame / elapsed);
      Serial.println(" fps");
    }
  }
}

