// START HERE, this is maybe the simplest example for Adafruit_TFTDMA.
// This is a 'bouncing boxes' example using the TFT_framebuffer object,
// which is a huge memory hog because the entire screen is buffered in
// RAM (using about 160K), but it's easiest to comprehend and use.

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

TFT_framebuffer tft(TC, RESET, CS, CD, RD, WR, D0, PERIPH);

// This example bounces some colored boxes around the screen.
// Info about them is stored here:
struct {
  int16_t  oldx, oldy; // Prior position
  int16_t  x, y;       // Current position
  int8_t   vx, vy;     // Current velocity (+1 or -1)  
} box[3];
// And their corresponding colors:
const uint16_t color[3] = { 0xF800, 0x07E0, 0x001F };

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

  tft.update(); // Clear display

  startTime = millis();
}

// This draws each frame of animation...
void loop() {
  int i;

  // Prior DMA transfer must complete before modifying framebuffer.
  // We can do stuff in the meantime, such as "game logic" -- in this
  // case we calculate the next box positions (but don't draw them yet):
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

  // This function blocks until it's safe to modify the framebuffer:
  tft.waitForUpdate();

  // Erase old boxes (write over them with the background color (0)):
  for(i=0; i<3; i++) {
    tft.fillRect(box[i].oldx, box[i].oldy, 100, 100, 0);
  }
  // There are some other ways we could do this.  One is simply to call
  // fillscreen(0) to erase everything...very easy but not efficient
  // because every pixel is modified (the update() function, which we'll
  // use in a moment, writes only the modified area to the display).
  // Another would be to call getDirtyRect() after boxes are drawn below,
  // save the values returned and use them as a single rectangular area to
  // erase on the next frame.

  // Draw new boxes:
  for(i=0; i<3; i++) {
    tft.fillRect(box[i].x, box[i].y, 100, 100, color[i]);
  }
  // The graphics primitives provided are very minimal -- just point,
  // rectangle fill and screen fill -- this is intentionally NOT a subclass
  // of Adafruit_GFX because most game-type graphics will be doing their
  // own more nuanced rendering (see other examples).

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

