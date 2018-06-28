// Example for the Adafruit_TFTDMA TFT_framebuffer object.
// This is a huge memory hog because the entire screen is buffered in RAM,
// but it's easiest to comprehend & use.

#include <Adafruit_TFTDMA.h>

#define TC     2       // Timer/counter index
#define RESET -1       // Reset pin (-1 if not used)
#define CS    -1       // Chip-select pin (-1 if not used -- tie to GND)
#define CD    A2       // Command/data pin
#define RD    A0       // Read-strobe pin
#define WR    A3       // Write-strobe pin (CCL-modified output from timer)
#define D0     0       // Data bit 0 pin (MUST be on PORT byte boundary)
#define PERIPH PIO_CCL // Peripheral type of WR pin (PIO_CCL, PIO_TIMER, etc.)

TFT_framebuffer tft(TC, RESET, CS, CD, RD, WR, D0, PERIPH);

// Some boxes...
struct {
  int16_t  oldx, oldy; // Prior position
  int16_t  x, y;       // Current position
  int8_t   vx, vy;     // Current velocity (+1 or -1)  
} box[3];
// And their corresponding colors...
const uint16_t color[3] = { 0xF800, 0x07E0, 0x001F };

uint32_t startTime, frame = 0;

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

  tft.update(); // Clear display

  // Initialize boxes to random positions & directions
  for(int i=0; i<3; i++) {
    box[i].x  = box[i].oldx = random(10, TFTWIDTH-110);
    box[i].y  = box[i].oldy = random(10, TFTHEIGHT-110);
    box[i].vx = random(2) ? 1 : -1;
    box[i].vy = random(2) ? 1 : -1;
  }

  startTime = millis();
}

void loop() {
  int i;

  // Prior DMA transfer must complete before modifying framebuffer.
  // We can do stuff in the meantime, such as game logic, or in this
  // case we calculate the next box positions (but don't draw them):
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

  // This function blocks until it's safe to modify the framebuffer...
  tft.waitForUpdate();

  // Erase old boxes.  While one could call fillScreen(), erasing
  // makes a better demonstration of 'dirty rectangle' updates.
  for(i=0; i<3; i++) {
    tft.fillRect(box[i].oldx, box[i].oldy, 100, 100, 0);
  }

  // Draw new boxes
  for(i=0; i<3; i++) {
    tft.fillRect(box[i].x, box[i].y, 100, 100, color[i]);
  }

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

