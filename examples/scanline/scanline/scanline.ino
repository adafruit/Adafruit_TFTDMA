// Example for the Adafruit_TFTDMA TFT_scanline object.

#include <Adafruit_TFTDMA.h>

#define TC     2       // Timer/counter index
#define RESET -1       // Reset pin (-1 if not used)
#define CS    -1       // Chip-select pin (-1 if not used -- tie to GND)
#define CD    A2       // Command/data pin
#define RD    A0       // Read-strobe pin
#define WR    A3       // Write-strobe pin (CCL-modified output from timer)
#define D0     0       // Data bit 0 pin (MUST be on PORT byte boundary)
#define PERIPH PIO_CCL // Peripheral type of WR pin (PIO_CCL, PIO_TIMER, etc.)

TFT_scanline tft(TC, RESET, CS, CD, RD, WR, D0, PERIPH);

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

  startTime = millis();
}

uint16_t c = 0; // 16-bit color

void myCallback(uint16_t *linebuf) {
  // Add a single span, full width, using linebuf
  tft.addSpan(linebuf, TFTWIDTH);
  // Fill linebuf with solid color
  uint16_t c2 = (c << 8) | (c >> 8);
  for(int i=0; i<TFTWIDTH; i++) {
    linebuf[i] = c2;
  }
  c++;
}

void loop() {
  int i;

  tft.update(0, 0, TFTWIDTH-1, TFTHEIGHT-1, myCallback);

  // Show approximate frame rate
  if(!(++frame & 255)) { // Every 256 frames...
    uint32_t elapsed = (millis() - startTime) / 1000; // Seconds
    if(elapsed) {
      Serial.print(frame / elapsed);
      Serial.println(" fps");
    }
  }
}

