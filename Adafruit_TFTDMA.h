#ifndef _ADAFRUIT_TFTDMA_H_
#define _ADAFRUIT_TFTDMA_H_

#define TFTWIDTH  320
#define TFTHEIGHT 240

#include <Adafruit_ZeroDMA.h>
#include "utility/dma.h"

// Provides hardware stuff; subsequent classes provide graphics context
// Pass in TC #, pins (tc, reset, cs, cd, rd, wr, d0), CCL flag
class Adafruit_TFTDMA {
  public:
    Adafruit_TFTDMA(int8_t tc, int8_t reset, int8_t cs, int8_t cd, int8_t rd,
      int8_t wr, int8_t d0, _EPioType periph);
    bool begin(void);
  protected:
    volatile uint8_t  *writePort, *readPort, *dirSet, *dirClr;
    volatile uint32_t *cdPortSet, *wrPortIdle  , *rdPortSet, *csPortSet,
                      *cdPortClr, *wrPortActive, *rdPortClr, *csPortClr;
    uint32_t           cdPinMask,  wrPinMask   ,  rdPinMask,  csPinMask;
    int8_t             cdPin    ,  wrPin       ,  rdPin    ,  csPin,
                       resetPin ,  d0Pin;
    uint8_t            tcNum;
    _EPioType          wrPeripheral;
    Adafruit_ZeroDMA   dma;
    void               writeReg8(uint8_t reg, uint8_t value);
    void               writeReg16(uint8_t reg, uint16_t value);
    void               writeReg32(uint8_t reg, uint32_t value);
    void               setDmaDescriptorBase(void *addr);
    void               setAddrWindow(int16_t x1, int16_t y1,
                                     int16_t x2, int16_t y2);
#if 0 // Not used:
    uint32_t           readID(void);
#endif
};

class TFT_framebuffer : public Adafruit_TFTDMA {
  public:
    TFT_framebuffer(int8_t tc, int8_t reset, int8_t cs, int8_t cd, int8_t rd,
      int8_t wr, int8_t d0, _EPioType periph);
    bool begin(void);
    inline void rawPixel(int16_t x, int16_t y, uint16_t color);
    void drawPixel(int16_t x, int16_t y, uint16_t color);
    void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
    void fillScreen(uint16_t color);
    void update(void);
    void waitForUpdate(void);
    void sully(int16_t x, int16_t y);
    uint16_t *getBuffer(void);
  private:
    DmacDescriptor descriptor[TFTHEIGHT] __attribute__((aligned(16)));
    uint16_t       framebuf[TFTWIDTH * TFTHEIGHT];
    int16_t        minx, miny, maxx, maxy; // Dirty rect
};

class TFT_segmented : public Adafruit_TFTDMA {
  public:
    TFT_segmented(int8_t tc, int8_t reset, int8_t cs, int8_t cd, int8_t rd,
      int8_t wr, int8_t d0, _EPioType periph);
    bool begin(void);
  private:
    DmacDescriptor *descriptor;
    void          (*callback)(uint16_t *dest, uint16_t len);
};

class TFT_demoscene : public Adafruit_TFTDMA {
  public:
    TFT_demoscene(int8_t tc, int8_t reset, int8_t cs, int8_t cd, int8_t rd,
      int8_t wr, int8_t d0, _EPioType periph);
    bool begin(void);
  private:
    struct {
      DmacDescriptor descriptor[TFTWIDTH] __attribute__((aligned(16)));
      uint16_t       linebuf[TFTWIDTH];
    } scanline[2];
    void (*callback)(void);
};

#endif // _ADAFRUIT_TFTDMA_H_
