#ifndef _ADAFRUIT_TFTDMA_H_
#define _ADAFRUIT_TFTDMA_H_

#define TFTWIDTH  320
#define TFTHEIGHT 240

#include <Adafruit_ZeroDMA.h>
#include "utility/dma.h"

// Base class interfaces with hardware stuff.  Subsequent classes provide
// different graphics contexts depending on application's needs.
// Pass in TC #, pins (tc, reset, cs, cd, rd, wr, d0), peripheral type
// for wr pin (PIO_TIMER, PIO_TIMER_ALT, PIO_CCL) -- if using TIMER or
// TIMER_ALT the wr signal will need to go through a hardware inverter.
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

// TFT_framebuffer is the easiest to understand, but requires gobs of RAM --
// it provides a full contiguous framebuffer that can be modified as needed.
// 320x240 16-bit pixels + 240 DMA descriptors = 157,440 bytes
class TFT_framebuffer : public Adafruit_TFTDMA {
  public:
    TFT_framebuffer(int8_t tc, int8_t reset, int8_t cs, int8_t cd, int8_t rd,
      int8_t wr, int8_t d0, _EPioType periph);
    bool begin(void);
    void           update(void);
    void           waitForUpdate(void);
    void           sully(int16_t x, int16_t y);
    uint16_t      *getBuffer(void);
    inline void    rawPixel(int16_t x, int16_t y, uint16_t color);
    void           drawPixel(int16_t x, int16_t y, uint16_t color);
    void           fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
                     uint16_t color);
    void           fillScreen(uint16_t color=0);
  private:
    DmacDescriptor descriptor[TFTHEIGHT] __attribute__((aligned(16)));
    uint16_t       framebuf[TFTWIDTH * TFTHEIGHT];
    int16_t        minx, miny, maxx, maxy; // Dirty rect
};

// TFT_segmented requires more forethought.  Uses less RAM -- only as much
// as you want to allocate to it (minimum TFTWIDTH * 2 16-bit pixels, or
// 1,280 bytes, up to a maximum of 65,535 bytes) -- but then requires use
// of a callback to redraw screen contents.
class TFT_segmented : public Adafruit_TFTDMA {
  public:
    TFT_segmented(int8_t tc, int8_t reset, int8_t cs, int8_t cd, int8_t rd,
      int8_t wr, int8_t d0, _EPioType periph);
    bool begin(void);
    inline void     rawPixel(int16_t x, int16_t y, uint16_t color);
    void            drawPixel(int16_t x, int16_t y, uint16_t color);
    void            fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
                      uint16_t color);
    void            fillScreen(uint16_t color=0);
    void            update(int16_t x1, int16_t y1, int16_t x2, int16_t y2,
                      uint16_t *buf, uint16_t maxSegmentSize,
                      void (*userCallback)(uint16_t *dest, uint16_t len));
    void            waitForUpdate(void);
  private:
    DmacDescriptor *descriptor;
    uint16_t       *framebuf;
    int16_t         xoffset, yoffset, width, height;
};

// TFT_demoscene is difficult to explain and isn't fully implemented yet.
// It uses a constant 11,520 bytes RAM and requires a per-scanline callback.
class TFT_demoscene : public Adafruit_TFTDMA {
  public:
    TFT_demoscene(int8_t tc, int8_t reset, int8_t cs, int8_t cd, int8_t rd,
      int8_t wr, int8_t d0, _EPioType periph);
    bool begin(void);
    void update(int16_t x1, int16_t y1, int16_t x2, int16_t y2,
      void (*userCallback)(uint16_t *dest));
  private:
    struct {
      DmacDescriptor descriptor[TFTWIDTH] __attribute__((aligned(16)));
      uint16_t       linebuf[TFTWIDTH];
    } scanline[2];
};

#endif // _ADAFRUIT_TFTDMA_H_
