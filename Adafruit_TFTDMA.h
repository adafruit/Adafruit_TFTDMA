/*!
 * @file Adafruit_TFTDMA.h
 *
 * This is part of Adafruit's SAMD51 DMA-driven ILI9341 display library.
 * It is designed specifically for this microcontroller and display combo,
 * using the display's parallel interface.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Phil "PaintYourDragon" Burgess for Adafruit Industries.
 *
 * BSD license, all text here must be included in any redistribution.
 */

#ifndef _ADAFRUIT_TFTDMA_H_
#define _ADAFRUIT_TFTDMA_H_

#define TFT_INTERFACE_8   0 ///< 8-bit parallel interface
#define TFT_INTERFACE_16  1 ///< 16-bit parallel - work-in-progress
#define TFT_INTERFACE_SPI 2 ///< SPI - totally not implemented at all

// Display interface must be defined at compile-time, it's baked-in deep.
#define TFT_INTERFACE TFT_INTERFACE_8  ///< Select interface methodology
#define TFTWIDTH      320  ///< Display width in pixels
#define TFTHEIGHT     240  ///< Display height in pixels

#include <Adafruit_ZeroDMA.h>
#include "utility/dma.h"

//--------------------------------------------------------------------------
/*! 
  @brief  A base class that interfaces with the SAMD51 hardware, including
          PORTs, timer/counters, DMA and other operipherals.  Unlikely to
          be instantiated on its own, a few subclasses are provided that
          represent the display and graphics memory in different ways.
          Applications must be designed around ONE of these subclasses and
          stick with it; can't toggle among them.
*/
class Adafruit_TFTDMA {
  public:
    /*!
      @brief  Constructor.  Takes note of the hardware configuration to use,
              but does not itself initialize any hardware yet.
      @param  tc      Index of timer/counter peripheral for PWM (used for
                      generating write-strobe pulses), e.g. pass 2 to use
                      the TC2 peripheral.  Certain timer/counters may be
                      in use by other libraries or reserved for the Arduino
                      millis()/micros() timers.
      @param  reset   Index of pin connected to the ILI9341's reset line, or
                      -1 if unconnected.  Using the reset line is optional
                      but strongly recommended.
      @param  cs      Index of pin connected to ILI9341 chip select line.
                      Required; must be >= 0.
      @param  cd      Index of pin connected to ILI9341 command/data line.
                      Required; must be >= 0.
      @param  rd      Index of pin connected to ILI9341 read-strobe line, or
                      -1 if unconnected.  This library currently doesn't
                      read any registers or pixel data from the device; it
                      is coded specifically for this driver and write-only,
                      so -1 is totally acceptable and even preferred here.
                      Maybe this parameter will be removed in the future.
      @param  wr      Index of pin connected to ILI9341 write-strobe line.
                      Required; must be >= 0.  Additionally, this pin must
                      be a valid TCx/WO[0] output for the timer/counter
                      specified by the first parameter, OR a CCL/OUT[x] pin
                      for the same timer counter (see last parameter).
      @param  d0      Index of pin connected to ILI9341 data bit 0 line.
                      Required; must be >= 0.  Additionally, the
                      corresponding PORT bit index for this pin MUST be the
                      least-significant bit of an 8-bit byte (e.g. 0, 8, 16
                      or 24) if using the 8-bit parallel interface, or the
                      least-significant bit of a 16-bit halfword (e.g. 0 or
                      16) if using the 16-bit parallel interface.  Use of
                      the next 7 or 15 bits of the PORT is implied, the pins
                      corresponding to those bits might not be contiguous or
                      sequential; refer to the schematic or device-specific
                      variant.cpp file for insights.
      @param  periph  Peripheral type connected to the write-strobe pin for
                      PWM out.  This can be PIO_TIMER or PIO_TIMER_ALT
                      corresponding to TCx/WO[0] for that pin, or PIO_CCL
                      for CCL/OUT[x] on a pin.  PIO_TIMER and PIO_TIMER_ALT
                      require the use of an external logic inverter (the
                      ILI9341 uses active-low control signals).  PIO_CCL
                      does not need an inverter, but the choice of pins is
                      very limited, perhaps just one or two.
    */
    Adafruit_TFTDMA(int8_t tc, int8_t reset, int8_t cs, int8_t cd,
                    int8_t rd, int8_t wr, int8_t d0, _EPioType periph);
    /*!
      @brief   Initializes all pins and peripherals used by the library.
      @return  true if an error occurred, false otherwise.  An error
               returned here is usually symptomatic of a constructor
               problem, such as an invalid pin or timer number.
    */
    bool               begin(void);
  protected:
#if TFT_INTERFACE == TFT_INTERFACE_8
    volatile uint8_t  *writePort;    ///< Pointer to 8-bit PORT OUT
    volatile uint8_t  *readPort;     ///< Pointer to 8-bit PORT IN
    volatile uint8_t  *dirSet;       ///< 8-bit PORT direction set
    volatile uint8_t  *dirClr;       ///< 8-bit Port direction clear
#elif TFT_INTERFACE == TFT_INTERFACE_16
    volatile uint16_t *writePort;    ///< Pointer to 16-bit PORT OUT
    volatile uint16_t *readPort;     ///< Pointer to 16-bit PORT IN
    volatile uint16_t *dirSet;       ///< 16-bit PORT direction set
    volatile uint16_t *dirClr;       ///< 16-bit PORT direction clear
#endif
    volatile uint32_t *csPortSet;    ///< Pointer to CS pin PORT SET register
    volatile uint32_t *csPortClr;    ///< Pointer to CS pin PORT CLEAR register
    volatile uint32_t *cdPortSet;    ///< Pointer to CD pin PORT SET register
    volatile uint32_t *cdPortClr;    ///< Pointer to CD pin PORT CLEAR register
    volatile uint32_t *rdPortSet;    ///< Pointer to RD pin PORT SET register
    volatile uint32_t *rdPortClr;    ///< Pointer to RD pin PORT CLEAR register
    volatile uint32_t *wrPortActive; ///< Pointer to WR pin PORT active register
    volatile uint32_t *wrPortIdle;   ///< Pointer to WR pin PORT idle register
    uint32_t           csPinMask;    ///< Bitmask for CS pin PORT
    uint32_t           cdPinMask;    ///< Bitmask for CD pin PORT
    uint32_t           rdPinMask;    ///< Bitmask for RD pin PORT
    uint32_t           wrPinMask;    ///< Bitmask for WR pin PORT
    int8_t             csPin;        ///< Index of CS pin (or -1)
    int8_t             cdPin;        ///< Index of CD pin
    int8_t             rdPin;        ///< Index of RD pin
    int8_t             wrPin;        ///< Index of WR pin
    int8_t             resetPin;     ///< Index of RESET pin (or -1)
    int8_t             d0Pin;        ///< Index of data bit 0 pin
    int8_t             tcNum;        ///< Timer/Counter number
    _EPioType          wrPeripheral; ///< WR strobe peripheral type
    Adafruit_ZeroDMA   dma;          ///< DMA instance
    /*!
      @brief  Writes an 8-bit value to an ILI9341 configuration register.
      @param  reg    Device register to write.
      @param  value  Value written to register.
    */
    void               writeReg8(uint8_t reg, uint8_t value);
    /*!
      @brief  Writes a 16-bit value to an ILI9341 configuration register.
      @param  reg    Device register to write.
      @param  value  Value written to register.
    */
    void               writeReg16(uint8_t reg, uint16_t value);
    /*!
      @brief  Writes a 32-bit value to an ILI9341 configuration register.
      @param  reg    Device register to write.
      @param  value  Value written to register.
    */
    void               writeReg32(uint8_t reg, uint32_t value);
    /*!
      @brief  Sets the ILI9341 'address window' for subsequent graphics
              operations.  This also issues a MEMORYWRITE command and leaves
              the device selected and in DATA mode for incoming pixel data.
              Inputs are NOT sorted or clipped; subclasses provide
              higher-level functions that ensure the validity of these
              parameters.
      @param  x1  Left edge of graphics window (0 to TFTWIDTH-1).
      @param  y1  Top edge of graphics window (0 to TFTHEIGHT-1).
      @param  x2  Right edge of graphics window (0 to TFTWIDTH-1).
      @param  y2  Bottom edge of graphics window (0 to TFTHEIGHT-1).
    */
    void               setAddrWindow(int16_t x1, int16_t y1,
                                     int16_t x2, int16_t y2);
#if 0 // Not used:
    /*!
      @brief  Reads device ID from ILI9341 and confirms device is this type.
              Currently not used...the library is intended specifically for
              this chip, on purpose-built hardware.
    */
    uint32_t           readID(void);
#endif
};


//--------------------------------------------------------------------------
/*! 
  @brief  Subclass of Adafruit_TFTDMA providing a whole-screen contiguous
          framebuffer.  This is probably the easiest subclass to understand,
          but requires an enormous amount of RAM, nearly 160 kilobytes.
*/
class TFT_framebuffer : public Adafruit_TFTDMA {
  public:
    /*!
      @brief  Constructor.  Takes note of the hardware configuration to use,
              but does not itself initialize any hardware yet.
      @param  tc      Index of timer/counter peripheral for PWM (used for
                      generating write-strobe pulses), e.g. pass 2 to use
                      the TC2 peripheral.  Certain timer/counters may be
                      in use by other libraries or reserved for the Arduino
                      millis()/micros() timers.
      @param  reset   Index of pin connected to the ILI9341's reset line, or
                      -1 if unconnected.  Using the reset line is optional
                      but strongly recommended.
      @param  cs      Index of pin connected to ILI9341 chip select line.
                      Required; must be >= 0.
      @param  cd      Index of pin connected to ILI9341 command/data line.
                      Required; must be >= 0.
      @param  rd      Index of pin connected to ILI9341 read-strobe line, or
                      -1 if unconnected.  This library currently doesn't
                      read any registers or pixel data from the device; it
                      is coded specifically for this driver and write-only,
                      so -1 is totally acceptable and even preferred here.
                      Maybe this parameter will be removed in the future.
      @param  wr      Index of pin connected to ILI9341 write-strobe line.
                      Required; must be >= 0.  Additionally, this pin must
                      be a valid TCx/WO[0] output for the timer/counter
                      specified by the first parameter, OR a CCL/OUT[x] pin
                      for the same timer counter (see last parameter).
      @param  d0      Index of pin connected to ILI9341 data bit 0 line.
                      Required; must be >= 0.  Additionally, the
                      corresponding PORT bit index for this pin MUST be the
                      least-significant bit of an 8-bit byte (e.g. 0, 8, 16
                      or 24) if using the 8-bit parallel interface, or the
                      least-significant bit of a 16-bit halfword (e.g. 0 or
                      16) if using the 16-bit parallel interface.  Use of
                      the next 7 or 15 bits of the PORT is implied, the pins
                      corresponding to those bits might not be contiguous or
                      sequential; refer to the schematic or device-specific
                      variant.cpp file for insights.
      @param  periph  Peripheral type connected to the write-strobe pin for
                      PWM out.  This can be PIO_TIMER or PIO_TIMER_ALT
                      corresponding to TCx/WO[0] for that pin, or PIO_CCL
                      for CCL/OUT[x] on a pin.  PIO_TIMER and PIO_TIMER_ALT
                      require the use of an external logic inverter (the
                      ILI9341 uses active-low control signals).  PIO_CCL
                      does not need an inverter, but the choice of pins is
                      very limited, perhaps just one or two.
    */
    TFT_framebuffer(int8_t tc, int8_t reset, int8_t cs, int8_t cd, int8_t rd,
                    int8_t wr, int8_t d0, _EPioType periph);
    /*!
      @brief   Initializes all pins and peripherals used by the library.
      @return  true if an error occurred, false otherwise.  An error
               returned here is usually symptomatic of a constructor
               problem, such as an invalid pin or timer number.
    */
    bool            begin(void);
    /*!
      @brief   Refreshes the display based on the current 'dirty rectangle'
               bounds set by other graphics functions.  Initiates a DMA
               transfer and returns immediately -- application can proceed
               with other code at that point but MUST not modify the
               framebuffer as it's transferred.
    */
    void            update(void);
    /*!
      @brief   Wait for the DMA transfer initiated by update() to complete.
               Dirty rectangle is reset and framebuffer can then be modified.
    */
    void            waitForUpdate(void);
    /*!
      @brief   Set or expand the current dirty rectangle to contain a given
               pixel coordinate.  Input MUST be valid; is NOT clipped.  The
               few graphics primitives provided will set or expand the dirty
               rectangle automatically, this function is mostly for one's own
               code that may be modifying the framebuffer contents.
      @param   x  Horizontal position of pixel (0 to TFTWIDTH-1).
      @param   y  Vertical position of pixel (0 to TFTWIDTH-1).
    */
    void            sully(int16_t x, int16_t y);
    /*!
      @brief   Get the base address of the framebuffer.
      @return  Pointer to framebuffer data; contiguous row-major TFTWIDTH x
               TFTHEIGHT uint16_t values.
    */
    uint16_t       *getBuffer(void);
    /*!
      @brief   Get the framebuffer's current dirty rectangle, if any.
               This may be helpful if an application finds it easier or more
               efficient to simply erase a whole bounding area rather than
               multiple small elements within.
      @param   x1  Pointer to signed 16-bit type to hold LEFT edge of dirty
                   rectangle, if set (else will be >= TFTWIDTH).
      @param   y1  Pointer to signed 16-bit type to hold TOP edge of dirty
                   rectangle, if set (else will be >= TFTHEIGHT).
      @param   x2  Pointer to signed 16-bit type to hold RIGHT edge of dirty
                   rectangle, if set (else will be -1).
      @param   y2  Pointer to signed 16-bit type to hold BOTTOM edge of
                   dirty rectangle, if set (else will be -1).
      @return  true if dirty rectangle exists and is valid, false if no
               rectangle has been set (e.g. immediately after update()).
    */
    bool            getDirtyRect(int16_t *x1, int16_t *y1,
                                 int16_t *x2, int16_t *y2);
    /*!
      @brief   Lowest-level pixel-setting operation.  Sets color of pixel
               at a given coordinate.  Does NOT provide clipping, and pixel
               color must already be endian-adjusted if needed to match
               format needed by ILI9341 (MSB first).  Dirty rectangle is
               not updated.
      @param   x      Horizontal position of pixel (0 to TFTWIDTH-1).
      @param   y      Vertical position of pixel (0 to TFTHEIGHT-1).
      @param   color  16-bit (5/6/5 R/G/B) color value, ILI9341 byte order.
    */
    inline void     rawPixel(int16_t x, int16_t y, uint16_t color);
    /*!
      @brief   Set pixel in framebuffer, with clipping, endian-adjustment
                and dirty rectangle update.
      @param   x      Horizontal position of pixel.
      @param   y      Vertical position of pixel.
      @param   color  16-bit (5/6/5 R/G/B) color value, native byte order.
    */
    void            drawPixel(int16_t x, int16_t y, uint16_t color);
    /*!
      @brief   Fill rectangle in framebuffer, with clipping and dirty
               rectangle update.
      @param   x      Left edge of rectangle (right edge if negative width).
      @param   y      Top edge of rectangle (bottom edge if negative height).
      @param   w      Width in pixels.
      @param   h      Height in pixels.
      @param   color  16-bit (5/6/5 R/G/B) color value, native byte order.
    */
    void            fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
                             uint16_t color);
    /*!
      @brief   Fill entire framebuffer with solid color.
      @param   color  16-bit (5/6/5 R/G/B) color value, native byte order.
    */
    void            fillScreen(uint16_t color=0);
  private:
    DmacDescriptor *dptr; // Initial allocated DMA descriptor
    DmacDescriptor  descriptor[TFTHEIGHT] __attribute__((aligned(16)));
    uint16_t        framebuf[TFTWIDTH * TFTHEIGHT];
    int16_t         minx, miny, maxx, maxy; // Dirty rect
};


//--------------------------------------------------------------------------
/*! 
  @brief  Subclass of Adafruit_TFTDMA that does not provide a contiguous
          framebuffer.  Instead, application provides a buffer for graphics
          operations, which are divided into segments and redrawn as needed.
          Uses less RAM that TFT_framebuffer -- only as much as one wants to
          allocate to it (minimum of TFTWIDTH*2 16-bit pixels, or 1,280
          bytes, up to a maximum of 65,535 bytes).  Invokes a user callback
          to draw graphics into this buffer before issuing to the screen.
          There is no permanent framebuffer; graphics are disposed after
          each update, and dirty rectangle handling is the application;s
          responsibility (or update the full screen or sections as needed).
*/
class TFT_segmented : public Adafruit_TFTDMA {
  public:
    /*!
      @brief  Constructor.  Takes note of the hardware configuration to use,
              but does not itself initialize any hardware yet.
      @param  tc      Index of timer/counter peripheral for PWM (used for
                      generating write-strobe pulses), e.g. pass 2 to use
                      the TC2 peripheral.  Certain timer/counters may be
                      in use by other libraries or reserved for the Arduino
                      millis()/micros() timers.
      @param  reset   Index of pin connected to the ILI9341's reset line, or
                      -1 if unconnected.  Using the reset line is optional
                      but strongly recommended.
      @param  cs      Index of pin connected to ILI9341 chip select line.
                      Required; must be >= 0.
      @param  cd      Index of pin connected to ILI9341 command/data line.
                      Required; must be >= 0.
      @param  rd      Index of pin connected to ILI9341 read-strobe line, or
                      -1 if unconnected.  This library currently doesn't
                      read any registers or pixel data from the device; it
                      is coded specifically for this driver and write-only,
                      so -1 is totally acceptable and even preferred here.
                      Maybe this parameter will be removed in the future.
      @param  wr      Index of pin connected to ILI9341 write-strobe line.
                      Required; must be >= 0.  Additionally, this pin must
                      be a valid TCx/WO[0] output for the timer/counter
                      specified by the first parameter, OR a CCL/OUT[x] pin
                      for the same timer counter (see last parameter).
      @param  d0      Index of pin connected to ILI9341 data bit 0 line.
                      Required; must be >= 0.  Additionally, the
                      corresponding PORT bit index for this pin MUST be the
                      least-significant bit of an 8-bit byte (e.g. 0, 8, 16
                      or 24) if using the 8-bit parallel interface, or the
                      least-significant bit of a 16-bit halfword (e.g. 0 or
                      16) if using the 16-bit parallel interface.  Use of
                      the next 7 or 15 bits of the PORT is implied, the pins
                      corresponding to those bits might not be contiguous or
                      sequential; refer to the schematic or device-specific
                      variant.cpp file for insights.
      @param  periph  Peripheral type connected to the write-strobe pin for
                      PWM out.  This can be PIO_TIMER or PIO_TIMER_ALT
                      corresponding to TCx/WO[0] for that pin, or PIO_CCL
                      for CCL/OUT[x] on a pin.  PIO_TIMER and PIO_TIMER_ALT
                      require the use of an external logic inverter (the
                      ILI9341 uses active-low control signals).  PIO_CCL
                      does not need an inverter, but the choice of pins is
                      very limited, perhaps just one or two.
    */
    TFT_segmented(int8_t tc, int8_t reset, int8_t cs, int8_t cd, int8_t rd,
                  int8_t wr, int8_t d0, _EPioType periph);
    /*!
      @brief   Initializes all pins and peripherals used by the library.
      @return  true if an error occurred, false otherwise.  An error
               returned here is usually symptomatic of a constructor
               problem, such as an invalid pin or timer number.
    */
    bool            begin(void);
    /*!
      @brief   Refreshes specified region of the display, using a given
               buffer as working space.  The region will be divided
               vertically into a number of segments, the fewest needed to
               fit into the buffer, and a callback is then invoked to draw
               each segment as needed (segment contents are not preserved;
               every invocation must fully render that segment) and a DMA
               transfer is initiated while then processing the next segment.
               Function returns when last segment transfer is started --
               application can proceed with other code at that point but
               MUST not modify the segment buffer as it's transferred.
               Segment buffer size, specified in bytes, must be a minimum of
               two full scanlines (1,280 bytes), but larger is generally
               better, as RAM allows (e.g. 16 scanlines x 2).  One half of
               the segment buffer is drawn (via callback) while the other
               half is being transferred, swapping each time until the
               entire region is complete.
      @param   x1              Left edge of region to update.
      @param   y1              Top edge of region to update.
      @param   x2              Right edge of region to update.
      @param   y2              Bottom edge of region to update.
      @param   buf             Segment buffer, minimum 1,280 bytes,
                               16 X this or more is recommended, up to a
                               maximum of 65,535 bytes.
      @param   maxSegmentSize  Size of ONE HALF of the segment buffer,
                               e.g. the amount of data that will be rendered
                               or transferred in each segment -- there are
                               two segments being worked with here.
      @param   userCallback    Function for rendering one segment of the
                               region being updated.  Callback is passed a
                               pointer to a segment buffer, the height of
                               the region being updated (in scanlines),
                               and a pointer to optional user data provided
                               by the application.  Callback should return
                               the number of actual scanlines rendered
                               (between 1 and the number of lines passed in,
                               which allows for subsequent calls to be
                               aligned to specific boundaries if needed).
      @param   userData        Pointer to user-defined data structure which
                               can be used to pass data to the callback or
                               between invocations of the callback.  Very
                               little is passed to the callback -- this is
                               by design -- so this structure, or just
                               global variables in one's code, can be used
                               to contain information such as the region
                               width or horizontal/vertical offsets of the
                               segment within the overall screen.  Segments
                               are always processed in-order from top to
                               bottom, so simple incremental counters may be
                               adequate for many situations.
    */
    void            update(int16_t x1, int16_t y1, int16_t x2, int16_t y2,
                           uint16_t *buf, uint16_t maxSegmentSize,
                           int16_t (*userCallback)(uint16_t *dest,
                                                   uint16_t len,
                                                   void *udPtr),
                                                   void *userData);
    /*!
      @brief   Wait for the last DMA transfer initiated by update() to
               complete.  Segment buffer can then be modified by another
               region update.
    */
    void            waitForUpdate(void);
    /*!
      @brief   Lowest-level pixel-setting operation.  Sets color of pixel
               at a given coordinate in the current segment buffer.  Does
               NOT provide clipping, and pixel color must already be
               endian-adjusted if needed to match format needed by ILI9341
               (MSB first).
      @param   x      Horizontal position of pixel (0 to buffer width-1)
      @param   y      Vertical position of pixel (0 to buffer height-1).
      @param   color  16-bit (5/6/5 R/G/B) color value, ILI9341 byte order.
    */
    inline void     rawPixel(int16_t x, int16_t y, uint16_t color);
    /*!
      @brief   Set pixel in segment buffer, with clipping, compensating for
               the offset of the buffer relative to the screen.
      @param   x      Horizontal position of pixel.
      @param   y      Vertical position of pixel.
      @param   color  16-bit (5/6/5 R/G/B) color value, native byte order.
    */
    void            drawPixel(int16_t x, int16_t y, uint16_t color);
    /*!
      @brief   Fill rectangle in segment buffer, with clipping, compensating
               the offset of the buffer relative to the screen.
      @param   x      Left edge of rectangle (right edge if negative width).
      @param   y      Top edge of rectangle (bottom edge if negative height).
      @param   w      Width in pixels.
      @param   h      Height in pixels.
      @param   color  16-bit (5/6/5 R/G/B) color value, native byte order.
    */
    void            fillRect(int16_t x, int16_t y, int16_t w, int16_t h,
                             uint16_t color);
    /*!
      @brief   Fill entire segment buffer with solid color.
      @param   color  16-bit (5/6/5 R/G/B) color value, native byte order.
    */
    void            fillScreen(uint16_t color=0);
  private:
    DmacDescriptor *descriptor;
    uint16_t       *framebuf;
    int16_t         xoffset, yoffset, width, height;
};


//--------------------------------------------------------------------------
/*! 
  @brief  Subclass of Adafruit_TFTDMA in which display (or section thereof)
          is generated one scanline at a time, using a callback which sets
          up one or more sequential 'spans,' providing a start address for
          each and a width in pixels.  The total span width MUST match the
          update region width.  This requires careful planning but might be
          good for scrollers, tile engines, anything with lots of canned
          pixel data in tables.  Uses about 12K RAM.
*/
class TFT_scanline : public Adafruit_TFTDMA {
  public:
    /*!
      @brief  Constructor.  Takes note of the hardware configuration to use,
              but does not itself initialize any hardware yet.
      @param  tc      Index of timer/counter peripheral for PWM (used for
                      generating write-strobe pulses), e.g. pass 2 to use
                      the TC2 peripheral.  Certain timer/counters may be
                      in use by other libraries or reserved for the Arduino
                      millis()/micros() timers.
      @param  reset   Index of pin connected to the ILI9341's reset line, or
                      -1 if unconnected.  Using the reset line is optional
                      but strongly recommended.
      @param  cs      Index of pin connected to ILI9341 chip select line.
                      Required; must be >= 0.
      @param  cd      Index of pin connected to ILI9341 command/data line.
                      Required; must be >= 0.
      @param  rd      Index of pin connected to ILI9341 read-strobe line, or
                      -1 if unconnected.  This library currently doesn't
                      read any registers or pixel data from the device; it
                      is coded specifically for this driver and write-only,
                      so -1 is totally acceptable and even preferred here.
                      Maybe this parameter will be removed in the future.
      @param  wr      Index of pin connected to ILI9341 write-strobe line.
                      Required; must be >= 0.  Additionally, this pin must
                      be a valid TCx/WO[0] output for the timer/counter
                      specified by the first parameter, OR a CCL/OUT[x] pin
                      for the same timer counter (see last parameter).
      @param  d0      Index of pin connected to ILI9341 data bit 0 line.
                      Required; must be >= 0.  Additionally, the
                      corresponding PORT bit index for this pin MUST be the
                      least-significant bit of an 8-bit byte (e.g. 0, 8, 16
                      or 24) if using the 8-bit parallel interface, or the
                      least-significant bit of a 16-bit halfword (e.g. 0 or
                      16) if using the 16-bit parallel interface.  Use of
                      the next 7 or 15 bits of the PORT is implied, the pins
                      corresponding to those bits might not be contiguous or
                      sequential; refer to the schematic or device-specific
                      variant.cpp file for insights.
      @param  periph  Peripheral type connected to the write-strobe pin for
                      PWM out.  This can be PIO_TIMER or PIO_TIMER_ALT
                      corresponding to TCx/WO[0] for that pin, or PIO_CCL
                      for CCL/OUT[x] on a pin.  PIO_TIMER and PIO_TIMER_ALT
                      require the use of an external logic inverter (the
                      ILI9341 uses active-low control signals).  PIO_CCL
                      does not need an inverter, but the choice of pins is
                      very limited, perhaps just one or two.
    */
    TFT_scanline(int8_t tc, int8_t reset, int8_t cs, int8_t cd, int8_t rd,
                 int8_t wr, int8_t d0, _EPioType periph);
    /*!
      @brief   Initializes all pins and peripherals used by the library.
      @return  true if an error occurred, false otherwise.  An error
               returned here is usually symptomatic of a constructor
               problem, such as an invalid pin or timer number.
    */
    bool            begin(void);
    /*!
      @brief   Refreshes specified region of the display.  Region will be
               divided vertically into scanlines, and a callback is then
               invoked to draw each line as needed (line contents are not
               preserved; every invocation must fully render that line) and
               a DMA transfer is initiated while then processing the next
               line.  Function returns when last DMA transfer completes.
      @param   x1            Left edge of region to update.
      @param   y1            Top edge of region to update.
      @param   x2            Right edge of region to update.
      @param   y2            Bottom edge of region to update.
      @param   userCallback  Function for rendering one scanline of the
                             region being updated.  Callback is passed a
                             pointer to a one-scanline working buffer if
                             needed, and a pointer to optional user data
                             provided by the application.
      @param   userData      Pointer to user-defined data structure which can
                             be used to pass data to the callback or between
                             invocations of the callback.  Very little is
                             passed to the callback -- this is by design --
                             so this structure, or just global variables in
                             one's code, can be used to contain information
                             such as the region width or horizontal/vertical
                             offsets of the scanline within the overall
                             screen.
    */
    void            update(int16_t x1, int16_t y1, int16_t x2, int16_t y2,
                           void (*userCallback)(uint16_t *dest, void *udPtr),
                           void *userData);
    /*!
      @brief   User callback invokes this function one or more times to
               specify the next 'span' -- a horizontal length of pixels --
               to issue along the scanline being rendered, left to right.
      @param   addr Pointer to 16-bit pixel data at the start of the span.
      @param   w    Horizontal length of span, in pixels.  The total length
                    of all spans on a given scanline MUST match the width of
                    the region passed to update(); no clipping is performed.
                    Spans are always added sequentially left-to-right, there
                    is no insert or reverse operation; speed is of the
                    essence.  The internal span counter is always reset to 0
                    (left edge of region) at the start of each scanline.
      @param   inc  Selects whether source address increment is enabled in
                    DMA transfer descriptor.  Default is 1 (true).  If 0,
                    and if using the 16-bit parallel interface, this will
                    fill the span with the single color value at 'addr'
                    (in 8-bit mode, this only makes sense if the color's
                    high and low bytes are the same, e.g. black, white and
                    a dubious and strange assortment of colors).
    */
    void            addSpan(uint16_t *addr, int16_t w, bool inc=1);
  private:
    uint8_t         lineIdx;
    uint16_t        spanIdx;
    DmacDescriptor *dptr; // Initial allocated DMA descriptor
    struct scanline {
      DmacDescriptor descriptor[TFTWIDTH] __attribute__((aligned(16)));
      uint16_t       linebuf[TFTWIDTH];
    } scanline[2];
};

#endif // _ADAFRUIT_TFTDMA_H_
