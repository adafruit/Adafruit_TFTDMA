/*!
 * @file Adafruit_TFTDMA.cpp
 *
 * @mainpage SAMD51 DMA-driven ILI9341 display library.
 *
 * @section intro_sec Introduction
 *
 * This is the documentation for Adafruit's SAMD51 DMA-driven ILI9341
 * library.  It is designed specifically for this microcontroller and
 * display combo, using the display's parallel interface.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * @section dependencies Dependencies
 *
 * This library depends on
 * <a href="https://github.com/adafruit/Adafruit_ZeroDMA">Adafruit_ZeroDMA</a>
 * being present on your system. Please make sure you have installed the
 * latest version before using this library.
 *
 * @section author Author
 *
 * Written by Phil "PaintYourDragon" Burgess for Adafruit Industries.
 *
 * @section license License
 *
 * BSD license, all text here must be included in any redistribution.
 */

#include "Adafruit_TFTDMA.h"
#include "wiring_private.h" // pinPeripheral() function

//--------------------------------------------------------------------------

#define ILI9341_SOFTRESET     0x01  ///< Software reset
#define ILI9341_SLEEPOUT      0x11  ///< Exit sleep mode
#define ILI9341_DISPLAYOFF    0x28  ///< Display OFF
#define ILI9341_DISPLAYON     0x29  ///< Display ON
#define ILI9341_COLADDRSET    0x2A  ///< Column Address Set
#define ILI9341_PAGEADDRSET   0x2B  ///< Page Address Set
#define ILI9341_MEMORYWRITE   0x2C  ///< Memory Write
#define ILI9341_MADCTL        0x36  ///< Memory Access Control
#define ILI9341_PIXELFORMAT   0x3A  ///< COLMOD / Pixel Format Set
#define ILI9341_FRAMECONTROL  0xB1  ///< Frame Rate Control
#define ILI9341_DISPLAYFUNC   0xB6  ///< Display Function Control
#define ILI9341_ENTRYMODE     0xB7  ///< Entry Mode Set
#define ILI9341_POWERCONTROL1 0xC0  ///< Power Control 1
#define ILI9341_POWERCONTROL2 0xC1  ///< Power Control 2
#define ILI9341_VCOMCONTROL1  0xC5  ///< VCOM Control 1
#define ILI9341_VCOMCONTROL2  0xC7  ///< VCOM Control 2

#define ILI9341_MADCTL_MY     0x80  ///< Row Address Order
#define ILI9341_MADCTL_MX     0x40  ///< Column Address Order
#define ILI9341_MADCTL_MV     0x20  ///< Row/Column Exchange
#define ILI9341_MADCTL_ML     0x10  ///< Vertical Regresh Order
#define ILI9341_MADCTL_RGB    0x00  ///< RGB Order
#define ILI9341_MADCTL_BGR    0x08  ///< BGR Order
#define ILI9341_MADCTL_MH     0x04  ///< Horizontal Regresh Order

// MADCTL settings for landscape and 180-rotated landscape
#define ROTATION1 ILI9341_MADCTL_MX | ILI9341_MADCTL_MY | \
                  ILI9341_MADCTL_MV | ILI9341_MADCTL_BGR  ///< Landscape
#define ROTATION2 ILI9341_MADCTL_MV | ILI9341_MADCTL_BGR  ///< Rotated 180

#define CS_ACTIVE  *csPortClr    = csPinMask  ///< Chip Select active
#define CS_IDLE    *csPortSet    = csPinMask  ///< Chip Select idle
#define CD_COMMAND *cdPortClr    = cdPinMask  ///< CD = Command
#define CD_DATA    *cdPortSet    = cdPinMask  ///< CD = Data
#define RD_ACTIVE  *rdPortClr    = rdPinMask  ///< Read Strobe active
#define RD_IDLE    *rdPortSet    = rdPinMask  ///< Read Strobe idle
#define WR_ACTIVE  *wrPortActive = wrPinMask  ///< Write Strobe active
#define WR_IDLE    *wrPortIdle   = wrPinMask  ///< Write Strobe idle

#define WR_STROBE { WR_ACTIVE; WR_IDLE; }  ///< Toggle write strobe once
#define write8(d) { *writePort=(d); WR_STROBE; }  ///< 8-bit write-and-strobe

#if TFT_INTERFACE == TFT_INTERFACE_8
 #define setWriteDir() { *dirSet = 0xFF; }  ///< Set PORT for output
 #define setReadDir()  { *dirClr = 0xFF; }  ///< Set PORT for input
 // BE CAREFUL when using write16() -- it's a macro, so avoid things like
 // write16(*foo++) which will expand 2X and have unexpected consequences.
 #define write16(d)    {*writePort=(d>>8);WR_STROBE;*writePort=(d);WR_STROBE;}  ///< 16-bit write-and-strobe
#elif TFT_INTERFACE == TFT_INTERFACE_16
 #define setWriteDir() { *dirSet = 0xFFFF; }  ///< Set PORT for output
 #define setReadDir()  { *dirClr = 0xFFFF; }  ///< Set PORT for input
 #define write16(d)    { *writePort=(d); WR_STROBE; }  ///< 16-bit write-and-strobe
#endif

#if 0 // Not currently used (was in readID() function, which is also not used)
#define read8(result) {   \
    RD_ACTIVE;            \
    delayMicroseconds(1); \
    result = *readPort;   \
    RD_IDLE; }
#endif

// Timer/counter info by index #
static const struct {
    Tc *tc;   // -> Timer/Counter base address
    int gclk; // GCLK ID
    int evu;  // EVSYS user ID
} tcList[] = {
  { TC0, TC0_GCLK_ID, EVSYS_ID_USER_TC0_EVU },
  { TC1, TC1_GCLK_ID, EVSYS_ID_USER_TC1_EVU },
  { TC2, TC2_GCLK_ID, EVSYS_ID_USER_TC2_EVU },
  { TC3, TC3_GCLK_ID, EVSYS_ID_USER_TC3_EVU },
#ifdef TC4
  { TC4, TC4_GCLK_ID, EVSYS_ID_USER_TC4_EVU },
#endif
#ifdef TC5
  { TC5, TC5_GCLK_ID, EVSYS_ID_USER_TC5_EVU },
#endif
#ifdef TC6
  { TC6, TC6_GCLK_ID, EVSYS_ID_USER_TC6_EVU },
#endif
#ifdef TC7
  { TC7, TC7_GCLK_ID, EVSYS_ID_USER_TC7_EVU }
#endif
};
#define NUM_TIMERS (sizeof tcList / sizeof tcList[0])  ///< Number of available timer/counter peripherals

// DMA transfer-in-progress indicator and callback
static volatile bool dma_busy = false;
static void dma_callback(Adafruit_ZeroDMA *dma) {
    dma_busy = false;
}

//--------------------------------------------------------------------------

Adafruit_TFTDMA::Adafruit_TFTDMA(int8_t tc, int8_t reset, int8_t cs,
  int8_t cd, int8_t rd, int8_t wr, int8_t d0, _EPioType periph) {

    tcNum = -1; // Init to -1 to indicate constructor trouble
    if((tc < 0) || (tc >= (int8_t)NUM_TIMERS)) return; // Invalid TC number!
    uint8_t dBit = g_APinDescription[d0].ulPin; // d0 bit # in PORT
#if TFT_INTERFACE == TFT_INTERFACE_8
    if(dBit &  7) return; // d0 is not byte-aligned in PORT!
#elif TFT_INTERFACE == TFT_INTERFACE_16
    if(dBit & 15) return; // d0 is not halfword-aligned in PORT!
#endif

    tcNum    = tc;    // Save timer/counter #
    csPin    = cs;    // Save chip select pin (-1 if unused)
    cdPin    = cd;    // " command/data pin # (required >= 0)
    rdPin    = rd;    // " read-strobe pin    (required >= 0)
    wrPin    = wr;    // " write-strobe pin   (required >= 0)
    resetPin = reset; // " reset pin          (-1 if unused)
    d0Pin    = d0;    // " data bit 0 pin     (required, PORT byte boundary)

    // Get pointers to PORT SET and CLR registers for the control signals
    if(cs >= 0) { // If chip select pin is specified...
        // Get PORT set/clr pointers and bitmask for CS pin
        csPinMask = digitalPinToBitMask(cs);
        csPortSet = &(PORT->Group[g_APinDescription[cs].ulPort].OUTSET.reg);
        csPortClr = &(PORT->Group[g_APinDescription[cs].ulPort].OUTCLR.reg);
    } else { // No CS specified (tied low)
        csPinMask = 0;
        csPortSet = csPortClr = NULL;
    }
    if(rdPin >= 0) { // If read-strobe pin is specified...
        rdPinMask    = digitalPinToBitMask(rd);
        rdPortSet    = &(PORT->Group[g_APinDescription[rd].ulPort].OUTSET.reg);
        rdPortClr    = &(PORT->Group[g_APinDescription[rd].ulPort].OUTCLR.reg);
    } else {
        rdPinMask = 0;
        rdPortSet = rdPortClr = NULL;
    }
    cdPinMask    = digitalPinToBitMask(cd); // And bitmasks as well
    cdPortSet    = &(PORT->Group[g_APinDescription[cd].ulPort].OUTSET.reg);
    cdPortClr    = &(PORT->Group[g_APinDescription[cd].ulPort].OUTCLR.reg);
    wrPinMask    = digitalPinToBitMask(wr);
    wrPeripheral = periph;
    if(periph == PIO_CCL) { // WR pin is using CCL to invert PWM (but not GPIO)
        // Normal - idle state is SET (high), active state is CLEAR (low)
        wrPortIdle   = &(PORT->Group[g_APinDescription[wr].ulPort].OUTSET.reg);
        wrPortActive = &(PORT->Group[g_APinDescription[wr].ulPort].OUTCLR.reg);
    } else {  // WR pin is regular PWM out; requires external inverter
        // Inverted - idle state is CLEAR (low), active state is SET (high)
        wrPortIdle   = &(PORT->Group[g_APinDescription[wr].ulPort].OUTCLR.reg);
        wrPortActive = &(PORT->Group[g_APinDescription[wr].ulPort].OUTSET.reg);
    }

    PortGroup *p = (&(PORT->Group[g_APinDescription[d0].ulPort]));
#if TFT_INTERFACE == TFT_INTERFACE_8
    // Get pointers to PORT write/read/dir bytes within 32-bit PORT
    uint8_t offset = dBit / 8; // d[7:0] byte # within PORT
    writePort = (volatile uint8_t *)&(p->OUT.reg)    + offset;
    readPort  = (volatile uint8_t *)&(p->IN.reg)     + offset;
    dirSet    = (volatile uint8_t *)&(p->DIRSET.reg) + offset;
    dirClr    = (volatile uint8_t *)&(p->DIRCLR.reg) + offset;
#elif TFT_INTERFACE == TFT_INTERFACE_16
    // Get pointers to PORT write/read/dir halfwords within 32-bit PORT
    uint8_t offset = dBit / 16; // d[15:0] word # within PORT
    writePort = (volatile uint16_t *)&(p->OUT.reg)    + offset;
    readPort  = (volatile uint16_t *)&(p->IN.reg)     + offset;
    dirSet    = (volatile uint16_t *)&(p->DIRSET.reg) + offset;
    dirClr    = (volatile uint16_t *)&(p->DIRCLR.reg) + offset;
#endif
}

bool Adafruit_TFTDMA::begin(void) {
    if(tcNum < 0) return true; // Constructor failed (prob bad tc or d0)

    // INITIALIZE DMA (descriptors, etc. are done in subclass) -------------

    if(dma.allocate() != DMA_STATUS_OK) return true;    // Allocate channel

    int dmaChannel = dma.getChannel();
    DMAC->Channel[dmaChannel].CHEVCTRL.bit.EVOE    = 1; // Enable event output
    DMAC->Channel[dmaChannel].CHEVCTRL.bit.EVOMODE = 0; // Use EVOSEL output
    dma.setCallback(dma_callback);

    // INITIALIZE CONTROL PIN STATES (all are active LOW) ------------------

    if(resetPin >= 0) {
        pinMode(resetPin, OUTPUT); digitalWrite(resetPin, HIGH);
    }
    if(csPin >= 0) {
        pinMode(csPin, OUTPUT); digitalWrite(csPin, HIGH);
    }
    if(rdPin >= 0) {
        pinMode(rdPin, OUTPUT); digitalWrite(rdPin, HIGH);
    }
    pinMode(cdPin, OUTPUT); digitalWrite(cdPin, HIGH); // All idle
    pinMode(wrPin, OUTPUT); digitalWrite(wrPin, HIGH);

    // Initialize data pins.  We were only passed d0, so scan the
    // pin description list looking for the other seven pins.
    // They'll be on the same PORT, and within the next 7 (or 15) bits
    // (because we need to write to a contiguous PORT byte).
    uint8_t portNum = g_APinDescription[d0Pin].ulPort, // d0 PORT #
            dBit    = g_APinDescription[d0Pin].ulPin;  // d0 bit # in PORT
    for(uint8_t i=0; i<PINS_COUNT; i++) {
        if((g_APinDescription[i].ulPort == portNum ) &&
           (g_APinDescription[i].ulPin  >= dBit    ) &&
#if TFT_INTERFACE == TFT_INTERFACE_8
           (g_APinDescription[i].ulPin  <= (uint32_t)(dBit +  7))) {
#elif TFT_INTERFACE == TFT_INTERFACE_16
           (g_APinDescription[i].ulPin  <= (uint32_t)(dBit + 15))) {
#endif
            pinMode(i, OUTPUT); digitalWrite(i, LOW);
        }
    }

    // CONFIGURE TIMER/COUNTER (used for write strobe pulses) --------------

#if 0 // DISABLED - THIS IS ALREADY DONE BY ARDUINO INIT CODE BY DEFAULT
    // Config GCLK0 for 120 MHz
    GCLK_GENCTRL_Type genctrl;
    genctrl.bit.SRC      = GCLK_GENCTRL_SRC_DPLL0_Val;
    genctrl.bit.GENEN    = 1; // Enable
    genctrl.bit.OE       = 1;
    genctrl.bit.DIVSEL   = 0; // Do not divide clock source
    genctrl.bit.DIV      = 0;
    GCLK->GENCTRL[0].reg = genctrl.reg;
    while(GCLK->SYNCBUSY.bit.GENCTRL1 == 1);
#endif

    Tc               *timer = tcList[tcNum].tc;   // -> Timer/Counter struct
    int               id    = tcList[tcNum].gclk; // Timer GCLK ID
    GCLK_PCHCTRL_Type pchctrl;

    // Set up timer clock source from GCLK above
    GCLK->PCHCTRL[id].bit.CHEN = 0;    // Disable timer
    while(GCLK->PCHCTRL[id].bit.CHEN); // Wait for disable
    pchctrl.bit.GEN            = GCLK_PCHCTRL_GEN_GCLK0_Val;
    pchctrl.bit.CHEN           = 1;
    GCLK->PCHCTRL[id].reg = pchctrl.reg;
    while(!GCLK->PCHCTRL[id].bit.CHEN); // Wait for enable

    // Disable timer/counter before configuring it
    timer->COUNT8.CTRLA.bit.ENABLE     = 0;
    while(timer->COUNT8.SYNCBUSY.bit.STATUS);

    timer->COUNT8.WAVE.bit.WAVEGEN     = 2; // Normal PWM mode (NPWM)
    timer->COUNT8.CTRLA.bit.MODE       = 1; // 8-bit counter mode
    timer->COUNT8.CTRLA.bit.PRESCALER  = 0; // 1:1 clock prescale
    while(timer->COUNT8.SYNCBUSY.bit.STATUS);

//  timer->COUNT8.CTRLBSET.bit.DIR     = 1; // Count DOWN
    timer->COUNT8.CTRLBCLR.bit.DIR     = 1; // Count UP
    while(timer->COUNT8.SYNCBUSY.bit.CTRLB);
    timer->COUNT8.CTRLBSET.bit.ONESHOT = 1; // One-shot operation
    while(timer->COUNT8.SYNCBUSY.bit.CTRLB);
    timer->COUNT8.PER.reg              = 6; // PWM top value
    while(timer->COUNT8.SYNCBUSY.bit.PER);
    timer->COUNT8.CC[0].reg            = 2; // Compare value for channel 0
    while(timer->COUNT8.SYNCBUSY.bit.CC0);
//  timer->COUNT8.DRVCTRL.bit.INVEN0   = 1; // Invert output
    timer->COUNT8.EVCTRL.bit.TCEI      = 1; // Enable async input events
    timer->COUNT8.EVCTRL.bit.EVACT     = 1; // Event action = restart

    // Enable timer
    timer->COUNT8.CTRLA.reg |= TC_CTRLA_ENABLE;
    while(timer->COUNT8.SYNCBUSY.bit.STATUS);

    // CONFIGURE CCL (if used) (inverts timer/counter output) --------------

    if(wrPeripheral == PIO_CCL) {
        MCLK->APBCMASK.bit.CCL_ = 1; // Enable CCL bus clock
        CCL->CTRL.bit.ENABLE    = 0; // Disable CCL to configure
        CCL->CTRL.bit.SWRST     = 1; // Reset CCL registers to defaults
        CCL->LUTCTRL[tcNum].bit.ENABLE  = 0; // Disable LUT to configure
        CCL->LUTCTRL[tcNum].bit.FILTSEL = 0; // No filter
        CCL->LUTCTRL[tcNum].bit.INSEL0  = 6; // TC input source
        CCL->LUTCTRL[tcNum].bit.INSEL1  = 0; // MASK
        CCL->LUTCTRL[tcNum].bit.INSEL2  = 0; // MASK
        CCL->LUTCTRL[tcNum].bit.TRUTH   = 1; // Invert input 0
        CCL->LUTCTRL[tcNum].bit.ENABLE  = 1; // Enable LUT
        CCL->CTRL.bit.ENABLE    = 1; // Enable CCL peripheral
    }

    // CONFIGURE EVENT SYSTEM ----------------------------------------------

    // Set up event system clock source from GCLK above
    GCLK->PCHCTRL[EVSYS_GCLK_ID_0].bit.CHEN = 0;     // Disable EVSYS
    while(GCLK->PCHCTRL[EVSYS_GCLK_ID_0].bit.CHEN);  // Wait for disable
    pchctrl.bit.GEN                    = GCLK_PCHCTRL_GEN_GCLK0_Val;
    pchctrl.bit.CHEN                   = 1;
    GCLK->PCHCTRL[EVSYS_GCLK_ID_0].reg = pchctrl.reg;
    while(!GCLK->PCHCTRL[EVSYS_GCLK_ID_0].bit.CHEN); // Wait for enable
    MCLK->APBBMASK.bit.EVSYS_ = 1;                   // Enable EVSYS clock

    EVSYS->USER[tcList[tcNum].evu].reg = 1; // Connect Timer EVU to ch 0
    // Datasheet recommends single write operation; reg instead of bit
    // Also datasheet: PATH bits must be zero when using async!
    EVSYS_CHANNEL_Type ev;
    ev.reg       = 0;
    ev.bit.PATH  = 2;                 // Asynchronous
    ev.bit.EVGEN = 0x22 + dmaChannel; // DMA channel 0+
    EVSYS->Channel[0].CHANNEL.reg = ev.reg;

    // INITIALIZE TFT ------------------------------------------------------

    // Hard reset
    if(resetPin >= 0) {
        digitalWrite(resetPin, LOW);
        delay(2);
        digitalWrite(resetPin, HIGH);
    }
    // Data transfer sync -- issue four '0' bytes
    CS_ACTIVE;
    CD_COMMAND;
    write8(0);
    WR_STROBE; WR_STROBE; WR_STROBE; // Repeat last data 3X
    CS_IDLE;

    CS_ACTIVE;
    writeReg8( ILI9341_SOFTRESET    , 0);
    delay(150);
    writeReg8( ILI9341_DISPLAYOFF   , 0);
    writeReg8( ILI9341_POWERCONTROL1, 0x23);
    writeReg8( ILI9341_POWERCONTROL2, 0x10);
    writeReg16(ILI9341_VCOMCONTROL1 , 0x2B2B);
    writeReg8( ILI9341_VCOMCONTROL2 , 0xC0);
    writeReg8( ILI9341_MADCTL       , ROTATION1);
    writeReg8( ILI9341_PIXELFORMAT  , 0x55);
    writeReg16(ILI9341_FRAMECONTROL , 0x001B);
    writeReg8( ILI9341_ENTRYMODE    , 0x07);
    /* writeReg32(ILI9341_DISPLAYFUNC, 0x0A822700);*/
    writeReg8( ILI9341_SLEEPOUT     , 0);
    delay(150);
    writeReg8( ILI9341_DISPLAYON    , 0);
    CS_IDLE;
    delay(500);

    return false; // Success
}

#if 0 // Not being used
// Not really a full ID-reading function; just verifies if ILI9341!
uint32_t Adafruit_TFTDMA::readID(void) {
    uint32_t id;
    uint8_t  i, x;

    for(i=0; i<5; i++) { // Retry a few times...
        CS_ACTIVE;
        CD_COMMAND;
        write8(0xD3);
        setReadDir();    // Set up LCD data port(s) for READ operations
        CD_DATA;
        delayMicroseconds(50);
        read8(x);        // Do NOT merge or otherwise 'simplify' these
        id   = x;        // lines. There are macro shenanigans going on!
        id <<= 8;
        read8(x);
        id  |= x;
        id <<= 8;
        read8(x);
        id  |= x;
        id <<= 8;
        read8(x);
        id  |= x;
        CS_IDLE;
        setWriteDir();   // Restore LCD data port(s) to WRITE configuration
        if(id == 0x9341) break;
    }

    return id;
}
#endif

void Adafruit_TFTDMA::writeReg8(uint8_t reg, uint8_t value) {
    CD_COMMAND;
    write8(reg);
    CD_DATA;
    write8(value);
}

void Adafruit_TFTDMA::writeReg16(uint8_t reg, uint16_t value) {
    CD_COMMAND;
    write8(reg);
    CD_DATA;
    write16(value);
}

// Not used?
void Adafruit_TFTDMA::writeReg32(uint8_t reg, uint32_t value) {
    CD_COMMAND;
    write8(reg);
    CD_DATA;
    write16(value >> 16);
    write16(value);
}

// Unlike setAddrWindow in Adafruit_TFTLCD, this version issues the
// MEMORYWRITE command and leaves the display selected & in DATA mode.
// Just follow up with pixel data...
void Adafruit_TFTDMA::setAddrWindow(
  int16_t x1, int16_t y1, int16_t x2, int16_t y2) {
    CS_ACTIVE;
    CD_COMMAND;
    write8(ILI9341_COLADDRSET);
    CD_DATA;
    write16(x1);
    write16(x2);
    CD_COMMAND;
    write8(ILI9341_PAGEADDRSET);
    CD_DATA;
    write16(y1);
    write16(y2);
    CD_COMMAND;
    write8(ILI9341_MEMORYWRITE);
    CD_DATA;
    // Device is left in selected state...follow up with pixel data...
}

//--------------------------------------------------------------------------

TFT_framebuffer::TFT_framebuffer(int8_t tc, int8_t reset, int8_t cs,
  int8_t cd, int8_t rd, int8_t wr, int8_t d0, _EPioType periph) :
  Adafruit_TFTDMA(tc, reset, cs, cd, rd, wr, d0, periph) {
}

bool TFT_framebuffer::begin(void) {
    if(Adafruit_TFTDMA::begin()) return true;

    // Initialize descriptor list (one per scanline).  Per-scanline
    // descriptors allows partial screen updates to be made as a single
    // DMA transaction; when updating just a section of the screen, the
    // data being issued is not contiguous (scanlines in RAM are contiguous,
    // but we're sending partial scanlines), the descriptors link these all
    // together for direct transmission, no per-line memcpy calls required).
    for(int d=0; d<TFTHEIGHT; d++) {
        // No need to seSRCADDR, DESCADDR or BTCNT -- done in update()
        descriptor[d].BTCTRL.bit.VALID    = true;
        descriptor[d].BTCTRL.bit.EVOSEL   = 0x3; // Event strobe on beat xfer
        descriptor[d].BTCTRL.bit.BLOCKACT = DMA_BLOCK_ACTION_NOACT;
#if TFT_INTERFACE == TFT_INTERFACE_8
        descriptor[d].BTCTRL.bit.BEATSIZE = DMA_BEAT_SIZE_BYTE;
#elif TFT_INTERFACE == TFT_INTERFACE_16
        descriptor[d].BTCTRL.bit.BEATSIZE = DMA_BEAT_SIZE_HWORD;
#endif
        descriptor[d].BTCTRL.bit.SRCINC   = 1; // Increment source
        descriptor[d].BTCTRL.bit.DSTINC   = 0; // Don't increment dest
        descriptor[d].BTCTRL.bit.STEPSEL  = DMA_STEPSEL_SRC;
        descriptor[d].BTCTRL.bit.STEPSIZE = DMA_ADDRESS_INCREMENT_STEP_SIZE_1;
        descriptor[d].DSTADDR.reg         = (uint32_t)writePort;
    }

    // The DMA library needs to allocate at least one valid descriptor,
    // so we do that here.  It's not used in the conventional sense though,
    // just before a transfer we copy descriptor[0] to this address.
    dptr = dma.addDescriptor(NULL, NULL, 42, DMA_BEAT_SIZE_BYTE, false, false);

    // Clear framebuffer (this also initializes the dirty rect
    // so first update() call always refreshes the full display).
    fillScreen(0);

    return false; // Success
}

// Set pixel in framebuffer, NO clipping or dirty rect update
// Color is already endian-swapped
inline void TFT_framebuffer::rawPixel(int16_t x, int16_t y, uint16_t color) {
    framebuf[y * TFTWIDTH + x] = color;
}

// Set pixel in framebuffer, with clipping & dirty rect update
void TFT_framebuffer::drawPixel(int16_t x, int16_t y, uint16_t color) {
    if((x >= 0) && (y >= 0) && (x < TFTWIDTH) && (y < TFTHEIGHT)) {
#if (TFT_INTERFACE == TFT_INTERFACE_8) || (TFT_INTERFACE == TFT_INTERFACE_SPI)
        color = __builtin_bswap16(color);
#endif
        rawPixel(x, y, color);
        sully(x, y);
    }
}

void TFT_framebuffer::fillRect(
  int16_t x1, int16_t y1, int16_t w, int16_t h, uint16_t color) {

    if((w == 0) || (h == 0)) return; // Zero size

    int16_t x2, y2;

    // Sort inputs
    if(w > 0) {
        x2 = x1 + w - 1;
    } else {
        x2  =  x1;
        x1 +=  w + 1;
        w   = -w;
    }
    if(h > 0) {
        y2 = y1 + h - 1;
    } else {
        y2  =  y1;
        y1 +=  h + 1;
        h   = -h;
    }

    // Off-screen clipping
    if((x1 >= TFTWIDTH) || (y1 >= TFTHEIGHT) || (x2 < 0) || (y2 < 0)) return;

    // Edge clipping
    if(x1 < 0) {
        w += x1;
        x1 = 0;
    }
    if(y1 < 0) {
        h += y1;
        y1 = 0;
    }
    if(x2 >= TFTWIDTH) {
        x2 = TFTWIDTH - 1;
        w  = TFTWIDTH - x1;
    }
    if(y2 >= TFTHEIGHT) {
        y2 = TFTHEIGHT - 1;
        h  = TFTHEIGHT - y1;
    }

    sully(x1, y1); // Update dirty rect with clipped rect bounds
    sully(x2, y2);

    uint16_t x, y, *ptr;
    ptr   = &framebuf[y1 * TFTWIDTH + x1];
#if (TFT_INTERFACE == TFT_INTERFACE_8) || (TFT_INTERFACE == TFT_INTERFACE_SPI)
    color = __builtin_bswap16(color); // Endian-swap color
#endif
    for(y=0; y<h; y++, ptr += TFTWIDTH) {
        for(x=0; x<w; x++) ptr[x] = color;
    }
}

// Fill framebuffer with solid color
void TFT_framebuffer::fillScreen(uint16_t color) {
#if (TFT_INTERFACE == TFT_INTERFACE_8) || (TFT_INTERFACE == TFT_INTERFACE_SPI)
    color = __builtin_bswap16(color); // Endian-swap color
#endif
    // We know framebuffer is 32-bit aligned and an even number
    // of pixels, so we can do some 32-bit wide shenanigans...
    uint32_t *fbuf32 = (uint32_t *)framebuf,
              p2     = TFTWIDTH * TFTHEIGHT / 2,
              c32    = color * 0x00010001;

    while(p2--) *fbuf32++ = c32;

    minx = miny = 0;  // Update dirty rect to screen bounds
    maxx = TFTWIDTH  - 1;
    maxy = TFTHEIGHT - 1;
}

// Update dirty rectangle bounds with new point
void TFT_framebuffer::sully(int16_t x, int16_t y) {
    if(x < minx) minx = x;
    if(y < miny) miny = y;
    if(x > maxx) maxx = x;
    if(y > maxy) maxy = y;
}

void TFT_framebuffer::update(void) {

    if(minx >= TFTWIDTH) return; // No changes since last update

    // User SHOULD have called waitForUpdate() so framebuffer isn't
    // modified during DMA transfer.  But just in case they forget,
    // wait for DMA completion if one's currently in progress.
    // They'll still get glitches and other awfulness due to dirty
    // rect changes (it's reset in waitForUpdate()), but at least
    // it won't crash or hang.
    while(dma_busy);

    // Modify DMA descriptor list
    uint16_t w      = maxx - minx + 1,
             h      = maxy - miny;     // Is actually height - 1
    uint32_t offset = miny * TFTWIDTH + minx; // Index of first pixel
    offset += w; // DMA src needs to be at END of data!
#if TFT_INTERFACE == TFT_INTERFACE_8
    w      *= 2; // Byte count = 2X width (else halfwords = 1X)
#endif
    for(uint16_t d=0; d<=h; d++, offset += TFTWIDTH) {
        descriptor[d].SRCADDR.reg  = (uint32_t)&framebuf[offset];
        descriptor[d].BTCNT.reg    = w;
        descriptor[d].DESCADDR.reg = (uint32_t)&descriptor[d + 1];
    }
    descriptor[h].DESCADDR.reg = 0; // End of list

    // Copy first descriptor to the DMA lib's descriptor table
    memcpy(dptr, &descriptor[0], sizeof(DmacDescriptor));
    // This is preferable to setting the DMA descriptor base address
    // as it will play well with any other code using the DMA library
    // on other channels.

    // This also issues memory write command, keeps CS_ACTIVE & CD_DATA set
    setAddrWindow(minx, miny, maxx, maxy);

    pinPeripheral(wrPin, wrPeripheral); // Switch WR pin to timer/CCL
    dma_busy = true;
    dma.startJob();
    dma.trigger();
    // Transfer continues in background.  Calling function can
    // go ahead with other tasks, but should NOT modify the
    // framebuffer until waitForUpdate() returns.
}

void TFT_framebuffer::waitForUpdate(void) {
    while(dma_busy);                    // Wait for DMA completion
    pinPeripheral(wrPin, PIO_OUTPUT);   // Switch WR pin back to GPIO
    CS_IDLE;                            // Deselect TFT
    minx = miny = 0x7FFF;               // Reset dirty rect
    maxx = maxy = -1;
}

uint16_t *TFT_framebuffer::getBuffer(void) {
    return framebuf;
}

bool TFT_framebuffer::getDirtyRect(int16_t *x1, int16_t *y1,
  int16_t *x2, int16_t *y2) {
    *x1 = minx;
    *y1 = miny;
    *x2 = maxx;
    *y2 = maxy;
    return (maxx >= minx);
}

//--------------------------------------------------------------------------

TFT_segmented::TFT_segmented(int8_t tc, int8_t reset, int8_t cs, int8_t cd,
  int8_t rd, int8_t wr, int8_t d0, _EPioType periph) :
  Adafruit_TFTDMA(tc, reset, cs, cd, rd, wr, d0, periph) {
}

bool TFT_segmented::begin(void) {
    if(Adafruit_TFTDMA::begin()) return true;

    descriptor = dma.addDescriptor(
      NULL,               // move data from here
      (void *)writePort,  // to here
      256,                // this many...
      DMA_BEAT_SIZE_BYTE, // bytes/hword/words
      true,               // increment source addr?
      false);             // increment dest addr?
    descriptor->BTCTRL.bit.EVOSEL = 0x3; // Event strobe on beat transfer

    return false; // Success
}

// Set pixel in framebuffer, NO clipping or dirty rect update
// Color is already endian-swapped
inline void TFT_segmented::rawPixel(int16_t x, int16_t y, uint16_t color) {
    framebuf[y * width + x] = color;
}

// Set pixel in framebuffer, with clipping
void TFT_segmented::drawPixel(int16_t x, int16_t y, uint16_t color) {
    x -= xoffset;
    y -= yoffset;
    if((x >= 0) && (y >= 0) && (x < width) && (y < height)) {
#if (TFT_INTERFACE == TFT_INTERFACE_8) || (TFT_INTERFACE == TFT_INTERFACE_SPI)
        color = __builtin_bswap16(color); // Endian-swap color
#endif
        rawPixel(x, y, color);
    }
}

void TFT_segmented::fillRect(
  int16_t x1, int16_t y1, int16_t w, int16_t h, uint16_t color) {

    if((w == 0) || (h == 0)) return; // Zero size

    x1 -= xoffset;
    y1 -= yoffset;

    int16_t x2, y2;

    // Sort inputs
    if(w > 0) {
        x2 = x1 + w - 1;
    } else {
        x2  =  x1;
        x1 +=  w + 1;
        w   = -w;
    }
    if(h > 0) {
        y2 = y1 + h - 1;
    } else {
        y2  =  y1;
        y1 +=  h + 1;
        h   = -h;
    }

    // Off-screen clipping
    if((x1 >= width) || (y1 >= height) || (x2 < 0) || (y2 < 0)) return;

    // Edge clipping
    if(x1 < 0) {
        w += x1;
        x1 = 0;
    }
    if(y1 < 0) {
        h += y1;
        y1 = 0;
    }
    if(x2 >= width) {
        x2 = width - 1;
        w  = width - x1;
    }
    if(y2 >= height) {
        y2 = height - 1;
        h  = height - y1;
    }

    uint16_t x, y, *ptr;
    ptr   = &framebuf[y1 * width + x1];
#if (TFT_INTERFACE == TFT_INTERFACE_8) || (TFT_INTERFACE == TFT_INTERFACE_SPI)
    color = __builtin_bswap16(color); // Endian-swap color
#endif
    for(y=0; y<h; y++, ptr += width) {
        for(x=0; x<w; x++) ptr[x] = color;
    }
}

// Fill framebuffer with solid color
void TFT_segmented::fillScreen(uint16_t color) {
#if (TFT_INTERFACE == TFT_INTERFACE_8) || (TFT_INTERFACE == TFT_INTERFACE_SPI)
    color = __builtin_bswap16(color); // Endian-swap color
#endif
    uint32_t i, pixels = width * height;

    // Framebuffer might not be 32-bit aligned,
    // so it's written 16 bits at a time.
    for(i=0; i<pixels; i++) framebuf[i] = color;
}

void TFT_segmented::update(int16_t x1, int16_t y1, int16_t x2, int16_t y2,
  uint16_t *segmentBuf, uint16_t maxSegmentBytes,
  int16_t (*userCallback)(uint16_t *dest, uint16_t len, void *udPtr),
  void *userData) {

    while(dma_busy);

    if(x1 > x2) { int16_t t = x1; x1 = x2; x2 = t; }
    if(y1 > y2) { int16_t t = y1; y1 = y2; y2 = t; }
    if(x1 < 0)          x1 = 0;
    if(y1 < 0)          y1 = 0;
    if(x2 >= TFTWIDTH)  x2 = TFTWIDTH  - 1;
    if(y2 >= TFTHEIGHT) y2 = TFTHEIGHT - 1;

    width = x2 - x1 + 1; // Width, in pixels, of update area & segment buffer
#if TFT_INTERFACE == TFT_INTERFACE_8
    int16_t maxSegmentLines = (maxSegmentBytes / 2) / width; // Max lines/pass
#elif TFT_INTERFACE == TFT_INTERFACE_16
    int16_t maxSegmentLines =  maxSegmentBytes      / width; // Max lines/pass
#endif
    if(maxSegmentLines <= 0) return; // Inadequate buffer size!

    // This also issues memory write command, keeps CS_ACTIVE & CD_DATA set
    setAddrWindow(x1, y1, x2, y2);

    xoffset = x1;          // Offsets for the drawing functions
    yoffset = y1;
    height  = y2 - y1 + 1; // Height, in pixels, of area to update
    if(maxSegmentLines > height) maxSegmentLines = height;

    uint16_t *buf[2];
    uint8_t   idx = 2;
    int16_t   linesRemaining   = height,
              linesThisSegment = maxSegmentLines;
    buf[0] =  segmentBuf;
    buf[1] = &segmentBuf[maxSegmentBytes / 2];

    pinPeripheral(wrPin, wrPeripheral); // Switch WR pin to timer/CCL

    while(linesRemaining > 0) {
        if(idx < 2) {
            while(dma_busy); // Wait for prior transfer to complete
            // Send from buf[idx]
#if TFT_INTERFACE == TFT_INTERFACE_8
            descriptor->BTCNT.reg   = width * linesThisSegment * 2;
            descriptor->SRCADDR.reg = (uint32_t)buf[idx] +
                                      descriptor->BTCNT.reg;
#elif TFT_INTERFACE == TFT_INTERFACE_16
            descriptor->BTCNT.reg   = width * linesThisSegment;
            descriptor->SRCADDR.reg = (uint32_t)buf[idx] +
                                      descriptor->BTCNT.reg * 2;
#endif
            dma_busy                = true;
            dma.startJob();
            dma.trigger();
            linesRemaining         -= linesThisSegment;
            idx                     = 1 - idx; // Toggle buffer index
        } else {
            idx = 0; // 1st pass; no DMA, begin filling buf[0]
        }
// Not sure why this delay is required...doesn't necessarily need
// to be at this point, but somewhere in the linesRemaining loop.
// Without it, the display shows occasional glitches that don't
// correspond to the address window or pixel data being pushed,
// but are nondestructive to the screen contents.  Strange.
// Maybe DMA start/trigger can only be so often?  Framebuffer mode
// doesn't exhibit this symptom, but it's one large DMA operation
// (long descriptor chain) rather than multiple small triggerings.
// If so -- determine minimal trigger time and compare micros()
// against prior time so it's only delaying as much as required.
delayMicroseconds(1600);

        // Calculate size of next segment, render if needed
        if(linesRemaining) {
            if(linesThisSegment > linesRemaining)
                linesThisSegment = linesRemaining;
            framebuf         = buf[idx];
            height           = linesThisSegment;
            linesThisSegment = (*userCallback)(framebuf, linesThisSegment,
                                               userData);
            yoffset         += linesThisSegment;
        }
    }
}

void TFT_segmented::waitForUpdate(void) {
    while(dma_busy);                  // Wait for DMA completion
    pinPeripheral(wrPin, PIO_OUTPUT); // Switch WR pin back to GPIO
    CS_IDLE;                          // Deselect TFT
}

//--------------------------------------------------------------------------

TFT_scanline::TFT_scanline(int8_t tc, int8_t reset, int8_t cs, int8_t cd,
  int8_t rd, int8_t wr, int8_t d0, _EPioType periph) :
  Adafruit_TFTDMA(tc, reset, cs, cd, rd, wr, d0, periph) {
}

bool TFT_scanline::begin(void) {
    if(Adafruit_TFTDMA::begin()) return true;

    // Initialize descriptor lists (see TFT_framebuffer begin() for an
    // explanation how and why descriptor lists are used).
    for(uint8_t s=0; s<2; s++) {
        for(int d=0; d<TFTWIDTH; d++) {
            // No need to set SRCADDR, DESCADDR or BTCNT -- done later
            scanline[s].descriptor[d].BTCTRL.bit.VALID    = true;
            scanline[s].descriptor[d].BTCTRL.bit.EVOSEL   = 0x3;
            scanline[s].descriptor[d].BTCTRL.bit.BLOCKACT =
              DMA_BLOCK_ACTION_NOACT;
#if TFT_INTERFACE == TFT_INTERFACE_8
            scanline[s].descriptor[d].BTCTRL.bit.BEATSIZE =
              DMA_BEAT_SIZE_BYTE;
#elif TFT_INTERFACE == TFT_INTERFACE_16
            scanline[s].descriptor[d].BTCTRL.bit.BEATSIZE =
              DMA_BEAT_SIZE_HWORD;
#endif
            scanline[s].descriptor[d].BTCTRL.bit.SRCINC   = 1;
            scanline[s].descriptor[d].BTCTRL.bit.DSTINC   = 0;
            scanline[s].descriptor[d].BTCTRL.bit.STEPSEL  = DMA_STEPSEL_SRC;
            scanline[s].descriptor[d].BTCTRL.bit.STEPSIZE =
              DMA_ADDRESS_INCREMENT_STEP_SIZE_1;
            scanline[s].descriptor[d].DSTADDR.reg         =
              (uint32_t)writePort;
        }
    }

    // The DMA library needs to allocate at least one valid descriptor,
    // so we do that here.  It's not used in the conventional sense though,
    // just before a transfer we copy the first scanline descriptor here.
    dptr = dma.addDescriptor(NULL, NULL, 42, DMA_BEAT_SIZE_BYTE, false, false);

    // DMA descriptor base setting will occur in update()

    return false; // Success
}

void TFT_scanline::addSpan(uint16_t *addr, int16_t w) {
#if TFT_INTERFACE == TFT_INTERFACE_8
    scanline[lineIdx].descriptor[spanIdx].BTCNT.reg    = w * 2; // Bytes
#elif TFT_INTERFACE == TFT_INTERFACE_16
    scanline[lineIdx].descriptor[spanIdx].BTCNT.reg    = w;     // Halfwords
#endif
    scanline[lineIdx].descriptor[spanIdx].SRCADDR.reg  = (uint32_t)addr+w*2;
    scanline[lineIdx].descriptor[spanIdx].DESCADDR.reg =
      (uint32_t)&scanline[lineIdx].descriptor[spanIdx + 1];
    spanIdx++;
}

void TFT_scanline::update(int16_t x1, int16_t y1, int16_t x2, int16_t y2,
  void (*userCallback)(uint16_t *dest, void *udPtr), void *userData) {

    if(x1 > x2) { int16_t t = x1; x1 = x2; x2 = t; }
    if(y1 > y2) { int16_t t = y1; y1 = y2; y2 = t; }
    if(x1 < 0)          x1 = 0;
    if(y1 < 0)          y1 = 0;
    if(x2 >= TFTWIDTH)  x2 = TFTWIDTH  - 1;
    if(y2 >= TFTHEIGHT) y2 = TFTHEIGHT - 1;

    // This also issues memory write command, keeps CS_ACTIVE & CD_DATA set
    setAddrWindow(x1, y1, x2, y2);

    pinPeripheral(wrPin, wrPeripheral); // Switch WR pin to timer/CCL

    lineIdx = 0; // Active scanline[] index

    while(y1++ <= y2) {
        // While prior transfer (if any) occurs...
        spanIdx = 0;                  // Reset span index counter
        (*userCallback)(scanline[lineIdx].linebuf, userData); // Build spans
        scanline[lineIdx].descriptor[spanIdx-1].DESCADDR.reg = 0; // End list
        while(dma_busy);              // Wait for prior transfer to complete
        // Copy scanline's first descriptor to the DMA lib's descriptor table
        memcpy(dptr, &scanline[lineIdx].descriptor[0], sizeof(DmacDescriptor));
        dma_busy = true;
        dma.startJob();
        dma.trigger();                // Start new transfer
        lineIdx = 1 - lineIdx;        // Toggle buffer index
    }
    while(dma_busy);                  // Wait for last transfer to complete
    pinPeripheral(wrPin, PIO_OUTPUT); // Switch WR pin back to GPIO
    CS_IDLE;                          // Deselect TFT
}

