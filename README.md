# Adafruit_TFTDMA [![Build Status](https://travis-ci.org/adafruit/Adafruit_TFTDMA.svg?branch=master)](https://travis-ci.org/adafruit/Adafruit_TFTDMA)
High performance ILI9341 display library using SAMD51-specific peripherals.

## Supported hardware

Display: ILI9341 320x240 pixels, 16 bits/pixel, landscape orientation. 8-bit parallel interface is known working. 16-bit parallel interface untested. SPI is not implemented, though there's some initial hooks.

Processor: SAMD51, e.g. Adafruit ItsyBitsy M4, Metro M4, Feather M4 (examples are written for the ItsyBitsy M4 pinout).

## Roadmap

This is currently focused around a specific future Adafruit project and it's unlikely many outside pull requests will be merged.

LIKELY to be merged:
  * Bug fixes.
  * Anything that can be REMOVED (functions, function arguments, variables, etc.) without impacting functionality.
  * Speed optimizations that do not incur significant bulk in RAM or code size.
NOT LIKELY to be merged:
  * Reformatting code for the sake of reformatting code. The resulting large "visual diff" makes it impossible to untangle actual bug fixes from merely rearranged lines.
  * Other display types or CPU architectures. This is targeting a very specific project. But the code is Open Source and you're always free and welcome to create a special fork!
  * Added convenience functions, function arguments, extra safety bounds-checking, etc. Library is intentionally very "enough rope" minimalist for performance reasons (see notes below).
  * Additional drawing primitives, especially subclassing Adafruit_GFX (see notes below). In fact I might remove the couple of drawing functions that are there; they're mostly for the examples, which could be written differently.
  * Additional examples, unless they very specifically illustrate a concept not already covered, and can do so in a brief way.
The idea is that these might serve as base classes for higher-level code. Tile or sprite engines, for example. Do it atop one of these classes, don't modify the class into a specific representation. User-facing classes can be written to be more friendly, with additional bounds checking and error reporting. This is all just the bare metal here.
