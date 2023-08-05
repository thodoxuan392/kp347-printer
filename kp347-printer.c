/*!
 * @file Adafruit_Thermal.cpp
 *
 * @mainpage Adafruit Thermal Printer Library
 *
 * @section intro_sec Introduction
 *
 * An Arduino library for the Adafruit Thermal Printer:
 *
 * https://www.adafruit.com/product/597
 *
 * These printers use TTL serial to communicate.  One pin (5V or 3.3V) is
 * required to issue data to the printer.  A second pin can OPTIONALLY be
 * used to poll the paper status, but not all printers support this, and
 * the output on this pin is 5V which may be damaging to some MCUs.
 *
 * Adafruit invests time and resources providing this open source code.
 * Please support Adafruit and open-source hardware by purchasing products
 * from Adafruit!
 *
 * @section author Author
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries, with
 * contributions from the open source community.  Originally based on
 * Thermal library from bildr.org
 *
 * @section license License
 *
 * MIT license, all text above must be included in any redistribution.
 */

#include "kp347-printer.h"

// Though most of these printers are factory configured for 19200 baud
// operation, a few rare specimens instead work at 9600.  If so, change
// this constant.  This will NOT make printing slower!  The physical
// print and feed mechanisms are the bottleneck, not the port speed.
#define BAUDRATE                                                               \
  19200 //!< How many bits per second the serial port should transfer

// ASCII codes used by some of the printer config commands:
#define ASCII_TAB '\t' //!< Horizontal tab
#define ASCII_LF '\n'  //!< Line feed
#define ASCII_FF '\f'  //!< Form feed
#define ASCII_CR '\r'  //!< Carriage return
#define ASCII_DC2 18   //!< Device control 2
#define ASCII_ESC 27   //!< Escape
#define ASCII_FS 28    //!< Field separator
#define ASCII_GS 29    //!< Group separator

// Because there's no flow control between the printer and Arduino,
// special care must be taken to avoid overrunning the printer's buffer.
// Serial output is throttled based on serial speed as well as an estimate
// of the device's print and feed rates (relatively slow, being bound to
// moving parts and physical reality).  After an operation is issued to
// the printer (e.g. bitmap print), a timeout is set before which any
// other printer operations will be suspended.  This is generally more
// efficient than using delay() in that it allows the parent code to
// continue with other duties (e.g. receiving or decoding an image)
// while the printer physically completes the task.

/*!
 * Number of microseconds to issue one byte to the printer.  11 bits
 * (not 8) to accommodate idle, start and stop bits.  Idle time might
 * be unnecessary, but erring on side of caution here.
 */
#define BYTE_TIME (((11L * 1000000L) + (BAUDRATE / 2)) / BAUDRATE)


// Internal function
static uint8_t printMode,
          prevByte,      // Last character issued to printer
          column,        // Last horizontal column printed
          maxColumn,     // Page width (output 'wraps' at this point)
          charHeight,    // Height of characters, in 'dots'
          lineSpacing,   // Inter-line spacing (not line height); in dots
          barcodeHeight, // Barcode height in dots, not including text
          maxChunkHeight,
          dtrPin;         // DTR handshaking pin (experimental)
static uint16_t firmware;  // Firmware version
static unsigned long  resumeTime,   // Wait until micros() exceeds this before sending byte
                      dotPrintTime, // Time to print a single dot line, in microseconds
                      dotFeedTime;  // Time to feed a single dot line, in microseconds
static void writeBytes(uint8_t a); 
static void writeDoubleBytes(uint8_t a, uint8_t b);
static void writeTripleBytes(uint8_t a, uint8_t b, uint8_t c);
static void writeQuadBytes(uint8_t a, uint8_t b, uint8_t c, uint8_t d);
static void setPrintMode(uint8_t mask); 
static void unsetPrintMode(uint8_t mask);
static void writePrintMode(); 
static void adjustCharValues(uint8_t printMode);

// This method sets the estimated completion time for a just-issued task.
void timeoutSet(unsigned long x) {
    resumeTime = micros() + x;
}

// This function waits (if necessary) for the prior task to complete.
void timeoutWait() {

    while ((long)(micros() - resumeTime) < 0L) {
      yield();
    }; // (syntax is rollover-proof)
}

// Printer performance may vary based on the power supply voltage,
// thickness of paper, phase of the moon and other seemingly random
// variables.  This method sets the times (in microseconds) for the
// paper to advance one vertical 'dot' when printing and when feeding.
// For example, in the default initialized state, normal-sized text is
// 24 dots tall and the line spacing is 30 dots, so the time for one
// line to be issued is approximately 24 * print time + 6 * feed time.
// The default print and feed times are based on a random test unit,
// but as stated above your reality may be influenced by many factors.
// This lets you tweak the timing to avoid excessive delays and/or
// overrunning the printer buffer.
void setTimes(unsigned long p, unsigned long f) {
  dotPrintTime = p;
  dotFeedTime = f;
}

// The next four helper methods are used when issuing configuration
// commands, printing bitmaps or barcodes, etc.  Not when printing text.

void writeBytes(uint8_t a) {
  timeoutWait();
  KP347_SEND_BYTE(a);
  timeoutSet(BYTE_TIME);
}

void writeDoubleBytes(uint8_t a, uint8_t b) {
  timeoutWait();
  KP347_SEND_BYTE(a);
  KP347_SEND_BYTE(b);
  timeoutSet(2 * BYTE_TIME);
}

void writeTripleBytes(uint8_t a, uint8_t b, uint8_t c) {
  timeoutWait();
  KP347_SEND_BYTE(a);
  KP347_SEND_BYTE(b);
  KP347_SEND_BYTE(c);
  timeoutSet(3 * BYTE_TIME);
}

void writeQuadBytes(uint8_t a, uint8_t b, uint8_t c, uint8_t d) {
  timeoutWait();
  KP347_SEND_BYTE(a);
  KP347_SEND_BYTE(b);
  KP347_SEND_BYTE(c);
  KP347_SEND_BYTE(d);
  timeoutSet(4 * BYTE_TIME);
}

// The underlying method for all high-level printing (e.g. println()).
// The inherited Print class handles the rest!
size_t write(uint8_t c) {

  if (c != 13) { // Strip carriage returns
    timeoutWait();
    KP347_SEND_BYTE(c);
    unsigned long d = BYTE_TIME;
    if ((c == '\n') || (column == maxColumn)) { // If newline or wrap
      d += (prevByte == '\n') ? ((charHeight + lineSpacing) * dotFeedTime)
                              : // Feed line
               ((charHeight * dotPrintTime) +
                (lineSpacing * dotFeedTime)); // Text line
      column = 0;
      c = '\n'; // Treat wrap as newline on next pass
    } else {
      column++;
    }
    timeoutSet(d);
    prevByte = c;
  }

  return 1;
}

void begin(uint16_t version) {

  firmware = version;

  // The printer can't start receiving data immediately upon power up --
  // it needs a moment to cold boot and initialize.  Allow at least 1/2
  // sec of uptime before printer can receive data.
  timeoutSet(500000L);

  wake();
  reset();

  setHeatConfig(11, 120, 40);

  // Enable DTR pin if requested
  if (dtrPin < 255) {
    writeTripleBytes(ASCII_GS, 'a', (1 << 5));
  }

  dotPrintTime = 30000; // See comments near top of file for
  dotFeedTime = 2100;   // an explanation of these values.
  maxChunkHeight = 255;
}

// Reset printer to default state.
void reset() {
  writeDoubleBytes(ASCII_ESC, '@'); // Init command
  prevByte = '\n';            // Treat as if prior line is blank
  column = 0;
  maxColumn = 32;
  charHeight = 24;
  lineSpacing = 6;
  barcodeHeight = 50;

  if (firmware >= 264) {
    // Configure tab stops on recent printers
    writeDoubleBytes(ASCII_ESC, 'D'); // Set tab stops...
    writeQuadBytes(4, 8, 12, 16);   // ...every 4 columns,
    writeQuadBytes(20, 24, 28, 0);  // 0 marks end-of-list.
  }
}

// Reset text formatting parameters.
void setDefault() {
  online();
  justify('L');
  inverseOff();
  doubleHeightOff();
  setLineHeight(30);
  boldOff();
  underlineOff();
  setBarcodeHeight(50);
  setSize('s');
  setCharset(0);
  setCodePage(0);
}

void test() {
  println(F("Hello World!"));
  feed(2);
}

void testPage() {
  writeDoubleBytes(ASCII_DC2, 'T');
  timeoutSet(dotPrintTime * 24 * 26 + // 26 lines w/text (ea. 24 dots high)
             dotFeedTime *
                 (6 * 26 + 30)); // 26 text lines (feed 6 dots) + blank line
}

void setBarcodeHeight(uint8_t val) { // Default is 50
  if (val < 1)
    val = 1;
  barcodeHeight = val;
  writeTripleBytes(ASCII_GS, 'h', val);
}

void printBarcode(const char *text, uint8_t type) {
  feed(1); // Recent firmware can't print barcode w/o feed first???
  if (firmware >= 264)
    type += 65;
  writeTripleBytes(ASCII_GS, 'H', 2);    // Print label below barcode
  writeTripleBytes(ASCII_GS, 'w', 3);    // Barcode width 3 (0.375/1.0mm thin/thick)
  writeTripleBytes(ASCII_GS, 'k', type); // Barcode type (listed in .h file)
  if (firmware >= 264) {
    int len = strlen(text);
    if (len > 255)
      len = 255;
    writeBytes(len); // Write length byte
    for (uint8_t i = 0; i < len; i++)
      writeBytes(text[i]); // Write string sans NUL
  } else {
    uint8_t c, i = 0;
    do { // Copy string + NUL terminator
      writeBytes(c = text[i++]);
    } while (c);
  }
  timeoutSet((barcodeHeight + 40) * dotPrintTime);
  prevByte = '\n';
}

// === Character commands ===
#define FONT_MASK (1 << 0) //!< Select character font A or B
#define INVERSE_MASK                                                           \
  (1 << 1) //!< Turn on/off white/black reverse printing mode. Not in 2.6.8
           //!< firmware (see inverseOn())
#define UPDOWN_MASK (1 << 2)        //!< Turn on/off upside-down printing mode
#define BOLD_MASK (1 << 3)          //!< Turn on/off bold printing mode
#define DOUBLE_HEIGHT_MASK (1 << 4) //!< Turn on/off double-height printing mode
#define DOUBLE_WIDTH_MASK (1 << 5)  //!< Turn on/off double-width printing mode
#define STRIKE_MASK (1 << 6)        //!< Turn on/off deleteline mode

void adjustCharValues(uint8_t printMode) {
  uint8_t charWidth;
  if (printMode & FONT_MASK) {
    // FontB
    charHeight = 17;
    charWidth = 9;
  } else {
    // FontA
    charHeight = 24;
    charWidth = 12;
  }
  // Double Width Mode
  if (printMode & DOUBLE_WIDTH_MASK) {
    maxColumn /= 2;
    charWidth *= 2;
  }
  // Double Height Mode
  if (printMode & DOUBLE_HEIGHT_MASK) {
    charHeight *= 2;
  }
  maxColumn = (384 / charWidth);
}

void setPrintMode(uint8_t mask) {
  printMode |= mask;
  writePrintMode();
  adjustCharValues(printMode);
  // charHeight = (printMode & DOUBLE_HEIGHT_MASK) ? 48 : 24;
  // maxColumn = (printMode & DOUBLE_WIDTH_MASK) ? 16 : 32;
}

void unsetPrintMode(uint8_t mask) {
  printMode &= ~mask;
  writePrintMode();
  adjustCharValues(printMode);
  // charHeight = (printMode & DOUBLE_HEIGHT_MASK) ? 48 : 24;
  // maxColumn = (printMode & DOUBLE_WIDTH_MASK) ? 16 : 32;
}

void writePrintMode() {
  writeTripleBytes(ASCII_ESC, '!', printMode);
}

void normal() {
  printMode = 0;
  writePrintMode();
}

void inverseOn() {
  if (firmware >= 268) {
    writeTripleBytes(ASCII_GS, 'B', 1);
  } else {
    setPrintMode(INVERSE_MASK);
  }
}

void inverseOff() {
  if (firmware >= 268) {
    writeTripleBytes(ASCII_GS, 'B', 0);
  } else {
    unsetPrintMode(INVERSE_MASK);
  }
}

void upsideDownOn() {
  if (firmware >= 268) {
    writeTripleBytes(ASCII_ESC, '{', 1);
  } else {
    setPrintMode(UPDOWN_MASK);
  }
}

void upsideDownOff() {
  if (firmware >= 268) {
    writeTripleBytes(ASCII_ESC, '{', 0);
  } else {
    unsetPrintMode(UPDOWN_MASK);
  }
}

void doubleHeightOn() { setPrintMode(DOUBLE_HEIGHT_MASK); }

void doubleHeightOff() { unsetPrintMode(DOUBLE_HEIGHT_MASK); }

void doubleWidthOn() { setPrintMode(DOUBLE_WIDTH_MASK); }

void doubleWidthOff() { unsetPrintMode(DOUBLE_WIDTH_MASK); }

void strikeOn() { setPrintMode(STRIKE_MASK); }

void strikeOff() { unsetPrintMode(STRIKE_MASK); }

void boldOn() { setPrintMode(BOLD_MASK); }

void boldOff() { unsetPrintMode(BOLD_MASK); }

void justify(char value) {
  uint8_t pos = 0;

  switch (toupper(value)) {
  case 'L':
    pos = 0;
    break;
  case 'C':
    pos = 1;
    break;
  case 'R':
    pos = 2;
    break;
  }

  writeTripleBytes(ASCII_ESC, 'a', pos);
}

// Feeds by the specified number of lines
void feed(uint8_t x) {
  if (firmware >= 264) {
    writeTripleBytes(ASCII_ESC, 'd', x);
    timeoutSet(dotFeedTime * charHeight);
    prevByte = '\n';
    column = 0;
  } else {
    while (x--)
      write('\n'); // Feed manually; old firmware feeds excess lines
  }
}

// Feeds by the specified number of individual pixel rows
void feedRows(uint8_t rows) {
  writeTripleBytes(ASCII_ESC, 'J', rows);
  timeoutSet(rows * dotFeedTime);
  prevByte = '\n';
  column = 0;
}

void flush() { writeBytes(ASCII_FF); }

void setSize(char value) {
  uint8_t size;

  switch (toupper(value)) {
  default: // Small: standard width and height
    // size = 0x00;
    // charHeight = 24;
    // maxColumn = 32;
    doubleWidthOff();
    doubleHeightOff();
    break;
  case 'M': // Medium: double height
    // size = 0x01;
    // charHeight = 48;
    // maxColumn = 32;
    doubleHeightOn();
    doubleWidthOff();
    break;
  case 'L': // Large: double width and height
    // size = 0x11;
    // charHeight = 48;
    // maxColumn = 16;
    doubleHeightOn();
    doubleWidthOn();
    break;
  }

  // writeBytes(ASCII_GS, '!', size);
  // prevByte = '\n'; // Setting the size adds a linefeed
}

// ESC 7 n1 n2 n3 Setting Control Parameter Command
// n1 = "max heating dots" 0-255 -- max number of thermal print head
//      elements that will fire simultaneously.  Units = 8 dots (minus 1).
//      Printer default is 7 (64 dots, or 1/6 of 384-dot width), this code
//      sets it to 11 (96 dots, or 1/4 of width).
// n2 = "heating time" 3-255 -- duration that heating dots are fired.
//      Units = 10 us.  Printer default is 80 (800 us), this code sets it
//      to value passed (default 120, or 1.2 ms -- a little longer than
//      the default because we've increased the max heating dots).
// n3 = "heating interval" 0-255 -- recovery time between groups of
//      heating dots on line; possibly a function of power supply.
//      Units = 10 us.  Printer default is 2 (20 us), this code sets it
//      to 40 (throttled back due to 2A supply).
// More heating dots = more peak current, but faster printing speed.
// More heating time = darker print, but slower printing speed and
// possibly paper 'stiction'.  More heating interval = clearer print,
// but slower printing speed.
void setHeatConfig(uint8_t dots, uint8_t time,
                                     uint8_t interval) {
  writeDoubleBytes(ASCII_ESC, '7');       // Esc 7 (print settings)
  writeTripleBytes(dots, time, interval); // Heating dots, heat time, heat interval
}

// Print density description from manual:
// DC2 # n Set printing density
// D4..D0 of n is used to set the printing density.  Density is
// 50% + 5% * n(D4-D0) printing density.
// D7..D5 of n is used to set the printing break time.  Break time
// is n(D7-D5)*250us.
// (Unsure of the default value for either -- not documented)
void setPrintDensity(uint8_t density, uint8_t breakTime) {
  writeTripleBytes(ASCII_DC2, '#', (density << 5) | breakTime);
}

// Underlines of different weights can be produced:
// 0 - no underline
// 1 - normal underline
// 2 - thick underline
void underlineOn(uint8_t weight) {
  if (weight > 2)
    weight = 2;
  writeTripleBytes(ASCII_ESC, '-', weight);
}

void underlineOff() { writeTripleBytes(ASCII_ESC, '-', 0); }

void printBitmapFromBitMap(int w, int h, const uint8_t *bitmap,
                                   bool fromProgMem) {
  int rowBytes, rowBytesClipped, rowStart, chunkHeight, chunkHeightLimit, x, y,
      i;

  rowBytes = (w + 7) / 8; // Round up to next byte boundary
  rowBytesClipped = (rowBytes >= 48) ? 48 : rowBytes; // 384 pixels max width

  chunkHeightLimit = 256 / rowBytesClipped;
  if (chunkHeightLimit > maxChunkHeight)
    chunkHeightLimit = maxChunkHeight;
  else if (chunkHeightLimit < 1)
    chunkHeightLimit = 1;

  for (i = rowStart = 0; rowStart < h; rowStart += chunkHeightLimit) {
    // Issue up to chunkHeightLimit rows at a time:
    chunkHeight = h - rowStart;
    if (chunkHeight > chunkHeightLimit)
      chunkHeight = chunkHeightLimit;

    writeQuadBytes(ASCII_DC2, '*', chunkHeight, rowBytesClipped);

    for (y = 0; y < chunkHeight; y++) {
      for (x = 0; x < rowBytesClipped; x++, i++) {
        timeoutWait();
        KP347_SEND_BYTE(fromProgMem ? pgm_read_byte(bitmap + i) : *(bitmap + i));
      }
      i += rowBytes - rowBytesClipped;
    }
    timeoutSet(chunkHeight * dotPrintTime);
  }
  prevByte = '\n';
}

void printBitmapFromStream(int w, int h) {
  int rowBytes, rowBytesClipped, rowStart, chunkHeight, chunkHeightLimit, x, y,
      i, c;

  rowBytes = (w + 7) / 8; // Round up to next byte boundary
  rowBytesClipped = (rowBytes >= 48) ? 48 : rowBytes; // 384 pixels max width

  chunkHeightLimit = 256 / rowBytesClipped;
  if (chunkHeightLimit > maxChunkHeight)
    chunkHeightLimit = maxChunkHeight;
  else if (chunkHeightLimit < 1)
    chunkHeightLimit = 1;

  for (rowStart = 0; rowStart < h; rowStart += chunkHeightLimit) {
    // Issue up to chunkHeightLimit rows at a time:
    chunkHeight = h - rowStart;
    if (chunkHeight > chunkHeightLimit)
      chunkHeight = chunkHeightLimit;

    writeQuadBytes(ASCII_DC2, '*', chunkHeight, rowBytesClipped);

    for (y = 0; y < chunkHeight; y++) {
      for (x = 0; x < rowBytesClipped; x++) {
        while ((c = KP347_STREAM_READ()) < 0)
          ;
        timeoutWait();
        KP347_SEND_BYTE((uint8_t)c);
      }
      for (i = rowBytes - rowBytesClipped; i > 0; i--) {
        while ((c = KP347_STREAM_READ()) < 0)
          ;
      }
    }
    timeoutSet(chunkHeight * dotPrintTime);
  }
  prevByte = '\n';
}

void printBitmap() {
  uint8_t tmp;
  uint16_t width, height;

  tmp = KP347_STREAM_READ();
  width = (KP347_STREAM_READ() << 8) + tmp;

  tmp = KP347_STREAM_READ();
  height = (KP347_STREAM_READ() << 8) + tmp;

  printBitmapFromStream(width, height);
}

// Take the printer offline. Print commands sent after this will be
// ignored until 'online' is called.
void offline() { writeTripleBytes(ASCII_ESC, '=', 0); }

// Take the printer back online. Subsequent print commands will be obeyed.
void online() { writeTripleBytes(ASCII_ESC, '=', 1); }

// Put the printer into a low-energy state immediately.
void sleep() {
  sleepAfter(1); // Can't be 0, that means 'don't sleep'
}

// Put the printer into a low-energy state after the given number
// of seconds.
void sleepAfter(uint16_t seconds) {
  if (firmware >= 264) {
    writeQuadBytes(ASCII_ESC, '8', seconds, seconds >> 8);
  } else {
    writeTripleBytes(ASCII_ESC, '8', seconds);
  }
}

// Wake the printer from a low-energy state.
void wake() {
  timeoutSet(0);   // Reset timeout counter
  writeBytes(255); // Wake
  if (firmware >= 264) {
    delay(50);
    writeQuadBytes(ASCII_ESC, '8', 0, 0); // Sleep off (important!)
  } else {
    // Datasheet recommends a 50 mS delay before issuing further commands,
    // but in practice this alone isn't sufficient (e.g. text size/style
    // commands may still be misinterpreted on wake).  A slightly longer
    // delay, interspersed with NUL chars (no-ops) seems to help.
    for (uint8_t i = 0; i < 10; i++) {
      writeBytes(0);
      timeoutSet(10000L);
    }
  }
}

// Check the status of the paper using the printer's self reporting
// ability.  Returns true for paper, false for no paper.
// Might not work on all printers!
bool hasPaper() {
  if (firmware >= 264) {
    writeTripleBytes(ASCII_ESC, 'v', 0);
  } else {
    writeTripleBytes(ASCII_GS, 'r', 0);
  }

  int status = -1;
  for (uint8_t i = 0; i < 10; i++) {
    if (KP347_IS_AVAILABLE()) {
      status = KP347_RECEIVE();
      break;
    }
    delay(100);
  }

  return !(status & 0b00000100);
}

void setLineHeight(int val) {
  if (val < 24)
    val = 24;
  lineSpacing = val - 24;

  // The printer doesn't take into account the current text height
  // when setting line height, making this more akin to inter-line
  // spacing.  Default line spacing is 30 (char height of 24, line
  // spacing of 6).
  writeTripleBytes(ASCII_ESC, '3', val);
}

void setMaxChunkHeight(int val) { maxChunkHeight = val; }

// These commands work only on printers w/recent firmware ------------------

// Alters some chars in ASCII 0x23-0x7E range; see datasheet
void setCharset(uint8_t val) {
  if (val > 15)
    val = 15;
  writeTripleBytes(ASCII_ESC, 'R', val);
}

// Selects alt symbols for 'upper' ASCII values 0x80-0xFF
void setCodePage(uint8_t val) {
  if (val > 47)
    val = 47;
  writeTripleBytes(ASCII_ESC, 't', val);
}

void tab() {
  writeBytes(ASCII_TAB);
  column = (column + 4) & 0b11111100;
}

void setFont(char font) {
  switch (toupper(font)) {
  case 'B':
    setPrintMode(FONT_MASK);
    break;
  case 'A':
  default:
    unsetPrintMode(FONT_MASK);
  }
}

void setCharSpacing(int spacing) {
  writeTripleBytes(ASCII_ESC, ' ', spacing);
}

// -------------------------------------------------------------------------
