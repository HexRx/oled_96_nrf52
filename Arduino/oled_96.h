#define FONT_NORMAL 0
#define FONT_LARGE 1
#define FONT_SMALL 2
//
// Initializes the OLED controller into "page mode"
//
void oledInit(int iAddr, int bFlip, int bInvert);
//
// Sends a command to turn off the OLED display
//
void oledShutdown();
//
// Sets the brightness (0=off, 255=brightest)
//
void oledSetContrast(unsigned char ucContrast);
//
// Draw a string of normal (8x8), small (6x8) or large (16x32) characters
// At the given col+row
//
int oledWriteString(int x, int y, char *szMsg, int iSize);
//
// Fill the frame buffer with a byte pattern
// e.g. all off (0x00) or all on (0xff)
//
void oledFill(unsigned char ucData);
