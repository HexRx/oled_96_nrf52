## OLED_96 (NRF52)

OLED_96 is a simple C library for working with SSD1306/SH1106 OLED displays
when connected to the I2C bus. Tested on various ARM SBCs and Arduinos.
The idea is to provide a simple interface for C programmers to make use of 
those low-cost OLED displays without having to read the data sheet or figure
out I2C programming.

Written by Larry Bank
Project started 1/15/2017
bitbank@pobox.com

## How to use
To build `example.c` for nrf52832 you need:
1. Change the `SDK_ROOT` to your nRF5_SDK path.
1. Change if needed flash program in Makefile, the default option is ST-Link (openocd)
    ```
    openocd -f interface/stlink.cfg -f target/nrf52.cfg -c 'init_reset halt; program $(OUTPUT_DIRECTORY)/nrf52832_xxaa.hex verify; reset; exit'
    ```
   If you use J-Link (openocd) you need to change it to
   ```
    openocd -f interface/jlink.cfg -c "transport select swd" -f target/nrf52.cfg -c 'init_reset halt; program $(OUTPUT_DIRECTORY)/nrf52832_xxaa.hex verify; reset; exit'
    ```
1. Run `make flash` to build and flash.

The default parameters:
* OLED address `0x3c`
* `SDA_PIN` 26 (P0.26) 
* `SCL_PIN` 27 (P0.27) 

These values can be changed in the `oled96.h`:
``` c
#define SCL_PIN 27
#define SDA_PIN 26

#define I2C_ADDR 0x3c
```

## Latest Changes:
* Change the library to support NRF52 MCUs

## Features:
- Efficient individual pixel drawing
- Small, Medium and large fonts (6x8, 8x8, 16x32)
- Invert and flip 180 options (entire display)
- Optional inverted text
- Brightness control
- Support for 64x32, 128x32, 128x64 and 132x64 (SH1106) displays
- Line draw function

---

If you find this code useful, please consider buying me a cup of coffee

[![paypal](https://www.paypalobjects.com/en_US/i/btn/btn_donateCC_LG.gif)](https://www.paypal.com/cgi-bin/webscr?cmd=_s-xclick&hosted_button_id=SR4F44J2UR8S4)
