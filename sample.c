//
// oled test program
// Written by Larry Bank
#include <stdio.h>

#include "app_util_platform.h"
#include "app_error.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_delay.h"

#include "oled96.h"

int main()
{
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("TWI started.");
    NRF_LOG_FLUSH();

    int iOLEDType = OLED_128x64; // Change this for your specific display
    int bFlip = 0, bInvert = 0;
    oledInit(iOLEDType, bFlip, bInvert);

    oledFill(0); // fill with black
    oledWriteString(0,0,"OLED 96 Library!",FONT_NORMAL);
    oledWriteString(3,1,"BIG!",FONT_BIG);
    oledWriteString(0,1,"Small", FONT_SMALL);
} /* main() */
