#ifndef ESPCYD_H_
#define ESPCYD_H_
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>
#include <SPI.h>

#include "time.h"
#include "canbus_msg.h"


/*  Install the "TFT_eSPI" library by Bodmer to interface with the TFT Display - https://github.com/Bodmer/TFT_eSPI
    *** IMPORTANT: User_Setup.h available on the internet will probably NOT work with the examples available at Random Nerd Tutorials ***
    *** YOU MUST USE THE User_Setup.h FILE PROVIDED IN THE LINK BELOW IN ORDER TO USE THE EXAMPLES FROM RANDOM NERD TUTORIALS ***
    FULL INSTRUCTIONS AVAILABLE ON HOW CONFIGURE THE LIBRARY: https://RandomNerdTutorials.com/cyd/ or https://RandomNerdTutorials.com/esp32-tft/   */
#include <TFT_eSPI.h>

/** Install the "XPT2046_Touchscreen" library by Paul Stoffregen to use the Touchscreen - https://github.com/PaulStoffregen/XPT2046_Touchscreen
*   Note: this library doesn't require further configuration */
#include <XPT2046_Touchscreen.h>

/** Touchscreen pins, this is setup in build_flags in platformio.ini */
#define XPT2046_IRQ 36   /* T_IRQ */
#define XPT2046_MOSI 32  /* T_DIN */
#define XPT2046_MISO 39  /* T_OUT */
#define XPT2046_CLK 25   /* T_CLK */
#define XPT2046_CS 33    /* T_CS */

#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240
#define FONT_SIZE 2

/** Cheap yellow display pin assignments */
#define LED_RED           4                
#define LED_BLUE          17
#define LED_GREEN         16
#define CYD_BACKLIGHT     21 
#define CYD_LDR           34
#define CYD_SPEAKER       26

/* Externalized variables for use in main logic if needed */
extern TFT_eSPI tft;
extern volatile bool can_driver_installed;
extern volatile bool can_suspended;

/* Set X and Y coordinates for center of display */
const int centerX = SCREEN_WIDTH / 2;
const int centerY = SCREEN_HEIGHT / 2;


/* Modular initialization function */
void initCYD();

struct TouchData {
  int x;
  int y;
  int z;
};

struct KeypadButton {
    int x, y, w, h;
    char label[10];
    uint16_t canID;
    uint16_t color;
};

extern KeypadButton buttons[4];

#endif  /* End ESPCYD_H_ */