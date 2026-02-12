#ifndef ESPCYD_H_
#define ESPCYD_H_
#include <Arduino.h>
#include <ArduinoOTA.h>
#include <WiFi.h>
#include "time.h"

#include <SPI.h>

/*  Install the "TFT_eSPI" library by Bodmer to interface with the TFT Display - https://github.com/Bodmer/TFT_eSPI
    *** IMPORTANT: User_Setup.h available on the internet will probably NOT work with the examples available at Random Nerd Tutorials ***
    *** YOU MUST USE THE User_Setup.h FILE PROVIDED IN THE LINK BELOW IN ORDER TO USE THE EXAMPLES FROM RANDOM NERD TUTORIALS ***
    FULL INSTRUCTIONS AVAILABLE ON HOW CONFIGURE THE LIBRARY: https://RandomNerdTutorials.com/cyd/ or https://RandomNerdTutorials.com/esp32-tft/   */
#include <TFT_eSPI.h>

// Install the "XPT2046_Touchscreen" library by Paul Stoffregen to use the Touchscreen - https://github.com/PaulStoffregen/XPT2046_Touchscreen
// Note: this library doesn't require further configuration
#include <XPT2046_Touchscreen.h>

// Touchscreen pins
#define XPT2046_IRQ 36   // T_IRQ
#define XPT2046_MOSI 32  // T_DIN
#define XPT2046_MISO 39  // T_OUT
#define XPT2046_CLK 25   // T_CLK
#define XPT2046_CS 33    // T_CS

#define SCREEN_WIDTH 320
#define SCREEN_HEIGHT 240
#define FONT_SIZE 2

// Task Handles
TaskHandle_t TaskTouchHandle;
TaskHandle_t TaskDisplayHandle;

// Mutex
SemaphoreHandle_t spiSemaphore; // This is our "talking stick"

struct TouchData {
  int x;
  int y;
  int z;
};

struct KeypadButton {
    int x, y, w, h;
    char label[10];
    uint32_t canID;
    uint16_t color;
};

KeypadButton buttons[4] = {
    {10,  50,  145, 70, "LIGHTS", 0x101, TFT_BLUE},
    {165, 50,  145, 70, "WIPERS", 0x102, TFT_DARKGREEN},
    {10,  130, 145, 70, "HORN",   0x103, TFT_RED},
    {165, 130, 145, 70, "AUX",    0x104, TFT_ORANGE}
};


// Create a handle for the mailbox
QueueHandle_t touchQueue;
QueueHandle_t timeQueue;



#endif  // End ESPCYD_H_