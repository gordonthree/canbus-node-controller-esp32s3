#include <WiFi.h>
#include "espcyd.h"

TFT_eSPI tft = TFT_eSPI();
SPIClass touchscreenSPI = SPIClass(VSPI);
XPT2046_Touchscreen touchscreen(XPT2046_CS, XPT2046_IRQ);

SemaphoreHandle_t spiSemaphore;
QueueHandle_t touchQueue;
QueueHandle_t timeQueue;

/* Forward declarations for tasks */
void TaskReadTouch(void * pvParameters);
void TaskUpdateDisplay(void * pvParameters);

// Touchscreen coordinates: (x, y) and pressure (z)
int x, y, z;

// Global variables for inter-task communication
volatile int globalX, globalY, globalZ;
volatile bool newData = false;

// used to print IP on the display
extern String wifiIP;

/* can tx function in main.cpp */
extern void send_message(uint16_t msgid, uint8_t *data, uint8_t dlc);

/* node ID for the data payload */
extern volatile uint8_t myNodeID[4];

// Task Handles
// TaskHandle_t TaskTouchHandle;
// TaskHandle_t TaskDisplayHandle;

extern bool wifi_connected;

KeypadButton buttons[4] = {
    {10,  50,  145, 70, "LIGHTS", 0, TFT_BLUE},
    {165, 50,  145, 70, "WIPERS", 1, TFT_DARKGREEN},
    {10,  130, 145, 70, "HORN",   2, TFT_RED},
    {165, 130, 145, 70, "AUX",    3, TFT_ORANGE}
};


void initCYD() {
    spiSemaphore = xSemaphoreCreateBinary(); /* semaphore to control SPI access */
    xSemaphoreGive(spiSemaphore); /* unlock SPI access */
    
    touchQueue = xQueueCreate(5, sizeof(TouchData));
    timeQueue = xQueueCreate(1, 10 * sizeof(char));

    Serial.println("CYD: Init");

    /* Power on the backlight */
    pinMode(CYD_BACKLIGHT, OUTPUT);
    digitalWrite(CYD_BACKLIGHT, HIGH);

    tft.begin(); /* initialize the display */
    tft.setRotation(1);
    /* Clear the screen before writing to it */
    tft.fillScreen(TFT_WHITE);
    tft.setTextColor(TFT_BLACK, TFT_WHITE);

    /* Setup the touchscreen */
    touchscreenSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS); /* configure SPI interface for touchscreen */
    touchscreen.begin(touchscreenSPI);
    touchscreen.setRotation(1);
  
    /* Start the tasks */
    xTaskCreate(TaskReadTouch, "TouchTask", 4096, NULL, 2, NULL);
    xTaskCreate(TaskUpdateDisplay, "DisplayTask", 4096, NULL, 1, NULL);
}

/**
 * @brief Draws a status indicator for the CAN bus
 * @details Assumes spiSemaphore is ALREADY HELD. Routine to draw a green or 
 *          red circle in the top corner to indicate CAN health.
 * @param connected Boolean indicating if the driver is active and running
 */
void drawCANStatus(bool active) {
    /* Positioned in the header bar: x=300, y=21 */
    uint16_t color = active ? TFT_GREEN : TFT_RED;
    
    /* Draw a background to clear old state */
    tft.fillCircle(300, 21, 9, TFT_BLUE); 
    
    /* Draw the status indicator */
    tft.fillCircle(300, 21, 7, color);
    tft.drawCircle(300, 21, 7, TFT_WHITE);
}

/**
 * @brief Internal helper function to draw the keypad buttons on the screen
 * @details Assumes spiSemaphore is ALREADY HELD. Draws a black background, then
 *          draws each button as a rounded rectangle with white outline and
 *          colored fill. Then draws the button label in white at the button's
 *          center.
 */
void drawKeypad() {
  /* Internal helper - assumes spiSemaphore is ALREADY HELD */
  tft.fillScreen(TFT_BLACK);
    for (int i = 0; i < 4; i++) {
        tft.fillRoundRect(buttons[i].x, buttons[i].y, buttons[i].w, buttons[i].h, 8, buttons[i].color);
        tft.drawRoundRect(buttons[i].x, buttons[i].y, buttons[i].w, buttons[i].h, 8, TFT_WHITE);
        tft.setTextColor(TFT_WHITE);
        tft.drawCentreString(buttons[i].label, buttons[i].x + (buttons[i].w / 2), buttons[i].y + (buttons[i].h / 2) - 8, 2);
    }
}

/** Task 1: Read Touch */
void TaskReadTouch(void * pvParameters) {
  TouchData currentTouch;
  Serial.println("CYD: Touch Task Started");

  for(;;) {
    /* Only process touch if CAN is healthy and not suspended */
    if (can_driver_installed && !can_suspended) {
      /* Try to take the mutex (wait up to 10ms if busy) */
      if (touchscreen.tirqTouched() && touchscreen.touched()) {
        if (spiSemaphore != NULL && xSemaphoreTake(spiSemaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
          TS_Point p = touchscreen.getPoint();
          currentTouch.x = map(p.x, 200, 3700, 1, SCREEN_WIDTH);
          currentTouch.y = map(p.y, 240, 3800, 1, SCREEN_HEIGHT);
          currentTouch.z = p.z;
          
          if (p.z > 800) { /* Only queue if the press is firm enough */
            xQueueSend(touchQueue, &currentTouch, 0);
          }

          /* Always give the mutex back! */
          xSemaphoreGive(spiSemaphore);
        }
      }
    } else {
      /* Optional: Clear queue if bus drops to prevent latent actions */
      xQueueReset(touchQueue);
    }
    digitalWrite(LED_GREEN, !digitalRead(LED_GREEN)); /* Toggle the green LED */
    vTaskDelay(pdMS_TO_TICKS(20)); /* High polling rate for touch */
  }
}

/** Task 2: Update Display */
void TaskUpdateDisplay(void * pvParameters) {
  TouchData receivedTouch;
  char receivedTime[10];
  uint32_t lastPressTime = 0; 
  const uint32_t debounceDelay = 750; /**< Milliseconds to wait between valid presses */
  static bool buttons_drawn = false;
  static bool ip_drawn = false;
  static uint32_t lastCANCheck = 0;  /**< Tracks the last time we checked the bus */
  static bool lastCANState = false;  /**< Tracks the previous state to detect changes */  
  
  /* Variables for local time polling */
  static uint32_t lastTimeUpdate = 0;
  struct tm timeinfo;
  char timeString[10];

  Serial.println("CYD: Display Task Started");
  digitalWrite(LED_BLUE, LOW); /* Turn on the blue LED */

  for(;;) {
    uint32_t currentMillis = millis();

    /* Initial Draw - Keep trying until the buttons get drawn. */
    if (!buttons_drawn) {
      if (xSemaphoreTake(spiSemaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
        /* Draw initial keypad buttons and labels */
        drawKeypad();
        /* Draw initial CAN status immediately so it isn't blank for the first 500ms */
        drawCANStatus(can_driver_installed && !can_suspended);        
        digitalWrite(LED_BLUE, HIGH); /* Turn off the blue LED */
        xSemaphoreGive(spiSemaphore);
        buttons_drawn = true;
      }
    }

    /* 2. Direct RTC Polling (Every 1 Second) */
    if (currentMillis - lastTimeUpdate >= 1000) {
      lastTimeUpdate = currentMillis;
      
      if (getLocalTime(&timeinfo)) {
        strftime(timeString, sizeof(timeString), "%H:%M:%S", &timeinfo);
        
        if (xSemaphoreTake(spiSemaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
          /* Draw Header */
          tft.fillRect(0, 0, 320, 43, TFT_BLUE);
          tft.setTextColor(TFT_WHITE, TFT_BLUE);
          tft.drawCentreString(timeString, 160, 10, 4);
          
          /* Keep CAN Status visible on top of header */
          drawCANStatus(can_driver_installed && !can_suspended);
          
          /* Footer/IP refresh logic if needed */
          if (wifi_connected && !ip_drawn) {
            tft.fillRect(0, 210, 320, 30, TFT_DARKGREY);
            tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
            tft.drawCentreString("IP: " + wifiIP, 160, 215, 2);
            ip_drawn = true;
          }
          xSemaphoreGive(spiSemaphore);
        }
      }
    }
    

    /* Only run this block if 500ms has passed since the last check */
    if (currentMillis - lastCANCheck >= 500) {
        lastCANCheck = currentMillis;

        /* Combined check: Is the driver active AND not in a suspended error state? */
        bool currentCANState = (can_driver_installed && !can_suspended);

        /* * We only redraw if the state actually changed. 
        * This prevents flickering and saves SPI bandwidth for other tasks.
        */
        if (currentCANState != lastCANState) {
            if (xSemaphoreTake(spiSemaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
                drawCANStatus(currentCANState);
                xSemaphoreGive(spiSemaphore);
                lastCANState = currentCANState;
                
                Serial.printf("CAN Status changed to: %s\n", currentCANState ? "ONLINE" : "OFFLINE");
            }
        }
    }    
    /* Check for Touch Data */
    if (xQueueReceive(touchQueue, &receivedTouch, 0)) {
      uint32_t currentTime = millis();
      /* Only process if enough time has passed */
      if (currentTime - lastPressTime > debounceDelay) {
        for (int i = 0; i < 4; i++) {
              /* Collision Detection: Is the touch inside button[i]? */
              if (receivedTouch.x >= buttons[i].x && receivedTouch.x <= (buttons[i].x + buttons[i].w) &&
                  receivedTouch.y >= buttons[i].y && receivedTouch.y <= (buttons[i].y + buttons[i].h)) {
                  
                  /* Visual Feedback: Flash the button */
                  if (xSemaphoreTake(spiSemaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
                      tft.drawRoundRect(buttons[i].x, buttons[i].y, buttons[i].w, buttons[i].h, 8, TFT_RED);
                      xSemaphoreGive(spiSemaphore);
                  }

                  /* --- Momentary CAN Transmit --- */
                  uint8_t canData[5];
                  /* Bytes 0:3 - Node ID */
                  memcpy(canData, (void*)myNodeID, 4);

                  /* Byte 4 - Button ID (from the struct) */
                  canData[4] = (uint8_t)buttons[i].canID;

                  /* All buttons transmit on 0x115 */
                  send_message(SW_MOM_PRESS_ID, canData, SW_MOM_PRESS_DLC);
                  
                  Serial.printf("Sent Button %d on CAN 0x115\n", (int)buttons[i].canID);

                  /* 1. Update the timestamp immediately to "lock" further presses */
                  lastPressTime = currentTime;

                  vTaskDelay(pdMS_TO_TICKS(250)); // Simple debouncing
                  
                  /* Reset border color */
                  if (xSemaphoreTake(spiSemaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
                      tft.drawRoundRect(buttons[i].x, buttons[i].y, buttons[i].w, buttons[i].h, 8, TFT_WHITE);
                      xSemaphoreGive(spiSemaphore);
                  }
              }
          }
        }
      }

      // Check for Time Data

      if (xQueueReceive(timeQueue, &receivedTime, pdMS_TO_TICKS(50))) {
        if (xSemaphoreTake(spiSemaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
        tft.fillRect(0, 0, 320, 43, TFT_BLUE); // Header bar
        tft.setTextColor(TFT_WHITE, TFT_BLUE);
        tft.drawCentreString(receivedTime, 160, 10, 4);

        // Footer / IP Address (Only needs to draw once or when changed)
        if (WiFi.status() == WL_CONNECTED) {
          tft.fillRect(0, 210, 320, 30, TFT_DARKGREY);
          tft.setTextColor(TFT_WHITE, TFT_DARKGREY);
          tft.drawCentreString("IP: " + wifiIP, 160, 215, 2);
        }
        xSemaphoreGive(spiSemaphore);
        }
      }

    /* 3. Explicitly yield to the IDLE task */
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

