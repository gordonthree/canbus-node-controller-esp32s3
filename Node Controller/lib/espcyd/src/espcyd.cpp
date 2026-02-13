#include <WiFi.h>
#include "espcyd.h"

/* espcyd.cpp */

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

/* CAN interface status from main.cpp */
extern volatile bool can_suspended;
extern volatile bool can_driver_installed;

// used to print IP on the display
extern String wifiIP;

/* Variables for the color picker routines - from main.cpp*/
// extern DisplayMode currentMode;
// extern ARGBNode discoveredNodes[5];
// extern int selectedNodeIdx;
DisplayMode currentMode = MODE_HOME; /**< Current display mode */
int selectedNodeIdx = 0;     /**< Currently targeted node for color commands */
ARGBNode discoveredNodes[5]; /**< List of discovered ARGB nodes */

/* can tx function in main.cpp */
extern void send_message(uint16_t msgid, uint8_t *data, uint8_t dlc);

/* node ID for the data payload */
extern volatile uint8_t myNodeID[4];

extern bool wifi_connected;

KeypadButton buttons[4] = {
    {10,  50,  145, 70, "LIGHTS", 0, TFT_BLUE},
    {165, 50,  145, 70, "WIPERS", 1, TFT_DARKGREEN},
    {10,  130, 145, 70, "HORN",   2, TFT_RED},
    {165, 130, 145, 70, "AUX",    3, TFT_ORANGE}
};

/**
 * @brief Menu items for the hamburger menu
 */
const char* menuLabels[] = {"HOME", "COLOR PICKER", "NODE SELECT", "SYSTEM INFO"};

void initCYD() {
    spiSemaphore = xSemaphoreCreateBinary(); /* semaphore to control SPI access */
    xSemaphoreGive(spiSemaphore); /* unlock SPI access */
    
    touchQueue = xQueueCreate(5, sizeof(TouchData));
    timeQueue = xQueueCreate(1, 10 * sizeof(char));

    Serial.println("CYD: Init");

    /* Clear the discovered nodes array to prevent garbage data on UI */
    memset(discoveredNodes, 0, sizeof(discoveredNodes));

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
 * @brief Draws the hamburger icon in the top-right
 */
void drawHamburgerIcon() {
    int x = 280;
    int y = 12;
    tft.fillRect(x, y, 25, 4, TFT_WHITE);
    tft.fillRect(x, y + 8, 25, 4, TFT_WHITE);
    tft.fillRect(x, y + 16, 25, 4, TFT_WHITE);
}

/**
 * @brief Draws the main navigation menu
 */
void drawHamburgerMenu() {
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE);
    tft.drawCentreString("MAIN MENU", 160, 20, 4);

    for (int i = 0; i < 4; i++) {
        int yPos = 60 + (i * 45);
        tft.fillRoundRect(40, yPos, 240, 35, 5, TFT_DARKCYAN);
        tft.drawRoundRect(40, yPos, 240, 35, 5, TFT_WHITE);
        tft.drawCentreString(menuLabels[i], 160, yPos + 8, 2);
    }
}

/**
 * @brief Draws the list of discovered nodes with a color status square
 */
void drawNodeSelector() {
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE);
    tft.drawCentreString("SELECT TARGET NODE", 160, 10, 2);

    for (int i = 0; i < 5; i++) {
        int yPos = 45 + (i * 38);
        uint16_t boxColor = (selectedNodeIdx == i) ? TFT_YELLOW : TFT_BLACK;
        
        /* Draw selection row container */
        tft.drawRect(5, yPos - 2, 310, 34, boxColor);

        if (discoveredNodes[i].id != 0) {
            char buf[25];
            sprintf(buf, "[%d] ID: 0x%08X", i, discoveredNodes[i].id);
            tft.drawString(buf, 15, yPos + 8, 2);

            /* Draw the small square showing the last sent color */
            int cIdx = discoveredNodes[i].lastColorIdx;
            uint16_t previewColor = tft.color565(SystemPalette[cIdx].r, 
                                                 SystemPalette[cIdx].g, 
                                                 SystemPalette[cIdx].b);
            
            tft.fillRect(280, yPos + 5, 20, 20, previewColor);
            tft.drawRect(280, yPos + 5, 20, 20, TFT_WHITE);
        } else {
            tft.setTextColor(TFT_DARKGREY);
            tft.drawString("--- Empty Slot ---", 15, yPos + 8, 2);
            tft.setTextColor(TFT_WHITE);
        }
    }
    
    /* Footer hint */
    tft.drawCentreString("Tap ID to select node", 160, 225, 1);
}

/**
 * @brief Draws a 32-color selection grid on the CYD
 */
void drawColorPicker() {
    int swatchW = 40;  /**< 320 / 8 columns */
    int swatchH = 45;  /**< 180 / 4 rows (leaving room for header/footer) */
    int startY = 45;   /**< Start below the header bar */

    for (int i = 0; i < 32; i++) {
        int col = i % 8;
        int row = i / 8;
        int x = col * swatchW;
        int y = startY + (row * swatchH);

        /* Convert CRGB to RGB565 for TFT_eSPI */
        uint16_t color565 = tft.color565(SystemPalette[i].r, SystemPalette[i].g, SystemPalette[i].b);
        
        tft.fillRect(x, y, swatchW, swatchH, color565);
        tft.drawRect(x, y, swatchW, swatchH, TFT_WHITE); // Border
    }
}

/**
 * @brief Draws a color palette icon in the header (x=50)
 */
void drawPickerIcon() {
    /* Rainbow-ish 16x16 icon */
    tft.fillRect(50, 12, 8, 8, TFT_RED);
    tft.fillRect(58, 12, 8, 8, TFT_YELLOW);
    tft.fillRect(50, 20, 8, 8, TFT_BLUE);
    tft.fillRect(58, 20, 8, 8, TFT_GREEN);
    tft.drawRect(49, 11, 18, 18, TFT_WHITE);
}

/**
 * @brief Logic to register or update a discovered ARGB node
 * @param id The 32-bit Node ID extracted from the CAN frame
 */
void registerARGBNode(uint32_t id) {
    int emptySlot = -1;

    for (int i = 0; i < 5; i++) {
        /* Case 1: Node already exists in our table */
        if (discoveredNodes[i].id == id) {
            discoveredNodes[i].lastSeen = millis();
            discoveredNodes[i].active = true;
            return;
        }

        /* Keep track of the first available empty slot */
        if (emptySlot == -1 && discoveredNodes[i].id == 0) {
            emptySlot = i;
        }
    }

    /* Case 2: New node discovered and we have room */
    if (emptySlot != -1) {
        discoveredNodes[emptySlot].id = id;
        discoveredNodes[emptySlot].active = true;
        discoveredNodes[emptySlot].lastSeen = millis();
        
        Serial.printf("UI: Registered New ARGB Node [0x%08X] at slot %d\n", id, emptySlot);
    } else {
        /* Case 3: Project limit reached (5 nodes) */
        Serial.println("UI Warning: Discovered node ignored, table full.");
    }
}

/**
 * @brief Draws a simple splash screen while waiting for CAN sync
 */
void drawSplashScreen(const char* message) {
    tft.fillScreen(TFT_BLACK);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawCentreString("INITIALIZING", 160, 100, 4);
    tft.drawCentreString(message, 160, 140, 2);
}

/**
 * @brief Draws a WiFi signal strength indicator in the top left
 * @param rssi The RSSI value from WiFi.RSSI()
 */
void drawWiFiStatus(int32_t rssi) {
    int x = 10;
    int y = 30;
    uint16_t color;

    /* Erase old indicator area in the header */
    tft.fillRect(x, 10, 30, 25, TFT_BLUE);

    /* Determine color based on strength */
    if (rssi > -67) color = TFT_GREEN;       /* Good */
    else if (rssi > -80) color = TFT_YELLOW; /* OK */
    else color = TFT_RED;                    /* Poor */

    /* Draw 4 bars of increasing height */
    for (int i = 0; i < 4; i++) {
        int barHeight = (i + 1) * 4;
        if (rssi > -90 + (i * 10)) {
            tft.fillRect(x + (i * 6), y - barHeight, 4, barHeight, color);
        } else {
            tft.drawRect(x + (i * 6), y - barHeight, 4, barHeight, TFT_WHITE);
        }
    }
}

/**
 * @brief Draws the footer with IP address and NodeID
 */
void drawFooter() {
    /* Erase footer area */
    tft.fillRect(0, 210, 320, 30, TFT_DARKGREY);
    tft.setTextColor(TFT_WHITE, TFT_DARKGREY);

    /* Format NodeID as Hex string (e.g., DEADBEEF) */
    char nodeStr[20];
    sprintf(nodeStr, "ID: %02X%02X%02X%02X", myNodeID[0], myNodeID[1], myNodeID[2], myNodeID[3]);

    /* Draw IP on left, NodeID on right */
    tft.drawString("IP: " + wifiIP, 10, 215, 2);
    tft.drawRightString(nodeStr, 310, 215, 2);
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
  const uint32_t debounceDelay = 750;  /**< Milliseconds to wait between valid presses */
  
  /* flags to keep track of what has been drawn */
  static bool buttons_drawn = false;
  static bool ip_drawn = false;
  static bool footer_drawn = false;

  static uint32_t lastCANCheck = 0;    /**< Tracks the last time we checked the bus */
  static bool lastCANState = false;    /**< Tracks the previous state to detect changes */  
  static bool ui_initialized = false;  /**< Flag to track if the UI has been initialized */
  static int32_t lastRSSI = 0;         /**< WiFi RSSI value for signal strength indicator */

  /* Variables for local time polling */
  static uint32_t lastTimeUpdate = 0;
  struct tm timeinfo;
  char timeString[10];

  Serial.println("CYD: Display Task Started");
  digitalWrite(LED_BLUE, LOW); /* Turn on the blue LED */

  for(;;) {
    uint32_t currentMillis = millis();

    /* STATE 1: Waiting for CAN Introduction Acknowledgement */
    if (!ui_initialized) {
        if (xSemaphoreTake(spiSemaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (FLAG_SEND_INTRODUCTION) {
                drawSplashScreen("Waiting for connection");
            } else {
                drawKeypad();
                drawCANStatus(can_driver_installed && !can_suspended);
                /* Draw footer immediately upon entry to main UI */
                if (wifi_connected) drawFooter();
                ui_initialized = true;
            }
            xSemaphoreGive(spiSemaphore);
        }
        vTaskDelay(pdMS_TO_TICKS(100));
        continue; 
    }

    /* STATE 2: Normal UI Operation */
    /* 1000ms Refresh Loop (Time, WiFi, CAN, Footer) */
    if (currentMillis - lastTimeUpdate >= 1000) {
        lastTimeUpdate = currentMillis;
        /* Check for stale nodes every second */
        for (int i = 0; i < 5; i++) {
            if (discoveredNodes[i].id != 0 && (currentMillis - discoveredNodes[i].lastSeen > 30000)) {
                discoveredNodes[i].active = false;
            }
        }

        if (xSemaphoreTake(spiSemaphore, pdMS_TO_TICKS(50)) == pdTRUE) {
            /* Clear Header area only, blue background */
            tft.fillRect(0, 0, 320, 43, TFT_BLUE);
            
            /* Draw time - white text blue background */
            if (getLocalTime(&timeinfo)) {
                strftime(timeString, sizeof(timeString), "%H:%M:%S", &timeinfo);
                tft.setTextColor(TFT_WHITE, TFT_BLUE);
                tft.drawCentreString(timeString, 160, 10, 4);
            }

            /* Draw System Icons */
            drawWiFiStatus(WiFi.RSSI()); /**< WiFi Status Icon */
            drawCANStatus(can_driver_installed && !can_suspended); /**< CAN Status Icon */
            drawPickerIcon(); /**< Color Picker Icon */

            /* Draw Selection Context */
            if (discoveredNodes[selectedNodeIdx].id != 0) {
                char nodeLbl[20];
                uint16_t txtCol = discoveredNodes[selectedNodeIdx].active ? TFT_WHITE : TFT_LIGHTGREY;
                tft.setTextColor(txtCol, TFT_BLUE);
                sprintf(nodeLbl, "To: 0x%08X", discoveredNodes[selectedNodeIdx].id);
                tft.drawString(nodeLbl, 75, 15, 2);
            }

            xSemaphoreGive(spiSemaphore);
        }

        /* Footer Update at startup after wifi comes up */
        if (wifi_connected && !footer_drawn) {
            if (xSemaphoreTake(spiSemaphore, pdMS_TO_TICKS(20)) == pdTRUE) {
                drawFooter();
                footer_drawn = true;
                xSemaphoreGive(spiSemaphore);
            }
        }

    }
   
    /* Check for Touch Data */
    if (xQueueReceive(touchQueue, &receivedTouch, 0)) {
      uint32_t currentTime = millis();
      /* Only process if enough time has passed */
      if (currentTime - lastPressTime > debounceDelay) {
            /* Check Header Taps (Change Mode or Change Node) */
            if (receivedTouch.y < 45) {
                if (receivedTouch.x > 45 && receivedTouch.x < 70) {
                    currentMode = (currentMode == MODE_HOME) ? MODE_COLOR_PICKER : MODE_HOME;
                    if (xSemaphoreTake(spiSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
                        (currentMode == MODE_HOME) ? drawKeypad() : drawColorPicker();
                        xSemaphoreGive(spiSemaphore);
                    }
                } else if (receivedTouch.x > 75 && receivedTouch.x < 150) {
                    /* Cycle through discovered nodes */
                    selectedNodeIdx = (selectedNodeIdx + 1) % 5;
                    if(discoveredNodes[selectedNodeIdx].id == 0) selectedNodeIdx = 0;
                } else if (receivedTouch.y < 45 && receivedTouch.x > 260) {
                    currentMode = MODE_MENU;
                    if (xSemaphoreTake(spiSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
                        drawHamburgerMenu();
                        xSemaphoreGive(spiSemaphore);
                    }
                    continue;
                }
                continue;
            }
            /* Main Content Taps */
            if (currentMode == MODE_HOME) {
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
        else if (currentMode == MODE_COLOR_PICKER) {
                /* Color Grid Collision (8 cols, 4 rows) */
                int col = receivedTouch.x / 40;
                int row = (receivedTouch.y - 45) / 45;
                int colorIdx = (row * 8) + col;

                if (colorIdx >= 0 && colorIdx < 32) {
                    uint8_t canData[6];
                    canData[4] = (uint8_t)(0U); /* TODO: argb nodes might have more than one LED. write LED number into the buffer */
                    canData[5] = (uint8_t)colorIdx; /* copy the color index into the buffer */
                    
                    /* Send to the selected discovered node or broadcast */
                    uint32_t targetID = discoveredNodes[selectedNodeIdx].id ;

                    /* Copy the target ID into the canData buffer */
                    canData[0] = (targetID >> 24) & 0xFF;
                    canData[1] = (targetID >> 16) & 0xFF;
                    canData[2] = (targetID >> 8) & 0xFF;
                    canData[3] = targetID & 0xFF;

                    /* Send the message */                    
                    send_message(SET_ARGB_STRIP_COLOR_ID, canData, SET_ARGB_STRIP_COLOR_DLC);
                    
                    /* save node state and color index to the table of nodes */
                    discoveredNodes[selectedNodeIdx].lastColorIdx = colorIdx; 

                    /* print debug message */
                    Serial.printf("Sent Color %d to 0x%X\n", colorIdx, targetID);
                } /* closing colorIdx */
        } 
        else if (currentMode == MODE_NODE_SEL) {
            int clickedIdx = (receivedTouch.y - 45) / 38;
            if (clickedIdx >= 0 && clickedIdx < 5 && discoveredNodes[clickedIdx].id != 0) {
                selectedNodeIdx = clickedIdx;
                if (xSemaphoreTake(spiSemaphore, pdMS_TO_TICKS(50)) == pdTRUE) {
                    drawNodeSelector(); // Refresh highlight
                    xSemaphoreGive(spiSemaphore);
                }
            }
            continue;
        } else if (currentMode == MODE_MENU) {
            if (receivedTouch.x > 40 && receivedTouch.x < 280) {
                int item = (receivedTouch.y - 60) / 45;
                if (item >= 0 && item <= 3) {
                    currentMode = (DisplayMode)item; // Maps 0-3 to your Enum
                    if (xSemaphoreTake(spiSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
                        switch(currentMode) {
                            case MODE_HOME:         drawKeypad();       break;
                            case MODE_COLOR_PICKER: drawColorPicker();  break;
                            case MODE_NODE_SEL:     drawNodeSelector(); break;
                            case MODE_MENU:         /* Already here */  break;
                        }
                        xSemaphoreGive(spiSemaphore);
                    }
                }
            }
            continue;
        } /* closing currentMode */
      } /* closing debounce */
    } /* closing receivedTouch */

    /* 3. Explicitly yield to the IDLE task */
    vTaskDelay(pdMS_TO_TICKS(5));
  } /* closing for(;;) */
} /* closing TaskUpdateDisplay() */

