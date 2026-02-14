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


/* Variables for the color picker routines - from main.cpp*/
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
const char* menuLabels[] = {"HOME", "COLOR PICKER", "NODE SELECT", "SYSTEM INFO", "HAMBURGER MENU"};

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
  
    /* Add a node to the ARGB list, for testing purposes */
    registerARGBNode(0x25A56D84);

    /* Start the tasks */
    xTaskCreate(TaskReadTouch, "TouchTask", 4096, NULL, 2, NULL);
    xTaskCreate(TaskUpdateDisplay, "DisplayTask", 6144, NULL, 1, NULL);
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
 * @brief Draws a stylized LED lightbar
 */
void drawLightbarIcon(int x, int y) {
    /* Main housing */
    tft.drawRoundRect(x - 12, y - 6, 24, 12, 2, TFT_WHITE);
    /* Individual LED "cells" */
    for (int i = 0; i < 3; i++) {
        tft.fillRect(x - 9 + (i * 7), y - 3, 5, 6, TFT_YELLOW);
    }
}

/**
 * @brief Draws a seat warmer (Seat profile with heat waves)
 */
void drawSeatWarmerIcon(int x, int y) {
    /* Seat Profile */
    tft.drawLine(x - 8, y - 5, x - 8, y + 8, TFT_WHITE); // Backrest
    tft.drawLine(x - 8, y + 8, x + 8, y + 8, TFT_WHITE); // Bottom
    
    /* Heat waves (squiggles) */
    tft.drawFastVLine(x - 2, y - 8, 4, TFT_RED);
    tft.drawFastVLine(x + 3, y - 8, 4, TFT_RED);
    tft.drawFastVLine(x + 8, y - 8, 4, TFT_RED);
}

/**
 * @brief Draws a water pump (Centrifugal housing style)
 */
void drawWaterPumpIcon(int x, int y) {
    /* Main circular body */
    tft.drawCircle(x, y, 8, TFT_WHITE);
    /* Outlet pipe */
    tft.fillRect(x + 4, y - 10, 6, 4, TFT_WHITE);
    /* Internal impeller cross */
    tft.drawFastHLine(x - 4, y, 8, TFT_CYAN);
    tft.drawFastVLine(x, y - 4, 8, TFT_CYAN);
}

/**
 * @brief Draws a windshield defroster (Curved pane with rising heat)
 */
void drawDefrosterIcon(int x, int y) {
    /* Curved windshield base */
    tft.drawEllipse(x, y + 8, 14, 4, TFT_WHITE);
    tft.fillRect(x - 14, y + 8, 28, 5, TFT_BLACK); // Mask bottom half of ellipse
    
    /* Rising heat lines */
    for (int i = 0; i < 3; i++) {
        int xOff = -8 + (i * 8);
        tft.drawLine(x + xOff, y + 4, x + xOff + 2, y - 4, TFT_ORANGE);
    }
}



/**
 * @brief Draws a color palette icon in the header (x=27)
 */
void drawPickerIcon() {
    /* Rainbow-ish 16x16 icon */
    tft.fillRect(27, 12, 8, 8, TFT_RED);
    tft.fillRect(35, 12, 8, 8, TFT_YELLOW);
    tft.fillRect(27, 20, 8, 8, TFT_BLUE);
    tft.fillRect(35, 20, 8, 8, TFT_GREEN);
    tft.drawRect(26, 11, 18, 18, TFT_WHITE);
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
 * @brief Draws the simplified header with mode toggle and hamburger menu.
 * @details This replaces the logic previously inside the 1000ms loop.
 */
void drawHeader(const char* title) {
    /* Clear header area with blue background */
    tft.fillRect(0, 0, 320, 43, TFT_BLUE);
    
    /* Left: Mode Toggle Icon (Home/Color) */
    drawPickerIcon(); /**< Color Picker Icon at x=27 */

    /* Center: Title and Selected Node context */
    tft.setTextColor(TFT_WHITE, TFT_BLUE);
    tft.drawCentreString(title, 160, 10, 2);
    
    if (discoveredNodes[selectedNodeIdx].id != 0) {
        char nodeLbl[20];
        uint16_t txtCol = discoveredNodes[selectedNodeIdx].active ? TFT_WHITE : TFT_LIGHTGREY;
        tft.setTextColor(txtCol, TFT_BLUE);
        sprintf(nodeLbl, "Node: 0x%08X", discoveredNodes[selectedNodeIdx].id);
        tft.drawString(nodeLbl, 80, 28, 1);
    }

    /* Right: Hamburger Menu Icon */
    drawHamburgerIcon(); /**< Hamburger icon at x=280 */
}

/**
 * @brief Draws a 2x2 grid of buttons based on the provided items
 * @param title The header title for the screen
 * @param items Array of 4 GridItem structs
 */
void drawUnifiedGrid(const char* title, GridItem* items) {
    tft.fillScreen(TFT_BLACK);
    drawHeader(title);
    drawFooter();

    for (int i = 0; i < 4; i++) {
        int bx = buttons[i].x;
        int by = buttons[i].y;
        int bw = buttons[i].w;
        int bh = buttons[i].h;

        /* Draw Button Body */
        tft.fillRoundRect(bx, by, bw, bh, 8, items[i].color);
        tft.drawRoundRect(bx, by, bw, bh, 8, TFT_WHITE);
        
        /* Draw Icon (Upper half) */
        if (items[i].drawIcon != NULL) {
            items[i].drawIcon(bx + (bw / 2), by + (bh / 2) - 10);
        }

        /* Draw Label (Lower half) */
        tft.setTextColor(TFT_WHITE);
        tft.drawCentreString(items[i].label, bx + (bw / 2), by + bh - 22, 2);
    }
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


void drawHomeIcon(int x, int y) {
    tft.fillTriangle(x, y-15, x-12, y, x+12, y, TFT_WHITE); // Roof
    tft.fillRect(x-8, y, 16, 12, TFT_WHITE);               // Body
    tft.fillRect(x-2, y+4, 4, 8, TFT_BLUE);                // Door
}

void drawPaletteIcon(int x, int y) {
    tft.fillCircle(x, y, 12, TFT_WHITE);
    tft.fillCircle(x-4, y-4, 3, TFT_RED);
    tft.fillCircle(x+4, y-4, 3, TFT_GREEN);
    tft.fillCircle(x, y+5, 3, TFT_BLUE);
}

void drawNetworkIcon(int x, int y) {
    tft.fillCircle(x, y-8, 4, TFT_WHITE);    // Top Node
    tft.fillCircle(x-8, y+8, 4, TFT_WHITE);  // Left Node
    tft.fillCircle(x+8, y+8, 4, TFT_WHITE);  // Right Node
    tft.drawLine(x, y-4, x-6, y+6, TFT_WHITE);
    tft.drawLine(x, y-4, x+6, y+6, TFT_WHITE);
}

void drawInfoIcon(int x, int y) {
    tft.fillCircle(x, y, 12, TFT_WHITE);
    tft.setTextColor(TFT_NAVY);
    tft.drawCentreString("i", x, y - 6, 2); // Simple 'i' for info
}

void drawColorPicker() {
    int swatchW = 40;
    int swatchH = 45;
    int startY = 45;
    drawHeader("COLOR PICKER");

    /* Get the currently active color for the selected node */
    int activeIdx = discoveredNodes[selectedNodeIdx].lastColorIdx;

    for (int i = 0; i < 32; i++) {
        int col = i % 8;
        int row = i / 8;
        int x = col * swatchW;
        int y = startY + (row * swatchH);

        uint16_t color565 = tft.color565(SystemPalette[i].r, SystemPalette[i].g, SystemPalette[i].b);
        tft.fillRect(x, y, swatchW, swatchH, color565);
        
        /* Draw selection highlight if this is the active color */
        if (i == activeIdx) {
            tft.drawRect(x, y, swatchW, swatchH, TFT_RED);
            tft.drawRect(x + 1, y + 1, swatchW - 2, swatchH - 2, TFT_RED); // Thick 2px border
        } else {
            tft.drawRect(x, y, swatchW, swatchH, TFT_WHITE);
        }
    }
}

void drawHamburgerMenu() {
    GridItem menuItems[4] = {
        {"HOME",    TFT_BLUE,       drawHomeIcon},
        {"COLORS",  TFT_DARKGREEN,  drawPaletteIcon},
        {"NODES",   TFT_MAROON,     drawNetworkIcon},
        {"SYSTEM",  TFT_NAVY,       drawInfoIcon}
    };
    drawUnifiedGrid("MAIN MENU", menuItems);
}

void drawKeypad() {
    GridItem keypadItems[4] = {
        {"LT BAR", TFT_BLUE,       drawLightbarIcon},
        {"SEAT WARMER",   TFT_MAROON,     drawSeatWarmerIcon},
        {"WTR PUMP",     TFT_DARKGREEN,  drawWaterPumpIcon},
        {"DEFROST",  TFT_ORANGE,     drawDefrosterIcon}
    };
    
    drawUnifiedGrid("VEHICLE CONTROL", keypadItems);
}

/**
 * @brief Draws the node selection screen with a consistent header
 */
void drawNodeSelector() {
    /* 1. Draw the standard blue header */
    tft.fillRect(0, 0, 320, 43, TFT_BLUE);
    tft.setTextColor(TFT_WHITE, TFT_BLUE);
    tft.drawCentreString("SELECT TARGET NODE", 160, 10, 2);
    
    /* Draw standard navigation icons in header */
    drawPickerIcon();    /**< Left toggle icon */
    drawHamburgerIcon(); /**< Right menu icon */

    /* 2. Draw the Content Area */
    tft.fillRect(0, 44, 320, 196, TFT_BLACK);

    for (int i = 0; i < 5; i++) {
        int yPos = 50 + (i * 38);
        uint16_t boxColor = (selectedNodeIdx == i) ? TFT_YELLOW : TFT_BLACK;
        
        /* Draw selection row container */
        tft.drawRect(5, yPos - 2, 310, 34, boxColor);

        if (discoveredNodes[i].id != 0) {
            char buf[25];
            sprintf(buf, "[%d] ID: 0x%08X", i, discoveredNodes[i].id);
            tft.setTextColor(discoveredNodes[i].active ? TFT_WHITE : TFT_LIGHTGREY);
            tft.drawString(buf, 15, yPos + 8, 2);

            /* Small square showing the last sent color */
            int cIdx = discoveredNodes[i].lastColorIdx;
            uint16_t previewColor = tft.color565(SystemPalette[cIdx].r, 
                                                 SystemPalette[cIdx].g, 
                                                 SystemPalette[cIdx].b);
            
            tft.fillRect(280, yPos + 5, 20, 20, previewColor);
            tft.drawRect(280, yPos + 5, 20, 20, TFT_WHITE);
        } else {
            tft.setTextColor(TFT_DARKGREY);
            tft.drawString("--- Empty Slot ---", 15, yPos + 8, 2);
        }
    }
    
    /* 3. Footer hint */
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.drawCentreString("Tap ID to select target", 160, 225, 1);
}


/**
 * @brief Draws diagnostic info including the relocated clock and CAN metrics.
 */
void drawSystemInfo() {
    tft.fillScreen(TFT_BLACK);
    drawHeader("SYSTEM INFO");

    /* --- Relocated Clock --- */
    struct tm timeinfo;
    if (getLocalTime(&timeinfo)) {
        char timeStr[20];
        strftime(timeStr, sizeof(timeStr), "%H:%M:%S", &timeinfo);
        tft.setTextColor(TFT_YELLOW, TFT_BLACK);
        tft.drawCentreString(timeStr, 160, 60, 4); 
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        tft.drawCentreString("System Time (UTC/Local)", 160, 95, 1);
    }

    /* --- Detailed CAN & Network Metrics --- */
    twai_status_info_t status;
    int yPos = 120;
    
    if (twai_get_status_info(&status) == ESP_OK) {
        tft.setTextColor(TFT_CYAN, TFT_BLACK);
        tft.drawString("CAN BUS STATUS:", 20, yPos, 2);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);
        
        tft.setCursor(30, yPos + 20);
        tft.printf("State: %s", (status.state == TWAI_STATE_RUNNING) ? "RUNNING" : "ERROR");
        tft.setCursor(30, yPos + 40);
        tft.printf("TX Errs: %d | RX Errs: %d", status.tx_error_counter, status.rx_error_counter);
    }

    /* Network Info */
    tft.setTextColor(TFT_GREEN, TFT_BLACK);
    tft.drawString("NETWORK:", 20, yPos + 70, 2);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setCursor(30, yPos + 90);
    tft.printf("IP: %s", wifiIP.c_str());
    tft.setCursor(30, yPos + 110);
    tft.printf("RSSI: %d dBm", WiFi.RSSI());
}

/**
 * @brief Redraws the current screen based on the active mode. Keep this function below other draw functions
 */
void refreshCurrentScreen() {
    switch(currentMode) {
        case MODE_HOME:           drawKeypad();        break;
        case MODE_COLOR_PICKER:   drawColorPicker();   break;
        case MODE_NODE_SEL:       drawNodeSelector();  break;
        case MODE_SYSTEM_INFO:    drawSystemInfo();    break;
        case MODE_HAMBURGER_MENU: drawHamburgerMenu(); break;
    }
}

/** Task 1: Read Touch */
void TaskReadTouch(void * pvParameters) {
  TouchData currentTouch;
  Serial.println("CYD: Touch Task Started");

  for(;;) {
    /* Only process touch if CAN is healthy and not suspended */
    if (can_driver_installed && !can_suspended) {
      digitalWrite(LED_GREEN, !digitalRead(LED_GREEN)); /* Toggle the green LED */

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
//   digitalWrite(LED_BLUE, LOW); /* Turn on the blue LED */

  for(;;) {
    uint32_t currentMillis = millis();

    /* STATE 1: Waiting for CAN Introduction Acknowledgement */
    if (!ui_initialized) {
        if (xSemaphoreTake(spiSemaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
            if (FLAG_SEND_INTRODUCTION) {
                drawHeader("Waiting for connection");
            } else {
                drawKeypad();
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
    /* 1000ms Refresh Loop */
    if (currentMillis - lastTimeUpdate >= 1000) {
        lastTimeUpdate = currentMillis;

        /* Check for stale nodes */
        for (int i = 0; i < 5; i++) {
            if (discoveredNodes[i].id != 0 && (currentMillis - discoveredNodes[i].lastSeen > 30000)) {
                discoveredNodes[i].active = false;
            }
        }

        /* ONLY perform a full redraw if we are on the System Info screen 
           (to update the clock/RSSI) or if the footer needs its first draw */
        if (xSemaphoreTake(spiSemaphore, pdMS_TO_TICKS(50)) == pdTRUE) {
            
            if (currentMode == MODE_SYSTEM_INFO) {
                drawSystemInfo();
            }

            /* Footer Update (only once at startup) */
            if (wifi_connected && !footer_drawn) {
                drawFooter();
                footer_drawn = true;
            }
            
            xSemaphoreGive(spiSemaphore);
        }
    } /* End 1000ms refresh loop */
    
    /* Check for Touch Data */
    if (xQueueReceive(touchQueue, &receivedTouch, 0)) {
        uint32_t currentTime = millis();
        
        if (currentTime - lastPressTime > debounceDelay) {
            lastPressTime = currentTime; // Move debounce lock to the start

            /**
             * @section Header Processing
             * Handle global navigation (Mode switching, Node cycling, Hamburger)
             */
            if (receivedTouch.y < 45) {
                bool handled = false;

                /* Left Target: Mode Toggle (x=0 to 80) */
                if (receivedTouch.x < 80) {
                    currentMode = (currentMode == MODE_HOME) ? MODE_COLOR_PICKER : MODE_HOME;
                    handled = true;
                } 
                /* Right Target: Hamburger Menu (x=240 to 320) */
                else if (receivedTouch.x > 240) {
                    currentMode = MODE_HAMBURGER_MENU;
                    handled = true;
                }
                /* Center Target: Cycle Nodes (x=80 to 240) */
                else if (receivedTouch.x >= 80 && receivedTouch.x <= 240) {
                    selectedNodeIdx = (selectedNodeIdx + 1) % 5;
                    if(discoveredNodes[selectedNodeIdx].id == 0) selectedNodeIdx = 0;
                    handled = true;
                }

                if (handled) {
                    if (xSemaphoreTake(spiSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
                        refreshCurrentScreen(); // Uses the helper function below
                        xSemaphoreGive(spiSemaphore);
                    }
                    continue;
                }
            }

            /**
             * @section Content Processing
             * Use switch-case for mode-specific screen interactions
             */
            switch (currentMode) {
                case MODE_HOME: {
                    for (int i = 0; i < 4; i++) {
                        if (receivedTouch.x >= buttons[i].x && receivedTouch.x <= (buttons[i].x + buttons[i].w) &&
                            receivedTouch.y >= buttons[i].y && receivedTouch.y <= (buttons[i].y + buttons[i].h)) {
                            
                            /* Visual Feedback */
                            if (xSemaphoreTake(spiSemaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
                                tft.drawRoundRect(buttons[i].x, buttons[i].y, buttons[i].w, buttons[i].h, 8, TFT_RED);
                                xSemaphoreGive(spiSemaphore);
                            }

                            uint8_t canData[5];
                            memcpy(canData, (void*)myNodeID, 4);
                            canData[4] = (uint8_t)buttons[i].canID;
                            send_message(SW_MOM_PRESS_ID, canData, SW_MOM_PRESS_DLC);

                            vTaskDelay(pdMS_TO_TICKS(150)); 

                            if (xSemaphoreTake(spiSemaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
                                tft.drawRoundRect(buttons[i].x, buttons[i].y, buttons[i].w, buttons[i].h, 8, TFT_WHITE);
                                xSemaphoreGive(spiSemaphore);
                            }
                        }
                    }
                    break;
                }

                case MODE_COLOR_PICKER: {
                    /* Ensure touch is within the palette grid area */
                    if (receivedTouch.y >= 45 && receivedTouch.y < 225) {
                        int col = receivedTouch.x / 40;
                        int row = (receivedTouch.y - 45) / 45;
                        
                        /* Clamp values to grid bounds */
                        if (col > 7) col = 7;
                        if (row > 3) row = 3;

                        int colorIdx = (row * 8) + col;

                        if (colorIdx >= 0 && colorIdx < 32) {
                            /* Update local state */
                            discoveredNodes[selectedNodeIdx].lastColorIdx = colorIdx;

                            /* Construct and send CAN message */
                            uint32_t targetID = discoveredNodes[selectedNodeIdx].id;
                            uint8_t canData[6];
                            canData[0] = (targetID >> 24) & 0xFF;
                            canData[1] = (targetID >> 16) & 0xFF;
                            canData[2] = (targetID >> 8) & 0xFF;
                            canData[3] = targetID & 0xFF;
                            canData[4] = 0; // LED Strip/Index
                            canData[5] = (uint8_t)colorIdx;

                            send_message(SET_ARGB_STRIP_COLOR_ID, canData, SET_ARGB_STRIP_COLOR_DLC);

                            /* Trigger immediate redraw for the selection highlight */
                            if (xSemaphoreTake(spiSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
                                drawColorPicker(); 
                                xSemaphoreGive(spiSemaphore);
                            }
                        }
                    }
                    break;
                }

                case MODE_NODE_SEL: {
                    int clickedIdx = (receivedTouch.y - 45) / 38;
                    if (clickedIdx >= 0 && clickedIdx < 5 && discoveredNodes[clickedIdx].id != 0) {
                        selectedNodeIdx = clickedIdx;
                        if (xSemaphoreTake(spiSemaphore, pdMS_TO_TICKS(50)) == pdTRUE) {
                            drawNodeSelector();
                            xSemaphoreGive(spiSemaphore);
                        }
                    }
                    break;
                }

            case MODE_HAMBURGER_MENU: {
                for (int i = 0; i < 4; i++) {
                    if (receivedTouch.x >= buttons[i].x && receivedTouch.x <= (buttons[i].x + buttons[i].w) &&
                        receivedTouch.y >= buttons[i].y && receivedTouch.y <= (buttons[i].y + buttons[i].h)) {
                        
                        currentMode = (DisplayMode)i; // 0=Home, 1=Picker, 2=Nodes, 3=System
                        
                        if (xSemaphoreTake(spiSemaphore, pdMS_TO_TICKS(100)) == pdTRUE) {
                            refreshCurrentScreen();
                            xSemaphoreGive(spiSemaphore);
                        }
                        break;
                    }
                }
                break;
            }

            case MODE_SYSTEM_INFO: {
                /* Optional: Add a button here to reset CAN counters or Reconnect WiFi */
                break;
            }
            } /* end switch(currentMode) */
        } /* end debounce */
    } /* end queue receive */

    /* 3. Explicitly yield to the IDLE task */
    vTaskDelay(pdMS_TO_TICKS(5));
  } /* closing for(;;) */
} /* closing TaskUpdateDisplay() */

