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
     digitalWrite(LED_GREEN, !digitalRead(LED_GREEN)); /* Toggle the green LED */

    vTaskDelay(pdMS_TO_TICKS(20)); /* High polling rate for touch */
  }
}

/** Task 2: Update Display (Core 0) */
void TaskUpdateDisplay(void * pvParameters) {
  TouchData receivedTouch;
  char receivedTime[10];
  uint32_t lastPressTime = 0; 
  const uint32_t debounceDelay = 750; // Milliseconds to wait between valid presses
  static bool firstDrawDone = false;
  
  Serial.println("CYD: Display Task Started");
  digitalWrite(LED_BLUE, LOW); /* Turn on the blue LED */

  for(;;) {
    /* Initial Draw - Keep trying until the buttons get drawn. */
    if (!firstDrawDone) {
      if (xSemaphoreTake(spiSemaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
        drawKeypad();
        firstDrawDone = true;
        digitalWrite(LED_BLUE, HIGH); /* Turn off the blue LED */

        xSemaphoreGive(spiSemaphore);
      }
    }

    // Check for Touch Data
    if (xQueueReceive(touchQueue, &receivedTouch, 0)) {
      uint32_t currentTime = millis();
      // Only process if enough time has passed
      if (currentTime - lastPressTime > debounceDelay) {
        for (int i = 0; i < 4; i++) {
              // Collision Detection: Is the touch inside button[i]?
              if (receivedTouch.x >= buttons[i].x && receivedTouch.x <= (buttons[i].x + buttons[i].w) &&
                  receivedTouch.y >= buttons[i].y && receivedTouch.y <= (buttons[i].y + buttons[i].h)) {
                  
                  // Visual Feedback: Flash the button
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

                  // 1. Update the timestamp immediately to "lock" further presses
                  lastPressTime = currentTime;

                  vTaskDelay(pdMS_TO_TICKS(250)); // Simple debouncing
                  
                  // Reset border color
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

    // 3. Explicitly yield to the IDLE task
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

