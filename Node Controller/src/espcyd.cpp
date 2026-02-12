#include "espcyd.h"


TFT_eSPI tft = TFT_eSPI();

SPIClass touchscreenSPI = SPIClass(VSPI);
XPT2046_Touchscreen touchscreen(XPT2046_CS, XPT2046_IRQ);

// Touchscreen coordinates: (x, y) and pressure (z)
int x, y, z;

// Global variables for inter-task communication
volatile int globalX, globalY, globalZ;
volatile bool newData = false;

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

const char* ssid     = "Tell my WiFi I love her";
const char* password = "2317239216";
const char* ntpServer = "pool.ntp.org";
String wifiIP = "Connecting...";

void drawKeypad() {
    if (xSemaphoreTake(spiSemaphore, pdMS_TO_TICKS(50)) == pdTRUE) {
        tft.fillScreen(TFT_BLACK);
        for (int i = 0; i < 4; i++) {
            tft.fillRoundRect(buttons[i].x, buttons[i].y, buttons[i].w, buttons[i].h, 8, buttons[i].color);
            tft.drawRoundRect(buttons[i].x, buttons[i].y, buttons[i].w, buttons[i].h, 8, TFT_WHITE);
            tft.setTextColor(TFT_WHITE);
            tft.drawCentreString(buttons[i].label, buttons[i].x + (buttons[i].w / 2), buttons[i].y + (buttons[i].h / 2) - 8, 2);
        }
        xSemaphoreGive(spiSemaphore);
    }
}

void TaskWiFiTime(void * pvParameters) {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(pdMS_TO_TICKS(500));
  }

  // Capture the IP address
  wifiIP = WiFi.localIP().toString();
  Serial.println("WiFi Connected. IP: " + wifiIP);
  
  // --- Configure OTA ---
  ArduinoOTA.setHostname("esp32-cyd-dashboard");
  // ArduinoOTA.setPassword("admin"); // Optional: adds a password prompt in VSCode

  ArduinoOTA.onStart([]() {
      Serial.println("OTA Update Starting...");
  });
  
  ArduinoOTA.begin();

  // Init and get the time
  configTime(0, 0, ntpServer); // UTC time

  for(;;) {
    struct tm timeinfo;

    // Essential: Keep listening for OTA packets!
    ArduinoOTA.handle();

    if(getLocalTime(&timeinfo)){
      char timeString[10];
      strftime(timeString, sizeof(timeString), "%H:%M:%S", &timeinfo);
      
      // Send time string to the time queue
      xQueueSend(timeQueue, &timeString, 0);
    }
    vTaskDelay(pdMS_TO_TICKS(1000)); // Update every second
  }
}

// --- Task 1: Read Touch (Core 1) ---
void TaskReadTouch(void * pvParameters) {
  TouchData currentTouch;
  for(;;) {
    // Try to take the mutex (wait up to 10ms if busy)
    if (touchscreen.tirqTouched() && touchscreen.touched()) {
      if (spiSemaphore != NULL && xSemaphoreTake(spiSemaphore, pdMS_TO_TICKS(10)) == pdTRUE) {
        TS_Point p = touchscreen.getPoint();
        currentTouch.x = map(p.x, 200, 3700, 1, SCREEN_WIDTH);
        currentTouch.y = map(p.y, 240, 3800, 1, SCREEN_HEIGHT);
        currentTouch.z = p.z;
        // newData = true; 
        
        if (p.z > 800) { // Only queue if the press is firm enough
          xQueueSend(touchQueue, &currentTouch, 0);
        }
      }
      // Always give the mutex back!
      xSemaphoreGive(spiSemaphore);
    }
    vTaskDelay(pdMS_TO_TICKS(20)); // High polling rate for touch
  }
}

// --- Task 2: Update Display (Core 0) ---
void TaskUpdateDisplay(void * pvParameters) {
  TouchData receivedTouch;
  char receivedTime[10];
  
  uint32_t lastPressTime = 0; 
  const uint32_t debounceDelay = 750; // Milliseconds to wait between valid presses
  // Initial Draw
  drawKeypad();

  for(;;) {
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

                  // VIRTUAL CAN TRANSMIT
                  Serial.println("--------------------------------");
                  Serial.print("CAN FRAME SENT: ID 0x");
                  Serial.println(buttons[i].canID, HEX);
                  Serial.print("DATA: [ 0x01 0x00 0x00 0x00 ] - CMD: ");
                  Serial.println(buttons[i].label);
                  Serial.print("TOUCH DATA [X, Y, Z]: ");
                  Serial.print(receivedTouch.x);
                  Serial.print(", ");
                  Serial.print(receivedTouch.y);
                  Serial.print(", ");
                  Serial.println(receivedTouch.z);
                  Serial.print("DEBOUNCE TIME: ");
                  Serial.println((currentTime - lastPressTime));
                  Serial.println("--------------------------------");

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

void setup() {
  Serial.begin(115200);

  // 1. CREATE THE MUTEX FIRST
  // Use xSemaphoreCreateBinary instead
  spiSemaphore = xSemaphoreCreateBinary();
  xSemaphoreGive(spiSemaphore); // You must "Give" it once initially so it's available
  
  touchQueue = xQueueCreate(5, sizeof(TouchData));
  timeQueue = xQueueCreate(1, 10 * sizeof(char)); // Queue for 10-char string

 
  // Start the SPI for the touchscreen and init the touchscreen
  touchscreenSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
  touchscreen.begin(touchscreenSPI);
  // Set the Touchscreen rotation in landscape mode
  // Note: in some displays, the touchscreen might be upside down, so you might need to set the rotation to 3: touchscreen.setRotation(3);
  touchscreen.setRotation(1);

  // Start the tft display
  tft.init();
  // Set the TFT display rotation in landscape mode
  tft.setRotation(1);

  // Clear the screen before writing to it
  tft.fillScreen(TFT_WHITE);
  tft.setTextColor(TFT_BLACK, TFT_WHITE);
  
  // Set X and Y coordinates for center of display
  int centerX = SCREEN_WIDTH / 2;
  int centerY = SCREEN_HEIGHT / 2;

  tft.drawCentreString("Hello, world!", centerX, 30, FONT_SIZE);
  tft.drawCentreString("Touch screen to test", centerX, centerY, FONT_SIZE);

  // Create Tasks
  xTaskCreatePinnedToCore(
    TaskReadTouch,    // Function
    "TouchTask",      // Name
    4096,             // Stack size
    NULL,             // Parameters
    2,                // Priority
    &TaskTouchHandle, // Handle
    1                 // Core 1
  );

  xTaskCreatePinnedToCore(
    TaskUpdateDisplay,
    "DisplayTask",
    4096,
    NULL,
    1,
    &TaskDisplayHandle,
    0                 // Core 0
  );

  // Create the WiFi Task
  xTaskCreatePinnedToCore(
    TaskWiFiTime, 
    "WiFiTask", 
    8192, 
    NULL, 
    1, 
    NULL, // no handle needed 
    0);
}

void loop() {
  vTaskDelete(NULL);
}