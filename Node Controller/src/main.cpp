#include <Arduino.h>

#include <ElegantOTA.h>

// Load Wi-Fi networking
#include <WiFi.h>
#include <AsyncTCP.h>
#include "ESPmDNS.h"

// Load FastLED
#include <FastLED.h>

// CAN bus library
#include <ESP32-TWAI-CAN.hpp>

// Webserver and file system
#include <ESPAsyncWebServer.h>
#include <LittleFS.h>

// my secrets
#include "secrets.h"

static AsyncWebServer server(80);

// no-op routine
#define NOP __asm__("nop");


// setup the ARGB led
#define NUM_LEDS 1
#define DATA_PIN 35

// CAN Transciever pins
#define CAN_TX		39
#define CAN_RX		38

// interrupt stuff
hw_timer_t *Timer0_Cfg = NULL;
 
const char* ssid = SECRET_SSID;
const char* password = SECRET_PSK;
const char *hostname = "cancontrol";

CanFrame rxFrame;

volatile int i=4;
volatile int isrFlag=false;

int period = 1000;
unsigned long time_now = 0;

CRGB leds[NUM_LEDS];

unsigned long ota_progress_millis = 0;

void IRAM_ATTR Timer0_ISR()
{
  isrFlag = true;
}

void onOTAStart() {
  // Log when OTA has started
  Serial.println("OTA update started!");
  // <Add your own code here>
}

void onOTAProgress(size_t current, size_t final) {
  // Log every 1 second
  if (millis() - ota_progress_millis > 1000) {
    ota_progress_millis = millis();
    Serial.printf("OTA Progress Current: %u bytes, Final: %u bytes\n", current, final);
  }
}

void onOTAEnd(bool success) {
  // Log when OTA has finished
  if (success) {
    Serial.println("OTA update finished successfully!");
  } else {
    Serial.println("There was an error during OTA update!");
  }
  // <Add your own code here>
}

void setup() {
  Timer0_Cfg = timerBegin(0, 80, true);
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
  timerAlarmWrite(Timer0_Cfg, 500000, true);
  timerAlarmEnable(Timer0_Cfg);

  FastLED.addLeds<SK6812, DATA_PIN, GRB>(leds, NUM_LEDS);

  Serial.begin(115200);
  Serial.println("Start blinky");

  Serial.print("WIFI init");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Make it possible to access webserver at http://myEsp32.local
  if (!MDNS.begin(hostname)) {
    Serial.println("Error setting up mDNS responder!");
  } else {
    Serial.printf("Access at http://%s.local\n", hostname);
  }

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
    request->send(200, "text/plain", "Hi! This is ElegantOTA AsyncDemo.");
  });

  ElegantOTA.begin(&server);    // Start ElegantOTA
  // ElegantOTA callbacks
  ElegantOTA.onStart(onOTAStart);
  ElegantOTA.onProgress(onOTAProgress);
  ElegantOTA.onEnd(onOTAEnd);

  server.begin();
  Serial.println("HTTP server started");
}

void checkLed() {

  static volatile int last_i = -1; // Keep track of the last state

// Only update LED and print if 'i' has changed

  if (i != last_i) {
    if (i == 1) 
    { 
      leds[0] = CRGB::Red;
      FastLED.show();
      Serial.println("RED LED is ON");
    } 
    else if (i == 2)
    {
      leds[0] = CRGB::Green;
      FastLED.show();
      Serial.println("GREEN LED is ON");
    } 
    else if (i == 3)
    {
      leds[0] = CRGB::Blue;
      FastLED.show();
      Serial.println("BLUE LED is ON");
    } 
    else if (i >= 4) 
    {
      leds[0] = CRGB::Black;
      FastLED.show();
      Serial.println("LED's are OFF");
      i = 0;
    }

    last_i = i; // Update the last known state
  }
}

void loop() {
  int cnt = 0;
  ElegantOTA.loop();

  // if(millis() >= time_now + period){
  //   time_now += period;
  //   i++;
  // }

  if (isrFlag) {
    // Serial.println("Interrupt");
    i++;
    checkLed();
    isrFlag = false;
  }
  // checkLed();
  // NOP;
}