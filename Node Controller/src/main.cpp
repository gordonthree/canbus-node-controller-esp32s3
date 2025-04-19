#include <Arduino.h>

#include <ElegantOTA.h>

// Load Wi-Fi networking
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPmDNS.h>

// #include <ESPAsync_WiFiManager.h>               //https://github.com/khoih-prog/ESPAsync_WiFiManager

// Load FastLED
#include <FastLED.h>

// CAN bus library
#include <ESP32-TWAI-CAN.hpp>

// Webserver and file system
#define SPIFFS LittleFS
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h> // https://github.com/bblanchon/ArduinoJson?utm_source=platformio&utm_medium=piohome

// my secrets
#include "secrets.h"

// my canbus message ids
#include "canbus_msg.h"

static AsyncWebServer server(80);

// no-op routine
#define NOP __asm__("nop");


// setup the ARGB led
#define NUM_LEDS 1
#define DATA_PIN 35

// CAN Transciever pins
#define CAN_TX		39
#define CAN_RX		38

#define AP_SSID  "cancontrol"

// interrupt stuff
hw_timer_t *Timer0_Cfg = NULL;
 
const char* ssid = SECRET_SSID;
const char* password = SECRET_PSK;
const char* hostname =AP_SSID;

CanFrame rxFrame;

volatile int i=4;
volatile bool isrFlag=false;
volatile bool ipaddFlag=true;

int period = 1000;
int8_t ipCnt = 0;

unsigned long time_now = 0;

CRGB leds[NUM_LEDS];

unsigned long ota_progress_millis = 0;

static volatile bool wifi_connected = false;

void IRAM_ATTR Timer0_ISR()
{
  isrFlag = true;
}

void wifiOnConnect(){
  Serial.println("STA Connected");
  Serial.print("STA SSID: ");
  Serial.println(WiFi.SSID());
  Serial.print("STA IPv4: ");
  Serial.println(WiFi.localIP());
  Serial.print("STA IPv6: ");
  Serial.println(WiFi.localIPv6());
}

//when wifi disconnects
void wifiOnDisconnect(){
  Serial.println("STA Disconnected");
  delay(1000);
  WiFi.begin(ssid, password);
}

void WiFiEvent(WiFiEvent_t event){
    switch(event) {

        case SYSTEM_EVENT_AP_START:
            //can set ap hostname here
            WiFi.softAPsetHostname(AP_SSID);
            //enable ap ipv6 here
            WiFi.softAPenableIpV6();
            break;

        case SYSTEM_EVENT_STA_START:
            //set sta hostname here
            WiFi.setHostname(AP_SSID);
            break;

        case SYSTEM_EVENT_STA_CONNECTED:
            //enable sta ipv6 here
            WiFi.enableIpV6();
            break;

        case SYSTEM_EVENT_AP_STA_GOT_IP6:
            //both interfaces get the same event
            Serial.print("STA IPv6: ");
            Serial.println(WiFi.localIPv6());
            Serial.print("AP IPv6: ");
            Serial.println(WiFi.softAPIPv6());
            break;

        case SYSTEM_EVENT_STA_GOT_IP:
            wifiOnConnect();
            wifi_connected = true;
            break;

        case SYSTEM_EVENT_STA_DISCONNECTED:
            wifi_connected = false;
            wifiOnDisconnect();
            break;

        default:
            break;
    }
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
  delay(5000);

  Timer0_Cfg = timerBegin(0, 80, true);
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
  timerAlarmWrite(Timer0_Cfg, 100000, true);
  timerAlarmEnable(Timer0_Cfg);

  FastLED.addLeds<SK6812, DATA_PIN, GRB>(leds, NUM_LEDS);

  Serial.begin(115200);

  WiFi.onEvent(WiFiEvent);
  WiFi.mode(WIFI_MODE_APSTA);
  WiFi.softAP(AP_SSID);
  WiFi.begin(ssid, password);

  Serial.println("AP Started");
  Serial.print("AP SSID: ");
  Serial.println(AP_SSID);
  Serial.print("AP IPv4: ");
  Serial.println(WiFi.softAPIP());


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

void printWifi() {
  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
}

void checkLed() {

  static volatile int last_i = -1; // Keep track of the last state

// Only update LED and print if 'i' has changed

  if (i != last_i) {
    if (i == 1) 
    { 
      leds[0] = CRGB::Red;
      FastLED.show();
      // Serial.println("RED LED is ON");
    } 
    else if (i == 2)
    {
      leds[0] = CRGB::Green;
      FastLED.show();
      // Serial.println("GREEN LED is ON");
    } 
    else if (i == 3)
    {
      leds[0] = CRGB::Blue;
      FastLED.show();
      // Serial.println("BLUE LED is ON");
    } 
    else if (i >= 4) 
    {
      leds[0] = CRGB::Black;
      FastLED.show();
      // Serial.println("LED's are OFF");
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
  //   // Serial.println("Tick");
  // }

  if (isrFlag) {
    // Serial.println("Interrupt");
    isrFlag = false;

    i++;
    ipCnt++;
    checkLed();

  }

  if (ipCnt>=100 && ipaddFlag) {
    ipaddFlag = false;
    printWifi();
  }
  // checkLed();
  // NOP;
}