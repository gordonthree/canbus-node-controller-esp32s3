// #ifdef CORE_DEBUG_LEVEL
// #undef CORE_DEBUG_LEVEL
// #endif

// #define CORE_DEBUG_LEVEL 3
// #define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include <Arduino.h>
#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Load Wi-Fi networking
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPmDNS.h>

// #include <ESPAsync_WiFiManager.h>               //https://github.com/khoih-prog/ESPAsync_WiFiManager

// Load FastLED
#include <FastLED.h>

// CAN bus library
//#include <ESP32-TWAI-CAN.hpp>

// Webserver and file system
#define SPIFFS LittleFS
#include <LittleFS.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h> // https://github.com/bblanchon/ArduinoJson?utm_source=platformio&utm_medium=piohome

static AsyncWebServer server(80);

// my secrets
#include "secrets.h"

// my canbus message ids
#include "canbus_msg.h"

#include "esp_log.h"


#include "driver/twai.h"

// Pins used to connect to CAN bus transceiver:
#define RX_PIN 39
#define TX_PIN 38

// Intervall:
#define TRANSMIT_RATE_MS 1000
#define POLLING_RATE_MS 1000

static bool driver_installed = false;

unsigned long previousMillis = 0;  // will store last time a message was send
String texto;

static const char *TAG = "can_control";

// Calls = 0;

// no-op routine
#define NOP __asm__("nop");


// setup the ARGB led
#define NUM_LEDS 1
#define DATA_PIN 35

#define AP_SSID  "cancontrol"

// interrupt stuff
hw_timer_t *Timer0_Cfg = NULL;
 
const char* ssid = SECRET_SSID;
const char* password = SECRET_PSK;
const char* hostname =AP_SSID;

// CanFrame rxFrame;

volatile int i=4;
volatile bool isrFlag=false;
volatile bool ipaddFlag=true;

// volatile can_msg_t canMsgID;

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

}

//when wifi disconnects
void wifiOnDisconnect(){
  Serial.println("STA disconnected, reconnecting...");
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

static void send_message( twai_message_t message ) {
  // Send message

  // Queue message for transmission
  if (twai_transmit(&message, pdMS_TO_TICKS(3000)) == ESP_OK) {
    // ESP_LOGI(TAG, "Message queued for transmission\n");
    // printf("Message queued for transmission\n");
  } else {
    // ESP_LOGE(TAG, "Failed to queue message for transmission, initiating recovery");
    printf("Failed to queue message for transmission\n");
    twai_initiate_recovery();
    twai_stop();
    printf("twai Stoped\n");
    vTaskDelay(500);
    twai_start();
    printf("twai Started\n");
    // ESP_LOGI(TAG, "twai restarted\n");
    // wifiOnConnect();
    vTaskDelay(500);
  }
  // vTaskDelay(100);
}





///*

static void handle_rx_message(twai_message_t &message) {
  static twai_message_t altmessage;

  // Process received message
  // if (message.extd) {
  //   Serial.println("Message is in Extended Format");
  // } else {
  //   Serial.println("Message is in Standard Format");
  // }
  if (message.data_length_code > 0) {
    Serial.printf("RECV ID: 0x%x\nByte:", message.identifier);
    if (!(message.rtr)) {
      for (int i = 0; i < message.data_length_code; i++) {
        Serial.printf(" %d = %02x,", i, message.data[i]);
      }
      Serial.println("");
    }
  } else {
    Serial.printf("RECV ID: 0x%x\n", message.identifier);
  }


  // WORKING0
  if (message.identifier == REQ_INTERFACES) {
    Serial.println("Introduction request, responding with 0x702");
      // Send message
    altmessage.extd = 0;                // 0 = standard frame, 1 = extended frame
    altmessage.rtr = 0;                 // 0 = data frame, 1 = remote frame
    altmessage.identifier = IFACE_TOUCHSCREEN_TYPE_A;  // message ID
    altmessage.self = 1;                // 0 = normal transmission, 1 = self reception request 
    altmessage.data_length_code = 6;    // data length code (0-8 bytes)
    altmessage.dlc_non_comp = 0;
    uint8_t dataBytes[] = {0xA0, 0xA0, 0x55, 0x55, 0x7F, 0xE4}; // data bytes
    memcpy(altmessage.data, dataBytes, sizeof(dataBytes));
    
    send_message(altmessage);
  } else if ((message.identifier & MASK_INTERFACE) == INTRO_INTERFACE) { // received an interface introduction
    Serial.printf("Received introduction 0x%x\n", message.identifier);
      // Send message
    altmessage.extd = 0;                // 0 = standard frame, 1 = extended frame
    altmessage.rtr = 0;                 // 0 = data frame, 1 = remote frame
    altmessage.identifier = ACK_INTRODUCTION;  // acknowledge introduction 
    altmessage.self = 1;                // 0 = normal transmission, 1 = self reception request 
    altmessage.data_length_code = 4;    // data length code (0-8 bytes)
    altmessage.dlc_non_comp = 0;
    uint8_t dataBytes[] = {0xA0, 0xA0, 0x55, 0x55}; // data bytes
    memcpy(altmessage.data, dataBytes, sizeof(dataBytes));
    
    send_message(altmessage);
  } else if (message.identifier == ACK_INTRODUCTION) { // received a message
    Serial.println("Received introduction acknowledgement, starting over!\n");
  }


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
      leds[0] = CRGB::Yellow;
      FastLED.show();
      // Serial.println("GREEN LED is ON");
    } 
    else if (i == 3)
    {
      leds[0] = CRGB::Green;
      FastLED.show();
      // Serial.println("BLUE LED is ON");
    } 
    else if (i >= 4) 
    {
      leds[0] = CRGB::Blue;
      FastLED.show();
      // Serial.println("LED's are OFF");
      i = 0;
    }

    last_i = i; // Update the last known state
  }
}

void TaskTWAI(void *pvParameters) {
  // give some time at boot the cpu setup other parameters
  vTaskDelay(1000 / portTICK_PERIOD_MS);

  // Initialize configuration structures using macro initializers
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NO_ACK);  // TWAI_MODE_NO_ACK , TWAI_MODE_LISTEN_ONLY , TWAI_MODE_NORMAL
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_250KBITS();  //Look in the api-reference for other speed sets.
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("Driver installed");
  } else {
    Serial.println("Failed to install driver");
    return;
  }

  // Start TWAI driver
  if (twai_start() == ESP_OK) {
    Serial.println("Driver started");
  } else {
    Serial.println("Failed to start driver");
    return;
  }

  // Reconfigure alerts to detect frame receive, Bus-Off error and RX queue full states
  uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_TX_IDLE | TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED | TWAI_ALERT_BUS_ERROR;
  if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
    Serial.println("CAN Alerts reconfigured");
  } else {
    Serial.println("Failed to reconfigure alerts");
    return;
  }

  // TWAI driver is now successfully installed and started
  driver_installed = true;


  for (;;) {
    if (!driver_installed) {
      // Driver not installed
      vTaskDelay(1000);
      return;
    }
    // Check if alert happened
    uint32_t alerts_triggered;
    twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS));
    twai_status_info_t twaistatus;
    twai_get_status_info(&twaistatus);

    // Handle alerts
    if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
      Serial.println("Alert: TWAI controller has become error passive.");
    }

    if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
      Serial.println("Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
      Serial.printf("Bus error count: %d\n", twaistatus.bus_error_count);
    }

    if (alerts_triggered & TWAI_ALERT_TX_FAILED) {
      Serial.println("Alert: The Transmission failed.");
      Serial.printf("TX buffered: %d\t", twaistatus.msgs_to_tx);
      Serial.printf("TX error: %d\t", twaistatus.tx_error_counter);
      Serial.printf("TX failed: %d\n", twaistatus.tx_failed_count);
    }

    if (alerts_triggered & TWAI_ALERT_TX_SUCCESS) {
      // Serial.println("Alert: The Transmission was successful.");
      // Serial.printf("TX buffered: %d\t", twaistatus.msgs_to_tx);
    }

    if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
      Serial.println("Alert: The RX queue is full causing a received frame to be lost.");
      Serial.printf("RX buffered: %d\t", twaistatus.msgs_to_rx);
      Serial.printf("RX missed: %d\t", twaistatus.rx_missed_count);
      Serial.printf("RX overrun %d\n", twaistatus.rx_overrun_count);
    }

    // Check if message is received
    if (alerts_triggered & TWAI_ALERT_RX_DATA) {
      // Serial.println("Testing line");
      // One or more messages received. Handle all.
      twai_message_t message;
      while (twai_receive(&message, 0) == ESP_OK) {
        handle_rx_message(message);
      }
    }
    // Send message
    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= TRANSMIT_RATE_MS) {
      previousMillis = currentMillis;
      twai_message_t altmessage;
      altmessage.extd = 0;                // 0 = standard frame, 1 = extended frame
      altmessage.rtr = 0;                 // 0 = data frame, 1 = remote frame
      altmessage.identifier = REQ_INTERFACES;  // message ID
      altmessage.self = 1;                // 0 = normal transmission, 1 = self reception request 
      altmessage.data_length_code = 0;    // data length code (0-8 bytes)
      altmessage.dlc_non_comp = 0; 
  
      send_message(altmessage);
    }
    vTaskDelay(10);
  }
}

void TaskFLED(void *pvParameters) {
  checkLed();
  vTaskDelay(250);
}


void setup() {
  delay(5000);

  Timer0_Cfg = timerBegin(0, 80, true);
  timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
  timerAlarmWrite(Timer0_Cfg, 100000, true);
  timerAlarmEnable(Timer0_Cfg);


  xTaskCreate(
    TaskTWAI,     // Task function.
    "Task TWAI",  // name of task.
    3172,         // Stack size of task
    NULL,         // parameter of the task
    1,            // priority of the task
    NULL          // Task handle to keep track of created task
  );              // pin task to core 0
  //tskNO_AFFINITY); // pin task to core is automatic depends the load of each core


  // xTaskCreate(
  //   TaskFLED,     // Task function.
  //   "Task FLED",  // name of task.
  //   2048,         // Stack size of task
  //   NULL,         // parameter of the task
  //   1,            // priority of the task
  //   NULL    // Task handle to keep track of created task
  // );              // pin task to core 0
  //tskNO_AFFINITY); // pin task to core is automatic depends the load of each core

  FastLED.addLeds<SK6812, DATA_PIN, GRB>(leds, NUM_LEDS);

  Serial.begin(921600);
  Serial.setDebugOutput(true);

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
    request->send(200, "text/plain", "Hi! This is AsyncWebServer.");
  });


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



void loop() {
  int cnt = 0;

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