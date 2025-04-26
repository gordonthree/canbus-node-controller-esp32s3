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
#include <ESPAsyncWebServer.h>

static AsyncWebServer server(80);

// #include <ESPAsync_WiFiManager.h>               //https://github.com/khoih-prog/ESPAsync_WiFiManager

// Load FastLED
#include <FastLED.h>

// Webserver and file system
#define SPIFFS LittleFS
#include <LittleFS.h>
#include <ArduinoJson.h> // https://github.com/bblanchon/ArduinoJson?utm_source=platformio&utm_medium=piohome


// my secrets
#include "secrets.h"

// my canbus stuff
#include "canbus_msg.h"
#include "canbus_flags.h"

#define CAN_MY_IFACE_TYPE IFACE_TOUCHSCREEN_TYPE_A
#define CAN_SELF_MSG 1


// esp32 native TWAI / CAN library
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

static void send_message( uint16_t msgid, uint8_t *data, uint8_t dlc) {
  static twai_message_t message;
  static uint8_t dataBytes[] = {0, 0, 0, 0, 0, 0, 0, 0}; // initialize dataBytes array with 8 bytes of 0

  // Format message
  message.identifier = msgid;       // set message ID
  message.extd = 0;                 // 0 = standard frame, 1 = extended frame
  message.rtr = 0;                  // 0 = data frame, 1 = remote frame
  message.self = CAN_SELF_MSG;      // 0 = normal transmission, 1 = self reception request 
  message.dlc_non_comp = 0;         // non-compliant DLC (0-8 bytes)  
  message.data_length_code = dlc;   // data length code (0-8 bytes)
  memcpy(message.data, data, dlc);  // copy data to message data field 
  
  // Queue message for transmission
  if (twai_transmit(&message, pdMS_TO_TICKS(3000)) == ESP_OK) {
    // ESP_LOGI(TAG, "Message queued for transmission\n");
    // printf("Message queued for transmission\n");
  } else {
    leds[0] = CRGB::Red;
    FastLED.show();
    // ESP_LOGE(TAG, "Failed to queue message for transmission, initiating recovery");
    printf("Failed to queue message for transmission, resetting controller\n");
    twai_initiate_recovery();
    twai_stop();
    printf("twai Stoped\n");
    vTaskDelay(500);
    twai_start();
    printf("twai Started\n");
    // ESP_LOGI(TAG, "twai restarted\n");
    // wifiOnConnect();
    vTaskDelay(500);
    leds[0] = CRGB::Black;
    FastLED.show();
  }
  // vTaskDelay(100);
}

static void setDisplayMode(uint8_t *data, uint8_t displayMode) {
  // uint8_t dataBytes[] = {0xA0, 0xA0, 0x55, 0x55, 0x7F, 0xE4}; // data bytes
  static uint16_t displayID = (data[4] << 8) | data[5]; // switch ID
  static uint32_t unitID = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]; // unit ID
  
  switch (displayMode) {
    case 0: // display off
      Serial.printf("Unit %d Display %d OFF\n", unitID, displayID);
      break;
    case 1: // display on
      Serial.printf("Unit %d Display %d ON\n", unitID, displayID);
      break;
    case 2: // clear display
      Serial.printf("Unit %d Display %d CLEAR\n", unitID, displayID);
      break;
    case 3: // flash display
      Serial.printf("Unit %d Display %d FLASH\n", unitID, displayID);
      break;
    default:
      Serial.println("Invalid display mode");
      break;
  }
}

static void setSwMomDur(uint8_t *data) {
  static uint16_t switchID = (data[4] << 8) | data[5]; // switch ID 
  static uint32_t unitID = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]; // unit ID
  static uint16_t swDuration = (data[6] << 8) | data[7]; // duration in msD 
}


static void setSwBlinkDelay(uint8_t *data) {
  static uint16_t switchID = (data[4] << 8) | data[5]; // switch ID 
  static uint32_t unitID = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]; // unit ID
  static uint16_t swBlinkDelay = (data[6] << 8) | data[7]; // delay in ms 
}

static void setSwStrobePat(uint8_t *data) {
  static uint16_t switchID = (data[4] << 8) | data[5]; // switch ID 
  static uint32_t unitID = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]; // unit ID
  static uint8_t swStrobePat = data[6]; // strobe pattern
}


static void setPWMDuty(uint8_t *data) {
  static uint16_t switchID = (data[4] << 8) | data[5]; // switch ID 
  static uint32_t unitID = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]; // unit ID
  static uint16_t PWMDuty = (data[6] << 8) | data[7]; // switch ID 
}

static void setPWMFreq(uint8_t *data) {
  static uint16_t switchID = (data[4] << 8) | data[5]; // switch ID 
  static uint32_t unitID = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]; // unit ID
  static uint16_t PWMFreq = (data[6] << 8) | data[7]; // switch ID 
}
static void setSwitchMode(uint8_t *data) {
  static uint16_t switchID = (data[4] << 8) | data[5]; // switch ID 
  static uint32_t unitID = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]; // unit ID
  static uint8_t switchMode = data[6]; // switch mode

  switch (switchMode) {
    case 0: // solid state (on/off)
      break;
    case 1: // one-shot momentary
      break;
    case 2: // blinking
      break;
    case 3: // strobing
      break;
    case 4: // pwm
      break;
    default:
      Serial.println("Invalid switch mode");
      break;
  }

}

static void setSwitchState(uint8_t *data, uint8_t swState) {
  // uint8_t dataBytes[] = {0xA0, 0xA0, 0x55, 0x55, 0x7F, 0xE4}; // data bytes
  static uint16_t switchID = (data[4] << 8) | data[5]; // switch ID
  static uint32_t unitID = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]; // unit ID
  
  switch (swState) {
    case 0: // switch off
      Serial.printf("Unit %d Switch %d OFF\n", unitID, switchID);
      break;
    case 1: // switch on
      Serial.printf("Unit %d Switch %d ON\n", unitID, switchID);
      break;
    case 2: // momentary press
      Serial.printf("Unit %d Switch %d MOMENTARY PRESS\n", unitID, switchID);
      break;
    default:
      Serial.println("Invalid switch state");
      break;
  }
}



static void sendIntroduction() {
  uint8_t dataBytes[] = {0xA0, 0xA0, 0x55, 0x55, 0x7F, 0xE4}; // data bytes
  send_message(CAN_MY_IFACE_TYPE, dataBytes, sizeof(dataBytes));

}

static void sendIntroack() {
  uint8_t dataBytes[] = {0xA0, 0xA0, 0x55, 0x55}; // data bytes
  send_message(ACK_INTRODUCTION, dataBytes, sizeof(dataBytes));
}


static void handle_rx_message(twai_message_t &message) {
  // static twai_message_t altmessage;
  static bool msgFlag = false;
  
  leds[0] = CRGB::Green;
  FastLED.show();
  // Process received message
  // if (message.extd) {
  //   Serial.println("Message is in Extended Format");
  // } else {
  //   Serial.println("Message is in Standard Format");
  // }
  if (message.data_length_code > 0) {
    Serial.printf("RECV ID: 0x%x Bytes:", message.identifier);
    if (!(message.rtr)) {
      for (int i = 0; i < message.data_length_code; i++) {
        Serial.printf(" %d = 0x%02x", i, message.data[i]);
      }
      Serial.println("");
    }
  } else {
    Serial.printf("RECV ID: 0x%x\n", message.identifier);
  }


  switch (message.identifier) {
    case SW_SET_OFF:            // set output switch off
      setSwitchState(message.data, 0);
      break;
    case SW_SET_ON:             // set output switch on
      setSwitchState(message.data, 1);
      break;
    case SW_MOM_PRESS:          // set output momentary
      setSwitchState(message.data, 2);
      break;
    case SW_SET_MODE:           // setup output switch modes
      setSwitchMode(message.data);
      break;
    case SW_SET_PWM_DUTY:          // set output switch pwm duty
      setPWMDuty(message.data);  
      break;
    case SW_SET_PWM_FREQ:          // set output switch pwm frequency
      setPWMFreq(message.data);
      break;
    case SW_SET_MOM_DUR:          // set output switch momentary duration
      setSwMomDur(message.data);
      break;
    case SW_SET_BLINK_DELAY:          // set output switch blink delay
      setSwBlinkDelay(message.data);
      break;
    case SW_SET_STROBE_PAT:          // set output switch strobe pattern
      setSwStrobePat(message.data);
      break;
    case SET_DISPLAY_OFF:          // set display off
      setDisplayMode(message.data, 0); 
      break;
    case SET_DISPLAY_ON:          // set display on
      setDisplayMode(message.data, 1); 
      break;    
    case SET_DISPLAY_CLEAR:          // clear display
      setDisplayMode(message.data, 2); 
      break;
    case SET_DISPLAY_FLASH:          // flash display
      setDisplayMode(message.data, 3); 
      break;
    case REQ_INTERFACES:
      Serial.println("Interface intro request, responding with 0x702");
      FLAG_SEND_INTRODUCTION = true; // set flag to send introduction message
      sendIntroduction(); // send our introduction message
      break;

    case ACK_INTRODUCTION:
      Serial.println("Received introduction acknowledgement, starting over!\n");    
      FLAG_SEND_INTRODUCTION = false; // stop sending introduction messages
      break;
    
    default:
      if ((message.identifier & MASK_INTERFACE) == INTRO_INTERFACE) { // received an interface introduction
        Serial.printf("Received introduction 0x%x\n", message.identifier);
        sendIntroack();
      }
  
      break;
  }


} // end of handle_rx_message

// void checkLed() {

//   static volatile int last_i = -1; // Keep track of the last state

// // Only update LED and print if 'i' has changed

//   if (i != last_i) {
//     if (i == 1) 
//     { 
//       leds[0] = CRGB::Red;
//       FastLED.show();
//       // Serial.println("RED LED is ON");
//     } 
//     else if (i == 2)
//     {
//       leds[0] = CRGB::Yellow;
//       FastLED.show();
//       // Serial.println("GREEN LED is ON");
//     } 
//     else if (i == 3)
//     {
//       leds[0] = CRGB::Green;
//       FastLED.show();
//       // Serial.println("BLUE LED is ON");
//     } 
//     else if (i >= 4) 
//     {
//       leds[0] = CRGB::Blue;
//       FastLED.show();
//       // Serial.println("LED's are OFF");
//       i = 0;
//     }

//     last_i = i; // Update the last known state
//   }
// }

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
      leds[0] = CRGB::Red;
      FastLED.show();
    }

    if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
      Serial.println("Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
      Serial.printf("Bus error count: %d\n", twaistatus.bus_error_count);
      leds[0] = CRGB::Red;
      FastLED.show();
    }

    if (alerts_triggered & TWAI_ALERT_TX_FAILED) {
      Serial.println("Alert: The Transmission failed.");
      Serial.printf("TX buffered: %d\t", twaistatus.msgs_to_tx);
      Serial.printf("TX error: %d\t", twaistatus.tx_error_counter);
      Serial.printf("TX failed: %d\n", twaistatus.tx_failed_count);
      leds[0] = CRGB::Red;
      FastLED.show();
    }

    if (alerts_triggered & TWAI_ALERT_TX_SUCCESS) {
      leds[0] = CRGB::Green;
      FastLED.show();
      // Serial.println("Alert: The Transmission was successful.");
      // Serial.printf("TX buffered: %d\t", twaistatus.msgs_to_tx);
    }

    if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
      leds[0] = CRGB::Red;
      FastLED.show();
      Serial.println("Alert: The RX queue is full causing a received frame to be lost.");
      Serial.printf("RX buffered: %d\t", twaistatus.msgs_to_rx);
      Serial.printf("RX missed: %d\t", twaistatus.rx_missed_count);
      Serial.printf("RX overrun %d\n", twaistatus.rx_overrun_count);
    }

    // Check if message is received
    if (alerts_triggered & TWAI_ALERT_RX_DATA) {
      leds[0] = CRGB::Yellow;
      FastLED.show();
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
      leds[0] = CRGB::Blue;
      FastLED.show();
      previousMillis = currentMillis;
      send_message(REQ_INTERFACES, NULL, 0); // send our introduction request
    }
    vTaskDelay(10);
  }
}

void setup() {
  delay(5000);

  // Timer0_Cfg = timerBegin(0, 80, true);
  // timerAttachInterrupt(Timer0_Cfg, &Timer0_ISR, true);
  // timerAlarmWrite(Timer0_Cfg, 100000, true);
  // timerAlarmEnable(Timer0_Cfg);

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
  leds[0] = CRGB::Black;
  FastLED.show();

  Serial.begin(115200);
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

  // NOP;
}