// #ifdef CORE_DEBUG_LEVEL
// #undef CORE_DEBUG_LEVEL
// #endif

// #define CORE_DEBUG_LEVEL 3
// #define LOG_LOCAL_LEVEL ESP_LOG_DEBUG

#include <Arduino.h>
#include <ArduinoOTA.h>

#include <stdio.h>

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

// Load Wi-Fi networking
#include <WiFi.h>
#include <ESPmDNS.h>
#include "esp_wifi.h"

#ifndef ESP32CYD
// Load FastLED
#include <FastLED.h>
#endif


// Timekeeping library
#include <time.h>

// my secrets
#include "secrets.h"

// OTA task control
volatile bool ota_enabled = false;
volatile bool ota_started = false;
const char* ota_password = SECRET_PSK; // change this


// my canbus stuff
#include "canbus_msg.h"
#include "canbus_flags.h"

#define CAN_MY_IFACE_TYPE (0x701U) /* ARGB LED */
#define CAN_SELF_MSG 1


// esp32 native TWAI / CAN library
#include "driver/twai.h"

// Pins used to connect to CAN bus transceiver:
#ifndef RX_PIN
#define RX_PIN 22
#endif

#ifndef TX_PIN
#define TX_PIN 21
#endif

#ifdef ESP32CYD
#define LED_RED 4
#define LED_BLUE 17
#define LED_GREEN 16
#endif

// Interval:
#define TRANSMIT_RATE_MS 1000
#define POLLING_RATE_MS 1000

bool driver_installed = false;

unsigned long previousMillis = 0;  // will store last time a message was send
String texto;


static const char *TAG = "canesp32";

// Calls = 0;

// no-op routine
#define NOP __asm__("nop");


// setup the ARGB led
#define NUM_LEDS 1
#define DATA_PIN 27

#define AP_SSID  "canesp32"

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

#ifndef ESP32CYD
CRGB leds[NUM_LEDS];
#endif

unsigned long ota_progress_millis = 0;

volatile bool wifi_connected = false;
static volatile uint8_t myNodeID[] = {0, 0, 0, 0}; // node ID

void IRAM_ATTR Timer0_ISR()
{
  isrFlag = true;
}


void TaskOTA(void *pvParameters) {
  // Wait until WiFi is connected
  while (!wifi_connected) {
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }

  // Configure ArduinoOTA
  ArduinoOTA.setHostname(hostname);
  ArduinoOTA.setPassword(ota_password);

  ArduinoOTA.onStart([]() {
    Serial.println("OTA Start");
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nOTA End");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("OTA Progress: %u%%\n", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("OTA Error[%u]\n", error);
  });

  // Option A: Always enable OTA listener
  ArduinoOTA.begin();
  ota_started = true;
  Serial.println("ArduinoOTA ready");

  // Main loop: handle OTA requests
  for (;;) {
    ArduinoOTA.handle();
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }

  vTaskDelete(NULL);
}


void readMacAddress(){
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
    /* Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
                  baseMac[0], baseMac[1], baseMac[2],
                  baseMac[3], baseMac[4], baseMac[5]); */
    myNodeID[0] = baseMac[2];
    myNodeID[1] = baseMac[3];
    myNodeID[2] = baseMac[4];
    myNodeID[3] = baseMac[5];
    // Serial.printf("Node ID: %02x:%02x:%02x:%02x\n", myNodeID[0], myNodeID[1], myNodeID[2], myNodeID[3]);
  } else {
    Serial.println("Failed to set NODE ID");
  }
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

/**
 * @brief Send a message to the CAN bus
 * @param msgid The message ID of the frame to be sent
 * @param data The data to be sent in the frame
 * @param dlc The data length code of the frame, which is the number of bytes of data to be sent
 */
static void send_message( uint16_t msgid, uint8_t *data, uint8_t dlc) {
  twai_message_t message;
  uint8_t dataBytes[] = {0, 0, 0, 0, 0, 0, 0, 0}; // initialize dataBytes array with 8 bytes of 0
  // Format message
  message.identifier = msgid;       // set message ID
  message.extd = 0;                 // 0 = standard frame, 1 = extended frame
  message.rtr = 0;                  // 0 = data frame, 1 = remote frame
  message.self = 0;                 // 0 = normal transmission, 1 = self reception request 
  message.dlc_non_comp = 0;         // non-compliant DLC (0-8 bytes)  
  message.data_length_code = dlc;   // data length code (0-8 bytes)
  memcpy(message.data, data, dlc);  // copy data to message data field 
  
  // Queue message for transmission
  if (twai_transmit(&message, pdMS_TO_TICKS(3000)) == ESP_OK) {
    // ESP_LOGI(TAG, "Message queued for transmission\n");
    printf("Message queued for transmission\n");
  } else {
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
  }
  // vTaskDelay(100);
}

static void setDisplayMode(uint8_t *data, uint8_t displayMode) {
  // uint8_t dataBytes[] = {0xA0, 0xA0, 0x55, 0x55, 0x7F, 0xE4}; // data bytes
  uint16_t rxdisplayID = (data[4] << 8) | data[5]; // switch ID
  uint32_t rxunitID = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]; // unit ID
  
  switch (displayMode) {
    case 0: // display off
      Serial.printf("Unit %d Display %d OFF\n", rxunitID, rxdisplayID);
      break;
    case 1: // display on
      Serial.printf("Unit %d Display %d ON\n", rxunitID, rxdisplayID);
      break;
    case 2: // clear display
      Serial.printf("Unit %d Display %d CLEAR\n", rxunitID, rxdisplayID);
      break;
    case 3: // flash display
      Serial.printf("Unit %d Display %d FLASH\n", rxunitID, rxdisplayID);
      break;
    default:
      Serial.println("Invalid display mode");
      break;
  }
}

static void setSwMomDur(uint8_t *data) {
  static uint16_t switchID = (data[4] << 8) | data[5]; // switch ID 
  static uint32_t rxunitID = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]; // unit ID
  static uint16_t swDuration = (data[6] << 8) | data[7]; // duration in msD 
}


static void setSwBlinkDelay(uint8_t *data) {
  static uint16_t switchID = (data[4] << 8) | data[5]; // switch ID 
  static uint32_t rxunitID = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]; // unit ID
  static uint16_t swBlinkDelay = (data[6] << 8) | data[7]; // delay in ms 
}

static void setSwStrobePat(uint8_t *data) {
  static uint16_t switchID = (data[4] << 8) | data[5]; // switch ID 
  static uint32_t rxunitID = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]; // unit ID
  static uint8_t swStrobePat = data[6]; // strobe pattern
}


static void setPWMDuty(uint8_t *data) {
  static uint16_t switchID = (data[4] << 8) | data[5]; // switch ID 
  static uint32_t rxunitID = (data[0] << 24) | (data[1] << 16) | (data[2] << 8) | data[3]; // unit ID
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

static void txSwitchState(uint8_t *txUnitID, uint16_t txSwitchID, uint8_t swState) {
  uint8_t dataBytes[8];
  static const uint8_t txDLC = 5;
  
  dataBytes[0] = txUnitID[0]; // set unit ID
  dataBytes[1] = txUnitID[1]; // set unit ID
  dataBytes[2] = txUnitID[2]; // set unit ID
  dataBytes[3] = txUnitID[3]; // set unit ID
  dataBytes[4] = (txSwitchID); // set switch ID
  

  switch (swState) {

  case 0: // switch off
    send_message(SW_SET_OFF_ID, dataBytes, txDLC);
    break;
  case 1: // switch on
    send_message(SW_SET_ON_ID, dataBytes, txDLC);
    break;
  case 2: // momentary press
    send_message(SW_MOM_PRESS_ID, dataBytes, txDLC);
    break;
  default: // unsupported state
    Serial.println("Invalid switch state for transmission");
    break;
  }
}


static void setSwitchState(uint8_t *data, uint8_t swState) {
  // uint8_t dataBytes[] = {0xA0, 0xA0, 0x55, 0x55, 0x7F, 0xE4}; // data bytes
  static uint16_t switchID = (data[4] << 8) | data[5]; // switch ID
  static uint8_t unitID[] = {data[0], data[1], data[2], data[3]}; // unit ID
  
  switch (swState) {
    case 0: // switch off
      Serial.printf("Unit %02x:%02x:%02x:%02x Switch %d OFF\n", unitID[0],unitID[1],unitID[2],unitID[3], switchID);
      break;
    case 1: // switch on
      Serial.printf("Unit %02x:%02x:%02x:%02x Switch %d ON\n", unitID[0],unitID[1],unitID[2],unitID[3], switchID);
      break;
    case 2: // momentary press
      Serial.printf("Unit %02x:%02x:%02x:%02x Switch %d MOMENTARY PRESS\n", unitID[0],unitID[1],unitID[2],unitID[3], switchID);
      break;
    default:
      Serial.println("Invalid switch state");
      break;
  }
}

#ifndef NODE_ID
#define NODE_ID BOX_MULTI_IO_ID // default node ID
#define NODE_DLC BOX_MULTI_IO_DLC
#endif

/**
 * @brief Send an introduction message to the gateway node with the node's ID and feature mask.
 *
 * This function is used to send the node's ID and feature mask to the gateway node.
 * The feature mask is used to indicate which features the node supports.
 *
 * @param None
 * @return None
 */
static void sendIntroduction() {
  uint8_t dataBytes[NODE_DLC];
  dataBytes[0] = myNodeID[0]; // set node ID
  dataBytes[1] = myNodeID[1]; // set node ID
  dataBytes[2] = myNodeID[2]; // set node ID
  dataBytes[3] = myNodeID[3]; // set node ID
  dataBytes[4] = (0x0F);      // display id
  dataBytes[5] = (0xA0);      // feature mask 0
  dataBytes[6] = (0xB0);      // feature mask 1

  send_message(NODE_ID, (uint8_t *)dataBytes, NODE_DLC); /**< send introduction message to the gateway node with the node's ID and feature mask */

}

static void sendIntroack() {
  // uint8_t dataBytes[] = {0xA0, 0xA0, 0x55, 0x55}; // data bytes
  send_message(ACK_INTRO_ID, (uint8_t *)myNodeID, ACK_INTRO_DLC);
}

/**
 * @brief Get the current epoch time from the system clock.
 *
 * This function reads the current time from the ESP32 system clock
 * and returns it as a uint32_t representing the number of seconds
 * since the epoch (January 1, 1970, 00:00:00 UTC).
 *
 * @return uint32_t The current epoch time in seconds.
 */
static uint32_t getEpochTime() {
  /* Get time from the system clock and return it as a uint32_t */
  struct timespec newTime;

  clock_gettime(CLOCK_REALTIME, &newTime); /* Read time from ESP32 clock*/

  return (uint32_t)newTime.tv_sec; 
}

/**
 * @brief Send a uint32_t on the CAN bus
 * @param bigNumber The uint32_t to be sent
 * @param canMsgId The message ID to use for transmission (default: DATA_EPOCH_ID)
 * @param dlc The data length code to use for transmission (default: DATA_EPOCH_DLC)
 * 
 * This function takes a uint32_t and sends it on the CAN bus
 * with the given message ID and data length code.
 */
static void sendCanUint32(uint32_t bigNumber, uint32_t canMsgId = DATA_EPOCH_ID, uint8_t dlc = DATA_EPOCH_DLC) {
  /* Take a uint32_t and put it on the bus */
  uint8_t dataBytes[dlc];
  
  dataBytes[0] = myNodeID[0]; // set node ID
  dataBytes[1] = myNodeID[1]; // set node ID
  dataBytes[2] = myNodeID[2]; // set node ID
  dataBytes[3] = myNodeID[3]; // set node ID
  dataBytes[4] = (bigNumber >> 24) & 0xFF;
  dataBytes[5] = (bigNumber >> 16) & 0xFF;
  dataBytes[6] = (bigNumber >> 8) & 0xFF;
  dataBytes[7] = (bigNumber & 0xFF);

  send_message(canMsgId, (uint8_t *)dataBytes, dlc);

}
/**
 * @brief Receive time in seconds and write it to the ESP32 clock
 *
 * This function is used to receive time in seconds from the gateway node and write it to the ESP32 clock.
 * The time received is used to synchronize the ESP32 clock with the gateway node's clock.
 *
 * @param epochTime The time in seconds to be written to the ESP32 clock
 * @return None
 */
static void setEpochTime(uint32_t epochTime) {
/* Receive time in seconds and write it to the ESP32 clock */

  struct timespec newTime;
  newTime.tv_sec = (time_t)epochTime;
  newTime.tv_nsec = 0;
  clock_settime(CLOCK_REALTIME, &newTime);
  
}

static void handle_rx_message(twai_message_t &message) {
  // twai_message_t altmessage;
  bool msgFlag = false;


  if (message.data_length_code > 0) { // message contains data, check if it is for us
    uint8_t rxUnitID[4] = {message.data[0], message.data[1], message.data[2], message.data[3]};
    // memcmp((const uint8_t *)rxUnitID, (const uint8_t *)myNodeID, 4);

    if (memcmp(message.data, (const uint8_t *)myNodeID, 4) == 0) {
      msgFlag = true; // message is for us
      // Serial.printf("Node ID matched for message id 0x%x\n", message.identifier);
    } else {
      msgFlag = false; // message is not for us
      Serial.printf("Overheard message 0x%03x for node %02x:%02x:%02x:%02x\n", message.identifier, rxUnitID[0], rxUnitID[1], rxUnitID[2], rxUnitID[3]);
    }
  } else {
    msgFlag = true; // general broadcast message is valid
    Serial.printf("RX MSG: 0x%x NO DATA\n", message.identifier);
  }

  /*   
  if (msgFlag == false) {
    return; // message is not for us, exit function
  }
 */
  if (!msgFlag) {
    // Serial.println("Message does not match our ID, end of process.");
    return;
  }

  switch (message.identifier) {
    case SW_SET_OFF_ID:            // set output switch off
      setSwitchState(message.data, 0);
      txSwitchState((uint8_t *)myNodeID, 320, 2); 
      break;
    case SW_SET_ON_ID:             // set output switch on
      setSwitchState(message.data, 1);
      txSwitchState((uint8_t *)myNodeID, 320, 0); 
      break;
    case SW_SET_MODE_ID:           // setup output switch modes
      setSwitchMode(message.data);
      break;
    case SW_SET_BLINK_DELAY_ID:          // set output switch blink delay
      setSwBlinkDelay(message.data);
      break;
    case SW_SET_STROBE_PAT_ID:          // set output switch strobe pattern
      setSwStrobePat(message.data);
      break;
    case SET_DISPLAY_OFF_ID:          // set display off
      setDisplayMode(message.data, 0); 
      break;
    case SET_DISPLAY_ON_ID:          // set display on
      setDisplayMode(message.data, 1); 
      break;    
    case REQ_NODE_INTRO_ID:
      Serial.println("Interface intro request, responding with 0x702");
      FLAG_SEND_INTRODUCTION = true; // set flag to send introduction message
      sendIntroduction(); // send our introduction message
      break;
    case ACK_INTRO_ID:
      Serial.println("Received introduction acknowledgement, clearing flag");    
      FLAG_SEND_INTRODUCTION = false; // stop sending introduction messages
      txSwitchState((uint8_t *)myNodeID, 320, 1); 
      break;
    case DATA_EPOCH_ID:
      uint32_t epochTime;
      epochTime = ((message.data[4] << 24) | (message.data[5] << 16) | (message.data[6] << 8) | message.data[7]);
      setEpochTime(epochTime);
      Serial.println("Received epoch from master; updating clock");
      break;
    default:
      Serial.printf("Unknown message received 0x%x\n", message.identifier);
      // sendIntroack();
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


  /** Range 1: 0x200 to 0x23F (Binary: 010 0000 0000 to 010 0011 1111)
   * Range 2: 0x400 to 0x43F (Binary: 100 0000 0000 to 100 0011 1111)
   */

  /* * Acceptance Code: Defines the bits that must match.
  * Code 1 (High 16 bits): 0x4000 (ID 0x200 shifted for SJA1000)
  * Code 2 (Low 16 bits):  0x8000 (ID 0x400 shifted for SJA1000)
  */
  // uint32_t acc_code = 0x40008000;

  /* * Acceptance Mask: Defines "don't care" bits.
  * For the SJA1000, a '1' in the mask means "don't care".
  * We want to ignore the lower 6 bits of the ID.
  */
  // uint32_t acc_mask = 0x07F807F8; 

  // twai_filter_config_t f_config = {
  //     .acceptance_code = acc_code,
  //     .acceptance_mask = acc_mask,
  //     .single_filter = false /* false = Dual Filter Mode */
  // };

  /** Hardware filter configuration for two ID ranges.
  * Range 1: 0x200 - 0x23F
  * Range 2: 0x400 - 0x43F
  */
  twai_filter_config_t f_config;
  f_config.single_filter = false; /* Enable Dual Filter Mode */

  /** Filter 1:
  * Code: Base ID shifted left 5 bits.
  * Mask: The bits we want to ignore (0x3F) shifted left 5, 
  * OR'd with the lower 5 bits (0x1F) which are used for 
  * flags like RTR that we also want to ignore here.
  */
  uint16_t code1 = (0x200 << 5);
  uint16_t mask1 = (0x03F << 5) | 0x1F;

  /** Filter 2:
  * Same logic for the 0x400 range.
  */
  uint16_t code2 = (0x400 << 5);
  uint16_t mask2 = (0x03F << 5) | 0x1F;

  /** Combine into the 32-bit acceptance registers.
  * Filter 1 occupies the high 16 bits, Filter 2 the low 16 bits.
  */
  f_config.acceptance_code = (code1 << 16) | code2;
  f_config.acceptance_mask = (mask1 << 16) | mask2;

  // Initialize configuration structures using macro initializers
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL);  // TWAI_MODE_NO_ACK , TWAI_MODE_LISTEN_ONLY , TWAI_MODE_NORMAL
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();  //Look in the api-reference for other speed sets.
  // twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  // Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("TWAI installed");
  } else {
    Serial.println("Failed to install TWAI");
    return;
  }

  // Start TWAI driver
  if (twai_start() == ESP_OK) {
    Serial.println("TWAI started");
  } else {
    Serial.println("Failed to start TWAI");
    return;
  }

  // Reconfigure alerts to detect frame receive, Bus-Off error and RX queue full states
  uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_TX_IDLE | TWAI_ALERT_TX_SUCCESS | TWAI_ALERT_TX_FAILED | TWAI_ALERT_BUS_ERROR;
  if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
    Serial.println("TWAI alerts reconfigured");
  } else {
    Serial.println("Failed to reconfigure alerts"); 
    return;
  }

  // TWAI driver is now successfully installed and started
  driver_installed = true;
  FLAG_SEND_INTRODUCTION = true; /* send an introduction message */
  int loopCount = 0;

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
      // Serial.printf("TX buffered: %d\t", twaistatus.msgs_to_tx);
      // Serial.printf("TX error: %d\t", twaistatus.tx_error_counter);
      // Serial.printf("TX failed: %d\n", twaistatus.tx_failed_count);
    }

    if (alerts_triggered & TWAI_ALERT_TX_SUCCESS) {
      // Serial.println("Alert: The Transmission was successful.");
      // Serial.printf("TX buffered: %d\t", twaistatus.msgs_to_tx);
    }

    if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
      Serial.println("Alert: The RX queue full.");
      // Serial.printf("RX buffered: %d\t", twaistatus.msgs_to_rx);
      // Serial.printf("RX missed: %d\t", twaistatus.rx_missed_count);
      // Serial.printf("RX overrun %d\n", twaistatus.rx_overrun_count);
    }

    // Check if message is received
    if (alerts_triggered & TWAI_ALERT_RX_DATA) {
      // leds[0] = CRGB::Yellow;
      // FastLED.show();
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
      loopCount++;
      previousMillis = currentMillis;
      if (FLAG_SEND_INTRODUCTION) {
        FLAG_SEND_INTRODUCTION = false; /* Reset flag */
        sendIntroduction(); /* Send introduction message */
      }
      if (loopCount >= 10) {
        sendCanUint32(getEpochTime(), DATA_EPOCH_ID, DATA_EPOCH_DLC); /* Send epoch time as a heartbeat */
        loopCount = 0;
      }
      // send_message(REQ_NODE_INTRO_ID, NULL, REQ_NODE_INTRO_DLC); // send our introduction request
    }
    vTaskDelay(10);
  }
}

void setup() {

  xTaskCreate(
    TaskTWAI,     // Task function.
    "Task TWAI",  // name of task.
    4096,         // Stack size of task
    NULL,         // parameter of the task
    1,            // priority of the task
    NULL          // Task handle to keep track of created task
  );              // pin task to core 0
  //tskNO_AFFINITY); // pin task to core is automatic depends the load of each core

  // Start OTA task (small stack is fine; OTA uses some memory)
  xTaskCreate(
    TaskOTA,
    "Task OTA",
    4096,   // stack size (increase if you see stack errors)
    NULL,
    1,
    NULL
  );

#ifndef ESP32CYD
  FastLED.addLeds<SK6812, DATA_PIN, GRB>(leds, NUM_LEDS);
  leds[0] = CRGB::Black;
  FastLED.show();
#endif

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

  Serial.print("[DEFAULT] ESP32 Board MAC Address: ");
  readMacAddress();
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