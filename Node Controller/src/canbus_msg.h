#ifndef CAN_MSG_ID_H
#define CAN_MSG_ID_H

// Structure for CAN message ID
typedef struct {
    uint16_t id : 11; // 11-bit identifier
    uint8_t rtr : 1;  // Remote transmission request bit (0 for data, 1 for remote request)
    uint8_t ide : 1;  // Identifier extension bit (0 for standard, 1 for extended)
    uint8_t reserved : 1; // always 0
    uint8_t dlc : 4; // data length code (number of data bytes)
  } can_msg_id_t;
  
// Constants for specific message IDs 
#define ERROR_OVER_CURRENT 0x100 // over current
#define ERROR_OVER_TEMP 0x101 // over temp
#define ERROR_OVER_VOLT 0x102 // over volt

#define SW_SET_OUT_MODE 0x10A // switch set output mode (pwm, one-shot, solid-state, blinking, strobing)
#define SW_OFF 0x10B // switch off
#define SW_ON 0x10C // switch on
#define SW_MOM_PRESS 0x10D // switch momentary press
#define SW_MOM_RELEASE 0x10E // switch momentary release
#define SW_SET_PWM_DUTY 0x10F // switch set pwm duty
#define SW_SET_PWM_FREQ 0x110 // switch set pwm freq
#define SW_SET_BLINK_HS 0x111 // switch set blink/flash rate in hz
#define SW_SET_STROBE_PAT 0x112 // switch set strobe pattern

#define SET_DISPLAY_OFF 0x200 // set display off
#define SET_DISPLAY_ON 0x201 // set display on
#define SET_DISPLAY_CLEAR 0x202 // set display clear
#define SET_DISPLAY_FLASH 0x203 // set display flash
#define SET_ARGB_BUTTON_COLOR 0x204 // set argb button color
#define SET_ARGB_BUTTON_BLINK 0x205 // set argb button blink
#define SET_ARGB_BUTTON_STROBE 0x206 // set argb button strobe

#define SET_DISPLAY_BACKLIGHT_COLOR 0x208 // set display backlight color
#define SET_DISPLAY_BACKLIGHT_BRIGHTNESS 0x209 // set display backlight brightness
#define SET_1X1_REQ_DATA 0x20A // set 1x1 req data
#define SET_OLED_REQ_FIELD_CONFIG 0x20B // set oled req field config
#define SET_OLED_REQ_DATA 0x20C // set oled req data
#define SET_OLED_FIELD_COLOR 0x20D // set oled field color
#define SET_OLED_FIELD_BLINK 0x20E // set oled field blink
#define SET_OLED_FIELD_STROBE 0x20F // set oled field strobe
#define SET_LCD_REQUEST_DATA 0x210 // set lcd request data

#define DATA_BUTTON_DOWN 0x300 // button down
#define DATA_BUTTON_UP 0x301 // button up
#define DATA_KEYSWITCH_LOCK 0x302 // keyswitch lock
#define DATA_KEYSWITCH_UNLOCK 0x303 // keyswitch unlock
#define DATA_KNOB_CLOCKWISE 0x304 // knob clockwise
#define DATA_KNOB_COUNTER_CLOCKWISE 0x305 // knob counter clockwise
#define DATA_KNOB_CLICK 0x306 // knob click
#define DATA_RFID_READ 0x307 // rfid read
#define DATA_CONTACT_CLOSED 0x308 // contact closed
#define DATA_CONTACT_OPENED 0x309 // contact opened
#define DATA_INTERNAL_TEMP 0x30A // internal temp
#define DATA_INTERNAL_VOLTS 0x30B // internal volts
#define DATA_INTERNAL_CURRENT 0x30C // internal current
#define DATA_EXTERNAL_TEMP 0x30D // external temp
#define DATA_EXTERNAL_VOLTS 0x30E // external volts
#define DATA_EXTERNAL_CURRENT 0x30F // external current
#define DATA_AMBIENT_LIGHT 0x310 // ambient light
#define DATA_IMU_X_AXIS 0x311 // IMU X Axis
#define DATA_IMU_Y_AXIS 0x312 // IMU Y Axis
#define DATA_IMU_Z_AXIS 0x313 // IMU Z Axis
#define DATA_IMU_X_GYRO 0x314 // IMU X Gyro
#define DATA_IMU_Y_GYRO 0x315 // IMU Y Gyro
#define DATA_IMU_Z_GYRO 0x316 // IMU Z Gyro

#define REQ_INTERFACES 0x400 // req interfaces

#define REQ_BUTTONS 0x402 // req buttons
#define REQ_OUTPUTS 0x403 // req outputs
#define REQ_DISPLAYS 0x404 // req displays
#define REQ_TEMP_SENSORS 0x405 // req temp sensors
#define REQ_VOLT_SENSORS 0x406 // req volt sensors
#define REQ_AMP_SENSORS 0x407 // req amp sensors
#define REQ_CLOSURE_INPUTS 0x408 // req closure inputs
#define REQ_AMBIENT_LIGHT_SENSORS 0x409 // req ambient light sensors
#define REQ_IMU_SENSORS 0x40A // req imu sensors
#define REQ_BOXES 0x40B // req boxes

#define IFACE_8X4_ARGB_KEYPAD 0x700 // 8x4 argb keypad
#define IFACE_4X4_ARGB_KEYPAD 0x701 // 4x4 argb keypad
#define IFACE_TOUCHSCREEN_TYPE_A 0x702 // touchscreen type a
#define IFACE_TOUCHSCREEN_TYPE_B 0x703 // touchscreen type b

#define IFACE_NEXTION_TYPE_A 0x706 // nextion type a
#define IFACE_NEXTION_TYPE_B 0x707 // nextion type b

#define IFACE_3X5_BUTTON_BOX 0x70C // 3x5 button box
#define IFACE_4X6_BUTTON_BOX 0x70D // 4x6 button box

#define ANALOG_LED_STRIP 0x710 // analog led strip
#define ARGBW_LED_STRIP 0x711 // argbw led strip
#define ARGB_LED_STRIP 0x712 // argb led strip
#define SINGLE_ITEM_DISPLAY 0x713 // single item display
#define MULTI_ITEM_DISPLAY_ 0x714 // multi item display 
#define OLED_DISPLAY_2_4IN 0x715 // oled display 2 4in
#define OLED_DISPLAY_4_3IN 0x716 // oled display 4 3in
#define LCD_20_CHAR 0x717 // lcd 20 char
#define LCD_40_CHAR 0x718 // lcd 40 char

#define HIGH_CURRENT_SWITCH 0x71A // high current switch
#define LOW_CURRENT_SWITCH 0x71B // low current switch
#define OPEN_DRAIN_OUTPUT 0x71C // open drain output

#define BUTTON_NO_BACKLIGHT 0x720 // button no backlight
#define BUTTON_ARGB_BACKLIGHT 0x721 // button argb backlight
#define BUTTON_VIRTUAL 0x722 // button virtual
#define BUTTON_WITH_DISPLAY 0x723 // button with display
#define STAND_ALONE_KNOB 0x724 // stand alone knob
#define STAND_ALONE_JOG_DIAL 0x725 // stand alone jog dial
#define KEY_SWITCH 0x726 // key switch
#define RFID_READER 0x727 // rfid reader
#define BUTTON_TOUCHSCREEN 0x728 // button touchscreen
#define BUTTON_CAP_TOUCH 0x729 // button cap touch

#define EXT_DIGITAL_TEMP 0x72E // ext digital temp
#define EXT_ANALOG_K-TYPE_TEMP 0x72F // ext analog k-type temp

#define INTERNAL_PCB_TEMP 0x732 // internal pcb temp
#define INTERNAL_CHIP_TEMP 0x733 // internal chip temp
#define EXTERNAL_VOLTAGE_SENSOR 0x734 // external voltage sensor
#define INTERNAL_VOLTAGE_SENSOR 0x735 // internal voltage sensor

#define INTERNAL_PCB_CURRENT_SENSOR 0x738 // internal pcb current sensor
#define EXTERNAL_CURRENT_SHUNT_TYPE 0x739 // external current shunt type
#define EXTERNAL_CURRENT_HALL_EFFECT 0x73A // external current hall effect

#define CONTACT_CLOSURE_PULL_DOWN 0x73D // contact closure pull down
#define CONTACT_CLOSURE_PULL_UP 0x73E // contact closure pull up
#define CONTACT_CLOSURE_RESERVED 0x73F // contact closure reserved
#define CONTACT_CLOSURE_RESERVED 0x740 // contact closure reserved
#define AMBIENT_LIGHT_SENSOR 0x741 // ambient light sensor
#define IMU_X_AXIS_SENSOR 0x742 // IMU X Axis sensor
#define IMU_Y_AXIS_SENSOR 0x743 // IMU Y Axis sensor
#define IMU_Z_AXIS_SENSOR 0x744 // IMU Z Axis sensor
#define IMU_X_GYRO_SENSOR 0x745 // IMU X Gyro sensor
#define IMU_Y_GYRO_SENSOR 0x746 // IMU Y Gyro sensor
#define IMU_Z_GYRO_SENSOR 0x747 // IMU Z Gyro sensor

#define SWITCH_BOX_3GANG 0x750 // 3 gang switch box, 2 high, 1 low
#define SWITCH_BOX_4GANG 0x751 // 4-gang switch box, 2 high, 2 low
#define SWITCH_BOX_6GANG_HIGH 0x752 // 6-gang switch box, 4 high, 2 low
#define SWITCH_BOX_6GANG_LOW 0x753 // 6-gang switch box, 2 high, 4 low
#define SWITCH_BOX_2GANG_HIGH 0x754 // 2-gang switch box, 2 high
#define SWITCH_BOX_2GANG_LOW 0x755 // 2-gang switch box, 2 low
#define DIGI_MULTI_TEMP_4X 0x756 // 4-digital multi temp input box

#define TVA_INPUT_BOX 0x758 // temp, volt, current input box

#define BOX_4X4_IO 0x75A // input / output box, 4 in, 4 out

#endif // CAN_MSG_ID_H