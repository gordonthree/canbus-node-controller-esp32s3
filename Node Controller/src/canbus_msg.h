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
#define ERROR_OVER_CURRENT_ERROR 0x100
#define ERROR_OVER_TEMP_ERROR 0x101
#define ERROR_OVER_VOLT_ERROR 0x102

#define SW_SET_OUT_MODE 0x10A
#define SW_OFF 0x10B
#define SW_ON 0x10C
#define SW_MOM_PRESS 0x10D
#define SW_MOM_RELEASE 0x10E
#define SW_SET_PWM_DUTY 0x10F
#define SW_SET_PWM_FREQ 0x110
#define SW_SET_BLINK_HS 0x111
#define SW_SET_STROBE_PAT 0x112

#define SET_DISPLAY_OFF 0x200
#define SET_DISPLAY_ON 0x201
#define SET_DISPLAY_CLEAR 0x202
#define SET_DISPLAY_FLASH 0x203
#define SET_ARGB_BUTTON_COLOR 0x204
#define SET_ARGB_BUTTON_BLINK 0x205
#define SET_ARGB_BUTTON_STROBE 0x206

#define SET_DISPLAY_BACKLIGHT_COLOR 0x208
#define SET_DISPLAY_BACKLIGHT_BRIGHTNESS 0x209
#define SET_1X1_REQ_DATA 0x20A
#define SET_OLED_REQ_FIELD_CONFIG 0x20B
#define SET_OLED_REQ_DATA 0x20C
#define SET_OLED_FIELD_COLOR 0x20D
#define SET_OLED_FIELD_BLINK 0x20E
#define SET_OLED_FIELD_STROBE 0x20F
#define SET_LCD_REQUEST_DATA 0x210

#define DATA_BUTTON_DOWN 0x300
#define DATA_BUTTON_UP 0x301
#define DATA_KEYSWITCH_LOCK 0x302
#define DATA_KEYSWITCH_UNLOCK 0x303
#define DATA_KNOB_CLOCKWISE 0x304
#define DATA_KNOB_COUNTER_CLOCKWISE 0x305
#define DATA_KNOB_CLICK 0x306
#define DATA_RFID_READ 0x307
#define DATA_CONTACT_CLOSED 0x308
#define DATA_CONTACT_OPENED 0x309
#define DATA_INTERNAL_TEMP 0x30A
#define DATA_INTERNAL_VOLTS 0x30B
#define DATA_INTERNAL_CURRENT 0x30C
#define DATA_EXTERNAL_TEMP 0x30D
#define DATA_EXTERNAL_VOLTS 0x30E
#define DATA_EXTERNAL_CURRENT 0x30F
#define DATA_AMBIENT_LIGHT 0x310
#define DATA_IMU_X-AXIS 0x311
#define DATA_IMU_Y-AXIS 0x312
#define DATA_IMU_Z-AXIS 0x313
#define DATA_IMU_X-GYRO 0x314
#define DATA_IMU_Y-GYRO 0x315
#define DATA_IMU_Z-GYRO 0x316

#define REQ_INTERFACES 0x400

#define REQ__BUTTONS 0x402
#define REQ_OUTPUTS 0x403
#define REQ_DISPLAYS 0x404
#define REQ_TEMP_SENSORS 0x405
#define REQ_VOLT_SENSORS 0x406
#define REQ_AMP_SENSORS 0x407
#define REQ_CLOSURE_INPUTS 0x408
#define REQ_AMBIENT_LIGHT_SENSORS 0x409
#define REQ_IMU_SENSORS 0x40A
#define REQ_BOXES 0x40B

#define IFACE_8X4_ARGB_KEYPAD 0x700
#define IFACE_4X4_ARGB_KEYPAD 0x701
#define IFACE_TOUCHSCREEN_TYPE_A 0x702
#define IFACE_TOUCHSCREEN_TYPE_B 0x703

#define IFACE_NEXTION_TYPE_A 0x706
#define IFACE_NEXTION_TYPE_B 0x707

#define IFACE_3X5_BUTTON_BOX 0x70C
#define IFACE_4X6_BUTTON_BOX 0x70D

#define ANALOG_LED_STRIP 0x710
#define ARGBW_LED_STRIP 0x711
#define ARGB_LED_STRIP 0x712
#define SINGLE_ITEM_DISPLAY 0x713
#define MULTI_ITEM_DISPLAY_ 0x714
#define OLED_DISPLAY_24IN 0x715
#define OLED_DISPLAY_43IN 0x716
#define LCD_20_CHAR 0x717
#define LCD_40_CHAR 0x718

#define HIGH_CURRENT_SWITCH 0x71A
#define LOW_CURRENT_SWITCH 0x71B
#define OPEN_DRAIN_OUTPUT 0x71C

#define BUTTON_NO_BACKLIGHT 0x720
#define BUTTON_ARGB_BACKLIGHT 0x721
#define BUTTON_VIRTUAL 0x722
#define BUTTON_WITH_DISPLAY 0x723
#define STAND_ALONE_KNOB 0x724
#define STAND_ALONE_JOG_DIAL 0x725
#define KEY_SWITCH 0x726
#define RFID_READER 0x727
#define BUTTON_TOUCHSCREEN 0x728
#define BUTTON_CAP_TOUCH 0x729

#define EXT_DIGITAL_TEMP 0x72E
#define EXT_ANALOG_K-TYPE_TEMP 0x72F

#define INTERNAL_PCB_TEMP 0x732
#define INTERNAL_CHIP_TEMP 0x733
#define EXTERNAL_VOLTAGE_SENSOR 0x734
#define INTERNAL_VOLTAGE_SENSOR 0x735

#define INTERNAL_PCB_CURRENT_SENSOR 0x738
#define EXTERNAL_CURRENT_SHUNT_TYPE 0x739
#define EXTERNAL_CURRENT_HALL_EFFECT 0x73A

#define CONTACT_CLOSURE_PULL_DOWN 0x73D
#define CONTACT_CLOSURE_PULL_UP 0x73E
#define CONTACT_CLOSURE_RESERVED 0x73F
#define CONTACT_CLOSURE_RESERVED 0x740
#define AMBIENT_LIGHT_SENSOR 0x741
#define IMU_X-AXIS_SENSOR 0x742
#define IMU_Y-AXIS_SENSOR 0x743
#define IMU_Z-AXIS_SENSOR 0x744
#define IMU_X-GYRO_SENSOR 0x745
#define IMU_Y-GYRO_SENSOR 0x746
#define IMU_Z-GYRO_SENSOR 0x747

#define SWITCH_BOX_3-GANG 0x750
#define SWITCH_BOX_4-GANG 0x751
#define SWITCH_BOX_6-GANG_HIGH 0x752
#define SWITCH_BOX_6-GANG_LOW 0x753
#define SWITCH_BOX_2-GANG_HIGH 0x754
#define SWITCH_BOX_2-GANG_LOW 0x755
#define DIGI_MULTI_TEMP_4X 0x756

#define TVA_INPUT_BOX 0x758

#define BOX_4X4_IO 0x75A

#endif // CAN_MSG_ID_H