#ifndef CAN_MSG_ID_H
#define CAN_MSG_ID_H

// bitmasks for message IDs
#define MASK_INTERFACE 0xFF0 // returns 0x700 if message ID is between 0x700 and 0x70F

// introduction message types
#define INTRO_INTERFACE 0x700 // introduction messages for interface nodes


// Constants for specific message IDs 
#define ERROR_OVER_CURRENT 0x100 // over current DLC 6
#define ERROR_OVER_TEMP 0x101 // over temp DLC 6
#define ERROR_OVER_VOLT 0x102 // over volt DLC 6

#define MSG_NORM_OPER 0x108 // message to all nodes to being normal operation, sent after introduction and enumeration are complete DLC 
#define MSG_HALT_OPER 0x109 // message to all nodes to stop transmitting messages and wait for instructions DLC 

#define SW_SET_MODE 0x10A // switch set output mode (pwm, one-shot, solid-state, blinking, strobing) DLC 7
#define SW_SET_OFF 0x10B // switch off DLC 6
#define SW_SET_ON 0x10C // switch on DLC 6
#define SW_MOM_PRESS 0x10D // switch momentary press DLC 6#define SW_SET_MOM_DUR 0x10E // set momentary switch duration ms DLC 8
#define SW_SET_MOM_DUR 0x10E // set momentary switch duration ms DLC 8
#define SW_SET_PWM_DUTY 0x10F // switch set pwm duty DLC 8
#define SW_SET_PWM_FREQ 0x110 // switch set pwm freq DLC 8
#define SW_SET_BLINK_DELAY 0x111 // switch set blink delay in tenths of a second 1-100 DLC 7
#define SW_SET_STROBE_PAT 0x112 // switch set strobe pattern DLC 7

#define SET_DISPLAY_OFF 0x200 // set display off DLC 6
#define SET_DISPLAY_ON 0x201 // set display on DLC 6
#define SET_DISPLAY_CLEAR 0x202 // set display clear DLC 6
#define SET_DISPLAY_FLASH 0x203 // set display flash DLC 7
#define SET_ARGB_BUTTON_COLOR 0x204 // set argb button color DLC 8
#define SET_ARGB_BUTTON_BLINK 0x205 // set argb button blink DLC 7
#define SET_ARGB_BUTTON_STROBE 0x206 // set argb button strobe DLC 7
#define SET_ARGB_BUTTON_LED_MODE 0x207 // set argb button led mode DLC 7
#define SET_DISPLAY_BACKLIGHT_COLOR 0x208 // set display backlight color DLC 8
#define SET_DISPLAY_BACKLIGHT_BRIGHTNESS 0x209 // set display backlight brightness DLC 8

#define SET_OLED_REQ_FIELD_CONFIG 0x20B // set oled req field config DLC 8
#define SET_DISP_REQ_DATA 0x20C // set disp req data DLC 8
#define SET_OLED_FIELD_COLOR 0x20D // set oled field color DLC 8
#define SET_OLED_FIELD_BLINK 0x20E // set oled field blink DLC 7
#define SET_OLED_FIELD_STROBE 0x20F // set oled field strobe DLC 7

#define SET_ARGB_STRIP_COLOR 0x211 // set argb strip color DLC 8
#define SET_ARGBW_STRIP_COLOR 0x212 // set argbw strip color DLC 8
#define SET_ANALOG_STRIP_COLOR 0x213 // set analog strip color DLC 8
#define SET_ADDR_STRIP_EFFECT 0x214 // set addr strip effect DLC 8
#define SET_LED_STRIP_BRIGHTNESS 0x215 // set led strip brightness DLC 8
#define SET_LED_STRIP_OFF 0x216 // set led strip off DLC 8
#define SET_LED_STRIP_ON 0x217 // set led strip on DLC 8

#define DISPLAY_DATA_MSG 0x21A // display data msg DLC 8
#define DISPLAY_CONF_MSG 0x21B // display conf msg DLC 8

#define ACK_INTRODUCTION 0x400 // acknowledge introduction, clear flag on remote device DLC 4
#define REQ_INTERFACES 0x401 // req interfaces DLC 4
#define REQ_BUTTONS 0x402 // req buttons DLC 4
#define REQ_OUTPUTS 0x403 // req outputs DLC 4
#define REQ_DISPLAYS 0x404 // req displays DLC 4
#define REQ_TEMP_SENSORS 0x405 // req temp sensors DLC 4
#define REQ_VOLT_SENSORS 0x406 // req volt sensors DLC 4
#define REQ_AMP_SENSORS 0x407 // req amp sensors DLC 4
#define REQ_CLOSURE_INPUTS 0x408 // req closure inputs DLC 4
#define REQ_AMBIENT_LIGHT_SENSORS 0x409 // req ambient light sensors DLC 4
#define REQ_IMU_SENSORS 0x40A // req imu sensors DLC 4
#define REQ_BOXES 0x40B // req boxes DLC 4
#define REQ_NODECHECK 0x40C // remote nodes should respond with their node id and last boot timestamp DLC 4
#define REQ_HEALTHCHECK 0x40D // remote nodes should respond with diagnostic sensor data DLC 4

#define DATA_BUTTON_DOWN 0x500 // button down DLC 8
#define DATA_BUTTON_UP 0x501 // button up DLC 8
#define DATA_KEYSWITCH_LOCK 0x502 // keyswitch lock DLC 8
#define DATA_KEYSWITCH_UNLOCK 0x503 // keyswitch unlock DLC 8
#define DATA_KNOB_CLOCKWISE 0x504 // knob clockwise DLC 8
#define DATA_KNOB_COUNTER_CLOCKWISE 0x505 // knob counter clockwise DLC 8
#define DATA_KNOB_CLICK 0x506 // knob click DLC 8
#define DATA_RFID_READ 0x507 // rfid read DLC 8
#define DATA_CONTACT_CLOSED 0x508 // contact closed DLC 8
#define DATA_CONTACT_OPENED 0x509 // contact opened DLC 8
#define DATA_RESERVED 0x50A // reserved DLC 8
#define DATA_INTERNAL_PCB_VOLTS 0x50B // internal pcb volts DLC 8
#define DATA_INTERNAL_PCB_CURRENT 0x50C // internal pcb current DLC 8
#define DATA_EXTERNAL_TEMP 0x50D // external temp DLC 8
#define DATA_EXTERNAL_VOLTS 0x50E // external volts DLC 8
#define DATA_EXTERNAL_CURRENT 0x50F // external current DLC 8
#define DATA_AMBIENT_LIGHT 0x510 // ambient light DLC 8
#define DATA_IMU_X_AXIS 0x511 // IMU X Axis DLC 8
#define DATA_IMU_Y_AXIS 0x512 // IMU Y Axis DLC 8
#define DATA_IMU_Z_AXIS 0x513 // IMU Z Axis DLC 8
#define DATA_IMU_X_GYRO 0x514 // IMU X Gyro DLC 8
#define DATA_IMU_Y_GYRO 0x515 // IMU Y Gyro DLC 8
#define DATA_IMU_Z_GYRO 0x516 // IMU Z Gyro DLC 8

#define DATA_NODE_CPU_TEMP 0x51A // node cpu temp DLC 6
#define DATA_NODE_LAST_BOOT_TIMESTAMP 0x51B // node last boot timestamp DLC 8
#define DATA_NODE_PCB_TEMP 0x51C // node pcb temp DLC 6
#define DATA_OUTPUT_SWITCH_OFF 0x51D // output switch off DLC 6
#define DATA_OUTPUT_SWITCH_ON 0x51E // output switch on DLC 6
#define DATA_OUTPUT_SWITCH_MODE 0x51F // output switch mode DLC 7
#define DATA_DISPLAY_OFF 0x520 // display off DLC 6
#define DATA_DISPLAY_ON 0x521 // display on DLC 6
#define DATA_DISPLAY_CLEAR 0x522 // display clear DLC 6
#define DATA_DISPLAY_FLASHING 0x523 // display flashing DLC 7
#define DATA_ARGB_BUTTON_COLOR 0x524 // argb button color DLC 8
#define DATA_ARGB_BUTTON_LED_MODE 0x525 // argb button led mode DLC 8

#define IFACE_8X4_ARGB_KEYPAD 0x700 // 8x4 argb keypad DLC 6
#define IFACE_4X4_ARGB_KEYPAD 0x701 // 4x4 argb keypad DLC 6
#define IFACE_TOUCHSCREEN_TYPE_A 0x702 // touchscreen type a DLC 6
#define IFACE_TOUCHSCREEN_TYPE_B 0x703 // touchscreen type b DLC 6

#define IFACE_NEXTION_TYPE_A 0x706 // nextion type a DLC 6
#define IFACE_NEXTION_TYPE_B 0x707 // nextion type b DLC 6

#define IFACE_3X5_BUTTON_BOX 0x70C // 3x5 button box DLC 6
#define IFACE_4X6_BUTTON_BOX 0x70D // 4x6 button box DLC 6

#define DISP_ANALOG_LED_STRIP 0x710 // analog led strip DLC 8
#define DISP_ARGBW_LED_STRIP 0x711 // argbw led strip DLC 8
#define DISP_ARGB_LED_STRIP 0x712 // argb led strip DLC 8

#define DISP_OLED 0x715 // non-touch oled display DLC 8

#define DISP_LCD 0x717 // lcd display DLC 8

#define OUT_HIGH_CURRENT_SW 0x71A // high current solid state switch DLC 8
#define OUT_LOW_CURRENT_SW 0x71B // low current solid state switch DLC 8
#define OUT_OPEN_DRAIN 0x71C // open drain output DLC 8
#define OUT_MECH_RELAY 0x71D // mechanical relay DLC 8


#define BUTTON_NO_BACKLIGHT 0x720 // button no backlight DLC 6
#define BUTTON_ARGB_BACKLIGHT 0x721 // button argb backlight DLC 6
#define BUTTON_VIRTUAL 0x722 // button virtual DLC 6
#define BUTTON_WITH_DISPLAY 0x723 // button with display DLC 6
#define BUTTON_ANALOG_KNOB 0x724 // analog knob input DLC 6
#define BUTTON_JOG_DIAL 0x725 // jog dial with or without click DLC 6
#define BUTTON_KEY_SWITCH 0x726 // key switch DLC 6
#define BUTTON_MOTION_DET 0x727 // motion detector DLC 6
#define BUTTON_CAP_TOUCH 0x728 // capacitive touch switch stand-alone DLC 6

#define INPUT_RFID_READER 0x72A // RFID reader input DLC 6

#define EXT_DIGITAL_TEMP 0x72E // ext digital temp DLC 6
#define EXT_ANALOG_K_TYPE_TEMP 0x72F // ext analog k type temp DLC 6

#define NODE_LASTBOOT 0x731 // last boot timestamp DLC 6
#define NODE_PCB_TEMP 0x732 // node pcb temp DLC 6
#define NODE_CPU_TEMP 0x733 // node cpu temp DLC 6
#define EXTERNAL_VOLTAGE_SENSOR 0x734 // external voltage sensor DLC 6
#define INTERNAL_VOLTAGE_SENSOR 0x735 // internal voltage sensor DLC 6

#define INTERNAL_PCB_CURRENT_SENSOR 0x738 // internal pcb current sensor DLC 6
#define EXTERNAL_CURRENT_SHUNT 0x739 // external current shunt DLC 6
#define EXTERNAL_CURRENT_HALL 0x73A // external current hall DLC 6

#define CONTACT_CLOSURE_PULL_DOWN 0x73D // contact closure pull down DLC 6
#define CONTACT_CLOSURE_PULL_UP 0x73E // contact closure pull up DLC 6
#define CONTACT_CLOSURE_RESERVED 0x73F // contact closure reserved DLC 6
#define CONTACT_CLOSURE_RESERVED 0x740 // contact closure reserved DLC 6
#define AMBIENT_LIGHT_SENSOR 0x741 // ambient light sensor DLC 6
#define IMU_X_AXIS_SENSOR 0x742 // IMU X Axis sensor DLC 6
#define IMU_Y_AXIS_SENSOR 0x743 // IMU Y Axis sensor DLC 6
#define IMU_Z_AXIS_SENSOR 0x744 // IMU Z Axis sensor DLC 6
#define IMU_X_GYRO_SENSOR 0x745 // IMU X Gyro sensor DLC 6
#define IMU_Y_GYRO_SENSOR 0x746 // IMU Y Gyro sensor DLC 6
#define IMU_Z_GYRO_SENSOR 0x747 // IMU Z Gyro sensor DLC 6


#define SWITCH_BOX_3GANG 0x750 // 3 gang switch box, 2 high, 1 low DLC 6
#define SWITCH_BOX_4GANG 0x751 // 4-gang switch box, 2 high, 2 low DLC 6
#define SWITCH_BOX_6GANG_HIGH 0x752 // 6-gang switch box, 4 high, 2 low DLC 6
#define SWITCH_BOX_6GANG_LOW 0x753 // 6-gang switch box, 2 high, 4 low DLC 6
#define SWITCH_BOX_2GANG_HIGH 0x754 // 2-gang switch box, 2 high DLC 6
#define SWITCH_BOX_2GANG_LOW 0x755 // 2-gang switch box, 2 low DLC 6
#define DIGI_MULTI_TEMP_4X 0x756 // 4-digital multi temp input box DLC 6
#define SWITCH_BOX_4RELAY 0x757 // 4-gang relay output box DLC 6
#define TVA_INPUT_BOX 0x758 // temp, volt, current input box DLC 6
#define SWITCH_BOX_8RELAY 0x759 // 8-gang relay output box DLC 6
#define BOX_4X4_IO 0x75A // input - output box, 4 in, 4 out DLC 6

#endif // END CAN_MSG_ID_H