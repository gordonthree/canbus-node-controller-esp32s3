#ifndef CAN_FLAGS_H
#define CAN_FLAGS_H
//

uint8_t FLAG_SEND_INTRODUCTION = false;    // stop introduction messages
uint8_t FLAG_BEGIN_NORMAL_OPER = false;    // being normal operation
uint8_t FLAG_HALT_NORMAL_OPER = false ;    // halt normal operation
uint8_t FLAG_SEND_HEALTHCHECK = false ;    // send diagnostic health information
uint8_t FLAG_SEND_NODECHECK = false   ;    // send node if and boot timestamp

//
#endif // end can_flags.h
