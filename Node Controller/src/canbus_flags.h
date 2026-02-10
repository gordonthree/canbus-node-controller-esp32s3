#ifndef CAN_FLAGS_H
#define CAN_FLAGS_H
//

uint8_t FLAG_SEND_INTRODUCTION;    // stop introduction messages
uint8_t FLAG_BEGIN_NORMAL_OPER;    // being normal operation
uint8_t FLAG_HALT_NORMAL_OPER ;    // halt normal operation
uint8_t FLAG_SEND_HEALTHCHECK ;    // send diagnostic health information
uint8_t FLAG_SEND_NODECHECK   ;    // send node if and boot timestamp
uint8_t FLAG_PRINT_TIMESTAMP  ;    // print timestamp in human readable format

//
#endif // end can_flags.h
