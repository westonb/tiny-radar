//functions for formatting data over serial
#ifndef __SERIAL_FORMAT_H
#define __SERIAL_FORMAT_H

#define ASCII_ENCODE_U(x)   ((x>>6) + 0x21)
#define ASCII_ENCODE_L(x)   ((x & 0x3f) + 0x21) 

uint8_t* ascii_encode_12b(uint16_t * data_in, uint16_t len, uint8_t * out_buff);




#endif