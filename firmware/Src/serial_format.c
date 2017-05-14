#include <stdint.h>
#include "serial_format.h" 


uint8_t* ascii_encode_12b(uint16_t * data_in, uint16_t len, uint8_t * out_buff){
	uint16_t index;
	for(index = 0; index < len; index++){
		out_buff[index*2] = ASCII_ENCODE_U(data_in[index]);
		out_buff[index*2+1] = ASCII_ENCODE_L(data_in[index]);
	}
	return out_buff+(index*2);



}
