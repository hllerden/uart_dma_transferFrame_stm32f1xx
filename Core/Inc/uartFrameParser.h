/*
 * uartFrameParser.h
 *
 *  Created on: Mar 20, 2024
 *      Author: KEMERDEN
 */
#include <stdint.h>
#include <stdbool.h>
#ifndef SRC_UARTFRAMEPARSER_H_
#define SRC_UARTFRAMEPARSER_H_

typedef union{
 struct{
    uint8_t  len;
    uint8_t  addr;
    uint8_t  cmd;
    uint8_t  buf[64];
    bool enableCrc;
    uint16_t chksum;
 	 	 } ;
}uart_data_t;


typedef enum{
    uart_h1   = 0,
    uart_h2   = 1,
    uart_len        = 2,
    uart_adr        = 3,
    uart_cmd        = 4,
    uart_data       = 5,
    uart_chksum     = 6,
    uart_done       = 7,
    uart_chksum_bytes  = 2,
}uart_states_t;

enum CrcCommands
{
	crc82   = 0x82,
	crc83  = 0x83,
};


// |0x55|0xAA|LENGTH|ADDRESS|CMD|DATA0....DATAn|CHKSUM|.

extern volatile uart_states_t uart_state;
extern volatile uart_data_t uart_rx_data;


void resetUartRxBuffer();

bool parserRx(const void* data);

bool checkHeadersComeOnProccess(const uint8_t* d);

#endif /* SRC_UARTFRAMEPARSER_H_ */
