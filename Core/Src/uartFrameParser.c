/*
 * uartFrameParser.c
 *
 *  Created on: Mar 20, 2024
 *      Author: hllokrates
 */
#include "uartFrameParser.h"



volatile uart_states_t uart_state;
volatile uart_data_t uart_rx_data;
volatile uint8_t counter=0;
volatile uint8_t failCounter=0;

volatile uint8_t cs=0;
volatile bool fh1=0;
uint8_t h1=0x55;
uint8_t h2=0xAA;

void resetUartRxBuffer()
{
	fh1=0;
	memset(&uart_rx_data, '\0', sizeof(uart_data_t));
	uart_state=0;
	counter=0;
}

bool parserRx(const void* data)
{
	const uint8_t* d = data;


    switch(uart_state){
    case uart_h1:

    	if(d==h1)
    	{
    		uart_state++;
    	}
    	return 0;
    	break;

    case uart_h2:
    	if(d==h2){
    		uart_state++;
    	  }
    	else{
    	 uart_state = uart_h1;
    	 }
    	return 0;
    	break;

    case uart_len:
    	if(checkHeadersComeOnProccess(d)){return 0;break;}
    	uart_rx_data.len=d;
    	uart_state++;
    	return 0;
    	break;

    case uart_adr:
        if(checkHeadersComeOnProccess(d)){return 0;break;}
        uart_rx_data.addr=d;
        uart_state++;

        return 0;
        break;
    case uart_cmd:
        if(checkHeadersComeOnProccess(d)){return 0;break;}
        if(d==crc82 || d==crc83){uart_rx_data.enableCrc=1;}
        uart_rx_data.cmd=d;
        uart_state++;
        counter=2;
        return 0;
        break;

    case uart_data:
        if(checkHeadersComeOnProccess(d)){return 0;break;}
        uart_rx_data.buf[counter-2]=d;
        counter++;

        if(!uart_rx_data.enableCrc && counter==uart_rx_data.len)
        { // crc kontrol pasif ve data tamamlanmış
        	counter=0;
            uart_state=uart_done;
            return 1;
            break;

        }
        else if(uart_rx_data.enableCrc && counter==(uart_rx_data.len-2))
        { // crc aktif ve gelecek yeni veri var

        	counter=0;
        	uart_state++;
        }
        return 0;
        break;

    case uart_chksum: // 39 - 40 gelend ata chksum'a 4039 şeklinde atanıyor.
        if(checkHeadersComeOnProccess(d)){return 0;break;}
        uint8_t temp =d;
        uart_rx_data.chksum |= temp << (uint8_t)(8*counter++); // önce low byte sonra high byte gelmeli

        if(counter == uart_chksum_bytes){
        	counter=0;
        	uart_state=uart_done;
        	return 1;
        	break;
        }
        return 0;
    	break;

    }
}

bool checkHeadersComeOnProccess(const uint8_t* d) // sakin kafayla tekrardan bak
{

	if(!fh1 && d==h1)
	{
		fh1=1;
		return 0;
	}
	else if(fh1&& d==h2)
	{
		uart_state=uart_len;
		fh1=0;
		memset(&uart_rx_data, 0, sizeof(uart_data_t));
		failCounter++;
		return 1;
	}
	else{
		fh1=0;
		return 0;
	}

}
