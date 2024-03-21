# RECEİVİNG DATA PACKET FRAME VİA DMA İN STM32F1XX SERİES
 
## 1. Introduction
This project is about receiving data packet frame via DMA in STM32F1xx series. The part related to DMA will not be mentioned. The DMA codes I use in my project are taken as reference from [stm32-usart-uart-dma-rx-tx](https://github.com/MaJerle/stm32-usart-uart-dma-rx-tx/tree/main) project and are intended to read the data package with minimal changes.
## 2. Data Frame

The data frame is as follows:


| Header1 | Header2 | Data Length | Address | Command | Data[0] | Data[..] | Data[n] | CRC[0] | CRC[1] |
| ------- | ------- | ----------- | ------- | ------- | ------- | -------- | ------- | ------ | ------ |

- Header1: 1 byte
- Header2: 1 byte
- Data Length: 1 byte (total length of address, command, data and crc)
- Address: 1 byte
- Command: 1 byte
- Data[n]: 0-255 byte
- CRC: 2 byte

## How it works

- The data received from UART is transferred to the DMA buffer using DMA interrupts.
- When interrupts occur on each DMA half-transfer (HT) and transfer complete (TC), the data processing function is executed.
- DMA provides the size of the incoming data, allowing each byte to be checked within a loop.
- The checking processes take place within uartFrameParser.h.
- Header1 and Header2 are checked, and then data of the length to be received is collected.
- If data does not arrive for the entire length during the transfer, the transfer is not considered successful.
- Additionally, if the incoming data is interrupted for any reason and a new data packet arrives, the previous packet being worked on is considered damaged, and the new data packet is read.


