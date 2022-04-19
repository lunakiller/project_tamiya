/**
  ******************************************************************************
  * @file           : project_tamiya.h
  * @brief          : Header file with defines and functions common for both
  *                   platforms (car and transmitter)
  *                     
  * @author         : Kristian Slehofer
  * @date           : 19. 4. 2022
  ******************************************************************************
  */

#ifndef __PROJECT_TAMIYA_H
#define __PROJECT_TAMIYA_H

#include "platform.h"
#include "nrf24.h"

// nRF24 macros
#define PT_nRF24_CHANNEL 115    // Channel frequency = (2400 + RF_CHANNEL)
#define PT_nRF24_DATARATE nRF24_DR_250kbps
#define PT_nRF24_CRC nRF24_CRC_1byte
#define PT_nRF24_ADDR_WIDTH 3
#define PT_nRF24_ADDR 0xAB, 0x01, 0x55
#define PT_nRF24_TXPWR nRF24_TXPWR_0dBm   // maximum power
#define PT_nRF24_ACK_SIZE 4
#define PT_nRF24_PACKET_SIZE 4

// maximum range constrained by mechanical limits
#define PT_MAXSTEER 300
#define PT_MAXTHRTL 450

// RC servo control neutral (equals to 1.5ms pulsewidth)
#define PT_RC_NEUTRAL 1500

// maximum values from control triggers
#define PT_TX_STEER_RANGE 935
#define PT_TX_THRTL_RANGE 950


uint8_t check_wifi(void);
uint8_t nRF24_InitModule(void);

#ifdef nRF24_RX
void nRF24_ReceivePacket(uint8_t* payload, uint8_t* payload_length, uint8_t* ack);

#elif defined nRF24_TX
void nRF24_TransmitPacket(uint8_t *pBuf, uint8_t length);
nRF24_TXResult nRF24_FinishTransmission(void);
void nRF24_GetCounters(uint8_t* plos, uint8_t* arc);
#else
#error "You should define nRF24_TX or nRF24_RX macro (support.h)!"
#endif

#ifdef UART
uint8_t strlength(const char *s);
void UART_SendChar(char b);
void UART_SendStr(char *string);
void UART_SendBufHex(char *buf, uint16_t bufsize);
void UART_SendHex8(uint16_t num);
void UART_SendInt(int32_t num);
#else
#error "You should assign UART port to UART macro (support.h)!"
#endif

#endif // __PROJECT_TAMIYA_H
