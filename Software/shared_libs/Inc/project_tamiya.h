/**
  ******************************************************************************
  * @file           : project_tamiya.h
  * @brief          : Header file with defines and functions common for both
  *                   platforms (car and transmitter)
  *                     
  * @author         : Kristian Slehofer
  * @date           : 2. 5. 2022
  ******************************************************************************
  */

#ifndef __PROJECT_TAMIYA_H
#define __PROJECT_TAMIYA_H

#include "platform.h"
#include "nrf24.h"
#include "ssd1306.h"
#include "batt_logo.h"        // OLED battery logo
#include "battoff_logo.h"     // OLED drained battery logo
#include "temp_logo.h"        // OLED temperature logo
#include "signal_logo.h"      // OLED signal logo
#include "msg_sec.h"

// nRF24 configuration
#define PT_nRF24_CHANNEL      115    // Channel frequency = (2400 + RF_CHANNEL)
#define PT_nRF24_DATARATE     nRF24_DR_250kbps
#define PT_nRF24_CRC          nRF24_CRC_1byte
#define PT_nRF24_ADDR_WIDTH   3
#define PT_nRF24_ADDR         0xAB, 0x01, 0x55
#define PT_nRF24_TXPWR        nRF24_TXPWR_0dBm   // maximum power
#define PT_nRF24_ACK_SIZE     3
#define PT_nRF24_PACKET_SIZE  4

// maximum range constrained by mechanical limits
#define PT_MAXSTEER           400
#define PT_MAXTHRTL           450

// RC servo control neutral (equals to 1.5ms pulsewidth)
#define PT_MOTOR_NEUTRAL      1485
#define PT_SERVO_NEUTRAL      1400

// maximum values from control triggers
#define PT_TX_STEER_RANGE     935
#define PT_TX_THRTL_RANGE     950

/* nRF24 helper functions */
uint8_t check_wifi(void);
uint8_t nRF24_InitModule(void);

#ifdef nRF24_RX
void nRF24_ReceivePacket(uint8_t* payload, uint8_t* payload_length, uint8_t* ack);

#elif defined nRF24_TX
void nRF24_TransmitPacket(uint8_t *pBuf, uint8_t length);
nRF24_TXResult nRF24_FinishTransmission(void);
void nRF24_GetCounters(uint8_t* plos, uint8_t* arc);
#else
#error "You should define nRF24_TX or nRF24_RX macro (platform.h)!"
#endif
/* -------------------------------------------------------------------------- */

/* OLED helper functions */
void OLED_BatInfo(uint8_t x, uint8_t y, uint16_t milivolts, SSD1306_COLOR color, FontDef font, uint8_t show_battoff);
void OLED_TempInfo(uint8_t x, uint8_t y, uint8_t temp, uint8_t frac, SSD1306_COLOR color, FontDef font);
void OLED_SignalInfo(uint8_t x, uint8_t y, uint16_t freq, SSD1306_COLOR color, FontDef font);
/* -------------------------------------------------------------------------- */

/* UART helper functions */
#ifndef UART
#error "You should assign UART port to UART macro (platform.h)!"
#endif
uint8_t strlength(const char *s);
void UART_SendChar(char b);
void UART_SendStr(char *string);
void UART_SendBufHex(char *buf, uint16_t bufsize);
void UART_SendHex8(uint16_t num);
void UART_SendInt(int32_t num);
/* -------------------------------------------------------------------------- */

#endif // __PROJECT_TAMIYA_H
