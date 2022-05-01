/**
  ******************************************************************************
  * @file           : project_tamiya.c
  * @brief          : Common functions implementations
  *                     
  * @author         : Kristian Slehofer
  * @date           : 24. 4. 2022
  ******************************************************************************
  */

#include "project_tamiya.h"
#include <stdio.h>

// Check module presence on bus.
// Most of the times the first try fails, hence the loop
// return:
//    0 - module not present
//    1 - module present and online
uint8_t check_wifi(void) {
  volatile uint8_t nrf_active = 0;
  uint8_t i = 0;

  UART_SendStr("nRF24L01+ check...\n");

  do {
    nrf_active = nRF24_Check();
    HAL_Delay(100);
    ++i;
    } while (!nrf_active && i < 10);

  UART_SendStr("Module status: ");
  if(nrf_active) {
    UART_SendStr("OK!\n");
  } else {
    UART_SendStr("FAILED!\n");
    // Error_Handler();
  }
  return nrf_active;
}

// Init module in RX/TX mode (controlled by macro)
// return:
//    0 - module not initialized
//    1 - module successfully initialized
uint8_t nRF24_InitModule(void) {
  // RX/TX disabled
  nRF24_CE_L();

  // Check module presence
  if(!check_wifi())
    return 0;

  // Initialize the nRF24L01 to its default state
  nRF24_Init();

  // Set RF channel
  nRF24_SetRFChannel(PT_nRF24_CHANNEL);

  // Set data rate
  nRF24_SetDataRate(PT_nRF24_DATARATE);

  // Set CRC scheme
  nRF24_SetCRCScheme(PT_nRF24_CRC);

  // Set address width (common for all pipes (RX and TX))
  nRF24_SetAddrWidth(PT_nRF24_ADDR_WIDTH);

  static const uint8_t nRF24_ADDR[] = { PT_nRF24_ADDR };
#ifdef nRF24_RX
  // Configure RX PIPE
  nRF24_SetAddr(nRF24_PIPE1, nRF24_ADDR); // program RX adress
  nRF24_SetRXPipe(nRF24_PIPE1, nRF24_AA_ON, PT_nRF24_ACK_SIZE); // Auto-ACK: enabled
  
#elif defined nRF24_TX
  // Configure TX PIPE
  nRF24_SetAddr(nRF24_PIPETX, nRF24_ADDR); // program TX address
  nRF24_SetAddr(nRF24_PIPE0, nRF24_ADDR); // addr for pipe0 must be same as TX
#endif // nRF24_TX

  // Set TX power (maximum)
  nRF24_SetTXPower(PT_nRF24_TXPWR);

#ifdef nRF24_RX
  // Set operational mode (RX == receiver)
  nRF24_SetOperationalMode(nRF24_MODE_RX);
  
#elif defined nRF24_TX
  // Configure auto retransmit: 5 retransmissions with pause of 2500us in between
  nRF24_SetAutoRetr(nRF24_ARD_2500us, 5);

  // Enable Auto-ACK for pipe#0 (for ACK packets)
  nRF24_EnableAA(nRF24_PIPE0);

  // Set operational mode (TX == transmitter)
  nRF24_SetOperationalMode(nRF24_MODE_TX);
#endif // nRF24_TX

  // Clear any pending IRQ flags
  nRF24_ClearIRQFlags();

  // Enable DPL
  nRF24_SetDynamicPayloadLength(nRF24_DPL_ON);
  nRF24_SetPayloadWithAck(1);

  // Wake the transceiver
  nRF24_SetPowerMode(nRF24_PWR_UP);

  // Flush both FIFOs, just in case
  nRF24_FlushRX();
  nRF24_FlushTX();

#ifdef nRF24_RX
  // Put the transceiver to the RX mode
  nRF24_CE_H();
#endif // nRF24_RX

  return 1;
}


#ifdef nRF24_RX
// Receive the data packet
void nRF24_ReceivePacket(uint8_t* payload, uint8_t* payload_length, uint8_t* ack) {
  nRF24_RXResult pipe;

  pipe = nRF24_ReadPayloadDpl(payload, payload_length);

  if(payload_length > 0) {
      nRF24_WriteAckPayload(pipe, (char*)ack, PT_nRF24_ACK_SIZE);
  }

  // Clear all pending IRQ flags
  nRF24_ClearIRQFlags();

  // Print a payload contents to UART
  // UART_SendStr("RCV PIPE#");
  // UART_SendInt(pipe);
  // UART_SendStr(" PAYLOAD:>");
  // UART_SendBufHex((char *) payload, *payload_length);
  // UART_SendStr("<\r\n");

  return;
}
#endif // nRF24_RX

#ifdef nRF24_TX
// Function to transmit data packet, interrupt-driven, phase 1
// Initialize the transmission, wait for the interrupt
void nRF24_TransmitPacket(uint8_t *pBuf, uint8_t length) {
  // Deassert the CE pin (in case if it still high)
  nRF24_CE_L();

  // Transfer a data from the specified buffer to the TX FIFO
  nRF24_WritePayload(pBuf, length);

  // Start a transmission by asserting CE pin (must be held at least 10us)
  nRF24_CE_H();
  return ;
}

// Function to transmit data packet, interrupt-driven, phase 2
// Finish transmission and get the result when interrupt was triggered
nRF24_TXResult nRF24_FinishTransmission(void) {
    uint8_t status;
    nRF24_TXResult result;

    // Deassert the CE pin (Standby-II --> Standby-I)
    nRF24_CE_L();

    // Check the flags in STATUS register
    status = nRF24_GetStatus();
    // UART_SendStr("[");
    // UART_SendHex8(status);
    // UART_SendStr("] ");

    // Clear pending IRQ flags
    nRF24_ClearIRQFlags();

    if (status & nRF24_FLAG_MAX_RT) {
        // Auto retransmit counter exceeds the programmed maximum limit (FIFO is not removed)
        result = nRF24_TX_MAXRT;
    }

    if (status & nRF24_FLAG_TX_DS) {
        // Successful transmission
        result = nRF24_TX_SUCCESS;
    }

    // Some banana happens, a payload remains in the TX FIFO, flush it
    nRF24_FlushTX();

    return result;
}

// Get retransmit and packet lost counters
void nRF24_GetCounters(uint8_t* plos, uint8_t* arc) {
    uint8_t otx = nRF24_GetRetransmitCounters();
    *plos = (otx & nRF24_MASK_PLOS_CNT) >> 4; // packets lost counter
    *arc  = (otx & nRF24_MASK_ARC_CNT); // auto retransmissions counter
    return;
}
#endif // nRF24_TX

/* OLED helper functions */
void OLED_BatInfo(uint8_t x, uint8_t y, uint16_t milivolts, SSD1306_COLOR color, FontDef font) {
  if(milivolts > 8050) {    // battery visualization
    ssd1306_DrawPixel(x + 2, y + 3, White);
    ssd1306_DrawPixel(x + 3, y + 3, White);
  }
  if(milivolts > 7650) {
    ssd1306_DrawPixel(x + 2, y + 5, White);
    ssd1306_DrawPixel(x + 3, y + 5, White);
  }
  if(milivolts > 7350) {
    ssd1306_DrawPixel(x + 2, y + 7, White);
    ssd1306_DrawPixel(x + 3, y + 7, White);
  }
  if(milivolts > 7000) {
    ssd1306_DrawPixel(x + 2, y + 9, White);
    ssd1306_DrawPixel(x + 3, y + 9, White);
  }
  if(milivolts < 6800) {
    ssd1306_DrawXBitmap(x, y, batoff_logo_bits, 6, 12, White);
  } else {
    ssd1306_DrawXBitmap(x, y, bat_logo_bits, 6, 12, White);
  }

  char ascii_bat[10];   // string buffer
  uint8_t volts = milivolts / 1000;
  milivolts = ((milivolts % 1000) + 50) / 100;    // rounding
    if(milivolts > 9) milivolts = 9;
  snprintf(ascii_bat, 10, "%u,%uV", volts, milivolts);
  ssd1306_SetCursor(x + 8, y + 3);
  ssd1306_WriteString(ascii_bat, font, White);
} 

void OLED_TempInfo(uint8_t x, uint8_t y, uint8_t temp, uint8_t frac, SSD1306_COLOR color, FontDef font) {
  char ascii_temp[8];
  snprintf(ascii_temp, 8, "%u", temp);
  ssd1306_DrawXBitmap(x, y, temp_logo_bits, 13, 12, White);
  ssd1306_SetCursor(x + 15, y + 3);
  ssd1306_WriteString(ascii_temp, font, White);
  if(frac > 0 && frac < 10) {
    ssd1306_WriteChar(',', font, White);
    ssd1306_WriteChar('0' + frac, font, White);
  }
  ssd1306_WriteChar(' ', font, White);
  ssd1306_WriteChar('C', font, White);
  ssd1306_MoveCursor(-(font.FontWidth + 4), 0);
  ssd1306_WriteCircle(White);
}


/* UART helper functions */
uint8_t strlength(const char *s) {
    size_t i;
    for (i = 0; s[i] != '\0'; i++) ;
    return i;
}

#define HEX_CHARS      "0123456789ABCDEF"

void UART_SendChar(char b) {
    HAL_UART_Transmit(&UART, (uint8_t*)&b, 1, UART_TIMEOUT);
}

void UART_SendStr(char *string) {
    HAL_UART_Transmit(&UART, (uint8_t*)string, (uint16_t)strlength(string), UART_TIMEOUT);
}

void UART_SendBufHex(char *buf, uint16_t bufsize) {
    uint16_t i;
    char ch;
    for (i = 0; i < bufsize; i++) {
        ch = *buf++;
        UART_SendChar(HEX_CHARS[(ch >> 4)   % 0x10]);
        UART_SendChar(HEX_CHARS[(ch & 0x0f) % 0x10]);
    }
}
void UART_SendHex8(uint16_t num) {
    UART_SendChar(HEX_CHARS[(num >> 4)   % 0x10]);
    UART_SendChar(HEX_CHARS[(num & 0x0f) % 0x10]);
}

void UART_SendInt(int32_t num) {
    char str[10]; // 10 chars max for INT32_MAX
    int i = 0;
    if (num < 0) {
        UART_SendChar('-');
        num *= -1;
    }
    do str[i++] = (char)(num % 10 + '0'); while ((num /= 10) > 0);
    for (i--; i >= 0; i--) UART_SendChar(str[i]);
}
