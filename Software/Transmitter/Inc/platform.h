/**
  ******************************************************************************
  * @file           : platform.h
  * @brief          : Single header file with platform specific parameters
  *                   and functions
  *                     
  * @author         : Kristian Slehofer
  * @date           : 19. 4. 2022
  ******************************************************************************
  */

#ifndef __PLATFORM_H
#define __PLATFORM_H

#include "main.h"
#include "tamiya_big.h"     // logo
#include "car_logo.h"

#define GREETING "\n\n-- ProjectTamiya - Transmitter --\n"

#define nRF24_TX
#define nRF24_TIMEOUT     250
#define UART_TIMEOUT      250

#define UART              huart2
#define nRF24_SPI         hspi2
#define SSD1306_I2C_PORT  hi2c1



extern UART_HandleTypeDef UART;
extern SPI_HandleTypeDef nRF24_SPI;


static inline void nRF24_CE_L() {
  HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_RESET);
}

static inline void nRF24_CE_H() {
  HAL_GPIO_WritePin(NRF_CE_GPIO_Port, NRF_CE_Pin, GPIO_PIN_SET);
}

static inline void nRF24_CSN_L() {
  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_RESET);
}

static inline void nRF24_CSN_H() {
  HAL_GPIO_WritePin(NRF_CSN_GPIO_Port, NRF_CSN_Pin, GPIO_PIN_SET);
}

static inline uint8_t nRF24_LL_RW(uint8_t data) {
  // Wait until TX buffer is empty
  uint8_t result;
  if(HAL_SPI_TransmitReceive(&nRF24_SPI, &data, &result, 1, nRF24_TIMEOUT) != HAL_OK) {
      Error_Handler();
  }
  return result;
}

static inline void Delay_ms(uint32_t ms) { HAL_Delay(ms); }


#endif // __PLATFORM_H
