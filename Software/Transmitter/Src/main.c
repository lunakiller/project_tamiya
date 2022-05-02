/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : ProjectTamiya - Control program for transmitter
  *                   Working:
  *                     - triggers with ADC "double buffer"
  *                       - split buffer in half, let the half fill, then process
  *                         it while the other half is being filled
  *                     - communication
  *                     - LEDs
  *                       - green:  communication OK, batt voltage >=6.8V
  *                       - red:    communication OK, batt voltage <6.8V
  *                       - blue:   no signal
  *                     - communication statistics (messages/sec)
  *                     - UART debug prints controled by Switch1
  *                     - control trimmers (rottary encoders)
  *                     - OLED status display with various info
  *                       - car batt voltage and bldc temperature
  *                       - tx batt voltage, msgs/sec, trigger and trimmer values
  *                       - signal loss
  *                     
  * @author         : Kristian Slehofer
  * @date           : 2. 5. 2022
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include <stdarg.h>

#include "project_tamiya.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUFF_SIZE 512

#define STEER0            2005                                                  // trigger default position ADC values
#define THRTL0            1924

#define RES_22K           21830.0                                               // real resistor values
#define RES_10K           9770.0 
#define BAT_DIVIDER       (RES_22K + RES_10K) / RES_10K                         // constant to compute battery voltage

#define VREFINT_CAL_ADDR  0x1FFFF7BA                                            // internal reference voltage calibration values address
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc3;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc3;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

bool no_signal = false, show_batoff = false;                                    // flags
bool NRF_IRQ = false, ENC1_IRQ = false, ENC1_reset = false, ENC2_IRQ = false, ENC2_reset = false;
bool display_refresh = false, trigger_data_rdy = false, adc_data_bat_rdy = false;

bool UART_debug = true;                                                         // flags controlled by dip switches

uint16_t STATUS_LED = LED_G_Pin;

uint16_t adc_data_bat[2*ADC_BUFF_SIZE];                                         // ADC data buffer
uint16_t buff_offset = 0;                                                       // ADC buffer offset

uint32_t voltage = 7500, vrefint;
uint16_t tx_freq = 0, tx_freq_cnt = 0;                                          // command frequency per sec
uint32_t retransmits_in_row = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM16_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM6_Init(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);

void BlinkLeds(void);
void PrintError(const char *s, ...);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_TIM2_Init();
  MX_ADC3_Init();
  MX_TIM4_Init();
  MX_TIM16_Init();
  MX_USART2_UART_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);                    // turn on LED

  ssd1306_Init();                                                               // init display
  ssd1306_DrawXBitmap(0, 0, tamiya_big_bits, tamiya_big_width, tamiya_big_height, White);
  ssd1306_UpdateScreen();

  HAL_UART_Transmit(&UART, (uint8_t*)GREETING, strlength(GREETING), UART_TIMEOUT);  // send greeting

  BlinkLeds();                                                                  // blink RGB led
  
  if(HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) !=  HAL_OK) {        // ADC calibration
    PrintError("ADC1 calib failed!");
    Error_Handler();
  }
  if(HAL_ADCEx_Calibration_Start(&hadc3, ADC_SINGLE_ENDED) !=  HAL_OK) {
    PrintError("ADC2 calib failed!");
    Error_Handler();
  }
  UART_SendStr("Both ADCs succesfully calibrated!\n");

  uint16_t vrefint_cal = *((uint16_t*)VREFINT_CAL_ADDR);                        // get internal ref voltage calibration value

  if(!nRF24_InitModule()) {                                                     // init communication module
    PrintError("nRF24 init error!");
    Error_Handler();
  };

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint8_t recv_len = 0;

  uint8_t buffer[PT_nRF24_PACKET_SIZE], ack[PT_nRF24_ACK_SIZE];                 // nRF24 data and ack buffer
    
  uint16_t triggers[2*ADC_BUFF_SIZE] = {0};                                     // ADC buffer and trimmers
  uint16_t last_enc1 = 0, last_enc2 = 0;
  int16_t steer_trim = 0, thrtl_trim = 0;
  
  int16_t throttle = 0, steer = 0;                                              // control values
  
  uint16_t car_voltage = 0;                                                     // car data
  uint8_t car_temp = 0, car_temp_frac = 0;
  
  bool sending = false;                                                         // flag for interrupt-driven packet transmission
  
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)triggers, 2*ADC_BUFF_SIZE);              // start ADC to get trigger positions

                                                                                // start timers
  HAL_TIM_Base_Start_IT(&htim16);                                               // OLED display refresh & BAT data (4Hz)
  HAL_TIM_Base_Start_IT(&htim6);                                                // per-sec statistics
  UART_SendStr("Timers started.\n");

                                                                                // init trimmers
  __HAL_TIM_SET_COUNTER(&htim2, 32768);                                         // reset to the middle
  last_enc1 = __HAL_TIM_GET_COUNTER(&htim2);
  __HAL_TIM_SET_COUNTER(&htim4, 32768);
  last_enc2 = __HAL_TIM_GET_COUNTER(&htim4);
  HAL_TIM_Encoder_Start_IT(&htim2, htim2.Channel);
  HAL_TIM_Encoder_Start_IT(&htim4, htim4.Channel);
  UART_SendStr("Rotary encoders initialized.\n");

  UART_SendStr("Entering main control loop...\n");

  if(HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == GPIO_PIN_SET) {                // deinitialize UART debug if switch is OFF
    UART_SendStr("Deinitializing UART...\n");
    HAL_UART_DeInit(&UART);
    UART_debug = false;
  }

  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);                  // turn off the led as the initialization is done

  while (1)
  {
    if(trigger_data_rdy) {                                                      // ADC conversion complete interrupt
      uint32_t throttle_raw = 0, steer_raw = 0;

      for(int i = 0; i < ADC_BUFF_SIZE; i += 2) {                               // average values
        throttle_raw += (uint16_t)triggers[i + 0 + buff_offset];
        steer_raw += (uint16_t)triggers[i + 1 + buff_offset];
      }
      throttle_raw /= ADC_BUFF_SIZE / 2;
      steer_raw /= ADC_BUFF_SIZE / 2;

      throttle = throttle_raw - THRTL0;                                         // "normalize" values
      steer = steer_raw - STEER0;

      throttle *= (float)PT_MAXTHRTL / (float)PT_TX_THRTL_RANGE;                // map values to max control range
      steer *= (float)PT_MAXSTEER / (float)PT_TX_STEER_RANGE;

      trigger_data_rdy = false;                                                 // reset flag
    }


    if(adc_data_bat_rdy) {
      voltage = vrefint = 0;                                                    // reset
      for(int i = 0; i < 2*ADC_BUFF_SIZE; i += 2) {                             // average values
        vrefint += (uint16_t)adc_data_bat[i + 0];
        voltage += (uint16_t)adc_data_bat[i + 1];
      }
      vrefint /= ADC_BUFF_SIZE;
      voltage /= ADC_BUFF_SIZE;

      uint16_t vdda = 3300 * vrefint_cal / vrefint;                             // calculate VDDA

      voltage = voltage * vdda / 4095;                                          // calculate the actual battery voltage
      voltage *= BAT_DIVIDER;

      if(UART_debug) {
        UART_SendStr("BAT: ");
        UART_SendInt(voltage);
        UART_SendStr("mV; SUPPLY: ");
        UART_SendInt(vdda);
        UART_SendStr("mV\n");
      }

      adc_data_bat_rdy = false;                                                 // reset flags
    }


    if(!sending) {                                                              // Send packet, phase 1 - interrupt driven sending
      sending = true;
      int16_t throttle_data, steer_data;

      throttle_data = throttle + thrtl_trim;                                    // prepare packet
      steer_data = steer + steer_trim;
      memmove(&buffer, &throttle_data, sizeof(int16_t));
      memmove(&buffer[2], &steer_data, sizeof(int16_t));

      nRF24_TransmitPacket(buffer, sizeof(buffer));
    }


    if(NRF_IRQ) {                                                               // Send packet, phase 2 - interrupt driven sending
      nRF24_TXResult tx_res;
      tx_res = nRF24_FinishTransmission();
      nRF24_ReadPayloadDpl(ack, &recv_len);

      if(recv_len == PT_nRF24_ACK_SIZE) {
        uint8_t temp_byte = 0;
        memmove(&car_voltage, &ack[0], sizeof(uint16_t));                       // parse ACK packet
        memmove(&temp_byte, &ack[2], sizeof(uint8_t));
        car_temp = (temp_byte & 0x7F);
        car_temp_frac = (temp_byte >> 7) & 0x1 ? 5 : 0;
      } else {
        car_voltage = 0, car_temp = 0, car_temp_frac = 0;
      }

      // nRF24_GetCounters(&plos, &arc);
      switch(tx_res) {
        case nRF24_TX_SUCCESS:
          HAL_GPIO_TogglePin(GPIOA, STATUS_LED);                                // toggle led

          retransmits_in_row = 0;                                               // reset
          tx_freq_cnt++;                                                        // increment messages/sec counter
          break;
        case nRF24_TX_TIMEOUT:
          // UART_SendStr("TIMEOUT");
          break;
        case nRF24_TX_MAXRT:
          // UART_SendStr("MAX RETRANSMIT");
          retransmits_in_row++;
          nRF24_ResetPLOS();
          break;
        default:
          // UART_SendStr("ERROR");
          break;
      }

      sending = NRF_IRQ = false;                                                // reset flags
    }


    if(display_refresh) {                                                       // OLED display refresh interrupt
      ssd1306_Fill(Black);
      ssd1306_DrawXBitmap(0, 3, car_logo_bits, car_logo_width, car_logo_height, White);
      if(no_signal) {                                                           // if no signal detected, print it on display
        ssd1306_SetCursor(35, 2);
        ssd1306_WriteString("NO SIGNAL!", Font_7x10, White);
      } else {                                                                  // else print car status
        OLED_TempInfo(27, 0, car_temp, car_temp_frac, White, Font_6x8);
        OLED_BatInfo(96, 0, car_voltage, White, Font_6x8, show_batoff);
      }
      OLED_BatInfo(96, 50, voltage, White, Font_6x8, show_batoff);
      ssd1306_DrawHLine(13, White);

      OLED_SignalInfo(0, 50, tx_freq, White, Font_6x8);                         // display commands per sec

      char ascii_buffer[8];
      ssd1306_SetCursor(6, 22);                                                 // display trigger values
      ssd1306_WriteString("STEER: ", Font_7x10, White);
      snprintf(ascii_buffer, 8, "%d", steer);
      ssd1306_WriteString(ascii_buffer, Font_7x10, White);

      ssd1306_SetCursor(6, 36);
      ssd1306_WriteString("THRTL: ", Font_7x10, White);
      snprintf(ascii_buffer, 8, "%d", throttle);
      ssd1306_WriteString(ascii_buffer, Font_7x10, White);

      if(steer_trim >= 0) {                                                     // display trimmer values
        snprintf(ascii_buffer, 8, "+%i", steer_trim);
      } else {
        snprintf(ascii_buffer, 8, "%i", steer_trim);
      }
      ssd1306_SetCursor(100, 22);
      ssd1306_WriteString(ascii_buffer, Font_7x10, White);

      if(thrtl_trim >= 0) {
        snprintf(ascii_buffer, 8, "+%i", thrtl_trim);
      } else {
        snprintf(ascii_buffer, 8, "%i", thrtl_trim);
      }
      ssd1306_SetCursor(100, 36);
      ssd1306_WriteString(ascii_buffer, Font_7x10, White);
      
      ssd1306_UpdateScreen();                                                   // print values

      display_refresh = false;                                                  // reset flag
    }


    if(ENC1_IRQ) {                                                              // Steering trim control
      if(__HAL_TIM_GET_COUNTER(&htim2) > last_enc1) {
        UART_SendStr("ENC1 - UP\n");
        steer_trim += 1;
      } else {
        UART_SendStr("ENC1 - DOWN\n");
        steer_trim -= 1;
      }
      last_enc1 = __HAL_TIM_GET_COUNTER(&htim2);
      ENC1_IRQ = false;                                                         // reset flag
    }


    if(ENC1_reset) {
      steer_trim = 0;
      ENC1_reset = false;
    }


    if(ENC2_IRQ) {                                                              // Throttle trim control
      if(__HAL_TIM_GET_COUNTER(&htim4) < last_enc2) {                           // "<" to maintain same direction
        UART_SendStr("ENC2 - UP\n");
        thrtl_trim += 1;
      } else {
        UART_SendStr("ENC2 - DOWN\n");
        thrtl_trim -= 1;
      }
      last_enc2 = __HAL_TIM_GET_COUNTER(&htim4);
      ENC2_IRQ = false;                                                         // reset flag
    }


    if(ENC2_reset) {
      thrtl_trim = 0;
      ENC2_reset = false;
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_I2C1
                              |RCC_PERIPHCLK_ADC12|RCC_PERIPHCLK_ADC34;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.Adc34ClockSelection = RCC_ADC34PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Enables the Clock Security System
  */
  HAL_RCC_EnableCSS();
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */
  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 2;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc3, &multimode) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_VREFINT;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_181CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */

  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x0000020B;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 10;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 10;
  if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 7199;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 9999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM16 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM16_Init(void)
{

  /* USER CODE BEGIN TIM16_Init 0 */

  /* USER CODE END TIM16_Init 0 */

  /* USER CODE BEGIN TIM16_Init 1 */

  /* USER CODE END TIM16_Init 1 */
  htim16.Instance = TIM16;
  htim16.Init.Prescaler = 7199;
  htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim16.Init.Period = 2499;
  htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim16) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM16_Init 2 */

  /* USER CODE END TIM16_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 921600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
  /* DMA2_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel5_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED_R_Pin|LED_G_Pin|NRF_CSN_Pin|NRF_CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA4 PA5 PA10 PA11
                           PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_R_Pin LED_G_Pin NRF_CSN_Pin NRF_CE_Pin */
  GPIO_InitStruct.Pin = LED_R_Pin|LED_G_Pin|NRF_CSN_Pin|NRF_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_B_Pin */
  GPIO_InitStruct.Pin = LED_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_B_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SW1_Pin */
  GPIO_InitStruct.Pin = SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SW1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SW2_Pin ENC1_BTN_Pin ENC2_BTN_Pin */
  GPIO_InitStruct.Pin = SW2_Pin|ENC1_BTN_Pin|ENC2_BTN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : NRF_IRQ_Pin */
  GPIO_InitStruct.Pin = NRF_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(NRF_IRQ_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

/* CALLBACKS */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
  if(GPIO_Pin == NRF_IRQ_Pin) {                                                 // nRF24 interrupt
    NRF_IRQ = true;
  }
  else if(GPIO_Pin == SW1_Pin) {                                                // switch 1 (UART)
    UART_SendStr("SW1\n");

    if(HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == GPIO_PIN_RESET) {
      MX_USART2_UART_Init();
      UART_debug = true;
      UART_SendStr("UART initialized!\n");
    } else {
      UART_SendStr("Deinitializing UART...\n");
      HAL_UART_DeInit(&UART);
      UART_debug = false;
    }
  }
  else if(GPIO_Pin == SW2_Pin) {                                                // switch 2 (no functionality)
    UART_SendStr("SW2\n");
  }
  else if(GPIO_Pin == ENC1_BTN_Pin) {
    ENC1_reset = true;
  }
  else if(GPIO_Pin == ENC2_BTN_Pin) {
    ENC2_reset = true;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if(htim->Instance == TIM6) {                                                  // 1Hz timer interrupt
    tx_freq = tx_freq_cnt;                                                      // save value
    tx_freq_cnt = 0;                                                            // reset counter

    show_batoff = !show_batoff;                                                 // blink battery-off logo

    if(voltage < 6800 && STATUS_LED != LED_R_Pin) {                             // change status led color according to voltage level
      HAL_GPIO_WritePin(GPIOA, LED_G_Pin, GPIO_PIN_RESET);                      // turn off green LED
      STATUS_LED = LED_R_Pin;
    }
    else {
      HAL_GPIO_WritePin(GPIOA, LED_R_Pin, GPIO_PIN_RESET);                      // turn off red LED
      STATUS_LED = LED_G_Pin;
    }
  }
  else if(htim->Instance == TIM16) {                                            // 4Hz display refresh and batt data
    display_refresh = true;
    HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adc_data_bat, ADC_BUFF_SIZE*2);

    if(retransmits_in_row > 15) {                                               // no signal
      HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);            // reset status led
      HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
      
      HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);              // turn on no signal led
      UART_SendStr("NO SIGNAL!\n");
      no_signal = true;
    } else {
      HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
    }
  }
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
  if(htim->Instance == TIM2) {                                                  // encoder 1 moved
    ENC1_IRQ = true;
  } else if(htim->Instance == TIM4) {                                           // encoder 2 moved
    ENC2_IRQ = true;
  }
}

void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc) {
  if(hadc->Instance == ADC1) {                                                  // trigger buffer is half-full
    trigger_data_rdy = true;                                                    // set flag
    buff_offset = 0;                                                            // set offset to 0 to process the first half of the buffer
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  if(hadc->Instance == ADC1) {                                                  // trigger buffer is full
    trigger_data_rdy = true;                                                    // set flag, the first half of the buffer should be already processed
    buff_offset = ADC_BUFF_SIZE;                                                // set offset to the middle of the buffer to process the second half
  }
  else if(hadc->Instance == ADC3) {
    adc_data_bat_rdy = true;
  }
}
/* -------------------------------------------------------------------------- */

/* FUNCTIONS */
void BlinkLeds(void) {
  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
  HAL_Delay(250);
  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
  HAL_Delay(250);
  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);
  HAL_Delay(250);
  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);
}

void PrintError(const char *s, ...) {                                           // printf functionality
  static char buffer[256];
  va_list args;
  va_start(args, s);
  vsnprintf(buffer, sizeof(buffer), s, args);
  va_end(args);

  ssd1306_Fill(Black);
  ssd1306_SetCursor(5, 12);
  ssd1306_WriteString(buffer, Font_6x8, White);
  ssd1306_UpdateScreen();

  if(UART_debug) {
    UART_SendStr(buffer);
    UART_SendStr("\n");
  }
}
/* -------------------------------------------------------------------------- */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);                  // white light
  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);

  while(1) {                                                                    // periodically blink small led
    HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
    HAL_Delay(500);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
