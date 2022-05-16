/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : ProjectTamiya - Control program for car
  *                   Working:
  *                     - communication
  *                     - driving (both servo & bldc control)
  *                     - LEDs
  *                       - green:  communication OK, batt voltage >=6.8V
  *                       - red:    communication OK, batt voltage <6.8V
  *                       - blue:   no signal
  *                       - white:  error (Error_Handler())
  *                     - safety timer (250ms no command -> car stops)
  *                     - battery voltage measurements (calibrated)
  *                     - current measurements (calibrated?)
  *                     - gyro/accel measurements
  *                     - BLDC temperature measurements
  *                     - UART debug prints controlled by Switch1
  *                     - SD card logging controlled by Switch2
  *                     - OLED display with batt voltage, bldc temp and msgs/sec
  *                       - controlled by button
  * 
  *                     
  * @author         : Kristian Slehofer
  * @date           : 16. 5. 2022
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include <string.h>
#include <math.h>

#include <stdarg.h>
#include <stdio.h>

#include "project_tamiya.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADC_BUFF_SIZE     512

#define RES_22K           22050.0                                               // real resistor values
#define RES_10K           9710.0 
#define BAT_DIVIDER       (RES_22K + RES_10K) / RES_10K                         // constant to compute battery voltage

#define RES_47K_5V        46710.0
#define RES_47K_GND       46600.0
#define SPLY_DIVIDER      (RES_47K_5V + RES_47K_GND) / RES_47K_GND              // constant to compute supply voltage

#define VREFINT_CAL_ADDR  0x1FFFF7BA                                            // internal reference voltage calibration values address

#define SD_FORMAT         "TIME,VOLTAGE,CURRENT,BLDC_TEMP,STEER,THRTL,ACC_X,ACC_Y,ACC_Z,GYRO_X,GYRO_Y,GYRO_Z,MPU_TEMP\n"
#define SD_FORMAT_UNITS   "ms,mV,mA,deg C,-,-,mg,mg,mg,deg/s,deg/s,deg/s, deg C\n"
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
bool NRF_IRQ = false, adc_data_bat_rdy = false, adc_data_sply_rdy = false;      // flags
bool display_refresh = false, temp_received = false, temp_conv_ready = false, show_batoff = false;

bool UART_debug = true, log_data = false, unmount_sd = false;                   // flags controlled by dip switches

bool display_wakeup = false;                                                    // flag controlled by button

uint16_t STATUS_LED = LED_G_Pin;

MPU6050_t mpu;                                                                  // MPU6050 data struct

uint8_t temp_receive_buffer[9] = {0};                                           // temperature data buffer

uint16_t adc_data_bat[ADC_BUFF_SIZE*2], adc_data_sply[ADC_BUFF_SIZE*2];         // ADC data buffer

uint32_t voltage = 7500, current, vrefint, supply5, temp, temp_frac;            // values
uint16_t tx_freq = 0, tx_freq_cnt = 0;                                          // command frequency per sec
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI2_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM8_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM17_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM16_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM15_Init(void);
/* USER CODE BEGIN PFP */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);

void BlinkLeds(void);
void OW_Complete(void);
void OW_Error(void);
void DS_Convert(void);
void DS_Read(uint8_t* buffer);
uint16_t DS_GetIntTemp(uint16_t temp);
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
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  MX_I2C1_Init();
  MX_SPI2_Init();
  MX_I2C2_Init();
  MX_TIM8_Init();
  MX_USART3_UART_Init();
  MX_FATFS_Init();
  MX_TIM1_Init();
  MX_TIM17_Init();
  MX_TIM6_Init();
  MX_TIM16_Init();
  MX_ADC2_Init();
  MX_TIM15_Init();
  /* USER CODE BEGIN 2 */

  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);                  // turn on LED

  ssd1306_Init();                                                               // init display
  ssd1306_DrawXBitmap(0, 0, tamiya_car_bits, tamiya_car_width, tamiya_car_height, White);
  ssd1306_UpdateScreen();

  HAL_UART_Transmit(&UART, (uint8_t*)GREETING, strlength(GREETING), UART_TIMEOUT);  // send greeting

  if(HAL_GPIO_ReadPin(SW2_GPIO_Port, SW2_Pin) == GPIO_PIN_RESET) {              // enable SD logging only if switch is ON
    UART_SendStr("Logging enabled.\n");
    if(!MPU6050_Init()) {
      PrintError("MPU init error!");
      Error_Handler();
    }
    log_data = true;
    UART_SendStr("MPU6050 initialized!\n");
  }

                                                                                // start PWMs
  HAL_TIM_PWM_Start(&htim8, SERVO_CHANNEL);                                     // servo
  HAL_TIM_PWM_Start(&htim8, BLDC_CHANNEL);                                      // bldc
  // __HAL_TIM_SET_COMPARE(&htim8, SERVO_CHANNEL, PT_RC_NEUTRAL);               // reset servo
  // __HAL_TIM_SET_COMPARE(&htim8, BLDC_CHANNEL, PT_RC_NEUTRAL);                // reset bldc
  UART_SendStr("PWMs started!\n");

  OneWire_Init();                                                               // init 1-Wire bus and temperature sensor
  OneWire_SetCallback(OW_Complete, OW_Error);
  DS_Convert();                                                                 // request first temperature conversion (~750ms)
  
  BlinkLeds();                                                                  // blink RGB led

  if(HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED) !=  HAL_OK) {        // ADC calibration
    PrintError("ADC1 calib failed!");
    Error_Handler();
  }
  if(HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED) !=  HAL_OK) {
    PrintError("ADC2 calib failed!");
    Error_Handler();
  }
  UART_SendStr("Both ADCs succesfully calibrated!\n");

  uint16_t vrefint_cal = *((uint16_t*)VREFINT_CAL_ADDR);                        // get internal ref voltage calibration value

  if(!nRF24_InitModule()) {                                                     // init communication module
    PrintError("nRF24 init error!");
    Error_Handler();
  }

  FATFS FatFs;                                                                  // variables for FatFS
  FIL file;
  volatile FRESULT fres;
  static char log_buffer[128];

  if(log_data) {                                                                // prepare SD card
    UART_SendStr("Preparing SD card...\n");

    fres = f_mount(&FatFs, "", 1);                                              // 1 = mount now
    if(fres != FR_OK) {
      PrintError("f_mount error (%i)!", fres);
      Error_Handler();
    }

    DWORD free_clusters, free_sectors, total_sectors;                           // SD card statistics
    FATFS* getFreeFs;

    fres = f_getfree("", &free_clusters, &getFreeFs);
    if(fres != FR_OK) {
      PrintError("f_getfree error (%i)!", fres);
      Error_Handler();
    }

    total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;               // formula comes from ChaN's FatFS documentation
    free_sectors = free_clusters * getFreeFs->csize;

    UART_SendStr("SD card stats:\n  ");
    UART_SendInt(total_sectors / 2);
    UART_SendStr(" KiB total drive space, ");
    UART_SendInt(free_sectors / 2);
    UART_SendStr(" KiB available.\n");

    
    DIR dir;                                                                    // get number of files on SD
    FILINFO fno;
    uint16_t num_files = 0;

    UART_SendStr("Opening /logs folder...\n");
    fres = f_opendir(&dir, "/logs");                                            // open logs folder
    if(fres == FR_NO_PATH) {
      fres = f_mkdir("logs");                                                   // create it if it does not exist
      if (fres != FR_OK) {
        PrintError("f_mkdir error (%i)!", fres);
        Error_Handler();
      }
    }
    
    do {                                                                        // iterate over files in folder
        f_readdir(&dir, &fno);
        if(fno.fname[0] != 0) {
          UART_SendStr("File found: ");
          UART_SendStr(fno.fname);
          UART_SendStr("\n");

          num_files++;                                                          // increment counter
        }
    } while(fno.fname[0] != 0);
    f_closedir(&dir);

    UART_SendStr("Number of files in /logs folder: ");
    UART_SendInt(num_files);
    UART_SendStr("\n");

    static char log_fname[64];                                                  // static inits to zeros
    sprintf(log_fname, "/logs/log_%03i.csv", num_files);

    fres = f_open(&file, log_fname, FA_CREATE_ALWAYS | FA_WRITE);               // try to open file
    if(fres != FR_OK) {
      PrintError("f_open error (%i)!", fres);
      Error_Handler();
    }

    fres = f_lseek(&file, 10240000);                                             // pre-allocate 10MiB of space, shoul be enough for ~40 min of logs
    if(fres != FR_OK || f_tell(&file) != 10240000) {
      PrintError("f_lseek error (%i)!", fres);
      Error_Handler();
    }

    fres = f_lseek(&file, 0);                                                   // return back to the beginning
    if(fres != FR_OK || f_tell(&file) != 0) {
      PrintError("f_lseek error (%i)!", fres);
      Error_Handler();
    }
    UART_SendStr("Pre-allocation done.\n");

    strncpy(log_buffer, SD_FORMAT, strlen(SD_FORMAT)+1);                        // start file with description of units
    UINT bytesWrote;
    fres = f_write(&file, log_buffer, strlen(log_buffer), &bytesWrote);

    strncpy(log_buffer, SD_FORMAT_UNITS, strlen(SD_FORMAT_UNITS)+1);
    fres = f_write(&file, log_buffer, strlen(log_buffer), &bytesWrote);
    if(fres == FR_OK) {
      UART_SendStr("Write to ");
      UART_SendStr(log_fname);
      UART_SendStr(" succesful.\n");
    } else {
      PrintError("f_write error (%i)!", fres);
      Error_Handler();
    }
    f_sync(&file);                                                              // sync file

    UART_SendStr("SD card ready!\n");
  }

                                                                                // start timers
  HAL_TIM_Base_Start_IT(&htim1);                                                // 50hz data
  HAL_TIM_Base_Start_IT(&htim17);                                               // safety timer (4Hz)
  HAL_TIM_Base_Start_IT(&htim16);                                               // OLED display refresh (4Hz)
  HAL_TIM_Base_Start_IT(&htim6);                                                // per-sec statistics
  HAL_TIM_Base_Start_IT(&htim15);                                               // display ON timer; to trigger the first interrupt
  UART_SendStr("Timers started.\n");

  HAL_TIM_Base_Stop_IT(&htim15);                                                // turn the timer back off and wait for button

  ssd1306_SetDisplayOn(0);                                                      // turn off display

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  uint8_t status, size;                                                         // nRF24 variables
  
  uint8_t buffer[PT_nRF24_PACKET_SIZE], ack[PT_nRF24_ACK_SIZE];                 // nRF24 msg and ack bufferš
  
  int16_t throttle = 0, steer = 0;                                              // control values

  UART_SendStr("Entering main control loop...\n");

  if(HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == GPIO_PIN_SET) {                // deinitialize UART debug if switch is OFF
    UART_SendStr("Deinitializing UART...\n");
    HAL_UART_DeInit(&UART);
    UART_debug = false;
  }

  HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);                    // turn off the led as the initialization is done

  uint32_t starttime = HAL_GetTick();
  uint32_t bytes_total = 0;

  // CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;                            // enable cycle counter for debug purposes
  // DWT->CYCCNT = 0;
  // DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  // volatile unsigned long t1 = 0, t2 = 0;
  // t1 = DWT->CYCCNT;
  // t2 = DWT->CYCCNT;

  while (1)
  {
    if(NRF_IRQ) {                                                               // nRF external interrupt
      status = nRF24_GetStatus();

      if(status & nRF24_FLAG_RX_DR) {                                           // data in RX data register
        uint8_t temp_byte = (temp & 0x7F) | (temp_frac ? (1 << 7) : (0 << 7));  // prepare ACK packet
        memmove(&ack[0], &voltage, sizeof(uint16_t));
        memmove(&ack[2], &temp_byte, sizeof(uint8_t));
        
        nRF24_ReceivePacket(buffer, &size, ack);

        if(size == PT_nRF24_PACKET_SIZE) {
          memmove(&throttle, &buffer, sizeof(int16_t));                         // parse control data
          memmove(&steer, &buffer[2], sizeof(int16_t));

          if(steer > PT_MAXSTEER) {                                             // check and clip values
            steer = PT_MAXSTEER;
          }
          else if(steer < -PT_MAXSTEER) {
            steer = -PT_MAXSTEER;
          }
          if(throttle > PT_MAXTHRTL) {
            throttle = PT_MAXTHRTL;
          }
          else if(throttle < -PT_MAXTHRTL) {
            throttle = -PT_MAXTHRTL;
          }
          
          __HAL_TIM_SET_COMPARE(&htim8, SERVO_CHANNEL, PT_RC_NEUTRAL - steer);  // update PWM settings
          __HAL_TIM_SET_COMPARE(&htim8, BLDC_CHANNEL, PT_RC_NEUTRAL - throttle);

          __HAL_TIM_SET_COUNTER(&htim17, 0);                                    // reset safety timer

          tx_freq_cnt++;
        }
      }

      NRF_IRQ = false;                                                          // reset flag
    }


    if(temp_conv_ready) {                                                       // temperature conversion in sensor ready
      memset(temp_receive_buffer, 0, sizeof(temp_receive_buffer));
      DS_Read((uint8_t*)&temp_receive_buffer);

      if(log_data) {                                                            // also sync the file if logging is enabled
        f_sync(&file);

        if(UART_debug) {
          UART_SendStr("FILE SYNCED! ");
          UART_SendInt(bytes_total);
          UART_SendStr(" bytes total\n");
        }

        bytes_total = 0;                                                        // reset counter
      }
    }


    if(temp_received) {                                                         // temperature received   
      uint16_t temp_raw = (temp_receive_buffer[1] << 8) | temp_receive_buffer[0];
      temp = DS_GetIntTemp(temp_raw);                                           // convert integer temperature

      if((temp_raw & (0x8)) == 0x8) {                                           // check bit associated with 2^(-1) deg C
        temp_frac = 5;
      } else {
        temp_frac = 0;
      }
      
      temp_received = false;                                                    // reset flag
      DS_Convert();                                                             // start next conversion
    }


    if(adc_data_bat_rdy && adc_data_sply_rdy) {                                 // ADC conversion complete interrupt (20Hz)
      voltage = current = vrefint = supply5 = 0;                                // reset

      for(int i = 0; i < 2*ADC_BUFF_SIZE; i += 2) {                             // average values
        voltage += (uint16_t)adc_data_bat[i + 0];
        current += (uint16_t)adc_data_bat[i + 1];
        vrefint += (uint16_t)adc_data_sply[i + 0];
        supply5 += (uint16_t)adc_data_sply[i + 1];
      }
      voltage /= ADC_BUFF_SIZE;
      current /= ADC_BUFF_SIZE;
      vrefint /= ADC_BUFF_SIZE;
      supply5 /= ADC_BUFF_SIZE;

      uint16_t vdda = 3300 * vrefint_cal / vrefint;                             //calculate VDDA
      
      supply5 = supply5 * vdda / 4095 + 24;                                     // calculate supply voltage (5V)
                                                                                // 24mV is a "magic constant", it is an offset between BOOT1 and actual MCU pin
      supply5 *= SPLY_DIVIDER;
      voltage = voltage * vdda / 4095;                                          // calculate the actual battery voltage
      voltage *= BAT_DIVIDER;
      current = current * vdda / 4095;                                          // calculate current
      current = (((supply5 / 2.0) - current) * 1000 / 113.52) - 409.3;          // 66 mV/A from datasheet * 1.72 correction - 704/1.72 offset
      
      if(log_data) {
        MPU6050_ReadAll(&mpu);                                                  // get MPU data

        memset(log_buffer, 0, sizeof(log_buffer));                              // prepare log entry
        int num = snprintf(log_buffer, 128, "%u,%u,%u,%u.%u,%i,%i,%i,%i,%i,%i,%i,%i,%i\n",\
          HAL_GetTick() - starttime, voltage, current, temp, temp_frac, steer, throttle, (int)(mpu.Ax*1000),\
          (int)(mpu.Ay*1000), (int)(mpu.Az*1000), (int)mpu.Gx, (int)mpu.Gy, (int)mpu.Gz, (int)mpu.temp);

        UINT bytesWrote;
        fres = f_write(&file, log_buffer, num, &bytesWrote);                    // write log entry to the file
        bytes_total += bytesWrote;

        if(fres != FR_OK) {
          PrintError("f_write error (%i)\r\n");
          unmount_sd = true;                                                    // try to unmount
          if(UART_debug) {
            HAL_Delay(1000);                                                    // delay to notice the error
          }
        }
      }

      if(UART_debug) {                                                          // send data to UART
        UART_SendStr("BAT: ");
        UART_SendInt(voltage);
        UART_SendStr("mV, ");
        UART_SendInt(current);
        UART_SendStr("mA; SUPPLY: ");
        UART_SendInt(vdda);
        UART_SendStr("mV, ");
        UART_SendInt(supply5);
        UART_SendStr("mV; ");
        UART_SendStr("BLDC temp: ");
        UART_SendInt(temp);
        UART_SendStr(".");
        UART_SendInt(temp_frac);
        UART_SendStr("C\n");
      }

      adc_data_bat_rdy = adc_data_sply_rdy = false;                             // reset flags
    }


    if(log_data && unmount_sd) {
      f_truncate(&file);                                                        // truncate unused area
      f_close(&file);                                                           // close file
      UART_SendStr("File saved.\n");

      f_mount(NULL, "", 0);                                                     // unmount SD card
      UART_SendStr("SD unmounted succesfully!\n");

      log_data = unmount_sd = false;                                            // reset flags
    }


    if(display_refresh && display_wakeup) {                                     // OLED display refresh interrupt
      ssd1306_Fill(Black);                                                      // clear
      OLED_TempInfo(10, 0, temp, temp_frac, White, Font_7x10);
      OLED_BatInfo(90, 0, voltage, White, Font_7x10, show_batoff);
      OLED_SignalInfo(45, 19, tx_freq, White, Font_6x8);                        // display commands per sec
      ssd1306_UpdateScreen();

      display_refresh = false;                                                  // reset flag
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
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART2|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_I2C2
                              |RCC_PERIPHCLK_TIM1|RCC_PERIPHCLK_TIM8
                              |RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_PCLK1;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_HSI;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  PeriphClkInit.Tim8ClockSelection = RCC_TIM8CLK_HCLK;
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
  hadc1.Init.DMAContinuousRequests = DISABLE;
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
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */
  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 2;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
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
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00000001;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /** I2C Fast mode Plus enable
  */
  __HAL_SYSCFG_FASTMODEPLUS_ENABLE(I2C_FASTMODEPLUS_I2C2);
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7199;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 199;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 71;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 19999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.Pulse = 0;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 14399;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 24999;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_OnePulse_Init(&htim15, TIM_OPMODE_SINGLE) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_TIMING;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_OC_ConfigChannel(&htim15, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim15, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

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
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 7199;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 2499;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

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
  huart2.Init.BaudRate = 9600;
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
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 921600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);
  /* DMA2_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Channel1_IRQn);

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
  HAL_GPIO_WritePin(GPIOB, NRF_CE_Pin|NRF_CSN_Pin|SD_CS_Pin|LED_B_Pin
                          |LED_G_Pin|LED_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED1_Pin */
  GPIO_InitStruct.Pin = LED1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OLED_WKUP_Pin */
  GPIO_InitStruct.Pin = OLED_WKUP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(OLED_WKUP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : NRF_IRQ_Pin */
  GPIO_InitStruct.Pin = NRF_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(NRF_IRQ_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : NRF_CE_Pin NRF_CSN_Pin LED_B_Pin LED_G_Pin
                           LED_R_Pin */
  GPIO_InitStruct.Pin = NRF_CE_Pin|NRF_CSN_Pin|LED_B_Pin|LED_G_Pin
                          |LED_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SW1_Pin */
  GPIO_InitStruct.Pin = SW1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SW1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA11 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : SW2_Pin */
  GPIO_InitStruct.Pin = SW2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(SW2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 7, 0);
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

    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_RESET);              // reset no signal led
    HAL_GPIO_TogglePin(GPIOB, STATUS_LED);
  }
  else if(GPIO_Pin == OLED_WKUP_Pin) {                                          // OLED wakeup button
    UART_SendStr("OLED_WKUP\n");

    HAL_TIM_Base_Stop_IT(&htim15);                                              // reset counter state
    __HAL_TIM_SET_COUNTER(&htim15, 0);                                          // set counter back to 0 in case the timer was active
    // TIM15->CR1 |= TIM_CR1_URS;                                               // enable only timer overflow interrupts (doesnt work, IRQ still triggered)
    HAL_TIM_Base_Start_IT(&htim15);                                             // start timer

    display_wakeup = true;
    ssd1306_SetDisplayOn(1);                                                    // turn display on
  }
  else if(GPIO_Pin == SW1_Pin) {                                                // switch 1 (UART)
    UART_SendStr("SW1\n");

    if(HAL_GPIO_ReadPin(SW1_GPIO_Port, SW1_Pin) == GPIO_PIN_RESET) {
      MX_USART3_UART_Init();
      UART_debug = true;
      UART_SendStr("UART initialized!\n");
    } else {
      UART_SendStr("Deinitializing UART...\n");
      HAL_UART_DeInit(&UART);
      UART_debug = false;
    }
  }
  else if(GPIO_Pin == SW2_Pin) {                                                // switch 2 (SD logging)
    UART_SendStr("SW2\n");
    unmount_sd = true;
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if(htim->Instance == TIM1) {                                                  // data timer interrupt
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_data_bat, ADC_BUFF_SIZE*2);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t*)adc_data_sply, ADC_BUFF_SIZE*2);
  }
  else if(htim->Instance == TIM6) {                                             // 1Hz timer interrupt
    tx_freq = tx_freq_cnt;                                                      // save value
    tx_freq_cnt = 0;                                                            // reset counter

    show_batoff = !show_batoff;                                                 // blink battery-off logo (if active)

    if(voltage < 6800 && STATUS_LED != LED_R_Pin) {                             // change status led color according to voltage level
      HAL_GPIO_WritePin(GPIOA, LED_G_Pin, GPIO_PIN_RESET);                      // turn off green LED
      STATUS_LED = LED_R_Pin;
    }
    else {
      HAL_GPIO_WritePin(GPIOA, LED_R_Pin, GPIO_PIN_RESET);                      // turn off red LED
      STATUS_LED = LED_G_Pin;
    }

    temp_conv_ready = true;                                                     // set temp conv flag
  }
  else if(htim->Instance == TIM15) {                                            // displa ON timer
    ssd1306_SetDisplayOn(0);                                                    // set display off
    display_wakeup = false;
  } 
  else if(htim->Instance == TIM16) {                                            // display refresh interrupt
    display_refresh = true;
  }
  else if(htim->Instance == TIM17) {                                            // safety timer interrupt
    __HAL_TIM_SET_COMPARE(&htim8, SERVO_CHANNEL, 1500);                         // neutral
    if(__HAL_TIM_GET_COMPARE(&htim8, BLDC_CHANNEL))                             // reset BLDC only if its already initialized (>0)
      __HAL_TIM_SET_COMPARE(&htim8, BLDC_CHANNEL, 1500);

    HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);              // reset status led
    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
    
    HAL_GPIO_WritePin(LED_B_GPIO_Port, LED_B_Pin, GPIO_PIN_SET);                // turn on no signal led
    UART_SendStr("NO SIGNAL!\n");
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc) {
  if(hadc->Instance == ADC1) {
    adc_data_bat_rdy = true;
  }
  else if(hadc->Instance == ADC2) {
    adc_data_sply_rdy = true;
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if(huart->Instance == USART2) {
    HAL_GPIO_TogglePin(GPIOC, LED1_Pin);
    OneWire_RxCpltCallback();
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if(huart->Instance == USART2) {
    OneWire_TxCpltCallback();
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

void OW_Complete(void) {
  // UART_SendStr("OneWire transfer completed!\n");
  temp_received = true;
}

void OW_Error(void) {
  PrintError("OneWire error!");
  Error_Handler();
}

void DS_Convert(void) {                                                         // init temperature measurement conversion
  OneWire_Execute(0xcc,0,0x44,0);
  HAL_GPIO_WritePin(GPIOC, LED1_Pin, GPIO_PIN_RESET);
}

void DS_Read(uint8_t* buffer) {                                                 // read converted temperature
  temp_conv_ready = false;
  OneWire_Execute(0xcc,0,0xbe,buffer);
}

#define DS_INT_MASK 0x7F0
#define DS_SIGN_MASK 0x800

uint16_t DS_GetIntTemp(uint16_t temp) {                                         // convert raw temp to int
  uint16_t converted_temp = 0;
  temp &= DS_INT_MASK;
  for(int k = 0; k < 8; ++k) {
    converted_temp += ((temp >> (4 + k)) & 0x01) ? pow(2, k) : 0;
  }
  if(((temp & DS_SIGN_MASK) >> 11) == 0x01)
    converted_temp *= -1;
  return converted_temp;
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

  if(log_data) {
    f_mount(NULL, "", 0);                                                       // try to unmount SD
  }

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
