/**
  ******************************************************************************
  * @file           : mpu6050.c
  * @brief          : Library implementation
  *                     
  * @author         : Kristian Slehofer
  * @date           : 24. 4. 2022
  ******************************************************************************
  */
// inspired by https://controllerstech.com/how-to-interface-mpu6050-gy-521-with-stm32/

#include "mpu6050.h"

#ifndef MPU6050_I2C_PORT
#error "You should assign I2C port to MPU6050_I2C_PORT macro (platform.h)!"
#endif

static float acc_div;
static float gyro_div;

uint8_t MPU6050_Init(void) {
  uint8_t check;
  uint8_t Data;

  // check if device is on bus by reading WHO_AM_I register
  HAL_I2C_Mem_Read(&MPU6050_I2C_PORT, MPU6050_ADDR, WHO_AM_I, 1, &check, 1, MPU6050_TIMEOUT);

  if (check == 0x68)  // 0x68 will be returned by the sensor if everything goes well
  {
    // disable sleep mode and set PLL to gyro_x reference
    Data = 0x01;
    HAL_I2C_Mem_Write(&MPU6050_I2C_PORT, MPU6050_ADDR, PWR_MGMT_1, 1, &Data, 1, MPU6050_TIMEOUT);

    // set data rate to 50Hz
    Data = DR_50Hz;
    HAL_I2C_Mem_Write(&MPU6050_I2C_PORT, MPU6050_ADDR, SMPRT_DIV, 1, &Data, 1, MPU6050_TIMEOUT);

    // set accelerometer range
    Data = ACC_2g;
    HAL_I2C_Mem_Write(&MPU6050_I2C_PORT, MPU6050_ADDR, ACCEL_CONFIG, 1, &Data, 1, MPU6050_TIMEOUT);

    switch(Data) {    // set coresponding divider
      case ACC_2g:
        acc_div = ACC_2g_div;
        break;
      case ACC_4g:
        acc_div = ACC_4g_div;
        break;
      case ACC_8g:
        acc_div = ACC_8g_div;
        break;
      case ACC_16g:
        acc_div = ACC_16g_div;
        break;
    }

    // set gyroscope range
    Data = GYRO_250deg;
    HAL_I2C_Mem_Write(&MPU6050_I2C_PORT, MPU6050_ADDR, GYRO_CONFIG, 1, &Data, 1, MPU6050_TIMEOUT);

    switch(Data) {    // set coresponding divider
      case GYRO_250deg:
        gyro_div = GYRO_250deg_div;
        break;
      case GYRO_500deg:
        gyro_div = GYRO_500deg_div;
        break;
      case GYRO_1000deg:
        gyro_div = GYRO_1000deg_div;
        break;
      case GYRO_2000deg:
        gyro_div = GYRO_2000deg_div;
        break;
    }
    return 1;
  }
  // return error
  return 0;
}

void MPU6050_ReadAccel(MPU6050_t* mpu) {
  uint8_t Rec_Data[6];

  // read 6 bytes of data starting from ACCEL_XOUT_H register
  HAL_I2C_Mem_Read(&MPU6050_I2C_PORT, MPU6050_ADDR, ACCEL_XOUT_H, 1, Rec_Data, 6, MPU6050_TIMEOUT);

  mpu->Ax_raw = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
  mpu->Ay_raw = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
  mpu->Az_raw = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

  // convert raw values
  mpu->Ax = mpu->Ax_raw / acc_div;
  mpu->Ay = mpu->Ay_raw / acc_div;
  mpu->Az = mpu->Az_raw / acc_div;

  return;
}

void MPU6050_ReadTemp(MPU6050_t* mpu) {
  uint8_t Rec_Data[2];
  int16_t temp_raw;

  // read 2 bytes of data starting from TEMP_OUT_H register
  HAL_I2C_Mem_Read(&MPU6050_I2C_PORT, MPU6050_ADDR, TEMP_OUT_H, 1, Rec_Data, 2, MPU6050_TIMEOUT);

  temp_raw = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);

  // convert raw value
  mpu->temp = (float)(temp_raw / 340.0 + 36.53);

  return;
}

void MPU6050_ReadGyro(MPU6050_t* mpu) {
  uint8_t Rec_Data[6];

  // read 6 bytes of data starting from GYRO_XOUT_H register
  HAL_I2C_Mem_Read(&MPU6050_I2C_PORT, MPU6050_ADDR, GYRO_XOUT_H, 1, Rec_Data, 6, MPU6050_TIMEOUT);

  mpu->Gx_raw = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
  mpu->Gy_raw = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
  mpu->Gz_raw = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

  // convert raw values
  mpu->Gx = mpu->Gx_raw / gyro_div;
  mpu->Gy = mpu->Gy_raw / gyro_div;
  mpu->Gz = mpu->Gz_raw / gyro_div;

  return;
}

void MPU6050_ReadAll(MPU6050_t* mpu) {
  uint8_t Rec_Data[14];
  int16_t temp_raw;

  // read 14 bytes of data starting from ACCEL_XOUT_H register
  HAL_I2C_Mem_Read(&MPU6050_I2C_PORT, MPU6050_ADDR, ACCEL_XOUT_H, 1, Rec_Data, 14, MPU6050_TIMEOUT);

  // get accelerometer
  mpu->Ax_raw = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
  mpu->Ay_raw = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
  mpu->Az_raw = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

  // convert raw accelerometer values
  mpu->Ax = mpu->Ax_raw / acc_div;
  mpu->Ay = mpu->Ay_raw / acc_div;
  mpu->Az = mpu->Az_raw / acc_div;

  // get temperature
  temp_raw = (int16_t)(Rec_Data[6] << 8 | Rec_Data [7]);

  // convert raw value
  mpu->temp = (float)(temp_raw / 340.0 + 36.53);

  // get gyroscope
  mpu->Gx_raw = (int16_t)(Rec_Data[8] << 8 | Rec_Data [9]);
  mpu->Gy_raw = (int16_t)(Rec_Data[10] << 8 | Rec_Data [11]);
  mpu->Gz_raw = (int16_t)(Rec_Data[12] << 8 | Rec_Data [13]);

  // convert raw values
  mpu->Gx = mpu->Gx_raw / gyro_div;
  mpu->Gy = mpu->Gy_raw / gyro_div;
  mpu->Gz = mpu->Gz_raw / gyro_div;

  return;
}
