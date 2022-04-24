/**
  ******************************************************************************
  * @file           : mpu6050.h
  * @brief          : Library for interfacing MPU6050 accelerometer and
  *                   gyroscope
  *                     
  * @author         : Kristian Slehofer
  * @date           : 24. 4. 2022
  ******************************************************************************
  */

#ifndef __MPU6050_H
#define __MPU6050_H

#include "stm32f3xx_hal.h"
#include "platform.h"

// module I2C address
#define MPU6050_ADDR      0xD0

// registers
#define SMPRT_DIV         0x19
#define GYRO_CONFIG       0x1B
#define ACCEL_CONFIG      0x1C
#define ACCEL_XOUT_H      0x3B
#define TEMP_OUT_H        0x41
#define GYRO_XOUT_H       0x43
#define PWR_MGMT_1        0x6B
#define WHO_AM_I          0x75

// sample data rate values
#define DR_1kHz           7
#define DR_500Hz          15
#define DR_250Hz          31
#define DR_100Hz          79
#define DR_50Hz           159

// accelerometer range (g)
#define ACC_2g            0
#define ACC_4g            1
#define ACC_8g            2
#define ACC_16g           3

// accelerometer sensitivity
#define ACC_2g_div        16384.0
#define ACC_4g_div        8192.0
#define ACC_8g_div        4096.0
#define ACC_16g_div       2048.0

// gyroscope range (deg/s)
#define GYRO_250deg       0
#define GYRO_500deg       1
#define GYRO_1000deg      2
#define GYRO_2000deg      3

// gyroscope sensitivity
#define GYRO_250deg_div   131.0
#define GYRO_500deg_div   65.5
#define GYRO_1000deg_div  32.8
#define GYRO_2000deg_div  16.4

// struct holding the values
typedef struct {
  int16_t Ax_raw;
  int16_t Ay_raw;
  int16_t Az_raw;

  int16_t Gx_raw;
  int16_t Gy_raw;
  int16_t Gz_raw;

  float Ax, Ay, Az, Gx, Gy, Gz;
  float temp;
} MPU6050_t;


uint8_t MPU6050_Init(void);
void MPU6050_ReadAccel(MPU6050_t* mpu);
void MPU6050_ReadTemp(MPU6050_t* mpu);
void MPU6050_ReadGyro(MPU6050_t* mpu);
void MPU6050_ReadAll(MPU6050_t* mpu);


#endif // __MPU6050_H
