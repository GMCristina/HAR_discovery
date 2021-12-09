/*
 * GY_521_sensor.h
 *
 *  Created on: Dec 8, 2021
 *      Author: M.Cristina Giannini
 */

#ifndef APP_GY_521_SENSOR_H_
#define APP_GY_521_SENSOR_H_

#include "main.h"
#include "app_x-cube-ai.h"
#include <stdio.h>

#define MPU6050_ADDR 0xD0


#define SMPLRT_DIV_REG 0x19 //Sample rate divider
#define PWR_MGMT_1_REG 0x6B //Power management 1
#define WHO_AM_I_REG 0x75 //Who am I

#define ACCEL_CONFIG_REG 0x1C //Accelerometer configuration
#define ACCEL_XOUT_H_REG 0x3B //Accelerometer measurements

#define GYRO_CONFIG_REG 0x1B //Gyroscope configuration
#define TEMP_OUT_H_REG 0x41 //Temperature measurement
#define GYRO_XOUT_H_REG 0x43 //Gyroscope measurements


#define INT_PIN_CFG_REG 0x37 //Interrupt configuration
#define INT_ENABLE_REG 0x38 //Interrupt enable
#define INT_STATUS_REG 0x3A //Interrupt status

#define FIFO_EN_REG 0x23 //Choose FIFO data
#define FIFO_R_W_REG 0x74 //Read FIFO data
#define FIFO_COUNT_H_REG 0x72 //FIFO count 1
#define FIFO_COUNT_L_REG 0x73 //FIFO count 2
#define USER_CTRL_REG 0x6A //FIFO enable



extern I2C_HandleTypeDef hi2c3;

extern float g;

extern int16_t Accel_X_RAW;
extern int16_t Accel_Y_RAW;
extern int16_t Accel_Z_RAW;


extern float Ax, Ay, Az;

void MPU6050_Init (void);
void MPU6050_Read_Accel (void);
void MPU6050_Print_Accel(void);


#endif /* APP_GY_521_SENSOR_H_ */
