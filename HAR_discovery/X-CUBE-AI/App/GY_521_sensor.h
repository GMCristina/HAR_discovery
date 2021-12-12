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


//Sensor I2C address
#define MPU6050_ADDR 0xD0

//Sensor registers
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

#define dim_frame 90

extern I2C_HandleTypeDef hi2c3;

extern float g;
extern float LSB_Sensitivity;


extern int16_t Accel_X_RAW;
extern int16_t Accel_Y_RAW;
extern int16_t Accel_Z_RAW;


extern float Ax, Ay, Az;

extern int16_t Queue_Ax_Raw[dim_frame], Queue_Ay_Raw[dim_frame], Queue_Az_Raw[dim_frame];
extern float Queue_Ax[dim_frame], Queue_Ay[dim_frame], Queue_Az[dim_frame];

extern int16_t n_interrupts;
extern uint8_t idx;
extern uint8_t n_giri;
extern uint16_t n_campioni;

extern int8_t flag_first;
extern int8_t flag_acquire;
extern int8_t count_first_frame;
extern uint32_t tickstart, tickend, n_tick;
extern int8_t flag_first_net;
extern uint32_t tickstart_net, tickend_net, n_tick_net;
extern int8_t flag_first_frame;
extern uint32_t tickstart_frame, tickend_frame, n_tick_frame;

void MPU6050_Init (void);
void MPU6050_Read_Accel (void);
void MPU6050_Print_Accel(void);

void MPU6050_Read_Accel_Raw (uint8_t);
void MPU6050_Conv_Order_Frame (void);
void MPU6050_Print_Frame (void);
void MPU6050_Print_Frame_Part (void);


#endif /* APP_GY_521_SENSOR_H_ */
