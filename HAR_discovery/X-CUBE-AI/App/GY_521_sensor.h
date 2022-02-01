/*GY_521_sensor.h
 * Header file for sensor's management
*/

#ifndef APP_GY_521_SENSOR_H_
#define APP_GY_521_SENSOR_H_

//************************** INCLUDE *************************************

#include "main.h"
#include "app_x-cube-ai.h"
#include <stdio.h>

//************************************************************************
//************************** DEFINE **************************************

//Sensor's I2C address
#define MPU6050_ADDR 0xD0

//Sensor's registers
#define SMPLRT_DIV_REG 0x19 //Sample Rate Divider
#define CONFIG_REG 0x1A //Configuration
#define PWR_MGMT_1_REG 0x6B //Power Management 1
#define ACCEL_CONFIG_REG 0x1C //Accelerometer Configuration
#define ACCEL_XOUT_H_REG 0x3B //Accelerometer Measurements
#define INT_PIN_CFG_REG 0x37 //INT Pin / Bypass Enable Configuration
#define INT_ENABLE_REG 0x38 //Interrupt Enable
#define FIFO_EN_REG 0x23 //FIFO Enable
#define FIFO_R_W_REG 0x74 //FIFO Read Write
#define FIFO_COUNT_H_REG 0x72 //FIFO Count H
#define FIFO_COUNT_L_REG 0x73 //FIFO Count L
#define USER_CTRL_REG 0x6A //User Control

//Frame's dimension
#define dim_frame 90

//************************************************************************
//************************** EXTERNAL VARIABLES **************************

//Main
extern I2C_HandleTypeDef hi2c3;
extern TIM_HandleTypeDef htim1;

//Gravity acceleration
extern float g;
//Sensor's sensitivity
extern float LSB_Sensitivity;

//Samples raw frame
extern int16_t Queue_Ax_Raw[dim_frame], Queue_Ay_Raw[dim_frame], Queue_Az_Raw[dim_frame];
//Sample frame
extern float Queue_Ax[dim_frame], Queue_Ay[dim_frame], Queue_Az[dim_frame];

//Flag first frame
extern int8_t flag_first_frame;
//Flag FIFO overflow interrupt
extern int8_t flag_FIFO_overflow;

//************************************************************************
//************************** FUNCTIONS ***********************************

//Sensor's initialization
void MPU6050_Init (void);

//50% overlapping management (translation of the frame's second half in the first half)
//and convert raw data into accelerometric measurements
void MPU6050_Conv_Order_Frame (void);

//Convert raw data into accelerometric measurements
void MPU6050_Conv_Frame (void);

//NN's inputs print
void MPU6050_Print_Frame_Part (void);

//Read 45 x 3 acquisitions (X,Y,Z) from FIFO buffer
void MPU6050_Read_FIFO_45(uint8_t);

//I2C recovery
void Recovery_i2c(void);

//Delay in microseconds for I2C recovery
void delay_us(uint16_t);

//Reset and reable FIFO buffer
void Reset_Reable_FIFO(void);

//Read FIFO Count register
uint16_t Read_FIFO_Count(void);

//************************************************************************

#endif /* APP_GY_521_SENSOR_H_ */
