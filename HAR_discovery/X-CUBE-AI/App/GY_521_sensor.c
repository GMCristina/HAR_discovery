/*
 * GY_521_sensor.c
 *
 *  Created on: Dec 8, 2021
 *      Author: M.Cristina Giannini
 */


#include "GY_521_sensor.h"
#include "main.h"
#include "app_x-cube-ai.h"
#include <stdio.h>

float g = 9.80665;

float LSB_Sensitivity = 8192.0;


int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

float Ax, Ay, Az;

int16_t n_interrupts = -1;
uint8_t idx = 0;

int8_t flag_first = 1;
int8_t count_first_frame = 0;
uint32_t tickstart =0 , tickend = 0, n_tick = 0;

int16_t Queue_Ax_Raw[dim_frame], Queue_Ay_Raw[dim_frame], Queue_Az_Raw[dim_frame];
float Queue_Ax[dim_frame], Queue_Ay[dim_frame], Queue_Az[dim_frame];

// The reset value is 0x00 for all registers other than the registers below.
// Register 107: 0x40 (Power sleep)
// Register 117: 0x68 (who_am_I)

void MPU6050_Init (void)
{
	uint8_t check;
	uint8_t Data;

	//Force Device Reset
	Data = 0x80;
	if (HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1,
			1000) != HAL_OK) {
		printf("Errore");
	}

	//check device ID WHO_AM_I
	if(HAL_I2C_Mem_Read (&hi2c3, MPU6050_ADDR,WHO_AM_I_REG,1, &check, 1, 1000)!=HAL_OK){
		  printf("Errore");
	}

	// power management register 0X6B we should write all 0's to wake the sensor up
	//Data = 0x00;
	Data = 0;
	if (HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1,
			1000) != HAL_OK) {
		printf("Errore");
	}

	// Set DATA RATE of 40 Hz by writing SMPLRT_DIV register
	// Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
	// where Gyroscope Output Rate = 8kHz when the DLPF is disabled (DLPF_CFG = 0 or 7),
	// and 1kHz when the DLPF is enabled (see Register 26).
	// SMPLRT_DIV =199 -> Sample Rate = 40 Hz
	Data = 0xC7;
	if (HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1,
			1000) != HAL_OK) {
		printf("Errore");
	}

	// Set accelerometer configuration in ACCEL_CONFIG Register
	// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=1 -> ± 4g
	Data = 0x08;
	if (HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1,
			1000) != HAL_OK) {
		printf("Errore");
	}

	// Configure Interrupt
	Data = 0x00;
	if (HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, INT_PIN_CFG_REG, 1, &Data, 1,
			1000) != HAL_OK) {
		printf("Errore");
	}

	// Enable Data Ready Interrupt
	Data = 0x01;
	if (HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, INT_ENABLE_REG, 1, &Data, 1,
			1000) != HAL_OK) {
		printf("Errore");
	}



}
void MPU6050_Read_Accel (void)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from ACCEL_XOUT_H register

	if(HAL_I2C_Mem_Read (&hi2c3, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000)!=HAL_OK){
		printf("Errore");
	}

	Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

	/*** convert the RAW values into acceleration in 'g'
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 1. So I am dividing by 8192
	     for more details check ACCEL_CONFIG Register              ****/

	Ax = Accel_X_RAW/ LSB_Sensitivity;
	Ay = Accel_Y_RAW/ LSB_Sensitivity;
	Az = Accel_Z_RAW/ LSB_Sensitivity;

	Ax = Ax * g; // m/s^2
	Ay = Ay * g;
	Az = Az * g;
}

void MPU6050_Read_Accel_Raw (uint8_t i)
{
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from ACCEL_XOUT_H register

	if(HAL_I2C_Mem_Read (&hi2c3, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, 1000)!=HAL_OK){
		printf("Errore");
	}

	Queue_Ax_Raw[i] = (int16_t)(Rec_Data[0] << 8 | Rec_Data [1]);
	Queue_Ay_Raw[i]= (int16_t)(Rec_Data[2] << 8 | Rec_Data [3]);
	Queue_Az_Raw[i]= (int16_t)(Rec_Data[4] << 8 | Rec_Data [5]);

}

void MPU6050_Print_Accel(void){
	printf("Ax: %.2f [m/s^2]\r\n", Ax);
	printf("Ay: %.2f [m/s^2]\r\n", Ay);
	printf("Az: %.2f [m/s^2]\r\n", Az);
}

void MPU6050_Conv_Order_Frame (void){
	for (uint8_t j = 89; j >= 0; --j) {
		Queue_Ax[j] = (Queue_Ax_Raw[idx] / LSB_Sensitivity) * g;
		Queue_Ay[j] = (Queue_Ay_Raw[idx] / LSB_Sensitivity) * g;
		Queue_Az[j] = (Queue_Az_Raw[idx] / LSB_Sensitivity) * g;
		if (idx == 0) {
			idx = 89;
		} else {
			idx = idx - 1;
		}
	}
}

void MPU6050_Print_Frame (void){
	printf("NEW FRAME \r\n");
	for(uint8_t j = 0; j < 90; ++j){
		printf("Ax: %.2f \t Ay: %.2f \t Az: %.2f \t [m/s^2]\r\n", Queue_Ax[j], Queue_Ay[j],Queue_Az[j]);
	}
}

void MPU6050_Print_Frame_Part (void){
	printf("NEW FRAME \r\n");
	printf("Ax: %.2f \t Ay: %.2f \t Az: %.2f \t [m/s^2]\r\n", Queue_Ax[0], Queue_Ay[0],Queue_Az[0]);
	printf("Ax: %.2f \t Ay: %.2f \t Az: %.2f \t [m/s^2]\r\n", Queue_Ax[45], Queue_Ay[45],Queue_Az[45]);
	printf("Ax: %.2f \t Ay: %.2f \t Az: %.2f \t [m/s^2]\r\n", Queue_Ax[89], Queue_Ay[89],Queue_Az[89]);
}