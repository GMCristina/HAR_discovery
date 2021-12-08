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

int16_t Accel_X_RAW = 0;
int16_t Accel_Y_RAW = 0;
int16_t Accel_Z_RAW = 0;

float Ax, Ay, Az;


void MPU6050_Init (void)
{
	uint8_t check=3;
	uint8_t Data;

	// check device ID WHO_AM_I

	if(HAL_I2C_Mem_Read (&hi2c3, MPU6050_ADDR,WHO_AM_I_REG,1, &check, 1, 1000)!=HAL_OK){
		  printf("Errore");
	}

	if (check == 104)  // 0x68 will be returned by the sensor if everything goes well
	{
		// power management register 0X6B we should write all 0's to wake the sensor up
		//Data = 0x00;
		Data = 0;
		if(HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, PWR_MGMT_1_REG, 1,&Data, 1, 1000)!=HAL_OK){
			printf("Errore");
		}

		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		Data = 0x07;
		if(HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000)!=HAL_OK){
			printf("Errore");
		}
		Data = 0x00;
		if(HAL_I2C_Mem_Read (&hi2c3, MPU6050_ADDR,SMPLRT_DIV_REG,1, &Data, 1, 1000)!=HAL_OK){
			  printf("Errore");
		}

		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> ± 2g
		Data = 0x00;
		if(HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000)!=HAL_OK){
			printf("Errore");
		}

		// Set Gyroscopic configuration in GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> ± 250 °/s
		Data = 0x00;
		if(HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, 1000)!=HAL_OK){
			printf("Errore");
		}

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
	     I have configured FS_SEL = 0. So I am dividing by 16384.0
	     for more details check ACCEL_CONFIG Register              ****/

	Ax = Accel_X_RAW/16384.0;
	Ay = Accel_Y_RAW/16384.0;
	Az = Accel_Z_RAW/16384.0;

	Ax = Ax * g; // m/s^2
	Ay = Ay * g;
	Az = Az * g;
}

void MPU6050_Print_Accel(void){
	printf("Ax: %.2f [m/s^2]\r\n", Ax);
	printf("Ay: %.2f [m/s^2]\r\n", Ay);
	printf("Az: %.2f [m/s^2]\r\n", Az);
}
