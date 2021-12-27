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

int16_t n_interrupts = 0;
uint8_t idx = 0;

uint16_t n_campioni = 0;
int8_t flag_half_frame = 1;

int8_t flag_first_frame = 1;
int8_t flag_FIFO_overflow = 0;

int8_t flag_first = 1;
int8_t flag_acquire = 1;
int8_t count_first_frame = 0;
uint32_t tickstart = 0, tickend = 0, n_tick = 0;

int8_t flag_first_net = 1;
uint32_t tickstart_net, tickend_net, n_tick_net;

uint32_t tickstart_frame, tickend_frame, n_tick_frame;

int16_t Queue_Ax_Raw[dim_frame], Queue_Ay_Raw[dim_frame],
		Queue_Az_Raw[dim_frame];
float Queue_Ax[dim_frame], Queue_Ay[dim_frame], Queue_Az[dim_frame];

// The reset value is 0x00 for all registers other than the registers below.
// Register 107: 0x40 (Power sleep)
// Register 117: 0x68 (who_am_I)

void MPU6050_Init(void) {
	Recovery_i2c();

	uint8_t check;
	uint8_t Data;

	//check device ID WHO_AM_I
	if (HAL_I2C_Mem_Read(&hi2c3, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, 1000)
			!= HAL_OK) {
		printf("Errore");
	}

	// power management register 0X6B we should write all 0's to wake the sensor up
	//Data = 0x00;
	Data = 0;
	if (HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1,
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

	// Filter
	// Enable Digital Low Pass Filter with delay 2ms
	// to have Gyro freq 1kHz
	Data = 0x01;
	if (HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, CONFIG_REG, 1, &Data, 1, 1000)
			!= HAL_OK) {
		printf("Errore");
	}

	// Set DATA RATE of 20 Hz by writing SMPLRT_DIV register
	// Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV)
	// where Gyroscope Output Rate = 8kHz when the DLPF is disabled (DLPF_CFG = 0 or 7),
	// and 1kHz when the DLPF is enabled .
	// SMPLRT_DIV =49 -> Sample Rate = 20 Hz
	Data = 0x31;
	if (HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1,
			1000) != HAL_OK) {
		printf("Errore");
	}

	// Enable FIFO
	Data = 0x40;
	if (HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, USER_CTRL_REG, 1, &Data, 1,
			1000) != HAL_OK) {
		printf("Errore");
	}

	// Enable acc data on FIFO
	Data = 0x08;
	if (HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, FIFO_EN_REG, 1, &Data, 1, 1000)
			!= HAL_OK) {
		printf("Errore");
	}

	// Configure Interrupt
	Data = 0x00;
	if (HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, INT_PIN_CFG_REG, 1, &Data, 1,
			1000) != HAL_OK) {
		printf("Errore");
	}

	// Enable Fifo overflow Interrupt
	Data = 0x10;
	if (HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, INT_ENABLE_REG, 1, &Data, 1,
			1000) != HAL_OK) {
		printf("Errore");
	}

}
void MPU6050_Read_Accel(void) {
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from ACCEL_XOUT_H register

	if (HAL_I2C_Mem_Read(&hi2c3, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6,
			1000) != HAL_OK) {
		printf("Errore");
	}

	Accel_X_RAW = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
	Accel_Y_RAW = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
	Accel_Z_RAW = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);

	/*** convert the RAW values into acceleration in 'g'
	 we have to divide according to the Full scale value set in FS_SEL
	 I have configured FS_SEL = 1. So I am dividing by 8192
	 for more details check ACCEL_CONFIG Register              ****/

	Ax = Accel_X_RAW / LSB_Sensitivity;
	Ay = Accel_Y_RAW / LSB_Sensitivity;
	Az = Accel_Z_RAW / LSB_Sensitivity;

	Ax = Ax * g; // m/s^2
	Ay = Ay * g;
	Az = Az * g;
}

void MPU6050_Read_Accel_Raw(uint8_t i) {
	uint8_t Rec_Data[6];

	// Read 6 BYTES of data starting from ACCEL_XOUT_H register

	if (HAL_I2C_Mem_Read(&hi2c3, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6,
			1000) != HAL_OK) {
		printf("Errore");
	}

	Queue_Ax_Raw[i] = (int16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
	Queue_Ay_Raw[i] = (int16_t) (Rec_Data[2] << 8 | Rec_Data[3]);
	Queue_Az_Raw[i] = (int16_t) (Rec_Data[4] << 8 | Rec_Data[5]);

}

void MPU6050_Read_FIFO_45(uint8_t i) {
	uint8_t Rec_Data[270];

	HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(&hi2c3, MPU6050_ADDR, FIFO_R_W_REG,
			1, Rec_Data, 270, 1000);

	if (ret != HAL_OK) {
		printf("Errore i2c read FIFO \r\n");
		switch (ret) {
		case HAL_ERROR:
			printf("Error\r\n");
			break;
		case HAL_BUSY:
			printf("Busy\r\n");
			break;
		case HAL_TIMEOUT:
			printf("Timeout\r\n");
			break;
		}
		return;
	}

	for (uint16_t j = 0; j < 270; j += 6) {

		//Queue_Ax_Raw[i] = (int16_t) (Rec_Data[j] << 8 | Rec_Data[j + 1]);
		//Queue_Ay_Raw[i] = (int16_t) (Rec_Data[j + 2] << 8 | Rec_Data[j + 3]);
		//Queue_Az_Raw[i] = (int16_t) (Rec_Data[j + 4] << 8 | Rec_Data[j + 5]);
		Queue_Ax_Raw[i] = (int16_t) Rec_Data[j] << 8
				| (int16_t) Rec_Data[j + 1];
		Queue_Ay_Raw[i] = (int16_t) Rec_Data[j + 2] << 8
				| (int16_t) Rec_Data[j + 3];
		Queue_Az_Raw[i] = (int16_t) Rec_Data[j + 4] << 8
				| (int16_t) Rec_Data[j + 5];

		i++;
	}

}

void MPU6050_Read_FIFO_n(uint16_t n) {
	uint8_t Rec_Data[n];

	if (HAL_I2C_Mem_Read(&hi2c3, MPU6050_ADDR, FIFO_R_W_REG, 1, Rec_Data, n,
			1000) != HAL_OK) {
		printf("Errore");
	}

	/*for (uint16_t k = 0; k < n; k++) {
	 if (HAL_I2C_Mem_Read(&hi2c3, MPU6050_ADDR, FIFO_R_W_REG, 1,
	 Rec_Data + k, 1, 1000) != HAL_OK) {
	 printf("Errore");
	 }
	 }
	 */
	/*
	 // Reset FIFO
	 uint8_t Data = 0x04;
	 if (HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, USER_CTRL_REG, 1, &Data, 1,
	 1000) != HAL_OK) {
	 printf("Errore");
	 }
	 Data = 0x40;
	 if (HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, USER_CTRL_REG, 1, &Data, 1,
	 1000) != HAL_OK) {
	 printf("Errore");
	 }
	 */
	for (uint16_t j = 0; j < n; j += 6) {

		Accel_X_RAW = (int16_t) Rec_Data[j] << 8 | (int16_t) Rec_Data[j + 1];
		Accel_Y_RAW = (int16_t) Rec_Data[j + 2] << 8
				| (int16_t) Rec_Data[j + 3];
		Accel_Z_RAW = (int16_t) Rec_Data[j + 4] << 8
				| (int16_t) Rec_Data[j + 5];

		Ax = Accel_X_RAW / LSB_Sensitivity;
		Ay = Accel_Y_RAW / LSB_Sensitivity;
		Az = Accel_Z_RAW / LSB_Sensitivity;

		Ax = Ax * g; // m/s^2
		Ay = Ay * g;
		Az = Az * g;

		printf("Ax: %.2f \t Ay: %.2f \t Az: %.2f [m/s^2]\r\n", Ax, Ay, Az);

	}

}

void MPU6050_Print_Accel(void) {
	printf("Ax: %.2f [m/s^2]\r\n", Ax);
	printf("Ay: %.2f [m/s^2]\r\n", Ay);
	printf("Az: %.2f [m/s^2]\r\n", Az);
}

void MPU6050_Conv_Order_Frame(void) {
	for (int8_t j = 0; j < 45; j++) {
		Queue_Ax[j] = Queue_Ax[j + 45];
		Queue_Ay[j] = Queue_Ay[j + 45];
		Queue_Az[j] = Queue_Az[j + 45];
	}

	int8_t i = 0;
	for (int8_t j = 45; j < dim_frame; j++) {
		Queue_Ax[j] = (Queue_Ax_Raw[i] / LSB_Sensitivity) * g;
		Queue_Ay[j] = (Queue_Ay_Raw[i] / LSB_Sensitivity) * g;
		Queue_Az[j] = (Queue_Az_Raw[i] / LSB_Sensitivity) * g;
		i++;
	}
}

void MPU6050_Conv_Frame(void) {
	for (int8_t j = 0; j < dim_frame; j++) {
		Queue_Ax[j] = (Queue_Ax_Raw[j] / LSB_Sensitivity) * g;
		Queue_Ay[j] = (Queue_Ay_Raw[j] / LSB_Sensitivity) * g;
		Queue_Az[j] = (Queue_Az_Raw[j] / LSB_Sensitivity) * g;
	}
}

void MPU6050_Print_Frame(void) {
	printf("NEW FRAME \r\n");
	for (uint8_t j = 0; j < 90; ++j) {
		printf("Ax: %.2f \t Ay: %.2f \t Az: %.2f \t [m/s^2]\r\n", Queue_Ax[j],
				Queue_Ay[j], Queue_Az[j]);
	}
}

void MPU6050_Print_Frame_Part(void) {

	printf("* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *\r\n");
	printf("Sample 0:\tAx= %.2f\tAy= %.2f\tAz= %.2f\t[m/s^2]\r\n", Queue_Ax[0],
			Queue_Ay[0], Queue_Az[0]);
	printf("Sample 44:\tAx= %.2f\tAy= %.2f\tAz= %.2f\t[m/s^2]\r\n",
			Queue_Ax[44], Queue_Ay[44], Queue_Az[44]);
	printf("Sample 45:\tAx= %.2f\tAy= %.2f\tAz= %.2f\t[m/s^2]\r\n",
			Queue_Ax[45], Queue_Ay[45], Queue_Az[45]);
	printf("Sample 89:\tAx= %.2f\tAy= %.2f\tAz= %.2f\t[m/s^2]\r\n",
			Queue_Ax[89], Queue_Ay[89], Queue_Az[89]);
}

void Recovery_i2c(void) {
	//printf("Start recovery \r\n");
	//printf("Deinit \r\n");
	HAL_I2C_DeInit(&hi2c3);

	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	for (int i = 0; i < 10; i++) {
		//printf("Invio clock %d \r\n", i);
		delay_us(10);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
		delay_us(10);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	}
	//printf("Deinit \r\n");
	HAL_I2C_DeInit(&hi2c3);
	//printf("Init \r\n");
	HAL_I2C_Init(&hi2c3);

}

void delay_us(uint16_t us) {
	__HAL_TIM_SET_COUNTER(&htim1, 0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);  // wait for the counter to reach the us input in the parameter
}

