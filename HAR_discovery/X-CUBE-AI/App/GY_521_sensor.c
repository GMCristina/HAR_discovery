/* GY_521_sensor.c
 * C file for sensor's management
 */

//************************** INCLUDE *************************************
#include "GY_521_sensor.h"
#include "main.h"
#include "app_x-cube-ai.h"
#include <stdio.h>

//************************************************************************
//************************** VARIABLES ***********************************

//Gravity acceleration
float g = 9.80665;
//Sensor's sensitivity
float LSB_Sensitivity = 8192.0;

//Flag first frame
int8_t flag_first_frame = 1;
//Flag FIFO overflow interrupt
int8_t flag_FIFO_overflow = 0;

//Samples raw frame
int16_t Queue_Ax_Raw[dim_frame], Queue_Ay_Raw[dim_frame],
		Queue_Az_Raw[dim_frame];
//Sample frame
float Queue_Ax[dim_frame], Queue_Ay[dim_frame], Queue_Az[dim_frame];

//************************************************************************
//************************** FUNCTIONS ***********************************

//Sensor's initialization
void MPU6050_Init(void) {

	uint8_t Data;
	HAL_StatusTypeDef ret;

	//I2C Recovery
	Recovery_i2c();

	//Power Management 1 register
	//sensor's normal operation
	Data = 0;
	ret = HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 1000);

	//Check for I2C error
	while (ret != HAL_OK) {
		if(ret == HAL_BUSY){
			//I2C recovery
			Recovery_i2c();
		}
		ret = HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, 1000);
	}

	//Accelerometer Configuration register
	//set accelerometer's range to ± 4g
	Data = 0x08;
	ret = HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);

	//Check for I2C error
	while (ret != HAL_OK) {
		if(ret == HAL_BUSY){
			//I2C recovery
			Recovery_i2c();
		}
		ret = HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, 1000);
	}

	//Configuration register
	//enable Digital Low Pass Filter(DLPF) with highest bandwidth to have gyroscope frequency at 1kHz
	Data = 0x01;
	ret = HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, CONFIG_REG, 1, &Data, 1, 1000);

	//Check for I2C error
	while (ret != HAL_OK) {
		if(ret == HAL_BUSY){
			//I2C recovery
			Recovery_i2c();
		}
		ret = HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, CONFIG_REG, 1, &Data, 1, 1000);
	}

	//Sample Rate Divider register
	//set sample rate to 20 Hz
	Data = 0x31;
	ret = HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);

	//Check for I2C error
	while (ret != HAL_OK) {
		if(ret == HAL_BUSY){
			//I2C recovery
			Recovery_i2c();
		}
		ret = HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, 1000);
	}

	//User Control register
	//enable FIFO buffer
	Data = 0x40;
	ret = HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, USER_CTRL_REG, 1, &Data, 1, 1000);

	//Check for I2C error
	while (ret != HAL_OK) {
		if(ret == HAL_BUSY){
			//I2C recovery
			Recovery_i2c();
		}
		ret = HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, USER_CTRL_REG, 1, &Data, 1, 1000);
	}

	//FIFO Enable register
	//select accelerometer's data for FIFO buffer
	Data = 0x08;
	ret = HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, FIFO_EN_REG, 1, &Data, 1, 1000);

	//Check for I2C error
	while (ret != HAL_OK) {
		if(ret == HAL_BUSY){
			//I2C recovery
			Recovery_i2c();
		}
		ret = HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, FIFO_EN_REG, 1, &Data, 1, 1000);
	}

	//INT Pin / Bypass Enable Configuration register
	//configure interrupt signal (default)
	Data = 0x00;
	ret = HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, INT_PIN_CFG_REG, 1, &Data, 1, 1000);

	//Check for I2C error
	while (ret != HAL_OK) {
			if(ret == HAL_BUSY){
				//I2C recovery
				Recovery_i2c();
			}
			ret = HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, INT_PIN_CFG_REG, 1, &Data, 1, 1000);
	}

	//Interrupt Enable register
	//enable FIFO overflow interrupt
	Data = 0x10;
	ret = HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, INT_ENABLE_REG, 1, &Data, 1, 1000);

	//Check for I2C error
	while (ret != HAL_OK) {
			if(ret == HAL_BUSY){
				//I2C recovery
				Recovery_i2c();
			}
			ret = HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, INT_ENABLE_REG, 1, &Data, 1, 1000);
	}

}

//Read 45 x 3 acquisitions (X,Y,Z) from FIFO buffer (135 x 2 byte)
//input i : index of Queue_A*_Raw (* = X,Y,Z) where FIFO buffer's samples are stored
void MPU6050_Read_FIFO_45(uint8_t i) {
	uint8_t Rec_Data[270];

	//Read 45 x 3 acquisitions from FIFO buffer
	HAL_StatusTypeDef ret = HAL_I2C_Mem_Read(&hi2c3, MPU6050_ADDR, FIFO_R_W_REG, 1, Rec_Data, 270, 1000);

	//Check for I2C error
	while (ret != HAL_OK) {
		if(ret == HAL_BUSY){
			//I2C recovery
			Recovery_i2c();
		}
		ret = HAL_I2C_Mem_Read(&hi2c3, MPU6050_ADDR, FIFO_R_W_REG, 1, Rec_Data, 270, 1000);
	}

	//Store the 45x3 acquisitions in Queue_A*_Raw (* = X,Y,Z)
	for (uint16_t j = 0; j < 270; j += 6) {
		Queue_Ax_Raw[i] = (int16_t) Rec_Data[j] << 8 | (int16_t) Rec_Data[j + 1];
		Queue_Ay_Raw[i] = (int16_t) Rec_Data[j + 2] << 8 | (int16_t) Rec_Data[j + 3];
		Queue_Az_Raw[i] = (int16_t) Rec_Data[j + 4] << 8 | (int16_t) Rec_Data[j + 5];
		i++;
	}

}

//50% overlapping management (translation of the frame's second half in the first half)
//and convert raw data into accelerometric measurements
void MPU6050_Conv_Order_Frame(void) {

	//Translation of the frame's second half in the first half in Queue_A* (* = X,Y,Z)
	for (int8_t j = 0; j < 45; j++) {
		Queue_Ax[j] = Queue_Ax[j + 45];
		Queue_Ay[j] = Queue_Ay[j + 45];
		Queue_Az[j] = Queue_Az[j + 45];
	}

	//Convert raw data in Queue_A*_Raw (* = X,Y,Z) into accelerometric measurements Queue_A* (* = X,Y,Z)
	int8_t i = 0;
	for (int8_t j = 45; j < dim_frame; j++) {
		Queue_Ax[j] = (Queue_Ax_Raw[i] / LSB_Sensitivity) * g;
		Queue_Ay[j] = (Queue_Ay_Raw[i] / LSB_Sensitivity) * g;
		Queue_Az[j] = (Queue_Az_Raw[i] / LSB_Sensitivity) * g;
		i++;
	}
}

//Convert raw data into accelerometric measurements
void MPU6050_Conv_Frame(void) {

	//Convert raw data in Queue_A*_Raw (* = X,Y,Z) into accelerometric measurements Queue_A* (* = X,Y,Z)
	for (int8_t j = 0; j < dim_frame; j++) {
		Queue_Ax[j] = (Queue_Ax_Raw[j] / LSB_Sensitivity) * g;
		Queue_Ay[j] = (Queue_Ay_Raw[j] / LSB_Sensitivity) * g;
		Queue_Az[j] = (Queue_Az_Raw[j] / LSB_Sensitivity) * g;
	}
}

//NN's inputs print
void MPU6050_Print_Frame_Part(void) {

	//Print of some NN input frame's sample (index 0,44,45,89)
	//to check 50% overlapping
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

//I2C recovery from HAL_BUSY error
void Recovery_i2c(void) {

	//I2C de-initialization
	HAL_I2C_DeInit(&hi2c3);

	//Change the function of SCL pin (PA8) into GPIO pin
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);

	GPIO_InitStruct.Pin = GPIO_PIN_8;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	//Send 10 clock pulses on SCL pin at 100kHz (I2C Stardard Mode)
	for (int i = 0; i < 10; i++) {
		delay_us(10);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
		delay_us(10);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	}

	//I2C de-initialization
	HAL_I2C_DeInit(&hi2c3);

	//I2C re-initialization
	HAL_I2C_Init(&hi2c3);
}

//Delay in microseconds for I2C recovery
void delay_us(uint16_t us) {

	//Reset timer counter
	__HAL_TIM_SET_COUNTER(&htim1, 0);

	//Wait for the timer counter to reach the input microseconds
	while (__HAL_TIM_GET_COUNTER(&htim1) < us);
}

//Reset and reable FIFO buffer
void Reset_Reable_FIFO(){
	HAL_StatusTypeDef ret;

	//Reset FIFO
	uint8_t Data = 0x04;
	ret = HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, USER_CTRL_REG, 1, &Data, 1, 1000);

	//Check for I2C error
	while (ret != HAL_OK) {
		if (ret == HAL_BUSY) {
			//I2C recovery
			Recovery_i2c();
		}
		ret = HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, USER_CTRL_REG, 1, &Data, 1, 1000);
	}

	//Reable FIFO
	Data = 0x40;
	ret = HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, USER_CTRL_REG, 1, &Data, 1, 1000);

	//Check for I2C error
	while (ret != HAL_OK) {
		if (ret == HAL_BUSY){
			//I2C recovery
			Recovery_i2c();
		}
		ret = HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, USER_CTRL_REG, 1, &Data, 1, 1000);
	}
}

//Read FIFO Count register
//Output : FIFO Count value
uint16_t Read_FIFO_Count(){

	HAL_StatusTypeDef ret;
	uint16_t fifo_count = 0;
	uint8_t Rec_Data[2];

	//Read FIFO_COUNT_H
	ret = HAL_I2C_Mem_Read(&hi2c3, MPU6050_ADDR, FIFO_COUNT_H_REG, 1, Rec_Data, 1, 1000);

	//Check for I2C error
	while (ret != HAL_OK) {
		if(ret == HAL_BUSY){
			//I2C recovery
			Recovery_i2c();
		}
		ret = HAL_I2C_Mem_Read(&hi2c3, MPU6050_ADDR, FIFO_COUNT_H_REG, 1, Rec_Data, 1, 1000);
	}

	//Read FIFO_COUNT_L
	ret = HAL_I2C_Mem_Read(&hi2c3, MPU6050_ADDR, FIFO_COUNT_L_REG, 1, Rec_Data + 1, 1, 1000);

	//Check for I2C error
	while (ret != HAL_OK) {
		if(ret == HAL_BUSY){
			//I2C recovery
			Recovery_i2c();
		}
		ret = HAL_I2C_Mem_Read(&hi2c3, MPU6050_ADDR, FIFO_COUNT_H_REG, 1, Rec_Data + 1, 1, 1000);
	}

	//Get FIFO Count value
	fifo_count = (uint16_t) (Rec_Data[0] << 8 | Rec_Data[1]);

	return fifo_count;
}

//************************************************************************
