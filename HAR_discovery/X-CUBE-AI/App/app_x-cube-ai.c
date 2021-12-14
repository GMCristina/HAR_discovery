#ifdef __cplusplus
 extern "C" {
#endif
/**
 ******************************************************************************
 * @file           : app_x-cube-ai.c
 * @brief          : AI program body
 ******************************************************************************
 * This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * Copyright (c) 2018 STMicroelectronics International N.V.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/*
 * Description
 *   v1.0 - Minimum template to show how to use the Embedded Client API
 *          model. Only one input and one output is supported. All
 *          memory resources are allocated statically (AI_NETWORK_XX, defines
 *          are used).
 *          Re-target of the printf function is out-of-scope.
 *
 *   For more information, see the embeded documentation:
 *
 *       [1] %X_CUBE_AI_DIR%/Documentation/index.html
 *
 *   X_CUBE_AI_DIR indicates the location where the X-CUBE-AI pack is installed
 *   typical : C:\Users\<user_name>\STM32Cube\Repository\STMicroelectronics\X-CUBE-AI\6.0.0
 */
/* Includes ------------------------------------------------------------------*/
/* System headers */
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <inttypes.h>
#include <string.h>

#include "app_x-cube-ai.h"
#include "main.h"
#include "ai_datatypes_defines.h"

/* USER CODE BEGIN includes */
#include "GY_521_sensor.h"
/* USER CODE END includes */
/* Global AI objects */
static ai_handle har_5 = AI_HANDLE_NULL;
static ai_network_report har_5_info;

/* Global c-array to handle the activations buffer */
AI_ALIGNED(4)
static ai_u8 activations[AI_HAR_5_DATA_ACTIVATIONS_SIZE];

/*  In the case where "--allocate-inputs" option is used, memory buffer can be
 *  used from the activations buffer. This is not mandatory.
 */
#if !defined(AI_HAR_5_INPUTS_IN_ACTIVATIONS)
/* Allocate data payload for input tensor */
AI_ALIGNED(4)
static ai_u8 in_data_s[AI_HAR_5_IN_1_SIZE_BYTES];
#endif

/*  In the case where "--allocate-outputs" option is used, memory buffer can be
 *  used from the activations buffer. This is no mandatory.
 */
#if !defined(AI_HAR_5_OUTPUTS_IN_ACTIVATIONS)
/* Allocate data payload for the output tensor */
AI_ALIGNED(4)
static ai_u8 out_data_s[AI_HAR_5_OUT_1_SIZE_BYTES];
#endif

static void ai_log_err(const ai_error err, const char *fct) {
	/* USER CODE BEGIN log */
	if (fct)
		printf("TEMPLATE - Error (%s) - type=0x%02x code=0x%02x\r\n", fct,
				err.type, err.code);
	else
		printf("TEMPLATE - Error - type=0x%02x code=0x%02x\r\n", err.type,
				err.code);

	do {
	} while (1);
	/* USER CODE END log */
}

static int ai_boostrap(ai_handle w_addr, ai_handle act_addr) {
	ai_error err;

	/* 1 - Create an instance of the model */
	err = ai_har_5_create(&har_5, AI_HAR_5_DATA_CONFIG);
	if (err.type != AI_ERROR_NONE) {
		ai_log_err(err, "ai_har_5_create");
		return -1;
	}

	/* 2 - Initialize the instance */
	const ai_network_params params = AI_NETWORK_PARAMS_INIT(
			AI_HAR_5_DATA_WEIGHTS(w_addr),
			AI_HAR_5_DATA_ACTIVATIONS(act_addr) );

	if (!ai_har_5_init(har_5, &params)) {
		err = ai_har_5_get_error(har_5);
		ai_log_err(err, "ai_har_5_init");
		return -1;
	}

	/* 3 - Retrieve the network info of the created instance */
	if (!ai_har_5_get_info(har_5, &har_5_info)) {
		err = ai_har_5_get_error(har_5);
		ai_log_err(err, "ai_har_5_get_error");
		ai_har_5_destroy(har_5);
		har_5 = AI_HANDLE_NULL;
		return -3;
	}

	return 0;
}

static int ai_run(void *data_in, void *data_out) {
	ai_i32 batch;

	ai_buffer *ai_input = har_5_info.inputs;
	ai_buffer *ai_output = har_5_info.outputs;

	ai_input[0].data = AI_HANDLE_PTR(data_in);
	ai_output[0].data = AI_HANDLE_PTR(data_out);

	batch = ai_har_5_run(har_5, ai_input, ai_output);
	if (batch != 1) {
		ai_log_err(ai_har_5_get_error(har_5), "ai_har_5_run");
		return -1;
	}

	return 0;
}

/* USER CODE BEGIN 2 */
int acquire_and_process_data(void *data) {

	ai_i8 *pointer = (ai_i8*) data;

	//Primo frame
	if (flag_first_frame == 1) {

		flag_first_frame = 0;

		// Reset FIFO
		uint8_t Data = 0x04;
		if (HAL_I2C_Mem_Write(&hi2c3, MPU6050_ADDR, USER_CTRL_REG, 1, &Data, 1,
				1000) != HAL_OK) {
			printf("Errore");
		}

		//while (n_campioni < 45) {
		//	printf("Attesa \r\n");
		//}
		//n_campioni = n_campioni - 45;
		uint8_t Rec_Data[2];
		uint16_t fifo_count=0;
		while(fifo_count <270){
		if (HAL_I2C_Mem_Read(&hi2c3, MPU6050_ADDR, FIFO_COUNT_H_REG, 1,
				Rec_Data, 1, 1000) != HAL_OK) {
			printf("Errore");
		}
		if (HAL_I2C_Mem_Read(&hi2c3, MPU6050_ADDR, FIFO_COUNT_L_REG, 1,
				Rec_Data + 1, 1, 1000) != HAL_OK) {
			printf("Errore");
		}
		fifo_count = (uint16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
		printf("FIFO COUNT: %d \r\n", fifo_count);
		}

		MPU6050_Read_FIFO_45_2(0);
		//while (n_campioni < 45) {
		//}
		//n_campioni = n_campioni - 45;
		fifo_count = 0;
		while(fifo_count <270){
		if (HAL_I2C_Mem_Read(&hi2c3, MPU6050_ADDR, FIFO_COUNT_H_REG, 1,
				Rec_Data, 1, 1000) != HAL_OK) {
			printf("Errore");
		}
		if (HAL_I2C_Mem_Read(&hi2c3, MPU6050_ADDR, FIFO_COUNT_L_REG, 1,
				Rec_Data + 1, 1, 1000) != HAL_OK) {
			printf("Errore");
		}
		fifo_count = (uint16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
		printf("FIFO COUNT: %d \r\n", fifo_count);
		}
		MPU6050_Read_FIFO_45_2(45);

		MPU6050_Conv_Frame();

	} else {

		//while (n_campioni < 45) {
		//}
		//n_campioni = n_campioni - 45;

		uint8_t Rec_Data[2];
		uint16_t fifo_count=0;
		while(fifo_count <270){
		if (HAL_I2C_Mem_Read(&hi2c3, MPU6050_ADDR, FIFO_COUNT_H_REG, 1,
				Rec_Data, 1, 1000) != HAL_OK) {
			printf("Errore");
		}
		if (HAL_I2C_Mem_Read(&hi2c3, MPU6050_ADDR, FIFO_COUNT_L_REG, 1,
				Rec_Data + 1, 1, 1000) != HAL_OK) {
			printf("Errore");
		}
		fifo_count = (uint16_t) (Rec_Data[0] << 8 | Rec_Data[1]);
		printf("FIFO COUNT: %d \r\n", fifo_count);
		}

		MPU6050_Read_FIFO_45_2(0);
		MPU6050_Conv_Order_Frame();
	}

	MPU6050_Print_Frame_Part();
	for (uint8_t j = 0; j < dim_frame; ++j) {
		*(ai_float*) (pointer + j * 12) = Queue_Ax[j];
		*(ai_float*) (pointer + j * 12 + 4) = Queue_Ay[j];
		*(ai_float*) (pointer + j * 12 + 8) = Queue_Az[j];
	}

	return 0;
}

int post_process(void *data) {
	ai_i8 *pointer = (ai_i8*) data;

	float max;
	ai_u8 classe;
	float somma;
	float output[6];

	for (ai_size j = 0; j < 6; ++j) {
		float value = *(ai_float*) (pointer + j * 4);
		output[j] = value;
		somma = somma + value;
		if (value >= max) {
			max = value;
			classe = j + 1;
		}

	}
	//printf("Somma probabilità = %.3f \r\n", somma);

	switch (classe) {
	case 1:
		printf("Classe: Downstairs (%d) \r\n", classe);
		break;
	case 2:
		printf("Classe: Jogging (%d) \r\n", classe);
		break;
	case 3:
		printf("Classe: Sitting (%d) \r\n", classe);
		break;
	case 4:
		printf("Classe: Standing (%d) \r\n", classe);
		break;
	case 5:
		printf("Classe: Upstairs (%d) \r\n", classe);
		break;
	case 6:
		printf("Classe: Walking (%d) \r\n", classe);
		break;
	}
	printf("Probabilità: %.2f %% \r\n", max * 100);

	return 0;
}
/* USER CODE END 2 */

/*************************************************************************
 *
 */
void MX_X_CUBE_AI_Init(void) {
	/* USER CODE BEGIN 5 */
	printf("\r\nTEMPLATE - initialization\r\n");

	ai_boostrap(ai_har_5_data_weights_get(), activations);
	/* USER CODE END 5 */
}

void MX_X_CUBE_AI_Process(void) {
	/* USER CODE BEGIN 6 */
	int res = -1;
	uint8_t *in_data = NULL;
	uint8_t *out_data = NULL;

	printf("TEMPLATE - run - main loop\r\n");

	if (har_5) {

		if ((har_5_info.n_inputs != 1) || (har_5_info.n_outputs != 1)) {
			ai_error err =
					{ AI_ERROR_INVALID_PARAM, AI_ERROR_CODE_OUT_OF_RANGE };
			ai_log_err(err,
					"template code should be updated\r\n to support a model with multiple IO");
			return;
		}

		/* 1 - Set the I/O data buffer */

#if AI_HAR_5_INPUTS_IN_ACTIVATIONS
    in_data = har_5_info.inputs[0].data;
#else
		in_data = in_data_s;
#endif

#if AI_HAR_5_OUTPUTS_IN_ACTIVATIONS
    out_data = har_5_info.outputs[0].data;
#else
		out_data = out_data_s;
#endif

		if ((!in_data) || (!out_data)) {
			printf("TEMPLATE - I/O buffers are invalid\r\n");
			return;
		}

		/* 2 - main loop */

		do {

			/* 1 - acquire and pre-process input data */
			res = acquire_and_process_data(in_data);
			/* 2 - process the data - call inference engine */
			if (res == 0)
				res = ai_run(in_data, out_data);
			/* 3- post-process the predictions */
			if (res == 0)
				res = post_process(out_data);

			flag_acquire = 1;
		} while (res == 0);
	}

	if (res) {
		ai_error err = { AI_ERROR_INVALID_STATE, AI_ERROR_CODE_NETWORK };
		ai_log_err(err, "Process has FAILED");
	}
	/* USER CODE END 6 */
}
#ifdef __cplusplus
}
#endif
