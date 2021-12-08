
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

static void ai_log_err(const ai_error err, const char *fct)
{
  /* USER CODE BEGIN log */
  if (fct)
    printf("TEMPLATE - Error (%s) - type=0x%02x code=0x%02x\r\n", fct,
        err.type, err.code);
  else
    printf("TEMPLATE - Error - type=0x%02x code=0x%02x\r\n", err.type, err.code);

  do {} while (1);
  /* USER CODE END log */
}

static int ai_boostrap(ai_handle w_addr, ai_handle act_addr)
{
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

static int ai_run(void *data_in, void *data_out)
{
  ai_i32 batch;

  ai_buffer *ai_input = har_5_info.inputs;
  ai_buffer *ai_output = har_5_info.outputs;

  ai_input[0].data = AI_HANDLE_PTR(data_in);
  ai_output[0].data = AI_HANDLE_PTR(data_out);

  batch = ai_har_5_run(har_5, ai_input, ai_output);
  if (batch != 1) {
    ai_log_err(ai_har_5_get_error(har_5),
        "ai_har_5_run");
    return -1;
  }

  return 0;
}

/* USER CODE BEGIN 2 */
int acquire_and_process_data(void * data)
{
	MPU6050_Read_Accel();
	MPU6050_Print_Accel();

	ai_i8 *pointer = (ai_i8 *)data;

	float serie[270] = {-5.284695148468017578e+00,3.132680058479309082e-01,1.416516184806823730e+00,1.111420345306396484e+01,9.724927902221679688e+00,1.334794044494628906e+00,-1.416516184806823730e+00,1.225831317901611328e+01,5.134871006011962891e+00,8.771504402160644531e+00,4.058863639831542969e+00,2.315459102392196655e-01,-1.334794044494628906e+00,1.494152164459228516e+01,-3.827317714691162109e+00,8.430994987487792969e+00,1.026974201202392578e+01,5.134871006011962891e+00,-8.444615602493286133e-01,-6.020193576812744141e+00,-1.164539718627929688e+01,6.537767052650451660e-01,5.747786521911621094e+00,-2.873893260955810547e+00,9.234595298767089844e+00,1.547271537780761719e+01,6.660349845886230469e+00,3.786456584930419922e+00,5.407278060913085938e+00,1.457377195358276367e+00,5.311935544013977051e-01,1.134574985504150391e+01,-8.444615602493286133e-01,7.777218341827392578e+00,1.505048465728759766e+01,6.823794364929199219e+00,-1.498238295316696167e-01,-7.777218341827392578e+00,-1.443756866455078125e+01,-6.210878372192382812e+00,1.650786209106445312e+01,-8.117727279663085938e+00,8.662541389465332031e+00,1.953157806396484375e+01,5.053149223327636719e+00,8.172208815813064575e-02,5.243834018707275391e+00,3.568531036376953125e+00,-8.444615602493286133e-01,4.862463951110839844e+00,-7.218784093856811523e-01,-3.405086994171142578e-01,-5.325555801391601562e+00,-1.010629844665527344e+01,-3.486809015274047852e+00,8.281171798706054688e+00,-2.111153841018676758e+00,9.575104713439941406e+00,1.566339969635009766e+01,7.695496559143066406e+00,3.568531036376953125e+00,4.058863639831542969e+00,1.184970259666442871e+00,-6.946377158164978027e-01,2.873893260955810547e+00,3.173541069030761719e+00,-1.037870502471923828e+01,1.957243919372558594e+01,2.451662540435791016e+00,2.451662540435791016e+00,1.906848698854446411e-01,4.603677749633789062e+00,-5.666064739227294922e+00,6.823794364929199219e+00,8.771504402160644531e+00,4.821603298187255859e+00,6.537767052650451660e-01,-2.982856035232543945e+00,-5.094009876251220703e+00,8.539958000183105469e+00,4.821603298187255859e+00,8.430994987487792969e+00,2.560625314712524414e+00,-1.076007485389709473e+00,-8.662541389465332031e+00,1.953157806396484375e+01,-0.000000000000000000e+00,3.445947885513305664e+00,3.105439186096191406e+00,5.325555801391601562e+00,-2.260977745056152344e+00,-1.566339969635009766e+00,-4.331270694732666016e+00,4.208687305450439453e+00,-5.175732135772705078e+00,-3.636632919311523438e+00,7.273265838623046875e+00,1.808782196044921875e+01,5.747786521911621094e+00,3.813697397708892822e-01,4.562816619873046875e+00,3.064578294754028320e+00,-1.144109249114990234e+00,8.812364578247070312e+00,-1.498238295316696167e-01,6.442424297332763672e+00,1.536375236511230469e+01,6.851035118103027344e+00,1.498238295316696167e-01,-9.084772109985351562e+00,-1.283036708831787109e+01,-6.782933235168457031e+00,1.505048465728759766e+01,-5.666064739227294922e+00,6.782933235168457031e+00,1.953157806396484375e+01,3.949900865554809570e+00,-6.782933235168457031e+00,9.425280570983886719e+00,2.982856035232543945e+00,6.278980255126953125e+00,3.486809015274047852e+00,-8.172208815813064575e-02,-2.138394594192504883e+00,3.786456584930419922e+00,-4.630918204784393311e-01,-2.533384561538696289e+00,1.957243919372558594e+01,6.742072105407714844e+00,5.720546245574951172e-01,-6.742072105407714844e+00,-4.521955490112304688e+00,-7.654635429382324219e+00,1.931365394592285156e+01,4.944186210632324219e+00,5.747786521911621094e+00,1.880970001220703125e+01,3.064578294754028320e+00,-1.003819656372070312e+01,8.662541389465332031e+00,1.566339969635009766e+00,3.132680058479309082e-01,2.369940519332885742e+00,2.451662540435791016e+00,-7.627394676208496094e+00,1.957243919372558594e+01,1.037870502471923828e+01,-1.566339969635009766e+00,-2.683208465576171875e+00,5.311935544013977051e-01,-3.146300315856933594e+00,1.386551380157470703e+01,1.559529781341552734e+01,4.903325080871582031e+00,8.240310668945312500e+00,-1.525478959083557129e+00,-1.107334232330322266e+01,8.730643272399902344e+00,2.833032369613647461e+00,-2.070292949676513672e+00,2.642347574234008789e+00,9.942854046821594238e-01,-5.094009876251220703e+00,1.957243919372558594e+01,1.065111160278320312e+01,9.942854046821594238e-01,-2.533384561538696289e+00,4.086104407906532288e-02,-9.534243345260620117e-01,8.580819129943847656e+00,1.122316646575927734e+01,1.035146474838256836e+00,-1.116868495941162109e+00,-2.410801649093627930e+00,-3.146300315856933594e+00,9.003049850463867188e+00,3.405086994171142578e+00,7.205163955688476562e+00,1.307553410530090332e+00,-1.416516184806823730e+00,3.405086994171142578e-01,2.369940519332885742e+00,-8.172208815813064575e-02,-3.718354940414428711e+00,1.957243919372558594e+01,7.967903614044189453e+00,-3.813697397708892822e-01,-5.557101726531982422e+00,-3.568531036376953125e+00,-5.475379943847656250e+00,1.620821380615234375e+01,7.014479160308837891e+00,2.111153841018676758e+00,1.451929092407226562e+01,1.089627817273139954e-01,-8.962188720703125000e+00,6.851035118103027344e+00,2.792171239852905273e+00,-1.035146474838256836e+00,2.070292949676513672e+00,2.601486444473266602e+00,-1.103248119354248047e+01,1.957243919372558594e+01,8.812364578247070312e+00,1.035146474838256836e+00,-4.630918204784393311e-01,2.220116615295410156e+00,-6.020193576812744141e+00,9.275456428527832031e+00,4.521955490112304688e+00,-1.307553410530090332e+00,3.214401960372924805e+00,2.764930486679077148e+00,1.225831270217895508e+00,3.759216070175170898e+00,-1.307553410530090332e+00,9.261836409568786621e-01,1.953157806396484375e+01,7.246025085449218750e+00,1.416516184806823730e+00,-6.891895771026611328e+00,-8.158588409423828125e+00,-4.181446552276611328e+00,1.800609970092773438e+01,4.712640285491943359e+00,2.070292949676513672e+00,1.466911411285400391e+01,6.129156351089477539e-01,-8.158588409423828125e+00,9.806650161743164062e+00,3.786456584930419922e+00,1.089627817273139954e-01,1.144109249114990234e+00,3.173541069030761719e+00,3.445947885513305664e+00,1.026974201202392578e+01,-1.688923120498657227e+00,5.979332447052001953e+00,1.865987586975097656e+01,5.897610664367675781e+00,3.813697397708892822e-01,-8.076866149902343750e+00,-8.076866149902343750e+00,-4.372131824493408203e+00,1.957243919372558594e+01,-5.720546245574951172e-01,6.319841384887695312e+00,1.923193168640136719e+01,3.786456584930419922e+00,-2.533384561538696289e+00,6.088295459747314453e+00,3.023717164993286133e+00,5.039528608322143555e-01,2.601486444473266602e+00,-1.334794044494628906e+00,-1.648062109947204590e+00,1.957243919372558594e+01,7.463950634002685547e+00,-4.249548435211181641e+00,1.065111160278320312e+01,1.513220596313476562e+01,5.557101726531982422e+00,2.152014970779418945e+00,-1.225831270217895508e+00,-1.033784389495849609e+01,9.425280570983886719e+00,4.372131824493408203e+00,5.829508781433105469e+00,1.688923120498657227e+00,1.375655174255371094e+00
	};
	for (ai_size j = 0; j < 270; ++j) {
		*(ai_float *)(pointer + j * 4) = serie[j];
	}
	printf("Input acquired: %f, %f ...\r\n", serie[0], serie[1]);

    return 0;
}

int post_process(void * data)
{
	ai_i8 *pointer = (ai_i8 *)data;

		float max;
		ai_u8 classe;
		float somma;
		float output[6];

		for (ai_size j = 0; j < 6; ++j) {
			float value = *(ai_float *)(pointer + j * 4);
			output[j]=value;
			somma = somma + value;
			if(value>=max){
				max=value;
				classe = j+1;
			}

		}
		printf("Somma probabilità = %.3f \r\n", somma);
		switch (classe){
		case 1: printf("Classe: Downstairs (%d) \r\n", classe); break;
		case 2:printf("Classe: Jogging (%d) \r\n", classe); break;
		case 3:printf("Classe: Sitting (%d) \r\n", classe); break;
		case 4:printf("Classe: Standing (%d) \r\n", classe); break;
		case 5:printf("Classe: Upstairs (%d) \r\n", classe); break;
		case 6:printf("Classe: Walking (%d) \r\n", classe); break;
		}
	  printf("Probabilità: %.2f %% \r\n", max*100);
  return 0;
}
/* USER CODE END 2 */

/*************************************************************************
  *
  */
void MX_X_CUBE_AI_Init(void)
{
    /* USER CODE BEGIN 5 */
  printf("\r\nTEMPLATE - initialization\r\n");

  ai_boostrap(ai_har_5_data_weights_get(), activations);
    /* USER CODE END 5 */
}

void MX_X_CUBE_AI_Process(void)
{
    /* USER CODE BEGIN 6 */
  int res = -1;
  uint8_t *in_data = NULL;
  uint8_t *out_data = NULL;

  printf("TEMPLATE - run - main loop\r\n");

  if (har_5) {

    if ((har_5_info.n_inputs != 1) || (har_5_info.n_outputs != 1)) {
      ai_error err = {AI_ERROR_INVALID_PARAM, AI_ERROR_CODE_OUT_OF_RANGE};
      ai_log_err(err, "template code should be updated\r\n to support a model with multiple IO");
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
    } while (res==0);
  }

  if (res) {
    ai_error err = {AI_ERROR_INVALID_STATE, AI_ERROR_CODE_NETWORK};
    ai_log_err(err, "Process has FAILED");
  }
    /* USER CODE END 6 */
}
#ifdef __cplusplus
}
#endif
