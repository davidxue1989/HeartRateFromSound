#include "dsp_CMSIS.h"
#include "arm_const_structs.h"
#include "rom_map.h"
#include "utils.h"

#include <stdlib.h>

#include "uart_if.h"
#include "common.h"


volatile tboolean gb_DSPprocessing = FALSE;
static tDSPInstance* gDSP_DSPInstance = NULL;

tboolean DSPReadyForSignal() {
  return gb_DSPprocessing==FALSE;
}

tDSPInstance* DSPCeateInstance(uint32_t signalSize, uint32_t Fs) {
  tDSPInstance* tempInstance;
  tempInstance = (tDSPInstance*)malloc(sizeof(tDSPInstance));
  if (tempInstance == NULL){
    UART_PRINT("DSPCreate malloc error!");
    LOOP_FOREVER(); //malloc error. heap too small?
    return NULL;
  }
  tempInstance->signalSize = signalSize;
  tempInstance->Fs = Fs;
  tempInstance->ucpSignal = (unsigned char*)malloc(sizeof(unsigned char)*signalSize);
  tempInstance->fpSignal = (float32_t*)malloc(sizeof(float32_t)*signalSize/2);
  if (tempInstance->ucpSignal==NULL || \
      tempInstance->fpSignal==NULL) {
    UART_PRINT("DSPCreate malloc error!");
    LOOP_FOREVER(); //malloc error. heap too small?
    return NULL;
  }
  return tempInstance;
}

void DSPDestroyInstance(tDSPInstance* instance) {
  if (instance->ucpSignal) {
    free(instance->ucpSignal);
  }
  if (instance->fpSignal) {
    free(instance->fpSignal);
  }
  free(instance);
}

void DSPProcessSignal(tDSPInstance* instance) {
  if (instance==NULL || instance->ucpSignal==NULL || instance->signalSize<=0) {
    return; //instance was not properly created
  }
  if (gb_DSPprocessing) {
    return; //not ready to process more
  }
  gDSP_DSPInstance = instance;
  gb_DSPprocessing = TRUE;
}

//the dsp main thread
void DSP( void* pvParameters ) {
  while (1) {
    if (gb_DSPprocessing) {
      DSPConvertUCtoF32(gDSP_DSPInstance);
      gb_DSPprocessing  = FALSE;
    }
    MAP_UtilsDelay(1000);
  }
}

void DSPConvertF32toUC_extern(float32_t* input, uint8_t* output, uint32_t size) {
  //assume input is length size/2, output is length size
  if (input == NULL || output == NULL)
	return;
  unsigned int i;
  for (i=0; i<size; i+=2) {
	uint16_t combined = (uint16_t) (input[i/2]);
	output[i] = (uint8_t) (combined & 0b0000000011111111);
	output[i+1] = (uint8_t) (combined>>8);
  }
//	UART_PRINT("\n\r\n\r");
//	for (i=0; i<size/2; i++) {
//		UART_PRINT("%f ", input[i]);
//	}
//	UART_PRINT("\n\r\n\r");
//
//	UART_PRINT("\n\r\n\r");
//	for (i=0; i<size; i++) {
//		UART_PRINT("%d ", output[i]);
//	}
//	UART_PRINT("\n\r\n\r");
}

void DSPConvertUCtoF32_extern(uint8_t* input, float32_t* output, uint32_t size) {
  //assume input is length size, output is length size/2
  if (input == NULL || output == NULL)
    return;
  unsigned int i;
  for (i=0; i<size; i+=2) {
	//the signal should be 16 kHz 16bit PCM (http://www.ti.com/tool/TIDC-CC3200AUDBOOST)
	//so need to combine two unsigned char together
	uint16_t combined = (uint16_t) ((input[i+1]<<8) | input[i]);
	output[i/2] = (float32_t) (int16_t)combined;
  }

//	UART_PRINT("\n\r\n\r");
//	for (i=0; i<size; i++) {
//		UART_PRINT("%d ", input[i]);
//	}
//	UART_PRINT("\n\r\n\r");
//
//	UART_PRINT("\n\r\n\r");
//	for (i=0; i<size/2; i++) {
//		UART_PRINT("%f ", output[i]);
//	}
//	UART_PRINT("\n\r\n\r");
}

void DSPConvertUCtoF32(tDSPInstance* instance) {
  unsigned char* ucpSignal = instance->ucpSignal;
  float32_t* fpSignal = instance->fpSignal;
  DSPConvertUCtoF32_extern(ucpSignal, fpSignal, instance->signalSize);
}

void DSPDecimation(arm_fir_decimate_instance_f32* decimator, float32_t* p_inp, float32_t* p_out, uint32_t inputSize) {
	int tap, num_out;
	uint16_t H_size = decimator->numTaps;
	uint8_t factor_M = decimator->M;
	float32_t *p_H = decimator->pCoeffs;
	float32_t *p_Z = decimator->pState;
	uint32_t num_inp = inputSize;
	double sum;
	num_out = 0;
	while (num_inp >= factor_M) {
	    /* shift Z delay line up to make room for next samples */
	    for (tap = H_size - 1; tap >= factor_M; tap--) {
	        p_Z[tap] = p_Z[tap - factor_M];
	    }
	    /* copy next samples from input buffer to bottom of Z delay line */
	    for (tap = factor_M - 1; tap >= 0; tap--) {
	        p_Z[tap] = *p_inp++;
	    }
	    num_inp -= factor_M;
	    /* calculate FIR sum */
	    sum = 0.0;
	    for (tap = 0; tap < H_size; tap++) {
	        sum += p_H[tap] * p_Z[tap];
	    }
	    *p_out++ = sum;     /* store sum and point to next output */
	    num_out++;
	}
	UART_PRINT("num_out: %d\n\r", num_out);
}
