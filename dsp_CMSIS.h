#include "arm_math.h"

#include "uart_if.h"
#include "common.h"

typedef struct
{
  unsigned char* ucpSignal; //8bit (representing 16bit signal)
  uint32_t signalSize; //length of ucpSignal
  uint32_t Fs; //sampling frequency in Hz of signal
  float32_t* fpSignal; //combined back to 16bit, then converted to float.  half the length of ucpSignal
}
tDSPInstance;

typedef unsigned int tboolean;

#ifndef TRUE
#define TRUE                    1
#endif

#ifndef FALSE
#define FALSE                   0
#endif


extern tboolean DSPReadyForSignal();
extern tDSPInstance* DSPCeateInstance(uint32_t signalSize, uint32_t Fs);
extern void DSPDestroyInstance(tDSPInstance* instance);
extern void DSPProcessSignal(tDSPInstance* instance);
extern void DSPConvertUCtoF32_extern(uint8_t* input, float32_t* output, uint32_t size);
extern void DSPConvertF32toUC_extern(float32_t* input, uint8_t* output, uint32_t size);
extern void DSPDecimation(arm_fir_decimate_instance_f32* decimator, float32_t* input, float32_t* output, uint32_t blocksize);

void DSP( void *pvParameters );  //the dsp main thread

void DSPConvertUCtoF32(tDSPInstance* instance);


