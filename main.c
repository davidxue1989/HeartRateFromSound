
#include <stdlib.h>
#include <string.h>
#include <stdio.h>

// free-rtos/ ti_rtos includes
#include "osi.h"

// Hardware & DriverLib library includes.
#include "hw_types.h"
#include "hw_ints.h"
#include "hw_memmap.h"
#include "hw_common_reg.h"
#include "interrupt.h"
#include "i2s.h"
#include "udma.h"
#include "prcm.h"
#include "rom.h"
#include "rom_map.h"
#include "pin.h"
#include "utils.h"

//Common interface includes
#include "common.h"
#include "udma_if.h"
#include "uart_if.h"
#include "i2c_if.h"


//App include
#include "pinmux.h"
#include "circ_buff.h"
#include "audioCodec.h"
#include "i2s_if.h"
#include "pcm_handler.h"
#include "dsp_CMSIS.h"
#include "FIRFilterCoefficients.h"


#define OSI_STACK_SIZE          0X900

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************
tCircularBuffer *pRecordBuffer;
tCircularBuffer *pPlayBuffer;
OsiTaskHandle g_MicTask = NULL ;

tDSPInstance* g_pDSPInstance;
OsiTaskHandle g_DSPTask = NULL ;

arm_fir_decimate_instance_f32* g_Decimator;

#if defined(ccs)
extern void (* const g_pfnVectors[])(void);
#endif
#if defined(ewarm)
extern uVectorEntry __vector_table;
#endif
//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************

//******************************************************************************
//                    FUNCTION DECLARATIONS
//******************************************************************************
extern void Microphone( void *pvParameters );
extern void DSP( void *pvParameters );

//*****************************************************************************
//
//! Application defined hook (or callback) function - the tick hook.
//! The tick interrupt can optionally call this
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
void
vApplicationTickHook( void )
{
}

//*****************************************************************************
//
//! Application defined hook (or callback) function - assert
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
void
vAssertCalled( const char *pcFile, unsigned long ulLine )
{
    while(1)
    {

    }
}

//*****************************************************************************
//
//! Application defined idle task hook
//!
//! \param  none
//!
//! \return none
//!
//*****************************************************************************
void
vApplicationIdleHook( void )
{

}

//*****************************************************************************
//
//! Application provided stack overflow hook function.
//!
//! \param  handle of the offending task
//! \param  name  of the offending task
//!
//! \return none
//!
//*****************************************************************************
void
vApplicationStackOverflowHook( OsiTaskHandle *pxTask, signed char *pcTaskName)
{
    ( void ) pxTask;
    ( void ) pcTaskName;

    for( ;; );
}

void vApplicationMallocFailedHook()
{
    while(1)
    {
        // Infinite loop;
    }
}


//*****************************************************************************
//
//! Board Initialization & Configuration
//!
//! \param  None
//!
//! \return None
//
//*****************************************************************************
void
BoardInit(void)
{
    /* In case of TI-RTOS vector table is initialize by OS itself */
#ifndef USE_TIRTOS
    //
    // Set vector table base
    //
#if defined(ccs)
    MAP_IntVTableBaseSet((unsigned long)&g_pfnVectors[0]);
#endif
#if defined(ewarm)
    MAP_IntVTableBaseSet((unsigned long)&__vector_table);
#endif
#endif
    //
    // Enable Processor
    //
    MAP_IntMasterEnable();
    MAP_IntEnable(FAULT_SYSTICK);

    PRCMCC3200MCUInit();
}

arm_fir_decimate_instance_f32* DemicatorInit() {
	arm_fir_decimate_instance_f32* decimator = (arm_fir_decimate_instance_f32*)malloc(sizeof(arm_fir_decimate_instance_f32));
	if (decimator == NULL) return NULL;
	decimator->M = 8;
	decimator->numTaps = FILTER_TAP_NUM;
	decimator->pCoeffs = filter_taps; //static float array in FIRFilterCoefficients.h
//	decimator->pState = (float32_t*)malloc(sizeof(float32_t)*(FILTER_TAP_NUM+PACKET_SIZE-1)); //for using arm_fir_decimate_f32
	decimator->pState = (float32_t*)malloc(sizeof(float32_t)*FILTER_TAP_NUM); //for using DSPDecimation
	if (decimator->pState == NULL)
		return NULL;
	return decimator;
}

//******************************************************************************
//                            MAIN FUNCTION
//******************************************************************************
int main(void) {

    BoardInit();

    //
    // Pinmux Configuration
    //
    PinMuxConfig();

    //
    // Initialising the UART terminal
    //
    InitTerm();

    long lRetVal = -1;

    //
    // Initialising the I2C Interface
    //
    lRetVal = I2C_IF_Open(1);
    if(lRetVal < 0)
    {
        ERR_PRINT(lRetVal);
        LOOP_FOREVER();
    }

    //
    // Create RX and TX Buffer
    //
	pRecordBuffer = CreateCircularBuffer(RECORD_BUFFER_SIZE);
	if(pRecordBuffer == NULL)
	{
		UART_PRINT("Unable to Allocate Memory for Tx Buffer\n\r");
		LOOP_FOREVER();
	}

	pPlayBuffer = CreateCircularBuffer(PLAY_BUFFER_SIZE);
	if(pPlayBuffer == NULL)
	{
		UART_PRINT("Unable to Allocate Memory for Rx Buffer\n\r");
		LOOP_FOREVER();
	}

    // allocate memory for the DSP instance
    g_pDSPInstance = DSPCeateInstance(PACKET_SIZE, 16000);
    if(g_pDSPInstance == NULL)
    {
        UART_PRINT("Unable to Allocate Memory for DSP Instance\n\r");
        LOOP_FOREVER();
    }

    //allocate memory for and initialize the Decimator
    g_Decimator = DemicatorInit();
    if (g_Decimator == NULL) {
        UART_PRINT("Unable to Allocate Memory for Decimator\n\r");
        LOOP_FOREVER();
    }

    //
    // Configure Audio Codec
    //
    AudioCodecReset(AUDIO_CODEC_TI_3254, NULL);
    AudioCodecConfig(AUDIO_CODEC_TI_3254, AUDIO_CODEC_16_BIT, 16000,
                      AUDIO_CODEC_STEREO, AUDIO_CODEC_SPEAKER_ALL,
                      AUDIO_CODEC_MIC_ALL);

    AudioCodecSpeakerVolCtrl(AUDIO_CODEC_TI_3254, AUDIO_CODEC_SPEAKER_ALL, 50);
    AudioCodecMicVolCtrl(AUDIO_CODEC_TI_3254, AUDIO_CODEC_SPEAKER_ALL, 50);

    //
    // Initialize the Audio(I2S) Module
    //

    AudioInit();

    //
    // Initialize the DMA Module
    //
    UDMAInit();
	UDMAChannelSelect(UDMA_CH5_I2S_TX, NULL);
	SetupPingPongDMATransferRx(pPlayBuffer);
	UDMAChannelSelect(UDMA_CH4_I2S_RX, NULL);
	SetupPingPongDMATransferTx(pRecordBuffer);

	unsigned char RecordPlay = I2S_MODE_RX_TX;
    //
    // Setup the Audio In/Out
    //
    lRetVal = AudioSetupDMAMode(DMAPingPongCompleteAppCB_opt, \
                                 CB_EVENT_CONFIG_SZ, RecordPlay);
    if(lRetVal < 0)
    {
        ERR_PRINT(lRetVal);
        LOOP_FOREVER();
    }
    AudioCaptureRendererConfigure(AUDIO_CODEC_16_BIT, 16000, AUDIO_CODEC_STEREO, RecordPlay, 1);


    //
    // Start the Microphone Task
    //
    lRetVal = osi_TaskCreate( Microphone,(signed char*)"MicroPhone", \
                               OSI_STACK_SIZE, NULL,
                               1, &g_MicTask );
    if(lRetVal < 0)
    {
        ERR_PRINT(lRetVal);
        LOOP_FOREVER();
    }

    //
    // Start the DSP Task
    //
    lRetVal = osi_TaskCreate( DSP, (signed char*)"DSP",OSI_STACK_SIZE, \
                               NULL, 1, &g_DSPTask );
    if(lRetVal < 0)
    {
        ERR_PRINT(lRetVal);
        LOOP_FOREVER();
    }


    //
    // Start Audio Tx/Rx
    //
    Audio_Start(RecordPlay);

    //
    // Start the task scheduler
    //
    osi_start();
}
