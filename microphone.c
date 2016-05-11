//*****************************************************************************
// Microphone.c
//
// Line IN (Microphone interface)
//
// Copyright (C) 2014 Texas Instruments Incorporated - http://www.ti.com/ 
// 
// 
//  Redistribution and use in source and binary forms, with or without 
//  modification, are permitted provided that the following conditions 
//  are met:
//
//    Redistributions of source code must retain the above copyright 
//    notice, this list of conditions and the following disclaimer.
//
//    Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the 
//    documentation and/or other materials provided with the   
//    distribution.
//
//    Neither the name of Texas Instruments Incorporated nor the names of
//    its contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
//  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
//  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
//  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

// Hardware & driverlib library includes
#include "rom.h"
#include "rom_map.h"
#include "utils.h"
#include "hw_ints.h"

// common interface includes
#include "common.h"
#include "uart_if.h"

// Demo app includes
#include "circ_buff.h"
#include "audioCodec.h"
#include "pcm_handler.h"

#include "dsp_CMSIS.h"

#include <stdlib.h>

//*****************************************************************************
//                 GLOBAL VARIABLES -- Start
//*****************************************************************************

int g_iSentCount = 0;
int g_iReceiveCount = 0;
extern tCircularBuffer *pPlayBuffer;
extern tCircularBuffer *pRecordBuffer;

extern tDSPInstance* g_pDSPInstance;
extern volatile tboolean gb_DSPprocessing;

extern arm_fir_decimate_instance_f32* g_Decimator;

float32_t tempBufferInput[PACKET_SIZE/2];
float32_t tempBufferOutput[PACKET_SIZE/16];
uint8_t dummyIn[PACKET_SIZE];
uint8_t dummy[PACKET_SIZE/8];

//*****************************************************************************
//                 GLOBAL VARIABLES -- End
//*****************************************************************************


//*****************************************************************************
//
//! Microphone Routine 
//!
//! \param pvParameters     Parameters to the task's entry function
//!
//! \return None
//
//*****************************************************************************

void Microphone( void *pvParameters )
{
    long lRetVal = -1;

    while(1)
    {
		int iBufferFilled = 0;
		iBufferFilled = GetBufferSize(pRecordBuffer);
		if(iBufferFilled >= (PACKET_SIZE*2))
		{
			//reflect to play buffer for monitoring through speaker/headphone
			lRetVal = FillBuffer(pPlayBuffer, (unsigned char*)(pRecordBuffer->pucReadPtr), PACKET_SIZE);

			//decimating to dsp buffer for processing later on
			//convert to float32_t input array
			 DSPConvertUCtoF32_extern((unsigned char*)(pRecordBuffer->pucReadPtr), tempBufferInput, PACKET_SIZE);
			//do decimation;
//			arm_fir_decimate_f32(g_Decimator, tempBufferInput, tempBufferOutput, PACKET_SIZE/2);
			DSPDecimation(g_Decimator, tempBufferInput, tempBufferOutput, PACKET_SIZE/2);
//			int i;
//			UART_PRINT("\n\r\n\r");
//			for (i=0; i<PACKET_SIZE/2; i++) {
//				UART_PRINT("%f ", tempBufferInput[i]);
//			}
//			UART_PRINT("\n\r\n\r");
//			for (i=0; i<PACKET_SIZE/16; i++) {
//				UART_PRINT("%f ", tempBufferOutput[i]);
//			}
//			UART_PRINT("\n\r\n\r");
//			UART_PRINT("\n\r\n\r");
			//convert output back to unsigned char array into DSP buffer.
			//DSP buffer needs to hold two processing buffers, each is 3 seconds of heart beat (2 beats for 40bpm or 3 beats for 60bpm)
			//given we decimated the signal from 16khz to 2khz, we have 3*2k = 6k of data for each buffer.
			DSPConvertF32toUC_extern(tempBufferOutput, (unsigned char*)(pDSPBuffer->pucWritePtr), PACKET_SIZE/8);
			UpdateWritePtr(pDSPBuffer, PACKET_SIZE);

			if(lRetVal < 0)
			{
				UART_PRINT("Unable to fill buffer\n\r");
			}
			g_iReceiveCount++;
//            UART_PRINT("total packets received: %d\n\r", g_iReceiveCount);

			if (DSPReadyForSignal()) {
				//grab into buffer for dsp
				lRetVal = PeekBuffer(pRecordBuffer, g_pDSPInstance->ucpSignal, \
					g_pDSPInstance->signalSize);
				DSPProcessSignal(g_pDSPInstance);
//				UART_PRINT("size read into buffer: %d\n\r", lRetVal);
			}

			UpdateReadPtr(pRecordBuffer, PACKET_SIZE);
			g_iSentCount++;
		}

        MAP_UtilsDelay(1000);
    }
}
