/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

/* Standard includes. */
#include <assert.h>
#include <stdio.h>
#include <string.h>

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "board.h"
#include "fsl_dac.h"
#include "fsl_adc16.h"
#include "fsl_dac.h"

#include "pin_mux.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DEMO_DAC_BASEADDR DAC0
uint16_t sine_val[100] = {
		2482,2559.985825,2637.663876,2714.727593,2790.87284,2865.799107,2939.210694,3010.81788,3080.338063,3147.496879,3212.029283,3273.680595,3332.207506,3387.379035,3438.977448,3486.799107,3530.655284,3570.372897,3605.795199,3636.782396,3663.212193,3684.980286,3702.000765,3714.206459,3721.549197,3724,3721.549197,3714.206459,3702.000765,3684.980286,3663.212193,3636.782395,3605.795199,3570.372896,3530.655283,3486.799107,3438.977447,3387.379035,3332.207505,3273.680595,3212.029283,3147.496879,3080.338063,3010.81788,2939.210694,2865.799107,2790.872839,2714.727592,2637.663876,2559.985825,2481.999999,2404.014174,2326.336123,2249.272407,2173.12716,2098.200892,2024.789305,1953.182119,1883.661936,1816.50312,1751.970716,1690.319404,1631.792494,1576.620964,1525.022552,1477.200893,1433.344716,1393.627103,1358.204801,1327.217604,1300.787807,1279.019714,1261.999234,1249.793541,1242.450803,1240,1242.450803,1249.793541,1261.999235,1279.019714,1300.787807,1327.217605,1358.204801,1393.627104,1433.344717,1477.200893,1525.022553,1576.620965,1631.792495,1690.319405,1751.970717,1816.503121,1883.661938,1953.182121,2024.789306,2098.200894,2173.127161,2249.272408,2326.336125,2404.014176
};

#define DEMO_ADC16_BASE ADC0
#define DEMO_ADC16_CHANNEL_GROUP 0U
#define DEMO_ADC16_USER_CHANNEL 0U /*PTE20, ADC0_SE0 */

volatile uint8_t dac_write = 0;
volatile uint8_t adc_read = 1;
#define dac_task_PRIORITY (tskIDLE_PRIORITY + 2)
#define adc_task_PRIORITY (tskIDLE_PRIORITY + 1)


/* The software timer period. */
#define DAC_WRITE_MS (20 / portTICK_PERIOD_MS)
#define SW_TIMER_PERIOD_MS (100 / portTICK_PERIOD_MS)
/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/* The callback function. */
static void SwTimerCallback(TimerHandle_t xTimer);
static void dac_task(void *pvParameters);
static void adc_task(void *pvParameters);


/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Main function
 */
TimerHandle_t SwTimerHandle = NULL;
int main(void)
{
//    TimerHandle_t SwTimerHandle = NULL;
    dac_config_t dacConfigStruct;



    /* Init board hardware. */
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();
    SystemCoreClockUpdate();
    /* Create the software timer. */
    SwTimerHandle = xTimerCreate("SwTimer",          /* Text name. */
    							 pdMS_TO_TICKS(10), /* Timer period. */
                                 pdFALSE,             /* Enable auto reload. */
                                 0,                  /* ID is not used. */
                                 SwTimerCallback);   /* The callback function. */

    DAC_GetDefaultConfig(&dacConfigStruct);
    DAC_Init(DEMO_DAC_BASEADDR, &dacConfigStruct);
    DAC_Enable(DEMO_DAC_BASEADDR, true);             /* Enable output. */
    DAC_SetBufferReadPointer(DEMO_DAC_BASEADDR, 0U);

    xTaskCreate(dac_task, "DAC_task", configMINIMAL_STACK_SIZE + 10, NULL, dac_task_PRIORITY, NULL);
    xTaskCreate(adc_task, "ADC_task", configMINIMAL_STACK_SIZE + 10, NULL, adc_task_PRIORITY, NULL);

    /* Start timer. */
    xTimerStart(SwTimerHandle, 0);
    /* Start scheduling. */
    vTaskStartScheduler();
    for (;;)
        ;
}

/*!
 * @brief Software timer callback.
 */
static void SwTimerCallback(TimerHandle_t xTimer)
{
    dac_write = 1;
    adc_read = 1;

    xTimerStart(SwTimerHandle, 0);
}

static void dac_task(void *pvParameters)
{
	static uint8_t i = 0;
	TickType_t xNextWakeTime;
	xNextWakeTime = xTaskGetTickCount();
    for (;;)
    {
    	vTaskDelayUntil(&xNextWakeTime, DAC_WRITE_MS);

        PRINTF("\n\rHello world.");

    	if(dac_write)
    	{
    		LED_GREEN_TOGGLE();
    		DAC_SetBufferValue(DEMO_DAC_BASEADDR, 0U, sine_val[i]);
    		PRINTF("\n\rDAC Value: %d",sine_val[i]);
    		i++;
    		if(i == 100)
    			i = 0;
    		dac_write = 0;

    	}
    	//xTaskCreate(adc_task, "ADC_task", configMINIMAL_STACK_SIZE + 10, NULL, adc_task_PRIORITY, NULL);
    	//vTaskDelete(NULL);
    }
}


static void adc_task(void *pvParameters)
{

	adc16_config_t adc16ConfigStruct;
	adc16_channel_config_t adc16ChannelConfigStruct;

	ADC16_GetDefaultConfig(&adc16ConfigStruct);
	#ifdef BOARD_ADC_USE_ALT_VREF
	    adc16ConfigStruct.referenceVoltageSource = kADC16_ReferenceVoltageSourceValt;
	#endif
	    ADC16_Init(DEMO_ADC16_BASE, &adc16ConfigStruct);
	    ADC16_EnableHardwareTrigger(DEMO_ADC16_BASE, false); /* Make sure the software trigger is used. */
	#if defined(FSL_FEATURE_ADC16_HAS_CALIBRATION) && FSL_FEATURE_ADC16_HAS_CALIBRATION
	    if (kStatus_Success == ADC16_DoAutoCalibration(DEMO_ADC16_BASE))
	    {
	        PRINTF("ADC16_DoAutoCalibration() Done.\r\n");
	    }
	    else
	    {
	        PRINTF("ADC16_DoAutoCalibration() Failed.\r\n");
	    }
	#endif /* FSL_FEATURE_ADC16_HAS_CALIBRATION */

	    adc16ChannelConfigStruct.channelNumber = DEMO_ADC16_USER_CHANNEL;
	    adc16ChannelConfigStruct.enableInterruptOnConversionCompleted = false;
	#if defined(FSL_FEATURE_ADC16_HAS_DIFF_MODE) && FSL_FEATURE_ADC16_HAS_DIFF_MODE
	    adc16ChannelConfigStruct.enableDifferentialConversion = false;
	#endif /* FSL_FEATURE_ADC16_HAS_DIFF_MODE */
	   while(1)
	   {
		   if(adc_read)
		   {
		ADC16_SetChannelConfig(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP, &adc16ChannelConfigStruct);
		        while (0U == (kADC16_ChannelConversionDoneFlag &
		                      ADC16_GetChannelStatusFlags(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP)))
		        {
		        }
		        PRINTF("ADC Value: %d\r\n", ADC16_GetChannelConversionValue(DEMO_ADC16_BASE, DEMO_ADC16_CHANNEL_GROUP));
		        adc_read = 0;
		   }
	   }
}
