/******************************************************************************
* File Name:   main.c
*
* Description: This is the main file for mtb-example-ml-deepcraft-deploy-motion 
* Code Example.
*
* Related Document: See README.md
*
*
*******************************************************************************
* Copyright 2024-2025, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*
* This software, including source code, documentation and related
* materials ("Software") is owned by Cypress Semiconductor Corporation
* or one of its affiliates ("Cypress") and is protected by and subject to
* worldwide patent protection (United States and foreign),
* United States copyright laws and international treaty provisions.
* Therefore, you may use this Software only as provided in the license
* agreement accompanying the software package from which you
* obtained this Software ("EULA").
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software
* source code solely for use in connection with Cypress's
* integrated circuit products.  Any reproduction, modification, translation,
* compilation, or representation of this Software except as specified
* above is prohibited without the express written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer
* of such system or application assumes all risk of such use and in doing
* so agrees to indemnify Cypress against all liability.
*******************************************************************************/

#include "cy_pdl.h"
#include "cyhal.h"
#include "cybsp.h"
#include "cy_retarget_io.h"
#include <float.h>
#include <stdbool.h>

/* Model to use */
#include <models/model.h>


/*******************************************************************************
* Macros
*******************************************************************************/
/* Macro for ADC Channel configuration*/
#define SINGLE_CHANNEL 1
#define MULTI_CHANNEL  2
#define ADC_EXAMPLE_MODE SINGLE_CHANNEL
#define VPLUS_CHANNEL_0  (P10_0)


#define MICRO_TO_MILLI_CONV_RATIO        (int32_t)(1000u)
#define ACQUISITION_TIME_NS              (1000u)
#define ADC_SCAN_DELAY_MS                (200u)

/*******************************************************************************
*       Enumerated Types
*******************************************************************************/
/* ADC Channel constants*/
enum ADC_CHANNELS
{
  CHANNEL_0 = 0,
  CHANNEL_1,
  NUM_CHANNELS
} adc_channel;

/*******************************************************************************
* Function Prototypes
*******************************************************************************/
void adc_single_channel_init(void);

static void init_board(void);
static void halt_error(int code);
static float adc_read();

/*******************************************************************************
* Global Variables
*******************************************************************************/
/* EKG Values*/
#define BUFFER_SIZE            (800)

/* ADC Object */
cyhal_adc_t adc_obj;

/* ADC Channel 0 Object */
cyhal_adc_channel_t adc_chan_0_obj;

/* Default ADC configuration */
const cyhal_adc_config_t adc_config = {
        .continuous_scanning=false, // Continuous Scanning is disabled
        .average_count=1,           // Average count disabled
        .vref=CYHAL_ADC_REF_VDDA,   // VREF for Single ended channel set to VDDA
        .vneg=CYHAL_ADC_VNEG_VSSA,  // VNEG for Single ended channel set to VSSA
        .resolution = 12u,          // 12-bit resolution
        .ext_vref = NC,             // No connection
        .bypass_pin = NC };       // No connection

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary:
* This is the main function for CM4 CPU. It initializes BSP, IMU and the ML model.
* It reads data from IMU sensor continuously, processes it within the model and 
* displays the output.
*
* Parameters:
*  void
*
* Return:
*  int
*

*******************************************************************************/
int main(void)
{
    float data_buffer[BUFFER_SIZE];
    float label_scores[IMAI_DATA_OUT_COUNT];
    char *label_text[] = IMAI_DATA_OUT_SYMBOLS;
    cy_rslt_t result;
    int16_t best_label;
    float max_score;

    /* Basic board setup */
    init_board();

    /* Initialize model */
    result = IMAI_init();
    halt_error(result);

    /* Initialize IMU sampling */
    adc_single_channel_init();
    result = cyhal_adc_configure(&adc_obj, &adc_config);

    if(result != CY_RSLT_SUCCESS)
	{
		printf("ADC configuration update failed. Error: %ld\n", (long unsigned int)result);
		CY_ASSERT(0);
	}

    for (;;)
    {
        /* Move cursor home */
        printf("\033[H");
        memset(data_buffer, 0, BUFFER_SIZE * sizeof(float));
        for(int i=0;i < BUFFER_SIZE;i++){
        	data_buffer[i] = adc_read();
        	/* Give sensor data to model */
			result = IMAI_enqueue(&data_buffer[i]);
			halt_error(result);

			/* Check if there is any model output */
			best_label = 0;
			max_score = -1000.0f;
			switch(IMAI_dequeue(label_scores))
			{
				case IMAI_RET_SUCCESS:      /* We have data, display it */
					for(int i = 0; i < IMAI_DATA_OUT_COUNT; i++)
					{
						if (label_scores[i] > max_score)
						{
							max_score = label_scores[i];
							best_label = i;
						}
					}
					char* label = "Normal";
					if(best_label==2){
						label = "Abnormal";
					}
					printf("\033[H");
					printf("Output: %-30s\r\n", label);
					break;
				case IMAI_RET_NODATA:   /* No new output, continue with sampling */
					break;
				case IMAI_RET_ERROR:    /* Abort on error */
					halt_error(IMAI_RET_ERROR);
					break;
			}
        }
    }
}

static float adc_read()
{
	int32_t adc_result_0 = 0;
	adc_result_0 = cyhal_adc_read_uv(&adc_chan_0_obj) / MICRO_TO_MILLI_CONV_RATIO;
	cyhal_system_delay_us(2500);
    return (float)adc_result_0;
}


/*******************************************************************************
* Function Name: init_board
********************************************************************************
* Summary:
*    This function is a one-time setup for the board that initializes the device 
*    and board peripherals
*
* Parameters:
*    void
*
* Return:
*    void
*
*
*******************************************************************************/
static void init_board(void)
{
    cy_rslt_t result;
    /* Clear watchdog timer so that it doesn't trigger a reset */
    #if defined (CY_DEVICE_SECURE)
        cyhal_wdt_t wdt_obj;
        result = cyhal_wdt_init(&wdt_obj, cyhal_wdt_get_max_timeout_ms());
        CY_ASSERT(CY_RSLT_SUCCESS == result);
        cyhal_wdt_free(&wdt_obj);
    #endif

    /* Initialize the device and board peripherals */
    result = cybsp_init();
    halt_error(result);

    /* Enable global interrupts */
    __enable_irq();

    /* Initialize retarget-io to use the debug UART port */
    result = cy_retarget_io_init(CYBSP_DEBUG_UART_TX, CYBSP_DEBUG_UART_RX,
                                     CY_RETARGET_IO_BAUDRATE);

	/* retarget-io init failed. Stop program execution */
	if (result != CY_RSLT_SUCCESS)
	{
		CY_ASSERT(0);
	}

}


/*******************************************************************************
* Function Name: halt_error
********************************************************************************
* Summary:
*    This function halts the execution using an infinite loop. If the given
*    parameter is 0 (for success) this function does nothing.    
*
* Parameters:
*    code          Return code from the calling function 
*                  
* Return:
*    void
*
*
*******************************************************************************/
static void halt_error(int code)
{
    if(code != 0) /* Universal success code */
    {
        for(;;) /* Infinite loop to halt the execution */
        {
        }
    }
}

/////// ADC Functions

void adc_single_channel_init(void)
{
    /* Variable to capture return value of functions */
    cy_rslt_t result;

    /* Initialize ADC. The ADC block which can connect to the channel 0 input pin is selected */
    result = cyhal_adc_init(&adc_obj, VPLUS_CHANNEL_0, NULL);
    if(result != CY_RSLT_SUCCESS)
    {
        printf("ADC initialization failed. Error: %ld\n", (long unsigned int)result);
        CY_ASSERT(0);
    }

    /* ADC channel configuration */
    const cyhal_adc_channel_config_t channel_config = {
            .enable_averaging = false,  // Disable averaging for channel
            .min_acquisition_ns = ACQUISITION_TIME_NS, // Minimum acquisition time set to 1us
            .enabled = true };          // Sample this channel when ADC performs a scan

    /* Initialize a channel 0 and configure it to scan the channel 0 input pin in single ended mode. */
    result  = cyhal_adc_channel_init_diff(&adc_chan_0_obj, &adc_obj, VPLUS_CHANNEL_0,
                                          CYHAL_ADC_VNEG, &channel_config);
    if(result != CY_RSLT_SUCCESS)
    {
        printf("ADC single ended channel initialization failed. Error: %ld\n", (long unsigned int)result);
        CY_ASSERT(0);
    }

    printf("ADC is configured in single channel configuration\r\n\n");
    printf("Provide input voltage at the channel 0 input pin. \r\n\n");
}
