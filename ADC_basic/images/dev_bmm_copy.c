/******************************************************************************
* File Name: dev_bmm350.c
*
* Description: This file implements the interface with the magnetometer sensor.
*
* Related Document: See README.md
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

#ifdef IM_ENABLE_BMM350

#include <stdio.h>
#include <cy_pdl.h>
#include <cyhal.h>
#include <cybsp.h>
#include <bmm350.h>
#include "cy_retarget_io.h"

#include "protocol/protocol.h"
#include "protocol/pb_encode.h"
#include "clock.h"
#include "dev_bmm350.h"
#include "common.h"

/*******************************************************************************
* Macros
*******************************************************************************/
/* Macro for ADC Channel configuration*/
#define SINGLE_CHANNEL 1
#define MULTI_CHANNEL  2

/*
 * Macro to choose between single channel and multiple channel configuration of
 * ADC. Single channel configuration uses channel 0 in single ended mode.
 * Multiple channel configuration uses two channels, channel 0 in single ended
 * mode and channel 1 in differential mode.
 *
 * The default configuration is set to use single channel.
 * To use multiple channel configuration set ADC_EXAMPLE_MODE macro to MULTI_CHANNEL.
 *
 */
#define ADC_EXAMPLE_MODE SINGLE_CHANNEL

#if defined(CY_DEVICE_PSOC6A512K)  /* if the target is CY8CPROTO-062S3-4343W or CY8CPROTO-064B0S3*/
/* Channel 0 input pin */
#define VPLUS_CHANNEL_0  (P10_3)
#else
#define VPLUS_CHANNEL_0  (P10_0)
#endif

#if ADC_EXAMPLE_MODE == MULTI_CHANNEL

#if defined(CY_DEVICE_PSOC6A256K)  /* if the target is CY8CKIT-062S4 */
/* Channel 1 VPLUS input pin */
#define VPLUS_CHANNEL_1  (P10_1)
/* Channel 1 VREF input pin */
#define VREF_CHANNEL_1   (P10_2)
#else
/* Channel 1 VPLUS input pin */
#define VPLUS_CHANNEL_1  (P10_4)
/* Channel 1 VREF input pin */
#define VREF_CHANNEL_1   (P10_5)
#endif

/* Number of scans every time ADC read is initiated */
#define NUM_SCAN                    (1)

#endif /* ADC_EXAMPLE_MODE == MULTI_CHANNEL */

/* Conversion factor */
#define MICRO_TO_MILLI_CONV_RATIO        (int32_t)(1000u)

/* Acquistion time in nanosecond */
#define ACQUISITION_TIME_NS              (1000u)

/* ADC Scan delay in millisecond */
#define ADC_SCAN_DELAY_MS                (200u)

#define MAG_OPTION_KEY_FREQUENCY 1

#ifdef IM_XSS_BMM350
#define BMM350_ADDRESS (BMM350_I2C_ADSEL_SET_LOW)
#else
#define BMM350_ADDRESS (BMM350_I2C_ADSEL_SET_HIGH)
#endif

#define _I2C_TIMEOUT_MS            (10U)
#define _I2C_WRITE_BUFFER_LENGTH   (32U)
#define _SOFT_RESET_DELAY_US       (300)

/* At 400Hz/8 = 50 Hz chunk frequency */
/* Increase this to 16 if 800Hz mode is enabled */
#define MAX_FRAMES_IN_CHUNK   (8)

/* X Y Z */
#define AXIS_COUNT            (3)

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
* Types
*******************************************************************************/


typedef struct {
    /* Hardware device */
    struct bmm350_dev sensor;

    /* Tick of last sample */
    clock_tick_t sample_time_tick;

    /* Flag indicating if it the first sample */
    bool first_sample;

    /* The sample period in ticks */
    uint32_t period_tick;

    /* Converted data */
    float data[AXIS_COUNT * MAX_FRAMES_IN_CHUNK];

    /* Number of frames collected in accel_data and gyro_data. */
    /* Cleared after each sent data-chunk. Equal or less than frames_in_chunk */
    int frames_sampled;

    /* Max number of frames in each chunk. Is less or equal to MAX_FRAMES_IN_CHUNK*/
    int frames_target;

    /* Number of frames dropped. This is reset each data-chunk. */
    int frames_dropped;
} dev_bmm350_t;


/*******************************************************************************
* Function Prototypes
*******************************************************************************/
static BMM350_INTF_RET_TYPE _bmm350_i2c_read(
        uint8_t reg_addr,
        uint8_t* reg_data,
        uint32_t len,
        void* intf_ptr);

static BMM350_INTF_RET_TYPE _bmm350_i2c_write(
        uint8_t reg_addr,
        const uint8_t* reg_data,
        uint32_t len,
        void* intf_ptr);

static void _bmm350_delay_us(uint32_t us, void* intf_ptr);

static bool _init_hw(dev_bmm350_t *dev, cyhal_i2c_t* i2c);

static bool _config_hw(dev_bmm350_t *dev, int rate);

static bool _read_hw(dev_bmm350_t* dev);

static bool _configure_streams(protocol_t* protocol, int device, void* arg);

static void _start_streams(protocol_t* protocol, int device, pb_ostream_t* ostream, void* arg);

static void _stop_streams(protocol_t* protocol, int device, pb_ostream_t* ostream, void* arg);

static bool _write_payload(protocol_t* protocol, int device_id, int stream_id, int frame_count, int total_bytes, pb_ostream_t* ostream, void* arg);

static void _poll_streams(protocol_t* protocol, int device, pb_ostream_t* ostream, void* arg);

#if ADC_EXAMPLE_MODE == MULTI_CHANNEL

/* Multichannel initialization function */
void adc_multi_channel_init(void);

/* Function to read input voltage from multiple channels */
void adc_multi_channel_process(void);

/* ADC Event Handler */
static void adc_event_handler(void* arg, cyhal_adc_event_t event);

#else /* ADC_EXAMPLE_MODE == SINGLE_CHANNEL */

/* Single channel initialization function*/
void adc_single_channel_init(void);

/* Function to read input voltage from channel 0 */
void adc_single_channel_process(void);

#endif /* ADC_EXAMPLE_MODE == MULTI_CHANNEL */

/*******************************************************************************
* Global Variables
*******************************************************************************/
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

#if ADC_EXAMPLE_MODE == MULTI_CHANNEL

/* Asynchronous read complete flag, used in Event Handler */
static bool async_read_complete = false;

/* ADC Channel 1 Object */
cyhal_adc_channel_t adc_chan_1_obj;

/* Variable to store results from multiple channels during asynchronous read*/
int32_t result_arr[NUM_CHANNELS * NUM_SCAN] = {0};

#endif /* ADC_EXAMPLE_MODE == MULTI_CHANNEL */

/*******************************************************************************
* Function Definitions
*******************************************************************************/

/*****************************************************************************
* Function name: _bmm350_i2c_read
*****************************************************************************
* Summary:
* This internal function reads I2C function map to host MCU
*
* Parameters:
*  reg_addr    8bit register address of the sensor
*  reg_data    Data from the specified address
*  len         Length of the reg_data array
*  intf_ptr    Void pointer that can enable the linking of descriptors for interface related
*  callbacks
*
* Return:
*  int8_t     Status of execution
*
*****************************************************************************/
static BMM350_INTF_RET_TYPE _bmm350_i2c_read(
        uint8_t reg_addr,
        uint8_t* reg_data,
        uint32_t len,
        void* intf_ptr)
{
    cyhal_i2c_t *i2c = (cyhal_i2c_t*)intf_ptr;

    return (BMM350_INTF_RET_TYPE)cyhal_i2c_master_mem_read(
        i2c,
        BMM350_ADDRESS,
        reg_addr,
        1,
        reg_data,
        (uint16_t)len,
        _I2C_TIMEOUT_MS);
}


/*****************************************************************************
* Function name: _bmm350_i2c_write
*****************************************************************************
* Summary:
* This internal function writes I2C function map to host MCU
*
* Parameters:
*  reg_addr    8bit register address of the sensor
*  reg_data    Data from the specified address
*  len         Length of the reg_data array
*  intf_ptr    Void pointer that can enable the linking of descriptors for interface related
*  callbacks
*
* Return:
*  int8_t     Status of execution
*
*****************************************************************************/
static BMM350_INTF_RET_TYPE _bmm350_i2c_write(
        uint8_t reg_addr,
        const uint8_t* reg_data,
        uint32_t len,
        void* intf_ptr)
{
    cyhal_i2c_t *i2c = (cyhal_i2c_t*)intf_ptr;

    return (BMM350_INTF_RET_TYPE)cyhal_i2c_master_mem_write(
            i2c,
            BMM350_ADDRESS,
            reg_addr,
            1,
            reg_data,
            (uint16_t)len,
            _I2C_TIMEOUT_MS);
}


/*****************************************************************************
* Function name: _bmm350_delay_us
*****************************************************************************
* Summary:
* This internal function maps delay function to host MCU
*
* Parameters:
*  us    The time period in microseconds
*  intf_ptr  Void pointer that can enable the linking of descriptors for
*  interface related callbacks
*
*****************************************************************************/
static void _bmm350_delay_us(uint32_t us, void* intf_ptr)
{
    UNUSED(intf_ptr);

    cyhal_system_delay_us(us);
}


/******************************************************************************
* Function Name: _init_hw
********************************************************************************
* Summary:
*   Initializes the bmm350.
*
* Parameters:
*   dev: Pointer to the dev_bmm350_t device handle.
*   i2c: Pointer to the I2C interface resource.
*
* Return:
*   True if operation is successful, otherwise false.
*
*******************************************************************************/
static bool _init_hw(dev_bmm350_t *dev, cyhal_i2c_t* i2c)
{
	/////////ADC
    /* Variable to capture return value of functions */
    cy_rslt_t result;

#if defined(CY_DEVICE_SECURE)
    cyhal_wdt_t wdt_obj;
    /* Clear watchdog timer so that it doesn't trigger a reset */
    result = cyhal_wdt_init(&wdt_obj, cyhal_wdt_get_max_timeout_ms());
    CY_ASSERT(CY_RSLT_SUCCESS == result);
    cyhal_wdt_free(&wdt_obj);
#endif

    /* Initialize the device and board peripherals */
    result = cybsp_init();

    /* Board init failed. Stop program execution */
    if (result != CY_RSLT_SUCCESS)
    {
        CY_ASSERT(0);
    }

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

    /* Print message */
    /* \x1b[2J\x1b[;H - ANSI ESC sequence for clear screen */
    printf("\x1b[2J\x1b[;H");
    printf("-----------------------------------------------------------\r\n");
    printf("HAL: ADC using HAL\r\n");
    printf("-----------------------------------------------------------\r\n\n");

#if ADC_EXAMPLE_MODE == MULTI_CHANNEL

    /* Initialize Channel 0 and Channel 1 */
    adc_multi_channel_init();

#else /* ADC_EXAMPLE_MODE == SINGLE_CHANNEL */

    /* Initialize Channel 0 */
    adc_single_channel_init();

#endif /* ADC_EXAMPLE_MODE == MULTI_CHANNEL */

    /* Update ADC configuration */
    result = cyhal_adc_configure(&adc_obj, &adc_config);
    if(result != CY_RSLT_SUCCESS)
    {
        printf("ADC configuration update failed. Error: %ld\n", (long unsigned int)result);
        CY_ASSERT(0);
    }
	///////ADC END
    int8_t rslt;
    uint8_t int_ctrl = 0;
    uint8_t err_reg_data = 0;
    struct bmm350_pmu_cmd_status_0 pmu_cmd_stat_0;

    struct bmm350_dev *sensor = &(dev->sensor);

    dev->sensor.intf_ptr = i2c;
    dev->sensor.read = _bmm350_i2c_read;
    dev->sensor.write = _bmm350_i2c_write;
    dev->sensor.delay_us = _bmm350_delay_us;

    /* Initialize BMM350 */
    rslt = bmm350_init(sensor);
    if (rslt != BMM350_OK)
    {
        return false;
    }

    /* Check PMU busy */
    rslt = bmm350_get_pmu_cmd_status_0(&pmu_cmd_stat_0, sensor);
    if (rslt != BMM350_OK)
    {
        return false;
    }

    /* Get error data */
    rslt = bmm350_get_regs(BMM350_REG_ERR_REG, &err_reg_data, 1, sensor);
    if (rslt != BMM350_OK)
    {
        return false;
    }

    /* Configure interrupt settings */
    rslt = bmm350_configure_interrupt(
        BMM350_PULSED,
        BMM350_ACTIVE_HIGH,
        BMM350_INTR_PUSH_PULL,
        BMM350_UNMAP_FROM_PIN,
        sensor);
    if (rslt != BMM350_OK)
    {
        return false;
    }

    /* Enable data ready interrupt */
    rslt = bmm350_enable_interrupt(BMM350_ENABLE_INTERRUPT, sensor);
    if (rslt != BMM350_OK)
           return false;

    /* Get interrupt settings */
    rslt = bmm350_get_regs(BMM350_REG_INT_CTRL, &int_ctrl, 1, sensor);
    if (rslt != BMM350_OK)
           return false;

    /* Set ODR and performance */
    rslt = bmm350_set_odr_performance(BMM350_DATA_RATE_50HZ, BMM350_AVERAGING_8, sensor);
    if (rslt != BMM350_OK)
           return false;

    /* Enable all axis */
    rslt = bmm350_enable_axes(BMM350_X_EN, BMM350_Y_EN, BMM350_Z_EN, sensor);
    if (rslt != BMM350_OK)
           return false;

    /* Set power mode */
    rslt = bmm350_set_powermode(BMM350_NORMAL_MODE, &(dev->sensor));
    if (rslt != BMM350_OK)
           return false;

    printf("bmm350: Initialized device.\r\n");

    return true;
}

/******************************************************************************
* Function Name: _config_hw
********************************************************************************
* Summary:
*   Configures the bmm350 output data rate and measurement range.
*
* Parameters:
*   dev: Pointer to the dev_bmm350_t device handle.
*   rate: Sample frequency (Hz).
*
* Return:
*   True if operation is successful, otherwise false.
*
*******************************************************************************/
static bool _config_hw(dev_bmm350_t *dev, int rate)
{
    int8_t result;
    enum bmm350_data_rates odr;
    enum bmm350_performance_parameters avg;

    dev->sample_time_tick = 0;
    dev->period_tick = CLOCK_TICK_PER_SECOND / rate;
    dev->frames_dropped = 0;
    dev->frames_sampled = 0;

    switch(rate)
    {
    case 50:
        odr = BMM350_DATA_RATE_50HZ;
        avg = BMM350_AVERAGING_8;
        dev->frames_target = 1;
        break;
    case 100:
        odr = BMM350_DATA_RATE_100HZ;
        avg = BMM350_AVERAGING_4;
        dev->frames_target = 2;
        break;
    case 200:
        odr = BMM350_DATA_RATE_200HZ;
        avg = BMM350_AVERAGING_2;
        dev->frames_target = 4;
        break;
    case 400:
        odr = BMM350_DATA_RATE_400HZ;
        avg = BMM350_NO_AVERAGING;
        dev->frames_target = 8;
        break;
    default:
        return false;
    }

    /* Set ODR and performance */
    result = bmm350_set_odr_performance(odr, avg, &(dev->sensor));
    if (result != BMM350_OK)
           return false;

    printf("bmi350: Configured device. rate=%d Hz, frames/chunk=%d\r\n", rate, dev->frames_target);

    return true;
}

/******************************************************************************
* Function Name: _read_hw
********************************************************************************
* Summary:
*   Reads the current bmm350 data and convert it.
*
* Parameters:
*   dev: Pointer to the dev_bmm350_t device handle.
*
* Return:
*   True if data retrieval is successful, otherwise false.
*
*******************************************************************************/
static bool _read_hw(dev_bmm350_t* dev)
{
    int8_t result;
    uint8_t int_status;
    struct bmm350_dev *sensor = &dev->sensor;

    result = bmm350_get_regs(BMM350_REG_INT_STATUS, &int_status, 1, sensor);

    if(result != BMM350_OK || !(int_status & BMM350_DRDY_DATA_REG_MSK))
    {
        return false;
    }

   struct bmm350_mag_temp_data data;
   result = bmm350_get_compensated_mag_xyz_temp_data(&data, sensor);
   if(result != BMM350_OK)
       return false;

   int32_t adc_result_0 = 0;
   adc_result_0 = cyhal_adc_read_uv(&adc_chan_0_obj) / MICRO_TO_MILLI_CONV_RATIO;
   float res = (float)adc_result_0;
   data.y = res;
   float *dest = dev->data + dev->frames_sampled * AXIS_COUNT;
   *dest++ = data.y;
   *dest++ = data.x;
   *dest++ = data.z;

   dev->frames_sampled++;
   return true;
}

/*******************************************************************************
* Function Name: _configure_streams
********************************************************************************
* Summary:
*   Configures the device streams based on the selected options.
*
* Parameters:
*   protocol: Pointer to the protocol handle.
*   device: The device index.
*   arg: Pointer to the device structure (dev_bmm350_t).
*
* Return:
*   True to keep the connection open, false to close.
*
*******************************************************************************/
static bool _configure_streams(protocol_t* protocol, int device, void* arg)
{
    dev_bmm350_t* dev = (dev_bmm350_t*)arg;
    int status;

    int frequency_index;
    int rate;
    status = protocol_get_option_oneof(protocol, device, MAG_OPTION_KEY_FREQUENCY, &frequency_index);
    if(status != PROTOCOL_STATUS_SUCCESS) {
        protocol_set_device_status(
                protocol,
                device,
                protocol_DeviceStatus_DEVICE_STATUS_ERROR,
                "Failed to get option frequency.");
        return true;
    }

    switch(frequency_index) {
       case 0: rate = 50; break;
       case 1: rate = 100; break;
       case 2: rate = 200; break;
       case 3: rate = 400; break;
       default: return false;
    }

    /* Clear any existing streams */
    if(protocol_clear_streams(protocol, device) != PROTOCOL_STATUS_SUCCESS) {
        protocol_set_device_status(
                protocol,
                device,
                protocol_DeviceStatus_DEVICE_STATUS_ERROR,
                "Failed to clear streams.");
        return true;
    }

    /* Add a stream based on current configuration */
    int stream = protocol_add_stream(
        protocol,
        device,
        "Mag",
        protocol_StreamDirection_STREAM_DIRECTION_OUTPUT,
        protocol_DataType_DATA_TYPE_F32,
        rate,
        1,
        "µT");

    if(stream < 0) {
        protocol_set_device_status(
                protocol,
                device,
                protocol_DeviceStatus_DEVICE_STATUS_ERROR,
                "Failed to add streams.");
        return true;
    }

    protocol_add_stream_rank(
            protocol,
            device,
            stream,
            "Axis",
            3,
            (const char* []) { "X", "Y", "Z" });

   if(!_config_hw(dev, rate)) {
       protocol_set_device_status(
                 protocol,
                 device,
                 protocol_DeviceStatus_DEVICE_STATUS_ERROR,
                 "Failed to configure device.");
       return true;
   }

   protocol_set_device_status(
             protocol,
             device,
             protocol_DeviceStatus_DEVICE_STATUS_READY,
             "Device is ready.");

    return true;
}

/*******************************************************************************
* Function Name: _start_streams
********************************************************************************
* Summary:
*  Called when streaming is started. This may also initialize the device.
*
* Parameters:
*  protocol: Pointer to the protocol handle.
*  device: The device index.
*  ostream: Pointer to the output stream to write to
*  arg: Pointer to the device structure (dev_bmm350_t).
*
*******************************************************************************/
static void _start_streams(protocol_t* protocol, int device, pb_ostream_t* ostream, void* arg)
{
    UNUSED(ostream);
    dev_bmm350_t* dev = (dev_bmm350_t*)arg;
    protocol_set_device_status(
            protocol,
            device,
            protocol_DeviceStatus_DEVICE_STATUS_ACTIVE,
            "Device is streaming");

    dev->first_sample = true;
    dev->sample_time_tick = clock_get_tick();
}

/*******************************************************************************
* Function Name: _stop_streams
********************************************************************************
* Summary:
*  Called when streaming is stopped.
*
* Parameters:
*  protocol: Pointer to the protocol handle.
*  device: The device index.
*  ostream: Pointer to the output stream to write to
*  arg: Pointer to the device structure (dev_bmm350_t).
*
*******************************************************************************/
static void _stop_streams(protocol_t* protocol, int device, pb_ostream_t* ostream, void* arg)
{
    dev_bmm350_t* dev = (dev_bmm350_t*)arg;

    UNUSED(dev);
    UNUSED(ostream);

    protocol_set_device_status(
            protocol,
            device,
            protocol_DeviceStatus_DEVICE_STATUS_READY,
            "Device stopped");
}

/*******************************************************************************
* Function Name: _write_payload
********************************************************************************
* Summary:
*  Used by protocol_send_data_chunk to write the actual data.
*
* Parameters:
*  protocol: Pointer to the protocol handle.
*  device_id: The device index.
*  stream_id: The stream index.
*  frame_count: Number of frames to write.
*  total_bytes: Total number of bytes to write (= frame_count * sizeof(type) * frame_shape.flat).
*  ostream: Pointer to the output stream to write to.
*  arg: Pointer to the device structure (dev_bmm350_t).
*
* Return:
*  True if the payload was written successfully, else false.
*
*******************************************************************************/
static bool _write_payload(
    protocol_t* protocol,
    int device_id,
    int stream_id,
    int frame_count,
    int total_bytes,
    pb_ostream_t* ostream,
    void* arg)
{
    UNUSED(protocol);
    UNUSED(device_id);
    UNUSED(stream_id);
    UNUSED(frame_count);
    UNUSED(protocol);

    dev_bmm350_t* dev = (dev_bmm350_t*)arg;

    if (!pb_write(ostream, (const pb_byte_t *)dev->data, total_bytes))
    {
        return false;
    }

    return true;
}


/*******************************************************************************
* Function Name: _poll_streams
********************************************************************************
* Summary:
*  Called periodically to send data messages.
*
* Parameters:
*  protocol: Pointer to the protocol handle.
*  device: The device index.
*  ostream: Pointer to the output stream to write to.
*  arg: Pointer to the device structure (dev_bmm350_t).
*
*******************************************************************************/
static void _poll_streams(protocol_t* protocol, int device, pb_ostream_t* ostream, void* arg)
{
    dev_bmm350_t* dev = (dev_bmm350_t*)arg;
    clock_tick_t current_time = clock_get_tick();

/* Reinterpret this timing as the time we wish the sample to happen at.
 * It is initialized to the current clock when starting the stream. */
    clock_tick_t current_threshold = dev->sample_time_tick;

/* Calculate how late we are. */
    clock_tick_t total_drift = current_time - current_threshold;

/* If we are to late we skip this frame and save time.
 * Previous data package will be resent. */
    bool late = false;
    uint32_t drift_ms = (uint32_t)total_drift;
    if (drift_ms > 200 ){ /* Tens of microseconds. 200 is equal to 2 ms */
        late=1;
    }

    if(current_time >= current_threshold ){

        /* If we are late.. Skip the reading of the sensor.. */
        if ( late && !dev->first_sample){ /* Check the total drift here.. */
            late = false;
        }
        else{   /* Ok, we are on time so try reading the sensor. */
            if(!_read_hw(dev)) {
                return; /* If it fails it will retry until we will be late and skipping this frame. */
            }
            dev->first_sample = false;
        }

        /* This is updated whenever there is an _read_hw call! However since the _read_hw might fail */
        /* and when we are resending the previous data we have to force this to 1 anyway. */
        dev->frames_sampled = 1;

        /* Since we in reality dont drop any frames anymore. */
        dev->frames_dropped = 0;

        /* When we should do the next frame. */
        dev->sample_time_tick += dev->period_tick;

        {
            protocol_send_data_chunk(protocol, device, 0, dev->frames_sampled, dev->frames_dropped, ostream, _write_payload);
            dev->frames_dropped = 0;
            dev->frames_sampled = 0;
        }
    }
}

/*******************************************************************************
* Function Name: dev_bmm350_register
********************************************************************************
* Summary:
*  Registers this device. This is the only exported symbol from this file.
*
* Parameters:
*  protocol: Pointer to the protocol handle.
*  i2c: Pointer to the I2C handle.
*  spi: Pointer to the SPI handle.
*
* Returns:
*  True on success, else false.
*
*******************************************************************************/
bool dev_bmm350_register(protocol_t* protocol, cyhal_i2c_t* i2c)
{
    int status;

    dev_bmm350_t* dev = (dev_bmm350_t*)malloc(sizeof(dev_bmm350_t));
    if(dev == NULL)
    {
        return false;
    }
    memset(dev, 0, sizeof(dev_bmm350_t));


    if(!_init_hw(dev, i2c))
    {
        free(dev);
        return false;
    }

    device_manager_t manager = {
        .arg = dev,
        .configure_streams =  _configure_streams,
        .start =  _start_streams,
        .stop =  _stop_streams,
        .poll =  _poll_streams,
        .data_received =  NULL /* has no input streams */
    };

    int device = protocol_add_device(
        protocol,
        protocol_DeviceType_DEVICE_TYPE_SENSOR,
        "Magnetometer",
        "Magnetometer (BMM350)",
        manager);

    if(device < 0)
    {
        return false;
    }

    status = protocol_add_option_oneof(
            protocol,
            device,
            MAG_OPTION_KEY_FREQUENCY,
            "Frequency",
            "Sample frequency (Hz)",
            0,
            (const char* []) { "50 Hz", "100 Hz", "200 Hz", "400 Hz" },
            4);

    if(status != PROTOCOL_STATUS_SUCCESS)
    {
        return false;
    }

    if(!_configure_streams(protocol, device, manager.arg))
    {
        return false;
    }

    return true;
}

#if ADC_EXAMPLE_MODE == MULTI_CHANNEL

void adc_multi_channel_init(void)
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
        printf("ADC first channel initialization failed. Error: %ld\n", (long unsigned int)result);
        CY_ASSERT(0);
    }

    /*
     * For multichannel configuration use to same channel configuration structure
     * "channel_config" to configure the second channel.
     * For second channel to be set to differential mode, two inputs from Pins
     * channel 1 input pin and channel 1 voltage reference pin are configured to be inputs.
     *
     */

    /* Initialize channel 1 in differential mode with VPLUS and VMINUS input pins */
    result = cyhal_adc_channel_init_diff(&adc_chan_1_obj, &adc_obj, VPLUS_CHANNEL_1,
                                         VREF_CHANNEL_1, &channel_config);
    if(result != CY_RSLT_SUCCESS)
    {
        printf("ADC second channel initialization failed. Error: %ld\n", (long unsigned int)result);
        CY_ASSERT(0);
    }

    /* Register a callback to handle asynchronous read completion */
     cyhal_adc_register_callback(&adc_obj, &adc_event_handler, result_arr);

     /* Subscribe to the async read complete event to process the results */
     cyhal_adc_enable_event(&adc_obj, CYHAL_ADC_ASYNC_READ_COMPLETE, CYHAL_ISR_PRIORITY_DEFAULT, true);

     printf("ADC is configured in multichannel configuration.\r\n\n");
     printf("Channel 0 is configured in single ended mode, connected to the \r\n");
     printf("channel 0 input pin. Provide input voltage at the channel 0 input pin \r\n");
     printf("Channel 1 is configured in differential mode, connected to the \r\n");
     printf("channel 1 input pin and channel 1 voltage reference pin. Provide input voltage at the channel 1 input pin and reference \r\n");
     printf("voltage at the Channel 1 voltage reference pin \r\n\n");
}

/*******************************************************************************
 * Function Name: adc_multi_channel_process
 *******************************************************************************
 *
 * Summary:
 *  ADC single channel process function. This function reads the input voltage
 *  from channel 0 and channel 1. Prints the input voltage on UART.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void adc_multi_channel_process(void)
{
    /* Variable to capture return value of functions */
    cy_rslt_t result;

    /* Variable to store ADC conversion result from channel 0 */
    int32_t adc_result_0 = 0;

    /* Variable to store ADC conversion result from channel 1 */
    int32_t adc_result_1 = 0;

    /* Initiate an asynchronous read operation. The event handler will be called
     * when it is complete. */
    result = cyhal_adc_read_async_uv(&adc_obj, NUM_SCAN, result_arr);
    if(result != CY_RSLT_SUCCESS)
    {
        printf("ADC async read failed. Error: %ld\n", (long unsigned int)result);
        CY_ASSERT(0);
    }

    /*
     * Read data from result list, input voltage in the result list is in
     * microvolts. Convert it millivolts and print input voltage
     *
     */
    adc_result_0 = result_arr[CHANNEL_0] / MICRO_TO_MILLI_CONV_RATIO;
    adc_result_1 = result_arr[CHANNEL_1] / MICRO_TO_MILLI_CONV_RATIO;
    printf("Channel 0 input: %4ldmV \t Channel 1 input: %4ldmV\r\n", (long int)adc_result_0, (long int)adc_result_1);

    /* Clear async read complete flag */
    async_read_complete = false;
}


/*******************************************************************************
 * Function Name: adc_event_handler
 *******************************************************************************
 *
 * Summary:
 *  ADC event handler. This function handles the asynchronous read complete event
 *  and sets the async_read_complete flag to true.
 *
 * Parameters:
 *  void *arg : pointer to result list
 *  cyhal_adc_event_t event : ADC event type
 *
 * Return:
 *  void
 *
 *******************************************************************************/
static void adc_event_handler(void* arg, cyhal_adc_event_t event)
{
    if(0u != (event & CYHAL_ADC_ASYNC_READ_COMPLETE))
    {
        /* Set async read complete flag to true */
        async_read_complete = true;
    }
}

#else

/*******************************************************************************
 * Function Name: adc_single_channel_init
 *******************************************************************************
 *
 * Summary:
 *  ADC single channel initialization function. This function initializes and
 *  configures channel 0 of ADC.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *******************************************************************************/
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

/*******************************************************************************
 * Function Name: adc_single_channel_process
 *******************************************************************************
 *
 * Summary:
 *  ADC single channel process function. This function reads the input voltage
 *  and prints the input voltage on UART.
 *
 * Parameters:
 *  void
 *
 * Return:
 *  void
 *
 *******************************************************************************/
void adc_single_channel_process(void)
{
    /* Variable to store ADC conversion result from channel 0 */
    int32_t adc_result_0 = 0;

    /* Read input voltage, convert it to millivolts and print input voltage */
    adc_result_0 = cyhal_adc_read_uv(&adc_chan_0_obj) / MICRO_TO_MILLI_CONV_RATIO;
    printf("Channel 0 input: %4ldmV\r\n", (long int)adc_result_0);
}

#endif /* ADC_EXAMPLE_MODE == MULTI_CHANNEL */

/* [] END OF FILE */


#endif /* IM_ENABLE_BMM350 */

/* [] END OF FILE */
