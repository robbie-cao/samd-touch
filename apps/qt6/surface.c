/* This source file is part of the ATMEL QTouch Surface Library 1.0.3 */

/*****************************************************************************
 *
 * \file
 *
 * \brief  This file contains the QTouch Surface Library pin, register and
 * sensors configuration options for Capacitive Touch acquisition using
 * the PTC module.
 *
 * - User guide:         QTouch Library - QTouch Surface User Guide.
 * - Support email:      www.atmel.com/design-support/
 *
 *
 * Copyright (c) 2013 Atmel Corporation. All rights reserved.
 *
 * \asf_license_start
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. The name of Atmel may not be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * 4. This software may only be redistributed and used in connection with an
 *    Atmel microcontroller product.
 *
 * THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * EXPRESSLY AND SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * \asf_license_stop
 *
 ******************************************************************************/
#include <asf.h>
#include "touch.h"
#include "surface.h"
#include "surf_api.h"
#include "touch_api_SAMD.h"

#include "log.h"


/*----------------------------------------------------------------------------
 *                                   macros
 *----------------------------------------------------------------------------*/

#define PTC_APBC_BITMASK        (1u << 19u)

/*Do not Modify this macro used by library for internal purpose*/
#define DEF_MUTLCAP_NUM_LUMPED_SENSORS  1u

#if (DEF_SURF_LOW_POWER_SENSOR_ENABLE == 1)
/*Low Power Sensor Support */

/*macros for controlling application mode*/
#define NORMAL_MODE             0
#define LOW_POWER_MODE          1
#define DRIFT_MODE              2

/*macros for controlling low power mode running status */
#define NOT_RUNNING             0
#define RUNNING                 1
#define INTERRUPTED             2

#if DEF_SURF_TCH_DRIFT_PERIOD <= DEF_SURF_ATCH_DRIFT_PERIOD
#define DRIFT_PERIOD_MS         (DEF_SURF_TCH_DRIFT_PERIOD * 200u)
#else
#define DRIFT_PERIOD_MS         (DEF_SURF_ATCH_DRIFT_PERIOD * 200u)
#endif

/*Count Delay to trigger Low Power measurement*/
#define LOW_POWER_WAIT_CNT      20
#endif /* DEF_SURF_LOW_POWER_SENSOR_ENABLE */


/*----------------------------------------------------------------------------
 *                               prototypes
 *----------------------------------------------------------------------------*/

/*!
 * \brief configure Sensors based on Mutual Cap Technology
 */
static touch_ret_t touch_sensors_config(void);
/*!
 * \brief Intialize the touch surface library.
 */
void qts_init_surface(void);
/*!
 * \brief start calibration for Touch Surface.
 */
void qts_start(void);
/*!
 * \brief To measure the Touch Surface by using surface library in normal mode.
 */
void qts_normal_process(void);
/*!
 * \brief Configure the different sensors for the Touch Surface.
 */
touch_ret_t qts_sensors_config(void);
/*!
 * \brief Callback function from library to indicate that measurement cycle is complete.
 */
uint8_t surf_complete_callback(uint16_t);

/*!
 * \brief Initialize and enable PTC clock.
 */
void surface_configure_ptc_clock(void);

#if (DEF_SURF_LOW_POWER_SENSOR_ENABLE == 1)
/*!
 * \brief Configure the Event System for Low Power Mode operation.
 */
void config_timer_evsys(void);
/*!
 * \brief Reset the Event System.
 */
void reset_evsys(void);
/*!
 * \brief To measure the Touch Surface by using surface library in Low Power mode.
 * Using Low Power Sensor.
 */
void qts_process_lp(void);

#endif /* DEF_SURF_LOW_POWER_SENSOR_ENABLE  */

/*!
 * \name surf_complete_callback function.
 */
#define SURF_COMPLETE_CALLBACK          (surf_complete_callback)


/*----------------------------------------------------------------------------
 *                               global variables
 *----------------------------------------------------------------------------*/

surf_seg_size_t surf_seg_size[DEF_SURF_NUM_SEGMENTS] =
{
    { SURF_SEG0_ROW_START , SURF_SEG0_ROW_END , SURF_SEG0_COL_START , SURF_SEG0_COL_END } ,
    { SURF_SEG1_ROW_START , SURF_SEG1_ROW_END , SURF_SEG1_COL_START , SURF_SEG1_COL_END } ,
    { SURF_SEG2_ROW_START , SURF_SEG2_ROW_END , SURF_SEG2_COL_START , SURF_SEG2_COL_END } ,
    { SURF_SEG3_ROW_START , SURF_SEG3_ROW_END , SURF_SEG3_COL_START , SURF_SEG3_COL_END } ,
    { SURF_SEG4_ROW_START , SURF_SEG4_ROW_END , SURF_SEG4_COL_START , SURF_SEG4_COL_END } ,
    { SURF_SEG5_ROW_START , SURF_SEG5_ROW_END , SURF_SEG5_COL_START , SURF_SEG5_COL_END } ,
    { SURF_SEG6_ROW_START , SURF_SEG6_ROW_END , SURF_SEG6_COL_START , SURF_SEG6_COL_END } ,
    { SURF_SEG7_ROW_START , SURF_SEG7_ROW_END , SURF_SEG7_COL_START , SURF_SEG7_COL_END }
};

surf_status_t surf_status;

uint8_t surf_data_blk[PRIV_SURF_DATA_BLK_SIZE];
surf_tch_status_t  surf_tch_status[DEF_SURF_MAX_TCH];


/**
 * Mutual Cap sensors measured data pointer.
 * Note: This pointer is initialized by the QTouch library once the
 * touch_mutlcap_sensors_init API is called.
 */
touch_measure_data_t *p_mutlcap_measure_data = NULL;

surf_config_t surf_config =
{
    { DEF_SURF_NUM_CHANNELS,DEF_SURF_NUM_XLINES,DEF_SURF_NUM_YLINES,DEF_SURF_NUM_SLEEP_CHANNELS },
    { DEF_SURF_DPI_X,DEF_SURF_DPI_Y,DEF_SURF_TOT_RES_X,DEF_SURF_TOT_RES_Y,DEF_SURF_SENSOR_SIZE_IN_X,DEF_SURF_SENSOR_SIZE_IN_Y,0 },
    { DEF_SURF_TCH_DRIFT_PERIOD,DEF_SURF_ATCH_DRIFT_PERIOD,SURF_COMPLETE_CALLBACK },
    { DEF_SURF_MAX_TCH,DEF_SURF_DI,DEF_SURF_MAX_ON_DURATION },
    surf_seg_size,
    surf_data_blk,
    PRIV_SURF_DATA_BLK_SIZE
};

uint16_t            surface_timer_msec = DEF_TOUCH_MEASUREMENT_PERIOD_MS;
volatile uint8_t    surface_app_burst_again = 1u;
volatile uint8_t    qts_process_done = 0u;

/* ! QTouch Library Timing info. */
touch_time_t touch_time;
touch_time_t rtc_time;

uint16_t prev_rtc_period = SURF_ACTIVE_TCH_SCAN_RATE_MS;
uint16_t next_rtc_period = SURF_NO_TCH_SCAN_RATE_MS;

uint16_t surface_process_status = 0;

#if (DEF_SURF_LOW_POWER_SENSOR_ENABLE == 1)
/*Variables to track low power mode and drifting status(drift mode) during low power mode*/
uint8_t surface_op_mode = NORMAL_MODE;
uint8_t prev_mode = NORMAL_MODE;
uint8_t low_power_drift_pending = 0;
uint8_t low_power_state = NOT_RUNNING;
volatile uint16_t no_activity_counter = 0;
#endif /* DEF_SURF_LOW_POWER_SENSOR_ENABLE */


/*----------------------------------------------------------------------------
 *                           extern functions and variables
 *----------------------------------------------------------------------------*/
extern uint16_t rtc_timer_msec;
extern struct rtc_module rtc_instance;
extern void timer_init(void);
extern uint8_t sensor_state;


/*----------------------------------------------------------------------------
 *                               static variables
 *----------------------------------------------------------------------------*/

/**
 * Mutual Cap Data block provided as input to Touch library.
 */
static uint8_t mutlcap_data_blk[PRIV_MUTLCAP_DATA_BLK_SIZE];

/**
 * Mutual Cap Sensor Pins Info.
 */
#ifdef __ICCARM__
const uint16_t  mutlcap_xy_nodes[DEF_MUTLCAP_NUM_CHANNELS * 2]  @ "FLASH" = { DEF_MUTLCAP_NODES };
const gain_t mutlcap_gain_per_node[DEF_MUTLCAP_NUM_CHANNELS]  @ "FLASH" = { DEF_MUTLCAP_GAIN_PER_NODE };
uint8_t    sensor_detect_threshold[DEF_MUTLCAP_NUM_CHANNELS]  = { DEF_MUTLCAP_DETECT_THRESHOLD };
#endif

#ifdef __GNUC__
const uint16_t  mutlcap_xy_nodes[DEF_MUTLCAP_NUM_CHANNELS * 2]  = { DEF_MUTLCAP_NODES };
const gain_t mutlcap_gain_per_node[DEF_MUTLCAP_NUM_CHANNELS] = { DEF_MUTLCAP_GAIN_PER_NODE };
uint8_t sensor_detect_threshold[DEF_MUTLCAP_NUM_CHANNELS] = { DEF_MUTLCAP_DETECT_THRESHOLD };
#endif

freq_hop_sel_t mutlcap_freq_hops[3u] = { DEF_MUTLCAP_HOP_FREQS };

/**
 * Mutual Cap Configuration structure provided as input to Touch Library.
 *
 * Note: Use the touch.h configuration header file to fill in
 * the elements of this structure.  DO NOT modify any of the input values
 * directly in this structure.
 */
static touch_mutlcap_config_t mutlcap_config = {
    DEF_MUTLCAP_NUM_CHANNELS,               /* Mutual Cap number of channels. */
    DEF_MUTLCAP_NUM_SENSORS,                /* Mutual Cap number of sensors. */
    DEF_MUTLCAP_NUM_LUMPED_SENSORS,
    DEF_MUTLCAP_NUM_ROTORS_SLIDERS,         /* Mutual Cap number of rotors and sliders. */

    /* Mutual Cap GLOBAL SENSOR CONFIGURATION INFO. */
    {
        DEF_MUTLCAP_DI,                     /* Sensor detect integration (DI) limit. */
        /* Interchanging Negative and Positive Drift rate, since Signal
         * increases on Touch. */
        DEF_MUTLCAP_ATCH_DRIFT_RATE,        /* Sensor negative drift rate. */
        DEF_MUTLCAP_TCH_DRIFT_RATE,         /* Sensor positive drift rate. */
        DEF_MUTLCAP_MAX_ON_DURATION,        /* Sensor maximum on duration. */
        DEF_MUTLCAP_DRIFT_HOLD_TIME,        /* Sensor drift hold time. */
        DEF_MUTLCAP_ATCH_RECAL_DELAY,       /* Sensor positive recalibration delay. */
        DEF_MUTLCAP_CAL_SEQ1_COUNT,
        DEF_MUTLCAP_CAL_SEQ2_COUNT,
        DEF_MUTLCAP_ATCH_RECAL_THRESHOLD,   /*  Sensor recalibration threshold. */

        DEF_MUTLCAP_FREQ_AUTO_TUNE_SIGNAL_STABILITY_LIMIT,
        DEF_MUTLCAP_FREQ_AUTO_TUNE_IN_CNT,
        DEF_MUTLCAP_NOISE_MEAS_SIGNAL_STABILITY_LIMIT,  /* Signal stability */
        DEF_MUTLCAP_NOISE_LIMIT,
        DEF_MUTLCAP_LOCKOUT_SEL,
        DEF_MUTLCAP_LOCKOUT_CNTDOWN,
    },
    {
        mutlcap_gain_per_node,              /* Mutual Cap channel gain setting. */
        DEF_MUTLCAP_FREQ_MODE,              /* Mutual Cap noise counter measure enable/disable. */
        DEF_MUTLCAP_CLK_PRESCALE,           /* Mutual Capacitance PTC clock prescale */
        DEF_MUTLCAP_SENSE_RESISTOR,         /* Mutual Cap sense resistor value */
        DEF_MUTLCAP_CC_CAL_CLK_PRESCALE,
        DEF_MUTLCAP_CC_CAL_SENSE_RESISTOR,
        mutlcap_freq_hops,                  /* Mutual cap hopping freqencies */
        DEF_MUTLCAP_FILTER_LEVEL,           /* Mutual Cap filter level setting. */
        DEF_MUTLCAP_AUTO_OS,                /* Mutual Cap auto oversamples setting. */
    },
    mutlcap_data_blk,                       /* Mutual Cap data block index. */
    PRIV_MUTLCAP_DATA_BLK_SIZE,             /* Mutual Cap data block size. */
    mutlcap_xy_nodes,                       /* Mutual Cap channel nodes. */
    DEF_MUTLCAP_QUICK_REBURST_ENABLE,
    MODE_ALL,
    DEF_MUTLCAP_FILTER_CALLBACK,            /* Mutual Cap filter callback function pointer. */
    DEF_MUTLCAP_FREQ_AUTO_TUNE_ENABLE,
    DEF_MUTLCAP_NOISE_MEAS_ENABLE,
    DEF_MUTLCAP_NOISE_MEAS_BUFFER_CNT
};

/**
 * Touch Library input configuration structure.
 */
touch_config_t touch_config = {
    &mutlcap_config,                /* Pointer to Mutual Cap configuration structure. */
    DEF_TOUCH_PTC_ISR_LVL,          /* PTC interrupt level. */
};

/*----------------------------------------------------------------------------
 *                             Function Definitions
 *----------------------------------------------------------------------------*/

/*==============================================================================
 * Name    : qts_start
 *------------------------------------------------------------------------------
 * Purpose : Start calibration for Touch Surface
 * Input   : None
 * Output  : None
 * Notes   : None
 *==============================================================================*/
void qts_start(void)
{
    surf_ret_t surf_ret;
    surf_ret = surf_calibrate_all();
    if (surf_ret != SURF_SUCCESS)
    {
        while (1)
        {
        }
    }
}

/*==============================================================================
 * Name    : qts_init_surface
 *------------------------------------------------------------------------------
 * Purpose : Initialize the touch surface library
 * Input   : None
 * Output  : None
 * Notes   : None
 *==============================================================================*/
void qts_init_surface()
{
    surf_init(&surf_config, &surf_status, &touch_config);
    surf_status.ptr_surf_tch_status = &(surf_tch_status[0]);
}

/*==============================================================================
 * Name    : qts_normal_process
 *------------------------------------------------------------------------------
 * Purpose : To measure the Touch Surface by using surface library in normal mode.
 * Input   : None
 * Output  : None
 * Notes   : None
 *==============================================================================*/
void qts_normal_process(void)
{
    surf_ret_t surf_ret = SURF_SUCCESS;

    if (qts_process_done == 1)
    {
        if (surface_app_burst_again)
        {
            surface_timer_msec = SURF_ACTIVE_TCH_SCAN_RATE_MS;
        }
        else
        {
            surface_timer_msec = SURF_NO_TCH_SCAN_RATE_MS;
        }


        if ((touch_time.time_to_measure_touch == 1u))
        {
            next_rtc_period = surface_timer_msec;

            touch_time.time_to_measure_touch = 0u;
#if (DEF_SURF_LOW_POWER_SENSOR_ENABLE == 0)
            if (prev_rtc_period != next_rtc_period)
            {
                Disable_global_interrupt();
                rtc_timer_msec = next_rtc_period;
                enum status_code s1;
                s1 = rtc_count_set_count(&rtc_instance,0);
                if (s1 != STATUS_OK)
                {
                    while (1);
                }
                s1 = rtc_count_set_compare(&rtc_instance,next_rtc_period,RTC_COUNT_COMPARE_0);
                if (s1 != STATUS_OK)
                {
                    while (1);
                }
                rtc_count_enable(&rtc_instance);

                prev_rtc_period = next_rtc_period;
                Enable_global_interrupt();
            }
#endif /* DEF_SURF_LOW_POWER_SENSOR_ENABLE */
            surf_ret = surf_measure(touch_time.current_time_ms);
            if (surf_ret == SURF_SUCCESS)
            {
                qts_process_done = 0;
            }
            else
            {
                while (1)
                {
                    //error
                }
            }
        }
    }
}

/*==============================================================================
 * Name    : qts_process_lp
 *------------------------------------------------------------------------------
 * Purpose : To process surface library in low power mode using low power sensor.
 * Input   : None
 * Output  : None
 * Notes   : None
 *==============================================================================*/
#if (DEF_SURF_LOW_POWER_SENSOR_ENABLE == 1)
void qts_process_lp(void)
{
    surf_ret_t surf_ret = SURF_SUCCESS;

    if (!qts_process_done) {
        return ;
    }

    switch (surface_op_mode) {
        case DRIFT_MODE:
            if (!qts_process_done) {
                break;
            }
            if (low_power_state == RUNNING) {
                reset_evsys();
                surf_low_power_stop();
                low_power_state = NOT_RUNNING;
                next_rtc_period = DRIFT_PERIOD_MS;
                low_power_drift_pending = 0;
                qts_normal_process();
            }
            break;

        case NORMAL_MODE:
            if (!qts_process_done) {
                break;
            }
            low_power_drift_pending = 0;
            if (prev_mode == LOW_POWER_MODE)
            {
                low_power_state = NOT_RUNNING;
                reset_evsys();
                surf_ret = surf_low_power_stop();
                if (surf_ret != SURF_SUCCESS)
                {
                    while (1);
                }
            }
            qts_normal_process();
            break;

        case LOW_POWER_MODE:
            if (!qts_process_done) {
                break;
            }
            if (low_power_state != RUNNING) {
                low_power_state = RUNNING;
                config_timer_evsys();
                surf_ret = surf_low_power_start();
                if (surf_ret == SURF_SUCCESS) {
                    qts_process_done = 0;
                } else {
                    while (1);
                }
                next_rtc_period = DRIFT_PERIOD_MS;

                /*enable drift*/
                low_power_drift_pending = 1;
                surface_timer_msec = DRIFT_PERIOD_MS;
            }
            break;

        default:
            break;
    }

    if (prev_rtc_period != next_rtc_period) {
        rtc_timer_msec = next_rtc_period;
        enum status_code s1;
        s1 = rtc_count_set_count(&rtc_instance,0);
        if (s1 != STATUS_OK) {
            while (1);
        }
        s1 = rtc_count_set_compare(&rtc_instance,next_rtc_period,0);
        if (s1 != STATUS_OK) {
            while (1);
        }
        rtc_count_enable(&rtc_instance);
        prev_rtc_period = next_rtc_period;
    }
}
#endif /* DEF_SURF_LOW_POWER_SENSOR_ENABLE */

/*==============================================================================
 * Name    : surf_complete_callback
 *------------------------------------------------------------------------------
 * Purpose : Touch Surface Library will call this function once  measurement cycle is complete.
 * Input   : qt_surf_acq_status
 * Output  : None
 * Notes   : None
 *==============================================================================*/
uint8_t surf_complete_callback(uint16_t qt_surf_acq_status)
{

    p_mutlcap_measure_data->measurement_done_touch = 1u;
    qts_process_done = 1u;
    surface_process_status = qt_surf_acq_status;
    if (qt_surf_acq_status & TOUCH_BURST_AGAIN)
    {
        surface_app_burst_again = 1u;
    }
    else
    {
        surface_app_burst_again = 0u;
    }

#if (DEF_SURF_LOW_POWER_SENSOR_ENABLE == 1)
    prev_mode = surface_op_mode;
    if (prev_mode == DRIFT_MODE)
    {
        if (surface_app_burst_again)
        {
            surface_op_mode = NORMAL_MODE;
            no_activity_counter = 0;
        }
        else
        {
            surface_op_mode = LOW_POWER_MODE;
            no_activity_counter += rtc_timer_msec;

            //if previous mode is drift.then we should not wait for no activity time.
            //we should immediately go to low power mode.
        }
    }
    else if (prev_mode == LOW_POWER_MODE)
    {
        /*a callback could come only when the Low Power Sensor went into detect.*/
        surface_op_mode = NORMAL_MODE;
        no_activity_counter = 0;
    }
    else if (prev_mode == NORMAL_MODE)
    {
        if (surface_app_burst_again)
        {
            surface_op_mode = NORMAL_MODE;
            no_activity_counter = 0;
        }
        else
        {
            no_activity_counter += rtc_timer_msec;

            if (no_activity_counter >= NO_ACTIVITY_TRIGGER_TIME)
            {
                surface_op_mode=LOW_POWER_MODE;
            }
        }
    }
#endif

#if DEF_TOUCH_QDEBUG_ENABLE == 1
    /* Send out the Surface debug information data each time when Surface
     *   measurement process is completed .
     *   The Sensor Signal and Sensor Delta values are always sent.
     *   Surface Status change and Sensor Reference change
     *   values can be optionally sent using the masks below.
     */
    QDebug_SendData(TOUCH_CHANNEL_REF_CHANGE | TOUCH_STATUS_CHANGE);
    /* two-way QDebug communication  */
    /* Process any commands received from QTouch Analyser. */
    QDebug_ProcessCommands();
#endif

    return surface_app_burst_again;
}

/*==============================================================================
 * Name    : surface_configure_ptc_clock
 *------------------------------------------------------------------------------
 * Purpose : Configure the clocks required for PTC
 * Input   : None
 * Output  : None
 * Notes   : None
 *==============================================================================*/
void surface_configure_ptc_clock(void)
{
    struct system_gclk_chan_config gclk_chan_conf;
    typedef enum gclk_generator gclk_generator_t ;

    system_gclk_chan_get_config_defaults(&gclk_chan_conf);

    gclk_chan_conf.source_generator = (gclk_generator_t)GCLK_GEN_SRC_PTC;

    system_gclk_chan_set_config(PTC_GCLK_ID, &gclk_chan_conf);

    system_gclk_chan_enable(PTC_GCLK_ID);

    system_apb_clock_set_mask(SYSTEM_CLOCK_APB_APBC, PTC_APBC_BITMASK);
}

/*==============================================================================
 * Name    : qts_sensors_config
 *------------------------------------------------------------------------------
 * Purpose : Configure the different sensors for the Touch Surface
 * Input   : None
 * Output  : None
 * Notes   : None
 *==============================================================================*/
touch_ret_t qts_sensors_config(void)
{
    touch_ret_t touch_ret;

    /* configure the surface library sensors. */
    touch_ret = touch_sensors_config();
    if (touch_ret != TOUCH_SUCCESS) {
        while (1u) {    /* Check API Error return code. */
        }
    }

    /* Initialize the Qdebug  */
#if DEF_TOUCH_QDEBUG_ENABLE == 1
    QDebug_Init();
#endif

    return (touch_ret);
}

/*==============================================================================
 * Name    : touch_sensors_config
 *------------------------------------------------------------------------------
 * Purpose : Configure the different sensors based on Mutual Cap Technology
 * Input   : None
 * Output  : None
 * Notes   : None
 *==============================================================================*/
touch_ret_t touch_sensors_config(void)
{
    touch_ret_t touch_ret = TOUCH_SUCCESS;
    sensor_id_t sensor_id;

    for(uint16_t index = 0; index < DEF_SURF_NUM_IND_SENSORS; index++)
    {
        touch_ret = touch_mutlcap_sensor_config(SENSOR_TYPE_KEY, index, index, NO_AKS_GROUP, SURF_IND_SEN_DT,HYST_6_25, RES_8_BIT,0,&sensor_id);
        if (touch_ret != TOUCH_SUCCESS)
        {
            while (1) ;
        }
    }

    for(uint16_t index = DEF_SURF_NUM_IND_SENSORS; index < (DEF_SURF_NUM_IND_SENSORS + DEF_SURF_NUM_SLEEP_CHANNELS); index++)
    {
        touch_ret = touch_mutlcap_sensor_config(SENSOR_TYPE_KEY, index, index, NO_AKS_GROUP, SURF_SLEEP_CHANNELS_DT,HYST_6_25, RES_8_BIT,0,&sensor_id);
        if (touch_ret != TOUCH_SUCCESS)
        {
            while (1) ;
        }
    }

    for(uint16_t index = (DEF_SURF_NUM_IND_SENSORS + DEF_SURF_NUM_SLEEP_CHANNELS); index < (DEF_SURF_NUM_IND_SENSORS + DEF_SURF_NUM_SLEEP_CHANNELS + DEF_SURF_NUM_SEGMENTS); index++)
    {
        touch_ret = touch_mutlcap_sensor_config(SENSOR_TYPE_KEY, index, index, NO_AKS_GROUP, SURF_SEGMENTS_DT,HYST_6_25, RES_8_BIT,0,&sensor_id);
        if (touch_ret != TOUCH_SUCCESS)
        {
            while (1) ;
        }
    }

    for(uint16_t index = (DEF_SURF_NUM_IND_SENSORS + DEF_SURF_NUM_SLEEP_CHANNELS + DEF_SURF_NUM_SEGMENTS); index < (DEF_SURF_NUM_IND_SENSORS + DEF_SURF_NUM_SLEEP_CHANNELS + DEF_SURF_NUM_SEGMENTS+DEF_SURF_NUM_XLINES); index++)
    {
        touch_ret = touch_mutlcap_sensor_config(SENSOR_TYPE_KEY, index, index, NO_AKS_GROUP, SURF_ROW_LINE_DT,HYST_6_25, RES_8_BIT,0,&sensor_id);
        if (touch_ret != TOUCH_SUCCESS)
        {
            while (1) ;
        }
    }

    for(uint16_t index = (DEF_SURF_NUM_IND_SENSORS + DEF_SURF_NUM_SLEEP_CHANNELS + DEF_SURF_NUM_SEGMENTS+DEF_SURF_NUM_XLINES); index < DEF_SURF_NUM_CHANNELS; index++)
    {
        touch_ret = touch_mutlcap_sensor_config(SENSOR_TYPE_KEY, index, index, NO_AKS_GROUP, SURF_COL_LINE_DT,HYST_6_25, RES_8_BIT,0,&sensor_id);
        if (touch_ret != TOUCH_SUCCESS)
        {
            while (1) ;
        }
    }

    return (touch_ret);
}

