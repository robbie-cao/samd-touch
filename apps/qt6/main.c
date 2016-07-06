/* This source file is part of the ATMEL QTouch Surface Library 1.0.3 */

/*****************************************************************************
 *
 * \file
 *
 * \brief  This file contains the SAMD QTouch library sample user application.
 *
 *
 * - Userguide:          QTouch Surface Library User Guide.
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

/**
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
#include <asf.h>
#include "surface.h"
#include "touch.h"
#include "touch_api_SAMD.h"

#include "hal_if_usart.h"
#include "log.h"

/*----------------------------------------------------------------------------
 *                                   macros
 *----------------------------------------------------------------------------*/

/* Configure Power Saving Options
 *  And LOw POwer scan interval
 *
 */
/*
 * TO enable the Power optimization routines.
 *
 */

//#define POWER_OPT_ENABLE


#if (DEF_SURF_LOW_POWER_SENSOR_ENABLE == 1)

/* Event user  */
#define EVENT_USER              EVSYS_ID_USER_PTC_STCONV

#define NORMAL_MODE             0
#define LOW_POWER_MODE          1
#define DRIFT_MODE              2
#define RTC_TO_PTC_EVSYS_CH     1u
/*wakeup for smaller drift period*/
#if DEF_SURF_TCH_DRIFT_PERIOD <= DEF_SURF_ATCH_DRIFT_PERIOD
#define DRIFT_PERIOD_MS         (DEF_SURF_TCH_DRIFT_PERIOD * 200u)
#else
#define DRIFT_PERIOD_MS         (DEF_SURF_ATCH_DRIFT_PERIOD * 200u)
#endif

#define DEF_LOWPOWER_SENSOR_EVENT_PERIODICITY       LOWPOWER_PER7_SCAN_500_MS

#endif /* DEF_SURF_LOW_POWER_SENSOR_ENABLE */

/**
 * RTC Interrupt timing definition
 */
#define TIME_PERIOD_1MSEC       1u


/*----------------------------------------------------------------------------
 *                                 variables
 *----------------------------------------------------------------------------*/

extern touch_time_t     rtc_time;
extern volatile uint8_t qts_process_done;
extern uint16_t         surface_timer_msec;

struct rtc_module       surface_rtc_module;
volatile uint16_t       touch_time_counter = 0u;
uint16_t                rtc_timer_msec = DEF_TOUCH_MEASUREMENT_PERIOD_MS;
struct rtc_module       rtc_instance;

#if (DEF_SURF_LOW_POWER_SENSOR_ENABLE == 1)
extern uint8_t          surface_op_mode;
extern uint8_t          prev_mode;
extern uint8_t          low_power_drift_pending;

struct events_config    events_conf;
/* The event channel handle */
struct events_resource  events;

#endif /* DEF_SURF_LOW_POWER_SENSOR_ENABLE */


/*----------------------------------------------------------------------------
 *                                 prototypes
 *----------------------------------------------------------------------------*/

/**
 *! \brief Initialize timer
 */
void timer_init(void);

/*! \brief RTC timer Compare Callback
 *
 */
void rtc_compare_callback(void);

/*! \brief Configure the RTC timer callback
 */
void configure_rtc_callbacks(void);

/*! \brief to configure the timer for Qdebug
 */
void set_timer_period(uint16_t time);

#if (DEF_SURF_LOW_POWER_SENSOR_ENABLE == 1)
/*! \brief Configure the EVSYS for Low Power Mode */
void config_timer_evsys(void);

/*! \brief Reset the EVSYS using Software Reset.
 */
void reset_evsys(void);
#endif

/*! \brief Configure the RTC timer count after which interrupts comes
 */
void configure_rtc_count(void);

/*! \brief list of functions for power optimizations.
 */
/*! \brief Disable Unused Clocks to save power.
 * Note: Disabling Clocks which are used by the application or library might lead to
 * unexpected state of the system.
 */
void configure_power_manager(void);

/*! \brief Disable Unused Clocks to save power.
 */
void turn_off_bod33(void);

/*! \brief Disable the Cache.
 *  Note:Disabling Cache will increase the response time.
 *  Disabling Cache might reduce touch surface performance
 */
void disable_cache(void);


/*! \brief Extern functions
 */

/*! \brief Initialize the Touch Surface Library
 */
extern void qts_init_surface(void);

/*! \brief Configure the sensors for the Touch Surface
 */
extern void qts_sensors_config(void);

/*! \brief Start the calibration
 */
extern void qts_start(void);
/*! \brief Measure the Touch Surface in normal mode
 */
extern void qts_normal_process(void);
/*! \brief Configure the Touch Surface Library PTC Clock settings
 */
extern void surface_configure_ptc_clock(void);

#if (DEF_SURF_LOW_POWER_SENSOR_ENABLE == 1)
/*! \brief Measure the Touch Surface in low power mode
 * Note:Applicable only if DEF_SURF_NUM_SLEEP_CHANNELS is 1u.
 * Uses Low Power Sensor and Event System.
 */
extern void qts_process_lp(void);
void init_evsys_config(void);
#endif /* DEF_SURF_LOW_POWER_SENSOR_ENABLE */


/*----------------------------------------------------------------------------
 *                           function definitions
 *----------------------------------------------------------------------------*/
/*! \brief Configure the RTC timer compare callback
 */
void rtc_compare_callback(void)
{
    rtc_count_clear_compare_match(&rtc_instance,RTC_COUNT_COMPARE_0);
    rtc_time.current_time_ms += rtc_timer_msec;
#if (DEF_SURF_LOW_POWER_SENSOR_ENABLE == 1)
    if ((surface_op_mode == LOW_POWER_MODE) && low_power_drift_pending == 1)
    {
        touch_time.time_to_measure_touch = 1u;
        qts_process_done = 1;
        rtc_time.current_time_ms = 0;
        touch_time.current_time_ms = touch_time.current_time_ms + surface_timer_msec;
        surface_op_mode = DRIFT_MODE;
    }
#endif /* DEF_SURF_LOW_POWER_SENSOR_ENABLE */
    if (rtc_time.current_time_ms >= surface_timer_msec)
    {
        touch_time.time_to_measure_touch = 1u;
        rtc_time.current_time_ms = 0;
        touch_time.current_time_ms = touch_time.current_time_ms + surface_timer_msec;
    }
}

/*! \brief Configure the RTC timer callback
 */
void configure_rtc_callbacks(void)
{
    /* Register callback */
    rtc_count_register_callback(
            &rtc_instance,
            rtc_compare_callback,
            RTC_COUNT_CALLBACK_COMPARE_0
            );
    /* Enable callback */
    rtc_count_enable_callback(&rtc_instance,RTC_COUNT_CALLBACK_COMPARE_0);
}

/*! \brief Configure the RTC timer count after which interrupts comes
 */
void configure_rtc_count(void)
{
    //! [init_conf]
    struct rtc_count_config config_rtc_count;
    typedef enum rtc_count_prescaler rtc_count_prescaler_t;

#if DEF_SURF_LOW_POWER_SENSOR_ENABLE == 1
    struct rtc_count_events config_rtc_event =
        {
            .generate_event_on_periodic[DEF_LOWPOWER_SENSOR_EVENT_PERIODICITY] = true
        };
#endif

    rtc_count_get_config_defaults(&config_rtc_count);
    volatile uint16_t temp;
    //! [set_config]
    config_rtc_count.prescaler           = (rtc_count_prescaler_t)RTC_MODE0_CTRL_PRESCALER_DIV2;
    config_rtc_count.mode                = RTC_COUNT_MODE_32BIT;
#ifdef FEATURE_RTC_CONTINUOUSLY_UPDATED
    config_rtc_count.continuously_update = true;
#endif
    config_rtc_count.clear_on_match      = true;
    //! [init_rtc]
    rtc_count_init(&rtc_instance, RTC, &config_rtc_count);

#if DEF_SURF_LOW_POWER_SENSOR_ENABLE == 1
    /* Enable RTC events */
    config_rtc_event.generate_event_on_periodic[DEF_LOWPOWER_SENSOR_EVENT_PERIODICITY] = true;

    rtc_count_enable_events(&rtc_instance, &config_rtc_event);
#endif


    /*Set timer period */
    temp = TIME_PERIOD_1MSEC * rtc_timer_msec;
    rtc_count_set_compare(&rtc_instance, temp, RTC_COUNT_COMPARE_0);

    //! [enable]
    rtc_count_enable(&rtc_instance);
    //! [enable]
}

/*! \brief Initialize timer
 *
 */
void timer_init(void)
{
    /* Configure and enable RTC */
    configure_rtc_count();

    /* Configure and enable callback */
    configure_rtc_callbacks();
}

/**
 * \brief Function to turn of the BOD33 detector.
 * Note: This function turns the BOD33 off.
 */

void turn_off_bod33(void)
{
    SYSCTRL->BOD33.reg = 0;
}

/*
 * \brief Function to configure Power Manager (PM)
 *
 * This function configures the PM. The PM controls what synchronous
 * clocks clocks are running and not. This configuration of the PM is the one used in
 * the Power Consumption section in the data sheet. Some of the clocks in the clock masks
 * used here are disabled by default but have been added to give an easy overview of what
 * clocks are disabled and not.
 */
void configure_power_manager(void)
{
    system_apb_clock_clear_mask(
            SYSTEM_CLOCK_APB_APBA,
            (PM_APBAMASK_WDT  /*Watch dog timer*/
             | PM_APBAMASK_PAC0  /*Peripheral access controller for restricting register access on peripherals*/
             | PM_APBAMASK_EIC  /*external interrupt controller*/
             /*| PM_APBAMASK_GCLK*/  /* used by surface should not be disabled*/
             /* These clocks should remain enabled on this bus
                | PM_APBAMASK_SYSCTRL \
                | PM_APBAMASK_PM \
                | PM_APBAMASK_RTC \
                */
            ));

    system_apb_clock_clear_mask(
            SYSTEM_CLOCK_APB_APBB,
            (PM_APBBMASK_PAC1
             /* this clock is used in this project
                | PM_APBBMASK_PORT */

             /*| PM_APBBMASK_DSU*/   /*used for device identification, debugging etc*/
             | PM_APBBMASK_NVMCTRL
             /* These clocks should remain enabled on this bus
             */
            ));

    system_apb_clock_clear_mask(
            SYSTEM_CLOCK_APB_APBC,
            (PM_APBCMASK_ADC \
             | PM_APBCMASK_PAC2 \
             | PM_APBCMASK_DAC \
             | PM_APBCMASK_AC \
             | PM_APBCMASK_TC7 \
             | PM_APBCMASK_TC6 \
             | PM_APBCMASK_TC5 \
             | PM_APBCMASK_TC4 \
             | PM_APBCMASK_TC3
#if (SAMD20)
             | PM_APBCMASK_TC2
             | PM_APBCMASK_TC1
             | PM_APBCMASK_TC0
#elif (SAMD21)
             | PM_APBCMASK_TCC2
             | PM_APBCMASK_TCC1
             | PM_APBCMASK_TCC0
#endif
             | PM_APBCMASK_SERCOM5 \
             | PM_APBCMASK_SERCOM4 \
             | PM_APBCMASK_SERCOM3 \
             | PM_APBCMASK_SERCOM2 \
             | PM_APBCMASK_SERCOM1 \
             | PM_APBCMASK_SERCOM0
#if (DEF_SURF_LOW_POWER_SENSOR_ENABLE == 0)
             | PM_APBCMASK_EVSYS
#endif
             ));


    system_apb_clock_set_divider(
            SYSTEM_CLOCK_APB_APBA,
            SYSTEM_MAIN_CLOCK_DIV_4
            );
}

/*! \brief Disable the Cache..
 *  Note:Disabling Cache will increase the response time
 *  Disabling Cache might reduce touch surface performance
 */
void disable_cache(void)
{
    NVMCTRL->CTRLB.bit.CACHEDIS = 1;
    //Readmode settings : 0-3
    NVMCTRL->CTRLB.bit.READMODE = 0;
}


#if (DEF_SURF_LOW_POWER_SENSOR_ENABLE == 1)
/*! \brief Configure the Event System for Low Power measurements
 * Note:Applicable only when DEF_SURF_LOW_POWER_SENSOR_ENABLE macro is defined as 1
 * and DEF_SURF_NUM_SLEEP_CHANNELS is 1u.
 */
void config_timer_evsys(void)
{
    PM_APBCMASK_Type evsys_enable_mask;
    evsys_enable_mask.reg = (1u << PM_APBCMASK_EVSYS_Pos);
    PM->APBCMASK.reg |= evsys_enable_mask.reg;
    enum status_code status_ret = STATUS_OK;
    status_ret = events_attach_user(&events, EVENT_USER);

    if (status_ret != STATUS_OK) {
        while (1) {
        }
    }

    touch_time.time_to_measure_touch = 1;


}
/*! \brief Software Reset the Event System.
*/
void reset_evsys(void)
{
    enum status_code status_ret=STATUS_OK;
    status_ret=events_detach_user(&events, EVENT_USER);

    if (status_ret!=STATUS_OK)
    {
        while (1) {
        }
    }

}

void init_evsys_config(void)
{
    PM_APBCMASK_Type evsys_enable_mask;

    evsys_enable_mask.reg = (1u << PM_APBCMASK_EVSYS_Pos);

    PM->APBCMASK.reg |= evsys_enable_mask.reg;


    enum status_code status_ret = STATUS_OK;

    /* Get default event channel configuration */
    events_get_config_defaults(&events_conf);

    events_conf.path           = EVENTS_PATH_ASYNCHRONOUS;
    events_conf.generator      = 4 + DEF_LOWPOWER_SENSOR_EVENT_PERIODICITY;

    status_ret = events_allocate(&events, &events_conf);

    if (status_ret != STATUS_OK)
    {
        while (1) {
        }
    }

}

#endif
/*! \brief Set timer period.Called from Qdebug .
*/
void set_timer_period(uint16_t time)
{
    rtc_count_set_compare(&rtc_instance,time,RTC_COUNT_COMPARE_0);

    //! [enable]
    rtc_count_enable(&rtc_instance);
}

int main(void)
{

    /**
     * Initialize and configure system and generic clocks.
     * Use conf_clocks.h to configure system and generic clocks.
     */
    system_init();

    hal_if_usart_init();

    /**
     * Enable global interrupts.
     */
    system_interrupt_enable_global();

    /**
     * Initialize delay service.
     */
    delay_init();

    /* initialize timer */
    timer_init();

    /* Setup and enable generic clock source for PTC module. */
    surface_configure_ptc_clock();

    touch_time.measurement_period_ms = DEF_TOUCH_MEASUREMENT_PERIOD_MS;


    /* Initialize touchpad input parameters */
    qts_init_surface();

    qts_sensors_config();

    /*initialize event system*/
#if (DEF_SURF_LOW_POWER_SENSOR_ENABLE == 1)
    init_evsys_config();
#endif

    /* Configure System Sleep mode to STANDBY. */

    system_set_sleepmode(SYSTEM_SLEEPMODE_STANDBY);
#ifdef POWER_OPT_ENABLE
    turn_off_bod33();
    configure_power_manager();

#endif

    /* Calibration */
    qts_start();

    LOG("Hello QT6~\r\n");

    /* Appl maintains this flag,
     * marked as 1 initially to start measurement cycle.
     */
    qts_process_done = 1u;

    /* Appl maintains this flag,
     * marked as 1 initially to start measurement cycle.
     */
    touch_time.time_to_measure_touch = 1u;

    while (1) {
        /**
         * Start touch surface process
         */

#if DEF_SURF_LOW_POWER_SENSOR_ENABLE == 1
        qts_process_lp();
#else
        if (qts_process_done == 1) {
            qts_normal_process();
        }
#endif
        if (p_mutlcap_measure_data->measurement_done_touch == 1u) {
            p_mutlcap_measure_data->measurement_done_touch = 0u;
        }

        system_sleep();
    }
}

