/* This header file is part of the ATMEL QTouch Surface Library 1.0.3 */

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

#ifndef TOUCH_H
#define TOUCH_H
/*----------------------------------------------------------------------------
 *                   Mutual Cap method enable/disable.
 *----------------------------------------------------------------------------*/
/**
 * Enable Mutual Capacitance method.
 */
#define DEF_TOUCH_MUTLCAP               (1)
/*----------------------------------------------------------------------------
 *                   PTC module clock and interrupt level configuration.
 *----------------------------------------------------------------------------*/

/**
 * PTC Module clock configuration.
 * Before using the QTouch Surface library API, the PTC module clock generator source
 * should be configured appropriately.  The PTC module clock can be generated
 * using any of the eight generic clock generators (GCLK0-GCLK7).  The associated
 * generic clock multiplexer should be configured such that the PTC module clock
 * is set to 4MHz.  Refer surface_configure_ptc_clock API in surface.c for more
 * information.
 * Select the generic clock generator for surface measurements
 */
#define GCLK_GEN_SRC_PTC                1
/**
 * PTC Module interrupt level.
 * The Nested Vectored Interrupt Controller (NVIC) in the SAMD supports
 * four different priority levels.  The priority level of the PTC end of
 * conversion ISR, used within QTouch Surface library can be chosen based on application
 * requirements in order to accommodate time critical operations.
 * Range: 0u (Highest priority) to 3u (Lowest priority)
 * For more details refer to the Cortex-M0 Technical Reference Manual.
 */
#define DEF_TOUCH_PTC_ISR_LVL           (3u)

/*----------------------------------------------------------------------------
 *                   Mutual Cap method pin configuration.
 *----------------------------------------------------------------------------*/
/**
 * Mutual Cap method touch channel nodes (GPIO pins) selected.
 * Mutual capacitance method use a pair of sensing electrodes for each touch
 * channel.  These are denoted as X and Y lines.  Touch channel numbering
 * follows the order in which X-Y nodes are specified.  Capacitance measurement
 * is done sequentially in the order in which touch channel nodes are specified.
 */
#define LUMP_ALL_X      LUMP_X(2,3,8,9,12,13,14,15,5,1)
#define LUMP_ALL_Y      LUMP_Y(6,7,12,13,10,11,3,4,2,5)
#define LUMP_1_HALF_X   LUMP_X(2,3,8,9,12)
#define LUMP_2_HALF_X   LUMP_X(13,14,15,5,1)

#define DEF_MUTLCAP_NODES       \
    X(2) , Y(6)  , X(3) , Y(6)  , X(8) , Y(6)  , X(9) , Y(6)  , X(12) , Y(6)  , X(13) , Y(6)  , X(14) , Y(6)  , X(15) , Y(6)  , X(5) , Y(6)  , X(1) , Y(6)  , \
    X(2) , Y(7)  , X(3) , Y(7)  , X(8) , Y(7)  , X(9) , Y(7)  , X(12) , Y(7)  , X(13) , Y(7)  , X(14) , Y(7)  , X(15) , Y(7)  , X(5) , Y(7)  , X(1) , Y(7)  , \
    X(2) , Y(12) , X(3) , Y(12) , X(8) , Y(12) , X(9) , Y(12) , X(12) , Y(12) , X(13) , Y(12) , X(14) , Y(12) , X(15) , Y(12) , X(5) , Y(12) , X(1) , Y(12) , \
    X(2) , Y(13) , X(3) , Y(13) , X(8) , Y(13) , X(9) , Y(13) , X(12) , Y(13) , X(13) , Y(13) , X(14) , Y(13) , X(15) , Y(13) , X(5) , Y(13) , X(1) , Y(13) , \
    X(2) , Y(10) , X(3) , Y(10) , X(8) , Y(10) , X(9) , Y(10) , X(12) , Y(10) , X(13) , Y(10) , X(14) , Y(10) , X(15) , Y(10) , X(5) , Y(10) , X(1) , Y(10) , \
    X(2) , Y(11) , X(3) , Y(11) , X(8) , Y(11) , X(9) , Y(11) , X(12) , Y(11) , X(13) , Y(11) , X(14) , Y(11) , X(15) , Y(11) , X(5) , Y(11) , X(1) , Y(11) , \
    X(2) , Y(3)  , X(3) , Y(3)  , X(8) , Y(3)  , X(9) , Y(3)  , X(12) , Y(3)  , X(13) , Y(3)  , X(14) , Y(3)  , X(15) , Y(3)  , X(5) , Y(3)  , X(1) , Y(3)  , \
    X(2) , Y(4)  , X(3) , Y(4)  , X(8) , Y(4)  , X(9) , Y(4)  , X(12) , Y(4)  , X(13) , Y(4)  , X(14) , Y(4)  , X(15) , Y(4)  , X(5) , Y(4)  , X(1) , Y(4)  , \
    X(2) , Y(2)  , X(3) , Y(2)  , X(8) , Y(2)  , X(9) , Y(2)  , X(12) , Y(2)  , X(13) , Y(2)  , X(14) , Y(2)  , X(15) , Y(2)  , X(5) , Y(2)  , X(1) , Y(2)  , \
    X(2) , Y(5)  , X(3) , Y(5)  , X(8) , Y(5)  , X(9) , Y(5)  , X(12) , Y(5)  , X(13) , Y(5)  , X(14) , Y(5)  , X(15) , Y(5)  , X(5) , Y(5)  , X(1) , Y(5)  , \
    LUMP_ALL_X,LUMP_Y(6,7,12),LUMP_ALL_X,LUMP_Y(13,10,11),LUMP_ALL_X,LUMP_Y(3,4,2),LUMP_ALL_X,LUMP_Y(5),                                                                                                                             \
    LUMP_1_HALF_X,LUMP_Y(6,7,12),LUMP_2_HALF_X,LUMP_Y(6,7,12),LUMP_1_HALF_X,LUMP_Y(13,10,11),LUMP_2_HALF_X,LUMP_Y(13,10,11),LUMP_1_HALF_X,LUMP_Y(3,4,2),LUMP_2_HALF_X,LUMP_Y(3,4,2),LUMP_1_HALF_X,LUMP_Y(5),LUMP_2_HALF_X,LUMP_Y(5), \
    LUMP_ALL_X,LUMP_Y(6),LUMP_ALL_X,LUMP_Y(7),LUMP_ALL_X,LUMP_Y(12),LUMP_ALL_X,LUMP_Y(13),LUMP_ALL_X,LUMP_Y(10),LUMP_ALL_X,LUMP_Y(11),LUMP_ALL_X,LUMP_Y(3),LUMP_ALL_X,LUMP_Y(4),LUMP_ALL_X,LUMP_Y(2),LUMP_ALL_X,LUMP_Y(5),           \
    LUMP_X(2),LUMP_ALL_Y,LUMP_X(3),LUMP_ALL_Y,LUMP_X(8),LUMP_ALL_Y,LUMP_X(9),LUMP_ALL_Y,LUMP_X(12),LUMP_ALL_Y,LUMP_X(13),LUMP_ALL_Y,LUMP_X(14),LUMP_ALL_Y,LUMP_X(15),LUMP_ALL_Y,LUMP_X(5),LUMP_ALL_Y,LUMP_X(1),LUMP_ALL_Y


/*----------------------------------------------------------------------------
 *                   Mutual Cap method channel and sensor configuration.
 *----------------------------------------------------------------------------*/

/**
 * Mutual Cap number of channels.
 * Specify the number of Mutual Cap touch channels to be used by the Touch
 * Library. A key is formed used one touch channel.
 */
#define DEF_MUTLCAP_NUM_CHANNELS        (132)

/**
 * Mutual Cap number of Sensors.
 * Specify the number of Mutual Cap touch sensors to be used by the QTouch Surface
 * Library.
 */
#define DEF_MUTLCAP_NUM_SENSORS         (132)

/**
 * Detect threshold setting for Sleep Channel.
 */
#define SURF_SLEEP_CHANNELS_DT          (20)
/**
 * Detect threshold setting for Segment Sensor.
 */
#define SURF_SEGMENTS_DT                (20)
/**
 * Detect threshold setting for Row Sensors.
 */
#define SURF_ROW_LINE_DT                (20)

/**
 * Detect threshold setting for Column Sensors.
 */
 #define SURF_COL_LINE_DT               (20)
/**
 * Detect threshold setting Individual Sensor.
 */
#define SURF_IND_SEN_DT                 (20)



/*----------------------------------------------------------------------------
 *                   Mutual Cap method acquisition parameters.
 *----------------------------------------------------------------------------*/
/* ! @} */
/**
 * Mutual Cap filter level setting.
 * The filter level setting controls the number of samples taken
 * to resolve each acquisition. A higher filter level setting provides
 * improved signal to noise ratio under noisy conditions, while
 * increasing the total time for measurement resulting in increased
 * power consumption.  Refer filter_level_t in touch_api_SAMD.h
 * Range: FILTER_LEVEL_1 (one sample) to FILTER_LEVEL_64 ( 64 samples).
 */
#define DEF_MUTLCAP_FILTER_LEVEL        FILTER_LEVEL_8      // Filter level

/**
 * Mutual Cap auto oversample setting.
 * Auto oversample controls the automatic oversampling of sensor channels when
 * unstable signals are detected with the default setting of ?Filter level?.
 * Enabling Auto oversample results in 'Filter level' x 'Auto Oversample' number
 * of samples taken on the corresponding sensor channel when an unstable signal
 * is observed.  In a case where ?Filter level? is set to FILTER_LEVEL_4 and
 * ?Auto Oversample? is set to AUTO_OS_4, 4 oversamples are taken with stable
 * signal values and 16 oversamples are taken when unstable signal is detected.
 * Refer auto_os_t in touch_api_SAMD.h
 * Range: AUTO_OS_DISABLE (oversample disabled) to AUTO_OS_128 (128
 * oversamples).
 */
#define DEF_MUTLCAP_AUTO_OS             0                   // Automatic OverSampling

/**
 * Mutual Cap gain per touch channel.
 * Gain is applied on a per-channel basis to allow a scaling-up of the touch
 * sensitivity on contact.
 * Note: delta on touch contact, not the resting signal which is measured on
 * each sensor.
 * Refer gain_t in touch_api_SAMD.h
 * Range:GAIN_1 (no scaling) to GAIN_32 (scale-up by 32)
 */
#define DEF_MUTLCAP_GAIN_PER_NODE       \
    GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , \
    GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , \
    GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , \
    GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , \
    GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , \
    GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , \
    GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , \
    GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , \
    GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , \
    GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , \
    GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , \
    GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , \
    GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , \
    GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , \
    GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , \
    GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4 , \
    GAIN_4 , GAIN_4 , GAIN_4 , GAIN_4


#define DEF_MUTLCAP_DETECT_THRESHOLD    \
    SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , \
    SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , \
    SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , \
    SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , \
    SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , \
    SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , \
    SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , \
    SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , \
    SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , \
    SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , \
    SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , \
    SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , \
    SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , \
    SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , \
    SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , \
    SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , \
    SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , \
    SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , \
    SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , \
    SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , \
    SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , \
    SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , \
    SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , \
    SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , \
    SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , SURF_IND_SEN_DT , \
    SURF_SLEEP_CHANNELS_DT , SURF_SLEEP_CHANNELS_DT , SURF_SLEEP_CHANNELS_DT , SURF_SLEEP_CHANNELS_DT , \
    SURF_SEGMENTS_DT , SURF_SEGMENTS_DT , SURF_SEGMENTS_DT , SURF_SEGMENTS_DT , SURF_SEGMENTS_DT , SURF_SEGMENTS_DT , SURF_SEGMENTS_DT , SURF_SEGMENTS_DT , \
    SURF_ROW_LINE_DT , SURF_ROW_LINE_DT , SURF_ROW_LINE_DT , SURF_ROW_LINE_DT , SURF_ROW_LINE_DT , SURF_ROW_LINE_DT , SURF_ROW_LINE_DT , SURF_ROW_LINE_DT , SURF_ROW_LINE_DT , SURF_ROW_LINE_DT , \
    SURF_COL_LINE_DT , SURF_COL_LINE_DT , SURF_COL_LINE_DT , SURF_COL_LINE_DT , SURF_COL_LINE_DT , SURF_COL_LINE_DT , SURF_COL_LINE_DT , SURF_COL_LINE_DT , SURF_COL_LINE_DT , SURF_COL_LINE_DT


/* ! @} */
/*----------------------------------------------------------------------------
 *   Tuning for Noise performance, surface response time and Power consumption.
 *
 *----------------------------------------------------------------------------*/
/* ! @} */
/*! \name Mutual Cap Global acquisition parameters.
 * Refer the Touch Library User guide for more information on these parameters.
 */
/**
 * Refer QTouch Library Peripheral Touch Controller User Guide for detailed
 * information on tuning for Noise performance, touch response time and  Power
 * consumption
 */

/**
 * For best noise performance, set -
 * - DEF_MUTLCAP_FREQ_MODE to FREQ_MODE_HOP
 * - DEF_MUTLCAP_SENSE_RESISTOR to RSEL_VAL_100
 * Based on the type of noise, FREQ_MODE_SPREAD can also be used.
 *
 * For best power consumption, set -
 * - DEF_MUTLCAP_FREQ_MODE to FREQ_MODE_NONE
 * - DEF_MUTLCAP_CLK_PRESCALE to PRSC_DIV_SEL_1
 */

/**
 * Mutual Cap acquisition frequency mode.
 *
 * FREQ_MODE_HOP:
 * When frequency mode hopping option is selected, the PTC runs a
 * frequency hopping cycle with subsequent measurements done using
 * the three PTC acquisition frequency delay settings as specified in
 * DEF_MUTLCAP_HOP_FREQS.
 *
 * FREQ_MODE_SPREAD:
 * When frequency mode spread spectrum option is selected, the PTC
 * runs with spread spectrum enabled for jittered delay based
 * acquisition.
 *
 * FREQ_MODE_NONE:
 * When frequency mode none option is selected, the PTC runs at
 * constant speed.  This mode is suited for best power consumption.
 */
#define DEF_MUTLCAP_FREQ_MODE           2
/**
 * PTC acquisition frequency delay setting.
 *
 * Specify three frequency hop delay settings.
 *
 * The PTC acquisition frequency is dependent on the Generic clock
 * input to PTC and PTC clock pre-scaler setting.  This delay setting
 * inserts "n" PTC clock cycles between consecutive measurements on
 * a given sensor, thereby changing the PTC acquisition frequency.
 * FREQ_HOP_SEL_1 setting inserts 0 PTC clock cycle between consecutive
 * measurements.  FREQ_HOP_SEL_16 setting inserts 15 PTC clock cycles.
 * Hence, higher delay setting will increase the total time taken for
 * capacitance measurement on a given sensor as compared to a lower
 * delay setting.
 *
 * A desired setting can be used to avoid noise around the same frequency
 * as the acquisition frequency.
 * Range: FREQ_HOP_SEL_1 to FREQ_HOP_SEL_16
 */
#define DEF_MUTLCAP_HOP_FREQS           FREQ_HOP_SEL_1,FREQ_HOP_SEL_2,FREQ_HOP_SEL_3

/**
 * Mutual cap PTC clock pre-scalar.
 * PTC clock prescale setting.   Refer surface_configure_ptc_clock() API in
 * surface.c
 * Example: if Generic clock input to PTC = 4MHz, then:
 * PRSC_DIV_SEL_1   sets PTC Clock to  4MHz
 * PRSC_DIV_SEL_2   sets PTC Clock to  2MHz
 * PRSC_DIV_SEL_4   sets PTC Clock to 1MHz
 * PRSC_DIV_SEL_8   sets PTC Clock to 500KHz
 */
#define DEF_MUTLCAP_CLK_PRESCALE        PRSC_DIV_SEL_1

/**
 * PTC series resistor setting.  For Mutual cap mode, this series
 * resistor is switched internally on the Y-pin.
 * Example:
 * RSEL_VAL_0   sets internal series resistor to 0ohms.
 * RSEL_VAL_20  sets internal series resistor to 20Kohms.
 * RSEL_VAL_50  sets internal series resistor to 50Kohms.
 * RSEL_VAL_100 sets internal series resistor to 100Kohms.
 */
#define DEF_MUTLCAP_SENSE_RESISTOR      RSEL_VAL_0
/**
 * Mutual cap PTC clock pre-scalar for calibration.
 * PTC clock prescale setting.   Refer surface_configure_ptc_clock() API in
 * surface.c
 * Example: if Generic clock input to PTC = 4MHz, then:
 * PRSC_DIV_SEL_1   sets PTC Clock to  4MHz
 * PRSC_DIV_SEL_2   sets PTC Clock to  2MHz
 * PRSC_DIV_SEL_4   sets PTC Clock to 1MHz
 * PRSC_DIV_SEL_8   sets PTC Clock to 500KHz
 * Default : The same prescaler used for normal measurement  ie DEF_MUTLCAP_CLK_PRESCALE.
 */
#define DEF_MUTLCAP_CC_CAL_CLK_PRESCALE         DEF_MUTLCAP_CLK_PRESCALE
/**
 * PTC series resistor setting for calibration.  For Mutual cap mode, this series
 * resistor is switched internally on the Y-pin.
 * Example:
 * RSEL_VAL_0   sets internal series resistor to 0ohms.
 * RSEL_VAL_20  sets internal series resistor to 20Kohms.
 * RSEL_VAL_50  sets internal series resistor to 50Kohms.
 * RSEL_VAL_100 sets internal series resistor to 100Kohms.
 * Default : The same resistor value  used for normal measurement  ie DEF_MUTLCAP_SENSE_RESISTOR.
 */
#define DEF_MUTLCAP_CC_CAL_SENSE_RESISTOR       DEF_MUTLCAP_SENSE_RESISTOR
/**
 * Mutual Cap calibration seq count 1 setting.
 * calibration sequence count 1 setting allows users to configure the total
 * number of calibration adjustment bursting after hardware calibration is
 * completed. This count should be always larger than the Mutual cap calibration
 * seq count 2 (DEF_MUTLCAP_CAL_SEQ2_COUNT).
 *
 * Range:5 to 255
 */
#define DEF_MUTLCAP_CAL_SEQ1_COUNT	    10
/**
 * Mutual Cap calibration seq count 2 setting.
 * calibration sequence count 2 setting allows users to configure the number of
 * bursting after hardware calibration where signal is copied to the reference.
 * This count should be always be smaller than the Mutual cap calibration
 * seq count 1 (DEF_MUTLCAP_CAL_SEQ1_COUNT).
 *
 * Range:4 to 254
 */
#define DEF_MUTLCAP_CAL_SEQ2_COUNT	    5
/* ! @} */
/*----------------------------------------------------------------------------
 *                   Mutual Cap method sensor global parameters.
 *
 *
 *----------------------------------------------------------------------------*/

/*! \name Mutual Cap Global acquisition parameters.
 * Refer the Touch Library User guide for more information on these parameters.
 */
/* ! @{ */
/**
 * Mutual Cap Sensor maximum ON time duration.
 * Units: 200ms (Example: a value 5u indicated Max ON duration of 1 second.)
 * Default value: 0 (No maximum ON time limit).
 * Range: 0u to 255u.
 */
#define DEF_MUTLCAP_MAX_ON_DURATION     (0)
/**
 * Mutual Cap Sensor away from touch recalibration delay.
 * Default value: 10.
 * Range: 1u to 255u.
 */
#define DEF_MUTLCAP_ATCH_RECAL_DELAY    10u


/** Mutual Cap Sensor away from touch recalibration threshold.
 * Default: RECAL_50 (recalibration threshold = 50% of detection threshold).
 * Range: refer recal_threshold_t enum in touch_api_SAMD.h.
 */
#define DEF_MUTLCAP_ATCH_RECAL_THRESHOLD        0u


/* ! @} */

/*-----------------------------------------------------------------------------
 *                   Mutual Cap method noise measurement & lockout.
 *----------------------------------------------------------------------------*/
/* ! @{ */

/**
 * Noise measurement enable/disable
 * If configured as 1, noise measurement will be enabled
 * If configured as 0, noise measurement will be disabled
 */
#define DEF_MUTLCAP_NOISE_MEAS_ENABLE           (0u)

/**
 * Defines the stability limit of signals for noise calculation
 * Range: 1 to 1000
 */
#define DEF_MUTLCAP_NOISE_MEAS_SIGNAL_STABILITY_LIMIT   10u

/**
 * Noise limit
 * If the channel signal is noisy for a count value higher than the max count
 *defined by user,
 * system trigger Lockout functionality
 * Range: 1 to 255
 */
#define DEF_MUTLCAP_NOISE_LIMIT                 12u

/**
 * Noise buffer count
 * Selection of buffer count for noise calculation.
 * Defines the buffer limit for internal noise measurement
 * Range: 3 to 10   (select value N + 1, here N number of  samples)
 * if N = 4 then set  DEF_NM_BUFFER_CNT  5u ->> (N + 1).
 * Default : 5u
 */
#define DEF_MUTLCAP_NOISE_MEAS_BUFFER_CNT       (5u)

/**
 * Mutual cap method : noisy Sensor lockout settings.
 * 0u: single sensor lockout.
 * 1u: Global sensor lockout.
 * 2u : No lockout
 * Range : 0 to 2
 */
#define DEF_MUTLCAP_LOCKOUT_SEL                 0

/**
 * Mutual cap Lockout count down
 * If the sensor signal is moves from noisy to a good condition and stays there
 * for a count value higher than the max count defined by user, sensor is
 * declared as stable
 * Range: 1 to 255
 */
#define DEF_MUTLCAP_LOCKOUT_CNTDOWN             10
/*! \name Mutual Cap Filter Callback functions.
 */
/**
 * Mutual Cap Filter callback function.
 * A filter callback (when not NULL) is called by the Touch Library each time
 * a new set of Signal values are available.
 * An Example filter callback function prototype.
 * void touch_filter_callback( touch_filter_data_t *p_filter_data );
 */
#define DEF_MUTLCAP_FILTER_CALLBACK             (NULL)

/* ! @} */

/*----------------------------------------------------------------------------
 *                   Mutual Cap method Frequency auto tune.
 *----------------------------------------------------------------------------*/

/*! \name Mutual Cap Global acquisition parameters.
 * Refer the Touch Library User guide for more information on these parameters.
 */
/* ! @{ */

/**
 * Freq auto tune enable/disable ( applicable only to freq_hop mode)
 * If configured as 1, Freq auto tune will be enabled
 * If configured as 0, Freq auto tune will be disabled
 */
#define DEF_MUTLCAP_FREQ_AUTO_TUNE_ENABLE       (0u)

/**
 * Defines the stability limit of signals for Freq auto tune calculation
 * Range: 1 to 1000
 * Note : this applies only for FREQ_MODE_HOP
 */
#define DEF_MUTLCAP_FREQ_AUTO_TUNE_SIGNAL_STABILITY_LIMIT           10u

/**
 * Frequency Auto tune-in count
 * If the channel signal is noisy for a count value higher than the max count
 *defined by user,
 * system will trigger auto tune
 * Range: 1 to 255
 * Note : this applies only for FREQ_MODE_HOP
 */
#define DEF_MUTLCAP_FREQ_AUTO_TUNE_IN_CNT       12

/* ! @} */
/*----------------------------------------------------------------------------
*                   QDebug debug communication parameters.
*  ----------------------------------------------------------------------------*/

/*! \name QDebug debug communication parameters.
 */
/* ! @{ */


#define DEF_TOUCH_QDEBUG_ENABLE_MUTLCAP
#define DEF_TOUCH_QDEBUG_ENABLE_SURFACE
#define DEF_TOUCH_QDEBUG_ENABLE         1

#define INTERFACE    SPI2W

#define QDEBUG_BITBANG_SPI

#define QDEBUG_SPI_BB_SS_PIN        31
#define QDEBUG_SPI_BB_SCK_PIN       23
#define QDEBUG_SPI_BB_MOSI_PIN      22
#define QDEBUG_SPI_BB_MISO_PIN      16

#define QDEBUG_SPI_BB_SS_PORT       B
#define QDEBUG_SPI_BB_SCK_PORT      B
#define QDEBUG_SPI_BB_MOSI_PORT     B
#define QDEBUG_SPI_BB_MISO_PORT     B


#define PROJECT_ID  3


/* ! @{ */
/*---------------------------------------------------------------------------------------------
*               Do not Modify. Used by Touch Surface Beta Library for internal purpose
*  --------------------------------------------------------------------------------------------*/
/* ! @{ */

/**
 * Mutual Cap number of Rotors and Sliders.
 * Specify the total number of Mutual Cap Rotors and Sliders to be used by
 * the Touch Library.  The number of Rotors and Sliders mentioned here is part
 * of the Total number of sensors specified in the QT_NUM_SENSORS macro.  When
 * no rotors or slider are required, specify a value of 0u.
 * Range: 0u to 8u.
 */
#define DEF_MUTLCAP_NUM_ROTORS_SLIDERS      (0)  /* Number of rotor sliders */
/**
This is to enable the Quick Re burst feature
*/
#define DEF_MUTLCAP_QUICK_REBURST_ENABLE    0
/**
 * Mutual Cap Sensor detect integration (DI) limit.
 * Range: 0u to 255u.
 */
#define DEF_MUTLCAP_DI              0u

/**
 * Mutual Cap Sensor towards touch drift rate.
 * Units: 200ms
 * Default value: 20 = 4 seconds.
 * Range: 1u to 127u.
 */
#define DEF_MUTLCAP_TCH_DRIFT_RATE  20u

/**
 * Mutual Cap Sensor away from touch drift rate.
 * Units: 200ms
 * Default value: 5u = 1 second.
 * Range: 1u to 127u.
 */
#define DEF_MUTLCAP_ATCH_DRIFT_RATE         5u
/**
 * Mutual Cap Sensor measurement interval.
 * Specify period in milliseconds.  Example, DEF_TOUCH_MEASUREMENT_PERIOD_MS
 *50u
 * will perform measurement on touch sensors every 50msec.
 */
#define DEF_TOUCH_MEASUREMENT_PERIOD_MS SURF_ACTIVE_TCH_SCAN_RATE_MS
/**
 * Mutual Cap Sensor drift hold time.
 * Units: 200ms
 * Default value: 20 (hold off drifting for 4 seconds after leaving detect).
 * Range: 1u to 255u.
 */
#define DEF_MUTLCAP_DRIFT_HOLD_TIME         20u
/**
 * Disable Self Capacitance method.
 */
#define DEF_TOUCH_SELFCAP       (0u)
/* ! @} */

#endif /* TOUCH_H */
