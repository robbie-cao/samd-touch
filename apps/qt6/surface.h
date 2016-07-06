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
#ifndef SURFACE_H
#define SURFACE_H

/*----------------------------------------------------------------------------
 *                  Surface H/W Configuration
 *----------------------------------------------------------------------------*/
/**
 * Details about the Touch Surface Hardware should be configured here.
 */
/* ! @} */
/**
 * Surface Number of  channels.
 * Specify the number of Surface Cap to be used by the Touch Surface
 * Library.
 */
#define DEF_SURF_NUM_CHANNELS           (132)
/**
 * Surface Number of X lines used for the Touch Surface Hardware.
 * For Eg,Number of X lines  is equal to the number of Columns.
 * Refer the Touch Surfaces Beta User guide for more information on these parameters.
 */
#define DEF_SURF_NUM_XLINES             (10)
/**
 * Surface Number of Y lines used for the Touch Surface Hardware.
 * For Eg,Number of Y lines  is equal to the number of Columns.
 * Refer the Touch Surfaces Beta User guide for more information on these parameters.
 */
#define DEF_SURF_NUM_YLINES             (10)
/**
 * Defines the Number of Sleep channels used by the Touch Surface Library.
 */
#define DEF_SURF_NUM_SLEEP_CHANNELS     (4)
/* ! @} */

/*----------------------------------------------------------------------------
 *                  Touch Surface Parameter Configuration
 *----------------------------------------------------------------------------*/
/* ! @} */
/**
 * Maximum Number of Touches, that should detected by the Touch Surface Library.
 * For Eg, if DEF_SURF_MAX_TCH is configured as 1u,
 * then at the most information about 1 touch will be reported.
 * For Eg,if DEF_SURF_MAX_TCH is configured as 2u,
 * then at the most information about 2 touches will be reported.
 * Refer the Touch Surfaces Beta User guide for more information on these parameters.
 */
#define DEF_SURF_MAX_TCH                (2)

/**
 * Surface Detect Integrity
 * Refer the Touch Surfaces Beta User guide for more information on these parameters.
 */
#define DEF_SURF_DI                     (0)
/**
 * Touch Surface maximum ON time duration.
 * Maximum duration upto which touch can be present. sensor that should detected by the Touch Surface Library.
 * If a touch is present more than this duration, the whole surface would be calibrated.
 * Units: 200ms (Example: a value 5u indicated Max ON duration of 1 second.)
 * Default value: 0 (No maximum ON time limit).
 * Range: 0u to 255u.
 * Refer the Touch Surfaces Beta User guide for more information on these parameters.
 */
#define DEF_SURF_MAX_ON_DURATION        (0)
/**
 * Surface Towards Touch Drift Rate.
 * Units: 200ms
 * Default value: 20 = 4 seconds.
 * Range: 1u to 127u.
 * Refer the Touch Surfaces Beta User guide for more information on these parameters.
 */
#define DEF_SURF_TCH_DRIFT_PERIOD       (20)
/**
 * Surface Away From Touch Drift Rate.
 * Units: 200ms
 * Default value: 5 = 1 seconds.
 * Range: 1u to 127u.
 * Refer the Touch Surfaces Beta User guide for more information on these parameters.
 */
#define DEF_SURF_ATCH_DRIFT_PERIOD      (5)
/**
 * Dots Per Inch on the X Axis.The total resolution along the X Axis would be calculated based
 * on DEF_SURF_DPI_X_AXIS and the sensor size of a node on x axis
 */
#define DEF_SURF_DPI_X                  (160)
/**
 * Dots Per Inch on the Y Axis.The total resolution along the Y Axis would be calculated based
 * on DEF_SURF_DPI_X_AXIS and the sensor size of a node on y axis
 */
#define DEF_SURF_DPI_Y                  (160)
/**
 * Size of the whole surface on the X-Axis.This should be filled in mm
 * Units: mm
 * Default value: = 50 mm.
 */
#define DEF_SURF_SIZE_IN_X              (50)
/**
 * Size of the whole surface on the Y-Axis.This should be filled in mm
 * Units: mm
 * Default value: = 50 mm.
 */
#define DEF_SURF_SIZE_IN_Y              (50)
/**
 * Surface measurement interval.
 * Specify period in milliseconds.  Example, SURF_ACTIVE_TCH_SCAN_RATE_MS
 * 50u
 * will perform measurement on surface  every 50msec.
 */
#define SURF_ACTIVE_TCH_SCAN_RATE_MS    (20)
/**
 * Surface measurement When No touch is present in Surface.
 * Specify period in milliseconds.  Example, SURF_NO_TCH_SCAN_RATE_MS
 * 100u
 * will perform measurement on surface  every 100msec when there is no touch.
 */
#define SURF_NO_TCH_SCAN_RATE_MS        (100)
/**
 * Waiting Time before Low Power mode is activated when a touch is not present.
 * Specify period in milliseconds.  Example, NO_ACTIVITY_TRIGGER_TIME
 * 5000u
 * will activate low power mode when a touch is not present on the surface for more than 5000ms
 * Applicable only when DEF_SURF_LOW_POWER_SENSOR_ENABLE is defined as 1,
 *  and DEF_SURF_NUM_SLEEP_CHANNELS is 1.
 */
#define NO_ACTIVITY_TRIGGER_TIME        (5000)
/**
 * To use QTouch Analyser along with Low Power sensor, make a touch while connecting the QTouch Analyser
 * which will wake up the system to connect to QTouch Analyser.
 * or  define the DEF_SURF_LOW_POWER_SENSOR_ENABLE macro as 0 in surface.h
 * Applicable only when DEF_SURF_NUM_SLEEP_CHANNELS is equal to 1.
 */

#define DEF_SURF_LOW_POWER_SENSOR_ENABLE    (1)

/* ! @} */


/*---------------------------------------------------------------------------------------------
 *               Do not Modify. Used by Touch Surface Beta Library for internal purpose
 *--------------------------------------------------------------------------------------------*/
/* ! @} */


/**
 * Defines the Number of segments used by the Touch Surface Library.
 */
#define DEF_SURF_NUM_SEGMENTS           (8)

/**
 * Defines the Maximum Number of Channels that can be grouped/lumped to form a lumped sensor.
 */
#define DEF_MAX_LUMP_SIZE               (30)
/**
 * Defines the Total Number of Rows and Columns in the by the Touch Surface.
 * Note: This will be equal to sum of Number of X lines and Number of Y lines in Touch Surface.
 */
#define DEF_SURF_NUM_ROWS_COLS          (20)
/**
 * Defines the Number of Individual Sensors in the Touch Surface.
 * Note: This will be equal to product of Number of X lines and Number of Y lines in Touch Surface.
 */
#define DEF_SURF_NUM_IND_SENSORS        (100)
/**
 * Defines the Segment information which is used by the Touch Surface Library.
 */
#define SURF_SEG0_ROW_START             (0)
#define SURF_SEG0_ROW_END               (2)
#define SURF_SEG0_COL_START             (0)
#define SURF_SEG0_COL_END               (4)

#define SURF_SEG1_ROW_START             (0)
#define SURF_SEG1_ROW_END               (2)
#define SURF_SEG1_COL_START             (5)
#define SURF_SEG1_COL_END               (9)

#define SURF_SEG2_ROW_START             (3)
#define SURF_SEG2_ROW_END               (5)
#define SURF_SEG2_COL_START             (0)
#define SURF_SEG2_COL_END               (4)

#define SURF_SEG3_ROW_START             (3)
#define SURF_SEG3_ROW_END               (5)
#define SURF_SEG3_COL_START             (5)
#define SURF_SEG3_COL_END               (9)

#define SURF_SEG4_ROW_START             (6)
#define SURF_SEG4_ROW_END               (8)
#define SURF_SEG4_COL_START             (0)
#define SURF_SEG4_COL_END               (4)

#define SURF_SEG5_ROW_START             (6)
#define SURF_SEG5_ROW_END               (8)
#define SURF_SEG5_COL_START             (5)
#define SURF_SEG5_COL_END               (9)

#define SURF_SEG6_ROW_START             (9)
#define SURF_SEG6_ROW_END               (9)
#define SURF_SEG6_COL_START             (0)
#define SURF_SEG6_COL_END               (4)

#define SURF_SEG7_ROW_START             (9)
#define SURF_SEG7_ROW_END               (9)
#define SURF_SEG7_COL_START             (5)
#define SURF_SEG7_COL_END               (9)


/**
 * Defines the Total Resolution on the X axis..
 */
#define DEF_SURF_TOT_RES_X              (315)
/**
 * Defines the Total Resolution on the Y axis..
 */
#define DEF_SURF_TOT_RES_Y              (315)

/**
 * Size of the node on the X-Axis.This should be filled in mm
 * Units: mm
 * Default value: = 5 mm.
 */
#define DEF_SURF_SENSOR_SIZE_IN_X       (5)
/**
 * Size of the node on the Y-Axis.This should be filled in mm
 * Units: mm
 * Default value: = 5 mm.
 */
#define DEF_SURF_SENSOR_SIZE_IN_Y       (5)
/* ! @} */

#endif /* SURFACE_H */
