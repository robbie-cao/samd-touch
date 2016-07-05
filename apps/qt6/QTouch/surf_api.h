/**********************************************************************
*   FILE:  surf_api.h
*   Version Number: $Revision: 16086 $
*   Last Updated   : $Date: 2015-12-11 05:52:00 +0100 (Fri, 11 Dec 2015) $
*   Atmel Corporation:  http://www.atmel.com \n
*   Support email:  touch@atmel.com
**********************************************************************/
/*  License
*   Copyright (c) 2013, Atmel Corporation All rights reserved.
*
*   Redistribution and use in source and binary forms, with or without
*   modification, are permitted provided that the following conditions are met:
*
*   1. Redistributions of source code must retain the above copyright notice,
*   this list of conditions and the following disclaimer.
*
*   2. Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
*   3. The name of ATMEL may not be used to endorse or promote products derived
*   from this software without specific prior written permission.
*
*   THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR IMPLIED
*   WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
*   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY AND
*   SPECIFICALLY DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT,
*   INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
*   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
*   ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
*   THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/


#ifndef SURF_API_H_
#define SURF_API_H_
#include "touch_api_SAMD.h"
#include "surface.h"

/*
SURF_GET_TOUCH_STATE (TOUCH_ID)
To get the state of the particular touch id(whether detect or not) that corresponds to the touch id specified using the TOUCH_ID.
The macro returns either 0 or 1.
If the bit value is 0, the touch is not in detect
If the bit value is 1, the touch is in detect
*/
#define SURF_GET_TOUCH_STATE(TOUCH_ID) surf_status.surf_tch_state & (1 << (TOUCH_ID % 8))
/*
SURF_GET_X_POSITION (TOUCH_ID)
To get the X position for the particular touch id.
*/
#define SURF_GET_X_POSITION(TOUCH_ID) surf_status.ptr_surf_tch_status[TOUCH_ID].x_position

/*
SURF_GET_Y_POSITION (TOUCH_ID)
To get the Y position for the particular touch id.
*/
#define SURF_GET_Y_POSITION(TOUCH_ID) surf_status.ptr_surf_tch_status[TOUCH_ID].y_position

/*
SURF_GET_TOUCH_AREA (TOUCH_ID)
To get the area for the particular touch id.
*/
#define SURF_GET_TOUCH_AREA(TOUCH_ID) surf_status.ptr_surf_tch_status[TOUCH_ID].area


/* Below MACROS should not be modified */
/* Size of one Touch ID structure */
#define SURF_ID_SIZE                    (8u)
#define SURF_PAD_BYTE_SIZE              (90u)
#define SURF_DELTA_SUM_SIZE             (2)
#define SURF_QTS_TCH_PROP_SIZE          (38)
#define SURF_QTS_GRP_TCH_INFO_SIZE      (10)
#define SURF_NODE_PROP_SIZE             (2)
#define SURF_QTS_GRP_PROP_SIZE          (10)
#define SURF_SEG_SIZE                   (4)
#define SURF_NUM_BM_BA_VAR              (5)
#define SURF_GRP_TCH_INFO_SIZE          (10)
#define SURF_GLOBAL_GRP_TCH_INFO_SIZE   (31)
#define SURF_SORTING_GRP_PROP_SIZE      (16)
/**
 * Macros for Lump Channel Configuration
 */
#define LUMP_1(n) ((uint16_t)(1u << n))
#define LUMP_2(a,b)   ((uint16_t)((1       <<     a)     |      (1     <<     b)))
#define LUMP_3(a,b,c) ((uint16_t)((1       <<     a)     |      (1     <<     b)     |      (1       <<     c)))
#define LUMP_4(a,b,c,d)      ((uint16_t)((1       <<     a)     |      (1     <<     b)     |       (1     <<     c)     |      (1     <<     d)))
#define LUMP_5(a,b,c,d,e) ((1 << e) | (LUMP_4(a,b,c,d)))
#define LUMP_6(a,b,c,d,e,f) ((1 << f) | (LUMP_5(a,b,c,d,e)))
#define LUMP_7(a,b,c,d,e,f,g) ((1 << g) | (LUMP_6(a,b,c,d,e,f)))
#define LUMP_8(a,b,c,d,e,f,g,h) ((1 << h) | (LUMP_7(a,b,c,d,e,f,g)))
#define LUMP_9(a,b,c,d,e,f,g,h,i) ((1 << i) | (LUMP_8(a,b,c,d,e,f,g,h)))
#define LUMP_10(a,b,c,d,e,f,g,h,i,j) ((1 << j) | (LUMP_9(a,b,c,d,e,f,g,h,i)))
#define LUMP_11(a,b,c,d,e,f,g,h,i,j,k) ((1 << k) | (LUMP_10(a,b,c,d,e,f,g,h,i,j)))
#define LUMP_12(a,b,c,d,e,f,g,h,i,j,k,l) ((1 << l) | (LUMP_11(a,b,c,d,e,f,g,h,i,j,k)))
#define LUMP_13(a,b,c,d,e,f,g,h,i,j,k,l,m) ((1 << m) | (LUMP_12(a,b,c,d,e,f,g,h,i,j,k,l)))
#define LUMP_14(a,b,c,d,e,f,g,h,i,j,k,l,m,n) ((1 << n) | (LUMP_13(a,b,c,d,e,f,g,h,i,j,k,l,m)))
#define LUMP_15(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o) ((1 << o) | (LUMP_14(a,b,c,d,e,f,g,h,i,j,k,l,m,n)))
#define LUMP_16(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o,p) ((1 << p) | (LUMP_15(a,b,c,d,e,f,g,h,i,j,k,l,m,n,o)))

#define _NUM_ARGS2(X,X16,X15,X14,X13,X12,X11,X10,X9,X8,X7,X6,X5,X4,X3,X2,X1,N,...) N
#define NUM_ARGS(...) _NUM_ARGS2(0, __VA_ARGS__ ,16,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0)

#define _LUMP_OF_L1(N, ...) LUMP_ ## N(__VA_ARGS__)
#define _LUMP_OF_L2(N, ...) _LUMP_OF_L1(N, __VA_ARGS__)
#define LUMP_X(...)      _LUMP_OF_L2(NUM_ARGS(__VA_ARGS__), __VA_ARGS__)
#define LUMP_Y(...)      _LUMP_OF_L2(NUM_ARGS(__VA_ARGS__), __VA_ARGS__)

#define SURF_MAX_NUM_GRP                (5)
#define SURF_BM_IND                     ((DEF_SURF_NUM_CHANNELS%32 == 0?(DEF_SURF_NUM_CHANNELS/32):((DEF_SURF_NUM_CHANNELS/32)+1)))
#define SIZE_OF_UINT32                  (4)

/**
 * Touch Data block size.
 */
#define PRIV_SURF_DATA_BLK_SIZE    ((SURF_DELTA_SUM_SIZE * DEF_SURF_NUM_XLINES) +                                           \
                                    (SURF_DELTA_SUM_SIZE * DEF_SURF_NUM_YLINES) +                                           \
                                    ((SURF_QTS_TCH_PROP_SIZE + SURF_QTS_GRP_TCH_INFO_SIZE)* DEF_SURF_MAX_TCH ) +            \
                                    (SURF_NODE_PROP_SIZE * DEF_SURF_NUM_IND_SENSORS) +                                      \
                                    (SURF_QTS_GRP_PROP_SIZE * SURF_MAX_NUM_GRP ) +                                          \
                                    (DEF_SURF_NUM_IND_SENSORS) +                                                            \
                                    (SIZE_OF_UINT32 * SURF_BM_IND * SURF_NUM_BM_BA_VAR) +                                   \
                                    (4 * SURF_BM_IND * SURF_NUM_BM_BA_VAR) +                                                \
                                    (2 * SURF_MAX_NUM_GRP * SURF_GRP_TCH_INFO_SIZE) +                                       \
                                    (SURF_MAX_NUM_GRP * SURF_GLOBAL_GRP_TCH_INFO_SIZE) +                                    \
                                    ((SURF_MAX_NUM_GRP * SURF_SORTING_GRP_PROP_SIZE) + (SIZE_OF_UINT32*SURF_MAX_NUM_GRP)) + \
                                    (SURF_PAD_BYTE_SIZE))

/*----------------------------------------------------------------------------
                            type definitions
----------------------------------------------------------------------------*/
/* ! Current time type. */
typedef uint16_t surf_current_time_t;


/* ! Surface Library error codes. */
typedef enum tag_surf_ret_t {
       /* ! Successful completion of operation. */
       SURF_SUCCESS,
       /* ! Invalid input parameter. */
       SURF_INVALID_INPUT_PARAM,
       /* ! Operation not allowed in the current Surface Library state. */
       SURF_INVALID_LIB_STATE,
       /* ! Channel number parameter exceeded total number of channels
       * configured. */
       SURF_INVALID_CHANNEL_NUM,
       /* ! Invalid Sensor number parameter. */
       SURF_INVALID_SENSOR_ID
}
surf_ret_t;


/*----------------------------------------------------------------------------
*                                Structures
*  ----------------------------------------------------------------------------*/
typedef struct tag_surf_seg_size_t
{
   uint8_t row_beg;
   uint8_t row_end;
   uint8_t col_beg;
   uint8_t col_end;

}
surf_seg_size_t;

////////////////////////////Touch Surface Status Information End////////////////////

typedef struct tag_surf_global_param_t {
   uint8_t tch_drift_period;
        uint8_t atch_drift_period;
   uint8_t (*surf_complete_callback)( uint16_t );

}
surf_global_param_t;


//////Touch Config Information //////////
typedef struct tag_surf_tch_config_t {
   uint8_t num_touches;
   uint8_t tch_di;
   uint8_t tch_MOD;

}
surf_tch_config_t;

typedef struct tag_surf_pos_prop_t {
 uint16_t dpi_x;
 uint16_t dpi_y;
 uint16_t total_resolution_x;
 uint16_t total_resolution_y;
 uint8_t size_of_sensor_in_mm_on_x;
 uint8_t size_of_sensor_in_mm_on_y;
 uint8_t pos_hysteresis;
}
surf_pos_prop_t;

typedef struct tag_surf_tch_status_t {
   uint8_t tch_id;
   uint8_t int_tch_state;
   uint8_t area;
   uint16_t x_position;
   uint16_t y_position;
}
surf_tch_status_t;

typedef struct tag_surf_status_t {
   uint8_t             num_active_touches;
   uint8_t             surf_tch_state;//each bit mask represent ON/OFF for each touch
   surf_tch_status_t  *ptr_surf_tch_status;//index is touch id.
   uint32_t           *ptr_surf_cyc_burst_mask;

}
surf_status_t;

typedef struct tag_surf_hw_config_t {
   uint16_t num_total_channels;
   uint8_t  num_x_lines;
   uint8_t  num_y_lines;
   uint8_t  num_sleep_channels;
}
surf_hw_config_t;

/* ! Touch Surface Input Configuration type. */
typedef struct tag_surf_config_t {
   surf_hw_config_t surf_hw_config;
   surf_pos_prop_t surf_pos_prop;
   surf_global_param_t surf_global_param;
   surf_tch_config_t surf_tch_config;//num_touches,di,mod
   surf_seg_size_t *ptr_surf_seg_size;
   uint8_t *ptr_surf_data_block;
   uint16_t surf_data_block_size;
}
surf_config_t;

/* ! Surface library version information type. */
typedef struct tag_surf_libver_info_t {
   /* ! Chip ID */
   uint32_t chip_id;
   /* ! Product ID */
   uint8_t product_id;
   /* ! Touch Library version. */
   uint16_t fw_version;
}
surf_libver_info_t;


/*----------------------------------------------------------------------------
*                                APIs
*  ----------------------------------------------------------------------------*/


/*! \brief This API can be used to initialize the different parameters of the touch surface
 *
 *  \param ptr_surf_config:Pointer to the surface config structure
 *         ptr_surf_status:Pointer to the surface status structure
 *         ptr_touch_config:Pointer to the touch configuration structure
 *  \return surf_ret_t: Surface Library Error status.
 */
surf_ret_t surf_init(surf_config_t *ptr_surf_config, surf_status_t *ptr_surf_status,touch_config_t *ptr_touch_config);


/*! \brief This API can be used to calibrate all the sensors which are part of the touch surface
 *
 *  \param none:
 *
 *  \return surf_ret_t: Surface Library Error status.
 */
surf_ret_t surf_calibrate_all(void);


/*! \brief This API can be used to measure the touch surface
 *
 *  \param surf_current_time_t: Current time in ms passed from the application
 *
 *  \return surf_ret_t: Surface Library Error status.
 */
surf_ret_t surf_measure(surf_current_time_t current_time_ms);

/*! \brief This API can be used to get the surface library version information
 *
 *  \param surf_libver_info_t: Pointer to the surface library version information.
 *
 *  \return surf_ret_t: Surface Library Error status.
 */
surf_ret_t surf_library_get_version_info( surf_libver_info_t *ptr_surf_libver_info );

/*! \brief This API can be used to start the low power measurement.
 *
 *  \param none:
 *
 *  \return surf_ret_t: Surface Library Error status.
 */

surf_ret_t surf_low_power_start(void);

/*! \brief This API can be used to stop the low power measurement.
 *
 *  \param none:
 *
 *  \return surf_ret_t: Surface Library Error status.
 */
surf_ret_t surf_low_power_stop(void);

#endif /* SURF_API_H_ */
