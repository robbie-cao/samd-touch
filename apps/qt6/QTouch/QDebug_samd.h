/* This source file is part of the ATMEL QTouch Surface Library 1.0.3 */

/*****************************************************************************
 *
 * \file
 *
 * \brief  This file contains the QDebug public API that can be used to
 * transfer data from a Touch Device to QTouch Studio using the QT600
 * USB Bridge.
 *
 *
 * - Userguide:          QTouch Library Peripheral Touch Controller User Guide.
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

#ifndef _QDEBUG_SAMD_H_
#define _QDEBUG_SAMD_H_

#ifdef __cplusplus
extern "C"
{
#endif

/*============================ INCLUDES ======================================*/
#include "touch.h"
#include "touch_api_SAMD.h"
#include "surf_api.h"
#include "surface.h"
/* ! compile file only when QDebug is enabled. */
#if DEF_TOUCH_QDEBUG_ENABLE == 1

/*======================== MANIFEST CONSTANTS ================================*/

/*! \name Subscription definitions.
 */
/* ! @{ */

#define SUBS_SIGN_ON                    0
#define SUBS_GLOBAL_CONFIG              1
#define SUBS_SENSOR_CONFIG              2
#define SUBS_SIGNALS                    3
#define SUBS_REF                        4
#define SUBS_DELTA                      5
#define SUBS_STATES                     6
#define SUBS_QM_BURST_LENGTHS           7
#define SUBS_TIMESTAMPS                 8
#define SUBS_USER_DATA                  9
#define SUBS_SURF_CONFIG                10
#define SUBS_SURF_STATUS                11
#define SUBS_SURF_CHANNEL_INFO          12
    /* ! @} */

    /*! \name PC commands.
    */
    /* ! @{ */

#define QT_CMD_DUMMY                    0x10
#define QT_CMD_SET_SUBS                 0x11
#define QT_CMD_SET_GLOBAL_CONFIG        0x12
#define QT_CMD_SET_CH_CONFIG            0x13
#define QT_CMD_SET_MEASUREMENT_PERIOD   0x14
#define QT_CMD_SET_QM_BURST_LENGTHS     0x15

    /* ! @} */

    /*! \name Touch MCU data packets.
    */
    /* ! @{ */

#define QT_DUMMY                        0x20
#define QT_SIGN_ON                      0x21
#define QT_GLOBAL_CONFIG                0x22
#define QT_SENSOR_CONFIG                0x23
#define QT_SIGNALS                      0x24
#define QT_REFERENCES                   0x25
#define QT_DELTAS                       0x26
#define QT_STATES                       0x27
#define QT_QM_BURST_LENGTHS             0x28
#define QT_TIMESTAMPS                   0x29
#define QT_USER_DATA                    0x2A
#define QT_SURF_CONFIG                  0x2B
#define QT_SURF_STATUS                  0x2C
#define QT_SURF_CHANNEL_INFO            0x2D



/* ! @} */

/*! \name Surface Packet related Macros
 */
/* ! @{ */
#define QT_SURF_MAX_NO_OF_TCH                   surf_config.surf_tch_config.num_touches
#define QT_SURF_NUM_ROWS                        surf_config.surf_hw_config.num_y_lines
#define QT_SURF_NUM_COLS                        surf_config.surf_hw_config.num_x_lines
#define QT_SURF_X_RANGE                         ((uint16_t)(surf_config.surf_pos_prop.total_resolution_x))
#define QT_SURF_Y_RANGE                         ((uint16_t)(surf_config.surf_pos_prop.total_resolution_y))
#define QT_SURF_CHANNEL_BURST_MASK(x)           surf_status.ptr_surf_cyc_burst_mask[x]



#define QT_SURF_NUM_TOUCHES                     surf_status.num_active_touches
#define SURF_TCH_STATUS(x)                      surf_status.ptr_surf_tch_status[x]
#define SURF_TCH_GET_AREA(x)                    surf_status.ptr_surf_tch_status[x].area
#define SURF_TCH_GET_X_POSITION(x)              surf_status.ptr_surf_tch_status[x].x_position
#define SURF_TCH_GET_Y_POSITION(x)              surf_status.ptr_surf_tch_status[x].y_position
#define SURF_TCH_GET_STATE(x)                   surf_status.ptr_surf_tch_status[x].int_tch_state
#define SURF_TCH_GET_ID(x)                      surf_status.ptr_surf_tch_status[x].tch_id

#define SURF_SLEEP_SENSOR_ID_START              ( DEF_SURF_NUM_IND_SENSORS )
#define SURF_SLEEP_SENSOR_ID_END                ( DEF_SURF_NUM_IND_SENSORS + DEF_SURF_NUM_SLEEP_CHANNELS - 1u )

#define SURF_SEGMENT_SENSOR_ID_START            ( DEF_SURF_NUM_IND_SENSORS + DEF_SURF_NUM_SLEEP_CHANNELS )
#define SURF_SEGMENT_SENSOR_ID_END              ( DEF_SURF_NUM_IND_SENSORS + DEF_SURF_NUM_SLEEP_CHANNELS + DEF_SURF_NUM_SEGMENTS - 1u )

#define SURF_ROW_COL_SENSOR_ID_START            ( DEF_SURF_NUM_IND_SENSORS + DEF_SURF_NUM_SLEEP_CHANNELS + DEF_SURF_NUM_SEGMENTS)
#define SURF_ROW_COL_SENSOR_ID_END              ( DEF_SURF_NUM_IND_SENSORS + DEF_SURF_NUM_SLEEP_CHANNELS + DEF_SURF_NUM_SEGMENTS + DEF_SURF_NUM_ROWS_COLS - 1u )

#define SURF_IND_SENSOR_ID_START                (0u)
#define SURF_IND_SENSOR_ID_END                  (DEF_SURF_NUM_IND_SENSORS - 1u )


/* ! @} */

/*============================ MACROS ========================================*/

/*! \name Macros for QTouch API and data structure.
 */
#ifdef DEF_TOUCH_QDEBUG_ENABLE_MUTLCAP

#define QDEBUG_SENSOR_STATES_PTR               p_mutlcap_measure_data->           \
    p_sensor_states
#define QDEBUG_SENSOR_PTR                      p_mutlcap_measure_data->           \
    p_sensors
#define QDEBUG_SIGNALS_PTR                     p_mutlcap_measure_data->           \
    p_channel_signals
#define QDEBUG_REFERENCES_PTR                  p_mutlcap_measure_data->           \
    p_channel_references
#define QDEBUG_SENSOR_RS_VALUES                p_mutlcap_measure_data->           \
    p_rotor_slider_values
#define QDEBUG_LIBINFO                         qt_surf_libver_info
#define QDEBUG_NUM_SENSOR_STATE_BYTES          ((DEF_MUTLCAP_NUM_CHANNELS +       \
            7u) / 8u)
#define QDEBUG_GET_DELTA_FUNC(x, y)            touch_mutlcap_sensor_get_delta(    \
        x, y)
#define QDEBUG_GET_LIBINFO_FUNC(x)             surf_library_get_version_info(x)
#define QDEBUG_GET_GLOBAL_PARAM_FUNC(x)        touch_mutlcap_get_global_param(x)
#define QDEBUG_UPDATE_GLOBAL_PARAM_FUNC(x)     touch_mutlcap_update_global_param( \
        x)
#define QDEBUG_GET_SENSOR_CONFIG_FUNC(x, y)    touch_mutlcap_sensor_get_config(   \
        x, y)
#define QDEBUG_UPDATE_SENSOR_CONFIG_FUNC(x,                                       \
        y)  touch_mutlcap_sensor_update_config(x, y)
#define QDEBUG_NUM_SENSORS                     DEF_MUTLCAP_NUM_SENSORS
#define QDEBUG_NUM_CHANNELS                    DEF_MUTLCAP_NUM_CHANNELS
#define QDEBUG_NUM_ROTORS_SLIDERS              DEF_MUTLCAP_NUM_ROTORS_SLIDERS

    //GLOBAL_PARAMS
#define QDBEUG_GLOBAL_PARAM_RECAL_THRESHOLD    p_mutlcap_config->global_param.recal_threshold
#define QDBEUG_GLOBAL_PARAM_DI                 p_mutlcap_config->global_param.di
#define QDEBUG_GLOBAL_PARAM_DRIFT_HOLD_TIME    p_mutlcap_config->global_param.drift_hold_time
#define QDEBUG_GLOBAL_PARAM_MAX_ON_DURATION    p_mutlcap_config->global_param.max_on_duration
#define QDEBUG_GLOBAL_PARAM_TCH_DRIFT_RATE     p_mutlcap_config->global_param.tch_drift_rate
#define QDEBUG_GLOBAL_PARAM_ATCH_DRIFT_RATE    p_mutlcap_config->global_param.atch_drift_rate
#define QDEBUG_GLOBAL_PARAM_ATCH_RECAL_DELAY   p_mutlcap_config->global_param.atch_recal_delay

    //SENSOR_CONFIG
#define QDBEUG_SENSOR_CONFIG_DETECT_THRESHOLD(x)    (sensor_detect_threshold[x])
#define SENSOR_TYPE_AKS_POS_HYST(x)                 (0x00000)
#define QDEBUG_SENSOR_FROM_CHANNEL(x)               (x)
#define QDEBUG_SENSOR_TO_CHANNEL(x)                 (x)




#endif

#ifdef DEF_TOUCH_QDEBUG_ENABLE_SELFCAP
#define QDEBUG_SENSOR_STATES_PTR               p_selfcap_measure_data->           \
    p_sensor_states
#define QDEBUG_SENSOR_PTR                      p_selfcap_measure_data->           \
    p_sensors
#define QDEBUG_SIGNALS_PTR                     p_selfcap_measure_data->           \
    p_channel_signals
#define QDEBUG_REFERENCES_PTR                  p_selfcap_measure_data->           \
    p_channel_references
#define QDEBUG_SENSOR_RS_VALUES                p_selfcap_measure_data->           \
    p_rotor_slider_values
#define QDEBUG_LIBINFO                         qt_touch_libver_info
#define QDEBUG_NUM_SENSOR_STATE_BYTES          ((DEF_SELFCAP_NUM_CHANNELS +       \
            7u) / 8u)
#define QDEBUG_GET_DELTA_FUNC(x, y)             touch_selfcap_sensor_get_delta(   \
        x, y)
#define QDEBUG_GET_LIBINFO_FUNC(x)             touch_library_get_version_info(x)
#define QDEBUG_GET_GLOBAL_PARAM_FUNC(x)        touch_selfcap_get_global_param(x)
#define QDEBUG_UPDATE_GLOBAL_PARAM_FUNC(x)     touch_selfcap_update_global_param( \
        x)
#define QDEBUG_GET_SENSOR_CONFIG_FUNC(x, y)    touch_selfcap_sensor_get_config(   \
        x, y)
#define QDEBUG_UPDATE_SENSOR_CONFIG_FUNC(x,                                       \
        y)  touch_selfcap_sensor_update_config(x, y)
#define QDEBUG_NUM_SENSORS                     DEF_SELFCAP_NUM_SENSORS
#define QDEBUG_NUM_CHANNELS                    DEF_SELFCAP_NUM_CHANNELS
#define QDEBUG_NUM_ROTORS_SLIDERS              DEF_SELFCAP_NUM_ROTORS_SLIDERS
#endif



/*============================ PROTOTYPES ====================================*/

/*! \name Public functions.
 */
/* ! @{ */

/*! \brief This API initializes QDebug interface, including the low level
 * hardware interface (SPI, TWI, USART etc).
 * \note Must be called before using any other QDebug API.
 */
void QDebug_Init(void);

/*! \brief Command handler for the data received from QTouch Studio
 * \note This function should be called in the main loop after
 * measure_sensors to process the data received from QTOuch Studio.
 */
void QDebug_ProcessCommands(void);

/*! \brief Send data to QTouch Studio based on the subscription.
 * \param  qt_lib_flags:Change flag from measure_sensors.
 */
void QDebug_SendData(uint16_t qt_lib_flags);

/*! \brief Set subscription values.
 * \note  This function can be used directly in main to set data subscription
 * if 1way SPI interface is used.
 */
void QDebug_SetSubscriptions(uint16_t once, uint16_t change,
        uint16_t allways);

/* ! @} */

/*! \name Private functions.
 */
/* ! @{ */

/*! \brief Extract the data packet from QTouch Studio and set global config.
 * \note  Should only be called from the command handler.
 */
void Set_Global_Config(void);

/*! \brief Extract the data packet from QTouch Studio and set channel config.
 * \note  Should only be called from the command handler.
 */
void Set_Channel_Config(void);

/*! \brief Set Data Subscription values.
 * \note  Should only be called from the command handler.
 */
void Set_Subscriptions(void);

/*! \brief Extract the data packet from QTouch Studio and set measurement
 * period.
 * \note  Should only be called from the command handler.
 */
void Set_Measurement_Period(void);

/*! \brief Extract the data packet from QTouch Studio and set QMatrix burst
 * lengths.
 * \note  Should only be called from the command handler.
 */
void Set_QM_Burst_Lengths(void);

/*! \brief Extracts user data from QTouch Studio to touch mcu memory.
 * \param pdata: data pointer.
 * \note  The data can be binary data.
 */
void Set_QT_User_Data(uint8_t *pdata);

/*! \brief Transmits a dummy packet if no other subscriptions are set.
 */
void Transmit_Dummy(void);

/*! \brief Transmits the sign on packet to QTouch Studio.
 */
void Transmit_Sign_On(void);

/*! \brief Transmits the global config struct to QTouch Studio.
 */
void Transmit_Global_Config(void);

/*! \brief Transmits the channel config struct to QTouch Studio.
 */
void Transmit_Sensor_Config(void);

/*! \brief Transmits the measurement values for each channel to QTouch Studio.
 */
void Transmit_Signals(void);

/*! \brief Transmits the channel reference values to QTouch Studio.
 */
void Transmit_Ref(void);

/*! \brief Transmits the channel delta values to QTouch Studio.
 */
void Transmit_Delta(void);

/*! \brief Transmits the state values to QTouch Studio.
 */
void Transmit_State(void);

/*! \brief Transmits the application execution timestamp values to QTouch
 * Studio.
 * \note This value is a combination of current_time_ms_touch (high word) &
 * timer counter register (low word).
 */
void Transmit_Time_Stamps(void);

/*! \brief Transmits user data to QTouch Studio.
 * \param pdata: data pointer.
 * \param c: length of data in bytes.
 * \note The data will be binary data.
 */
void Transmit_QT_User_Data(uint8_t *pdata, uint16_t c);
/*! \brief Transmits the QT Surface Touch properties  to QTouch Studio.
 */
void Transmit_QT_Surf_Status(void);
/*! \brief Transmits the QT surface Configuration packet to QTouch Studio.
 */
void Transmit_QT_Surf_Config(void);
/*! \brief Transmits the QT surface Channel Information
 */
void Transmit_QT_Surf_Channel_Info(void);

/* ! @} */



#ifdef __cplusplus
}
#endif

#endif /*DEF_TOUCH_QDEBUG_ENABLE==1*/
#endif /* _QDEBUG_SAMD_H_ */
/* EOF */
