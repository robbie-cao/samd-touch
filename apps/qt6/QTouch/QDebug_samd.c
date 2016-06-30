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

/*============================ INCLUDES ======================================*/
#include "asf.h"
#include "QDebug_samd.h"

/* ! compile file only when QDebug is enabled. */
#if DEF_TOUCH_QDEBUG_ENABLE == 1

#include "QDebugTransport.h"
#include "QDebugSettings.h"

#if (defined QDEBUG_SPI)
#include "SPI_Master.h"
#elif (defined QDEBUG_SERIAL)
#include "SERIAL.h"
#elif (defined QDEBUG_BITBANG_SPI)
#include "BitBangSPI_Master.h"
#else
#warning "No Debug Interface is selected in QDebugSettings.h"
#endif

/*============================ GLOBAL VARIABLES ==============================*/
/* ! These must somehow be updated from the library. */
uint8_t qgRefschanged;
uint8_t qgStateschanged;

/*! \name Object to get library info.
 */
surf_libver_info_t qt_surf_libver_info;
extern surf_status_t surf_status;
extern surf_config_t surf_config;
extern uint16_t mutlcap_xy_nodes[DEF_MUTLCAP_NUM_CHANNELS * 2];
/**
 * Time stamp information.
 */
uint16_t timestamp1_hword;
uint16_t timestamp1_lword;
uint16_t timestamp2_hword;
uint16_t timestamp2_lword;
uint16_t timestamp3_hword;
uint16_t timestamp3_lword;

/*============================ EXTERN VARIABLES ==============================*/
extern touch_config_t touch_config;
extern volatile int8_t autonomous_qtouch_in_touch;

/*============================ LOCAL VARIABLES ===============================*/
/* ! Subscriptions. */
static uint16_t qgSubsOnce = 0;
static uint16_t qgSubsChange = 0;
static uint16_t qgSubsAllways = 0;
static uint16_t qgLibraryChanges = 0;
static uint16_t delivery = 0;

#if (defined QDEBUG_SPI)
static bool transmit_dummy = true;
#elif (defined QDEBUG_SERIAL)
static bool transmit_dummy = false;
#elif (defined QDEBUG_BITBANG_SPI)
static bool transmit_dummy = true;
#else
#endif

#if ((BOARD == SAMD20_XPLAINED_PRO) || (BOARD == SAMD21_XPLAINED_PRO))
extern volatile uint16_t touch_time_counter;
#else
extern void set_timer_period(uint16_t time);

#endif
/*============================ IMPLEMENTATION ================================*/

/*! \brief This API initializes QDebug interface, including the low level
 * hardware interface (SPI, TWI, USART etc).
 * \note Must be called before using any other QDebug API.
 */
void QDebug_Init(void)
{
	touch_ret_t touch_ret = TOUCH_SUCCESS;

#if (defined QDEBUG_BITBANG_SPI)
	BitBangSPI_Master_Init();
#endif

	touch_ret = (touch_ret_t)QDEBUG_GET_LIBINFO_FUNC(&QDEBUG_LIBINFO);
	if (touch_ret != TOUCH_SUCCESS) {
		while (1) {
		}
	}

	Init_Buffers();
}

/*! \brief Command handler for the data received from QTouch Studio
 * \note This function should be called in the main loop after
 * measure_sensors to process the data received from QTOuch Studio.
 */
void QDebug_ProcessCommands(void)
{
	uint8_t CommandID;

	if (Receive_Message() == 0) {
		return;
	}

	/* Handle the commands */
	CommandID = GetChar();

	switch (CommandID) {
	case QT_CMD_DUMMY:
		break;

	case QT_CMD_SET_SUBS:
		Set_Subscriptions();
		break;

	case QT_CMD_SET_GLOBAL_CONFIG:
		Set_Global_Config();
		Set_Measurement_Period();
		break;

	case QT_CMD_SET_CH_CONFIG:
		Set_Channel_Config();
		break;
	}

	RX_Buffer[0] = 0;
	RX_index = 0;
}

/*! \brief Send data to QTouch Studio based on the subscription.
 * \param  qt_lib_flags:Change flag from measure_sensors.
 */
void QDebug_SendData(uint16_t qt_lib_flags)
{
	SequenceH = (SequenceH + 1) & 0x0F;

	/* Test if measure_sensors has reported change in key states or
	 * rotor/slider positions */
	if ((qt_lib_flags & TOUCH_BURST_AGAIN) &&

	(qt_lib_flags & (!(TOUCH_RESOLVE_CAL)))
	)
	{
		//qgLibraryChanges |= (1 << SUBS_SURF_STATUS);
	}

	/* Test if measure_sensors has reported change in at least one channel
	 * reference */
	if (qt_lib_flags & TOUCH_CHANNEL_REF_CHANGE) {
		//qgLibraryChanges |= (1 << SUBS_REF);
	}

	delivery = qgSubsAllways | qgSubsOnce |
			(qgLibraryChanges & qgSubsChange);

	if (delivery & (1 << SUBS_SURF_CONFIG)) {
		Transmit_QT_Surf_Config();
	}

	if (delivery & (1 << SUBS_SURF_CHANNEL_INFO)) {
		Transmit_QT_Surf_Channel_Info();
	}

	if (delivery & (1 << SUBS_SIGN_ON)) {
		Transmit_Sign_On();
	}

	if (delivery & (1 << SUBS_GLOBAL_CONFIG)) {
		Transmit_Global_Config();
	}

	if (delivery & (1 << SUBS_SENSOR_CONFIG)) {
		Transmit_Sensor_Config();
	}


	if (delivery & (1 << SUBS_SIGNALS)) {
		Transmit_Signals();
	}

	if (delivery & (1 << SUBS_REF)) {
		Transmit_Ref();
	}

	if (delivery & (1 << SUBS_STATES)) {
		Transmit_State();
	}

	if (delivery & (1 << SUBS_DELTA)) {
		Transmit_Delta();
	}

	if (delivery & (1 << SUBS_SURF_STATUS)) {
		Transmit_QT_Surf_Status();
	}


	if ((transmit_dummy == true) && (delivery == 0)) {
		Transmit_Dummy();
	}

	/* Reset masks until PC sends another request */
	qgSubsOnce = 0;

	/* Reset the changes we have just sent */
	qgLibraryChanges &= ~delivery;
}

/*! \brief Set subscription values.
 * \note  This function can be used directly in main to set data subscription
 * if 1way SPI interface is used.
 */
void QDebug_SetSubscriptions(uint16_t once, uint16_t change, uint16_t allways)
{
	qgSubsOnce = once;
	qgSubsChange = change;
	qgSubsAllways = allways;
}

/*! \brief Set Data Subscription values.
 * \note  Should only be called from the command handler.
 */
void Set_Subscriptions(void)
{
	uint16_t a=(GetChar()<<8);
	a |= GetChar();
	uint16_t b =(GetChar()<<8);
	b |=GetChar();
	uint16_t c = (GetChar()<<8);
	c |=GetChar();
	QDebug_SetSubscriptions(a, b, c);
}

/*! \brief Extract the data packet from QTouch Studio and set global config.
 * \note  Should only be called from the command handler.
 */
void Set_Global_Config(void)
{
	#if 0 //to be removed.
	touch_ret_t touch_ret;
#ifdef DEF_TOUCH_QDEBUG_ENABLE_MUTLCAP
	touch_config_t *p_touch_config = &touch_config;
	touch_mutlcap_config_t *p_mutlcap_config
		= p_touch_config->p_mutlcap_config;

	p_mutlcap_config->global_param.recal_threshold
		= GetChar();
	p_mutlcap_config->global_param.di = GetChar();
	p_mutlcap_config->global_param.drift_hold_time = GetChar();
	p_mutlcap_config->global_param.max_on_duration = GetChar();
	p_mutlcap_config->global_param.tch_drift_rate = GetChar();
	p_mutlcap_config->global_param.atch_drift_rate = GetChar();
	p_mutlcap_config->global_param.atch_recal_delay = GetChar();
#endif
#ifdef DEF_TOUCH_QDEBUG_ENABLE_SELFCAP
	touch_config_t *p_touch_config = &touch_config;
	touch_selfcap_config_t *p_selfcap_config
		= p_touch_config->p_selfcap_config;

	p_selfcap_config->global_param.recal_threshold
		= (recal_threshold_t)GetChar();
	p_selfcap_config->global_param.di = GetChar();
	p_selfcap_config->global_param.drift_hold_time = GetChar();
	p_selfcap_config->global_param.max_on_duration = GetChar();
	p_selfcap_config->global_param.tch_drift_rate = GetChar();
	p_selfcap_config->global_param.atch_drift_rate = GetChar();
	p_selfcap_config->global_param.atch_recal_delay = GetChar();
#endif

	touch_time.measurement_period_ms          = (GetChar() << 8u);
	touch_time.measurement_period_ms          |= GetChar();
#if ((BOARD == SAMD20_XPLAINED_PRO) || (BOARD == SAMD21_XPLAINED_PRO))
	touch_time_counter = 0u;
#else
	set_timer_period(touch_time.measurement_period_ms);
#endif
#ifdef DEF_TOUCH_QDEBUG_ENABLE_SELFCAP
	/* update the global parameter value inside the SAMD library */
	touch_ret = QDEBUG_UPDATE_GLOBAL_PARAM_FUNC(
			&p_selfcap_config->global_param);
	if (touch_ret != TOUCH_SUCCESS) {
		while (1) {
		}
	}

#endif

#ifdef DEF_TOUCH_QDEBUG_ENABLE_MUTLCAP
	/* update the global parameter value inside the SAMD library */
	touch_ret = QDEBUG_UPDATE_GLOBAL_PARAM_FUNC(
			&p_mutlcap_config->global_param);
	if (touch_ret != TOUCH_SUCCESS) {
		while (1) {
		}
	}

#endif

	#endif
}

/*! \brief Extract the data packet from QTouch Studio and set channel config.
 * \note  Should only be called from the command handler.
 */
void Set_Channel_Config(void)
{
#if 0
	touch_ret_t touch_ret = TOUCH_SUCCESS;
	sensor_id_t sensor_id;
	uint8_t type_aks_pos_hyst;
#ifdef DEF_TOUCH_QDEBUG_ENABLE_MUTLCAP
	touch_mutlcap_param_t touch_sensor_param;
#else
	touch_selfcap_param_t touch_sensor_param;
#endif

	sensor_id = GetChar();

	/* Read back and initialize the touch_sensor_param structure.
	 * This is because not all the touch_sensor_param members are
	 * updated by this API. */
	touch_ret
		= QDEBUG_GET_SENSOR_CONFIG_FUNC(sensor_id,
			&touch_sensor_param);
	if (touch_ret != TOUCH_SUCCESS) {
		while (1) {
		}
	}

	/* Update the necessary members of the touch_sensor_param structure
	 * and write back. */
	touch_sensor_param.detect_threshold = GetChar();
	type_aks_pos_hyst = GetChar();
	touch_sensor_param.aks_group
		= (aks_group_t)((uint8_t)((type_aks_pos_hyst & 0x38u) >> 3u));
	touch_sensor_param.detect_hysteresis
		= (hysteresis_t)((uint8_t)(type_aks_pos_hyst & 0x03u));

	touch_ret = QDEBUG_UPDATE_SENSOR_CONFIG_FUNC(sensor_id,
			&touch_sensor_param);
	if (touch_ret != TOUCH_SUCCESS) {
		while (1) {
		}
	}

#endif

}

/*! \brief Extract the data packet from QTouch Studio and set measurement
 * period.
 * \note  Should only be called from the command handler.
 */
void Set_Measurement_Period(void)
{
	touch_time.measurement_period_ms = touch_time.measurement_period_ms; /*
	                                                                      *
	                                                                      *Dummy.
	                                                                      **/
}

/*! \brief Extracts user data from QTouch Studio to touch mcu memory.
 * \param pdata: data pointer.
 * \note  The data can be binary data.
 */
void Set_QT_User_Data(uint8_t *pdata)
{
	uint16_t c = RX_Buffer[1];
	while (c > 0) {
		PutChar(*pdata++);
		c--;
	}
	Send_Message();
}

/*! \brief Transmits a dummy packet if no other subscriptions are set.
 */
void Transmit_Dummy(void)
{
	PutChar(QT_DUMMY);
	Send_Message();
}
/*! \brief Transmits the QT Surface Touch properties  to QTouch Studio.
 */
void Transmit_QT_Surf_Status(void)
{

	/*! \QT surface Touch PRop packet ID
	*/
	PutChar(QT_SURF_STATUS);
	/*! \No of touches detected
	* or no of touches currently	active
	*/
	PutChar(QT_SURF_NUM_TOUCHES);

	/*! \Properties of the active touch
	* Touch ID
	* X pos
	* y pos
	* Area of the touch
	*/

	for(uint8_t i = 0; i< QT_SURF_MAX_NO_OF_TCH; i++)
	{

		if(SURF_TCH_GET_STATE(i) != 255)
		{
			/*! \Touch ID
			*/
			PutChar(SURF_TCH_GET_ID(i));
			/*! \Touch state
			*/
			PutChar(SURF_TCH_GET_STATE(i));
			/*! \X position of the touch
			*/
			PutInt(SURF_TCH_GET_X_POSITION(i));
			/*! \Y position of the touch
			*/
			PutInt(SURF_TCH_GET_Y_POSITION(i));
			/*! \Area of the touch
			* or the number of nodes active in that touch
			*/
			PutChar(SURF_TCH_GET_AREA(i));

		}
		else
		{
			/*! \Touch is Invalid,
			* So No touch Property info is sent
			*/
		}

	}
		Send_Message();

		}
/*! \brief Transmits the QT Surf Channel Info
*/
void Transmit_QT_Surf_Channel_Info(void)
{

	/*! \Surface Channel Info packetID.
	*/
	PutChar(QT_SURF_CHANNEL_INFO);
	/*! \Number of sensors in the QT Surface.
	*/
	PutInt(QDEBUG_NUM_SENSORS);



	/*! \Sensor ID of the first Sleep sensor.
	*/
	PutInt(SURF_SLEEP_SENSOR_ID_START);
	/*! \Sensor ID of the last Sleep sensor.
	*/
	PutInt(SURF_SLEEP_SENSOR_ID_END);





	/*! \Sensor ID of the first Segment sensor.
	*/
	PutInt(SURF_SEGMENT_SENSOR_ID_START);
	/*! \Sensor ID of the last Segment sensor.
	*/
	PutInt(SURF_SEGMENT_SENSOR_ID_END);





	/*! \Sensor ID of the first Row Col Sensor.
	*/
	PutInt(SURF_ROW_COL_SENSOR_ID_START);
	/*! \Sensor ID of the last Row Col sensor.
	*/
	PutInt(SURF_ROW_COL_SENSOR_ID_END);




	/*! \Sensor ID of the first Individual sensor.
	*/
	PutInt(SURF_IND_SENSOR_ID_START);
	/*! \Sensor ID of the last Individual sensor.
	*/
	PutInt(SURF_IND_SENSOR_ID_END);




	/*! \X and Y mask information of the sleep channels
	*/
	for(uint8_t num=0;num<DEF_SURF_NUM_SLEEP_CHANNELS;num++)
	{
		/*! \X mask of the sleep channel
		*/
		PutInt(mutlcap_xy_nodes[(2* SURF_SLEEP_SENSOR_ID_START) + 2 * num]);
		/*! \Y mask of the sleep channel
		*/
		PutInt(mutlcap_xy_nodes[(2* SURF_SLEEP_SENSOR_ID_END) + 2 * num + 1 ]);

	}



	/*! \X and Y mask information of the Segments.
	*/
	for(uint8_t num=0;num<DEF_SURF_NUM_SEGMENTS;num++)
	{
		/*! \X mask of the segment
		*/
		PutInt(mutlcap_xy_nodes[(2* SURF_SEGMENT_SENSOR_ID_START) + 2 * num]);
		/*! \Y mask of the segment
		*/
		PutInt(mutlcap_xy_nodes[(2* SURF_SEGMENT_SENSOR_ID_END) + 2 * num + 1]);

	}



	Send_Message();

}
/*! \brief Transmits the QT surface Configuration packet to QTouch Studio.
 */
void Transmit_QT_Surf_Config(void)
{

	/*! \Surface Config PAcket ID.
	*/
	PutChar(QT_SURF_CONFIG);
	/*! \Maximum no of touches supported by the FIrmware.
	*/
	PutChar(QT_SURF_MAX_NO_OF_TCH);
	/*! Number of Rows
	*For eg if no of x/y lines used for rows are 10,then no of rows are 10
	*/
	PutChar(QT_SURF_NUM_ROWS);
	/*! Number of Columns
	*For eg if no of x/y lines used for columns are 10,then no of columns are 10
	*/
	PutChar(QT_SURF_NUM_COLS);
	/*! Range of X positions
	*For eg if x varies from 0-319,then Range = 320
	*/
	PutInt(QT_SURF_X_RANGE);
	/*! Range of Y positions
	*For eg if y varies from 0-319,then Range = 320
	*/
	PutInt(QT_SURF_Y_RANGE);


	Send_Message();

}

/*! \brief Transmits the sign on packet to QTouch Studio.
 */
void Transmit_Sign_On(void)
{
	uint16_t temp_output;
	PutChar(QT_SIGN_ON);
	PutInt(PROJECT_ID);
	PutChar(INTERFACE);
	PutChar(1);             /* PROTOCOL_TYPE */
	PutChar(2);             /* PROTOCOL_VERSION */
#if DEF_TOUCH_MUTLCAP == 1
	PutChar(1);             /* LIB_TYPE    Mutual Cap */
#else
	PutChar(0);             /* LIB_TYPE    Self Cap */
#endif

PutChar(QDEBUG_LIBINFO.product_id); /*Product ID*/

temp_output=(uint16_t)(QDEBUG_LIBINFO.chip_id >> 16u);
PutInt(temp_output);  /*CHIP Id First 2 Bytes (MSBytes)*/

temp_output=(uint16_t)(QDEBUG_LIBINFO.chip_id & 0x0000FFFF);
PutInt(temp_output); /*CHIP Id Last 2 Bytes (LSBytes)*/


	PutInt(QDEBUG_LIBINFO.fw_version); /* LIB_VERSION */
	PutInt(0);              /* LIB_VARIANT */
	PutChar(QDEBUG_NUM_CHANNELS);
	PutInt(delivery);       /* subscription info */
	Send_Message();
}

/*! \brief Transmits the global config struct to QTouch Studio.
 */
void Transmit_Global_Config(void)
{
#ifdef DEF_TOUCH_QDEBUG_ENABLE_MUTLCAP
	touch_config_t *p_touch_config = &touch_config;
	touch_mutlcap_config_t *p_mutlcap_config
		= p_touch_config->p_mutlcap_config;


	//To be removed.
	#if 0
	touch_ret_t touch_ret = TOUCH_SUCCESS;

	/* get the global parameters from the library and transmit the global
	 * parameters to the Qtouch Studio */
	touch_ret
		= QDEBUG_GET_GLOBAL_PARAM_FUNC(&p_mutlcap_config->global_param);
	if (touch_ret != TOUCH_SUCCESS) {
		while (1) {
		}
	}
	#endif


	PutChar(QT_GLOBAL_CONFIG);
	PutChar(QDBEUG_GLOBAL_PARAM_RECAL_THRESHOLD);
	PutChar(QDBEUG_GLOBAL_PARAM_DI);
	PutChar(QDEBUG_GLOBAL_PARAM_DRIFT_HOLD_TIME);
	PutChar(QDEBUG_GLOBAL_PARAM_MAX_ON_DURATION);
	PutChar(QDEBUG_GLOBAL_PARAM_TCH_DRIFT_RATE);
	PutChar(QDEBUG_GLOBAL_PARAM_ATCH_DRIFT_RATE);
	PutChar(QDEBUG_GLOBAL_PARAM_ATCH_RECAL_DELAY);
#endif
#ifdef DEF_TOUCH_QDEBUG_ENABLE_SELFCAP
	touch_config_t *p_touch_config = &touch_config;
	touch_selfcap_config_t *p_selfcap_config
		= p_touch_config->p_selfcap_config;
	touch_ret_t touch_ret = TOUCH_SUCCESS;

	/* get the global parameters from the library and transmit the global
	 * parameters to the Qtouch Studio */
	touch_ret
		= QDEBUG_GET_GLOBAL_PARAM_FUNC(&p_selfcap_config->global_param);
	if (touch_ret != TOUCH_SUCCESS) {
		while (1) {
		}
	}

	PutChar(QT_GLOBAL_CONFIG);
	PutChar(p_selfcap_config->global_param.recal_threshold);
	PutChar(p_selfcap_config->global_param.di);
	PutChar(p_selfcap_config->global_param.drift_hold_time);
	PutChar(p_selfcap_config->global_param.max_on_duration);
	PutChar(p_selfcap_config->global_param.tch_drift_rate);
	PutChar(p_selfcap_config->global_param.atch_drift_rate);
	PutChar(p_selfcap_config->global_param.atch_recal_delay);
#endif
	PutInt(touch_time.measurement_period_ms);
	PutInt(0);                      /* TICKS_PER_MS */
	PutChar(0);             /* Time_Setting */

	Send_Message();
}

/*! \brief Transmits the channel config struct to QTouch Studio.
 */
void Transmit_Sensor_Config(void)
{
	uint8_t c;

	PutChar(QT_SENSOR_CONFIG);

	PutChar(1);             /* 1 = KRS */
	for (c = 0; c < QDEBUG_NUM_SENSORS; c++) {
		PutChar(QDBEUG_SENSOR_CONFIG_DETECT_THRESHOLD(c));
		PutChar(SENSOR_TYPE_AKS_POS_HYST(c));
		PutChar(QDEBUG_SENSOR_FROM_CHANNEL(c));
		PutChar(QDEBUG_SENSOR_TO_CHANNEL(c));
		}
	Send_Message();
}

/*! \brief Transmits the measurement values for each channel to QTouch Studio.
 */
void Transmit_Signals(void)
{
	uint8_t c;
	uint8_t length_of_mask;
	uint16_t temp_output;
	uint16_t *p_signals = (uint16_t *)QDEBUG_SIGNALS_PTR;
	PutChar(QT_SIGNALS);
	length_of_mask=((QDEBUG_NUM_CHANNELS+31)/32);
	PutChar(length_of_mask);


	for (c = 0; c < length_of_mask; c++)
	{
			temp_output = (uint16_t)(((uint32_t)QT_SURF_CHANNEL_BURST_MASK(c) & 0xFFFF0000u)>>16);
			PutInt(temp_output);
			temp_output = (uint16_t)(((uint32_t)QT_SURF_CHANNEL_BURST_MASK(c)& 0x0000FFFFu));
			PutInt(temp_output);


	}
	for (c = 0; c < QDEBUG_NUM_CHANNELS; c++) {
		if(((QT_SURF_CHANNEL_BURST_MASK(c >> 5)) & ((uint32_t)1 << (c%32))) != 0u)
		{

			PutInt(*p_signals);
		}
		else
		{
			//do nothing


		}

		p_signals++;


	}
	Send_Message();
}

/*! \brief Transmits the channel reference values to QTouch Studio.
 */
void Transmit_Ref(void)
{
	uint8_t c,length_of_mask;
	uint16_t temp_output;
	uint16_t *p_references = (uint16_t *)QDEBUG_REFERENCES_PTR;

	PutChar(QT_REFERENCES);
	length_of_mask=((QDEBUG_NUM_CHANNELS+31)/32);
	PutChar(length_of_mask);


	for (c = 0; c < length_of_mask; c++)
	{
		temp_output = (uint16_t)(((uint32_t)QT_SURF_CHANNEL_BURST_MASK(c) & 0xFFFF0000u)>>16);
		PutInt(temp_output);
		temp_output = (uint16_t)(((uint32_t)QT_SURF_CHANNEL_BURST_MASK(c)& 0x0000FFFFu));
		PutInt(temp_output);


	}

	for (c = 0; c < QDEBUG_NUM_CHANNELS; c++) {
		if(((QT_SURF_CHANNEL_BURST_MASK(c >> 5)) & ((uint32_t)1 << (c%32))) != 0u)
		{

			PutInt(*p_references);
		}
		else
		{

			//PutInt(*p_references);//do nothing

		}

		p_references++;
	}
	Send_Message();
}

/*! \brief Transmits the channel delta values to QTouch Studio.
 */
void Transmit_Delta(void)
{
	touch_ret_t touch_ret = TOUCH_SUCCESS;
	int16_t delta;
	uint16_t temp_output;
	uint8_t c,length_of_mask;

	PutChar(QT_DELTAS);
	length_of_mask=((QDEBUG_NUM_CHANNELS+31)/32);
	PutChar(length_of_mask);


	for (c = 0; c < length_of_mask; c++)
	{
		temp_output = (uint16_t)(((uint32_t)QT_SURF_CHANNEL_BURST_MASK(c) & 0xFFFF0000u)>>16);
		PutInt(temp_output);
		temp_output = (uint16_t)(((uint32_t)QT_SURF_CHANNEL_BURST_MASK(c)& 0x0000FFFFu));
		PutInt(temp_output);


	}
	for (c = 0; c < QDEBUG_NUM_SENSORS; c++) {
				if(((QT_SURF_CHANNEL_BURST_MASK(c >> 5)) & ((uint32_t)1 << (c%32))) != 0u)
				{
					touch_ret = QDEBUG_GET_DELTA_FUNC(c, &delta);
					if (touch_ret != TOUCH_SUCCESS) {
						while (1u) { /* Check API Error return code. */
						}
					}
					PutInt(delta);

				}
				else
				{

					//PutInt(0);
					//do nothing

				}
	}
	Send_Message();
}

/*! \brief Transmits the state values to QTouch Studio.
 */
void Transmit_State(void)
{
	uint8_t c;
	uint8_t *p_sensor_states = QDEBUG_SENSOR_STATES_PTR;
        #if QDEBUG_NUM_ROTORS_SLIDERS
	uint8_t *p_rotor_slider_values = QDEBUG_SENSOR_RS_VALUES;
        #endif

	PutChar(QT_STATES);
	PutChar(QDEBUG_NUM_CHANNELS);
	PutChar(QDEBUG_NUM_ROTORS_SLIDERS);

	for (c = 0; c < QDEBUG_NUM_SENSOR_STATE_BYTES; c++) {
		PutChar(p_sensor_states[c]);
	}
#if QDEBUG_NUM_ROTORS_SLIDERS
	for (c = 0; c < QDEBUG_NUM_ROTORS_SLIDERS; c++) {
		PutChar(p_rotor_slider_values[c]);
	}
#endif
	Send_Message();
}

/*! \brief Transmits the application execution timestamp values to QTouch
 * Studio.
 * \note This value is a combination of current_time_ms_touch (high word) &
 * timer counter register (low word).
 */
void Transmit_Time_Stamps(void)
{
	PutInt(timestamp1_hword);
	PutInt(timestamp1_lword);
	PutInt(timestamp2_hword);
	PutInt(timestamp2_lword);
	PutInt(timestamp3_hword);
	PutInt(timestamp3_lword);
	Send_Message();
}

/*! \brief Transmits user data to QTouch Studio.
 * \param pdata: data pointer.
 * \param c: length of data in bytes.
 * \note The data will be binary data.
 */
void Transmit_QT_User_Data(uint8_t *pdata, uint16_t c)
{
	while (c > 0) {
		PutChar(*pdata++);
		c--;
	}
	Send_Message();
}
#endif
