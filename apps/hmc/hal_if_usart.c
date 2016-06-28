#include <asf.h>
#include <stdio.h>

#include "hal_if_usart.h"

static void usart_read_callback(struct usart_module *const usart_module);
static void usart_write_callback(struct usart_module *const usart_module);

static void configure_usart(void);
static void configure_usart_callbacks(void);

//! [module_inst]
static struct usart_module usart_instance;
//! [module_inst]

//! [rx_buffer_var]
#define MAX_RX_BUFFER_LENGTH   128

volatile uint8_t rx_buffer[MAX_RX_BUFFER_LENGTH];
//! [rx_buffer_var]

//! [callback_funcs]
void usart_read_callback(struct usart_module *const usart_module)
{
	usart_write_buffer_job(&usart_instance,
			(uint8_t *)rx_buffer, MAX_RX_BUFFER_LENGTH);
}

void usart_write_callback(struct usart_module *const usart_module)
{
	port_pin_toggle_output_level(LED_0_PIN);
}
//! [callback_funcs]

//! [setup]
void configure_usart(void)
{
//! [setup_config]
	struct usart_config config_usart;
//! [setup_config]
//! [setup_config_defaults]
	usart_get_config_defaults(&config_usart);
//! [setup_config_defaults]

//! [setup_change_config]
	config_usart.baudrate    = 9600;
	config_usart.mux_setting = EDBG_CDC_SERCOM_MUX_SETTING;
	config_usart.pinmux_pad0 = EDBG_CDC_SERCOM_PINMUX_PAD0;
	config_usart.pinmux_pad1 = EDBG_CDC_SERCOM_PINMUX_PAD1;
	config_usart.pinmux_pad2 = EDBG_CDC_SERCOM_PINMUX_PAD2;
	config_usart.pinmux_pad3 = EDBG_CDC_SERCOM_PINMUX_PAD3;
//! [setup_change_config]

//! [setup_set_config]
	while (usart_init(&usart_instance,
			EDBG_CDC_MODULE, &config_usart) != STATUS_OK) {
	}
//! [setup_set_config]

//! [setup_enable]
	usart_enable(&usart_instance);
//! [setup_enable]
}

uint8_t hal_if_usart_send(uint8_t *data, uint16_t len)
{
    return usart_write_buffer_wait(&usart_instance, data, len);
}

void configure_usart_callbacks(void)
{
//! [setup_register_callbacks]
	usart_register_callback(&usart_instance,
			usart_write_callback, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_register_callback(&usart_instance,
			usart_read_callback, USART_CALLBACK_BUFFER_RECEIVED);
//! [setup_register_callbacks]

//! [setup_enable_callbacks]
	usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_TRANSMITTED);
	usart_enable_callback(&usart_instance, USART_CALLBACK_BUFFER_RECEIVED);
//! [setup_enable_callbacks]
}
//! [setup]

void hal_if_usart_init(void)
{
//! [setup_init]
	configure_usart();
	configure_usart_callbacks();
//! [setup_init]
}

