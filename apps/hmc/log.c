#include <stdarg.h>
#include <string.h>

#include "mua_types.h"
#include "log.h"

#if defined STM32F4
#include "hal_if_uart.h"
#endif

#if defined __SAMD__
#include "hal_if_usart.h"
#endif


#define DEBUG_UART      0


#define LOG_TRIES_MAX   3

#if defined TI_CC2541
/* Prototype defined in 8051/src/lib/clib/icclbutl.h */
extern int _formatted_write(const char *, void (*)(char, void *), void *, va_list);

static void logPutOneChar(char c, void *dummy)
{
    (void)dummy;  /* Warning on this line OK (Optimized Away) */
}

/* Our main entry */
int logPrintf(const char *format, ...)
{
    va_list ap;
    int nr_of_chars;

#if DEBUG_UART
    P1_1 = !P1_1;
#endif

    va_start(ap, format);      /* Variable argument begin */
    nr_of_chars = _formatted_write(format, logPutOneChar, (void *)0, ap);
    va_end(ap);                /* Variable argument end */

    return nr_of_chars;        /* According to ANSI */
}
/* TI_CC2541 */

#elif defined STM32F4

__IO ITStatus UartReady = RESET;
static uint8_t sBuf[LOG_BUFFER_SIZE];

int logPrintf(const char *fmt, ...)
{
    va_list args;
    int nr_of_chars;

    va_start(args, fmt);
    memset(sBuf, 0, sizeof(sBuf));
    nr_of_chars = vsnprintf((char *)sBuf, LOG_BUFFER_SIZE - 1, fmt, args);
    HALIF_UART_Write(sBuf, strlen((char *)sBuf));
    va_end(args);

    return nr_of_chars;        /* According to ANSI */
}

#elif defined __SAMD__

static uint8_t sBuf[LOG_BUFFER_SIZE];

int logPrintf(const char *fmt, ...)
{
    va_list args;
    int nr_of_chars;

    va_start(args, fmt);
    memset(sBuf, 0, sizeof(sBuf));
    nr_of_chars = vsnprintf((char *)sBuf, LOG_BUFFER_SIZE - 1, fmt, args);
    hal_if_usart_send(sBuf, strlen((char *)sBuf));
    va_end(args);

    return nr_of_chars;        /* According to ANSI */
}

#endif /* STM32F4 */

void Log_Init(void)
{
}

/* vim: set ts=4 sw=4 tw=0 list : */
