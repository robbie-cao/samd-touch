#ifndef __LOG_H__
#define __LOG_H__

#if defined TI_CC3200
#include "uart_if.h"
#define logPrintf   Report
#endif /* TI_CC3200 */

#if defined STM32F4
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#if defined STM32F401xC
#include "stm32f401_discovery.h"
#else
#include "stm32f4_discovery.h"
#endif

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/
/* User can use this section to tailor USARTx/UARTx instance used and associated
 *    resources */
/* Definition for USARTx clock resources */

#define USARTx                           USART2
#define USARTx_BANDRATE                  115200
#define USARTx_CLK_ENABLE()              __HAL_RCC_USART2_CLK_ENABLE();
#define DMAx_CLK_ENABLE()                __HAL_RCC_DMA1_CLK_ENABLE()
#define USARTx_RX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()
#define USARTx_TX_GPIO_CLK_ENABLE()      __HAL_RCC_GPIOA_CLK_ENABLE()

#define USARTx_FORCE_RESET()             __HAL_RCC_USART2_FORCE_RESET()
#define USARTx_RELEASE_RESET()           __HAL_RCC_USART2_RELEASE_RESET()

/* Definition for USARTx Pins */
#define USARTx_TX_PIN                    GPIO_PIN_2
#define USARTx_TX_GPIO_PORT              GPIOA
#define USARTx_TX_AF                     GPIO_AF7_USART2
#define USARTx_RX_PIN                    GPIO_PIN_3
#define USARTx_RX_GPIO_PORT              GPIOA
#define USARTx_RX_AF                     GPIO_AF7_USART2

/* Definition for USARTx's DMA */
#define USARTx_TX_DMA_CHANNEL            DMA_CHANNEL_4
#define USARTx_TX_DMA_STREAM             DMA1_Stream6
#define USARTx_RX_DMA_CHANNEL            DMA_CHANNEL_4
#define USARTx_RX_DMA_STREAM             DMA1_Stream5

/* Definition for USARTx's NVIC */
#define USARTx_DMA_TX_IRQn               DMA1_Stream6_IRQn
#define USARTx_DMA_RX_IRQn               DMA1_Stream5_IRQn
#define USARTx_DMA_TX_IRQHandler         DMA1_Stream6_IRQHandler
#define USARTx_DMA_RX_IRQHandler         DMA1_Stream5_IRQHandler
#define USARTx_IRQn                      USART2_IRQn
#define USARTx_IRQHandler                USART2_IRQHandler

/* Size of Transmission buffer */
#define TXBUFFERSIZE                     (COUNTOF(aTxBuffer) - 1)
/* Size of Reception buffer */
#define RXBUFFERSIZE                     TXBUFFERSIZE

/* Exported macro ------------------------------------------------------------*/
#define COUNTOF(__BUFFER__)   (sizeof(__BUFFER__) / sizeof(*(__BUFFER__)))
#endif /* STM32F4 */

#define LOG_BUFFER_SIZE                  128

/**
 * Log Level:
 * - VERBOSE
 * - INFO
 * - DEBUG
 * - WARN
 * - ERROR
 *
 * Priority increase from VERBOSE to ERROR<br>
 * Lower priority log level will include higher priority log<br>
 */

/* WARN/ERROR log default ON */
#ifndef LOG_LEVEL_WARN
#define LOG_LEVEL_WARN
#endif
#ifndef LOG_LEVEL_ERROR
#define LOG_LEVEL_ERROR
#endif

/* Turn off all level log if UART not enabled */
#if (defined TI_CC2541) && (!defined HAL_UART || HAL_UART == FALSE)
#undef  LOG_LEVEL_VERBOSE
#undef  LOG_LEVEL_INFO
#undef  LOG_LEVEL_DEBUG
#undef  LOG_LEVEL_WARN
#undef  LOG_LEVEL_ERROR
#ifndef LOG_LEVEL_NONE
#define LOG_LEVEL_NONE
#endif
#endif

#if   defined LOG_LEVEL_NONE    // NONE
#undef  LOG_LEVEL_VERBOSE
#undef  LOG_LEVEL_INFO
#undef  LOG_LEVEL_DEBUG
#undef  LOG_LEVEL_WARN
#undef  LOG_LEVEL_ERROR
#elif defined LOG_LEVEL_VERBOSE // VERBOSE
#define LOG_LEVEL_INFO
#define LOG_LEVEL_DEBUG
#elif defined LOG_LEVEL_INFO    // INFO
#undef  LOG_LEVEL_VERBOSE
#define LOG_LEVEL_DEBUG
#elif defined LOG_LEVEL_DEBUG   // DEBUG
#undef  LOG_LEVEL_VERBOSE
#undef  LOG_LEVEL_INFO
#elif defined LOG_LEVEL_WARN    // WARN
#undef  LOG_LEVEL_VERBOSE
#undef  LOG_LEVEL_INFO
#undef  LOG_LEVEL_DEBUG
#elif defined LOG_LEVEL_ERROR   // ERROR
#undef  LOG_LEVEL_VERBOSE
#undef  LOG_LEVEL_INFO
#undef  LOG_LEVEL_DEBUG
#undef  LOG_LEVEL_WARN
#else                           // DEFAULT
#undef  LOG_LEVEL_VERBOSE
#undef  LOG_LEVEL_INFO
#undef  LOG_LEVEL_DEBUG
#endif

#define LOG_LEVEL_SYMBOL_VERBOSE        "V"
#define LOG_LEVEL_SYMBOL_INFO           "I"
#define LOG_LEVEL_SYMBOL_DEBUG          "D"
#define LOG_LEVEL_SYMBOL_WARN           "W"
#define LOG_LEVEL_SYMBOL_ERROR          "E"

#ifdef LOG_LEVEL_NONE
#define LOG(fmt, arg...)
#else
#if defined TI_CC2541
#define LOG(fmt, arg...)                 logPrintf(fmt, ##arg)
#elif defined TI_CC3200 || defined STM32F4 || defined __SAMD__
#define LOG(fmt, ...)                   logPrintf(fmt, ##__VA_ARGS__)
#else
#warn "LOG not support!"
#define LOG(fmt, arg...)
#endif
#endif

#ifdef LOG_LEVEL_VERBOSE
#if defined TI_CC2541
#define LOGV(tag, fmt, arg...)          logPrintf(LOG_LEVEL_SYMBOL_VERBOSE ## "\t" ## tag ## "\t" fmt, ## arg)
#elif defined TI_CC3200 || defined STM32F4 || defined __SAMD__
#define LOGV(tag, fmt, ...)             logPrintf(LOG_LEVEL_SYMBOL_VERBOSE "\t" tag "\t" fmt, ## __VA_ARGS__)
#else
#warn "LOG not support!"
#define LOGV(fmt, arg...)
#endif
#else
#define LOGV(tag, fmt, arg...)
#endif

#ifdef LOG_LEVEL_INFO
#if defined TI_CC2541
#define LOGI(tag, fmt, arg...)          logPrintf(LOG_LEVEL_SYMBOL_INFO    ## "\t" ## tag ## "\t" fmt, ## arg)
#elif defined TI_CC3200 || defined STM32F4 || defined __SAMD__
#define LOGI(tag, fmt, ...)             logPrintf(LOG_LEVEL_SYMBOL_INFO    "\t" tag "\t" fmt, ## __VA_ARGS__)
#else
#warn "LOG not support!"
#define LOGI(fmt, arg...)
#endif
#else
#define LOGI(tag, fmt, arg...)
#endif

#ifdef LOG_LEVEL_DEBUG
#if defined TI_CC2541
#define LOGD(tag, fmt, arg...)          logPrintf(LOG_LEVEL_SYMBOL_DEBUG   ## "\t" ## tag ## "\t" fmt, ## arg)
#elif defined TI_CC3200 || defined STM32F4 || defined __SAMD__
#define LOGD(tag, fmt, ...)             logPrintf(LOG_LEVEL_SYMBOL_DEBUG   "\t" tag "\t" fmt, ## __VA_ARGS__)
#else
#warn "LOG not support!"
#define LOGD(fmt, arg...)
#endif
#else
#define LOGD(tag, fmt, arg...)
#endif

#ifdef LOG_LEVEL_WARN
#if defined TI_CC2541
#define LOGW(tag, fmt, arg...)          logPrintf(LOG_LEVEL_SYMBOL_WARN    ## "\t" ## tag ## "\t" fmt, ## arg)
#elif defined TI_CC3200 || defined STM32F4 || defined __SAMD__
#define LOGW(tag, fmt, ...)             logPrintf(LOG_LEVEL_SYMBOL_WARN    "\t" tag "\t" fmt, ## __VA_ARGS__)
#else
#warn "LOG not support!"
#define LOGW(fmt, arg...)
#endif
#else
#define LOGW(tag, fmt, arg...)
#endif

#ifdef LOG_LEVEL_ERROR
#if defined TI_CC2541
#define LOGE(tag, fmt, arg...)          logPrintf(LOG_LEVEL_SYMBOL_ERROR   ## "\t" ## tag ## "\t" fmt, ## arg)
#elif defined TI_CC3200 || defined STM32F4 || defined __SAMD__
#define LOGE(tag, fmt, ...)             logPrintf(LOG_LEVEL_SYMBOL_ERROR   "\t" tag "\t" fmt, ## __VA_ARGS__)
#else
#warn "LOG not support!"
#define LOGE(fmt, arg...)
#endif
#else
#define LOGE(tag, fmt, arg...)
#endif

/*
 * Another API for disable log in code level.
 * It will not output log in trace.
 */
#define __LOG(fmt, arg...)
#define __LOGV(tag, fmt, arg...)
#define __LOGI(tag, fmt, arg...)
#define __LOGD(tag, fmt, arg...)
#define __LOGW(tag, fmt, arg...)
#define __LOGE(tag, fmt, arg...)

/**
 * Another API for quick debug
 * It will output line and function name
 */
#define D()                             logPrintf("* L%d, %s\r\n", __LINE__, __FUNCTION__)
#define DD(fmt, arg...)                 logPrintf("* L%d, %s: " fmt, __LINE__, __FUNCTION__, ##arg)
#define DDD(fmt, arg...)                logPrintf("* L%d: " fmt, __LINE__, ##arg)

#define DEBUG_UART_PORT                 UART_PORT

/* Public API for log */
#ifndef logPrintf
int logPrintf(const char *format, ...);
#endif

void Log_Init(void);

#endif /* __LOG_H__ */

/* vim: set ts=4 sw=4 tw=0 list : */
