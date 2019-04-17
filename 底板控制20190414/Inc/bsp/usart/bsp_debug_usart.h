#ifndef __BSP_DEBUG_USART_H__
#define __BSP_DEBUG_USART_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdio.h>

/* 类型定义 ------------------------------------------------------------------*/
/* 宏定义 --------------------------------------------------------------------*/
#define DEBUG_USARTx                                 USART1
#define DEBUG_USARTx_BAUDRATE                        115200
#define DEBUG_USART_RCC_CLK_ENABLE()                 __HAL_RCC_USART1_CLK_ENABLE()
#define DEBUG_USART_RCC_CLK_DISABLE()                __HAL_RCC_USART1_CLK_DISABLE()

#define DEBUG_USARTx_GPIO_ClK_ENABLE()               __HAL_RCC_GPIOB_CLK_ENABLE()
#define DEBUG_USARTx_Tx_GPIO_PIN                     GPIO_PIN_6
#define DEBUG_USARTx_Tx_GPIO                         GPIOB
#define DEBUG_USARTx_Rx_GPIO_PIN                     GPIO_PIN_7
#define DEBUG_USARTx_Rx_GPIO                         GPIOB

#define DEBUG_USART_IRQn                             USART1_IRQn


#define DEBUG_USART3_BAUDRATE                        9600//19200
/* 使用485通信的时候才会用到使能IO */
#define RS485_REDE_GPIO_ClK_ENABLE()                 __HAL_RCC_GPIOH_CLK_ENABLE()
#define RS485_REDE_PORT                              GPIOH
#define RS485_REDE_PIN                               GPIO_PIN_8
#define RS485_RX_MODE()                              HAL_GPIO_WritePin(RS485_REDE_PORT,RS485_REDE_PIN,GPIO_PIN_RESET)
#define RS485_TX_MODE()                              HAL_GPIO_WritePin(RS485_REDE_PORT,RS485_REDE_PIN,GPIO_PIN_SET)
/* RTU通信需要确定超时时间 */
#if DEBUG_USART3_BAUDRATE <= 19200
  /* 1.5个字符的超时时间 T = BAUDRATE/11/1000*/
  #define OVERTIME_15CHAR             1800//1562//((float)(30/(DEBUG_USART3_BAUDRATE*2))*1000000)//(((float)DEBUG_USART3_BAUDRATE/11)*1.5f)
  /* 3个字符的超时时间 */
  #define OVERTIME_35CHAR             4200//3654//((float)(70/(DEBUG_USART3_BAUDRATE*2))*1000000)//(((float)DEBUG_USART3_BAUDRATE/11)*3.5f)
#else 
  /* 波特率超过19200bit/s的情况下建议的超时时间 */
  #define OVERTIME_15CHAR                750.0f    // 750us 
  #define OVERTIME_35CHAR               1750.0f  // 1.75ms
#endif

struct Battery_State
{
  uint16_t alarm_State;
  uint16_t temper;
  uint16_t power;
  uint16_t voltage;
  uint16_t current;
};
/* 扩展变量 ------------------------------------------------------------------*/
extern UART_HandleTypeDef husart_debug;
extern UART_HandleTypeDef husart_debug_485;
extern __IO uint8_t Rx_Buf[256];    // 接收缓存,最大256字节
extern __IO uint8_t Tx_Buf[256];    // 接收缓存,最大256字节
extern __IO uint8_t tmp_Rx_Buf;     // 接收缓存
extern  __IO uint16_t RxCount;      // 接收字符计数
/* 函数声明 ------------------------------------------------------------------*/
void MX_DEBUG_USART_Init(void);
void UART_Tx(uint8_t *Tx_Buf,uint16_t TxCount);

#endif  /* __BSP_DEBUG_USART_H__ */

/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
