#ifndef __BSP_DEBUG_USART_H__
#define __BSP_DEBUG_USART_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdio.h>

/* ���Ͷ��� ------------------------------------------------------------------*/
/* �궨�� --------------------------------------------------------------------*/
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
/* ʹ��485ͨ�ŵ�ʱ��Ż��õ�ʹ��IO */
#define RS485_REDE_GPIO_ClK_ENABLE()                 __HAL_RCC_GPIOH_CLK_ENABLE()
#define RS485_REDE_PORT                              GPIOH
#define RS485_REDE_PIN                               GPIO_PIN_8
#define RS485_RX_MODE()                              HAL_GPIO_WritePin(RS485_REDE_PORT,RS485_REDE_PIN,GPIO_PIN_RESET)
#define RS485_TX_MODE()                              HAL_GPIO_WritePin(RS485_REDE_PORT,RS485_REDE_PIN,GPIO_PIN_SET)
/* RTUͨ����Ҫȷ����ʱʱ�� */
#if DEBUG_USART3_BAUDRATE <= 19200
  /* 1.5���ַ��ĳ�ʱʱ�� T = BAUDRATE/11/1000*/
  #define OVERTIME_15CHAR             1800//1562//((float)(30/(DEBUG_USART3_BAUDRATE*2))*1000000)//(((float)DEBUG_USART3_BAUDRATE/11)*1.5f)
  /* 3���ַ��ĳ�ʱʱ�� */
  #define OVERTIME_35CHAR             4200//3654//((float)(70/(DEBUG_USART3_BAUDRATE*2))*1000000)//(((float)DEBUG_USART3_BAUDRATE/11)*3.5f)
#else 
  /* �����ʳ���19200bit/s������½���ĳ�ʱʱ�� */
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
/* ��չ���� ------------------------------------------------------------------*/
extern UART_HandleTypeDef husart_debug;
extern UART_HandleTypeDef husart_debug_485;
extern __IO uint8_t Rx_Buf[256];    // ���ջ���,���256�ֽ�
extern __IO uint8_t Tx_Buf[256];    // ���ջ���,���256�ֽ�
extern __IO uint8_t tmp_Rx_Buf;     // ���ջ���
extern  __IO uint16_t RxCount;      // �����ַ�����
/* �������� ------------------------------------------------------------------*/
void MX_DEBUG_USART_Init(void);
void UART_Tx(uint8_t *Tx_Buf,uint16_t TxCount);

#endif  /* __BSP_DEBUG_USART_H__ */

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
