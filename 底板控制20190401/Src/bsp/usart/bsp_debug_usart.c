/**
  ******************************************************************************
  * �ļ�����: main.c 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2017-03-30
  * ��    ��: ���ص��Դ��ڵײ���������Ĭ��ʹ��USART1
  ******************************************************************************
  * ˵����
  * ����������Ӳʯstm32������YS-F4Proʹ�á�
  * 
  * �Ա���
  * ��̳��http://www.ing10bbs.com
  * ��Ȩ��ӲʯǶ��ʽ�����Ŷ����У��������á�
  ******************************************************************************
  */
#include "usart/bsp_debug_usart.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
UART_HandleTypeDef husart_debug;
UART_HandleTypeDef husart_debug_485;
__IO uint8_t Rx_Buf[256];            // ���ջ���,���256�ֽ�
__IO uint8_t Tx_Buf[256];            // ���ͻ���,���256�ֽ�
__IO uint8_t tmp_Rx_Buf;             // ��ʱ���ջ���
__IO uint16_t RxCount = 0;      // �����ַ�����
/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/

/**
  * ��������: ����Ӳ����ʼ������
  * �������: huart�����ھ������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
  */
void HAL_UART_MspInit(UART_HandleTypeDef* huart)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  if(huart->Instance==DEBUG_USARTx)
  {
    /* ��������ʱ��ʹ�� */
    DEBUG_USART_RCC_CLK_ENABLE();
  
    /* �������蹦��GPIO���� */
    GPIO_InitStruct.Pin = DEBUG_USARTx_Tx_GPIO_PIN|DEBUG_USARTx_Rx_GPIO_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
  }
  else if(huart->Instance==USART3)
  {
    //��������ʱ��ʹ��
    __HAL_RCC_USART3_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    //�������蹦��GPIO����
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART3;    
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);    
    
    //SP3485E��������ʹ�ܿ������ų�ʼ��
    HAL_GPIO_WritePin(GPIOH,GPIO_PIN_8,GPIO_PIN_RESET);
    GPIO_InitStruct.Pin = GPIO_PIN_8;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
  }   
}

/**
  * ��������: ����Ӳ������ʼ������
  * �������: huart�����ھ������ָ��
  * �� �� ֵ: ��
  * ˵    ��: �ú�����HAL���ڲ�����
  */
void HAL_UART_MspDeInit(UART_HandleTypeDef* huart)
{

  if(huart->Instance==DEBUG_USARTx)
  {
    /* ��������ʱ�ӽ��� */
    DEBUG_USART_RCC_CLK_DISABLE();
  
    /* �������蹦��GPIO���� */
    HAL_GPIO_DeInit(DEBUG_USARTx_Tx_GPIO, DEBUG_USARTx_Tx_GPIO_PIN);
    HAL_GPIO_DeInit(DEBUG_USARTx_Rx_GPIO, DEBUG_USARTx_Rx_GPIO_PIN);
    
    /* �����жϽ��� */
    HAL_NVIC_DisableIRQ(DEBUG_USART_IRQn);
  }
  else if(huart->Instance==USART3)
  {
    /* ��������ʱ�ӽ��� */
    DEBUG_USART_RCC_CLK_DISABLE();
  
    /* �������蹦��GPIO���� */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10);
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_11);
    
    /* �����жϽ��� */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  }
}

/**
  * ��������: ���ڲ�������.
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
void MX_DEBUG_USART_Init(void)
{
  /* ʹ�ܴ��ڹ�������GPIOʱ�� */
  DEBUG_USARTx_GPIO_ClK_ENABLE();
  
  husart_debug.Instance = DEBUG_USARTx;
  husart_debug.Init.BaudRate = DEBUG_USARTx_BAUDRATE;
  husart_debug.Init.WordLength = UART_WORDLENGTH_8B;
  husart_debug.Init.StopBits = UART_STOPBITS_1;
  husart_debug.Init.Parity = UART_PARITY_NONE;
  husart_debug.Init.Mode = UART_MODE_TX_RX;
  husart_debug.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  husart_debug.Init.OverSampling = UART_OVERSAMPLING_16;
  HAL_UART_Init(&husart_debug);
  
  //ʹ�ܴ��ڹ�������GPIOʱ�� 
  __HAL_RCC_USART3_CLK_ENABLE();
  printf("1\n");
  husart_debug_485.Instance = USART3;
  husart_debug_485.Init.BaudRate = DEBUG_USART3_BAUDRATE;
  husart_debug_485.Init.WordLength = UART_WORDLENGTH_9B;
  husart_debug_485.Init.StopBits = UART_STOPBITS_1;
  husart_debug_485.Init.Parity = UART_PARITY_EVEN;
  husart_debug_485.Init.Mode = UART_MODE_TX_RX;
  husart_debug_485.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  husart_debug_485.Init.OverSampling = UART_OVERSAMPLING_16;
  printf("1.1\n");
  HAL_UART_Init(&husart_debug_485);
  printf("2\n");
  HAL_NVIC_SetPriority(USART3_IRQn,0,0);
  HAL_NVIC_EnableIRQ(USART3_IRQn);
  printf("3\n");
  //�����ж�,�洢����ʱ������
  HAL_UART_Receive_IT(&husart_debug_485,(uint8_t*)&tmp_Rx_Buf,1);
  printf("4\n");
}
/**
  * ��������: ���ڷ��ͺ���
  * �������: Tx_Bul:�����ַ���,TxCount:���͵��ֽ���
  * �� �� ֵ: ��
  * ˵    ��: ���⺯�������װ.
  */
void UART_Tx(uint8_t *Tx_Buf,uint16_t TxCount)
{
  RS485_TX_MODE();
  HAL_UART_Transmit(&husart_debug_485, Tx_Buf, TxCount, 0xffff);
  RS485_RX_MODE();
}
/**
  * ��������: �ض���c�⺯��printf��DEBUG_USARTx
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
int fputc(int ch, FILE *f)
{
  HAL_UART_Transmit(&husart_debug, (uint8_t *)&ch, 1, 0xffff);
  return ch;
}

/**
  * ��������: �ض���c�⺯��getchar,scanf��DEBUG_USARTx
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
int fgetc(FILE * f)
{
  uint8_t ch = 0;
  HAL_UART_Receive(&husart_debug,&ch, 1, 0xffff);
  return ch;
}

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
