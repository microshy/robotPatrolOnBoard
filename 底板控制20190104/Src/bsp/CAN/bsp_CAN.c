/**
  ******************************************************************************
  * �ļ�����: bsp_debug_usart.c 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2015-10-04
  * ��    ��: ���ص��Դ��ڵײ���������Ĭ��ʹ��USART1
  ******************************************************************************
  * ˵����
  * ����������Ӳʯstm32������YS-F1Proʹ�á�
  * 
  * �Ա���
  * ��̳��http://www.ing10bbs.com
  * ��Ȩ��ӲʯǶ��ʽ�����Ŷ����У��������á�
  ******************************************************************************
  */

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "CAN/bsp_CAN.h"
#include "udp_echoclient.h"

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
CAN_HandleTypeDef hCAN;
static CanTxMsgTypeDef        TxMessage;
static CanRxMsgTypeDef        RxMessage;
struct MOTORpar MOTOR1,MOTOR2,MOTOR3,MOTOR4;

/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
/* ������ --------------------------------------------------------------------*/
/**
  * ��������: CAN�����ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void MX_CAN_Init(void)
{
  CAN_FilterConfTypeDef  sFilterConfig;
  
  /*CAN��Ԫ��ʼ��*/
  hCAN.Instance = CANx;             /* CAN���� */
  hCAN.pTxMsg = &TxMessage;
  hCAN.pRxMsg = &RxMessage;
  
  hCAN.Init.Prescaler = 6;          /* BTR-BRP �����ʷ�Ƶ��  ������ʱ�䵥Ԫ��ʱ�䳤�� 0.5Mbps */
  hCAN.Init.Mode = CAN_MODE_NORMAL; /* ��������ģʽ */
  hCAN.Init.SJW = CAN_SJW_1TQ;      /* BTR-SJW ����ͬ����Ծ��� 1��ʱ�䵥Ԫ */
  hCAN.Init.BS1 = CAN_BS1_6TQ;      /* BTR-TS1 ʱ���1 ռ����8��ʱ�䵥Ԫ */
  hCAN.Init.BS2 = CAN_BS2_7TQ;      /* BTR-TS1 ʱ���2 ռ����6��ʱ�䵥Ԫ */
  hCAN.Init.TTCM = DISABLE;         /* MCR-TTCM  �ر�ʱ�䴥��ͨ��ģʽʹ�� */
  hCAN.Init.ABOM = ENABLE;          /* MCR-ABOM  �Զ����߹��� */
  hCAN.Init.AWUM = ENABLE;          /* MCR-AWUM  ʹ���Զ�����ģʽ */
  hCAN.Init.NART = DISABLE;         /* MCR-NART  ��ֹ�����Զ��ش�	  DISABLE-�Զ��ش� */
  hCAN.Init.RFLM = DISABLE;         /* MCR-RFLM  ����FIFO ����ģʽ  DISABLE-���ʱ�±��ĻḲ��ԭ�б��� */
  hCAN.Init.TXFP = DISABLE;         /* MCR-TXFP  ����FIFO���ȼ� DISABLE-���ȼ�ȡ���ڱ��ı�ʾ�� */
  HAL_CAN_Init(&hCAN);
  
  /*CAN��������ʼ��*/
  sFilterConfig.FilterNumber = 0;                    /* ��������0 */
  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;  /* �����ڱ�ʶ������λģʽ */
  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT; /* ������λ��Ϊ����32λ��*/
  /* ʹ�ܱ��ı�ʾ�����������ձ�ʾ�������ݽ��бȶԹ��ˣ���չID�������µľ����������ǵĻ��������FIFO0�� */
  
  sFilterConfig.FilterIdHigh         = 0x0000;				/* Ҫ���˵�ID��λ */
  sFilterConfig.FilterIdLow          = 0x0000; /* Ҫ���˵�ID��λ */
  sFilterConfig.FilterMaskIdHigh     = 0x0000;			/* ��������16λÿλ����ƥ�� */
  sFilterConfig.FilterMaskIdLow      = 0x0000;			/* ��������16λÿλ����ƥ�� */
  sFilterConfig.FilterFIFOAssignment = 0;           /* ��������������FIFO 0 */
  sFilterConfig.FilterActivation = ENABLE;          /* ʹ�ܹ����� */ 
  sFilterConfig.BankNumber = 14;
  HAL_CAN_ConfigFilter(&hCAN, &sFilterConfig);
}

/**
  * ��������: CAN���ų�ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void HAL_CAN_MspInit(CAN_HandleTypeDef* hcan)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(hcan->Instance==CANx)
  {
    /* Peripheral clock enable */
    CANx_CLK_ENABLE();
    CANx_GPIO_CLK_ENABLE();
    
    /**CAN GPIO Configuration    
    PI9     ------> CAN_RX
    PB9     ------> CAN_TX 
    */
    GPIO_InitStruct.Pin = CANx_TX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Alternate=GPIO_AF9_CAN1;
    HAL_GPIO_Init(CANx_TX_GPIO_PORT, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = CANx_RX_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate=GPIO_AF9_CAN1;
    HAL_GPIO_Init(CANx_RX_GPIO_PORT, &GPIO_InitStruct);


    /* ��ʼ���ж����ȼ� */
    HAL_NVIC_SetPriority(CANx_RX_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(CANx_RX_IRQn);
  }
}

/**
  * ��������: CAN���ŷ���ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void HAL_CAN_MspDeInit(CAN_HandleTypeDef* hcan)
{

  if(hcan->Instance==CANx)
  {
    CANx_FORCE_RESET();
    CANx_RELEASE_RESET();
  
    /**CAN GPIO Configuration    
    PB8     ------> CAN_RX
    PB9     ------> CAN_TX 
    */
    HAL_GPIO_DeInit(CANx_TX_GPIO_PORT, CANx_TX_PIN);
    HAL_GPIO_DeInit(CANx_RX_GPIO_PORT, CANx_RX_PIN);

    /* Peripheral interrupt Deinit*/
    HAL_NVIC_DisableIRQ(CANx_RX_IRQn);
  }
} 


/**
  * ��������: CAN��������жϻص��������
  * �������: hcan��CAN������ָ��
  * �� �� ֵ: �� CAN1_RX0_IRQHandler
  * ˵    ������ HAL_CAN_RxCpltCallback
  */
void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef *hcan)
{
  
  uint16_t i;
  //printf( "\n�ɹ����յ����ӻ������ص�����\n ");	
  //printf("���յ��ı���Ϊ��\n"); 
  //printf("��ID��StdId��0x%x\n",hCAN.pRxMsg->StdId);
  data[1] = hCAN.pRxMsg->StdId;
  data[0] = hCAN.pRxMsg->StdId>>8;
  udp_send_t(2);
  //printf("�����յ����ݶγ��ȣ�%d\n",hCAN.pRxMsg->DLC);
  if(hCAN.pRxMsg->DLC!=0)
  {
    //printf("��0x");
    for(i=0;i<hCAN.pRxMsg->DLC;i++)
    {
      //printf("%X\t",hCAN.pRxMsg->Data[i]);
      data[i] = hCAN.pRxMsg->Data[i];
    }
    //printf("\n");
    udp_send_t(hCAN.pRxMsg->DLC);
  }
  //printf("�����ݶε�����:Data[0]= 0x%X ��Data[1]=0x%X \n",hCAN.pRxMsg->Data[0],hCAN.pRxMsg->Data[1]);
  HAL_CAN_Receive_IT(hcan, CAN_FIFO0);
}

/**
  * ��������: CANͨ�ű�����������
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ������
  */
void CAN_SetTxMsg(void)
{
  //hCAN.pTxMsg->StdId=0x0000;
  //hCAN.pTxMsg->IDE=CAN_ID_STD;					 /* ��׼ģʽ */
  hCAN.pTxMsg->ExtId=0x1314;					   /* ʹ�õ���չID */
  hCAN.pTxMsg->IDE=CAN_ID_EXT;					 /* ��չģʽ */
  hCAN.pTxMsg->RTR=CAN_RTR_DATA;				 /* ���͵������� */
  hCAN.pTxMsg->DLC=2;							       /* ���ݳ���Ϊ2�ֽ� */
  hCAN.pTxMsg->Data[0]=0x01;
  hCAN.pTxMsg->Data[1]=0x00;
}



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
