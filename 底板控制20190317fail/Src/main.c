/**
  ******************************************************************************
  * �ļ�����: main.c 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2017-03-30
  * ��    ��: UDP_Client
  ******************************************************************************
  * ˵����
  * ����������Ӳʯstm32������YS-F4Proʹ�á�
  * 
  * �Ա���
  * ��̳��http://www.ing10bbs.com
  * ��Ȩ��ӲʯǶ��ʽ�����Ŷ����У��������á�
  ******************************************************************************
  */
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"
#include "string.h"
#include "usart/bsp_debug_usart.h"
#include "key/bsp_key.h"
#include "CAN/bsp_CAN.h"

#include "lwip/opt.h"
#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/timeouts.h"
#include "netif/etharp.h"
#include "ethernetif.h"
#include "app_ethernet.h"
#include "udp_echoclient.h"
#include "dhcp.h"

#include "GeneralTIM/bsp_GeneralTIM.h"
#include "MB-host/bsp_MB_host.h"
#define MSG_ERR_FLAG  0xFFFF    // ���մ��� �ַ��䳬ʱ
#define MSG_IDLE      0x0000    // ����״̬
#define MSG_RXING     0x0001    // ���ڽ�������
#define MSG_COM       0x0002    // �������
#define MSG_INC       0x8000    // ����֡������(���ַ���Ŀ��м������1.5���ַ�ʱ��)
#define TIME_OVERRUN  100       // ���峬ʱʱ�� ms


__IO uint16_t Rx_MSG = MSG_IDLE;   // ���ձ���״̬

/* ˽�����Ͷ��� --------------------------------------------------------------*/
struct MOTORpar MOTOR1,MOTOR2,MOTOR3,MOTOR4,MOTOR5,MOTOR6,MOTOR7,MOTOR8;
unsigned char Patstateall;
unsigned char Patstateal2;
unsigned char CAN1_data[8];
unsigned int index;


unsigned  char can_Cmd_Q[can_Cmd_Q_Max_Size];
int can_Cmd_Q_Push = 0;
int can_Cmd_Q_Pop = 0;
int can_Cmd_Q_Ctr = 0;
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
struct netif gnetif;
extern ETH_HandleTypeDef EthHandle;
extern TIM_HandleTypeDef htimx;
//extern UART_HandleTypeDef husartx_rs485;
TIM_HandleTypeDef TimHandle;
TIM_HandleTypeDef TimHandle2;
KEY key1,key2,key3,key4,key5;
int total_cnt=0;
int state_check_flag = 0;
/* ��չ���� ------------------------------------------------------------------*/
extern __IO uint8_t DHCP_flag;
/* ˽�к���ԭ�� --------------------------------------------------------------*/
static void Netif_Config(void);
void can_Cmd_Q_Send(void);
void BASIC_TIM3_Init(void);
void send_state_check(void);
void BASIC_TIM2_Init(void);
void WaitTimeOut(void);
/* ������ --------------------------------------------------------------------*/
/**
  * ��������: ϵͳʱ������
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
 
  __HAL_RCC_PWR_CLK_ENABLE();                                     //ʹ��PWRʱ��

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);  //���õ�ѹ�������ѹ����1

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;      // �ⲿ����8MHz
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                        //��HSE 
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                    //��PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;            //PLLʱ��Դѡ��HSE
  RCC_OscInitStruct.PLL.PLLM = 8;                                 //8��ƵMHz
  RCC_OscInitStruct.PLL.PLLN = 336;                               //336��Ƶ
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;                     //2��Ƶ���õ�168MHz��ʱ��
  RCC_OscInitStruct.PLL.PLLQ = 7;                                 //USB/SDIO/������������ȵ���PLL��Ƶϵ��
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;       // ϵͳʱ�ӣ�168MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;              // AHBʱ�ӣ� 168MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;               // APB1ʱ�ӣ�42MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;               // APB2ʱ�ӣ�84MHz
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_RCC_EnableCSS();                                            // ʹ��CSS���ܣ�����ʹ���ⲿ�����ڲ�ʱ��ԴΪ����
  
 	// HAL_RCC_GetHCLKFreq()/1000    1ms�ж�һ��
	// HAL_RCC_GetHCLKFreq()/100000	 10us�ж�һ��
	// HAL_RCC_GetHCLKFreq()/1000000 1us�ж�һ��
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/100);                // ���ò�����ϵͳ�δ�ʱ��
  /* ϵͳ�δ�ʱ��ʱ��Դ */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* ϵͳ�δ�ʱ���ж����ȼ����� */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * ��������: ������.
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
int main(void)
{
  /* ��λ�������裬��ʼ��Flash�ӿں�ϵͳ�δ�ʱ�� */
  HAL_Init();
  /* ����ϵͳʱ�� */
  SystemClock_Config();
  /* ��ʼ�����ڲ����ô����ж����ȼ� */
  MX_DEBUG_USART_Init();
  
  //KEY_GPIO_Init();
  
  /* ���ø�ʽ�����������ӡ������� */
  printf("YS-F4Pro������ԣ�UDP����\n");
  /* ��ʼ��LWIP�ں� */
  lwip_init();
  
  /* ��������ӿ� */
  Netif_Config();
  
  /* �鿴�������ӿ����� */
  User_notification(&gnetif);
  
  MX_CAN_Init();
  HAL_ETH_GetReceivedFrame_IT(&EthHandle);  
  HAL_CAN_Receive_IT(&hCAN, CAN_FIFO0);
  
  BASIC_TIM2_Init();
  HAL_TIM_Base_Start_IT(&TimHandle2);
  BASIC_TIM3_Init();
  HAL_TIM_Base_Start_IT(&TimHandle);
  
  //test
  //Rx_MSG = MSG_IDLE;
  /* ��ȡreg_addr �ļĴ���״̬ */
  //MB_ReadHoldingReg_03H(0x01, 0x0E, 0x0002);
  /* �ȴ��ӻ���Ӧ */
  //WaitTimeOut(); //�ȴ�ʱ����Ϊ200ms
  
  /* ����ѭ�� */
  while (1)
  {     
          
      /* �������ݲ�������LwIP */
    //ethernetif_input(&gnetif);
    /* ��ʱ���� */
    sys_check_timeouts();
    /*if(Rx_MSG == MSG_COM)
    {
      //�յ��������Ĵ�վ���������� 
      if(Rx_Buf[0] != MB_SLAVEADDR)
        continue;
      static uint16_t crc_check = 0;
      crc_check = ( (Rx_Buf[RxCount-1]<<8) | Rx_Buf[RxCount-2] );
      // CRC У����ȷ 
      if(crc_check == MB_CRC16((uint8_t*)&Rx_Buf,RxCount-2)) 
      {
        //ͨ�Ŵ���
        if(Rx_Buf[1]&0x80)
        {
          printf("%x", &Rx_Buf);
          while(1)
          {
            ;
            HAL_Delay(500);
          }
        }
      }
      Rx_MSG = MSG_IDLE;
    }*/
    if((can_Cmd_Q_Ctr <= can_Cmd_Q_Max_Size) && (can_Cmd_Q_Ctr > 0))
      can_Cmd_Q_Send();
    //printf("can_Cmd_Q_Ctr = %d\n", can_Cmd_Q_Ctr);
    #ifdef USE_DHCP
    /* LwIP��Ҫ��ʱ����ĺ��� */
    DHCP_Periodic_Handle(&gnetif); 
    #endif
    if (state_check_flag)
      send_state_check();
    
  }
}
/****************************************************************
             ����ѭ��������ɽ���CANָ��ķ��͹���
****************************************************************/
void can_Cmd_Q_Send(void)
{
  
  printf("can_Cmd_Q_Ctr = %d\n", can_Cmd_Q_Ctr);
  int cmd_index = can_Cmd_Q[can_Cmd_Q_Pop];
  switch(cmd_index)
  {
  case MOVE_MOTOR_INIT:
    CAN_SendTxMsg(0x601, enable);
    HAL_Delay(1);
    CAN_SendTxMsg(0x602, enable);
    HAL_Delay(1);
    CAN_SendTxMsg(0x603, enable);
    HAL_Delay(1);
    CAN_SendTxMsg(0x604, enable);
    HAL_Delay(1);
    CAN_SendTxMsg(0x601, resetEncoder);
    HAL_Delay(1);
    CAN_SendTxMsg(0x602, resetEncoder);
    HAL_Delay(1);
    CAN_SendTxMsg(0x603, resetEncoder);
    HAL_Delay(1);
    CAN_SendTxMsg(0x604, resetEncoder);
    break;
  case MOVE_MOTOR_STOP:
    CAN_SendTxMsg(0x601, stop);
    HAL_Delay(1);
    CAN_SendTxMsg(0x602, stop);
    HAL_Delay(1);
    CAN_SendTxMsg(0x603, stop);
    HAL_Delay(1);
    CAN_SendTxMsg(0x604, stop);
    HAL_Delay(1);
    CAN_SendTxMsg(0x601, enable);
    HAL_Delay(1);
    CAN_SendTxMsg(0x602, enable);
    HAL_Delay(1);
    CAN_SendTxMsg(0x603, enable);
    HAL_Delay(1);
    CAN_SendTxMsg(0x604, enable);
    break;
  case MOVE_MOTOR_PARA_SET:
    CAN_SendTxMsg(0x601, setSpeed);
    HAL_Delay(1);
    CAN_SendTxMsg(0x602, setSpeed);
    HAL_Delay(1);
    CAN_SendTxMsg(0x603, setSpeed);
    HAL_Delay(1);
    CAN_SendTxMsg(0x604, setSpeed);
    break;
  case MOVE_MOTOR_ALARM_CLR:
    CAN_SendTxMsg(0x601,alarmclear);
    HAL_Delay(1);
    CAN_SendTxMsg(0x602,alarmclear);
    HAL_Delay(1);
    CAN_SendTxMsg(0x603,alarmclear);
    HAL_Delay(1);
    CAN_SendTxMsg(0x604,alarmclear);
    HAL_Delay(1);
    CAN_SendTxMsg(0x601, enable);
    HAL_Delay(1);
    CAN_SendTxMsg(0x602, enable);
    HAL_Delay(1);
    CAN_SendTxMsg(0x603, enable);
    HAL_Delay(1);
    CAN_SendTxMsg(0x604, enable);
    break;
  case ROUND_MOTOR_INIT:
    CAN_SendTxMsg(0x605, enable);
    HAL_Delay(1);
    CAN_SendTxMsg(0x606, enable);
    HAL_Delay(1);
    CAN_SendTxMsg(0x607, enable);
    HAL_Delay(1);
    CAN_SendTxMsg(0x608, enable);
    HAL_Delay(1);
    /*CAN_SendTxMsg(0x605, resetEncoder);
    HAL_Delay(1);
    CAN_SendTxMsg(0x606, resetEncoder);
    HAL_Delay(1);
    CAN_SendTxMsg(0x607, resetEncoder);
    HAL_Delay(1);
    CAN_SendTxMsg(0x608, resetEncoder);
    HAL_Delay(1);*/
    CAN_SendTxMsg(0x605, posModeAbs);
    HAL_Delay(1);
    CAN_SendTxMsg(0x606, posModeAbs);
    HAL_Delay(1);
    CAN_SendTxMsg(0x607, posModeAbs);
    HAL_Delay(1);
    CAN_SendTxMsg(0x608, posModeAbs);
    break;
  case ROUND_MOTOR_STOP:
    CAN_SendTxMsg(0x605, stop);
    HAL_Delay(1);
    CAN_SendTxMsg(0x606, stop);
    HAL_Delay(1);
    CAN_SendTxMsg(0x607, stop);
    HAL_Delay(1);
    CAN_SendTxMsg(0x608, stop);
    HAL_Delay(1);
    CAN_SendTxMsg(0x605, enable);
    HAL_Delay(1);
    CAN_SendTxMsg(0x606, enable);
    HAL_Delay(1);
    CAN_SendTxMsg(0x607, enable);
    HAL_Delay(1);
    CAN_SendTxMsg(0x608, enable);
    break;
  case ROUND_MOTOR_PARA_SET:
    CAN_SendTxMsg(0x605, posSet);
    //HAL_Delay(1);
    CAN_SendTxMsg(0x605, posStart);
    //HAL_Delay(1);
    CAN_SendTxMsg(0x606, posSet);
    HAL_Delay(1);
    CAN_SendTxMsg(0x606, posStart);
    //HAL_Delay(1);
    CAN_SendTxMsg(0x607, posSet);
    //HAL_Delay(1);
    CAN_SendTxMsg(0x607, posStart);
    HAL_Delay(1);
    CAN_SendTxMsg(0x608, posSet);
    //HAL_Delay(1);
    CAN_SendTxMsg(0x608, posStart);
    break;
  case ROUND_MOTOR_ALARM_CLR:
    CAN_SendTxMsg(0x605,alarmclear);
    HAL_Delay(1);
    CAN_SendTxMsg(0x606,alarmclear);
    HAL_Delay(1);
    CAN_SendTxMsg(0x607,alarmclear);
    HAL_Delay(1);
    CAN_SendTxMsg(0x608,alarmclear);
    HAL_Delay(1);
    CAN_SendTxMsg(0x605, enable);
    HAL_Delay(1);
    CAN_SendTxMsg(0x606, enable);
    HAL_Delay(1);
    CAN_SendTxMsg(0x607, enable);
    HAL_Delay(1);
    CAN_SendTxMsg(0x608, enable);
    break;
  default:
    break;
  }
  can_Cmd_Q_Pop = (can_Cmd_Q_Pop + 1) % 100;
  can_Cmd_Q_Ctr = can_Cmd_Q_Ctr - 1;
}
/**
  * ��������: ��������ӿ�
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
static void Netif_Config(void)
{
  ip_addr_t ipaddr;
  ip_addr_t netmask;
  ip_addr_t gw;
  
  /* Initializes the dynamic memory heap defined by MEM_SIZE.*/
  mem_init();
  
  /* Initializes the memory pools defined by MEMP_NUM_x.*/
  memp_init();  
#ifdef USE_DHCP
  ip_addr_set_zero_ip4(&ipaddr);
  ip_addr_set_zero_ip4(&netmask);
  ip_addr_set_zero_ip4(&gw);
#else
  IP_ADDR4(&ipaddr,IP_ADDR0,IP_ADDR1,IP_ADDR2,IP_ADDR3);
  IP_ADDR4(&netmask,NETMASK_ADDR0,NETMASK_ADDR1,NETMASK_ADDR2,NETMASK_ADDR3);
  IP_ADDR4(&gw,GW_ADDR0,GW_ADDR1,GW_ADDR2,GW_ADDR3);
  
	printf("��̬IP��ַ........................%d.%d.%d.%d\r\n",IP_ADDR0,IP_ADDR1,IP_ADDR2,IP_ADDR3);
	printf("��������..........................%d.%d.%d.%d\r\n",NETMASK_ADDR0,NETMASK_ADDR1,NETMASK_ADDR2,NETMASK_ADDR3);
	printf("Ĭ������..........................%d.%d.%d.%d\r\n",GW_ADDR0,GW_ADDR1,GW_ADDR2,GW_ADDR3);
#endif /* USE_DHCP */
  
  /* Add the network interface */    
  netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &ethernet_input);
  
  /* Registers the default network interface */
  netif_set_default(&gnetif);
  
  if (netif_is_link_up(&gnetif))
  {
    printf("�ɹ���������\n");
    /* When the netif is fully configured this function must be called */
    netif_set_up(&gnetif);
  }
  else
  {
    /* When the netif link is down this function must be called */
    netif_set_down(&gnetif);
  }
  
  /* Set the link callback function, this function is called on change of link status*/
  netif_set_link_callback(&gnetif, ethernetif_update_config);
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
  
  data[1] = hCAN.pRxMsg->StdId;
  data[0] = hCAN.pRxMsg->StdId>>8;
  //udp_send_t(2);
  
  /*if(hCAN.pRxMsg->DLC!=0)
  {

    for(i=0;i<hCAN.pRxMsg->DLC;i++)
    {
      
      data[i] = hCAN.pRxMsg->Data[i];
    }
    udp_send_t(8);
  }*/
  for(i=0;i<hCAN.pRxMsg->DLC;i++)
  {
      data[i] = hCAN.pRxMsg->Data[i];
      CAN1_data[i] = hCAN.pRxMsg->Data[i];
  }
  //udp_send_t(hCAN.pRxMsg->DLC);
  index=(((index&0)|CAN1_data[2])<<8)|CAN1_data[1];
  switch (hCAN.pRxMsg->StdId)
	{
		case MOTORIDrec1:
			if(index==0x2003)
				MOTOR1.emestop=CAN1_data[4];
			else if (index==0x2000)
				MOTOR1.enstate=CAN1_data[4];
			else if (index==0x6060)
				MOTOR1.Patstate=CAN1_data[4];
			else if (index==0x606C)
				MOTOR1.speed=(((MOTOR1.speed&0)|CAN1_data[5])<<8)|CAN1_data[4];
			else if (index==0x6078)
				MOTOR1.current=(((MOTOR1.current&0)|CAN1_data[5])<<8)|CAN1_data[4];
			else if (index==0x6091)
				MOTOR1.temp=(((MOTOR1.temp&0)|CAN1_data[5])<<8)|CAN1_data[4];
			else if (index==0x200B)
				MOTOR1.fault=((((((MOTOR1.fault&0)|CAN1_data[7])<<8)|CAN1_data[6])<<8)|CAN1_data[5])|CAN1_data[4];
			/*if(MOTOR1.fault!=0)
			{
				CAN1_WriteData(0x601,stop);
				CAN1_WriteData(0x602,stop);
				CAN1_WriteData(0x603,stop);
				CAN1_WriteData(0x604,stop);
			}*/
			  else if (index==0x606B)
				MOTOR1.given=(((MOTOR3.given&0)|CAN1_data[5])<<8)|CAN1_data[4];
			  else if (index==0x6063)
				MOTOR1.pos=(((((((MOTOR1.pos&0)|CAN1_data[7])<<8)|CAN1_data[6])<<8)|CAN1_data[5])<<8)|CAN1_data[4];
			else if(index==0x6090)
                                MOTOR1.flag==((((MOTOR1.flag&0)|CAN1_data[4])&0x10)>>4)&1;
                          break;
		case MOTORIDrec2:
			if(index==0x2003)
				MOTOR2.emestop=CAN1_data[4];
			else if (index==0x2000)
				MOTOR2.enstate=CAN1_data[4];
			else if (index==0x6060)
				MOTOR2.Patstate=CAN1_data[4];
			else if (index==0x606C)
				MOTOR2.speed=(((MOTOR1.speed&0)|CAN1_data[5])<<8)|CAN1_data[4];
			else if (index==0x6078)
				MOTOR2.current=(((MOTOR1.current&0)|CAN1_data[5])<<8)|CAN1_data[4];
			else if (index==0x6091)
				MOTOR2.temp=(((MOTOR1.temp&0)|CAN1_data[5])<<8)|CAN1_data[4];
			else if (index==0x200B)
				MOTOR2.fault=((((((MOTOR1.fault&0)|CAN1_data[7])<<8)|CAN1_data[6])<<8)|CAN1_data[5])|CAN1_data[4];
			else if (index==0x606B)
				MOTOR2.given=(((MOTOR3.given&0)|CAN1_data[5])<<8)|CAN1_data[4];
			else if (index==0x6063)
				MOTOR2.pos=(((((((MOTOR2.pos&0)|CAN1_data[7])<<8)|CAN1_data[6])<<8)|CAN1_data[5])<<8)|CAN1_data[4];
			else if(index==0x6090)
                                MOTOR2.flag==((((MOTOR2.flag&0)|CAN1_data[4])&0x10)>>4)&1;
                        break;		
		case MOTORIDrec3:
			if(index==0x2003)
				MOTOR3.emestop=CAN1_data[4];
			else if (index==0x2000)
				MOTOR3.enstate=CAN1_data[4];
			else if (index==0x6060)
				MOTOR3.Patstate=CAN1_data[4];
			else if (index==0x606C)
				MOTOR3.speed=(((MOTOR3.speed&0)|CAN1_data[5])<<8)|CAN1_data[4];
			else if (index==0x6078)
				MOTOR3.current=(((MOTOR3.current&0)|CAN1_data[5])<<8)|CAN1_data[4];
			else if (index==0x6091)
				MOTOR3.temp=(((MOTOR3.temp&0)|CAN1_data[5])<<8)|CAN1_data[4];
			else if (index==0x200B)
				MOTOR3.fault=((((((MOTOR3.fault&0)|CAN1_data[7])<<8)|CAN1_data[6])<<8)|CAN1_data[5])|CAN1_data[4];
			else if (index==0x606B)
				MOTOR3.given=(((MOTOR3.given&0)|CAN1_data[5])<<8)|CAN1_data[4];
			else if (index==0x6063)
				MOTOR3.pos=(((((((MOTOR3.pos&0)|CAN1_data[7])<<8)|CAN1_data[6])<<8)|CAN1_data[5])<<8)|CAN1_data[4];
			else if(index==0x6090)
                                MOTOR3.flag==((((MOTOR3.flag&0)|CAN1_data[4])&0x10)>>4)&1;
                        break;
		case MOTORIDrec4:
			if(index==0x2003)
				MOTOR4.emestop=CAN1_data[4];
			else if (index==0x2000)
				MOTOR4.enstate=CAN1_data[4];
			else if (index==0x6060)
				MOTOR4.Patstate=CAN1_data[4];
			else if (index==0x606C)
				MOTOR4.speed=(((MOTOR4.speed&0)|CAN1_data[5])<<8)|CAN1_data[4];
			else if (index==0x6078)
				MOTOR4.current=(((MOTOR4.current&0)|CAN1_data[5])<<8)|CAN1_data[4];
			else if (index==0x6091)
				MOTOR4.temp=(((MOTOR4.temp&0)|CAN1_data[5])<<8)|CAN1_data[4];
			else if (index==0x200B)
				MOTOR4.fault=((((((MOTOR4.fault&0)|CAN1_data[7])<<8)|CAN1_data[6])<<8)|CAN1_data[5])|CAN1_data[4];
			else if (index==0x606B)
				MOTOR4.given=(((MOTOR3.given&0)|CAN1_data[5])<<8)|CAN1_data[4];
			else if (index==0x6063)
				MOTOR4.pos=(((((((MOTOR4.pos&0)|CAN1_data[7])<<8)|CAN1_data[6])<<8)|CAN1_data[5])<<8)|CAN1_data[4];
			else if(index==0x6090)
                                MOTOR4.flag==((((MOTOR4.flag&0)|CAN1_data[4])&0x10)>>4)&1;
                        break;
                case MOTORIDrec5:
			if(index==0x2003)
				MOTOR5.emestop=CAN1_data[4];
			else if (index==0x2000)
				MOTOR5.enstate=CAN1_data[4];
			else if (index==0x6060)
				MOTOR5.Patstate=CAN1_data[4];
			else if (index==0x606C)
				MOTOR5.speed=(((MOTOR5.speed&0)|CAN1_data[5])<<8)|CAN1_data[4];
			else if (index==0x6078)
				MOTOR5.current=(((MOTOR5.current&0)|CAN1_data[5])<<8)|CAN1_data[4];
			else if (index==0x6091)
				MOTOR5.temp=(((MOTOR5.temp&0)|CAN1_data[5])<<8)|CAN1_data[4];
			else if (index==0x200B)
				MOTOR5.fault=((((((MOTOR5.fault&0)|CAN1_data[7])<<8)|CAN1_data[6])<<8)|CAN1_data[5])|CAN1_data[4];
			else if (index==0x606B)
				MOTOR5.given=(((MOTOR5.given&0)|CAN1_data[5])<<8)|CAN1_data[4];
			  else if (index==0x6063)
				MOTOR5.pos=(((((((MOTOR5.pos&0)|CAN1_data[7])<<8)|CAN1_data[6])<<8)|CAN1_data[5])<<8)|CAN1_data[4];
			else if(index==0x6090)
                                MOTOR5.flag==((((MOTOR5.flag&0)|CAN1_data[4])&0x10)>>4)&1;
                          break;
                case MOTORIDrec6:
			if(index==0x2003)
				MOTOR6.emestop=CAN1_data[4];
			else if (index==0x2000)
				MOTOR6.enstate=CAN1_data[4];
			else if (index==0x6060)
				MOTOR6.Patstate=CAN1_data[4];
			else if (index==0x606C)
				MOTOR6.speed=(((MOTOR6.speed&0)|CAN1_data[5])<<8)|CAN1_data[4];
			else if (index==0x6078)
				MOTOR6.current=(((MOTOR6.current&0)|CAN1_data[5])<<8)|CAN1_data[4];
			else if (index==0x6091)
				MOTOR6.temp=(((MOTOR6.temp&0)|CAN1_data[5])<<8)|CAN1_data[4];
			else if (index==0x200B)
				MOTOR6.fault=((((((MOTOR6.fault&0)|CAN1_data[7])<<8)|CAN1_data[6])<<8)|CAN1_data[5])|CAN1_data[4];
			else if (index==0x606B)
				MOTOR6.given=(((MOTOR6.given&0)|CAN1_data[5])<<8)|CAN1_data[4];
			  else if (index==0x6063)
				MOTOR6.pos=(((((((MOTOR6.pos&0)|CAN1_data[7])<<8)|CAN1_data[6])<<8)|CAN1_data[5])<<8)|CAN1_data[4];
			else if(index==0x6090)
                                MOTOR6.flag==((((MOTOR6.flag&0)|CAN1_data[4])&0x10)>>4)&1;
                          break;
                case MOTORIDrec7:
			if(index==0x2003)
				MOTOR7.emestop=CAN1_data[4];
			else if (index==0x2000)
				MOTOR7.enstate=CAN1_data[4];
			else if (index==0x6060)
				MOTOR7.Patstate=CAN1_data[4];
			else if (index==0x606C)
				MOTOR7.speed=(((MOTOR7.speed&0)|CAN1_data[5])<<8)|CAN1_data[4];
			else if (index==0x6078)
				MOTOR7.current=(((MOTOR7.current&0)|CAN1_data[5])<<8)|CAN1_data[4];
			else if (index==0x6091)
				MOTOR7.temp=(((MOTOR7.temp&0)|CAN1_data[5])<<8)|CAN1_data[4];
			else if (index==0x200B)
				MOTOR7.fault=((((((MOTOR7.fault&0)|CAN1_data[7])<<8)|CAN1_data[6])<<8)|CAN1_data[5])|CAN1_data[4];
			else if (index==0x606B)
				MOTOR7.given=(((MOTOR7.given&0)|CAN1_data[5])<<8)|CAN1_data[4];
			  else if (index==0x6063)
				MOTOR7.pos=(((((((MOTOR7.pos&0)|CAN1_data[7])<<8)|CAN1_data[6])<<8)|CAN1_data[5])<<8)|CAN1_data[4];
			else if(index==0x6090)
                                MOTOR7.flag==((((MOTOR7.flag&0)|CAN1_data[4])&0x10)>>4)&1;
                          break;
                case MOTORIDrec8:
			if(index==0x2003)
				MOTOR8.emestop=CAN1_data[4];
			else if (index==0x2000)
				MOTOR8.enstate=CAN1_data[4];
			else if (index==0x6060)
				MOTOR8.Patstate=CAN1_data[4];
			else if (index==0x606C)
				MOTOR8.speed=(((MOTOR8.speed&0)|CAN1_data[5])<<8)|CAN1_data[4];
			else if (index==0x6078)
				MOTOR8.current=(((MOTOR8.current&0)|CAN1_data[5])<<8)|CAN1_data[4];
			else if (index==0x6091)
				MOTOR8.temp=(((MOTOR8.temp&0)|CAN1_data[5])<<8)|CAN1_data[4];
			else if (index==0x200B)
				MOTOR8.fault=((((((MOTOR8.fault&0)|CAN1_data[7])<<8)|CAN1_data[6])<<8)|CAN1_data[5])|CAN1_data[4];
			else if (index==0x606B)
				MOTOR8.given=(((MOTOR8.given&0)|CAN1_data[5])<<8)|CAN1_data[4];
			  else if (index==0x6063)
				MOTOR8.pos=(((((((MOTOR8.pos&0)|CAN1_data[7])<<8)|CAN1_data[6])<<8)|CAN1_data[5])<<8)|CAN1_data[4];
			else if(index==0x6090)
                                MOTOR8.flag==((((MOTOR8.flag&0)|CAN1_data[4])&0x10)>>4)&1;
                          break;          
		}
	//can1_rec_flag = 1;
        posdone=(((((((posdone&0)|MOTOR1.flag)<<1)|MOTOR2.flag)<<1)|MOTOR3.flag)<<1)|MOTOR4.flag;
	enstateall=((((((enstateall&0)|MOTOR1.enstate)<<1)|MOTOR2.enstate)<<1)|MOTOR3.enstate)<<1|MOTOR4.enstate;
	Patstateall=(((Patstateall&0)|MOTOR1.Patstate)<<4)|MOTOR2.Patstate;
	Patstateal2=(((Patstateall&0)|MOTOR3.Patstate)<<4)|MOTOR4.Patstate;
  
  HAL_CAN_Receive_IT(hcan, CAN_FIFO0);
}
/**************************************************************************/
void HAL_ETH_RxCpltCallback(ETH_HandleTypeDef *heth)
{
  ethernetif_input(&gnetif);
  //HAL_ETH_GetReceivedFrame_IT(&EthHandle);
  printf("1\n");
}
/**************************************************************************/
void BASIC_TIM3_Init(void)
{
  __HAL_RCC_TIM3_CLK_ENABLE();
  
  TIM_MasterConfigTypeDef sMasterConfig;
  TimHandle.Instance=TIM3;//��ʱ����������
  TimHandle.Init.Period = 50000-1;
  TimHandle.Init.Prescaler = 1679;
  TimHandle.Init.ClockDivision = 0;
  TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  HAL_TIM_Base_Init(&TimHandle);
  
  sMasterConfig.MasterOutputTrigger=TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode=TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&TimHandle,&sMasterConfig);//��ģʽ����
  
  HAL_NVIC_SetPriority(TIM3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == (&TimHandle))
  {
    printf("send statecheck command!\n"); 
    state_check_flag = 1;
  }
  else if (htim == (&TimHandle2))
  {
    #ifdef USE_DHCP
    /* LwIP��Ҫ��ʱ����ĺ��� */
    DHCP_Periodic_Handle(&gnetif); 
    #endif
  }
  /*else if(htim == (&htimx))
  {
    //����Ѿ�����˽��յ�������������֡,��������Ϊ������������֡
    if(Rx_MSG == MSG_INC)
    { 
      Rx_MSG = MSG_INC;
    }
    //�����������ʱ�������
    else
    {
      Rx_MSG = MSG_COM;
    }
  }*/
  
            
  //HAL_TIM_Base_Start_IT(&TimHandle);
}
void send_state_check(void)
{
    CAN_SendTxMsg(0x601, enStateCheck);
    HAL_Delay(1);
    CAN_SendTxMsg(0x602, enStateCheck);
    HAL_Delay(1);
    CAN_SendTxMsg(0x603, enStateCheck);
    HAL_Delay(1);
    CAN_SendTxMsg(0x604, enStateCheck);
    HAL_Delay(1);
    CAN_SendTxMsg(0x601, givenSpeedCheck);
    HAL_Delay(1);
    CAN_SendTxMsg(0x602, givenSpeedCheck);
    HAL_Delay(1);
    CAN_SendTxMsg(0x603, givenSpeedCheck);
    HAL_Delay(1);
    CAN_SendTxMsg(0x604, givenSpeedCheck);
    HAL_Delay(1);
    CAN_SendTxMsg(0x601, actSpeedCheck);
    HAL_Delay(1);
    CAN_SendTxMsg(0x602, actSpeedCheck);
    HAL_Delay(1);
    CAN_SendTxMsg(0x603, actSpeedCheck);
    HAL_Delay(1);
    CAN_SendTxMsg(0x604, actSpeedCheck);
    HAL_Delay(1);
    CAN_SendTxMsg(0x601, actCurrentCheck);
    HAL_Delay(1);
    CAN_SendTxMsg(0x602, actCurrentCheck);
    HAL_Delay(1);
    CAN_SendTxMsg(0x603, actCurrentCheck);
    HAL_Delay(1);
    CAN_SendTxMsg(0x604, actCurrentCheck);
    HAL_Delay(1);
    CAN_SendTxMsg(0x601, errorCheck);
    HAL_Delay(1);
    CAN_SendTxMsg(0x602, errorCheck);
    HAL_Delay(1);
    CAN_SendTxMsg(0x603, errorCheck);
    HAL_Delay(1);
    CAN_SendTxMsg(0x604, errorCheck);
    HAL_Delay(1);
    CAN_SendTxMsg(0x601, positionCheck);
    HAL_Delay(1);
    CAN_SendTxMsg(0x602, positionCheck);
    HAL_Delay(1);
    CAN_SendTxMsg(0x603, positionCheck);
    HAL_Delay(1);
    CAN_SendTxMsg(0x604, positionCheck);
    HAL_Delay(1);     

    CAN_SendTxMsg(0x605, enStateCheck);
    HAL_Delay(1);
    CAN_SendTxMsg(0x606, enStateCheck);
    HAL_Delay(1);
    CAN_SendTxMsg(0x607, enStateCheck);
    HAL_Delay(1);
    CAN_SendTxMsg(0x608, enStateCheck);            
    HAL_Delay(1);
    CAN_SendTxMsg(0x605, actCurrentCheck);
    HAL_Delay(1);
    CAN_SendTxMsg(0x606, actCurrentCheck);
    HAL_Delay(1);
    CAN_SendTxMsg(0x607, actCurrentCheck);
    HAL_Delay(1);
    CAN_SendTxMsg(0x608, actCurrentCheck);
    HAL_Delay(1);
    CAN_SendTxMsg(0x605, errorCheck);
    HAL_Delay(1);
    CAN_SendTxMsg(0x606, errorCheck);
    HAL_Delay(1);
    CAN_SendTxMsg(0x607, errorCheck);
    HAL_Delay(1);
    CAN_SendTxMsg(0x608, errorCheck);
    HAL_Delay(1);
    CAN_SendTxMsg(0x605, positionCheck);
    HAL_Delay(1);
    CAN_SendTxMsg(0x606, positionCheck);
    HAL_Delay(1);
    CAN_SendTxMsg(0x607, positionCheck);
    HAL_Delay(1);
    CAN_SendTxMsg(0x608, positionCheck);
    HAL_Delay(1);
    CAN_SendTxMsg(0x605, driverFlagCheck);
    HAL_Delay(1);
    CAN_SendTxMsg(0x606, driverFlagCheck);
    HAL_Delay(1);
    CAN_SendTxMsg(0x607, driverFlagCheck);
    HAL_Delay(1);
    CAN_SendTxMsg(0x608, driverFlagCheck);
    HAL_Delay(1);
    
    state_check_flag = 0;
}
/*****************************************************************************/
void BASIC_TIM2_Init(void)
{
  __HAL_RCC_TIM2_CLK_ENABLE();
  
  TIM_MasterConfigTypeDef sMasterConfig;
  TimHandle.Instance=TIM2;//��ʱ����������
  TimHandle.Init.Period = 50000-1;
  TimHandle.Init.Prescaler = 83;
  TimHandle.Init.ClockDivision = 0;
  TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  HAL_TIM_Base_Init(&TimHandle);
  
  sMasterConfig.MasterOutputTrigger=TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode=TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&TimHandle,&sMasterConfig);//��ģʽ����
  
  HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}
/** 
  * ��������: ��ʱ�ȴ�����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��������֮��,�ȴ��ӻ���Ӧ,200ms֮������Ϊ��ʱ
  */
void WaitTimeOut(void )
{
  uint16_t TimeOut  = 0;// ͨѶ��ʱ ��λ:ms
  TimeOut = TIME_OVERRUN; // ���峬ʱʱ��Ϊ100ms,��ʵ�ʲ���ʱ��Ϊ200ms
  while(Rx_MSG != MSG_COM)
  {
    HAL_Delay(1);
    if(TimeOut-- == 0)
    {
      if(Rx_MSG != MSG_COM)     // 200ms����û�н������ݣ�����Ϊ��ʱ
      {
        ;
      }
      return ;
    }
  }
}
/** 
  * ��������: ���ڽ����жϻص�����
  * �������: ���ھ��
  * �� �� ֵ: ��
  * ˵    ��: ʹ��һ����ʱ���ıȽ��жϺ͸����ж���Ϊ���ճ�ʱ�ж�
  *           ֻҪ���յ����ݾͽ���ʱ����������0,�������Ƚ��жϵ�ʱ��
  *           ˵���Ѿ���ʱ1.5���ַ���ʱ��,�϶�Ϊ֡����,����Ǹ����ж�
  *           ����Ϊ�ǽ������
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  /*if(huart == &husart_debug)
  {
    switch(Rx_MSG)
    {
      //���յ���һ���ַ�֮��ʼ��ʱ1.5/3.5���ַ���ʱ�� 
      case MSG_IDLE:
        Rx_MSG = MSG_RXING;
        RxCount = 0;
        HAL_TIM_Base_Start(&htimx);
        break;
      
      //������һ�ν��յ������Ѿ�����1.5���ַ���ʱ����,�϶�Ϊ����֡������ 
      case MSG_ERR_FLAG:
        Rx_MSG = MSG_INC; // ����֡������
      break;
    }
    
    //ʹ�ܼ�������
    Rx_Buf[RxCount] = tmp_Rx_Buf;
    RxCount++;
    __HAL_TIM_SET_COUNTER(&htimx,0); // �������ʱ��
    HAL_UART_Receive_IT(&husart_debug,(uint8_t*)&tmp_Rx_Buf,1);
  }*/
}

/** 
  * ��������: ��ʱ���Ƚ��жϻص�����
  * �������: ��ʱ�����
  * �� �� ֵ: ��
  * ˵    ��: ����Ѿ���ʱ1.5���ַ���ʱ��û�н��յ�����
  */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* ����ǵ�һ�η����Ƚ��ж�,���ݶ�Ϊ������ */
  if(Rx_MSG != MSG_INC)
    Rx_MSG = MSG_ERR_FLAG;
  
  /* ����ǵڶ��ν���Ƚ��ж�,���϶�Ϊ���Ĳ����� */
  else
    Rx_MSG = MSG_INC;
}

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
