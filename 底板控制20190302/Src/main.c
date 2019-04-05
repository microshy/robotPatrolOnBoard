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

/* ˽�����Ͷ��� --------------------------------------------------------------*/
struct MOTORpar MOTOR1,MOTOR2,MOTOR3,MOTOR4,MOTOR5,MOTOR6,MOTOR7,MOTOR8;
unsigned char Patstateall;
unsigned char Patstateal2;
unsigned char CAN1_data[8];
unsigned int index;

/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
struct netif gnetif;
KEY key1,key2,key3,key4,key5;

/* ��չ���� ------------------------------------------------------------------*/
extern __IO uint8_t DHCP_flag;

/* ˽�к���ԭ�� --------------------------------------------------------------*/
static void Netif_Config(void);

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
  
  KEY_GPIO_Init();
  
  /* ���ø�ʽ�����������ӡ������� */
  printf("YS-F4Pro������ԣ�UDP����\n");
  /* ��ʼ��LWIP�ں� */
  lwip_init();
  
  /* ��������ӿ� */
  Netif_Config();
  
  /* �鿴�������ӿ����� */
  User_notification(&gnetif);
    
  MX_CAN_Init();
  
  HAL_CAN_Receive_IT(&hCAN, CAN_FIFO0); 
  
  KeyCreate(&key1,GetPinStateOfKey1);
  KeyCreate(&key2,GetPinStateOfKey2);  
  KeyCreate(&key3,GetPinStateOfKey3);
  KeyCreate(&key4,GetPinStateOfKey4);  
  KeyCreate(&key5,GetPinStateOfKey5);  
  
  
  /* ����ѭ�� */
  while (1)
  {     
     Key_RefreshState(&key1);//ˢ�°���״̬
     Key_RefreshState(&key2);//ˢ�°���״̬      
      /* �������ݲ�������LwIP */
    ethernetif_input(&gnetif);
    /* ��ʱ���� */
    sys_check_timeouts();
    if(Key_AccessTimes(&key1,KEY_ACCESS_READ)!=0)//���������¹�
    {
      printf("key1�����£����д���UDPserver\n");
		  //Initialize the server application 
      udp_echoclient_connect();
      Key_AccessTimes(&key1,KEY_ACCESS_WRITE_CLEAR);          
    } 
    if(Key_AccessTimes(&key2,KEY_ACCESS_READ)!=0)//���������¹�
    {
      printf("key2�����£�����������UDP_Server\n");
      udp_echoclient_send();
      Key_AccessTimes(&key2,KEY_ACCESS_WRITE_CLEAR);          
    }  

    #ifdef USE_DHCP
    /* LwIP��Ҫ��ʱ����ĺ��� */
    DHCP_Periodic_Handle(&gnetif); 
    #endif
    
  }
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
  udp_send_t(2);
  
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
  udp_send_t(hCAN.pRxMsg->DLC);
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

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
