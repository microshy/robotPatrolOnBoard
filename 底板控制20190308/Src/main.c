/**
  ******************************************************************************
  * 文件名程: main.c 
  * 作    者: 硬石嵌入式开发团队
  * 版    本: V1.0
  * 编写日期: 2017-03-30
  * 功    能: UDP_Client
  ******************************************************************************
  * 说明：
  * 本例程配套硬石stm32开发板YS-F4Pro使用。
  * 
  * 淘宝：
  * 论坛：http://www.ing10bbs.com
  * 版权归硬石嵌入式开发团队所有，请勿商用。
  ******************************************************************************
  */
/* 包含头文件 ----------------------------------------------------------------*/
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

/* 私有类型定义 --------------------------------------------------------------*/
struct MOTORpar MOTOR1,MOTOR2,MOTOR3,MOTOR4,MOTOR5,MOTOR6,MOTOR7,MOTOR8;
unsigned char Patstateall;
unsigned char Patstateal2;
unsigned char CAN1_data[8];
unsigned int index;
int iomotor1,iomotor2,iomotor3,iomotor4;
int dir1,dir2,dir3,dir4;
extern int btorigin;
unsigned  char can_Cmd_Q[can_Cmd_Q_Max_Size];
int can_Cmd_Q_Push = 0;
int can_Cmd_Q_Pop = 0;
int can_Cmd_Q_Ctr = 0;
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
struct netif gnetif;
extern ETH_HandleTypeDef EthHandle;
TIM_HandleTypeDef TimHandle;
TIM_HandleTypeDef TimHandle2;
KEY key1,key2,key3,key4,key5;
int total_cnt=0;
int state_check_flag = 0;
/* 扩展变量 ------------------------------------------------------------------*/
extern __IO uint8_t DHCP_flag;

/* 私有函数原形 --------------------------------------------------------------*/
static void Netif_Config(void);
void can_Cmd_Q_Send(void);
void turn_back_to_origin(void);
void BASIC_TIM3_Init(void);
void send_state_check(void);
void BASIC_TIM2_Init(void);
/* 函数体 --------------------------------------------------------------------*/
/**
  * 函数功能: 系统时钟配置
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
 
  __HAL_RCC_PWR_CLK_ENABLE();                                     //使能PWR时钟

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);  //设置调压器输出电压级别1

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;      // 外部晶振，8MHz
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;                        //打开HSE 
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;                    //打开PLL
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;            //PLL时钟源选择HSE
  RCC_OscInitStruct.PLL.PLLM = 8;                                 //8分频MHz
  RCC_OscInitStruct.PLL.PLLN = 336;                               //336倍频
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;                     //2分频，得到168MHz主时钟
  RCC_OscInitStruct.PLL.PLLQ = 7;                                 //USB/SDIO/随机数产生器等的主PLL分频系数
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;       // 系统时钟：168MHz
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;              // AHB时钟： 168MHz
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;               // APB1时钟：42MHz
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;               // APB2时钟：84MHz
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5);

  HAL_RCC_EnableCSS();                                            // 使能CSS功能，优先使用外部晶振，内部时钟源为备用
  
 	// HAL_RCC_GetHCLKFreq()/1000    1ms中断一次
	// HAL_RCC_GetHCLKFreq()/100000	 10us中断一次
	// HAL_RCC_GetHCLKFreq()/1000000 1us中断一次
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/100);                // 配置并启动系统滴答定时器
  /* 系统滴答定时器时钟源 */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* 系统滴答定时器中断优先级配置 */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * 函数功能: 主函数.
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
int main(void)
{
  /* 复位所有外设，初始化Flash接口和系统滴答定时器 */
  HAL_Init();
  /* 配置系统时钟 */
  SystemClock_Config();
  /* 初始化串口并配置串口中断优先级 */
  MX_DEBUG_USART_Init();
  
  //KEY_GPIO_Init();
  
  /* 调用格式化输出函数打印输出数据 */
  printf("YS-F4Pro网络测试，UDP测试\n");
  /* 初始化LWIP内核 */
  lwip_init();
  
  /* 配置网络接口 */
  Netif_Config();
  
  /* 查看相关网络接口配置 */
  User_notification(&gnetif);
  
  MX_CAN_Init();
  HAL_ETH_GetReceivedFrame_IT(&EthHandle);  
  HAL_CAN_Receive_IT(&hCAN, CAN_FIFO0);
  
  BASIC_TIM2_Init();
  HAL_TIM_Base_Start_IT(&TimHandle2);
  BASIC_TIM3_Init();
  HAL_TIM_Base_Start_IT(&TimHandle);
  
  /* 无限循环 */
  while (1)
  {     
          
      /* 接收数据并发送至LwIP */
    //ethernetif_input(&gnetif);
    /* 超时处理 */
    sys_check_timeouts();
    
    if((can_Cmd_Q_Ctr <= can_Cmd_Q_Max_Size) && (can_Cmd_Q_Ctr > 0))
      can_Cmd_Q_Send();
    //printf("can_Cmd_Q_Ctr = %d\n", can_Cmd_Q_Ctr);
    #ifdef USE_DHCP
    /* LwIP需要定时处理的函数 */
    DHCP_Periodic_Handle(&gnetif); 
    #endif
    if (state_check_flag)
      send_state_check();
    if(btorigin)
      turn_back_to_origin();
    
  }
}
/****************************************************************
             利用循环队列完成接收CAN指令的发送工作
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
    printf("move_enable\n");
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
    printf("move_stop\n");
    break;
  case MOVE_MOTOR_PARA_SET:
    CAN_SendTxMsg(0x601, setSpeed);
    HAL_Delay(1);
    CAN_SendTxMsg(0x602, setSpeed);
    HAL_Delay(1);
    CAN_SendTxMsg(0x603, setSpeed);
    HAL_Delay(1);
    CAN_SendTxMsg(0x604, setSpeed);
    printf("move_speed\n");
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
    printf("move_clr_alarm\n");
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
    printf("round_enable\n");
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
    printf("round_stop\n");
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
    printf("round_start\n");
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
    printf("round_clr_alarm\n");
    break;
  default:
    printf("to be def\n");
    break;
  }
  can_Cmd_Q_Pop = (can_Cmd_Q_Pop + 1) % 100;
  can_Cmd_Q_Ctr = can_Cmd_Q_Ctr - 1;
}
/**
  * 函数功能: 配置网络接口
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
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
  
	printf("静态IP地址........................%d.%d.%d.%d\r\n",IP_ADDR0,IP_ADDR1,IP_ADDR2,IP_ADDR3);
	printf("子网掩码..........................%d.%d.%d.%d\r\n",NETMASK_ADDR0,NETMASK_ADDR1,NETMASK_ADDR2,NETMASK_ADDR3);
	printf("默认网关..........................%d.%d.%d.%d\r\n",GW_ADDR0,GW_ADDR1,GW_ADDR2,GW_ADDR3);
#endif /* USE_DHCP */
  
  /* Add the network interface */    
  netif_add(&gnetif, &ipaddr, &netmask, &gw, NULL, &ethernetif_init, &ethernet_input);
  
  /* Registers the default network interface */
  netif_set_default(&gnetif);
  
  if (netif_is_link_up(&gnetif))
  {
    printf("成功连接网卡\n");
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
  * 函数功能: CAN接收完成中断回调服务程序
  * 输入参数: hcan：CAN外设句柄指针
  * 返 回 值: 无 CAN1_RX0_IRQHandler
  * 说    明：无 HAL_CAN_RxCpltCallback
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
                        else if (index==0x6089)
                                iomotor1=CAN1_data[4];
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
                        else if (index==0x6089)
                                iomotor2=CAN1_data[4];
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
                        else if (index==0x6089)
                                iomotor3=CAN1_data[4];
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
                        else if (index==0x6089)
                                iomotor4=CAN1_data[4];
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
  //printf("1\n");
}
/**************************************************************************/
void BASIC_TIM3_Init(void)
{
  __HAL_RCC_TIM3_CLK_ENABLE();
  
  TIM_MasterConfigTypeDef sMasterConfig;
  TimHandle.Instance=TIM3;//定时器基本配置
  TimHandle.Init.Period = 50000-1;
  TimHandle.Init.Prescaler = 1679;
  TimHandle.Init.ClockDivision = 0;
  TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  HAL_TIM_Base_Init(&TimHandle);
  
  sMasterConfig.MasterOutputTrigger=TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode=TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&TimHandle,&sMasterConfig);//主模式配置
  
  HAL_NVIC_SetPriority(TIM3_IRQn, 2, 0);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim == (&TimHandle))
  {
    //printf("send statecheck command!\n"); 
    state_check_flag = 1;
  }
  else if (htim == (&TimHandle2))
  {
    #ifdef USE_DHCP
    /* LwIP需要定时处理的函数 */
    DHCP_Periodic_Handle(&gnetif); 
    #endif
  }
  
            
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
  TimHandle.Instance=TIM2;//定时器基本配置
  TimHandle.Init.Period = 50000-1;
  TimHandle.Init.Prescaler = 83;
  TimHandle.Init.ClockDivision = 0;
  TimHandle.Init.CounterMode = TIM_COUNTERMODE_UP;
  HAL_TIM_Base_Init(&TimHandle);
  
  sMasterConfig.MasterOutputTrigger=TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode=TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&TimHandle,&sMasterConfig);//主模式配置
  
  HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}
/******************************旋转回原点***********************************/
void turn_back_to_origin(void)
{
  //first stop the moving motor
  CAN_SendTxMsg(0x601, stop);
  CAN_SendTxMsg(0x602, stop);
  CAN_SendTxMsg(0x603, stop);
  HAL_Delay(1);
  CAN_SendTxMsg(0x604, stop);
  CAN_SendTxMsg(0x601, enable);
  CAN_SendTxMsg(0x602, enable);
  HAL_Delay(1);
  CAN_SendTxMsg(0x603, enable);
  CAN_SendTxMsg(0x604, enable);
  
  iomotor1=iomotor2=iomotor3=iomotor4=0;
  //read digital io of drivers
  CAN_SendTxMsg(0x605, digIOCheck);
  CAN_SendTxMsg(0x606, digIOCheck);
  CAN_SendTxMsg(0x607, digIOCheck);
  HAL_Delay(1);
  CAN_SendTxMsg(0x608, digIOCheck);
  
  //use position to ensure the turning direction of each wheel
  //-1 means negative position 
  //while 0 means positive position
  dir1=MOTOR5.pos>>31;
  dir2=MOTOR5.pos>>31;
  dir3=MOTOR5.pos>>31;
  dir4=MOTOR5.pos>>31;
  
  //give the opposite direction postion to make sure that each wheel would pass the origin 
  if(dir1) CAN_SendTxMsg(0x605, posSetPosi);
  else CAN_SendTxMsg(0x605, posSetNega);
  if(dir2) CAN_SendTxMsg(0x606, posSetPosi);
  else CAN_SendTxMsg(0x606, posSetNega);
  if(dir3) CAN_SendTxMsg(0x607, posSetPosi);
  else CAN_SendTxMsg(0x607, posSetNega);
  if(dir4) CAN_SendTxMsg(0x608, posSetPosi);
  else CAN_SendTxMsg(0x608, posSetNega);
  
  //while motor5 not at origin
  if(iomotor1!=0xFE)
  {
    CAN_SendTxMsg(0x605, posStart);
    CAN_SendTxMsg(0x605, digIOCheck);
    while(iomotor1!=0xFE)
    {
      CAN_SendTxMsg(0x605, digIOCheck);
      HAL_Delay(10);
    }
    CAN_SendTxMsg(0x605, stop);
    CAN_SendTxMsg(0x605, enable);
    CAN_SendTxMsg(0x605, resetEncoder);
  }
  
}
/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
