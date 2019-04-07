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
#include "GeneralTIM/bsp_GeneralTIM.h"
    
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
struct Battery_State bat_state;
unsigned char Patstateall;
unsigned char Patstateal2;
unsigned char CAN1_data[8];
unsigned int index;
int iomotor1,iomotor2,iomotor3,iomotor4;
extern int motor1ang,motor2ang,motor3ang,motor4ang;
int dir1,dir2,dir3,dir4;
extern int btorigin;
extern unsigned char originstate;
unsigned  char can_Cmd_Q[can_Cmd_Q_Max_Size];
int can_Cmd_Q_Push = 0;
int can_Cmd_Q_Pop = 0;
int can_Cmd_Q_Ctr = 0;
unsigned  char modbus_Cmd_Q[modbus_Cmd_Q_Max_Size];
int modbus_Cmd_Q_Push = 0;
int modbus_Cmd_Q_Pop = 0;
int modbus_Cmd_Q_Ctr = 0;
int modbus_Cmd_Temp;
/* 私有宏定义 ----------------------------------------------------------------*/
#define MSG_ERR_FLAG  0xFFFF    // 接收错误 字符间超时
#define MSG_IDLE      0x0000    // 空闲状态
#define MSG_RX        0x0001    // 等待接受数据
#define MSG_RXING     0x0002    // 正在接收数据
#define MSG_COM       0x0003    // 接收完成
#define MSG_INC       0x8000    // 数据帧不完整(两字符间的空闲间隔大于1.5个字符时间)
#define TIME_OVERRUN  100       // 定义超时时间 ms
/* 私有变量 ------------------------------------------------------------------*/
__IO uint16_t Rx_MSG = MSG_IDLE;   // 接收报文状态
uint16_t modbus_crc_check,crc_check;
unsigned char posSetTest[8] = {0x23, 0x7A, 0x60, 0x00, 0xFF, 0xFF, 0x0F, 0x00};
unsigned char posSpeed[8] = {0x2B, 0x7F, 0x6, 0x00, 0xE8, 0x03, 0x00, 0x00};
/* 私有变量 ------------------------------------------------------------------*/
struct netif gnetif;
extern ETH_HandleTypeDef EthHandle;
TIM_HandleTypeDef TimHandle;
TIM_HandleTypeDef TimHandle2;
extern TIM_HandleTypeDef htimx;
extern CAN_HandleTypeDef hCAN;
KEY key1,key2,key3,key4,key5;
int total_cnt=0;
int state_check_flag = 0;
int battery_periodic = 0;
/* 扩展变量 ------------------------------------------------------------------*/
extern __IO uint8_t DHCP_flag;

/* 私有函数原形 --------------------------------------------------------------*/
static void Netif_Config(void);
void can_Cmd_Q_Send(void);
void modbus_Cmd_Q_Send(void);
void turn_back_to_origin(void);
void BASIC_TIM3_Init(void);
void send_state_check(void);
void BASIC_TIM2_Init(void);
void postrans(void);
void modbus_Periodic_Handler(void);
void modbus_Data_Process(void);
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
  //* 复位所有外设，初始化Flash接口和系统滴答定时器
  HAL_Init();
  //* 配置系统时钟 
  SystemClock_Config();
  //* 初始化串口并配置串口中断优先级
  MX_DEBUG_USART_Init();
  
  //初始化tim5
  GENERAL_TIMx_Init();
  __HAL_TIM_ENABLE_IT(&htimx,TIM_IT_CC1);
  __HAL_TIM_ENABLE_IT(&htimx,TIM_IT_UPDATE);
  HAL_TIM_Base_Start(&htimx);
  
  // 调用格式化输出函数打印输出数据
  printf("YS-F4Pro网络测试，UDP测试\n");

  //初始化LWIP内核
  lwip_init();
  
  //配置网络接口 
  Netif_Config();
  
  //* 查看相关网络接口配置
  User_notification(&gnetif);
  
  printf("进行创建UDPserver\n");
  udp_echoclient_connect();
  
  HAL_ETH_GetReceivedFrame_IT(&EthHandle); 
  
  MX_CAN_Init();
  HAL_CAN_Receive_IT(&hCAN, CAN_FIFO0);
  
  Rx_MSG=MSG_IDLE;
  
  BASIC_TIM2_Init();
  HAL_TIM_Base_Start_IT(&TimHandle2);
  BASIC_TIM3_Init();
  HAL_TIM_Base_Start_IT(&TimHandle);
  
  
  /*190401test
  CAN_SendTxMsg(0x608, positionCheck);
  HAL_Delay(1);
  CAN_SendTxMsg(0x608, enable);
  HAL_Delay(1);
  
  CAN_SendTxMsg(0x608, resetEncoder);
  HAL_Delay(1);
  CAN_SendTxMsg(0x608, posModeAbs);
  HAL_Delay(1);
  CAN_SendTxMsg(0x608, posSetTest);
  HAL_Delay(1);
  CAN_SendTxMsg(0x608, posStart);
  HAL_Delay(1000);
  CAN_SendTxMsg(0x608, positionCheck);
  HAL_Delay(1);
  CAN_SendTxMsg(0x608, digIOCheck);
  HAL_Delay(1);
  int dir4_test=MOTOR8.pos>>31;
    printf("dir %x\n",dir4_test);
    if(dir4_test) CAN_SendTxMsg(0x608, posSetPosi);
    else CAN_SendTxMsg(0x608, posSetNega);
  HAL_Delay(1);
    if(iomotor4!=0xFE)
    {
      CAN_SendTxMsg(0x608, posStart);
      HAL_Delay(1);
      CAN_SendTxMsg(0x608, digIOCheck);
      HAL_Delay(1);
      while(1)
      {
        CAN_SendTxMsg(0x608, digIOCheck);
        HAL_Delay(1);
        if(iomotor4==0xFE)
          break;
      }
      HAL_Delay(1);
      CAN_SendTxMsg(0x608, stop);
      HAL_Delay(1);
      CAN_SendTxMsg(0x608, resetEncoder);
      HAL_Delay(1);
      CAN_SendTxMsg(0x608, enable);
      HAL_Delay(1);
    }
    HAL_Delay(5);
    CAN_SendTxMsg(0x608, positionCheck);*/
  //* 无限循环

  while (1)
  {     
    HAL_CAN_Receive_IT(&hCAN, CAN_FIFO0); 
    /*  */  
    // 接收数据并发送至LwIP 
    //ethernetif_input(&gnetif);
    // 超时处理
    sys_check_timeouts();
    //找原点
    if(btorigin)
      turn_back_to_origin();
    //发送can指令
    if((can_Cmd_Q_Ctr <= can_Cmd_Q_Max_Size) && (can_Cmd_Q_Ctr > 0))
      can_Cmd_Q_Send();
    //Modbus消息处理
    modbus_Periodic_Handler();
    if((can_Cmd_Q_Ctr <= can_Cmd_Q_Max_Size) && (can_Cmd_Q_Ctr > 0))
      can_Cmd_Q_Send();
    #ifdef USE_DHCP
    // LwIP需要定时处理的函数
    DHCP_Periodic_Handle(&gnetif); 
    #endif
    LWIP_Process();
    if (state_check_flag)
      send_state_check();
    if((can_Cmd_Q_Ctr <= can_Cmd_Q_Max_Size) && (can_Cmd_Q_Ctr > 0))
      can_Cmd_Q_Send();

  }
}
/****************************************************************
             利用循环队列完成接收CAN指令的发送工作
****************************************************************/
void can_Cmd_Q_Send(void)
{
  while(can_Cmd_Q_Ctr)
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
      HAL_Delay(1);
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
      HAL_Delay(1);
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
      HAL_Delay(1);
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
      HAL_Delay(1);
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
      CAN_SendTxMsg(0x605, resetEncoder);
      HAL_Delay(1);
      CAN_SendTxMsg(0x606, resetEncoder);
      HAL_Delay(1);
      CAN_SendTxMsg(0x607, resetEncoder);
      HAL_Delay(1);
      CAN_SendTxMsg(0x608, resetEncoder);
      HAL_Delay(1);
      CAN_SendTxMsg(0x605, posModeAbs);
      HAL_Delay(1);
      CAN_SendTxMsg(0x606, posModeAbs);
      HAL_Delay(1);
      CAN_SendTxMsg(0x607, posModeAbs);
      HAL_Delay(1);
      CAN_SendTxMsg(0x608, posModeAbs);
      HAL_Delay(1);
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
      HAL_Delay(1);
      printf("round_stop\n");
      break;
    case ROUND_MOTOR_PARA_SET:
      CAN_SendTxMsg(0x605, posSet);
      HAL_Delay(1);
      CAN_SendTxMsg(0x605, posStart);
      HAL_Delay(1);
      CAN_SendTxMsg(0x606, posSet);
      HAL_Delay(1);
      CAN_SendTxMsg(0x606, posStart);
      HAL_Delay(1);
      CAN_SendTxMsg(0x607, posSet);
      HAL_Delay(1);
      CAN_SendTxMsg(0x607, posStart);
      HAL_Delay(1);
      CAN_SendTxMsg(0x608, posSet);
      HAL_Delay(1);
      CAN_SendTxMsg(0x608, posStart);
      HAL_Delay(1);
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
      HAL_Delay(1);
      printf("round_clr_alarm\n");
      break;
    default:
      printf("to be def\n");
      break;
    }
    can_Cmd_Q_Pop = (can_Cmd_Q_Pop + 1) % can_Cmd_Q_Max_Size;
    can_Cmd_Q_Ctr = can_Cmd_Q_Ctr - 1;
  }
  
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
  
  /*data[1] = hCAN.pRxMsg->StdId;
  data[0] = hCAN.pRxMsg->StdId>>8;
  //udp_send_t(2);
  
  if(hCAN.pRxMsg->DLC!=0)
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
  if(hCAN.pRxMsg->DLC==8)
  {
    //printf("can irq ......\n");
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
        MOTOR1.given=(((MOTOR1.given&0)|CAN1_data[5])<<8)|CAN1_data[4];
      else if (index==0x6063)
        MOTOR1.pos=(((((((MOTOR1.pos&0)|CAN1_data[7])<<8)|CAN1_data[6])<<8)|CAN1_data[5])<<8)|CAN1_data[4];
      else if(index==0x6090)
        MOTOR1.flag=((((MOTOR1.flag&0)|CAN1_data[4])&0x10)>>4)&1;
      break;
    case MOTORIDrec2:
      if(index==0x2003)
        MOTOR2.emestop=CAN1_data[4];
      else if (index==0x2000)
        MOTOR2.enstate=CAN1_data[4];
      else if (index==0x6060)
        MOTOR2.Patstate=CAN1_data[4];
      else if (index==0x606C)
        MOTOR2.speed=(((MOTOR2.speed&0)|CAN1_data[5])<<8)|CAN1_data[4];
      else if (index==0x6078)
        MOTOR2.current=(((MOTOR2.current&0)|CAN1_data[5])<<8)|CAN1_data[4];
      else if (index==0x6091)
        MOTOR2.temp=(((MOTOR2.temp&0)|CAN1_data[5])<<8)|CAN1_data[4];
      else if (index==0x200B)
        MOTOR2.fault=((((((MOTOR2.fault&0)|CAN1_data[7])<<8)|CAN1_data[6])<<8)|CAN1_data[5])|CAN1_data[4];
      else if (index==0x606B)
        MOTOR2.given=(((MOTOR2.given&0)|CAN1_data[5])<<8)|CAN1_data[4];
      else if (index==0x6063)
        MOTOR2.pos=(((((((MOTOR2.pos&0)|CAN1_data[7])<<8)|CAN1_data[6])<<8)|CAN1_data[5])<<8)|CAN1_data[4];
      else if(index==0x6090)
        MOTOR2.flag=((((MOTOR2.flag&0)|CAN1_data[4])&0x10)>>4)&1;
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
        MOTOR3.flag=((((MOTOR3.flag&0)|CAN1_data[4])&0x10)>>4)&1;
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
        MOTOR4.given=(((MOTOR4.given&0)|CAN1_data[5])<<8)|CAN1_data[4];
      else if (index==0x6063)
        MOTOR4.pos=(((((((MOTOR4.pos&0)|CAN1_data[7])<<8)|CAN1_data[6])<<8)|CAN1_data[5])<<8)|CAN1_data[4];
      else if(index==0x6090)
        MOTOR4.flag=((((MOTOR4.flag&0)|CAN1_data[4])&0x10)>>4)&1;
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
        MOTOR5.flag=CAN1_data[4]&1;
      else if (index==0x6089)
        iomotor1=CAN1_data[4];
      //printf("the number of pos done 5 issssssssssssssss %x\n",MOTOR5.flag);
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
        MOTOR6.flag=CAN1_data[4]&1;
      else if (index==0x6089)
        iomotor2=CAN1_data[4];
      //printf("the number of pos done 6 issssssssssssssss %x\n",MOTOR6.flag);
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
        MOTOR7.flag=CAN1_data[4]&1;
      else if (index==0x6089)
        iomotor3=CAN1_data[4];
      //printf("the number of pos done 7 issssssssssssssss %x\n",MOTOR7.flag);
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
        MOTOR8.flag=CAN1_data[4]&1;
      else if (index==0x6089)
        iomotor4=CAN1_data[4];
      //printf("io %x\n",iomotor4);
      //printf("motor 8 pos %x\n",MOTOR8.pos);
      //printf("the number of pos done 8 issssssssssssssss %x\n",MOTOR8.flag);
      break;          
    }
    //can1_rec_flag = 1;
    posdone=(((((((posdone&0)|MOTOR5.flag)<<1)|MOTOR6.flag)<<1)|MOTOR7.flag)<<1)|MOTOR8.flag;
    enstateall=((((((enstateall&0)|MOTOR5.enstate)<<1)|MOTOR6.enstate)<<1)|MOTOR7.enstate)<<1|MOTOR8.enstate;
    Patstateall=(((Patstateall&0)|MOTOR1.Patstate)<<4)|MOTOR2.Patstate;
    Patstateal2=(((Patstateall&0)|MOTOR3.Patstate)<<4)|MOTOR4.Patstate;
  }
  
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
void BASIC_TIM2_Init(void)
{
  __HAL_RCC_TIM2_CLK_ENABLE();
  
  TIM_MasterConfigTypeDef sMasterConfig;
  TimHandle2.Instance=TIM2;//定时器基本配置
  TimHandle2.Init.Period = 50000-1;
  TimHandle2.Init.Prescaler = 3359;
  TimHandle2.Init.ClockDivision = 0;
  TimHandle2.Init.CounterMode = TIM_COUNTERMODE_UP;
  HAL_TIM_Base_Init(&TimHandle2);
  
  sMasterConfig.MasterOutputTrigger=TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode=TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&TimHandle2,&sMasterConfig);//主模式配置
  
  HAL_NVIC_SetPriority(TIM2_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
}
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
  if (htim == (&TimHandle))//1s
  {
     //LWIP_Process();
    //printf("%x\n",Rx_MSG);
    state_check_flag = 1;
    if((modbus_Cmd_Q_Ctr <= modbus_Cmd_Q_Max_Size - 1) && modbus_Cmd_Q_Ctr>=0)
    { 
      modbus_Cmd_Q[modbus_Cmd_Q_Push] = BAT_READ_ALARM;
      modbus_Cmd_Q_Push = (modbus_Cmd_Q_Push + 1) % modbus_Cmd_Q_Max_Size;
      modbus_Cmd_Q_Ctr = modbus_Cmd_Q_Ctr + 1;
    }
  }
  else if (htim == (&TimHandle2))//2s
  {
    #ifdef USE_DHCP
    //printf("periodic tconnect\n");
    /* LwIP需要定时处理的函数 */
    DHCP_Periodic_Handle(&gnetif); 
    #endif
    if(battery_periodic==9)
    {
      if((modbus_Cmd_Q_Ctr <= modbus_Cmd_Q_Max_Size - 1) && modbus_Cmd_Q_Ctr>=0)
      { 
        modbus_Cmd_Q[modbus_Cmd_Q_Push] = BAT_READ_TEMPER;
        modbus_Cmd_Q_Push = (modbus_Cmd_Q_Push + 1) % modbus_Cmd_Q_Max_Size;
        modbus_Cmd_Q_Ctr = modbus_Cmd_Q_Ctr + 1;
      }
      if((modbus_Cmd_Q_Ctr <= modbus_Cmd_Q_Max_Size - 1) && modbus_Cmd_Q_Ctr>=0)
      { 
        modbus_Cmd_Q[modbus_Cmd_Q_Push] = BAT_READ_POWER;
        modbus_Cmd_Q_Push = (modbus_Cmd_Q_Push + 1) % modbus_Cmd_Q_Max_Size;
        modbus_Cmd_Q_Ctr = modbus_Cmd_Q_Ctr + 1;
      }
      if((modbus_Cmd_Q_Ctr <= modbus_Cmd_Q_Max_Size - 1) && modbus_Cmd_Q_Ctr>=0)
      { 
        modbus_Cmd_Q[modbus_Cmd_Q_Push] = BAT_READ_VOLTAGE;
        modbus_Cmd_Q_Push = (modbus_Cmd_Q_Push + 1) % modbus_Cmd_Q_Max_Size;
        modbus_Cmd_Q_Ctr = modbus_Cmd_Q_Ctr + 1;
      }
      if((modbus_Cmd_Q_Ctr <= modbus_Cmd_Q_Max_Size - 1) && modbus_Cmd_Q_Ctr>=0)
      { 
      modbus_Cmd_Q[modbus_Cmd_Q_Push] = BAT_READ_CURRENT;
      modbus_Cmd_Q_Push = (modbus_Cmd_Q_Push + 1) % modbus_Cmd_Q_Max_Size;
      modbus_Cmd_Q_Ctr = modbus_Cmd_Q_Ctr + 1;
      }
      battery_periodic=0;
    }
    else
      battery_periodic += 1;
  }
  else if(htim==&htimx)//modbus用
  {
    /* 如果已经标记了接收到不完整的数据帧,则继续标记为不完整的数据帧 */
    if(Rx_MSG == MSG_INC)
    { 
      Rx_MSG = MSG_INC;
    }
    /* 在正常情况下时接收完成 */
    else
    {
      Rx_MSG = MSG_COM;
    }
    //printf("100\n");
  }
}
/** 
  * 函数功能: 定时器比较中断回调函数
  * 输入参数: 定时器句柄
  * 返 回 值: 无
  * 说    明: 标记已经超时1.5个字符的时间没有接收到数据
  */
void HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim)
{
  if(htim==&htimx)
  {
    //* 如果是第一次发生比较中断,则暂定为错误标记
    if(Rx_MSG != MSG_INC)
      Rx_MSG = MSG_ERR_FLAG;
    
    //* 如果是第二次进入比较中断,则认定为报文不完整
    else
      Rx_MSG = MSG_INC;
  }
  //printf("1000\n");
}
/*****************************************************************************/
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

    if((can_Cmd_Q_Ctr <= can_Cmd_Q_Max_Size) && (can_Cmd_Q_Ctr > 0))
      can_Cmd_Q_Send();
    
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
    postrans();
    state_check_flag = 0;
    printf("send statecheck command!\n");
}
void postrans(void)
{
    motor1ang=1*((int)(MOTOR1.pos*360.0/6000.0));
    motor2ang=1*((int)(MOTOR2.pos*360.0/6000.0));
    motor3ang=1*((int)(MOTOR3.pos*360.0/6000.0));
    motor4ang=(int)(MOTOR4.pos*360.0/6000.0);
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
  HAL_Delay(1);
  CAN_SendTxMsg(0x606, digIOCheck);
  HAL_Delay(1);
  CAN_SendTxMsg(0x607, digIOCheck);
  HAL_Delay(1);
  CAN_SendTxMsg(0x608, digIOCheck);
  HAL_Delay(1);
  //use position to ensure the turning direction of each wheel
  //-1 means negative position 
  //while 0 means positive position
  dir1=MOTOR5.pos>>31;
  dir2=MOTOR6.pos>>31;
  dir3=MOTOR7.pos>>31;
  dir4=MOTOR8.pos>>31;
  
  //give the opposite direction postion to make sure that each wheel would pass the origin 
  if(dir1) CAN_SendTxMsg(0x605, posSetPosi);
  else CAN_SendTxMsg(0x605, posSetNega);
  HAL_Delay(1);
  if(dir2) CAN_SendTxMsg(0x606, posSetPosi);
  else CAN_SendTxMsg(0x606, posSetNega);
  HAL_Delay(1);
  if(dir3) CAN_SendTxMsg(0x607, posSetPosi);
  else CAN_SendTxMsg(0x607, posSetNega);
  HAL_Delay(1);
  if(dir4) CAN_SendTxMsg(0x608, posSetPosi);
  else CAN_SendTxMsg(0x608, posSetNega);
  HAL_Delay(1);
  //while motor5 not at origin
  if(iomotor1!=0xFE)
  {
    CAN_SendTxMsg(0x605, posStart);
    HAL_Delay(1);
    CAN_SendTxMsg(0x605, digIOCheck);
    HAL_Delay(1);
    while(iomotor1!=0xFE)
    {
      CAN_SendTxMsg(0x605, digIOCheck);
      HAL_Delay(10);
    }
    CAN_SendTxMsg(0x605, stop);
    HAL_Delay(1);
    CAN_SendTxMsg(0x605, enable);
    HAL_Delay(1);
    CAN_SendTxMsg(0x605, resetEncoder);
    HAL_Delay(1);
  }
  //while motor6 not at origin
  if((iomotor1==0xFE)&&(iomotor2!=0xFE))
  {
    CAN_SendTxMsg(0x606, posStart);
    HAL_Delay(1);
    CAN_SendTxMsg(0x606, digIOCheck);
    HAL_Delay(1);
    while(iomotor1!=0xFE)
    {
      CAN_SendTxMsg(0x606, digIOCheck);
      HAL_Delay(10);
    }
    CAN_SendTxMsg(0x606, stop);
    HAL_Delay(1);
    CAN_SendTxMsg(0x606, enable);
    HAL_Delay(1);
    CAN_SendTxMsg(0x606, resetEncoder);
    HAL_Delay(1);
  }
  //while motor7 not at origin
  if((iomotor1==0xFE)&&(iomotor2==0xFE)&&(iomotor3!=0xFE))
  {
    CAN_SendTxMsg(0x607, posStart);
    HAL_Delay(1);
    CAN_SendTxMsg(0x607, digIOCheck);
    HAL_Delay(1);
    while(iomotor1!=0xFE)
    {
      CAN_SendTxMsg(0x607, digIOCheck);
      HAL_Delay(1);
      HAL_Delay(10);
    }
    CAN_SendTxMsg(0x607, stop);
    HAL_Delay(1);
    CAN_SendTxMsg(0x607, enable);
    HAL_Delay(1);
    CAN_SendTxMsg(0x607, resetEncoder);
    HAL_Delay(1);
  }
  //while motor8 not at origin
  if((iomotor1==0xFE)&&(iomotor2==0xFE)&&(iomotor3==0xFE)&&(iomotor4!=0xFE))
  {
    CAN_SendTxMsg(0x608, posStart);HAL_Delay(1);
    CAN_SendTxMsg(0x608, digIOCheck);HAL_Delay(1);
    while(iomotor1!=0xFE)
    {
      CAN_SendTxMsg(0x608, digIOCheck);HAL_Delay(1);
      HAL_Delay(10);
    }
    CAN_SendTxMsg(0x608, stop);
    HAL_Delay(1);
    CAN_SendTxMsg(0x608, enable);
    HAL_Delay(1);
    CAN_SendTxMsg(0x608, resetEncoder);
    HAL_Delay(1);
  }
  //while all at origin
  if((iomotor1==0xFE)&&(iomotor2==0xFE)&&(iomotor3==0xFE)&&(iomotor4==0xFE))
  {
    originstate=1;
    btorigin=0;
  }
}
/** 
  * 函数功能: 串口接收中断回调函数
  * 输入参数: 串口句柄
  * 返 回 值: 无
  * 说    明: 使用一个定时器的比较中断和更新中断作为接收超时判断
  *           只要接收到数据就将定时器计数器清0,当发生比较中断的时候
  *           说明已经超时1.5个字符的时间,认定为帧错误,如果是更新中断
  *           则认为是接受完成
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart == &husart_debug_485)
  {
    switch(Rx_MSG)
    {
      /* 接收到第一个字符之后开始计时1.5/3.5个字符的时间 */
      case MSG_RX:
      //case MSG_IDLE:
        Rx_MSG = MSG_RXING;
        RxCount = 0;
        int ret;
        ret = HAL_TIM_Base_Start(&htimx);
        break;
      
      /* 距离上一次接收到数据已经超过1.5个字符的时间间隔,认定为数据帧不完整 */
      case MSG_ERR_FLAG:
        Rx_MSG = MSG_INC; // 数据帧不完整
      break;
    }
    
    /* 使能继续接收 */
    Rx_Buf[RxCount] = tmp_Rx_Buf;
    RxCount++;
    __HAL_TIM_SET_COUNTER(&htimx,0); // 重设计数时间
    HAL_UART_Receive_IT(&husart_debug_485,(uint8_t*)&tmp_Rx_Buf,1);
  }
}
/****************************************************************
             利用循环队列完成接收Modbus指令的发送工作
****************************************************************/
void modbus_Cmd_Q_Send(void)
{
  printf("modbus_Cmd_Q_Ctr = %d\n", modbus_Cmd_Q_Ctr);
  int cmd_index = modbus_Cmd_Q[modbus_Cmd_Q_Pop];
  modbus_Cmd_Temp=cmd_index;
  switch(cmd_index)
  {
  case BAT_READ_ALARM:
    MB_ReadHoldingReg_03H(0x01, 0x0043, 0x0001);
    //printf("send modbus read cmd\n");
    //HAL_Delay(100);
    //printf("time out\n");
    break;
  case BAT_READ_TEMPER:
    MB_ReadHoldingReg_03H(0x01, 0x0010, 0x0001);
    //printf("send modbus read cmd\n");
    //HAL_Delay(100);
    //printf("time out\n");
    break;
  case BAT_READ_POWER:
    MB_ReadHoldingReg_03H(0x01, 0x0014, 0x0001);
    //printf("send modbus read cmd\n");
    //HAL_Delay(100);
    //printf("time out\n");
    break;
  case BAT_READ_VOLTAGE:
    MB_ReadHoldingReg_03H(0x01, 0x000E, 0x0001);
    //printf("send modbus read cmd\n");
    //HAL_Delay(100);
    //printf("time out\n");
    break;
  case BAT_READ_CURRENT:
    MB_ReadHoldingReg_03H(0x01, 0x000F, 0x0001);
    //printf("send modbus read cmd\n");
    //HAL_Delay(100);
    //printf("time out\n");
    break;
  default:
    break;
  }
  modbus_Cmd_Q_Pop = (modbus_Cmd_Q_Pop + 1) % modbus_Cmd_Q_Max_Size;
  modbus_Cmd_Q_Ctr = modbus_Cmd_Q_Ctr - 1;
}
/******************************modbus消息处理********************************/
void modbus_Periodic_Handler(void)
{
  if(Rx_MSG==MSG_INC)
  {
    //printf("status:incomplete\n");
    Rx_MSG=MSG_IDLE;
  }
  if(Rx_MSG==MSG_COM)
  {
    /*printf("status:complete\n");
    for(int i=0;i<RxCount;i++)
    {
      printf("%x ",Rx_Buf[i]);
    }
    printf("\n");*/
    modbus_Data_Process();
  }
  if(Rx_MSG==MSG_IDLE)
  {
    if((modbus_Cmd_Q_Ctr <= modbus_Cmd_Q_Max_Size) && (modbus_Cmd_Q_Ctr > 0))
    {
      modbus_Cmd_Q_Send();
      Rx_MSG=MSG_RX;
    }
  }
}
void modbus_Data_Process(void)
{
  crc_check=MB_CRC16((uint8_t*)&Rx_Buf,RxCount-2);
  modbus_crc_check=(crc_check>>8)|(crc_check<<8);
  if(modbus_crc_check==(Rx_Buf[RxCount-2]<<8|Rx_Buf[RxCount-1]))
  {
    if(Rx_Buf[0]==0x01)
    {
      if(Rx_Buf[1]==0x03)
      {   
        if(Rx_Buf[2]==RxCount-5)
        {
          switch(modbus_Cmd_Temp)
          {
            case BAT_READ_ALARM:
              printf("battery alarm\n");
              bat_state.alarm_State=(Rx_Buf[3]<<8|Rx_Buf[4]);
              break;
            case BAT_READ_TEMPER:
              printf("battery temperature\n");
              bat_state.temper=(Rx_Buf[3]<<8|Rx_Buf[4]);
              break;
            case BAT_READ_POWER:
              printf("battery power\n");
              bat_state.power=(Rx_Buf[3]<<8|Rx_Buf[4]);
              bat_state.power=bat_state.power*4/10;
              break;
            case BAT_READ_VOLTAGE:
              printf("battery voltage\n");
              bat_state.voltage=(Rx_Buf[3]<<8|Rx_Buf[4]);
              break;
            case BAT_READ_CURRENT:
              printf("battery current\n");
              bat_state.current=(Rx_Buf[3]<<8|Rx_Buf[4]);
              break;
            default:
              break;  
          }
        }
        else
          printf("length error\n");
      }
      else
        printf("operate error\n");
    }
    else
      printf("peri error\n");
  }
  else
    printf("crc error\n");
  Rx_MSG=MSG_IDLE;
}
/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
