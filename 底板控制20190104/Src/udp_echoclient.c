/**
  ******************************************************************************
  * 文件名程: main.c 
  * 作    者: 硬石嵌入式开发团队
  * 版    本: V1.0
  * 编写日期: 2017-03-30
  * 功    能: UDP Client
  ******************************************************************************
  * 说明：
  * 本例程配套硬石stm32开发板YS-F4Pro使用。
  * 
  * 淘宝：
  * 论坛：http://www.ing10bbs.com
  * 版权归硬石嵌入式开发团队所有，请勿商用。
  ******************************************************************************
  */
#include "stm32f4xx_hal.h"
#include "main.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/tcp.h"
#include <string.h>
#include <stdio.h>
#include "udp_echoclient.h"
#include "CAN/bsp_CAN.h"

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
#define UDP_SERVER_PORT            8000  /* define the UDP local connection port */
#define UDP_CLIENT_PORT            1234   /* define the UDP remote connection port */

/* 私有变量 ------------------------------------------------------------------*/
struct udp_pcb *upcb;

uint8_t   data[100];
__IO uint32_t message_count = 0;

/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);

/* 函数体 --------------------------------------------------------------------*/
/**
  * 函数功能: UDP连接
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void udp_echoclient_connect(void)
{
  ip_addr_t DestIPaddr;
  err_t err;
  
  /* Create a new UDP control block  */
  upcb = udp_new();
  if (upcb!=NULL)
  {
    /*assign destination IP address */
    IP4_ADDR( &DestIPaddr, DEST_IP_ADDR0, DEST_IP_ADDR1, DEST_IP_ADDR2, DEST_IP_ADDR3 );
    
    /* configure destination IP address and port */
    err= udp_connect(upcb, &DestIPaddr, UDP_SERVER_PORT);
    
    if (err == ERR_OK)
    {
      err=udp_bind(upcb,IP_ADDR_ANY,UDP_CLIENT_PORT);
      if(err == ERR_OK)
      {
        /* Set a receive callback for the upcb */
        udp_recv(upcb, udp_receive_callback, NULL);          
      }
    }
  }
}

/**
  * 函数功能: UDP发送数据到服务器
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void udp_echoclient_send(void)
{
  struct pbuf *p;
  
  sprintf((char*)data, "sending udp client message %d\n", (int)message_count);
  
  /* allocate pbuf from pool*/
  p = pbuf_alloc(PBUF_TRANSPORT,strlen((char*)data), PBUF_POOL);
  
  if (p != NULL)
  {
    /* copy data to pbuf */
    pbuf_take(p, (char*)data, strlen((char*)data));
    
    /* send udp data */
    udp_send(upcb, p); 
    
    /* free pbuf */
    pbuf_free(p);
  }
}

/**
  * 函数功能: UDP发送数据到服务器
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void udp_send_t(uint8_t length)
{
  struct pbuf *p;
  //sprintf((char*)data, "sending udp client message %d\n", (int)message_count);
  
  /* allocate pbuf from pool*/
  p = pbuf_alloc(PBUF_TRANSPORT,length, PBUF_POOL);
  
  if (p != NULL)
  {
    /* copy data to pbuf */
    pbuf_take(p, data, length);
    
    /* send udp data */
    udp_send(upcb, p); 
    
    /* free pbuf */
    pbuf_free(p);
  }
}
/**
  * 函数功能: UDP数据接收回调函数
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
        uint8_t rec_data[4];
	char *recdata=0;
        //char *test1="test1";//从启动到100rpm
        //char *test2="test2";//切换转速
        //char *test3="test3";//停止，使能
  /*increment message count */
  message_count++;
	if(p !=NULL)
	{		
		recdata=(char *)malloc((p->len+1)*sizeof(char));
		if(recdata!=NULL)
		{
			//memcpy(recdata,p->payload,p->len+1);
                        memcpy(rec_data,p->payload,p->len+1);
                        
			//printf("upd_rec:%s\n",recdata);
                        //if(strcmp(recdata,test1)==0)
                        if(rec_data[0]==0x01)
                        {
                          hCAN.pTxMsg->StdId=0x0000;
                          hCAN.pTxMsg->IDE=CAN_ID_STD;					 /* 标准模式 */
                          hCAN.pTxMsg->RTR=CAN_RTR_DATA;				 /* 发送的是数据 */
                          hCAN.pTxMsg->DLC=2;							       /* 数据长度为2字节 */
                          hCAN.pTxMsg->Data[0]=0x01;
                          hCAN.pTxMsg->Data[1]=0x02;
                          HAL_CAN_Transmit(&hCAN,10);
                          
                          HAL_Delay(100);
                          hCAN.pTxMsg->StdId=0x201;
                          hCAN.pTxMsg->IDE=CAN_ID_STD;					 /* 标准模式 */
                          hCAN.pTxMsg->RTR=CAN_RTR_DATA;				 /* 发送的是数据 */
                          hCAN.pTxMsg->DLC=6;							       /* 数据长度为2字节 */
                          hCAN.pTxMsg->Data[0]=0x06;
                          hCAN.pTxMsg->Data[1]=0x00;
                          hCAN.pTxMsg->Data[2]=0x00;
                          hCAN.pTxMsg->Data[3]=0x00;
                          hCAN.pTxMsg->Data[4]=0x00;
                          hCAN.pTxMsg->Data[5]=0x00;
                          HAL_CAN_Transmit(&hCAN,10);
                          
                          HAL_Delay(100);
                          hCAN.pTxMsg->StdId=0x201;
                          hCAN.pTxMsg->IDE=CAN_ID_STD;					 /* 标准模式 */
                          hCAN.pTxMsg->RTR=CAN_RTR_DATA;				 /* 发送的是数据 */
                          hCAN.pTxMsg->DLC=6;							       /* 数据长度为2字节 */
                          hCAN.pTxMsg->Data[0]=0x07;
                          hCAN.pTxMsg->Data[1]=0x00;
                          hCAN.pTxMsg->Data[2]=0x00;
                          hCAN.pTxMsg->Data[3]=0x00;
                          hCAN.pTxMsg->Data[4]=0x00;
                          hCAN.pTxMsg->Data[5]=0x00;
                          HAL_CAN_Transmit(&hCAN,10);
                          
                          HAL_Delay(100);
                          hCAN.pTxMsg->StdId=0x201;
                          hCAN.pTxMsg->IDE=CAN_ID_STD;					 /* 标准模式 */
                          hCAN.pTxMsg->RTR=CAN_RTR_DATA;				 /* 发送的是数据 */
                          hCAN.pTxMsg->DLC=4;							       /* 数据长度为2字节 */
                          hCAN.pTxMsg->Data[0]=0x0f;
                          hCAN.pTxMsg->Data[1]=0x00;
                          hCAN.pTxMsg->Data[2]=0x64;
                          hCAN.pTxMsg->Data[3]=0x00;
                          HAL_CAN_Transmit(&hCAN,10);
                        }
                        //if(strcmp(recdata,test2)==0)
                        if(*rec_data==0x00000002)
                        {
                          hCAN.pTxMsg->StdId=0x0000;
                          hCAN.pTxMsg->IDE=CAN_ID_STD;					 /* 标准模式 */
                          hCAN.pTxMsg->RTR=CAN_RTR_DATA;				 /* 发送的是数据 */
                          hCAN.pTxMsg->DLC=2;							       /* 数据长度为2字节 */
                          hCAN.pTxMsg->Data[0]=0x01;
                          hCAN.pTxMsg->Data[1]=0x00;
                          HAL_CAN_Transmit(&hCAN,10);
                          
                          HAL_Delay(100);
                          hCAN.pTxMsg->StdId=0x201;
                          hCAN.pTxMsg->IDE=CAN_ID_STD;					 /* 标准模式 */
                          hCAN.pTxMsg->RTR=CAN_RTR_DATA;				 /* 发送的是数据 */
                          hCAN.pTxMsg->DLC=6;							       /* 数据长度为2字节 */
                          hCAN.pTxMsg->Data[0]=0x06;
                          hCAN.pTxMsg->Data[1]=0x00;
                          hCAN.pTxMsg->Data[2]=0x00;
                          hCAN.pTxMsg->Data[3]=0x00;
                          hCAN.pTxMsg->Data[4]=0x00;
                          hCAN.pTxMsg->Data[5]=0x00;
                          HAL_CAN_Transmit(&hCAN,10);
                          
                          HAL_Delay(100);
                          hCAN.pTxMsg->StdId=0x201;
                          hCAN.pTxMsg->IDE=CAN_ID_STD;					 /* 标准模式 */
                          hCAN.pTxMsg->RTR=CAN_RTR_DATA;				 /* 发送的是数据 */
                          hCAN.pTxMsg->DLC=6;							       /* 数据长度为2字节 */
                          hCAN.pTxMsg->Data[0]=0x07;
                          hCAN.pTxMsg->Data[1]=0x00;
                          hCAN.pTxMsg->Data[2]=0x00;
                          hCAN.pTxMsg->Data[3]=0x00;
                          hCAN.pTxMsg->Data[4]=0x00;
                          hCAN.pTxMsg->Data[5]=0x00;
                          HAL_CAN_Transmit(&hCAN,10);
                          hCAN.pTxMsg->StdId=0x201;
                          hCAN.pTxMsg->IDE=CAN_ID_STD;					 /* 标准模式 */
                          hCAN.pTxMsg->RTR=CAN_RTR_DATA;				 /* 发送的是数据 */
                          hCAN.pTxMsg->DLC=4;							       /* 数据长度为2字节 */
                          hCAN.pTxMsg->Data[0]=0x0f;
                          hCAN.pTxMsg->Data[1]=0x00;
                          hCAN.pTxMsg->Data[2]=0xE8;
                          hCAN.pTxMsg->Data[3]=0x03;
                          HAL_CAN_Transmit(&hCAN,10);
                        }
                        //if(strcmp(recdata,test3)==0)
                        if(*rec_data==0x00000003)
                        {
                          hCAN.pTxMsg->StdId=0x201;
                          hCAN.pTxMsg->IDE=CAN_ID_STD;					 /* 标准模式 */
                          hCAN.pTxMsg->RTR=CAN_RTR_DATA;				 /* 发送的是数据 */
                          hCAN.pTxMsg->DLC=4;							       /* 数据长度为2字节 */
                          hCAN.pTxMsg->Data[0]=0x06;
                          hCAN.pTxMsg->Data[1]=0x00;
                          hCAN.pTxMsg->Data[2]=0x00;
                          hCAN.pTxMsg->Data[3]=0x00;
                          HAL_CAN_Transmit(&hCAN,10);
                          
                          HAL_Delay(100);
                          hCAN.pTxMsg->StdId=0x201;
                          hCAN.pTxMsg->IDE=CAN_ID_STD;					 /* 标准模式 */
                          hCAN.pTxMsg->RTR=CAN_RTR_DATA;				 /* 发送的是数据 */
                          hCAN.pTxMsg->DLC=6;							       /* 数据长度为2字节 */
                          hCAN.pTxMsg->Data[0]=0x07;
                          hCAN.pTxMsg->Data[1]=0x00;
                          hCAN.pTxMsg->Data[2]=0x00;
                          hCAN.pTxMsg->Data[3]=0x00;
                          hCAN.pTxMsg->Data[4]=0x00;
                          hCAN.pTxMsg->Data[5]=0x00;
                          HAL_CAN_Transmit(&hCAN,10);
                        }
		}
		free(recdata);
		udp_send(upcb,p);
		/* Free receive pbuf */
		pbuf_free(p);
	}
}

/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE*************************/
