/**
  ******************************************************************************
  * �ļ�����: main.c 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2017-03-30
  * ��    ��: UDP Client
  ******************************************************************************
  * ˵����
  * ����������Ӳʯstm32������YS-F4Proʹ�á�
  * 
  * �Ա���
  * ��̳��http://www.ing10bbs.com
  * ��Ȩ��ӲʯǶ��ʽ�����Ŷ����У��������á�
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

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
#define UDP_SERVER_PORT            8000  /* define the UDP local connection port */
#define UDP_CLIENT_PORT            1234   /* define the UDP remote connection port */

/* ˽�б��� ------------------------------------------------------------------*/
struct udp_pcb *upcb;

unsigned char qidong[8]={0x2B,0x40,0x60,0x00,0x06,0x00,0x00,0x00};
unsigned char baozha[8]={0x2B,0x40,0x60,0x00,0x07,0x00,0x00,0x00};
unsigned char zhunbei[8]={0x2B,0x40,0x60,0x00,0x0F,0x00,0x00,0x00};
unsigned char rpm100[8]={0x2B,0xFF,0x60,0x00,0x64,0x00,0x00,0x00};
unsigned char rpm500[8]={0x2B,0xFF,0x60,0x00,0xF4,0x01,0x00,0x00};
unsigned char dusudu[8]={0x40,0x6C,0x60,0x00,0x00,0x00,0x00,0x00};
unsigned char xiufu[8]={0x2B,0x40,0x60,0x00,0x80,0x00,0x00,0x00};
//control instruction
unsigned char enable[8] = {0x2B, 0x00, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00};
unsigned char release[8] = {0x2B, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char stop[8] = {0x2B, 0x03, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00};
unsigned char resetEncoder[8] = {0x2B, 0x04, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00};
unsigned char alarmclear[8] = {0x2B, 0x05, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00};
unsigned char setSpeed[8] = {0x2B, 0xFF, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
//check instruction
unsigned char enStateCheck[8] = {0x40, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char givenSpeedCheck[8] = {0x40, 0x6B, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char actSpeedCheck[8] = {0x40, 0x6C, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char actCurrentCheck[8] = {0x40, 0x78, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char errorCheck[8] = {0x40, 0x0B, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char positionCheck[8] = {0x40, 0x63, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
uint8_t   data[100];
__IO uint32_t message_count = 0;

/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);

/* ������ --------------------------------------------------------------------*/
/**
  * ��������: UDP����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
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
  * ��������: UDP�������ݵ�������
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
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
  * ��������: UDP�������ݵ�������
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
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
  * ��������: UDP���ݽ��ջص�����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
        uint8_t rec_data[4];
	char *recdata=0;
        //char *test1="test1";//��������100rpm
        //char *test2="test2";//�л�ת��
        //char *test3="test3";//ֹͣ��ʹ��
  /*increment message count */
  //message_count++;
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
                           
                          CAN_SendTxMsg(0x601,qidong);                        
                          //HAL_Delay(100);
                          CAN_SendTxMsg(0x601,baozha);
                          //HAL_Delay(100);
                        }
                        //if(strcmp(recdata,test2)==0)
                        if(rec_data[0]==0x02)
                        {
                          CAN_SendTxMsg(0x601,positionCheck);
                          HAL_Delay(1);
                          CAN_SendTxMsg(0x601,errorCheck);
                          HAL_Delay(1);
                          CAN_SendTxMsg(0x601,actSpeedCheck);
                          HAL_Delay(1);
                          CAN_SendTxMsg(0x601,actCurrentCheck);
                          HAL_Delay(1);
                          //CAN_SendTxMsg(0x601,zhunbei);                          
                          //HAL_Delay(100);
                          //CAN_SendTxMsg(0x601,rpm100);                          
                          //HAL_Delay(100);
                        }
                        //if(strcmp(recdata,test3)==0)
                        if(rec_data[0]==0x03)
                        {
                          
                          //CAN_SendTxMsg(0x601,rpm500);
                          //HAL_Delay(100);
                        }
                        if(rec_data[0]==0x04)
                        {
                          
                          //CAN_SendTxMsg(0x601,stop);
                          //HAL_Delay(100);
                        }
                        if(rec_data[0]==0x05)
                        {
                          
                          //CAN_SendTxMsg(0x601,dusudu);                          
                          //HAL_Delay(100);
                        }
                        if(rec_data[0]==0x06)
                        {
                          //CAN_SendTxMsg(0x601,xiufu);                          
                          //HAL_Delay(100);
                        }
		}
		free(recdata);
		udp_send(upcb,p);
		/* Free receive pbuf */
		pbuf_free(p);
	}
}

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE*************************/
