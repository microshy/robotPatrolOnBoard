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
#include "speedcal/SPEED.h"
#include "usart/bsp_debug_usart.h"
    
/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
#define UDP_SERVER_PORT            8000  /* define the UDP local connection port */
#define UDP_CLIENT_PORT            1234   /* define the UDP remote connection port */

/* 私有变量 ------------------------------------------------------------------*/
struct udp_pcb *upcb;

//control instruction
unsigned char enable[8] = {0x2B, 0x00, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00};
unsigned char release[8] = {0x2B, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char stop[8] = {0x2B, 0x03, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00};
unsigned char resetEncoder[8] = {0x2B, 0x04, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00};
unsigned char alarmclear[8] = {0x2B, 0x05, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00};
unsigned char setSpeed[8] = {0x2B, 0xFF, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char posSetPosi[8] = {0x23, 0x7A, 0x60, 0x00, 0x00, 0x00, 0xFF, 0xFF};
unsigned char posSetNega[8] = {0x23, 0x7A, 0x60, 0x00, 0x80, 0x00, 0xFF, 0xFF};
unsigned char posSet[8] = {0x23, 0x7A, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char posModeAbs[8] = {0x2B, 0x06, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00};
unsigned char posStart[8] = {0x2B, 0x01, 0x20, 0x00, 0x01, 0x00, 0x00, 0x00};
//check instruction
unsigned char check[8] = {0x40, 0x40, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char enStateCheck[8] = {0x40, 0x00, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char givenSpeedCheck[8] = {0x40, 0x6B, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char actSpeedCheck[8] = {0x40, 0x6C, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char actCurrentCheck[8] = {0x40, 0x78, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char errorCheck[8] = {0x40, 0x0B, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char positionCheck[8] = {0x40, 0x63, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char driverFlagCheck[8] = {0x40, 0x90, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
unsigned char digIOCheck[8]={0x40,0x90,0x60,0x00,0x00,0x00,0x00,0x00};

short int ultraDis=0;
short int dropDis=0;
unsigned char ultraUp = 0;
unsigned char ultraDown = 0;
unsigned char emestopall = 0;
unsigned char enstateall = 0;
unsigned char selfspeedcontrol;
unsigned char turn90=0;
unsigned char posdone=0;
unsigned char breakclear1,breakclear2,breakclear3,breakclear4;
unsigned char btorigin1,btorigin2,btorigin3,btorigin4;
unsigned char readymotor1,readymotor2,readymotor3,readymotor4;
unsigned char seekoncedone1,seekoncedone2,seekoncedone3,seekoncedone4;
unsigned char origindone;
unsigned char origindone1,origindone2,origindone3,origindone4;
unsigned char originstate=0x01;
unsigned char originrec;
unsigned char moveflag2=0;
unsigned char brake=0;
int btorigin=0;
int motor1ang=0;
int motor2ang=0;
int motor3ang=0;
int motor4ang=0;
int l1ang,l2ang,r1ang,r2ang;
short third;
short int LSPPED;
short int RSPPED;
unsigned char l1speedset[8]={0x2B,0xFF,0x60,0x00,0x00,0x00,0x00,0x00};
unsigned char	r1speedset[8]={0x2B,0xFF,0x60,0x00,0x00,0x00,0x00,0x00};
unsigned char	l2speedset[8]={0x2B,0xFF,0x60,0x00,0x00,0x00,0x00,0x00};
unsigned char	r2speedset[8]={0x2B,0xFF,0x60,0x00,0x00,0x00,0x00,0x00};
int MOTOR1Alarm=0;
int MOTOR2Alarm=0;
int MOTOR3Alarm=0;
int MOTOR4Alarm=0;
int l1angset,r1angset,l2angset,r2angset;
short int vhall;
short int whall;
unsigned char timloss=0;
unsigned char timfault=0;
uint8_t   databuffer[150];
uint8_t   data[100];
__IO uint32_t message_count = 0;
unsigned int  TOLLEN = 0;
unsigned char crc8 = 0;
unsigned char crc_array[256] = {
	0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83,
	0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
	0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e,
	0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
	0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0,
	0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
	0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d,
	0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
	0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5,
	0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
	0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58,
	0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
	0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6,
	0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
	0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b,
	0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
	0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f,
	0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
	0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92,
	0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
	0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c,
	0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
	0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1,
	0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
	0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49,
	0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
	0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4,
	0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
	0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a,
	0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
	0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7,
	0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};

/* 扩展变量 ------------------------------------------------------------------*/
extern struct MOTORpar MOTOR1,MOTOR2,MOTOR3,MOTOR4,MOTOR5,MOTOR6,MOTOR7,MOTOR8;
extern struct Battery_State bat_state;
/* 私有函数原形 --------------------------------------------------------------*/
void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);

/* 函数体 --------------------------------------------------------------------*/
/**
  * 函数功能: UDP连接
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
ip_addr_t DestIPaddr;
void udp_echoclient_connect(void)
{
  //ip_addr_t DestIPaddr;
  //ip_addr_t LocalIPaddr;
  err_t err;
  
  /* Create a new UDP control block  */
  upcb = udp_new();
  if (upcb!=NULL)
  {
    /*assign destination IP address */
    IP4_ADDR( &DestIPaddr, DEST_IP_ADDR0, DEST_IP_ADDR1, DEST_IP_ADDR2, DEST_IP_ADDR3 );
    
    err=udp_bind(upcb,IP_ADDR_ANY,UDP_CLIENT_PORT);
    
    if (err == ERR_OK)
    {
      printf("erro1\n");
      /* configure destination IP address and port */
      //err= udp_connect(upcb, &DestIPaddr, UDP_SERVER_PORT);
      
      if(err == ERR_OK)
      {
        printf("erro2\n");
        /* Set a receive callback for the upcb */
        udp_recv(upcb, udp_receive_callback, NULL);          
      }
    }
  }
  printf("connect ok\n");
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

void udp_send_t2(uint8_t length, u16_t port)
{
  struct pbuf *p;
  //sprintf((char*)data, "sending udp client message %d\n", (int)message_count);
  
  /* allocate pbuf from pool*/
  p = pbuf_alloc(PBUF_TRANSPORT,length, PBUF_POOL);
  
  if (p != NULL)
  {
    /* copy data to pbuf */
    pbuf_take(p, databuffer, length);
    
    /* send udp data */
    udp_sendto(upcb, p, &DestIPaddr, port); 
    
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
extern unsigned char can_Cmd_Q[can_Cmd_Q_Max_Size];
extern int can_Cmd_Q_Push;
extern int can_Cmd_Q_Pop;
extern int can_Cmd_Q_Ctr;
extern int total_cnt;
void udp_receive_callback(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port)
{
  //struct ip4_addr destAddr = *addr;
  //struct pbuf *p_temp;
  short tempPara;
  unsigned int i;
  unsigned int length;
  unsigned char *bufferAddr;
  int cde;
  length = 0;
  //memcpy(p_temp, p, p->len);
  //bufferAddr=(unsigned char*)p_temp->payload;
  bufferAddr=(unsigned char*)p->payload;
  
  for (i = 0; i < 5; i++)
  {
    databuffer[i] = *(bufferAddr + i);
  }
  if(databuffer[0] == Head)
  {
    length = (int)(((length|databuffer[3])<<8) | databuffer[4]) + 7;  
    for (i = 0; i < length - 1; i++)
    {
      databuffer[i] = *(bufferAddr + i);	
    }
    cde = p->len;
    if (length == cde)
    {
      //for (i = 0; i <  length - 2; i++)
      //{
      //  crc8 = crc_array[crc8^(unsigned char)(databuffer[i])];
      //}
      //if (crc8 == databuffer[length - 2])
      {
        if (databuffer[2] == LocalID)
        {
          switch(databuffer[5])
          {
          case 0x00:
            //total_cnt+=1;
            //printf("total_cnt = %d\n", total_cnt);
            if(databuffer[6]==0x01)
	    {
              databuffer[3]=0x00;
	      databuffer[4]=0x02;
	      databuffer[5]=0x00;
              databuffer[6]=0x01;   
	      databuffer[7]=0;
	      for(i=0;i<7;i++)
	        databuffer[7]=crc_array[databuffer[7]^(unsigned char)(databuffer[i])];
	      databuffer[8]=0xFF;
            }
	    TOLLEN=0x09;
            if((can_Cmd_Q_Ctr <= can_Cmd_Q_Max_Size - 1) && can_Cmd_Q_Ctr>=0)
            { 
              can_Cmd_Q[can_Cmd_Q_Push] = MOVE_MOTOR_INIT;
              can_Cmd_Q_Push = (can_Cmd_Q_Push + 1) % can_Cmd_Q_Max_Size;
              can_Cmd_Q_Ctr = can_Cmd_Q_Ctr + 1;
              //printf("can_Cmd_Q_Ctr = %d\n", can_Cmd_Q_Ctr);
            }
            break;
          case 0x10:
            if(databuffer[6]==0x11)
              timloss=databuffer[7];
            if(databuffer[8]==0x12)
              timfault=databuffer[9];
            if(databuffer[10]==0x13)
              {
                ultraDis=databuffer[11];                 
                dropDis=databuffer[12];  
              }
            databuffer[3]=0x00;
            databuffer[4]=0x02;
            databuffer[5]=0x10;
            databuffer[6]=0x01;   
            databuffer[7]=0x00;
            for(i=0;i<7;i++)
              databuffer[7]=crc_array[databuffer[7]^(unsigned char)(databuffer[i])];
            databuffer[8]=0xFF;
            TOLLEN=0x09;
            break;
          case 0x20:
            databuffer[3]=0x00;
            databuffer[4]=0x48;
            databuffer[5]=0x20;
            databuffer[6]=0x01;
            databuffer[7]=0x21;
            databuffer[8]=0;
            databuffer[9]=0;
            databuffer[10]=ultraUp;
            databuffer[11]=ultraDown;
            databuffer[12]=0x22;
            databuffer[13]=emestopall;
            databuffer[14]=0x23;
            databuffer[15]=enstateall;
            databuffer[16]=0x24;
            databuffer[18]=MOTOR1.given;
            databuffer[17]=(MOTOR1.given>>8);
            databuffer[20]=MOTOR2.given;
            databuffer[19]=(MOTOR2.given>>8);
            databuffer[22]=MOTOR3.given;
            databuffer[21]=(MOTOR3.given>>8);
            databuffer[24]=MOTOR4.given;
            databuffer[23]=(MOTOR4.given>>8);
            databuffer[25]=0x25;
            tempPara=MOTOR1.speed;
            databuffer[27]=tempPara;
            databuffer[26]=(tempPara>>8);
            tempPara=MOTOR2.speed;
            databuffer[29]=tempPara;
            databuffer[28]=(tempPara>>8);
            tempPara=MOTOR3.speed;
            databuffer[31]=tempPara;
            databuffer[30]=(tempPara>>8);
            tempPara=MOTOR4.speed;
            databuffer[33]=tempPara;
            databuffer[32]=(tempPara>>8);
            databuffer[34]=0x26;
            tempPara=MOTOR1.current;
            databuffer[36]=tempPara;            
            databuffer[35]=(tempPara>>8);
            tempPara=MOTOR2.current;
            databuffer[38]=tempPara;
            databuffer[37]=(tempPara>>8);
            tempPara=MOTOR3.current;
            databuffer[40]=tempPara;
            databuffer[39]=(tempPara>>8);
            tempPara=MOTOR4.current;
            databuffer[42]=tempPara;
            databuffer[41]=(tempPara>>8); 
            databuffer[43]=0x27;
            databuffer[47]=MOTOR1.fault;
            databuffer[46]=MOTOR1.fault>>8;
            databuffer[45]=MOTOR1.fault>>16;
            databuffer[44]=MOTOR1.fault>>24;
            databuffer[51]=MOTOR2.fault;
            databuffer[50]=MOTOR2.fault>>8;
            databuffer[49]=MOTOR2.fault>>16;
            databuffer[48]=MOTOR2.fault>>24;
            databuffer[55]=MOTOR3.fault;
            databuffer[54]=MOTOR3.fault>>8;
            databuffer[53]=MOTOR3.fault>>16;
            databuffer[52]=MOTOR3.fault>>24;
            databuffer[59]=MOTOR4.fault;
            databuffer[58]=MOTOR4.fault>>8;
            databuffer[57]=MOTOR4.fault>>16;
            databuffer[56]=MOTOR4.fault>>24;
            databuffer[60]=0x28;
            databuffer[64]=MOTOR1.pos;
            databuffer[63]=MOTOR1.pos>>8;
            databuffer[62]=MOTOR1.pos>>16;
            databuffer[61]=MOTOR1.pos>>24;
            databuffer[68]=MOTOR2.pos;
            databuffer[67]=MOTOR2.pos>>8;
            databuffer[66]=MOTOR2.pos>>16;
            databuffer[65]=MOTOR2.pos>>24;
            databuffer[72]=MOTOR3.pos;
            databuffer[71]=MOTOR3.pos>>8;
            databuffer[70]=MOTOR3.pos>>16;
            databuffer[69]=MOTOR3.pos>>24;
            databuffer[76]=MOTOR4.pos;
            databuffer[75]=MOTOR4.pos>>8;
            databuffer[74]=MOTOR4.pos>>16;
            databuffer[73]=MOTOR4.pos>>24;		
            databuffer[78]=0xFF;
            databuffer[77]=0x00;
            for(i=0;i<77;i++)
            {
              databuffer[77]=crc_array[databuffer[77]^(unsigned char)(databuffer[i])];
            }
            TOLLEN=0x4F;
            break;
          case 0x30:
            if( databuffer[6]==0x01)
            {
              TOLLEN=0x09;
              databuffer[6]=0x01;
              databuffer[3]=0x00;
              databuffer[4]=0x02;
              databuffer[7]=0;
              databuffer[8]=0xFF;
              for(i=0;i<7;i++)
                databuffer[7]=crc_array[databuffer[7]^(unsigned char)(databuffer[i])];
              if((can_Cmd_Q_Ctr <= can_Cmd_Q_Max_Size - 1) && can_Cmd_Q_Ctr>=0)
              { 
                can_Cmd_Q[can_Cmd_Q_Push] = MOVE_MOTOR_STOP;
                can_Cmd_Q_Push = (can_Cmd_Q_Push + 1) % can_Cmd_Q_Max_Size;
                can_Cmd_Q_Ctr = can_Cmd_Q_Ctr + 1;
                //printf("can_Cmd_Q_Ctr = %d\n", can_Cmd_Q_Ctr);
              }
              __set_FAULTMASK(1);
              NVIC_SystemReset();
            }
            break;
          case 0x40:
            if( databuffer[6]==0x41)
              vhall =(((vhall&0)|databuffer[7])<<8)|databuffer[8];
            if(databuffer[9]==0x42)
              whall =(((whall&0)|databuffer[10])<<8)|databuffer[11];
            if(vhall<=0)
            {
              selfspeedcontrol=0;
            }
            if( databuffer[14]==0x44)
            {
              if(databuffer[15]==0x01)
              {
                turn90=1;
                selfspeedcontrol=0;
              } 
              else
              {
                turn90=0;
              }
            }						 
            LSPPED=LSPEEDca(vhall,whall);
            RSPPED=RSPEEDca(vhall,whall);
            //LSPPED = RSPPED = vhall;
            if(turn90==1)
            {
              RSPPED=-RSPPED;
            }
            setSpeed[4]=vhall;
            setSpeed[5]=vhall>>8;
            if((can_Cmd_Q_Ctr <= can_Cmd_Q_Max_Size - 1) && can_Cmd_Q_Ctr>=0)
            { 
              can_Cmd_Q[can_Cmd_Q_Push] = MOVE_MOTOR_PARA_SET;
              can_Cmd_Q_Push = (can_Cmd_Q_Push + 1) % can_Cmd_Q_Max_Size;
              can_Cmd_Q_Ctr = can_Cmd_Q_Ctr + 1;
              //printf("can_Cmd_Q_Ctr = %d\n", can_Cmd_Q_Ctr);
            }            
            
            /*setSpeed[4]=LSPPED;
            setSpeed[5]=LSPPED>>8;
            CAN_SendTxMsg(0x602, setSpeed);
            setSpeed[4]=LSPPED;
            setSpeed[5]=LSPPED>>8;
            CAN_SendTxMsg(0x603, setSpeed);
            setSpeed[4]=LSPPED;
            setSpeed[5]=LSPPED>>8;
            CAN_SendTxMsg(0x604, setSpeed);*/
            if( databuffer[12]==0x43)
            {
              if(databuffer[13]==0x01)
              {
                if((can_Cmd_Q_Ctr <= can_Cmd_Q_Max_Size - 1) && can_Cmd_Q_Ctr>=0)
                { 
                  can_Cmd_Q[can_Cmd_Q_Push] = MOVE_MOTOR_STOP;
                  can_Cmd_Q_Push = (can_Cmd_Q_Push + 1) % can_Cmd_Q_Max_Size;
                  can_Cmd_Q_Ctr = can_Cmd_Q_Ctr + 1;
                  //printf("can_Cmd_Q_Ctr = %d\n", can_Cmd_Q_Ctr);
                }
                l1speedset[4]=0;
                l1speedset[5]=0;
                r1speedset[4]=0;
                r1speedset[5]=0;
                emestopall=1;
              } 
              else
              {
                emestopall=0;
              }
            }	
            databuffer[3]=0x00;
            databuffer[4]=0x02;
            databuffer[5]=0x40;
            databuffer[6]=0x01;
            databuffer[7]=0;
            for(i=0;i<7;i++)
              databuffer[7]=crc_array[databuffer[7]^(unsigned char)(databuffer[i])];
            databuffer[8]=0xFF;
            TOLLEN=0x09;
            break;
          case 0x50:
            if((can_Cmd_Q_Ctr <= can_Cmd_Q_Max_Size - 1) && can_Cmd_Q_Ctr>=0)
            { 
              can_Cmd_Q[can_Cmd_Q_Push] = MOVE_MOTOR_ALARM_CLR;
              can_Cmd_Q_Push = (can_Cmd_Q_Push + 1) % can_Cmd_Q_Max_Size;
              can_Cmd_Q_Ctr = can_Cmd_Q_Ctr + 1;
              ///printf("can_Cmd_Q_Ctr = %d\n", can_Cmd_Q_Ctr);
            }
            databuffer[5]=0x50;
            databuffer[6]=0x01;
            databuffer[3]=0x00;
            databuffer[4]=0x02;
            databuffer[7]=0;
            for(i=0;i<7;i++)
              databuffer[7]=crc_array[databuffer[7]^(unsigned char)(databuffer[i])];
            databuffer[8]=0xFF;
            TOLLEN=0x09;
            break;
          }
        }
        else if (databuffer[2] == LocalID3)
        {
          switch(databuffer[5])
          {
          case 0x00:
            if(databuffer[6]==0x01)
	    {
              databuffer[3]=0x00;
	      databuffer[4]=0x02;
	      databuffer[5]=0x00;
              databuffer[6]=0x01;   
	      databuffer[7]=0;
	      for(i=0;i<7;i++)
	        databuffer[7]=crc_array[databuffer[7]^(unsigned char)(databuffer[i])];
	      databuffer[8]=0xFF;
            }
	    TOLLEN=0x09;
            break;
          case 0x10:
            if(databuffer[6]==0x11)
              //power_threshold=databuffer[7];
            if(databuffer[8]==0x12)
              //power_warn_threshold=databuffer[9];
            if(databuffer[10]==0x13)
              //com_int_time=databuffer[11];
            if(databuffer[12]==0x14)
              //com_err_time=databuffer[13];
            if(databuffer[14]==0x15)
            {
              //rob_temp_l=databuffer[15];
              //rob_temp_h=databuffer[16];
            }
            /*if((can_Cmd_Q_Ctr <= can_Cmd_Q_Max_Size - 1) && can_Cmd_Q_Ctr>=0)
            { 
              can_Cmd_Q[can_Cmd_Q_Push] = POWER_SET;
              can_Cmd_Q_Push = (can_Cmd_Q_Push + 1) % 100;
              can_Cmd_Q_Ctr = can_Cmd_Q_Ctr + 1;
            }*/
            databuffer[3]=0x00;
            databuffer[4]=0x02;
            databuffer[5]=0x10;
            databuffer[6]=0x01;   
            databuffer[7]=0;
            for(i=0;i<7;i++)
              databuffer[7]=crc_array[databuffer[7]^(unsigned char)(databuffer[i])];
            databuffer[8]=0xFF;
            TOLLEN=0x09;
            break;
          case 0x20:
            databuffer[3]=0x00;
            databuffer[4]=0x1E;
            databuffer[5]=0x20;
            databuffer[6]=0x01;
            databuffer[7]=0x22;
            databuffer[8]=bat_state.alarm_State>>8;
            databuffer[9]=bat_state.alarm_State;
            databuffer[10]=0x24;
            databuffer[11]=0x00;
            databuffer[12]=0x00;
            databuffer[13]=0x25;
            databuffer[14]=bat_state.temper>>8;
            databuffer[15]=bat_state.temper;
            databuffer[16]=0x26;
            databuffer[17]=0x40;
            databuffer[18]=0x27;
            databuffer[19]=0x00;
            databuffer[20]=0x00;
            databuffer[21]=0x00;
            databuffer[22]=0x00;
            databuffer[23]=0x00;
            databuffer[24]=0x00;
            databuffer[25]=0x00;
            databuffer[26]=0x00;
            databuffer[27]=0x28;
            databuffer[28]=bat_state.power;
            databuffer[29]=0x29;
            databuffer[30]=bat_state.voltage>>8;
            databuffer[31]=bat_state.voltage;
            databuffer[32]=0x2a;
            databuffer[33]=bat_state.current>>8;
            databuffer[34]=bat_state.current;
            databuffer[35]=0;
            for(i=0;i<35;i++)
              databuffer[35]=crc_array[databuffer[35]^(unsigned char)(databuffer[i])];
            databuffer[36]=0xFF;
            TOLLEN=0x25;
            break;
          case 0x30:
            databuffer[3]=0x00;
            databuffer[4]=0x02;
            databuffer[5]=0x30;
            databuffer[6]=0x01;   
            databuffer[7]=0;
            for(i=0;i<7;i++)
              databuffer[7]=crc_array[databuffer[7]^(unsigned char)(databuffer[i])];
            databuffer[8]=0xFF;
            TOLLEN=0x09;
            break;
          case 0x50:
            databuffer[3]=0x00;
            databuffer[4]=0x02;
            databuffer[5]=0x50;
            databuffer[6]=0x01;   
            databuffer[7]=0;
            for(i=0;i<7;i++)
              databuffer[7]=crc_array[databuffer[7]^(unsigned char)(databuffer[i])];
            databuffer[8]=0xFF;
            TOLLEN=0x09;
            break;
          case 0x60:
            databuffer[3]=0x00;
            databuffer[4]=0x85;
            databuffer[5]=0x60;
            //132字节数据
            databuffer[138]=0;
            for(i=0;i<138;i++)
              databuffer[138]=crc_array[databuffer[138]^(unsigned char)(databuffer[i])];
            databuffer[139]=0xFF;
            TOLLEN=0x8B;
            break;
          }
        }
        else if (databuffer[2] == LocalID2)
        {
          switch(databuffer[5])
          {
          case 0x00:
            if(databuffer[6]==0x01)
	    {
              databuffer[3]=0x00;
	      databuffer[4]=0x02;
	      databuffer[5]=0x00;
              databuffer[6]=0x01;   
	      databuffer[7]=0;
	      for(i=0;i<7;i++)
	        databuffer[7]=crc_array[databuffer[7]^(unsigned char)(databuffer[i])];
	      databuffer[8]=0xFF;
            }
	    TOLLEN=0x09;
            if((can_Cmd_Q_Ctr <= can_Cmd_Q_Max_Size - 1) && can_Cmd_Q_Ctr>=0)
            { 
              can_Cmd_Q[can_Cmd_Q_Push] = ROUND_MOTOR_INIT;
              can_Cmd_Q_Push = (can_Cmd_Q_Push + 1) % can_Cmd_Q_Max_Size;
              can_Cmd_Q_Ctr = can_Cmd_Q_Ctr + 1;
              //printf("can_Cmd_Q_Ctr = %d\n", can_Cmd_Q_Ctr);
            }
            break;
          case 0x10:
            if(databuffer[6]==0x11)
              timloss=databuffer[7];
            databuffer[3]=0x00;
            databuffer[4]=0x02;
            databuffer[5]=0x10;
            databuffer[6]=0x01;   
            databuffer[7]=0x00;
            for(i=0;i<7;i++)
              databuffer[7]=crc_array[databuffer[7]^(unsigned char)(databuffer[i])];
            databuffer[8]=0xFF;
            TOLLEN=0x09;
            break;
          case 0x20:
            databuffer[5]=0x20;
            databuffer[6]=0x01;
            databuffer[7]=0x21;
            databuffer[8]=emestopall;
            databuffer[9]=0x22;
            databuffer[10]=enstateall;				 				 
            databuffer[11]=0x23;			 
            databuffer[13]=MOTOR5.current;
            third=MOTOR1.current;
            databuffer[12]=(third>>8);
            databuffer[15]=MOTOR6.current;
            third=MOTOR2.current;
            databuffer[14]=(third>>8);
            databuffer[17]=MOTOR7.current;
            third=MOTOR3.current;
            databuffer[16]=(third>>8);
            databuffer[19]=MOTOR8.current;
            third=MOTOR4.current;
            databuffer[18]=(third>>8);	
            databuffer[20]=0x24;
            databuffer[24]=MOTOR5.fault;
            databuffer[23]=MOTOR5.fault>>8;
            databuffer[22]=MOTOR5.fault>>16;
            databuffer[21]=MOTOR5.fault>>24;
            databuffer[28]=MOTOR6.fault;
            databuffer[27]=MOTOR6.fault>>8;
            databuffer[26]=MOTOR6.fault>>16;
            databuffer[25]=MOTOR6.fault>>24;
            databuffer[32]=MOTOR7.fault;
            databuffer[31]=MOTOR7.fault>>8;
            databuffer[30]=MOTOR7.fault>>16;
            databuffer[29]=MOTOR7.fault>>24;
            databuffer[36]=MOTOR8.fault;
            databuffer[35]=MOTOR8.fault>>8;
            databuffer[34]=MOTOR8.fault>>16;
            databuffer[33]=MOTOR8.fault>>24;
            databuffer[37]=0x25;
            databuffer[38]=posdone;
            databuffer[39]=0x26;
            databuffer[40]=motor1ang>>24;
            databuffer[41]=motor1ang>>16;
            databuffer[42]=motor1ang>>8;
            databuffer[43]=motor1ang;
            databuffer[44]=motor2ang>>24;
            databuffer[45]=motor2ang>>16;
            databuffer[46]=motor2ang>>8;
            databuffer[47]=motor2ang;
            databuffer[48]=motor3ang>>24;
            databuffer[49]=motor3ang>>16;
            databuffer[50]=motor3ang>>8;
            databuffer[51]=motor3ang;                                          
            databuffer[52]=motor4ang>>24;
            databuffer[53]=motor4ang>>16;
            databuffer[54]=motor4ang>>8;
            databuffer[55]=motor4ang;
            databuffer[56]=0x27;
            databuffer[57]=1;//lifterstate;
            databuffer[58]=0x28;
            databuffer[59]=0x00;//height_high_H;
            databuffer[60]=0x00;//height_high_L;
            databuffer[3]=0x00;
            databuffer[4]=0x3A;
            databuffer[61]=0x29;
            databuffer[62]=originstate;
            //databuffer[62]=0x01;
            databuffer[63]=0x00;
            /********************************************************************************/
            databuffer[64]=0xFF;
            //databuffer[64]=databuffer[65]=databuffer[66]=databuffer[67]=databuffer[68]=databuffer[69]=0x01;
            //databuffer[70]=0x00;
            //databuffer[71]=0xFF;
            for(i=0;i<63;i++)
            {
              databuffer[63]=crc_array[databuffer[63]^(unsigned char)(databuffer[i])];
            }
            TOLLEN=0x41;
            break;
          case 0x30:
            if( databuffer[6]==0x01)
            {
              TOLLEN=0x09;
              databuffer[6]=0x01;
              databuffer[3]=0x00;
              databuffer[4]=0x02;
              databuffer[7]=0;
              databuffer[8]=0xFF;
              for(i=0;i<7;i++)
                databuffer[7]=crc_array[databuffer[7]^(unsigned char)(databuffer[i])];
              if((can_Cmd_Q_Ctr <= can_Cmd_Q_Max_Size - 1) && can_Cmd_Q_Ctr>=0)
              { 
                can_Cmd_Q[can_Cmd_Q_Push] = ROUND_MOTOR_STOP;
                can_Cmd_Q_Push = (can_Cmd_Q_Push + 1) % can_Cmd_Q_Max_Size;
                can_Cmd_Q_Ctr = can_Cmd_Q_Ctr + 1;
                //printf("can_Cmd_Q_Ctr = %d\n", can_Cmd_Q_Ctr);
              }
              __set_FAULTMASK(1);
              NVIC_SystemReset();
            }
            break;
          case 0x40:
            if( databuffer[6]==0x41)
            {
              if(databuffer[7]==1)
              {
                if(originrec==0)
                {
                  //originrec=1;
                  btorigin = databuffer[7];
                  if(btorigin==1)
                  {
                  seekoncedone1=0;
                  breakclear1=0;
                  btorigin1=0;
                  origindone1=0;
                  readymotor1=0;
                  seekoncedone2=0;
                  breakclear2=0;
                  btorigin2=0;
                  origindone2=0;
                  readymotor2=0;
                  seekoncedone3=0;
                  breakclear3=0;
                  btorigin3=0;
                  origindone3=0;
                  readymotor3=0;
                  seekoncedone4=0;
                  breakclear4=0;
                  btorigin4=0;
                  origindone4=0;
                  readymotor4=0;
                  originstate=0;
                }
                }
              }
            }
            if(databuffer[8]==0x42)
              brake = databuffer[9];
            if( databuffer[10]==0x43)
            {
              moveflag2=1;
              l1angset=(int)(((((((l1angset&0)|databuffer[11])<<8)|databuffer[12])<<8)|databuffer[13])<<8)|databuffer[14];
              r1angset=(int)(((((((r1angset&0)|databuffer[15])<<8)|databuffer[16])<<8)|databuffer[17])<<8)|databuffer[18];
              l2angset=(int)(((((((l2angset&0)|databuffer[19])<<8)|databuffer[20])<<8)|databuffer[21])<<8)|databuffer[22];
              r2angset=(int)(((((((r2angset&0)|databuffer[23])<<8)|databuffer[24])<<8)|databuffer[25])<<8)|databuffer[26];
              printf("%d\n", l1angset);
              l1ang=(int)(l1angset*6000.0/360.0);
              r1ang=(int)(r1angset*6000.0/360.0);
              l2ang=(int)(l2angset*6000.0/360.0);
              r2ang=(int)(r2angset*6000.0/360.0);
              printf("%d\n", l1ang);
              posSet[4]=l1ang;
              posSet[5]=l1ang>>8;
              posSet[6]=l1ang>>16;
              posSet[7]=l1ang>>24;
              if((can_Cmd_Q_Ctr <= can_Cmd_Q_Max_Size - 1) && can_Cmd_Q_Ctr>=0)
              { 
                can_Cmd_Q[can_Cmd_Q_Push] = ROUND_MOTOR_PARA_SET;
                can_Cmd_Q_Push = (can_Cmd_Q_Push + 1) % can_Cmd_Q_Max_Size;
                can_Cmd_Q_Ctr = can_Cmd_Q_Ctr + 1;
                //printf("can_Cmd_Q_Ctr = %d\n", can_Cmd_Q_Ctr);
              }
              
              /*posSet[4]=r1ang;
              posSet[5]=r1ang>>8;
              posSet[6]=r1ang>>16;
              posSet[7]=r1ang>>24;
              CAN_SendTxMsg(0x601, posSet);
              HAL_Delay(1);
              posSet[4]=l2ang;
              posSet[5]=l2ang>>8;
              posSet[6]=l2ang>>16;
              posSet[7]=l2ang>>24;
              CAN_SendTxMsg(0x601, posSet);
              HAL_Delay(1);
              posSet[4]=r2ang;
              posSet[5]=r2ang>>8;
              posSet[6]=r2ang>>16;
              posSet[7]=r2ang>>24;
              CAN_SendTxMsg(0x601, posSet);
              HAL_Delay(1);*/
            }		
            if( databuffer[27]==0x44)
            {
              if(databuffer[28]==0x01)
              {
                if((can_Cmd_Q_Ctr <= can_Cmd_Q_Max_Size - 1) && can_Cmd_Q_Ctr>=0)
                { 
                  can_Cmd_Q[can_Cmd_Q_Push] = ROUND_MOTOR_STOP;
                  can_Cmd_Q_Push = (can_Cmd_Q_Push + 1) % can_Cmd_Q_Max_Size;
                  can_Cmd_Q_Ctr = can_Cmd_Q_Ctr + 1;
                  //printf("can_Cmd_Q_Ctr = %d\n", can_Cmd_Q_Ctr);
                }
                emestopall=1;
              } 
              else
              {
                emestopall=0;
              }
            }      		
            databuffer[5]=0x40;
            databuffer[6]=0x01;
            databuffer[3]=0x00;
            databuffer[4]=0x02;
            databuffer[7]=0;
            for(i=0;i<7;i++)
            databuffer[7]=crc_array[databuffer[7]^(unsigned char)(databuffer[i])];
            databuffer[8]=0xFF;
            TOLLEN=0x09;
            break;          
          case 0x50:
            if((can_Cmd_Q_Ctr <= can_Cmd_Q_Max_Size - 1) && can_Cmd_Q_Ctr>=0)
              { 
                can_Cmd_Q[can_Cmd_Q_Push] = ROUND_MOTOR_ALARM_CLR;
                can_Cmd_Q_Push = (can_Cmd_Q_Push + 1) % can_Cmd_Q_Max_Size;
                can_Cmd_Q_Ctr = can_Cmd_Q_Ctr + 1;
                //printf("can_Cmd_Q_Ctr = %d\n", can_Cmd_Q_Ctr);
              }
            databuffer[5]=0x50;
            databuffer[6]=0x01;
            databuffer[3]=0x00;
            databuffer[4]=0x02;
            databuffer[7]=0;
            for(i=0;i<7;i++)
              databuffer[7]=crc_array[databuffer[7]^(unsigned char)(databuffer[i])];
            databuffer[8]=0xFF;
            TOLLEN=0x09;
            break;
          }
        }
        else
        {
          TOLLEN=0x09;
          databuffer[6]=0x00;
          databuffer[3]=0x00;
          databuffer[4]=0x02;
          for(i=0;i<7;i++)
          databuffer[7]=crc_array[databuffer[7]^(unsigned char)(databuffer[i])];
          databuffer[8]=0xFF;
        }
      }
      /*else
      {
        TOLLEN=0x09;
        databuffer[6]=0x00;
        databuffer[3]=0x00;
        databuffer[4]=0x02;
        for(i=0;i<7;i++)
        databuffer[7]=crc_array[databuffer[7]^(unsigned char)(databuffer[i])];
        databuffer[8]=0xFF;
      }*/
    }
    else
    {
      TOLLEN=0x09;
      databuffer[6]=0x00;
      databuffer[3]=0x00;
      databuffer[4]=0x02;
      for(i=0;i<7;i++)
      databuffer[7]=crc_array[databuffer[7]^(unsigned char)(databuffer[i])];
      databuffer[8]=0xFF;
    }
  }
  else
  {
    TOLLEN=0x09;
    databuffer[6]=0x00;
    databuffer[3]=0x00;
    databuffer[4]=0x02;
    for(i=0;i<7;i++)
    databuffer[7]=crc_array[databuffer[7]^(unsigned char)(databuffer[i])];
    databuffer[8]=0xFF;
  }
  udp_send_t2(TOLLEN, port);
  pbuf_free(p);	
}

/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE*************************/
