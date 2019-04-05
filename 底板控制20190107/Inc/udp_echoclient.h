#ifndef __UDP_ECHOCLIENT_H__
#define __UDP_ECHOCLIENT_H__
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
/* ���Ͷ��� ------------------------------------------------------------------*/
#define MOTORIDset1  0x601
#define MOTORIDset2  0x602
#define MOTORIDset3  0x603
#define MOTORIDset4  0x604


#define MOTORIDrec1  0x581
#define MOTORIDrec2  0x582
#define MOTORIDrec3  0x583
#define MOTORIDrec4  0x584
/* �궨�� --------------------------------------------------------------------*/
/* ��չ���� ------------------------------------------------------------------*/
/* �������� ------------------------------------------------------------------*/

void udp_echoclient_connect(void);
void udp_echoclient_send(void);
void udp_send_t(uint8_t length);
  
extern uint8_t   data[100];

#endif /* __UDP_ECHOCLIENT_H__ */


/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
