#ifndef __MAIN_H
#define __MAIN_H
/* ����ͷ�ļ� ----------------------------------------------------------------*/
/* ���Ͷ��� --------------------------------------------------------------*/
/* �궨�� --------------------------------------------------------------------*/

#define USE_DHCP       /* enable DHCP, if disabled static address is used */

#define DEST_IP_ADDR0               192
#define DEST_IP_ADDR1               168
#define DEST_IP_ADDR2                 3
#define DEST_IP_ADDR3               100

/* USER CODE BEGIN Includes */
#define IP_ADDR0   (uint8_t) 192
#define IP_ADDR1   (uint8_t) 168
#define IP_ADDR2   (uint8_t) 3
#define IP_ADDR3   (uint8_t) 101
   
/*NETMASK*/
#define NETMASK_ADDR0   (uint8_t) 255
#define NETMASK_ADDR1   (uint8_t) 255
#define NETMASK_ADDR2   (uint8_t) 255
#define NETMASK_ADDR3   (uint8_t) 0

/*Gateway Address*/
#define GW_ADDR0   (uint8_t) 192
#define GW_ADDR1   (uint8_t) 168
#define GW_ADDR2   (uint8_t) 3
#define GW_ADDR3   (uint8_t) 1

/* ��չ���� ------------------------------------------------------------------*/
/* �������� ------------------------------------------------------------------*/



#endif /* __MAIN_H */

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
