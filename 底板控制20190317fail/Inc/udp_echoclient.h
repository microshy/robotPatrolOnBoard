#ifndef __UDP_ECHOCLIENT_H__
#define __UDP_ECHOCLIENT_H__
/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
/* 类型定义 ------------------------------------------------------------------*/
#define MOTORIDset1  0x601
#define MOTORIDset2  0x602
#define MOTORIDset3  0x603
#define MOTORIDset4  0x604


#define MOTORIDrec1  0x581
#define MOTORIDrec2  0x582
#define MOTORIDrec3  0x583
#define MOTORIDrec4  0x584
#define MOTORIDrec5  0x585
#define MOTORIDrec6  0x586
#define MOTORIDrec7  0x587
#define MOTORIDrec8  0x588

#define LocalID (unsigned char)0x01
#define LocalID2 (unsigned char)0x03
#define LocalID3 (unsigned char)0x02
#define Head (unsigned char)0xFA

#define MOVE_MOTOR_INIT 0x01
#define MOVE_MOTOR_STOP 0x02
#define MOVE_MOTOR_PARA_SET 0x03
#define MOVE_MOTOR_ALARM_CLR 0x04
#define ROUND_MOTOR_INIT 0x11
#define ROUND_MOTOR_STOP 0x12
#define ROUND_MOTOR_PARA_SET 0x13
#define ROUND_MOTOR_ALARM_CLR 0x14
/* 宏定义 --------------------------------------------------------------------*/
/* 扩展变量 ------------------------------------------------------------------*/
//control instruction
extern unsigned char enable[8];
extern unsigned char release[8];
extern unsigned char stop[8];
extern unsigned char resetEncoder[8];
extern unsigned char alarmclear[8];
extern unsigned char setSpeed[8];
extern unsigned char posSet[8];
extern unsigned char posModeAbs[8];
extern unsigned char posStart[8];
//check instruction
extern unsigned char enStateCheck[8];
extern unsigned char givenSpeedCheck[8];
extern unsigned char actSpeedCheck[8];
extern unsigned char actCurrentCheck[8];
extern unsigned char errorCheck[8];
extern unsigned char positionCheck[8];
extern unsigned char driverFlagCheck[8];

extern unsigned char emestopall;
extern unsigned char enstateall;
extern unsigned char selfspeedcontrol;

extern unsigned char breakclear1,breakclear2,breakclear3,breakclear4;
extern unsigned char btorigin1,btorigin2,btorigin3,btorigin4;
extern unsigned char readymotor1,readymotor2,readymotor3,readymotor4;
extern unsigned char originstate;
extern unsigned char originrec;
extern unsigned char seekoncedone1,seekoncedone2,seekoncedone3,seekoncedone4;

/* 函数声明 ------------------------------------------------------------------*/

void udp_echoclient_connect(void);
void udp_echoclient_send(void);
void udp_send_t(uint8_t length);
  
extern uint8_t   data[100];

#endif /* __UDP_ECHOCLIENT_H__ */


/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
