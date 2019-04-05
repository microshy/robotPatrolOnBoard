/***********************************************************************

***********************************************************************/
#include "stm32f4xx_it.h"
#include "stm32f4xx.h"

//#ifndef _SPEED_
//#define _SPEED_

#define L 	0.535f
//#define L 	54
//#define R   0.15f
#define R   150
#define pi  3.1416f
#define ratio  20
#define limspeed 1800
//#define v100  1500
//#define w100  1500



float RSPEEDca(short int v,short int w);
float LSPEEDca(short int v,short int w);
//#endif
