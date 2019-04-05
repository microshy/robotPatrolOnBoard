/***********************************************************************
�ļ����ƣ�LED.C
caulat the right speed and the left speed
***********************************************************************/
#include "main.h"
#include "speedcal/SPEED.h"


float RSPEEDca(short int v,short int w)
{
	int nr;
  nr=300*(2*v+w*L)/pi/R;
//	nr=300*2*v+(300*w/R)*(L/pi);
//	return nr;
	if(nr>limspeed)
	{
		nr=limspeed;
	}
	else if (nr<-limspeed)
	{
 	  nr=-limspeed;
	}
		return -nr;
}


float LSPEEDca(short int v,short int w)
{
	int nl;
	nl=300*(2*v-w*L)/pi/R;
	//nl=300*2*v-(300*w/R)*(L/pi);
	if(nl>limspeed)
	{
		nl=limspeed;
	}
	else if (nl<-limspeed)
	{
 	  nl=-limspeed;
	}
	return nl;
}


//float RSPEEDca(short int v,short int w)
//{
//	int nr;
//	//nr=300*(2*v+w*L)/pi/R;
//	nr=300*2*v+(300*w/R)*(L/pi);
////	return nr;
//		return -nr;
//}


//float LSPEEDca(short int v,short int w)
//{
//	int nl;
//	//nl=300*(2*v-w*L)/pi/R;
//	nl=300*2*v-(300*w/R)*(L/pi);
//	return nl;
//}
