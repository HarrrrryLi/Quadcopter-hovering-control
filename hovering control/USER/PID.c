#include <stdio.h>
#include <stdint.h>
#include "usart1.h"
#include "math.h"
typedef struct
{
        uint64_t  time_sec;
        u8   id;
        int16_t flow_x;
        int16_t flow_y;
        float flow_comp_x;//Flow in m in x-sensor direction, angular-speed compensated
        float flow_comp_y;
        u8 quality; //Optical flow quality / confidence. 0: bad, 255: maximum quality
        float hight;//ground_distance        float        Ground distance in m. Positive value: distance known. Negative value: Unknown distance       
   
            
}FLOW;
int jishu=0,flow_cnt=0;
extern float yaw;
extern float jingdu,weidu;
int jiangluo=0;
 float gaodu1=0;
float gaodu2=0,x_sudu1=0,x_sudu2=0,y_sudu1=0,y_sudu2=0,x_sudu=0,y_sudu=0,x_sum=0,y_sum=0;
extern FLOW flow;
int high_stable=0;
typedef struct
{
    float origin;
	float suppose;
	float increment;
	float old;             
}data;

data high,x,y,xv,yv;

typedef struct
{
  float h;
	float x;
	float y; 
	float xv;
	float yv;
}pid;
pid Kp,Ki,Kd;
int i_flag=1;
extern int mode;
int land=0;
unsigned short int youmen,henggun,fuyang,pianhang=1520;
float fuyang_shell=0,henggun_shell=0;
int stable_cnt=0,land_cnt=0;
void param_load(void)
{
  high.suppose=1.1;
  Kp.h=85;		 //30
  Ki.h=2;		 //20
  Kd.h=423;		 //360

  x.suppose=0;
  Kp.x=0.03;
  Ki.x=0.01;
  Kd.x=0.1;

  y.suppose=0;
  Kp.y=0.03;
  Ki.y=0.01;
	Kd.y=0.1;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                  5;
	
	Kp.xv=50;
  Ki.xv=3.5;
  Kd.xv=560;
	
	Kp.yv=50;
  Ki.yv=3.5;
  Kd.yv=560;
	
}
extern float x_move,y_move,x_speed,y_speed;
void PID(void)
{  
  jishu++;

   if (jishu==60)
    {
   	jishu=0;
		gaodu2=gaodu1;//shangyici
		gaodu1=flow.hight;		 //benci 
		x.old=x.origin;
		xv.old=x_speed;
		y.old=y.origin;
		yv.old=y_speed;
     } 

		
	//高度PID 
   high.origin=high.suppose-flow.hight;		//计算当前高度误差       P
	

	
	 
		 if((high.origin>=-0.3)&&(high.origin<=0.))
	 {
		  if(stable_cnt<400)
	     stable_cnt++;
	 }
	 else
		 stable_cnt=0;
	 
	 
	 	 if((stable_cnt>=400)&&(land==0))
	 {
	  land_cnt++;
	 }
	 

	 if(land_cnt>=4000)
		 land=1;
	 if((land==1)&&(stable_cnt>=400))
		 high.suppose-=0.1;
	 if((high.suppose<=0.1)&&(stable_cnt>=400))
	 {
	   land=2;
		 youmen=0;
//		  USART_SendData(USART1,'2');
//	 while (!(USART1->SR & USART_FLAG_TXE));
	 }
//	if(high.origin<=0.2)
//		i_flag=1;
	if(i_flag==1)
	{
	 high.increment+=high.origin;			//计算高度误差的累积值	 I
   if(high.increment>=1)			    //积分限幅
      high.increment=1;  
   else if(high.increment<=-1)
      high.increment=-1;
	 Ki.h=2;
	}
	else 
	{
		high.increment=0;
	  Ki.h=1;
	}
	
//	if((high.origin>=-0.3)&&(high.origin<=0.3))
//	{
//		Kd.h=80;
//		Kp.h=40;
//	}
//	else
//	{
//		Kd.h=550;
//	  Kp.h=85;
//	}
    youmen=1486+Kp.h*high.origin+Ki.h*high.increment+Kd.h*(gaodu2-gaodu1);	//Kd.h*(high.origin-high.old)	1480  Ki.h*high.increment+	
	 if(youmen>1600)
	     youmen=1600;
	  else if(youmen<1200)
	     youmen=1200;  
   if(land==2)
      youmen=0;		 
	
  //俯仰外环
   x.origin=x.suppose-x_move;		//计算当前高度误差       P
//	 if(stable_cnt<20)
//			x.origin=0;
   x.increment+=x.origin;			//计算高度误差的累积值	 I
   if(x.increment>=10)			    //积分限幅
      x.increment=10;  
   else if(x.increment<=-10)
      x.increment=-10;
	 
	 	 
	 fuyang_shell=(Kp.x*x.origin+Ki.x*x.increment+Kd.x*(x.old-x.origin));
	 
	 //俯仰内环
	 xv.origin=fuyang_shell-x_speed;		//计算当前高度误差       P
   xv.increment+=-x_speed;			//计算高度误差的累积值	 I
   if(xv.increment>=10)			    //积分限幅
      xv.increment=10;  
   else if(xv.increment<=-10)
      xv.increment=-10;
	 

	 fuyang=1555-(Kp.xv*xv.origin+Ki.xv*xv.increment+Kd.xv*(xv.old-x_speed));	
	 
    if(fuyang>1600)
	     fuyang=1600;
	  else if(fuyang<1200)
	     fuyang=1200; 
	//横滚外环	
	y.origin=y.suppose-y_move;		//计算当前高度误差       P
//		if(stable_cnt<20)
//			y.origin=0;
   y.increment+=y.origin;			//计算高度误差的累积值	 I
   if(y.increment>=10)			    //积分限幅
      y.increment=10;  
   else if(y.increment<=-10)
      y.increment=-10;
	 henggun_shell=(Kp.y*y.origin+Ki.y*y.increment+Kd.y*(y.old-y.origin));
	 
	 //横滚内环
	 yv.origin=henggun_shell-y_speed;		//计算当前高度误差       P
   yv.increment+=-y_speed;			//计算高度误差的累积值	 I
   if(yv.increment>=10)			    //积分限幅
      yv.increment=10;  
   else if(yv.increment<=-10)
      yv.increment=-10;
	 henggun=1465+(Kp.yv*yv.origin+Ki.yv*yv.increment+Kd.yv*(yv.old-y_speed));	
	 
    if(henggun>1600)
	     henggun=1600;
	  else if(henggun<1200)
	     henggun=1200; 	
}

