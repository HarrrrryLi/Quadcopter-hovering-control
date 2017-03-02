/**
  ******************************************************************************
  * @file    Project/Template/stm32f10x_it.c 
  * @author  MCD Application Team
  * @version V3.0.0
  * @date    04/06/2009
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @copy
  *
  * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
  * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
  * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
  * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
  * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
  * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
  *
  * <h2><center>&copy; COPYRIGHT 2009 STMicroelectronics</center></h2>
  */ 

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_it.h"
#include "usart1.h"
#include "math.h"
/*变量定义*/
extern int16_t X_HMC,Y_HMC,Z_HMC;
extern float xflow,yflow,height;
extern u8 quality;
extern float comp_x;
extern float comp_y;
extern float height;
void FLOW_MAVLINK(unsigned char data);
extern float ByteToFloat(unsigned char* byteArry);
unsigned char *Int16_tToScreen(int16_t x);
extern int flowx;
extern int flowy;
u8  sentDateFlag=0;
unsigned  int jingdu_send=0,weidu_send=0;
extern uint8_t RxBuffer2[]; 
extern __IO uint8_t RxCounter2;

//unsigned char *FloatToScreen(float x);
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
uint64_t  time_old=0;
typedef struct
{
  float origin;
	float suppose;
	float increment;
	float old;             
} data;

u8 FLOW_STATE[4];
u8 flow_buf[30];
FLOW flow;
extern float jingdu,weidu;
void GPS(void);
float yaw=0;
int8_t yaw1=0;
int state=0;
extern float x_move,y_move;
extern unsigned char flow_x1[]={"      "};
extern unsigned char flow_y1[]={"      "};
extern unsigned char flow_comp_x1[];
extern unsigned char flow_comp_y1[];
extern unsigned char flow_height1[];
extern unsigned char quality1[]={"      "};
extern unsigned char id1[]={"      "};
extern float x_sudu,y_sudu;
    u8 com_data;
	static u8 s_flow=0,data_cnt=0;
  //  u8 cnt_offset=0;        
    u8 get_one_fame=0;
    unsigned char floattobyte[4];   //			 char->unsigned char
	u8 FLOW_STATE[4];
    u8 flow_buf[30];
float flow_filter_x[5]={0,0,0,0,0};
float flow_filter_y[5]={0,0,0,0,0};
float flow_filter_x_sum=0;
float flow_filter_y_sum=0;
int sum_cnt=0;
int filter_cnt=0;
int mode=0;


extern data high;
	/******************************************************************************/
/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M3 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval : None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval : None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval : None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval : None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval : None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval : None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval : None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval : None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval : None
  */
void SysTick_Handler(void)
{
}

 float ByteToFloat(unsigned char* byteArry)
{
  return *((float*)byteArry);
}

unsigned char *Int16_tToScreen(int16_t x)
{
  unsigned char a,b=0;
  static unsigned char *tmp;
  a=(unsigned char)x/100;
  b=(unsigned char)x%100;
  *tmp=a/1000%10+'0';
  *(tmp+1)=a/100%10+'0';
  *(tmp+2)=a/10%10+'0';
  *(tmp+3)=a%10+'0';

  *(tmp+4)=b/100%10+'0';
  *(tmp+5)=b/10%10+'0';
  *(tmp+6)=b%10+'0';
  return tmp;
}
extern void PID(void);
extern void navigation(void);
extern unsigned short int youmen,fuyang,henggun,pianhang;  	
extern uint8_t GPS_VA;

void TIM3_IRQHandler(void)		    //2.5ms中断一次
{	
    u8 send_data;

	if(TIM3->SR & TIM_IT_Update)	
	{    
   TIM3->SR = ~TIM_FLAG_Update;//清除中断标志
	 PID();
	 USART_SendData(USART1,'a');
	 while (!(USART1->SR & USART_FLAG_TXE));
	 send_data=youmen>>8;
	 USART_SendData(USART1,send_data);
	 while (!(USART1->SR & USART_FLAG_TXE));
	 send_data=youmen;
	 USART_SendData(USART1,send_data);
	 while (!(USART1->SR & USART_FLAG_TXE));

	
   USART_SendData(USART1,'b');
	 while (!(USART1->SR & USART_FLAG_TXE));
	 send_data=fuyang>>8;
	 USART_SendData(USART1,send_data);
	 while (!(USART1->SR & USART_FLAG_TXE));
	 send_data=fuyang;
	 USART_SendData(USART1,send_data);
	 while (!(USART1->SR & USART_FLAG_TXE));


	 USART_SendData(USART1,'c');
	 while (!(USART1->SR & USART_FLAG_TXE));
	 send_data=henggun>>8;
	 USART_SendData(USART1,send_data);
	 while (!(USART1->SR & USART_FLAG_TXE));
	 send_data=henggun;
	 USART_SendData(USART1,send_data);
	 while (!(USART1->SR & USART_FLAG_TXE));
	 
	 
//	 USART_SendData(USART1,'d');
//	 while (!(USART1->SR & USART_FLAG_TXE));
//	 send_data=pianhang>>8;
//	 USART_SendData(USART1,send_data);
//	 while (!(USART1->SR & USART_FLAG_TXE));
//	 send_data=pianhang;
//	 USART_SendData(USART1,send_data);
//	 while (!(USART1->SR & USART_FLAG_TXE));
	

	 USART_SendData(USART1,'e');
	 while (!(USART1->SR & USART_FLAG_TXE));
	 send_data=((int16_t)(flow.hight*100))>>8;
	 USART_SendData(USART1,send_data);
	 while (!(USART1->SR & USART_FLAG_TXE));
	 send_data=((int16_t)(flow.hight*100));
	 USART_SendData(USART1,send_data);
	 while (!(USART1->SR & USART_FLAG_TXE));
	 
	 USART_SendData(USART1,'f');
	 while (!(USART1->SR & USART_FLAG_TXE));
	 send_data=((int16_t)(x_move*100))>>8;
	 USART_SendData(USART1,send_data);
	 while (!(USART1->SR & USART_FLAG_TXE));
	 send_data=((int16_t)(x_move*100));
	 USART_SendData(USART1,send_data);
	 while (!(USART1->SR & USART_FLAG_TXE));

    }
	
}
float x_move=0,y_move=0,x_speed=0,y_speed=0;
void USART2_IRQHandler(void)
{	
       if(USART2->SR&(1<<5))
	 {

	com_data=USART2->DR;
	 
  switch(s_flow)
         {
    case 0: if(com_data==0xFE)
                        s_flow=1;
                        break;
                case 1: if(com_data==0x1A)
                       { s_flow=2;}
                        else
                        s_flow=0;
                         break;
                case 2:
                        if(data_cnt<4)
                        {s_flow=2; FLOW_STATE[data_cnt++]=com_data;}
                       else
                        {data_cnt=0;s_flow=3;flow_buf[data_cnt++]=com_data;}
                 break;
                case 3:
                 if(FLOW_STATE[3]==0x64)
				 {
                        if(data_cnt<26)
                        {s_flow=3; flow_buf[data_cnt++]=com_data;}
                  else
                        {data_cnt=0;s_flow=4;}
                }
                else
                        {data_cnt=0;s_flow=0;}
                   break;
                case 4:get_one_fame=1;s_flow=0;data_cnt=0;break;
                default:s_flow=0;data_cnt=0;break;
         }
   																
	      if(get_one_fame)
         {
                 floattobyte[0]=flow_buf[8];
                 floattobyte[1]=flow_buf[9];
                 floattobyte[2]=flow_buf[10];
                 floattobyte[3]=flow_buf[11];
                flow.flow_comp_x =ByteToFloat(floattobyte);
                 floattobyte[0]=flow_buf[12];
                 floattobyte[1]=flow_buf[13];
                 floattobyte[2]=flow_buf[14];
                 floattobyte[3]=flow_buf[15];
                flow.flow_comp_y =ByteToFloat(floattobyte);
                 floattobyte[0]=flow_buf[16];
                 floattobyte[1]=flow_buf[17];
                 floattobyte[2]=flow_buf[18];
                 floattobyte[3]=flow_buf[19];
           flow.hight=ByteToFloat(floattobyte);//ground_distance        float        Ground distance in m. Positive value: distance known. Negative value: Unknown distance     
       					 if(flow.hight<0.3)
			  flow.hight=0.3;
		   flow.flow_x=(int16_t)((flow_buf[20])|(flow_buf[21]<<8));
       flow.flow_y=(int16_t)((flow_buf[22])|(flow_buf[23]<<8));
//			 if((flow.flow_comp_x<0.2)&&(flow.flow_comp_x>-0.2))
//							 flow.flow_comp_x=0;
//			 if((flow.flow_comp_y<0.2)&&(flow.flow_comp_y>-0.2))
//							 flow.flow_comp_y=0;
           flow.id=flow_buf[24];
           flow.quality=flow_buf[25]; //Optical flow quality / confidence. 0: bad, 255: maximum quality
		   flowx=flow.flow_x;
		   flowy=flow.flow_y;
			flow_filter_x[filter_cnt]=flow.flow_comp_x;
			flow_filter_y[filter_cnt]=flow.flow_comp_y;	
      for(sum_cnt=0,flow_filter_x_sum=0,flow_filter_y_sum=0;sum_cnt<5;sum_cnt++)
        {
				  flow_filter_x_sum+=flow_filter_x[sum_cnt];
					flow_filter_y_sum+=flow_filter_y[sum_cnt];
				}						
			x_speed=flow_filter_x_sum/5;
		  y_speed=flow_filter_y_sum/5;
			if((x_speed<0.01)&&(x_speed>-0.01))
				x_speed=0;
			if((y_speed<0.01)&&(y_speed>-0.01))
				y_speed=0;
				x_move+=x_speed;
				y_move+=y_speed;
			if(x_move>10)
				x_move=10;
		  else if(x_move<-10)
				x_move=-10;
			if(y_move>10)
				y_move=10;
			else if(y_move<-10)
				y_move=-10;
	    if(filter_cnt>=5)
				filter_cnt=0;
			else 
				filter_cnt++;
			
	     /*转换数据类型*/
		 get_one_fame=0;
		  }	  
	}
}


void USART1_IRQHandler(void)
{	
 if(USART1->SR & (1<<5))
   {	  	 
     u8 com_data = USART1->DR;
		 if(com_data=='1')
			 mode=1;
	 }
	  if(USART_GetITStatus(USART1, USART_IT_TXE) != RESET)                   
  { 
     USART_ITConfig(USART1, USART_IT_TXE, DISABLE);	      
  }
}
/******************************************************************************/
/*                 STM32F10x Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f10x_xx.s).                                            */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval : None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
