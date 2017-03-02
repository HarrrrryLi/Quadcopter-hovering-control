/******************** (C) COPYRIGHT 2011 野火嵌入式开发工作室 ********************
 * 文件名  ：main.c
 * 描述    ：将c库中的printf()函数实现到串口1(USART1)。这样我们就可以用printf()将
 *           调试信息通过串口打印到电脑上。         
 * 实验平台：野火STM32开发板
 * 库版本  ：ST3.0.0
 *
 * 作者    ：fire  QQ: 313303034 
 * 博客    ：firestm32.blog.chinaunix.net
**********************************************************************************/
#include "stm32f10x.h"
#include "usart1.h"
#include "stm32f10x_it.h"


/*
 * 函数名：main
 * 描述  ：主函数
 * 输入  : 无
 * 输出  ：无
 */
unsigned char flow_comp_x1[]={"      "};
unsigned char flow_comp_y1[]={"      "};
unsigned char flow_height1[]={"      "};
float comp_x=0.00;
float comp_y=0.00;
float height=0.00;
int flowx=0;
int flowy=0;
float height_expect=0.00;
float height_adjust=0.00; 
void TIM3_Init(void);
void Nvic_Init(void);
void param_load(void);
int main(void)
{  
	/* 配置系统时钟为 72M */      
  SystemInit();

  /* USART1 config 115200 8-N-1 */
	USART1_Config();
//	I2C_INIT();
	param_load();
	TIM3_Init();
	Nvic_Init();
////	Init_HMC5883L();
  while (1);

}


/******************* (C) COPYRIGHT 2011 野火嵌入式开发工作室 *****END OF FILE****/
