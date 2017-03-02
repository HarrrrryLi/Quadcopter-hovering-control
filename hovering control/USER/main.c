/******************** (C) COPYRIGHT 2011 Ұ��Ƕ��ʽ���������� ********************
 * �ļ���  ��main.c
 * ����    ����c���е�printf()����ʵ�ֵ�����1(USART1)���������ǾͿ�����printf()��
 *           ������Ϣͨ�����ڴ�ӡ�������ϡ�         
 * ʵ��ƽ̨��Ұ��STM32������
 * ��汾  ��ST3.0.0
 *
 * ����    ��fire  QQ: 313303034 
 * ����    ��firestm32.blog.chinaunix.net
**********************************************************************************/
#include "stm32f10x.h"
#include "usart1.h"
#include "stm32f10x_it.h"


/*
 * ��������main
 * ����  ��������
 * ����  : ��
 * ���  ����
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
	/* ����ϵͳʱ��Ϊ 72M */      
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


/******************* (C) COPYRIGHT 2011 Ұ��Ƕ��ʽ���������� *****END OF FILE****/
