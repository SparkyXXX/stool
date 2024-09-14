/***********************************************
��˾����Ȥ�Ƽ�(��ݸ)���޹�˾
Ʒ�ƣ�WHEELTEC
������wheeltec.net
�Ա����̣�shop114407458.taobao.com 
����ͨ: https://minibalance.aliexpress.com/store/4455017
�汾��V1.0
�޸�ʱ�䣺2022-09-05

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update��2022-09-05

All rights reserved
***********************************************/
  
#include "beep.h"   

/**************************************************************************
Function: Buzzer initialization
Input   : none
Output  : none
�������ܣ���������ʼ��
��ڲ���: �� 
����  ֵ����
**************************************************************************/	 	
void BEEP_GPIO_Config(void)
{		

	
//	/*�ر�JTAG�ӿ�*/
//	JTAG_Set(JTAG_SWD_DISABLE);    
//	/*��SWD�ӿ� �������������SWD�ӿڵ���*/
//	JTAG_Set(SWD_ENABLE);           
	/* �رշ�����*/
	BEEP_OFF;
}



/**************************************************************************
Function: Buzzer_Alarm
Input   : Indicates the count of frequencies
Output  : none
�������ܣ�����������
��ڲ���: ָʾƵ�ʵļ��� 
����  ֵ����
**************************************************************************/	 	
////���жϺ�������
void Buzzer_Alarm(uint16_t count)
{
	static int count_time;
	if(0 == count)
	{
		BEEP_OFF;
	}
	else if(++count_time >= count)
	{
		BEEP_TOGGLE;
		count_time = 0;	
	}
}




/*********************************************END OF FILE**********************/
