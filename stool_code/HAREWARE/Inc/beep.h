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
#ifndef __BEEP_H
#define	__BEEP_H

#include "sys.h"



/* ������������ӵ�GPIO�˿�, �û�ֻ��Ҫ�޸�����Ĵ��뼴�ɸı���Ƶķ��������� */
#define BEEP_GPIO_PORT    	GPIOA			              	/* GPIO�˿� */
#define BEEP_GPIO_PIN				GPIO_PIN_15			        	/* ���ӵ���������GPIO */


/* ���κ꣬��������������һ��ʹ�� */
#define BEEP(a)	if (a)	\
					GPIO_SetBits(BEEP_GPIO_PORT,BEEP_GPIO_PIN);\
					else		\
					GPIO_ResetBits(BEEP_GPIO_PORT,BEEP_GPIO_PIN)

					
/* �������IO�ĺ� */
#define BEEP_TOGGLE		    digitalToggle(BEEP_GPIO_PORT,BEEP_GPIO_PIN)
#define BEEP_ON		   		digitalHi(BEEP_GPIO_PORT,BEEP_GPIO_PIN)
#define BEEP_OFF			digitalLo(BEEP_GPIO_PORT,BEEP_GPIO_PIN)


					
void BEEP_GPIO_Config(void);
void Buzzer_Alarm(uint16_t count);

					
#endif /* __BEEP_H */
