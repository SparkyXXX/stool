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
#ifndef __MOTOR_H
#define __MOTOR_H
#include <sys.h>	 

//�����ǵ�������PWM          //PWMX_IN1 ΪPWM����ʱ��PWMX_IN2 û��PWM����ʱ��������ת��˥��
                               //PWMX_IN1 Ϊ1����ʱ��PWMX_IN2 ��PWM����ʱ��������ת��˥��
#define PWMA_IN1 TIM3->CCR1   
#define PWMA_IN2 TIM3->CCR2   
#define PWMB_IN1 TIM3->CCR3
#define PWMB_IN2 TIM3->CCR4



#endif
