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
#ifndef __KEY_H
#define __KEY_H	 
#include "sys.h"
#define KEY PAin(0)
#define KEY_ON	1
#define KEY_OFF	0
//�û���������ֵ״̬
#define No_Action 					0
#define Click 						1
#define Long_Press 					2
#define Double_Click				3
#define KEY2_STATE  		 PCin(13)

//����״̬ö��
enum {
	key_stateless,
	single_click,
	double_click,
	long_click
};

uint8_t KEY_Scan(uint16_t Frequency,uint16_t filter_times);

uint8_t User_Key_Scan(void);
void Mode_Choose(void);
#endif  
