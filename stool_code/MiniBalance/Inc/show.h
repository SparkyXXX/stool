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
#ifndef __SHOW_H
#define __SHOW_H
#include "sys.h"
extern float Velocity_Left,Velocity_Right;//�����ٶȡ������ٶ�
extern float Displacement;                //λ��
void oled_show(void);
void APP_Show(void);
void DataScope(void);
void OLED_Show_CCD(void);
void OLED_DrawPoint_Shu(uint8_t x,uint8_t y,uint8_t t);
#endif
