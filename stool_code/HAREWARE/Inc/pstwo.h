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

#ifndef __PSTWO_H
#define __PSTWO_H

#include "sys.h"


//ָʾң�ؿ��ƵĿ���
#define RC_ON								1	
#define RC_OFF								0
#define Default_Velocity					300			//Ĭ��ң���ٶ�
#define Default_Turn_Bias					800			//Ĭ��ң���ٶ�

//ǰ���Ӽ��ٷ���ֵ��ÿ��ң�ؼӼ��Ĳ���ֵ
#define X_Step								100
//ת��Ӽ��ٷ���ֵ
#define Z_Step								300

//ң�ؿ���ǰ���ٶ����ֵ
#define MAX_RC_Velocity						800
//ң�ؿ���ת���ٶ����ֵ
#define	MAX_RC_Turn_Bias					2000
//ң�ؿ���ǰ���ٶ���Сֵ
#define MINI_RC_Velocity					100
//ң�ؿ���ת���ٶ���Сֵ
#define	MINI_RC_Turn_Velocity			800


/* ����PS2���ӵ�GPIO�˿�, �û�ֻ��Ҫ�޸�����Ĵ��뼴�ɸı���Ƶ�PS2���� */
#define PS2_DI_GPIO_PORT    	GPIOB			              /* GPIO�˿� */
#define PS2_DI_GPIO_CLK 	    RCC_APB2Periph_GPIOB			/* GPIO�˿�ʱ�� */
#define PS2_DI_GPIO_PIN			GPIO_Pin_8			       		 /* ���ӵ�GPIO */

#define PS2_DO_GPIO_PORT    	GPIOC			              /* GPIO�˿� */
#define PS2_DO_GPIO_CLK 	    RCC_APB2Periph_GPIOC			/* GPIO�˿�ʱ�� */
#define PS2_DO_GPIO_PIN			GPIO_Pin_9			       		 /* ���ӵ�GPIO */

#define PS2_CS_GPIO_PORT    	GPIOC			              /* GPIO�˿� */
#define PS2_CS_GPIO_CLK 	    RCC_APB2Periph_GPIOC			/* GPIO�˿�ʱ�� */
#define PS2_CS_GPIO_PIN			GPIO_Pin_4		       	  	  /* ���ӵ�GPIO */

#define PS2_CLK_GPIO_PORT    	GPIOC			              /* GPIO�˿� */
#define PS2_CLK_GPIO_CLK 	    RCC_APB2Periph_GPIOC			/* GPIO�˿�ʱ�� */
#define PS2_CLK_GPIO_PIN		GPIO_Pin_8			       		 /* ���ӵ�GPIO */


#define DI   PBin(8)          	//  ����

#define DO_H PCout(9)=1        	//����λ��
#define DO_L PCout(9)=0        	//����λ��

#define CS_H PCout(4)=1       	//CS����
#define CS_L PCout(4)=0       	//CS����

#define CLK_H PCout(8)=1      	//ʱ������
#define CLK_L PCout(8)=0      	//ʱ������




//�������
#define PSB_SELECT      1
#define PSB_L3          2
#define PSB_R3          3
#define PSB_START       4
#define PSB_PAD_UP      5
#define PSB_PAD_RIGHT   6
#define PSB_PAD_DOWN    7
#define PSB_PAD_LEFT    8
#define PSB_L2          9
#define PSB_R2          10
#define PSB_L1          11
#define PSB_R1          12
#define PSB_GREEN       13
#define PSB_RED         14
#define PSB_BLUE        15
#define PSB_PINK        16

#define PSB_TRIANGLE    13
#define PSB_CIRCLE      14
#define PSB_CROSS       15
#define PSB_SQUARE      16

//#define WHAMMY_BAR		8

//These are stick values
#define PSS_RX 5                //��ҡ��X������
#define PSS_RY 6
#define PSS_LX 7
#define PSS_LY 8

extern uint8_t Data[9];
extern uint16_t MASK[16];
extern uint16_t Handkey;
extern int PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY; 

uint8_t PS2_RedLight(void);  	 		//�ж��Ƿ�Ϊ���ģʽ
void PS2_ReadData(void); 			//���ֱ�����
void PS2_Cmd(uint8_t CMD);		  		//���ֱ���������
uint8_t PS2_DataKey(void);		  		//����ֵ��ȡ
uint8_t PS2_AnologData(uint8_t button); 		//�õ�һ��ҡ�˵�ģ����
void PS2_ClearData(void);	  		//������ݻ�����
void PS2_Vibration(uint8_t motor1, uint8_t motor2);//������motor1  0xFF���������أ�motor2  0x40~0xFF

void PS2_EnterConfing(void);	 	//��������
void PS2_TurnOnAnalogMode(void); 	//����ģ����
void PS2_VibrationMode(void);    	//������
void PS2_ExitConfing(void);	     	//�������
void PS2_SetInit(void);		     	//���ó�ʼ��
void PS2_Read(void);
void PS2_Control(void);

#endif

