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
#include "key.h"
/*************************************************************************
Function:Mode Choose
Iput    :None
Output  :Noce
�������ܣ�С��ģʽѡ��
��ڲ�������
����  ֵ����
*************************************************************************/
void Mode_Choose(void)
{
	switch(User_Key_Scan())
		{
			//�������������л���
			//1.��ͨң��ģʽ
			//2.�״�Ѳ��ģʽ
			//3.�״����ģʽ
			//4.���Ѳ��ģʽ
			//5.CCDѲ��ģʽ
			//��������������λ��
			case Click:
				Mode++;
				 if(Mode == 3)							//3��ģʽѭ���л�
				{
					Mode = Normal_Mode;
				}
				break;
			case Long_Press:
				Flag_Show = !Flag_Show;								//���� ����/�˳� ��λ��ģʽ
				break;
			default:break;
		}
}

/*************************************************************************
Function:User_Key_Scan
Input:None
Output:Key_status
�������ܣ��û��������
��ڲ�������
����ֵ  ������״̬
**************************************************************************/
//����5ms�ж��е���
uint8_t User_Key_Scan(void)
{
	static uint16_t count_time = 0;					//���㰴�µ�ʱ�䣬ÿ5ms��1
	static uint8_t key_step = 0;						//��¼��ʱ�Ĳ���
	switch(key_step)
	{
		case 0:
			if(PAin(0) == KEY_ON )
				key_step++;						//��⵽�а������£�������һ��
			break;
		case 1:
			if((++count_time) == 5)				//��ʱ����
			{
				if(PAin(0) == KEY_ON )//����ȷʵ������
					key_step++,count_time = 0;	//������һ��
				else
					count_time = 0,key_step = 0;//����λ
			}
			break;
		case 2:
			if(PAin(0) == KEY_ON )
				count_time++;					//���㰴�µ�ʱ��
			else 								//��ʱ���ɿ���
				key_step++;						//������һ��
			break;
		case 3:									//��ʱ�����µ�ʱ�䣬���ж��ǳ������Ƕ̰�
			if(count_time > 400)				//��5ms�ж��е��ã��ʰ���ʱ��������400*5 = 2000ms�����ֵ��
			{							
				key_step = 0;					//��־λ��λ
				count_time = 0;
				return Long_Press;				//���� ���� ��״̬ 
 			}
			else if(count_time > 5)				//��ʱ�ǵ�����һ��
			{
				key_step++;						//��ʱ������һ�����ж��Ƿ���˫��
				count_time = 0;					//���µ�ʱ������
			}
			else
			{
				key_step = 0;
				count_time = 0;	
			}
			break;
		case 4:									//�ж��Ƿ���˫���򵥻�
			if(++count_time == 38)				//5*38 = 190ms���жϰ����Ƿ���
			{
				if(PAin(0) == KEY_ON )	//����ȷʵ������
				{																	//����˫�����ܰ�̫�죬��ʶ��ɵ���
					key_step++;														//������һ������Ҫ�����ֲ����ͷ�״̬
					count_time = 0;
				}
				else																//190ms���ް������£���ʱ�ǵ�����״̬
				{
					key_step = 0;				//��־λ��λ
					count_time = 0;					
					return Click;				//���ص�����״̬
				}
			}
			break;
		case 5:
			if(PAin(0) == KEY_ON )//�������ڰ���
			{
				count_time++;
			}
			else								//�����Ѿ�����
			{
//				if(count_time>400)				//����ڶ��εĵ���Ҳ�����ж�ʱ��ģ�Ĭ�ϲ��ж�ʱ�䣬ȫ��������˫��
//				{
//				}
				count_time = 0;
				key_step = 0;
				return Double_Click;
			}
			break;
		default:break;
	}
	return No_Action;							//�޶���
}


/**************************************************************************
�������ܣ�����ɨ�躯��
��ڲ�����ִ�иú���������Ƶ�ʡ��ӳ��˲���ʱ��
����  ֵ��long_click��double_click��single_click��key_stateless��������˫������������״̬��
��    �ߣ�WHEELTEC
**************************************************************************/
uint8_t KEY_Scan(uint16_t Frequency,uint16_t filter_times)
{
    static uint16_t time_core;//��ʱ����
    static uint16_t long_press_time;//����ʶ��
    static uint8_t press_flag=0;//�������±��
    static uint8_t check_once=0;//�Ƿ��Ѿ�ʶ��1�α��
    static uint16_t delay_mini_1;
    static uint16_t delay_mini_2;
	
    float Count_time = (((float)(1.0f/(float)Frequency))*1000.0f);//�����1��Ҫ���ٸ�����

    if(check_once)//�����ʶ����������б���
    {
        press_flag=0;//�����1��ʶ�𣬱������
        time_core=0;//�����1��ʶ��ʱ������
        long_press_time=0;//�����1��ʶ��ʱ������
        delay_mini_1=0;
        delay_mini_2=0;
    }
    if(check_once&&KEY==KEY_OFF) check_once=0; //���ɨ��󰴼�������������һ��ɨ��

    if(KEY==KEY_ON&&check_once==0)//����ɨ��
    {
        press_flag=1;//��Ǳ�����1��
		
        if(++delay_mini_1>filter_times)
        {
            delay_mini_1=0;
            long_press_time++;		
        }
    }

    if(long_press_time>(uint16_t)(500.0f/Count_time))// ����1��
    {	
        check_once=1;//����ѱ�ʶ��
        return long_click; //����
    }

    //����������1���ֵ���󣬿����ں���ʱ
    if(press_flag&&KEY==KEY_OFF)
    {
        if(++delay_mini_2>filter_times)
        {
            delay_mini_2=0;
            time_core++; 
        }
    }		
	
    if(press_flag&&(time_core>(uint16_t)(50.0f/Count_time)&&time_core<(uint16_t)(300.0f/Count_time)))//50~700ms�ڱ��ٴΰ���
    {
        if(KEY==KEY_ON) //����ٴΰ���
        {
            check_once=1;//����ѱ�ʶ��
            return double_click; //���Ϊ˫��
        }
    }
    else if(press_flag&&time_core>(uint16_t)(300.0f/Count_time))
    {
        check_once=1;//����ѱ�ʶ��
        return single_click; //800ms��û�����£����ǵ���
    }

    return key_stateless;
}
