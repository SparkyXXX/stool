/***********************************************
公司：轮趣科技(东莞)有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：V1.0
修改时间：2022-09-05

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update：2022-09-05

All rights reserved
***********************************************/
#include "key.h"
/*************************************************************************
Function:Mode Choose
Iput    :None
Output  :Noce
函数功能：小车模式选择
入口参数：无
返回  值：无
*************************************************************************/
void Mode_Choose(void)
{
	switch(User_Key_Scan())
		{
			//单击按键可以切换到
			//1.普通遥控模式
			//2.雷达巡航模式
			//3.雷达跟随模式
			//4.电磁巡线模式
			//5.CCD巡线模式
			//长按按键进入上位机
			case Click:
				Mode++;
				 if(Mode == 3)							//3种模式循环切换
				{
					Mode = Normal_Mode;
				}
				break;
			case Long_Press:
				Flag_Show = !Flag_Show;								//长按 进入/退出 上位机模式
				break;
			default:break;
		}
}

/*************************************************************************
Function:User_Key_Scan
Input:None
Output:Key_status
函数功能：用户按键检测
入口参数：无
返回值  ：按键状态
**************************************************************************/
//放在5ms中断中调用
uint8_t User_Key_Scan(void)
{
	static uint16_t count_time = 0;					//计算按下的时间，每5ms加1
	static uint8_t key_step = 0;						//记录此时的步骤
	switch(key_step)
	{
		case 0:
			if(PAin(0) == KEY_ON )
				key_step++;						//检测到有按键按下，进入下一步
			break;
		case 1:
			if((++count_time) == 5)				//延时消抖
			{
				if(PAin(0) == KEY_ON )//按键确实按下了
					key_step++,count_time = 0;	//进入下一步
				else
					count_time = 0,key_step = 0;//否则复位
			}
			break;
		case 2:
			if(PAin(0) == KEY_ON )
				count_time++;					//计算按下的时间
			else 								//此时已松开手
				key_step++;						//进入下一步
			break;
		case 3:									//此时看按下的时间，来判断是长按还是短按
			if(count_time > 400)				//在5ms中断中调用，故按下时间若大于400*5 = 2000ms（大概值）
			{							
				key_step = 0;					//标志位复位
				count_time = 0;
				return Long_Press;				//返回 长按 的状态 
 			}
			else if(count_time > 5)				//此时是单击了一次
			{
				key_step++;						//此时进入下一步，判断是否是双击
				count_time = 0;					//按下的时间清零
			}
			else
			{
				key_step = 0;
				count_time = 0;	
			}
			break;
		case 4:									//判断是否是双击或单击
			if(++count_time == 38)				//5*38 = 190ms内判断按键是否按下
			{
				if(PAin(0) == KEY_ON )	//按键确实按下了
				{																	//这里双击不能按太快，会识别成单击
					key_step++;														//进入下一步，需要等松手才能释放状态
					count_time = 0;
				}
				else																//190ms内无按键按下，此时是单击的状态
				{
					key_step = 0;				//标志位复位
					count_time = 0;					
					return Click;				//返回单击的状态
				}
			}
			break;
		case 5:
			if(PAin(0) == KEY_ON )//按键还在按着
			{
				count_time++;
			}
			else								//按键已经松手
			{
//				if(count_time>400)				//这里第二次的单击也可以判断时间的，默认不判断时间，全部都返回双击
//				{
//				}
				count_time = 0;
				key_step = 0;
				return Double_Click;
			}
			break;
		default:break;
	}
	return No_Action;							//无动作
}


/**************************************************************************
函数功能：按键扫描函数
入口参数：执行该函数的任务频率、延迟滤波的时间
返回  值：long_click、double_click、single_click、key_stateless（长按、双击、单击、无状态）
作    者：WHEELTEC
**************************************************************************/
uint8_t KEY_Scan(uint16_t Frequency,uint16_t filter_times)
{
    static uint16_t time_core;//走时核心
    static uint16_t long_press_time;//长按识别
    static uint8_t press_flag=0;//按键按下标记
    static uint8_t check_once=0;//是否已经识别1次标记
    static uint16_t delay_mini_1;
    static uint16_t delay_mini_2;
	
    float Count_time = (((float)(1.0f/(float)Frequency))*1000.0f);//算出计1需要多少个毫秒

    if(check_once)//完成了识别，则清空所有变量
    {
        press_flag=0;//完成了1次识别，标记清零
        time_core=0;//完成了1次识别，时间清零
        long_press_time=0;//完成了1次识别，时间清零
        delay_mini_1=0;
        delay_mini_2=0;
    }
    if(check_once&&KEY==KEY_OFF) check_once=0; //完成扫描后按键被弹起，则开启下一次扫描

    if(KEY==KEY_ON&&check_once==0)//按键扫描
    {
        press_flag=1;//标记被按下1次
		
        if(++delay_mini_1>filter_times)
        {
            delay_mini_1=0;
            long_press_time++;		
        }
    }

    if(long_press_time>(uint16_t)(500.0f/Count_time))// 长按1秒
    {	
        check_once=1;//标记已被识别
        return long_click; //长按
    }

    //按键被按下1次又弹起后，开启内核走时
    if(press_flag&&KEY==KEY_OFF)
    {
        if(++delay_mini_2>filter_times)
        {
            delay_mini_2=0;
            time_core++; 
        }
    }		
	
    if(press_flag&&(time_core>(uint16_t)(50.0f/Count_time)&&time_core<(uint16_t)(300.0f/Count_time)))//50~700ms内被再次按下
    {
        if(KEY==KEY_ON) //如果再次按下
        {
            check_once=1;//标记已被识别
            return double_click; //标记为双击
        }
    }
    else if(press_flag&&time_core>(uint16_t)(300.0f/Count_time))
    {
        check_once=1;//标记已被识别
        return single_click; //800ms后还没被按下，则是单击
    }

    return key_stateless;
}
