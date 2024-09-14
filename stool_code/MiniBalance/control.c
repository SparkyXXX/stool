/***********************************************
公司：轮趣科技(东莞)有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
速卖通: https://minibalance.aliexpress.com/store/4455017
版本：V1.0
修改时间：2023-05-25

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update：2023-05-25

All rights reserved
***********************************************/
#include "control.h"
short Accel_Y,Accel_Z,Accel_X,Accel_Angle_x,Accel_Angle_y,Gyro_X,Gyro_Z,Gyro_Y;
//LQR状态反馈系数
float K11=81.2695, K12=-10.0616, K13=-5492.4061, K14=18921.7098, K15=100.3633, K16=8.0376, K17=447.3084, K18=2962.7738;
float K21=-10.0616, K22=81.2695, K23=-5492.4061, K24=18921.7098, K25=8.0376, K26=100.3633, K27=447.3084, K28=2962.7738;
//相关变量
float Target_theta_L, Target_theta_R, Target_theta_L_dot, Target_theta_R_dot, Target_theta_1;
float u_L, u_R;																							//系统的输入变量，左右轮的角加速度(rad/s^2)
float t=0.01;																   		    			//离散时间间隔10ms
float theta_2, last_theta_2, theta_dot_2, last_theta_dot_2;	//摆杆的倾角(rad),摆杆上一次的倾角(rad),摆杆的倾角角速度(rad/s),摆杆上一次的倾角角速度(rad/s).
float theta_L_dot_instant, theta_R_dot_instant;							//左轮的转速(rad/s)，右轮的转速(rad/s)，5ms.
float theta_L, theta_R, last_theta_L, last_theta_R;					//左轮转过的角度(rad)，右轮转过的角度(rad)，左轮上一次转过的角度(rad)，右轮上一次转过的角度(rad).
float theta_L_dot, theta_R_dot;															//左轮的平均转速(rad/s)，右轮的平均转速(rad/s),10ms.
float TargetVal_L, TargetVal_R;															//左轮的目标转速变量(rad/s)，右轮的目标转速变量(rad/s).
float theta_1,theta_dot_1;			//车身的倾角(rad)，车身的倾角角速度(rad/s).
float mid=3100;									//摆杆平衡位置对应的ADC值
int PWM_L, PWM_R;								//左轮和右轮的PWM值变量
int Moto_Ki=40, Moto_Kp=10;			//PI控制器系数
uint8_t count=0;											//计数器变量
uint8_t stop=0;

/**************************************************************************
Function: Control function
Input   : none
Output  : none
函数功能：所有的控制代码都在这里面
         5ms外部中断由MPU6050的INT引脚触发
         严格保证采样和数据处理的时间同步
入口参数：无
返回  值：无
**************************************************************************/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	static int Voltage_Temp,Voltage_Count,Voltage_All;		//电压测量相关变量
	static uint8_t Flag_Target;																//控制函数相关变量，提供10ms基准
	uint8_t key;
	if(GPIO_Pin==GPIO_PIN_9){
		
		Encoder_Left=Read_Encoder(4);            					  //读取左轮编码器的值，前进为正，后退为负
		Encoder_Right=-Read_Encoder(8);           					//读取右轮编码器的值，前进为正，后退为负
		Angle_ADC = Get_Adc_Average(Angle_Ch,30);
		Flag_Target=!Flag_Target;
		Get_Angle(Way_Angle);                     					//更新姿态，5ms一次，更高的采样频率可以改善卡尔曼滤波和互补滤波的效果
		count += 1;																					//计数器（10ms发布一次新的速度目标值）
		//按键切换上位机模式
		key = KEY_Scan(200,0);
		if( key==long_click ) Flag_Show  = !Flag_Show;
		
		if(delay_flag==1)
		{
			delay_50++;
			if(delay_50==10) delay_50=0,delay_flag=0;  		//给主函数提供50ms的精准延时，示波器需要50ms高精度延时
		}
		if(Flag_Target==1)                        					//10ms控制一次
		{
			Voltage_Temp=Get_battery_volt();		    					//读取电池电压
			Voltage_Count++;                       						//平均值计数器
			Voltage_All+=Voltage_Temp;              					//多次采样累积
			if(Voltage_Count==100) Voltage=Voltage_All/100,Voltage_All=0,Voltage_Count=0;//求平均值
			return ;	                                               
		}                                         					//10ms控制一次
		theta_L_dot_instant = Encoder_Left /60000.0f * PI*2.0f /0.005f;	//左轮的转速(rad/s)(5ms)
		Velocity_Left = theta_L_dot_instant*Diameter_67/2.0f;           //左轮转速(mm/s)
		theta_L += theta_L_dot_instant*0.005f;													//左轮转过的角度(rad)

		theta_R_dot_instant = Encoder_Right/60000.0f * PI*2.0f /0.005f;	//右轮的转速(rad/s)(5ms)
		Velocity_Right = theta_R_dot_instant*Diameter_67/2.0f;          //左轮转速(mm/s)
		theta_R += theta_R_dot_instant*0.005f;													//右轮转过的角度(rad)

		Displacement+=(Velocity_Left+Velocity_Right)*0.005/2;				//车子位移(mm)

		theta_1 = Angle_Balance/180.0f*PI;													//车身的倾角(rad)
		theta_dot_1 += Gyro_Balance/16.4f*(PI/180.0f);  						//车身的倾角角速度(rad/s)(5ms). 注：陀螺仪量程转换，量程±2000°/s对应灵敏度16.4，可查手册.

		if (count == 2)
		{
			count = 0;																				//计数器清零

			theta_L_dot = (theta_L - last_theta_L)/t;					//左轮的平均转速(rad/s)(10ms)
			last_theta_L = theta_L;

			theta_R_dot = (theta_R - last_theta_R)/t;					//右轮的平均转速(rad/s)(10ms)
			last_theta_R = theta_R;

			theta_dot_1 = theta_dot_1*0.5;										//车身的平均倾角角速度(rad/s)(10ms)

			theta_2 = ( Angle_Balance + angle_count(Angle_ADC, mid) ) / 180.0f * PI;  //摆杆的倾角(rad)
			theta_dot_2 = (theta_2 - last_theta_2) / t;																//摆杆的倾角角速度(rad/s)(10ms)
			theta_dot_2 = 0.4*theta_dot_2 + 0.6*last_theta_dot_2;											//一阶低通滤波器
			last_theta_dot_2 = theta_dot_2;
			last_theta_2 = theta_2;

			Normal();		//普通模式

			//计算输入变量(LQR控制器)
			u_L=-(K11*(theta_L-Target_theta_L) + K12*(theta_R-Target_theta_R) + K13*(theta_1-Target_theta_1) + K14*theta_2 \
			+ K15*(theta_L_dot-Target_theta_L_dot) + K16*(theta_R_dot-Target_theta_R_dot) + K17*theta_dot_1 + K18*theta_dot_2);

			u_R=-(K21*(theta_L-Target_theta_L) + K22*(theta_R-Target_theta_R) + K23*(theta_1-Target_theta_1) + K24*theta_2 \
			+ K25*(theta_L_dot-Target_theta_L_dot) + K26*(theta_R_dot-Target_theta_R_dot) + K27*theta_dot_1 + K28*theta_dot_2);

			if ( (theta_1<0.7854 && theta_1>-0.7854) )
			{
				TargetVal_L = theta_L_dot + u_L*t;											//左轮的目标速度
				TargetVal_R = theta_R_dot + u_R*t;											//右轮的目标速度
				PWM_L=Incremental_L(theta_L_dot_instant,TargetVal_L);		//对左轮进行速度PI控制
				PWM_R=Incremental_R(theta_R_dot_instant,TargetVal_R);		//对右轮进行速度PI控制
			}
			else
			{
				stop = 1;
				PWM_L = 0;
				PWM_R = 0;
			}
			theta_dot_1 = 0;
		}

		if(Mode==Normal_Mode)	Led_Flash(100);         //LED闪烁;常规模式 1s改变一次指示灯的状态
		else Led_Flash(0);                              //LED常亮;其余模式

		PWM_L=PWM_Limit(PWM_L,6900,-6900);		  		//PWM限幅
		PWM_R=PWM_Limit(PWM_R,6900,-6900);		  		//PWM限幅
		
		Motor_Left=PWM_L;                              //左电机PWM
		Motor_Right=PWM_R;                             //右电机PWM

		Set_Pwm(PWM_L,PWM_R);	                       //赋值给PWM寄存器
		
		//拨码急停开关
		Flag_Stop=KEY2_STATE;
		if(Voltage<10) Flag_Stop = 1;
		if(Flag_Stop) PWMA_IN1=0,PWMA_IN2=0,PWMB_IN1=0,PWMB_IN2=0;
	}
	return ;
}
///**************************************************************************
//函数功能：增量式PI控制器
//入口参数：测量速度、目标速度
//返 回 值：PWM值
//作    者：WHEELTEC
//**************************************************************************/
static int Incremental_L(float CurrentVal,float TargetVal)
{
	float Bias;
	static float  Last_bias;
	static int PWM;

	Bias =  TargetVal - CurrentVal;
	PWM += Moto_Ki*Bias + Moto_Kp*(Bias-Last_bias);
	Last_bias=Bias;
	
	//停止运行后清空历史数值
	if(Flag_Stop || stop ) PWM=0,stop=0;
	
	return PWM;
}
///**************************************************************************
//函数功能：增量式PI控制器
//入口参数：测量速度、目标速度
//返 回 值：PWM值
//作    者：WHEELTEC
//**************************************************************************/
static int Incremental_R(float CurrentVal,float TargetVal)
{
	float Bias;
	static float  Last_bias;
	static int PWM;

	Bias =  TargetVal - CurrentVal;
	PWM += Moto_Ki*Bias + Moto_Kp*(Bias-Last_bias);
	Last_bias=Bias;
	
	//停止运行后清空历史数值
	if(Flag_Stop || stop ) PWM=0,stop=0;
	
	return PWM;
}
///**************************************************************************
//函数功能：通过输入的ADC采集值计算角度
//入口参数：当前ADC采集值、计算时参考的零点（中值）
//返 回 值：当前角度
//作    者：WHEELTEC
//**************************************************************************/
static float angle_count(float Angle_ADC, float mid)
{
	float Angle;
	Angle = (mid - Angle_ADC)*0.0879f;
	return Angle;
}
///**************************************************************************
//Function: Assign to PWM register
//Input   : motor_left：Left wheel PWM；motor_right：Right wheel PWM
//Output  : none
//函数功能：赋值给PWM寄存器
//入口参数：左轮PWM、右轮PWM
//返回  值：无
//**************************************************************************/
void Set_Pwm(int motor_left,int motor_right)
{
	//左轮前进
  if(motor_left>0)
	{
		PWMA_IN1=7200;
		PWMA_IN2=7200-motor_left;
	}
	//左轮后退
	else
	{
		PWMA_IN1=7200+motor_left;
		PWMA_IN2=7200;
	}
	//右轮前进
  if(motor_right>0)
	{
		PWMB_IN1=7200-motor_right;
		PWMB_IN2=7200;
	}
	//右轮后退
	else
	{
		PWMB_IN1=7200;
		PWMB_IN2=7200+motor_right;
	}
}
///**************************************************************************
//Function: PWM limiting range
//Input   : IN：Input  max：Maximum value  min：Minimum value
//Output  : Output
//函数功能：限制PWM赋值 
//入口参数：IN：输入参数  max：限幅最大值  min：限幅最小值
//返回  值：限幅后的值
//**************************************************************************/
int PWM_Limit(int IN,int max,int min)
{
	int OUT = IN;
	if(OUT>max) OUT = max;
	if(OUT<min) OUT = min;
	return OUT;
}
///**************************************************************************
//Function: If abnormal, turn off the motor
//Input   : angle：Car inclination；voltage：Voltage
//Output  : 1：abnormal；0：normal
//函数功能：异常关闭电机		
//入口参数：angle：小车倾角；voltage：电压
//返回  值：1：异常  0：正常
//**************************************************************************/	
uint8_t Turn_Off(float angle, int voltage)
{
	uint8_t temp;
	Flag_Stop = KEY2_STATE;                             
	if(KEY2_STATE==1) Pick_up_stop=0;                  //key2关闭，Pick_up_stop恢复为0
	if(angle<-40||angle>40||1==Flag_Stop||voltage<1110||Pick_up_stop==1)//电池电压低于11.1V关闭电机
	{	                                                 //倾角大于40度关闭电机
		temp=1;                                          //Flag_Stop置1，即单击控制关闭电机
		PWMA_IN1=0;                                      //Pick_up_stop置1，即小车基本静止，在0度左右拿起小车      
		PWMA_IN2=0;
		PWMB_IN1=0;
		PWMB_IN2=0;
	}
	else
		temp=0;
	return temp;			
}
///**************************************************************************
//Function: Get angle
//Input   : way：The algorithm of getting angle 1：DMP  2：kalman  3：Complementary filtering
//Output  : none
//函数功能：获取角度	
//入口参数：way：获取角度的算法 1：DMP  2：卡尔曼 3：互补滤波
//返回  值：无
//**************************************************************************/	
void Get_Angle(uint8_t way)
{ 
  	float gyro_x,gyro_y,accel_x,accel_y,accel_z;
	//Temperature=Read_Temperature();      //读取MPU6050内置温度传感器数据，近似表示主板温度。
	if(way==1)                           //DMP的读取在数据采集中断读取，严格遵循时序要求
	{	
		Read_DMP();                      	 //读取加速度、角速度、倾角
		Angle_Balance=Pitch;             	 //更新平衡倾角,前倾为正，后倾为负
		Gyro_Balance=gyro[0];              //更新平衡角速度,前倾为正，后倾为负
		Gyro_Turn=gyro[2];                 //更新转向角速度
		Acceleration_Z=accel[2];           //更新Z轴加速度计
	}			
	else
	{
		Gyro_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_L);    //读取X轴陀螺仪
		Gyro_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_L);    //读取Y轴陀螺仪
		Gyro_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);    //读取Z轴陀螺仪
		Accel_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_L); //读取X轴加速度计
		Accel_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_L); //读取X轴加速度计
		Accel_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L); //读取Z轴加速度计
		Gyro_Balance=-Gyro_X;                            //更新平衡角速度
		accel_x=Accel_X/1671.84;
		accel_y=Accel_Y/1671.84;
		accel_z=Accel_Z/1671.84;
		gyro_x=Gyro_X/939.8;                              //陀螺仪量程转换
		gyro_y=Gyro_Y/939.8;                              //陀螺仪量程转换	
		if(Way_Angle==2)		  	
		{
			 Pitch= KF_X(accel_y,accel_z,-gyro_x)/PI*180;//卡尔曼滤波
			 Roll = KF_Y(accel_x,accel_z,gyro_y)/PI*180;
		}
		else if(Way_Angle==3) 
		{  
			 Pitch = -Complementary_Filter_x(Accel_Angle_x,gyro_x);//互补滤波
			 Roll = -Complementary_Filter_y(Accel_Angle_y,gyro_y);
		}
		Angle_Balance=Pitch;                              //更新平衡倾角
		Gyro_Turn=Gyro_Z;                                 //更新转向角速度
		Acceleration_Z=Accel_Z;                           //更新Z轴加速度计	
	}
}
///**************************************************************************
//Function: Absolute value function
//Input   : a：Number to be converted
//Output  : unsigned int
//函数功能：绝对值函数
//入口参数：a：需要计算绝对值的数
//返回  值：无符号整型
//**************************************************************************/	
int myabs(int a)
{ 		   
	int temp;
	if(a<0)  temp=-a;  
	else temp=a;
	return temp;
}
///**************************************************************************
//Function: Normal
//Input   : none
//Output  : none
//函数功能：普通模式
//入口参数：无
//返回  值：无
//**************************************************************************/
void Normal(void)
{
	if(Mode == Normal_Mode)									  //普通的控制模式可进行手柄控制
	{
		//控制小车前进和后退
		if(Flag_front==1)
		{
			Target_theta_L += 0.10;
			Target_theta_R += 0.10;
			Target_theta_L_dot = 0.01;
			Target_theta_R_dot = 0.01;
			Target_theta_1 = 0.0349*2;
		}
		else if(Flag_back==1)
		{
			Target_theta_L -= 0.10;
			Target_theta_R -= 0.10;
			Target_theta_L_dot = -0.01;
			Target_theta_R_dot = -0.01;
			Target_theta_1 = 0.0349*-2;
		}
		//控制小车左转和右转
		else if(Flag_Left==1)
		{
			Target_theta_L -= 0.01;
			Target_theta_R += 0.01;
			Target_theta_L_dot = -0.001;
			Target_theta_R_dot =  0.001;
			Target_theta_1 = 0;
		}
		else if(Flag_Right==1)
		{
			Target_theta_L += 0.01;
			Target_theta_R -= 0.01;
			Target_theta_L_dot =  0.001;
			Target_theta_R_dot = -0.001;
			Target_theta_1 = 0;
		}
		//取消控制
		{
			Target_theta_L_dot = 0;
			Target_theta_R_dot = 0;
			Target_theta_1 = 0;
		}
	}
}




