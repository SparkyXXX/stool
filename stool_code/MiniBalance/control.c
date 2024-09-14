/***********************************************
��˾����Ȥ�Ƽ�(��ݸ)���޹�˾
Ʒ�ƣ�WHEELTEC
������wheeltec.net
�Ա����̣�shop114407458.taobao.com 
����ͨ: https://minibalance.aliexpress.com/store/4455017
�汾��V1.0
�޸�ʱ�䣺2023-05-25

Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version: V1.0
Update��2023-05-25

All rights reserved
***********************************************/
#include "control.h"
short Accel_Y,Accel_Z,Accel_X,Accel_Angle_x,Accel_Angle_y,Gyro_X,Gyro_Z,Gyro_Y;
//LQR״̬����ϵ��
float K11=81.2695, K12=-10.0616, K13=-5492.4061, K14=18921.7098, K15=100.3633, K16=8.0376, K17=447.3084, K18=2962.7738;
float K21=-10.0616, K22=81.2695, K23=-5492.4061, K24=18921.7098, K25=8.0376, K26=100.3633, K27=447.3084, K28=2962.7738;
//��ر���
float Target_theta_L, Target_theta_R, Target_theta_L_dot, Target_theta_R_dot, Target_theta_1;
float u_L, u_R;																							//ϵͳ����������������ֵĽǼ��ٶ�(rad/s^2)
float t=0.01;																   		    			//��ɢʱ����10ms
float theta_2, last_theta_2, theta_dot_2, last_theta_dot_2;	//�ڸ˵����(rad),�ڸ���һ�ε����(rad),�ڸ˵���ǽ��ٶ�(rad/s),�ڸ���һ�ε���ǽ��ٶ�(rad/s).
float theta_L_dot_instant, theta_R_dot_instant;							//���ֵ�ת��(rad/s)�����ֵ�ת��(rad/s)��5ms.
float theta_L, theta_R, last_theta_L, last_theta_R;					//����ת���ĽǶ�(rad)������ת���ĽǶ�(rad)��������һ��ת���ĽǶ�(rad)��������һ��ת���ĽǶ�(rad).
float theta_L_dot, theta_R_dot;															//���ֵ�ƽ��ת��(rad/s)�����ֵ�ƽ��ת��(rad/s),10ms.
float TargetVal_L, TargetVal_R;															//���ֵ�Ŀ��ת�ٱ���(rad/s)�����ֵ�Ŀ��ת�ٱ���(rad/s).
float theta_1,theta_dot_1;			//��������(rad)���������ǽ��ٶ�(rad/s).
float mid=3100;									//�ڸ�ƽ��λ�ö�Ӧ��ADCֵ
int PWM_L, PWM_R;								//���ֺ����ֵ�PWMֵ����
int Moto_Ki=40, Moto_Kp=10;			//PI������ϵ��
uint8_t count=0;											//����������
uint8_t stop=0;

/**************************************************************************
Function: Control function
Input   : none
Output  : none
�������ܣ����еĿ��ƴ��붼��������
         5ms�ⲿ�ж���MPU6050��INT���Ŵ���
         �ϸ�֤���������ݴ����ʱ��ͬ��
��ڲ�������
����  ֵ����
**************************************************************************/

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	static int Voltage_Temp,Voltage_Count,Voltage_All;		//��ѹ������ر���
	static uint8_t Flag_Target;																//���ƺ�����ر������ṩ10ms��׼
	uint8_t key;
	if(GPIO_Pin==GPIO_PIN_9){
		
		Encoder_Left=Read_Encoder(4);            					  //��ȡ���ֱ�������ֵ��ǰ��Ϊ��������Ϊ��
		Encoder_Right=-Read_Encoder(8);           					//��ȡ���ֱ�������ֵ��ǰ��Ϊ��������Ϊ��
		Angle_ADC = Get_Adc_Average(Angle_Ch,30);
		Flag_Target=!Flag_Target;
		Get_Angle(Way_Angle);                     					//������̬��5msһ�Σ����ߵĲ���Ƶ�ʿ��Ը��ƿ������˲��ͻ����˲���Ч��
		count += 1;																					//��������10ms����һ���µ��ٶ�Ŀ��ֵ��
		//�����л���λ��ģʽ
		key = KEY_Scan(200,0);
		if( key==long_click ) Flag_Show  = !Flag_Show;
		
		if(delay_flag==1)
		{
			delay_50++;
			if(delay_50==10) delay_50=0,delay_flag=0;  		//���������ṩ50ms�ľ�׼��ʱ��ʾ������Ҫ50ms�߾�����ʱ
		}
		if(Flag_Target==1)                        					//10ms����һ��
		{
			Voltage_Temp=Get_battery_volt();		    					//��ȡ��ص�ѹ
			Voltage_Count++;                       						//ƽ��ֵ������
			Voltage_All+=Voltage_Temp;              					//��β����ۻ�
			if(Voltage_Count==100) Voltage=Voltage_All/100,Voltage_All=0,Voltage_Count=0;//��ƽ��ֵ
			return ;	                                               
		}                                         					//10ms����һ��
		theta_L_dot_instant = Encoder_Left /60000.0f * PI*2.0f /0.005f;	//���ֵ�ת��(rad/s)(5ms)
		Velocity_Left = theta_L_dot_instant*Diameter_67/2.0f;           //����ת��(mm/s)
		theta_L += theta_L_dot_instant*0.005f;													//����ת���ĽǶ�(rad)

		theta_R_dot_instant = Encoder_Right/60000.0f * PI*2.0f /0.005f;	//���ֵ�ת��(rad/s)(5ms)
		Velocity_Right = theta_R_dot_instant*Diameter_67/2.0f;          //����ת��(mm/s)
		theta_R += theta_R_dot_instant*0.005f;													//����ת���ĽǶ�(rad)

		Displacement+=(Velocity_Left+Velocity_Right)*0.005/2;				//����λ��(mm)

		theta_1 = Angle_Balance/180.0f*PI;													//��������(rad)
		theta_dot_1 += Gyro_Balance/16.4f*(PI/180.0f);  						//�������ǽ��ٶ�(rad/s)(5ms). ע������������ת�������̡�2000��/s��Ӧ������16.4���ɲ��ֲ�.

		if (count == 2)
		{
			count = 0;																				//����������

			theta_L_dot = (theta_L - last_theta_L)/t;					//���ֵ�ƽ��ת��(rad/s)(10ms)
			last_theta_L = theta_L;

			theta_R_dot = (theta_R - last_theta_R)/t;					//���ֵ�ƽ��ת��(rad/s)(10ms)
			last_theta_R = theta_R;

			theta_dot_1 = theta_dot_1*0.5;										//�����ƽ����ǽ��ٶ�(rad/s)(10ms)

			theta_2 = ( Angle_Balance + angle_count(Angle_ADC, mid) ) / 180.0f * PI;  //�ڸ˵����(rad)
			theta_dot_2 = (theta_2 - last_theta_2) / t;																//�ڸ˵���ǽ��ٶ�(rad/s)(10ms)
			theta_dot_2 = 0.4*theta_dot_2 + 0.6*last_theta_dot_2;											//һ�׵�ͨ�˲���
			last_theta_dot_2 = theta_dot_2;
			last_theta_2 = theta_2;

			Normal();		//��ͨģʽ

			//�����������(LQR������)
			u_L=-(K11*(theta_L-Target_theta_L) + K12*(theta_R-Target_theta_R) + K13*(theta_1-Target_theta_1) + K14*theta_2 \
			+ K15*(theta_L_dot-Target_theta_L_dot) + K16*(theta_R_dot-Target_theta_R_dot) + K17*theta_dot_1 + K18*theta_dot_2);

			u_R=-(K21*(theta_L-Target_theta_L) + K22*(theta_R-Target_theta_R) + K23*(theta_1-Target_theta_1) + K24*theta_2 \
			+ K25*(theta_L_dot-Target_theta_L_dot) + K26*(theta_R_dot-Target_theta_R_dot) + K27*theta_dot_1 + K28*theta_dot_2);

			if ( (theta_1<0.7854 && theta_1>-0.7854) )
			{
				TargetVal_L = theta_L_dot + u_L*t;											//���ֵ�Ŀ���ٶ�
				TargetVal_R = theta_R_dot + u_R*t;											//���ֵ�Ŀ���ٶ�
				PWM_L=Incremental_L(theta_L_dot_instant,TargetVal_L);		//�����ֽ����ٶ�PI����
				PWM_R=Incremental_R(theta_R_dot_instant,TargetVal_R);		//�����ֽ����ٶ�PI����
			}
			else
			{
				stop = 1;
				PWM_L = 0;
				PWM_R = 0;
			}
			theta_dot_1 = 0;
		}

		if(Mode==Normal_Mode)	Led_Flash(100);         //LED��˸;����ģʽ 1s�ı�һ��ָʾ�Ƶ�״̬
		else Led_Flash(0);                              //LED����;����ģʽ

		PWM_L=PWM_Limit(PWM_L,6900,-6900);		  		//PWM�޷�
		PWM_R=PWM_Limit(PWM_R,6900,-6900);		  		//PWM�޷�
		
		Motor_Left=PWM_L;                              //����PWM
		Motor_Right=PWM_R;                             //�ҵ��PWM

		Set_Pwm(PWM_L,PWM_R);	                       //��ֵ��PWM�Ĵ���
		
		//���뼱ͣ����
		Flag_Stop=KEY2_STATE;
		if(Voltage<10) Flag_Stop = 1;
		if(Flag_Stop) PWMA_IN1=0,PWMA_IN2=0,PWMB_IN1=0,PWMB_IN2=0;
	}
	return ;
}
///**************************************************************************
//�������ܣ�����ʽPI������
//��ڲ����������ٶȡ�Ŀ���ٶ�
//�� �� ֵ��PWMֵ
//��    �ߣ�WHEELTEC
//**************************************************************************/
static int Incremental_L(float CurrentVal,float TargetVal)
{
	float Bias;
	static float  Last_bias;
	static int PWM;

	Bias =  TargetVal - CurrentVal;
	PWM += Moto_Ki*Bias + Moto_Kp*(Bias-Last_bias);
	Last_bias=Bias;
	
	//ֹͣ���к������ʷ��ֵ
	if(Flag_Stop || stop ) PWM=0,stop=0;
	
	return PWM;
}
///**************************************************************************
//�������ܣ�����ʽPI������
//��ڲ����������ٶȡ�Ŀ���ٶ�
//�� �� ֵ��PWMֵ
//��    �ߣ�WHEELTEC
//**************************************************************************/
static int Incremental_R(float CurrentVal,float TargetVal)
{
	float Bias;
	static float  Last_bias;
	static int PWM;

	Bias =  TargetVal - CurrentVal;
	PWM += Moto_Ki*Bias + Moto_Kp*(Bias-Last_bias);
	Last_bias=Bias;
	
	//ֹͣ���к������ʷ��ֵ
	if(Flag_Stop || stop ) PWM=0,stop=0;
	
	return PWM;
}
///**************************************************************************
//�������ܣ�ͨ�������ADC�ɼ�ֵ����Ƕ�
//��ڲ�������ǰADC�ɼ�ֵ������ʱ�ο�����㣨��ֵ��
//�� �� ֵ����ǰ�Ƕ�
//��    �ߣ�WHEELTEC
//**************************************************************************/
static float angle_count(float Angle_ADC, float mid)
{
	float Angle;
	Angle = (mid - Angle_ADC)*0.0879f;
	return Angle;
}
///**************************************************************************
//Function: Assign to PWM register
//Input   : motor_left��Left wheel PWM��motor_right��Right wheel PWM
//Output  : none
//�������ܣ���ֵ��PWM�Ĵ���
//��ڲ���������PWM������PWM
//����  ֵ����
//**************************************************************************/
void Set_Pwm(int motor_left,int motor_right)
{
	//����ǰ��
  if(motor_left>0)
	{
		PWMA_IN1=7200;
		PWMA_IN2=7200-motor_left;
	}
	//���ֺ���
	else
	{
		PWMA_IN1=7200+motor_left;
		PWMA_IN2=7200;
	}
	//����ǰ��
  if(motor_right>0)
	{
		PWMB_IN1=7200-motor_right;
		PWMB_IN2=7200;
	}
	//���ֺ���
	else
	{
		PWMB_IN1=7200;
		PWMB_IN2=7200+motor_right;
	}
}
///**************************************************************************
//Function: PWM limiting range
//Input   : IN��Input  max��Maximum value  min��Minimum value
//Output  : Output
//�������ܣ�����PWM��ֵ 
//��ڲ�����IN���������  max���޷����ֵ  min���޷���Сֵ
//����  ֵ���޷����ֵ
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
//Input   : angle��Car inclination��voltage��Voltage
//Output  : 1��abnormal��0��normal
//�������ܣ��쳣�رյ��		
//��ڲ�����angle��С����ǣ�voltage����ѹ
//����  ֵ��1���쳣  0������
//**************************************************************************/	
uint8_t Turn_Off(float angle, int voltage)
{
	uint8_t temp;
	Flag_Stop = KEY2_STATE;                             
	if(KEY2_STATE==1) Pick_up_stop=0;                  //key2�رգ�Pick_up_stop�ָ�Ϊ0
	if(angle<-40||angle>40||1==Flag_Stop||voltage<1110||Pick_up_stop==1)//��ص�ѹ����11.1V�رյ��
	{	                                                 //��Ǵ���40�ȹرյ��
		temp=1;                                          //Flag_Stop��1�����������ƹرյ��
		PWMA_IN1=0;                                      //Pick_up_stop��1����С��������ֹ����0����������С��      
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
//Input   : way��The algorithm of getting angle 1��DMP  2��kalman  3��Complementary filtering
//Output  : none
//�������ܣ���ȡ�Ƕ�	
//��ڲ�����way����ȡ�Ƕȵ��㷨 1��DMP  2�������� 3�������˲�
//����  ֵ����
//**************************************************************************/	
void Get_Angle(uint8_t way)
{ 
  	float gyro_x,gyro_y,accel_x,accel_y,accel_z;
	//Temperature=Read_Temperature();      //��ȡMPU6050�����¶ȴ��������ݣ����Ʊ�ʾ�����¶ȡ�
	if(way==1)                           //DMP�Ķ�ȡ�����ݲɼ��ж϶�ȡ���ϸ���ѭʱ��Ҫ��
	{	
		Read_DMP();                      	 //��ȡ���ٶȡ����ٶȡ����
		Angle_Balance=Pitch;             	 //����ƽ�����,ǰ��Ϊ��������Ϊ��
		Gyro_Balance=gyro[0];              //����ƽ����ٶ�,ǰ��Ϊ��������Ϊ��
		Gyro_Turn=gyro[2];                 //����ת����ٶ�
		Acceleration_Z=accel[2];           //����Z����ٶȼ�
	}			
	else
	{
		Gyro_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_XOUT_L);    //��ȡX��������
		Gyro_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_YOUT_L);    //��ȡY��������
		Gyro_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_GYRO_ZOUT_L);    //��ȡZ��������
		Accel_X=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_XOUT_L); //��ȡX����ٶȼ�
		Accel_Y=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_YOUT_L); //��ȡX����ٶȼ�
		Accel_Z=(I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_H)<<8)+I2C_ReadOneByte(devAddr,MPU6050_RA_ACCEL_ZOUT_L); //��ȡZ����ٶȼ�
		Gyro_Balance=-Gyro_X;                            //����ƽ����ٶ�
		accel_x=Accel_X/1671.84;
		accel_y=Accel_Y/1671.84;
		accel_z=Accel_Z/1671.84;
		gyro_x=Gyro_X/939.8;                              //����������ת��
		gyro_y=Gyro_Y/939.8;                              //����������ת��	
		if(Way_Angle==2)		  	
		{
			 Pitch= KF_X(accel_y,accel_z,-gyro_x)/PI*180;//�������˲�
			 Roll = KF_Y(accel_x,accel_z,gyro_y)/PI*180;
		}
		else if(Way_Angle==3) 
		{  
			 Pitch = -Complementary_Filter_x(Accel_Angle_x,gyro_x);//�����˲�
			 Roll = -Complementary_Filter_y(Accel_Angle_y,gyro_y);
		}
		Angle_Balance=Pitch;                              //����ƽ�����
		Gyro_Turn=Gyro_Z;                                 //����ת����ٶ�
		Acceleration_Z=Accel_Z;                           //����Z����ٶȼ�	
	}
}
///**************************************************************************
//Function: Absolute value function
//Input   : a��Number to be converted
//Output  : unsigned int
//�������ܣ�����ֵ����
//��ڲ�����a����Ҫ�������ֵ����
//����  ֵ���޷�������
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
//�������ܣ���ͨģʽ
//��ڲ�������
//����  ֵ����
//**************************************************************************/
void Normal(void)
{
	if(Mode == Normal_Mode)									  //��ͨ�Ŀ���ģʽ�ɽ����ֱ�����
	{
		//����С��ǰ���ͺ���
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
		//����С����ת����ת
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
		//ȡ������
		{
			Target_theta_L_dot = 0;
			Target_theta_R_dot = 0;
			Target_theta_1 = 0;
		}
	}
}




