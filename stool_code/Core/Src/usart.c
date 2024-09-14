/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "sys.h"
SEND_DATA Send_Data;
RECEIVE_DATA Receive_Data;
extern int Time_count;
extern short Accel_Y,Accel_Z,Accel_X,Accel_Angle_x,Accel_Angle_y,Gyro_X,Gyro_Z,Gyro_Y;
extern float Velocity_Left,Velocity_Right;	//�����ٶ�(mm/s)
#define CONTROL_DELAY		1000
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos ʹ��	  
#endif

#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	if(Flag_Show==0)
	{	
	  while((USART3->SR&0X40)==0);//Flag_Show=0  ʹ�ô���3   
	  USART3->DR = (uint8_t) ch;      
	}
	else
	{	
		while((USART1->SR&0X40)==0);//Flag_Show!=0  ʹ�ô���1   
		USART1->DR = (uint8_t) ch;      
	}	
	return ch;
}

#endif
/* USER CODE END 0 */

UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* UART5 init function */
void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 230400;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */
	HAL_UART_Receive_IT(&huart5,&temp_data,sizeof(temp_data));
  /* USER CODE END UART5_Init 2 */

}
/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */
	
  /* USER CODE END USART1_Init 2 */

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */
	HAL_UART_Receive_IT(&huart3,Usart3_Receive_buf,sizeof(Usart3_Receive_buf)); //�򿪴���3�����ж�
  /* USER CODE END USART3_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==UART5)
  {
  /* USER CODE BEGIN UART5_MspInit 0 */

  /* USER CODE END UART5_MspInit 0 */
    /* UART5 clock enable */
    __HAL_RCC_UART5_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**UART5 GPIO Configuration
    PC12     ------> UART5_TX
    PD2     ------> UART5_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* UART5 interrupt Init */
    HAL_NVIC_SetPriority(UART5_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(UART5_IRQn);
  /* USER CODE BEGIN UART5_MspInit 1 */

  /* USER CODE END UART5_MspInit 1 */
  }
  else if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 1, 1);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==UART5)
  {
  /* USER CODE BEGIN UART5_MspDeInit 0 */

  /* USER CODE END UART5_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART5_CLK_DISABLE();

    /**UART5 GPIO Configuration
    PC12     ------> UART5_TX
    PD2     ------> UART5_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_12);

    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);

    /* UART5 interrupt Deinit */
    HAL_NVIC_DisableIRQ(UART5_IRQn);
  /* USER CODE BEGIN UART5_MspDeInit 1 */

  /* USER CODE END UART5_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */

///**************************************************
//Function:Serial port 1 receivees interrupted
//Input:None
//Output:None
//����������һ�ж�
//��ڲ�������
//�Ż�ֵ����
//**************************************************/
//void USART1_IRQHandler(void)
//{
//	static uint8_t Count=0;
//	uint8_t Usart_Receive;
//	if(USART_GetITStatus(USART1,USART_IT_RXNE)!= RESET)//cheak if data is receives �ж��Ƿ���յ�����
//	{
//		USART_ClearITPendingBit(USART1,USART_IT_RXNE);
//		Usart_Receive = USART_ReceiveData(USART1);//Read Data ������
//		Mode=ROS_Mode;//ros����ʱ����С��ģʽ��ΪROSģʽ
//////		if(Time_count < CONTROL_DELAY)
//////		{
//////			//Data is not processed until 10 seconds after startup
//////			//����ʮ��ǰ�������������
//////		}
//		Receive_Data.buffer[Count] = Usart_Receive;//Fill the array with serial data  ����������������
//		
//		//Ensure that the first data is the array is FRAME_HEADER
//		//ȷ����һ������ʱ֡ͷ
//		if(Usart_Receive == FRAME_HEADER ||Count>0)
//			Count++;
//		else Count=0;
//		if(Count == 11) //Verify the length of the packet  ��֤���ݰ��ĳ���
//		{
//			Count = 0;
//			if(Receive_Data.buffer[10] == FRAME_TAIL)
//			{
//				if(Receive_Data.buffer[9] == Check_Sum(9,0))
//				{
//					//Calculate the 3-axis target velocity from the serial data,which is divided into 8-bit high and 8-bit low units mm/s
//					Move_X=XYZ_Target_Speed_transition(Receive_Data.buffer[3],Receive_Data.buffer[4]);//ƽ��С��û�����õ��˶�ѧ�������
//					Move_Z=-XYZ_Target_Speed_transition(Receive_Data.buffer[7],Receive_Data.buffer[8])/1000*160/2/Control_Frequency*EncoderMultiples*Reduction_Ratio*Encoder_precision/Perimeter;//Move_Z��ֱ��������ת��
//					//Move_Z=ROS���Ļ���/1000*�־�/2/�ܳ�/��������ȡƵ��*Ƶ��*���ٱ�*���ȣ�
//					Ros_Rate=0;  //ֻҪ�жϽ������Ͱ�����0����ֹС�����ƵĿ���
//				}
//			}
//		}
//	}
//}

/**************************************************************************
Function: The data sent by the serial port is assigned
Input   : none
Output  : none
�������ܣ����ڷ��͵����ݽ��и�ֵ
��ڲ�������
����  ֵ����
**************************************************************************/
void data_transition(void)
{
	Send_Data.Sensor_Str.Frame_Header = FRAME_HEADER; //Frame_header //֡ͷ
	Send_Data.Sensor_Str.Frame_Tail = FRAME_TAIL;     //Frame_tail //֡β
	
	//According to different vehicle types, different kinematics algorithms were selected to carry out the forward kinematics solution, 
	//and the three-axis velocity was obtained from each wheel velocity
	//�����˶�ѧ�㷨�����˶�ѧ���⣬�Ӹ������ٶ���������ٶ�
	
	Send_Data.Sensor_Str.X_speed = ((Velocity_Left+Velocity_Right)/2); //ƽ��С�����ٶ���mm/s
	Send_Data.Sensor_Str.Y_speed = 0;
	Send_Data.Sensor_Str.Z_speed = ((Velocity_Right-Velocity_Left)/160)*1000;//160Ϊ�־�
	
	
	//The acceleration of the triaxial acceleration //���ٶȼ�������ٶ�
	Send_Data.Sensor_Str.Accelerometer.X_data= -Accel_Y; //The accelerometer Y-axis is converted to the ros coordinate X axis //���ٶȼ�Y��ת����ROS����X��
	Send_Data.Sensor_Str.Accelerometer.Y_data=Accel_X; //The accelerometer X-axis is converted to the ros coordinate y axis //���ٶȼ�X��ת����ROS����Y��
	Send_Data.Sensor_Str.Accelerometer.Z_data= Accel_Z; //The accelerometer Z-axis is converted to the ros coordinate Z axis //���ٶȼ�Z��ת����ROS����Z��
	
	//The Angle velocity of the triaxial velocity //���ٶȼ�������ٶ�
	Send_Data.Sensor_Str.Gyroscope.X_data= -Gyro_Y*4; //The Y-axis is converted to the ros coordinate X axis //���ٶȼ�Y��ת����ROS����X��
	Send_Data.Sensor_Str.Gyroscope.Y_data= Gyro_X*4; //The X-axis is converted to the ros coordinate y axis //���ٶȼ�X��ת����ROS����Y��
	if(Flag_Stop==0) 
		//If the motor control bit makes energy state, the z-axis velocity is sent normall
	  //����������λʹ��״̬����ô��������Z����ٶ�
		Send_Data.Sensor_Str.Gyroscope.Z_data=Gyro_Z*4;  
	else  
		//If the robot is static (motor control dislocation), the z-axis is 0
    //����������Ǿ�ֹ�ģ��������λʧ�ܣ�����ô���͵�Z����ٶ�Ϊ0		
		Send_Data.Sensor_Str.Gyroscope.Z_data=0;        
	
	//Battery voltage (this is a thousand times larger floating point number, which will be reduced by a thousand times as well as receiving the data).
	//��ص�ѹ(���ｫ�����Ŵ�ʮ�����䣬��Ӧ���ڽ��ն��ڽ��յ����ݺ����Сһǧ���õ�������ʾ�ĵ�ѹ������VoltageΪ1140���Ŵ�ʮ��������Сһǧ��Ϊ11.40)
	Send_Data.Sensor_Str.Power_Voltage = (float)Voltage*10; 
	
	Send_Data.buffer[0]=Send_Data.Sensor_Str.Frame_Header; //Frame_heade //֡ͷ
  Send_Data.buffer[1]=Flag_Stop; //Car software loss marker //С�����ʧ�ܱ�־λ
	
	//The three-axis speed of / / car is split into two eight digit Numbers
	//С�������ٶ�,���ᶼ���Ϊ����8λ�����ٷ���
	Send_Data.buffer[2]=Send_Data.Sensor_Str.X_speed>>8; 
	Send_Data.buffer[3]=Send_Data.Sensor_Str.X_speed;    
	Send_Data.buffer[4]=Send_Data.Sensor_Str.Y_speed>>8;  
	Send_Data.buffer[5]=Send_Data.Sensor_Str.Y_speed;     
	Send_Data.buffer[6]=Send_Data.Sensor_Str.Z_speed>>8; 
	Send_Data.buffer[7]=Send_Data.Sensor_Str.Z_speed;    
	
	//The acceleration of the triaxial axis of / / imu accelerometer is divided into two eight digit reams
	//IMU���ٶȼ�������ٶ�,���ᶼ���Ϊ����8λ�����ٷ���
	Send_Data.buffer[8]=Send_Data.Sensor_Str.Accelerometer.X_data>>8; 
	Send_Data.buffer[9]=Send_Data.Sensor_Str.Accelerometer.X_data;   
	Send_Data.buffer[10]=Send_Data.Sensor_Str.Accelerometer.Y_data>>8;
	Send_Data.buffer[11]=Send_Data.Sensor_Str.Accelerometer.Y_data;
	Send_Data.buffer[12]=Send_Data.Sensor_Str.Accelerometer.Z_data>>8;
	Send_Data.buffer[13]=Send_Data.Sensor_Str.Accelerometer.Z_data;
	
	//The axis of the triaxial velocity of the / /imu is divided into two eight digits
	//IMU���ٶȼ�������ٶ�,���ᶼ���Ϊ����8λ�����ٷ���
	Send_Data.buffer[14]=Send_Data.Sensor_Str.Gyroscope.X_data>>8;
	Send_Data.buffer[15]=Send_Data.Sensor_Str.Gyroscope.X_data;
	Send_Data.buffer[16]=Send_Data.Sensor_Str.Gyroscope.Y_data>>8;
	Send_Data.buffer[17]=Send_Data.Sensor_Str.Gyroscope.Y_data;
	Send_Data.buffer[18]=Send_Data.Sensor_Str.Gyroscope.Z_data>>8;
	Send_Data.buffer[19]=Send_Data.Sensor_Str.Gyroscope.Z_data;
	
	//Battery voltage, split into two 8 digit Numbers
	//��ص�ѹ,���Ϊ����8λ���ݷ���
	Send_Data.buffer[20]=Send_Data.Sensor_Str.Power_Voltage>>8; 
	Send_Data.buffer[21]=Send_Data.Sensor_Str.Power_Voltage; 

  //Data check digit calculation, Pattern 1 is a data check
  //����У��λ���㣬ģʽ1�Ƿ�������У��
	Send_Data.buffer[22]=Check_Sum(22,1); 
	
	Send_Data.buffer[23]=Send_Data.Sensor_Str.Frame_Tail; //Frame_tail //֡β
}


/**************************************************************************
Function: After the top 8 and low 8 figures are integrated into a short type data, the unit reduction is converted
Input   : 8 bits high, 8 bits low
Output  : The target velocity of the robot on the X/Y/Z axis
�������ܣ�����λ���������ĸ�8λ�͵�8λ�������ϳ�һ��short�����ݺ�������λ��ԭ����
��ڲ�������8λ����8λ
����  ֵ��������X/Y/Z���Ŀ���ٶ�
**************************************************************************/
float XYZ_Target_Speed_transition(uint8_t High,uint8_t Low)
{
	//Data conversion intermediate variable
	//����ת�����м����
	short transition; 
	
	//����8λ�͵�8λ���ϳ�һ��16λ��short������
	//The high 8 and low 8 bits are integrated into a 16-bit short data
	transition=((High<<8)+Low); 
	return transition; 
								
}
/**************************************************************************
Function: Serial port 1 sends data
Input   : The data to send
Output  : none
�������ܣ�����1��������
��ڲ�����Ҫ���͵�����
����  ֵ����
**************************************************************************/
void usart1_send(uint8_t data)
{
	USART1->DR = data;
	while((USART1->SR&0x40)==0);	
}

/**************************************************************************
Function: Serial port 1 sends data
Input   : none
Output  : none
�������ܣ�����1��������
��ڲ�������
����  ֵ����
**************************************************************************/
void USART1_SEND(void)
{
  unsigned char i = 0;	
	
	for(i=0; i<24; i++)
	{
		usart1_send(Send_Data.buffer[i]);
	}	 
}

/**************************************************************************
Function: Calculates the check bits of data to be sent/received
Input   : Count_Number: The first few digits of a check; Mode: 0-Verify the received data, 1-Validate the sent data
Output  : Check result
�������ܣ�����Ҫ����/���յ�����У����
��ڲ�����Count_Number��У���ǰ��λ����Mode��0-�Խ������ݽ���У�飬1-�Է������ݽ���У��
����  ֵ��У����
**************************************************************************/
uint8_t Check_Sum(unsigned char Count_Number,unsigned char mode)
{
	unsigned char check_sum=0,k;
	
	//Validate the data to be sent
	//��Ҫ���͵����ݽ���У��
	if(mode==1)
	for(k=0;k<Count_Number;k++)
	{
	  check_sum=check_sum^Send_Data.buffer[k];
	}
	
	//Verify the data received
	//�Խ��յ������ݽ���У��
	if(mode==0)
	for(k=0;k<Count_Number;k++)
	{
		check_sum=check_sum^Receive_Data.buffer[k];
	}
	return check_sum;
}
/* USER CODE END 1 */
