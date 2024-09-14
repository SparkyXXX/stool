#ifndef __SYS_H
#define __SYS_H
#include "stm32f1xx.h"

// λ������,ʵ��51���Ƶ�GPIO���ƹ���
// ����ʵ��˼��,�ο�<<CM3Ȩ��ָ��>>������(87ҳ~92ҳ).
// IO�ڲ����궨��
#define BITBAND(addr, bitnum) ((addr & 0xF0000000) + 0x2000000 + ((addr & 0xFFFFF) << 5) + (bitnum << 2))
#define MEM_ADDR(addr) *((volatile unsigned long *)(addr))
#define BIT_ADDR(addr, bitnum) MEM_ADDR(BITBAND(addr, bitnum))
// IO�ڵ�ַӳ��
#define GPIOA_ODR_Addr (GPIOA_BASE + 12) // 0x4001080C
#define GPIOB_ODR_Addr (GPIOB_BASE + 12) // 0x40010C0C
#define GPIOC_ODR_Addr (GPIOC_BASE + 12) // 0x4001100C
#define GPIOD_ODR_Addr (GPIOD_BASE + 12) // 0x4001140C
#define GPIOE_ODR_Addr (GPIOE_BASE + 12) // 0x4001180C
#define GPIOF_ODR_Addr (GPIOF_BASE + 12) // 0x40011A0C
#define GPIOG_ODR_Addr (GPIOG_BASE + 12) // 0x40011E0C

#define GPIOA_IDR_Addr (GPIOA_BASE + 8) // 0x40010808
#define GPIOB_IDR_Addr (GPIOB_BASE + 8) // 0x40010C08
#define GPIOC_IDR_Addr (GPIOC_BASE + 8) // 0x40011008
#define GPIOD_IDR_Addr (GPIOD_BASE + 8) // 0x40011408
#define GPIOE_IDR_Addr (GPIOE_BASE + 8) // 0x40011808
#define GPIOF_IDR_Addr (GPIOF_BASE + 8) // 0x40011A08
#define GPIOG_IDR_Addr (GPIOG_BASE + 8) // 0x40011E08

// IO�ڲ���,ֻ�Ե�һ��IO��!
// ȷ��n��ֵС��16!
#define PAout(n) BIT_ADDR(GPIOA_ODR_Addr, n) // ���
#define PAin(n) BIT_ADDR(GPIOA_IDR_Addr, n)  // ����

#define PBout(n) BIT_ADDR(GPIOB_ODR_Addr, n) // ���
#define PBin(n) BIT_ADDR(GPIOB_IDR_Addr, n)  // ����

#define PCout(n) BIT_ADDR(GPIOC_ODR_Addr, n) // ���
#define PCin(n) BIT_ADDR(GPIOC_IDR_Addr, n)  // ����

#define PDout(n) BIT_ADDR(GPIOD_ODR_Addr, n) // ���
#define PDin(n) BIT_ADDR(GPIOD_IDR_Addr, n)  // ����

#define PEout(n) BIT_ADDR(GPIOE_ODR_Addr, n) // ���
#define PEin(n) BIT_ADDR(GPIOE_IDR_Addr, n)  // ����

#define PFout(n) BIT_ADDR(GPIOF_ODR_Addr, n) // ���
#define PFin(n) BIT_ADDR(GPIOF_IDR_Addr, n)  // ����

#define PGout(n) BIT_ADDR(GPIOG_ODR_Addr, n) // ���
#define PGin(n) BIT_ADDR(GPIOG_IDR_Addr, n)  // ����

void JTAG_Set(uint8_t mode);

// JTAGģʽ���ö���
#define JTAG_SWD_DISABLE 0X02
#define SWD_ENABLE 0X01
#define JTAG_SWD_ENABLE 0X00

/* ֱ�Ӳ����Ĵ����ķ�������IO */
#define digitalHi(p, i) \
    {                   \
        p->BSRR = i;    \
    } // ���Ϊ�ߵ�ƽ
#define digitalLo(p, i) \
    {                   \
        p->BRR = i;     \
    } // ����͵�ƽ
#define digitalToggle(p, i) \
    {                       \
        p->ODR ^= i;        \
    } // �����ת״̬
#define Lidar_Detect_ON 1 // ���Ѳ���Ƿ����״����ϰ���
#define Lidar_Detect_OFF 0

extern uint8_t Ros_Rate;
extern volatile uint8_t Ros_count;
extern uint8_t Pick_up_stop; // ����Ƿ������־λ
extern int Middle_angle;     // ��е��ֵĬ��Ϊ0
extern uint8_t Lidar_Detect;
extern uint8_t Mode;                                                                          // ģʽѡ��Ĭ������ͨ�Ŀ���ģʽ
extern uint8_t PS2_ON_Flag;                                                                   // Ĭ�����з�ʽ������
extern float RC_Velocity, RC_Turn_Velocity;                                                   // ң�ؿ��Ƶ��ٶ�
extern uint8_t Way_Angle;                                                                     // ��ȡ�Ƕȵ��㷨��1����Ԫ��  2��������  3�������˲�
extern int Motor_Left, Motor_Right;                                                           // ���PWM���� Ӧ��motor�� ��moto�¾�
extern uint16_t Flag_front, Flag_back, Flag_Left, Flag_Right, Flag_velocity, Target_Velocity; // ����ң����صı���
extern uint8_t Flag_Stop, Flag_Show;                                                          // ֹͣ��־λ�� ��ʾ��־λ Ĭ��ֹͣ ��ʾ��
extern int Voltage;                                                                           // ��ص�ѹ������صı���
extern float Angle_Balance, Gyro_Balance, Gyro_Turn;                                          // ƽ����� ƽ�������� ת��������
extern int Temperature;
extern uint32_t Distance;               // �״���
extern uint16_t determine;              // ȷ����ֱ�ߵľ���ֵ
extern int Encoder_Left, Encoder_Right; // ���ұ��������������
extern float Move_X, Move_Z;
extern uint8_t Flag_follow, Flag_avoid, Flag_straight, delay_50, PID_Send;
extern volatile uint8_t delay_flag;
extern float Acceleration_Z; // Z����ٶȼ�
extern float Balance_Kp, Balance_Kd, Velocity_Kp, Velocity_Ki, Turn_Kp, Turn_Kd;
extern float Distance_KP, Distance_KD, Distance_KI; // �������PID����
extern uint8_t CCD_Zhongzhi, CCD_Yuzhi;             // ����CCD
extern uint16_t Angle_ADC;

// ����Ϊ��ຯ��
void WFI_SET(void);          // ִ��WFIָ��
void INTX_DISABLE(void);     // �ر������ж�
void INTX_ENABLE(void);      // ���������ж�
void MSR_MSP(uint32_t addr); // ���ö�ջ��ַ

#include "KF.h"
#include "filter.h"
#include "IOI2C.h"
#include "Lidar.h"
#include "usart3.h"
#include "DataScope_DP.h"
#include "pstwo.h"
#include "adc.h"
#include "motor.h"
#include "encoder.h"
#include "mpu6050.h"
#include "inv_mpu.h"
#include "inv_mpu_dmp_motion_driver.h"
#include "dmpKey.h"
#include "dmpmap.h"
#include "oled.h"
#include "show.h"
#include "tim.h"
#include "usart.h"
#include "beep.h"
#include "control.h"
#include "key.h"
#include "led.h"
#include "delay.h"
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#endif
