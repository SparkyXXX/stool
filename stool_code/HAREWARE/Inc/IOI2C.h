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
#include "sys.h"
#ifndef __IOI2C_H
#define __IOI2C_H

//IO方向设置
#define SDA_IN()  {GPIOB->CRH&=0X0FFFFFFF;GPIOB->CRH|=(uint32_t)8<<28;}
#define SDA_OUT() {GPIOB->CRH&=0X0FFFFFFF;GPIOB->CRH|=(uint32_t)3<<28;}

//IO操作函数
#define IIC_SCL    PBout(14) //SCL
#define IIC_SDA    PBout(15) //SDA
#define READ_SDA   PBin(15)  //输入SDA

//IIC所有操作函数
int IIC_Start(void);					 //发送IIC开始信号
void IIC_Stop(void);	  			 //发送IIC停止信号
void IIC_Send_Byte(uint8_t txd);		 //IIC发送一个字节
uint8_t IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
int IIC_Wait_Ack(void); 			 //IIC等待ACK信号
void IIC_Ack(void);						 //IIC发送ACK信号
void IIC_NAck(void);					 //IIC不发送ACK信号

void IIC_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data);
uint8_t IIC_Read_One_Byte(uint8_t daddr,uint8_t addr);
unsigned char I2C_Readkey(unsigned char I2C_Addr);

unsigned char I2C_ReadOneByte(unsigned char I2C_Addr,unsigned char addr);
unsigned char IICwriteByte(unsigned char dev, unsigned char reg, unsigned char data);
uint8_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t* data);
uint8_t IICwriteBits(uint8_t dev,uint8_t reg,uint8_t bitStart,uint8_t length,uint8_t data);
uint8_t IICwriteBit(uint8_t dev,uint8_t reg,uint8_t bitNum,uint8_t data);
uint8_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data);

int i2cWrite(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *data);
int i2cRead(uint8_t addr, uint8_t reg, uint8_t len, uint8_t *buf);

#endif

//------------------End of File----------------------------
