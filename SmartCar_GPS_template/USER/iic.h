#include "stm32f10x.h"
#ifndef   _IIC_H
#define   _IIC_H

//IO��������
#define MPU_SDA_IN()  {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=8<<12;}//B11
#define MPU_SDA_OUT() {GPIOB->CRH&=0XFFFF0FFF;GPIOB->CRH|=3<<12;}//B11

//IO��������	 
#define MPU_IIC_SCL    PBout(10) 		//SCL  B10
#define MPU_IIC_SDA    PBout(11) 		//SDA	 B11
#define MPU_READ_SDA   PBin(11) 		//����SDA 

//IIC���в�������
void MPU_IIC_Delay(void);				//MPU IIC��ʱ����
void MPU_IIC_Init(void);                //��ʼ��IIC��IO��				 
void MPU_IIC_Start(void);				//����IIC��ʼ�ź�
void MPU_IIC_Stop(void);	  			//����IICֹͣ�ź�
void MPU_IIC_Send_Byte(u8 dat);			//IIC����һ���ֽ�
u8 MPU_IIC_Read_Byte(u8 ack);//IIC��ȡһ���ֽ�
u8 MPU_IIC_Wait_Ack(void); 				//IIC�ȴ�ACK�ź�
void MPU_IIC_Ack(void);
void MPU_IIC_NAck(void);
#endif


