#include "delay.h"
#include "usart2.h"
#include "stdarg.h"	 	 
#include "stdio.h"	 	 
#include "string.h"
#include "control.h"	   
//���ڷ��ͻ����� 	
__align(8) u8 USART2_TX_BUF[USART2_MAX_SEND_LEN]; 	//���ͻ���,���USART2_MAX_SEND_LEN�ֽ� 
//���ڽ��ջ����� 	
u8 USART2_RX_BUF[USART2_MAX_RECV_LEN]; 				//���ջ���,���USART2_MAX_RECV_LEN���ֽ�.
//���ô��ڿ����жϵķ�ʽ������ݽ��գ����봮�ڿ����жϣ����ʾ�˴ν������.
//USART2_MAX_TX_LEN��USART2_MAX_RX_LEN��ͷ�ļ������˺궨�壬�ֱ�ָUSART2����ͳ��Ⱥ������ճ���
#define USART2_MAX_RX_LEN 512
#define USART2_MAX_TX_LEN 512
u8 u1rxbuf[USART2_MAX_RX_LEN];			//�������ݻ�����1
u8 u2rxbuf[USART2_MAX_RX_LEN];			//�������ݻ�����2
u8 witchbuf=0;                  		//��ǵ�ǰʹ�õ����ĸ�������,0��ʹ��u1rxbuf��1��ʹ��u2rxbuf
u8 USART2_TX_FLAG=0;					//USART2���ͱ�־����������ʱ��1
u8 USART2_RX_FLAG=0;					//USART2���ձ�־����������ʱ��1
u16 USART2_RX_STA=0;  

//���յ�������״̬
//[15]:0,û�н��յ�����;1,���յ���һ������.
//[14:0]:���յ������ݳ���
// void USART2_IRQHandler(void)
// {
// 	u8 res;	    
// 	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)//���յ����ݣ���������ж�
// 	{	 
// 		res =USART_ReceiveData(USART2);		
// 		if(USART2_RX_STA<USART2_MAX_RECV_LEN)		//�����Խ�������
// 		{
// 			USART2_RX_BUF[USART2_RX_STA++]=res;		//��¼���յ���ֵ	 
// 		}else 
// 		{
// 			USART2_RX_STA|=1<<15;					//ǿ�Ʊ�ǽ������
// 		} 
// 	}
// 	else if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)//�����жϣ����ս���
// 	{
// 		USART2_RX_STA|=1<<15;	//��ǽ������
// 	}
// }   

//����2�жϺ���
void USART2_IRQHandler(void)                	
{
	u8 *p;
	//u8 USART2_RX_LEN = 0;											//�������ݳ���
	if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)			//����2�����ж�
	{
		USART_ReceiveData(USART2); 									//�������2�����ж�IDLE��־λ
		USART_ClearFlag(USART2,USART_FLAG_TC);						//���USART2��־λ
		DMA_Cmd(DMA1_Channel6, DISABLE );   						//�ر�USART2 TX DMA1 ��ָʾ��ͨ��
	//	USART2_RX_LEN = USART2_MAX_RX_LEN - DMA1_Channel6->CNDTR;	//��ý��յ����ֽ���  ����������
		if(witchbuf)                        						//֮ǰ�õ�u2rxbuf���л�Ϊu1rxbuf
		{
			p=u2rxbuf;												//�ȱ���ǰһ�����ݵ�ַ���л�������
			DMA1_Channel6->CMAR=(u32)u1rxbuf;						//�л�Ϊu1rxbuf��������ַ
			witchbuf=0;                     						//��һ���л�Ϊu2rxbuf
		}else                               						//֮ǰ�õ�u1rxbuf���л�Ϊu2rxbuf
		{
			p=u1rxbuf;												//�ȱ���ǰһ�����ݵ�ַ���л�������
			DMA1_Channel6->CMAR=(u32)u2rxbuf;						//�л�Ϊu2rxbuf��������ַ
			witchbuf=1;                     						//��һ���л�Ϊu1rxbuf
		}
		DMA1_Channel6->CNDTR = USART2_MAX_RX_LEN;					//DMAͨ����DMA����Ĵ�С
		DMA_Cmd(DMA1_Channel6, ENABLE);     						//ʹ��USART2 TX DMA1 ��ָʾ��ͨ��
		
		//******************�������������������ݴ������������******************//
		
		//DMA_USART2_Tx_Data(p,USART2_RX_LEN);

		GPS_Analysis(&gpsx,p);			//�����ַ���
		gps_flag=1;						//��־λ��1
		
		//printf("tim: %d : %d\r\n",gpsx.utc.min,gpsx.utc.sec);	
		
		//******************�������������������ݴ������������******************//
		
  	}
}


//��ʼ��IO ����2
//pclk1:PCLK1ʱ��Ƶ��(Mhz)
//bound:������	  
void USART2_Init(u32 bound)
{  

	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	// GPIOAʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);

 	USART_DeInit(USART2);  //��λ����2
	//USART2_TX   PA.2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.2
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
	GPIO_Init(GPIOA, &GPIO_InitStructure); //��ʼ��PA2
    //USART2_RX	  PA.3
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);  //��ʼ��PA3
	
	USART_InitStructure.USART_BaudRate = bound;//GPS������Ϊ115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_Init(USART2, &USART_InitStructure); //��ʼ������	2
  
	//USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);  	//ʹ�ܴ���2��DMA����
	//UART_DMA_Config(DMA1_Channel7,(u32)&USART2->DR,(u32)USART2_TX_BUF);//DMA1ͨ��7,����Ϊ����2,�洢��ΪUSART2_TX_BUF 
	//USART_Cmd(USART2, ENABLE);                    //ʹ�ܴ��� 
	
	
	#ifdef USART2_RX_EN		  	//���ʹ���˽���
	//ʹ�ܽ����ж�
	//�жϿ�������
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);									//������⴮�ڿ���״̬�ж�
	USART_ClearFlag(USART2,USART_FLAG_TC);	
	USART_Cmd(USART2, ENABLE);	

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	DMA1_USART2_Init();	
	//USART2_RX_STA=0;		//����
	#endif	 	

}

void DMA1_USART2_Init(void)
{
	DMA_InitTypeDef DMA1_Init;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);						//ʹ��DMA1ʱ��

	//DMA_USART2_RX  USART2->RAM�����ݴ���
	DMA_DeInit(DMA1_Channel6);												//��DMA��ͨ��6�Ĵ�������Ϊȱʡֵ 
	DMA1_Init.DMA_PeripheralBaseAddr = (u32)(&USART2->DR);					//��������ǰװ��ʵ��RAM��ַ
	DMA1_Init.DMA_MemoryBaseAddr = (u32)u1rxbuf;            				//���ý��ջ������׵�ַ
	DMA1_Init.DMA_DIR = DMA_DIR_PeripheralSRC;								//���ݴ��䷽�򣬴������ȡ���ڴ�
	DMA1_Init.DMA_BufferSize = USART2_MAX_RX_LEN;							//DMAͨ����DMA����Ĵ�С
	DMA1_Init.DMA_PeripheralInc = DMA_PeripheralInc_Disable;				//�����ַ�Ĵ�������
	DMA1_Init.DMA_MemoryInc = DMA_MemoryInc_Enable;							//�ڴ��ַ�Ĵ�������
	DMA1_Init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;			//���ݿ��Ϊ8λ
	DMA1_Init.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;					//���ݿ��Ϊ8λ
	DMA1_Init.DMA_Mode = DMA_Mode_Normal;									//����������ģʽ
	DMA1_Init.DMA_Priority = DMA_Priority_High; 							//DMAͨ�� xӵ�и����ȼ� 
	DMA1_Init.DMA_M2M = DMA_M2M_Disable;									//DMAͨ��xû������Ϊ�ڴ浽�ڴ洫��
	 
	DMA_Init(DMA1_Channel6,&DMA1_Init); 									//��DMAͨ��6���г�ʼ��
	
	//DMA_USART2_TX  RAM->USART2�����ݴ���
	DMA_DeInit(DMA1_Channel7);												//��DMA��ͨ��7�Ĵ�������Ϊȱʡֵ 
	DMA1_Init.DMA_PeripheralBaseAddr = (u32)(&USART2->DR);					//��������ǰװ��ʵ��RAM��ַ
	DMA1_Init.DMA_MemoryBaseAddr = (u32)USART2_TX_BUF;              		//���÷��ͻ������׵�ַ
	DMA1_Init.DMA_DIR = DMA_DIR_PeripheralDST; 								//���ݴ��䷽�򣬴��ڴ淢�͵�����
	DMA1_Init.DMA_BufferSize = USART2_MAX_TX_LEN;							//DMAͨ����DMA����Ĵ�С
	DMA1_Init.DMA_PeripheralInc = DMA_PeripheralInc_Disable;				//�����ַ�Ĵ�������
	DMA1_Init.DMA_MemoryInc = DMA_MemoryInc_Enable;							//�ڴ��ַ�Ĵ�������
	DMA1_Init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;			//���ݿ��Ϊ8λ
	DMA1_Init.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;					//���ݿ��Ϊ8λ
	DMA1_Init.DMA_Mode = DMA_Mode_Normal;									//����������ģʽ
	DMA1_Init.DMA_Priority = DMA_Priority_High; 							//DMAͨ�� xӵ�и����ȼ� 
	DMA1_Init.DMA_M2M = DMA_M2M_Disable;									//DMAͨ��xû������Ϊ�ڴ浽�ڴ洫��

	DMA_Init(DMA1_Channel7,&DMA1_Init); 									//��DMAͨ��7���г�ʼ��
	
	//DMA1ͨ��6 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel6_IRQn;				//NVICͨ������
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1 ;				//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;						//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;							//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);											//����ָ���Ĳ�����ʼ��NVIC�Ĵ���
 
	//DMA1ͨ��7 NVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;				//NVICͨ������
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1 ;				//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;						//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;							//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);											//����ָ���Ĳ�����ʼ��NVIC�Ĵ���

	DMA_ITConfig(DMA1_Channel6,DMA_IT_TC,ENABLE);							//��USART2 Rx DMA�ж�
	DMA_ITConfig(DMA1_Channel7,DMA_IT_TC,ENABLE);							//��USART2 Tx DMA�ж�

	DMA_Cmd(DMA1_Channel6,ENABLE);           								//ʹDMAͨ��6ֹͣ����
	DMA_Cmd(DMA1_Channel7,DISABLE);           								//ʹDMAͨ��7ֹͣ����
	 
	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);        					//��������DMA����
	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);        					//��������DMA����
}

