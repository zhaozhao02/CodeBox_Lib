#include "delay.h"
#include "usart2.h"
#include "stdarg.h"	 	 
#include "stdio.h"	 	 
#include "string.h"
#include "control.h"	   
//串口发送缓存区 	
__align(8) u8 USART2_TX_BUF[USART2_MAX_SEND_LEN]; 	//发送缓冲,最大USART2_MAX_SEND_LEN字节 
//串口接收缓存区 	
u8 USART2_RX_BUF[USART2_MAX_RECV_LEN]; 				//接收缓冲,最大USART2_MAX_RECV_LEN个字节.
//采用串口空闲中断的方式完成数据接收，进入串口空闲中断，则表示此次接收完毕.
//USART2_MAX_TX_LEN和USART2_MAX_RX_LEN在头文件进行了宏定义，分别指USART2最大发送长度和最大接收长度
#define USART2_MAX_RX_LEN 512
#define USART2_MAX_TX_LEN 512
u8 u1rxbuf[USART2_MAX_RX_LEN];			//发送数据缓冲区1
u8 u2rxbuf[USART2_MAX_RX_LEN];			//发送数据缓冲区2
u8 witchbuf=0;                  		//标记当前使用的是哪个缓冲区,0：使用u1rxbuf；1：使用u2rxbuf
u8 USART2_TX_FLAG=0;					//USART2发送标志，启动发送时置1
u8 USART2_RX_FLAG=0;					//USART2接收标志，启动接收时置1
u16 USART2_RX_STA=0;  

//接收到的数据状态
//[15]:0,没有接收到数据;1,接收到了一批数据.
//[14:0]:接收到的数据长度
// void USART2_IRQHandler(void)
// {
// 	u8 res;	    
// 	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET)//接收到数据，进入接收中断
// 	{	 
// 		res =USART_ReceiveData(USART2);		
// 		if(USART2_RX_STA<USART2_MAX_RECV_LEN)		//还可以接收数据
// 		{
// 			USART2_RX_BUF[USART2_RX_STA++]=res;		//记录接收到的值	 
// 		}else 
// 		{
// 			USART2_RX_STA|=1<<15;					//强制标记接收完成
// 		} 
// 	}
// 	else if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)//空闲中断，接收结束
// 	{
// 		USART2_RX_STA|=1<<15;	//标记接收完成
// 	}
// }   

//串口2中断函数
void USART2_IRQHandler(void)                	
{
	u8 *p;
	//u8 USART2_RX_LEN = 0;											//接收数据长度
	if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)			//串口2空闲中断
	{
		USART_ReceiveData(USART2); 									//清除串口2空闲中断IDLE标志位
		USART_ClearFlag(USART2,USART_FLAG_TC);						//清除USART2标志位
		DMA_Cmd(DMA1_Channel6, DISABLE );   						//关闭USART2 TX DMA1 所指示的通道
	//	USART2_RX_LEN = USART2_MAX_RX_LEN - DMA1_Channel6->CNDTR;	//获得接收到的字节数  不定长数据
		if(witchbuf)                        						//之前用的u2rxbuf，切换为u1rxbuf
		{
			p=u2rxbuf;												//先保存前一次数据地址再切换缓冲区
			DMA1_Channel6->CMAR=(u32)u1rxbuf;						//切换为u1rxbuf缓冲区地址
			witchbuf=0;                     						//下一次切换为u2rxbuf
		}else                               						//之前用的u1rxbuf，切换为u2rxbuf
		{
			p=u1rxbuf;												//先保存前一次数据地址再切换缓冲区
			DMA1_Channel6->CMAR=(u32)u2rxbuf;						//切换为u2rxbuf缓冲区地址
			witchbuf=1;                     						//下一次切换为u1rxbuf
		}
		DMA1_Channel6->CNDTR = USART2_MAX_RX_LEN;					//DMA通道的DMA缓存的大小
		DMA_Cmd(DMA1_Channel6, ENABLE);     						//使能USART2 TX DMA1 所指示的通道
		
		//******************↓↓↓↓↓这里作数据处理↓↓↓↓↓******************//
		
		//DMA_USART2_Tx_Data(p,USART2_RX_LEN);

		GPS_Analysis(&gpsx,p);			//分析字符串
		gps_flag=1;						//标志位置1
		
		//printf("tim: %d : %d\r\n",gpsx.utc.min,gpsx.utc.sec);	
		
		//******************↑↑↑↑↑这里作数据处理↑↑↑↑↑******************//
		
  	}
}


//初始化IO 串口2
//pclk1:PCLK1时钟频率(Mhz)
//bound:波特率	  
void USART2_Init(u32 bound)
{  

	NVIC_InitTypeDef NVIC_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	// GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);

 	USART_DeInit(USART2);  //复位串口2
	//USART2_TX   PA.2
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2; //PA.2
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
	GPIO_Init(GPIOA, &GPIO_InitStructure); //初始化PA2
    //USART2_RX	  PA.3
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
	GPIO_Init(GPIOA, &GPIO_InitStructure);  //初始化PA3
	
	USART_InitStructure.USART_BaudRate = bound;//GPS波特率为115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(USART2, &USART_InitStructure); //初始化串口	2
  
	//USART_DMACmd(USART2,USART_DMAReq_Tx,ENABLE);  	//使能串口2的DMA发送
	//UART_DMA_Config(DMA1_Channel7,(u32)&USART2->DR,(u32)USART2_TX_BUF);//DMA1通道7,外设为串口2,存储器为USART2_TX_BUF 
	//USART_Cmd(USART2, ENABLE);                    //使能串口 
	
	
	#ifdef USART2_RX_EN		  	//如果使能了接收
	//使能接收中断
	//中断开启设置
	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);									//开启检测串口空闲状态中断
	USART_ClearFlag(USART2,USART_FLAG_TC);	
	USART_Cmd(USART2, ENABLE);	

	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	DMA1_USART2_Init();	
	//USART2_RX_STA=0;		//清零
	#endif	 	

}

void DMA1_USART2_Init(void)
{
	DMA_InitTypeDef DMA1_Init;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1,ENABLE);						//使能DMA1时钟

	//DMA_USART2_RX  USART2->RAM的数据传输
	DMA_DeInit(DMA1_Channel6);												//将DMA的通道6寄存器重设为缺省值 
	DMA1_Init.DMA_PeripheralBaseAddr = (u32)(&USART2->DR);					//启动传输前装入实际RAM地址
	DMA1_Init.DMA_MemoryBaseAddr = (u32)u1rxbuf;            				//设置接收缓冲区首地址
	DMA1_Init.DMA_DIR = DMA_DIR_PeripheralSRC;								//数据传输方向，从外设读取到内存
	DMA1_Init.DMA_BufferSize = USART2_MAX_RX_LEN;							//DMA通道的DMA缓存的大小
	DMA1_Init.DMA_PeripheralInc = DMA_PeripheralInc_Disable;				//外设地址寄存器不变
	DMA1_Init.DMA_MemoryInc = DMA_MemoryInc_Enable;							//内存地址寄存器递增
	DMA1_Init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;			//数据宽度为8位
	DMA1_Init.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;					//数据宽度为8位
	DMA1_Init.DMA_Mode = DMA_Mode_Normal;									//工作在正常模式
	DMA1_Init.DMA_Priority = DMA_Priority_High; 							//DMA通道 x拥有高优先级 
	DMA1_Init.DMA_M2M = DMA_M2M_Disable;									//DMA通道x没有设置为内存到内存传输
	 
	DMA_Init(DMA1_Channel6,&DMA1_Init); 									//对DMA通道6进行初始化
	
	//DMA_USART2_TX  RAM->USART2的数据传输
	DMA_DeInit(DMA1_Channel7);												//将DMA的通道7寄存器重设为缺省值 
	DMA1_Init.DMA_PeripheralBaseAddr = (u32)(&USART2->DR);					//启动传输前装入实际RAM地址
	DMA1_Init.DMA_MemoryBaseAddr = (u32)USART2_TX_BUF;              		//设置发送缓冲区首地址
	DMA1_Init.DMA_DIR = DMA_DIR_PeripheralDST; 								//数据传输方向，从内存发送到外设
	DMA1_Init.DMA_BufferSize = USART2_MAX_TX_LEN;							//DMA通道的DMA缓存的大小
	DMA1_Init.DMA_PeripheralInc = DMA_PeripheralInc_Disable;				//外设地址寄存器不变
	DMA1_Init.DMA_MemoryInc = DMA_MemoryInc_Enable;							//内存地址寄存器递增
	DMA1_Init.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;			//数据宽度为8位
	DMA1_Init.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;					//数据宽度为8位
	DMA1_Init.DMA_Mode = DMA_Mode_Normal;									//工作在正常模式
	DMA1_Init.DMA_Priority = DMA_Priority_High; 							//DMA通道 x拥有高优先级 
	DMA1_Init.DMA_M2M = DMA_M2M_Disable;									//DMA通道x没有设置为内存到内存传输

	DMA_Init(DMA1_Channel7,&DMA1_Init); 									//对DMA通道7进行初始化
	
	//DMA1通道6 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel6_IRQn;				//NVIC通道设置
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1 ;				//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;						//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;							//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);											//根据指定的参数初始化NVIC寄存器
 
	//DMA1通道7 NVIC 配置
	NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel7_IRQn;				//NVIC通道设置
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1 ;				//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;						//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;							//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);											//根据指定的参数初始化NVIC寄存器

	DMA_ITConfig(DMA1_Channel6,DMA_IT_TC,ENABLE);							//开USART2 Rx DMA中断
	DMA_ITConfig(DMA1_Channel7,DMA_IT_TC,ENABLE);							//开USART2 Tx DMA中断

	DMA_Cmd(DMA1_Channel6,ENABLE);           								//使DMA通道6停止工作
	DMA_Cmd(DMA1_Channel7,DISABLE);           								//使DMA通道7停止工作
	 
	USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);        					//开启串口DMA发送
	USART_DMACmd(USART2, USART_DMAReq_Rx, ENABLE);        					//开启串口DMA接收
}

