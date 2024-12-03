#include "control.h"

uint8_t gps_flag=0; 							//GPS解算标志位		
nmea_msg gpsx; 									//GPS信息

//-------------------------------------------------------------------------------------------------------------------
// @brief       主初始化
// @param		
// @return      void
// Sample usage:    
//-------------------------------------------------------------------------------------------------------------------
void Contorl_Init()
{
	SysTick_Init();									//delay初始化
	uart_init(115200);							//调试串口初始化
	printf("Hello World!");
	LED_Init();											//LED初始化
	TIM2_PWM_Init(20000-1,72-1);		//舵机PWM初始化 50Hz 
	TIM4_PWM_Init(SPEED-1,72-1);		//电机PWM Hz 电机频率通过更改SPEED 这个宏的值来更改 这个具体大家按照自己的接线来改
	KEY_GPIO_Config();							//按键读取初始化
	//LCD_Init();								//LCD初始化 			如果买了TFT1.8的屏幕的话可以试试看 
	//LCD_Fill(0,0,LCD_W,LCD_H,WHITE);			//屏幕上色
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	USART2_Init(9600);							//GPS串口初始化
	////////////////////////////////////////////////////
	//这里写GPS采集点的函数
	////////////////////////////////////////////////////
	delay_ms(1000);
	MPU_Init();												//陀螺仪初始化
	delay_ms(500);
	Gyroscope_Offset_Init();					//零漂初始化
	delay_ms(500);
	TIM3_Init(5000-1,72-1);						//陀螺仪解算 5ms定时器
}
//-------------------------------------------------------------------------------------------------------------------
// @brief       主控制
// @param		
// @return      void
// Sample usage:    5ms执行一次
//-------------------------------------------------------------------------------------------------------------------
void Contorl()
{
	static uint8_t cnt_10ms=0;
	static uint8_t cnt_500ms=0;
	cnt_10ms++;
	cnt_500ms++;	
	Data_Send(eulerAngle.roll,eulerAngle.pitch,eulerAngle.yaw,0,0,0,0,0);//向上位机发送数据 这个是数据示波器那个
	ANO_DT_Send_Status(eulerAngle.roll,eulerAngle.pitch,eulerAngle.yaw,0,0,0);//向上位机发送姿态 这个是3D小飞机那个
	if(cnt_10ms>=2)
	{
		if(gps_flag==1)				//定位有效 
		{
			//GPS循迹
			gps_flag=0;
		}
		//舵机控制		//记得加限幅//改变pwm占空比控制舵机
		Speed_Control(0);//速度控制 这个地方参数0没意义 
		cnt_10ms=0;
	}
	if(cnt_500ms>=100)				
	{		
//		printf("lat: %f \r\n",(float)gpsx.latitude/100000.0);		//500ms发送一次状态	
//		printf("lon: %f \r\n",(float)gpsx.longitude/100000.0);		//经纬度
//		printf("tim: %d : %d\r\n",gpsx.utc.min,gpsx.utc.sec);		//时间
//		printf("yaw: %f \r\n",eulerAngle.yaw);					//车身方位角			
		cnt_500ms=0;
	}
}
//-------------------------------------------------------------------------------------------------------------------
// @brief       按键采点
// @param		
// @return      void
// Sample usage:    
//-------------------------------------------------------------------------------------------------------------------
void get_point()
{
	while(1)
	{
		if(Key_Scan(GPIOB,GPIO_Pin_14)==1)		//按下采点键
		{
			//在这里采集点
			delay_ms(500);//按键消抖
		}
		if(Key_Scan(GPIOB,GPIO_Pin_13)==1 )		//按下结束踩点的按键
		{
			break;
		}
	}
}
//-------------------------------------------------------------------------------------------------------------------
// @brief       速度控制
// @param		int speed_set 设定速度
// @return      void
// Sample usage:    按键调整速度
//-------------------------------------------------------------------------------------------------------------------
void Speed_Control(int speed_set)
{
	static int speed=0;
	if(Key_Scan(GPIOB,GPIO_Pin_12)==1)
	{
		LED_Toggle(GPIOC,GPIO_Pin_13);
		if(speed<=SPEED)
			speed+=SPEED/10;			//按键加速 一次加10%的占空比
	}
	if(Key_Scan(GPIOB,GPIO_Pin_13)==1)
	{
		LED_Toggle(GPIOC,GPIO_Pin_13);
		if(speed>=SPEED/10)
			speed-=SPEED/10;			//按键减速 一次加10%的占空比
	}
	TIM_SetCompare1(TIM4,speed);			
	TIM_SetCompare3(TIM4,speed);

}
