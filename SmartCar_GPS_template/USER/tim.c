#include "tim.h"
//-------------------------------------------------------------------------------------------------------------------
// @brief       TIM1 定时器中断初始化 
// @param       arr：自动重装值
// @param       psc：时钟预分频数
// @return      void
// Sample usage:    
//-------------------------------------------------------------------------------------------------------------------
void TIM1_Init(u16 per,u16 psc)
{
  TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE); //时钟使能
  TIM_TimeBaseStructure.TIM_Period = per; //设置自动重装载寄存器周期值
  TIM_TimeBaseStructure.TIM_Prescaler =(psc-1);//设置预分频值
  TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;//向上计数模式
  TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;//重复计数设置
  TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //参数初始化
  TIM_ClearFlag(TIM1, TIM_FLAG_Update);//清中断标志位
  TIM_ITConfig(      //使能或者失能指定的TIM中断
    TIM1,            //TIM1
    TIM_IT_Update  | //TIM 更新中断源
    TIM_IT_Trigger,  //TIM 触发中断源 
    ENABLE  	     //使能
    );
  //设置优先级
  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;  
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;//先占优先级0级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  	   //从优先级0级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure); 
  TIM_Cmd(TIM1, ENABLE);  //使能TIMx外设
}
//-------------------------------------------------------------------------------------------------------------------
// @brief       TIM3 定时器中断初始化 
// @param       arr：自动重装值
// @param       psc：时钟预分频数
// @return      void
// Sample usage:    
//-------------------------------------------------------------------------------------------------------------------
void TIM3_Init(u16 per,u16 psc)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);//使能TIM3时钟
	TIM_TimeBaseInitStructure.TIM_Period=per;   //自动装载值    1hz所需要的时间乘以per  就是定时器设置的时间
	TIM_TimeBaseInitStructure.TIM_Prescaler=psc; //分频系数     计算赫兹=系统最大频率分频之后 72000000/psc 的倒数  为时间单位为秒     
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1;
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //设置向上计数模式
	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //开启定时器中断
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);
	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;//定时器中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//抢占优先级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	
	TIM_Cmd(TIM3,ENABLE); //使能定时器	
}

