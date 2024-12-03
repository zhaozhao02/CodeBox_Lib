#include "control.h"

//-------------------------------------------------------------------------------------------------------------------
// @brief       TIM1 PWM部分初始化 
// @param       arr 自动重装值
// @param       psc 预分频系数
// @return      void
// Sample usage:    PWM初始化
//-------------------------------------------------------------------------------------------------------------------
void TIM1_PWM_Init(uint16_t arr,uint16_t psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);	//使能定时器3时钟
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //使能GPIO外设和AFIO复用功能模块时钟
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);        //IO 复用

    //GPIO_PinRemapConfig(GPIO_PartialRemap_TIM1, ENABLE);  //TIM1的部分重映射，加这句，并修改IO配置    
    //GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE);     //TIM1的完全重映射，加这句，并修改IO配置
    
    //没有重映射时，TIM1的四个通道CH1，CH2，CH3，CH4分别对应   PA8     PA9     PA10    PA11
    //部分重映射时，TIM1的四个通道CH1，CH2，CH3，CH4分别对应   PA8     PA9     PA10    PA11
    //完全重映射时，TIM1的四个通道CH1，CH2，CH3，CH4分别对应   PE7     PE9     PE11    PE13
    
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIO
 
  //初始化TIM1
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	
  TIM_OCInitStructure.TIM_Pulse = 0; 
    
	//初始化TIM3 Channel2 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //选择定时器模式:TIM脉冲宽度调制模式2 
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable); 
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);  //根据T指定的参数初始化外设
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);  //使能TIM3在CCR2上的预装载寄存器
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable); 
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);    
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_Cmd(TIM1, ENABLE);  //使能TIM3
  TIM_CtrlPWMOutputs(TIM1, ENABLE);                             //高级定时器必须使能 pwm输出
	TIM_SetCompare1(TIM1,0);        //占空比 = 0 / TIM_Period * 100%  
  TIM_SetCompare2(TIM1,0);
	TIM_SetCompare3(TIM1,0);
  TIM_SetCompare4(TIM1,0);
}
//-------------------------------------------------------------------------------------------------------------------
// @brief       TIM2 PWM部分初始化 
// @param       arr 自动重装值
// @param       psc 预分频系数
// @return      void
// Sample usage:    PWM初始化
//-------------------------------------------------------------------------------------------------------------------
void TIM2_PWM_Init(uint16_t arr,uint16_t psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM2, ENABLE);
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);//禁止JTAG功能，把PB3，PB4作为普通IO口使用
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 ;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  TIM_TimeBaseInitStructure.TIM_Period = arr;//Hz
  TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
  TIM_TimeBaseInitStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM2, & TIM_TimeBaseInitStructure);
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;    
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OCInitStructure.TIM_Pulse = 140;		
  TIM_OC2Init(TIM2, &TIM_OCInitStructure);
  TIM_OC2PreloadConfig(TIM2, TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(TIM2, ENABLE);
  TIM_ITConfig(TIM2, TIM_IT_CC1, ENABLE);
  TIM_CtrlPWMOutputs(TIM2, ENABLE);
	TIM_Cmd(TIM2, ENABLE);
  TIM_SetCompare2(TIM2,SERVO_MID);					//舵机中值1570
}
//-------------------------------------------------------------------------------------------------------------------
// @brief       TIM4 PWM部分初始化 
// @param       arr 自动重装值
// @param       psc 预分频系数
// @return      void
// Sample usage:    PWM初始化
//-------------------------------------------------------------------------------------------------------------------
void TIM4_PWM_Init(uint16_t arr,uint16_t psc)
{  
  GPIO_InitTypeDef GPIO_InitStructure; 
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
  TIM_OCInitTypeDef TIM_OCInitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_8 ;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  TIM_TimeBaseInitStructure.TIM_Period = arr;//Hz
  TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
  TIM_TimeBaseInitStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, & TIM_TimeBaseInitStructure);
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;    
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; 
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OCInitStructure.TIM_Pulse = 500;  
  TIM_OC1Init(TIM4, &TIM_OCInitStructure);
  TIM_OC1PreloadConfig(TIM4, TIM_OCPreload_Enable);
  TIM_OCInitStructure.TIM_Pulse = 0;	
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);
  TIM_OC3PreloadConfig(TIM4, TIM_OCPreload_Enable);
  TIM_ARRPreloadConfig(TIM4, ENABLE);
  TIM_ITConfig(TIM4, TIM_IT_CC1, ENABLE);
  TIM_CtrlPWMOutputs(TIM4, ENABLE);
  TIM_Cmd(TIM4, ENABLE);
  TIM_SetCompare1(TIM4,0);					//电机关停
	TIM_SetCompare3(TIM4,0);					//电机关停
}

