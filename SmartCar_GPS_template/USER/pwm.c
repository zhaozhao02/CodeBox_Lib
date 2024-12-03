#include "control.h"

//-------------------------------------------------------------------------------------------------------------------
// @brief       TIM1 PWM���ֳ�ʼ�� 
// @param       arr �Զ���װֵ
// @param       psc Ԥ��Ƶϵ��
// @return      void
// Sample usage:    PWM��ʼ��
//-------------------------------------------------------------------------------------------------------------------
void TIM1_PWM_Init(uint16_t arr,uint16_t psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);	//ʹ�ܶ�ʱ��3ʱ��
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //ʹ��GPIO�����AFIO���ù���ģ��ʱ��
	//RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);        //IO ����

    //GPIO_PinRemapConfig(GPIO_PartialRemap_TIM1, ENABLE);  //TIM1�Ĳ�����ӳ�䣬����䣬���޸�IO����    
    //GPIO_PinRemapConfig(GPIO_FullRemap_TIM1, ENABLE);     //TIM1����ȫ��ӳ�䣬����䣬���޸�IO����
    
    //û����ӳ��ʱ��TIM1���ĸ�ͨ��CH1��CH2��CH3��CH4�ֱ��Ӧ   PA8     PA9     PA10    PA11
    //������ӳ��ʱ��TIM1���ĸ�ͨ��CH1��CH2��CH3��CH4�ֱ��Ӧ   PA8     PA9     PA10    PA11
    //��ȫ��ӳ��ʱ��TIM1���ĸ�ͨ��CH1��CH2��CH3��CH4�ֱ��Ӧ   PE7     PE9     PE11    PE13
    
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8|GPIO_Pin_9|GPIO_Pin_10|GPIO_Pin_11; 
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIO
 
  //��ʼ��TIM1
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
	
  TIM_OCInitStructure.TIM_Pulse = 0; 
    
	//��ʼ��TIM3 Channel2 PWMģʽ	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ2 
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //�������:TIM����Ƚϼ��Ը�
	TIM_OC1Init(TIM1, &TIM_OCInitStructure);
	TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable); 
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);  //����Tָ���Ĳ�����ʼ������
	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);  //ʹ��TIM3��CCR2�ϵ�Ԥװ�ؼĴ���
	TIM_OC3Init(TIM1, &TIM_OCInitStructure);
	TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable); 
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);    
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);
	TIM_Cmd(TIM1, ENABLE);  //ʹ��TIM3
  TIM_CtrlPWMOutputs(TIM1, ENABLE);                             //�߼���ʱ������ʹ�� pwm���
	TIM_SetCompare1(TIM1,0);        //ռ�ձ� = 0 / TIM_Period * 100%  
  TIM_SetCompare2(TIM1,0);
	TIM_SetCompare3(TIM1,0);
  TIM_SetCompare4(TIM1,0);
}
//-------------------------------------------------------------------------------------------------------------------
// @brief       TIM2 PWM���ֳ�ʼ�� 
// @param       arr �Զ���װֵ
// @param       psc Ԥ��Ƶϵ��
// @return      void
// Sample usage:    PWM��ʼ��
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
  GPIO_PinRemapConfig(GPIO_Remap_SWJ_JTAGDisable , ENABLE);//��ֹJTAG���ܣ���PB3��PB4��Ϊ��ͨIO��ʹ��
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
  TIM_SetCompare2(TIM2,SERVO_MID);					//�����ֵ1570
}
//-------------------------------------------------------------------------------------------------------------------
// @brief       TIM4 PWM���ֳ�ʼ�� 
// @param       arr �Զ���װֵ
// @param       psc Ԥ��Ƶϵ��
// @return      void
// Sample usage:    PWM��ʼ��
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
  TIM_SetCompare1(TIM4,0);					//�����ͣ
	TIM_SetCompare3(TIM4,0);					//�����ͣ
}

