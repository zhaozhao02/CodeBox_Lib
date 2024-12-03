#include "led.h"
#include "stm32f10x.h"

void LED_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,ENABLE);//GPIOC
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//GPIOB
	GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin=GPIO_Pin_13;						//核心板上的LED
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
}
//-------------------------------------------------------------------------------------------------------------------
// @brief       GPIO电平翻转
// @param		GPIO_TypeDef *GPIOx
// @param		uint16_t GPIO_Pin
// @return      void
// Sample usage:   LED_Toggle(GPIOC,GPIO_Pin_13);
//-------------------------------------------------------------------------------------------------------------------
void LED_Toggle(GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin)
{
	GPIO_WriteBit(GPIOx, GPIO_Pin, (BitAction)(1 - GPIO_ReadOutputDataBit(GPIOx, GPIO_Pin)));
}

