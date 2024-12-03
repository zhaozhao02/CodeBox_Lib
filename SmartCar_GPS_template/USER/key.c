#include "key.h"
#include "delay.h"
//-------------------------------------------------------------------------------------------------------------------
// @brief       按键GPIO初始化
// @param       
// @return      
// Sample usage:    
//-------------------------------------------------------------------------------------------------------------------
void KEY_GPIO_Config(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_12 |GPIO_Pin_13 |GPIO_Pin_14;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
}
//-------------------------------------------------------------------------------------------------------------------
// @brief       按键检测函数
// @param       GPIO_TypeDef *GPIOx
// @param       uint16_t GPIO_Pin
// @return      uint8_t 按键状态 按下为1 未按下为0
// Sample usage:    
//-------------------------------------------------------------------------------------------------------------------
uint8_t Key_Scan(GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin)
{
	if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin) == Bit_RESET)//0
	{
		delay_ms(10);				//消抖
		if(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin) == Bit_RESET)//0
		{
			while(GPIO_ReadInputDataBit(GPIOB, GPIO_Pin) == Bit_RESET);//1
		}
		delay_ms(10);
		return 1;
	}
	else return 0;
}

