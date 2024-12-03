#ifndef __KEY_H
#define __KEY_H
#include "sys.h"

// ^异或，C语言的一个二进制的运算符
//	与1异或改变，与0异或不变
//实现按下亮，不按熄灭
#define LED_G_TOGGLE {LED_G_GPIO_PORT->ODR ^= LED_G_GPIO_PIN;}


void KEY_GPIO_Config(void);
uint8_t Key_Scan(GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin);

#endif
