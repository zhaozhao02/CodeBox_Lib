#ifndef __KEY_H
#define __KEY_H
#include "sys.h"

// ^���C���Ե�һ�������Ƶ������
//	��1���ı䣬��0��򲻱�
//ʵ�ְ�����������Ϩ��
#define LED_G_TOGGLE {LED_G_GPIO_PORT->ODR ^= LED_G_GPIO_PIN;}


void KEY_GPIO_Config(void);
uint8_t Key_Scan(GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin);

#endif
