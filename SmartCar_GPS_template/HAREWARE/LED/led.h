#ifndef __LED_H
#define __LED_H
#include "sys.h"

void LED_Init(void);
void LED_Toggle(GPIO_TypeDef *GPIOx,uint16_t GPIO_Pin);

#endif


