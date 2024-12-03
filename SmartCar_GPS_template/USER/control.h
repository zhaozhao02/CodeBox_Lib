#include "sys.h"
#include "usart.h"
#include "pwm.h"
#include "key.h"
#include "delay.h"
#include "led.h"
#include "lcd_init.h"
#include "lcd.h"
#include "gps.h"
#include "string.h"		
#include "usart2.h"
#include "iic.h"
#include "tim.h"
#include "mpu6050.h"
#include "ANO_DT.h"
#include "Gyroscope_solve.h"

#define SPEED 500
#define SERVO_MID 1570

extern uint8_t solve_flag;	                    //陀螺仪解算标志位		 
extern uint8_t contorl_flag;				    		
extern uint8_t gps_flag; 						//GPS解算标志位		 	
extern nmea_msg gpsx; 							//GPS信息

void Contorl_Init(void);
void Contorl(void);
void get_point(void);	
void Speed_Control(int speed_set);
