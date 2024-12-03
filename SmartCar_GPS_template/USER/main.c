#include "control.h"

int main(void)
{
	Contorl_Init();
	while(1)
	{
		if(solve_flag==1)
		{
			Contorl();   
			solve_flag=0;//标志变量置0
		}
	}
}
