#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "si5351a.h"
 


int main(void)
{
	int Fre_1M = 10000;
	int fre = 50;
	delay_init();	    //延时函数初始化	  
	
	IIC_SI5351A_GPIO_Init();
	
	si5351aSetFrequency(1*Fre_1M , 0);
	si5351aSetFrequency(2*Fre_1M , 1);
	si5351aSetFrequency(3*Fre_1M , 2);
	
	while(1)
	{
		si5351aSetFrequency(fre*Fre_1M,2);
		delay_ms(50);	 //延时
		if(fre++>=15000) fre=50;
	}
}



