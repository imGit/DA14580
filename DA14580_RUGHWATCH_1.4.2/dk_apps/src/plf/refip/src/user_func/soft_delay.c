#include "soft_delay.h"
/*
函数功能：延时delay_ms个毫秒
参数：延时时间，单位毫秒
返回值：无
*/
void delay_ms(unsigned int delay)
{
	unsigned int i = 80*delay;
	while(i--)
	{

  }
}
/*
函数功能：延时delay_us个毫秒
参数：延时时间，单位微妙
返回值：无
*/
void Delay_us(unsigned int delay_us)
{
	unsigned j = 18;
	for(delay_us; delay_us > 0; delay_us--){
		while(j--);
		j = 18;
	}
}