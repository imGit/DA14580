#include "soft_delay.h"
/*
�������ܣ���ʱdelay_ms������
��������ʱʱ�䣬��λ����
����ֵ����
*/
void delay_ms(unsigned int delay)
{
	unsigned int i = 80*delay;
	while(i--)
	{

  }
}
/*
�������ܣ���ʱdelay_us������
��������ʱʱ�䣬��λ΢��
����ֵ����
*/
void Delay_us(unsigned int delay_us)
{
	unsigned j = 18;
	for(delay_us; delay_us > 0; delay_us--){
		while(j--);
		j = 18;
	}
}