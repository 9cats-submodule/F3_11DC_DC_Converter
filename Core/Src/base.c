// base.c?��
#include "base.h"
static u8 fac_us = 0; //us��ʱ������
//��ʼ���ӳٺ���
//SYSTICK��ʱ�ӹ̶�ΪHCLKʱ�ӵ�1/8
//SYSCLK:ϵͳʱ��
void delay_init(u8 SYSCLK) {
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK); //SysTickƵ��ΪHCLK
	fac_us = SYSCLK;
}
void delay_ns(u8 t) {
	do {
		;
	} while (--t);
}
void delay_us(u32 nus) {
	u32 ticks;
	u32 told, tnow, tcnt = 0;
	u32 reload = SysTick->LOAD; //LOAD��ֵ
	ticks = nus * fac_us;       //��Ҫ�Ľ�����
	told = SysTick->VAL;        //�ս�?ʱ�ļ�����ֵ
	while (1) {
		tnow = SysTick->VAL;
		if (tnow != told) {
			if (tnow < told)
				tcnt += told - tnow; //��?ע��?��SYSTICK��?���ݼ��ļ������Ϳ�����.
			else
				tcnt += reload - tnow + told;
			told = tnow;
			if (tcnt >= ticks)
				break; //ʱ�䳬��/����Ҫ�ӳٵ�ʱ��,���˳�.
		}
	};
}
//��ʱnms
//nms:Ҫ��ʱ��ms��
void delay_ms(u16 nms) {
	u32 i;
	for (i = 0; i < nms; i++)
		delay_us(1000);
}
//����ɨ�躯��
//��ʹ��ʱע��
//���ذ���ֵ
//mode:0,��֧��������;1,֧��������;
//0��û���κΰ�������
//1��WKUP���� WK_UP
//ע��˺�������Ӧ���ȼ�,KEY0>KEY1>KEY2>WK_UP!!
u8 KEY_Scan(u8 mode)
{
    static u8 key_up=1;     //�����ɿ���־
    if(mode==1)key_up=1;    //֧������
    if(key_up&&(KEY0==0||KEY1==0||WK_UP==1))
    {
        delay_ms(10);
        key_up=0;
        if(KEY0==0)       return KEY0_PRES;
        else if(KEY1==0)  return KEY1_PRES;
        else if(WK_UP==1) return WKUP_PRES;          
    }else if(KEY0==1&&KEY1==1&&WK_UP==0)key_up=1;
    return 0;   //�ް�������
}
