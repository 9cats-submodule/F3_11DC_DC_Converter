#include "pid.h"

//TODO: 三模式/   Uo   Ui  Ud
// 1->  Ui == (1A)，Uo ==   0， Ud < 10V，K = 0
// 2->  Ui  < (1A)，Uo ==   0， Ud = 8.4V，K = 0
// 3->  Ui  ?     ，Uo == 14V， Ud > 4.2V，K = 1
// 4-> 不定(按1处理) -> K=0 ->  PWM最小，
//    判断，Ui方向（很可能反向），默认
// !不考     虑无负载情况
//  PWM比
float voltage;

PID PWM_PID = {
	PWM_PID_Init,
	PWM_PID_Realize,
	0
};

//初始化
void PWM_PID_Init(void)
{
	PWM_PID.mode = 0;
	PWM_PID.Kp = 1.5;
	PWM_PID.Kd = 1.5;
	PWM_PID.Ki = 1.5;
	PWM_PID.vol  = 0;
	PWM_PID.last_vol=0;
}

//达标值
void PWM_PID_Realize(void)
{
	switch(PWM_PID.mode)
	{
	case 1:
		{
			char err = PWM_PID.Next_Uo - PWM_PID.samplingVal[0].Uo;
			val = kp
		}break;
	case 2:break;
	case 3:break;
	default: break;
	}
}

PID PWM_PID;
void PWM_PID_Init(void)
{

}

void PWM_PID_Realize(float vol)
{
	PWM_PID.SetVol    = vol;
	PWM_PID.err       = PWM_PID.SetVol - PWM_PID.SamplingVol;
	PWM_PID.integral += PWM_PID.err;

	OUTPUT = PWM_PID.Kp * PWM_PID.err +
			 PWM_PID.Ki * PWM_PID.integral +
			 PWM_PID.Kp * PWM_PID.Kd *(PWM_PID.err + PWM_PID.err_last);

	PWM_PID.err_last = PWM_PID.err;

	//TODO:初始时需要先校准供电电压
	OC = (unsigned short)OUTPUT/vold(供电电压) * ARR;

}


