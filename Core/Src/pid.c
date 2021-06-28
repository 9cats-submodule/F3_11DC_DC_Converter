#include "base.h"
#include "pid.h"

float vcc = 12.0f;

void PID_Init(float Kp,float Kd,float Ki) 
{
    PID.Kp = Kp;
    PID.Kd = Kd;
    PID.Ki = Ki;

    PID.setVol      = 0;
    PID.samplingVol = 0;
    PID.err         = 0;
    PID.err_last    = 0;
    PID.integral    = 0;
}


float PID_Realize(float vol)
{
	PID.setVol    = vol;
	PID.err       = PID.setVol - PID.samplingVol;
	PID.integral += PID.err;

	PID.voltage  = PID.Kp * PID.err +
			       PID.Ki * PID.integral +
			       PID.Kp * PID.Kd *(PID.err + PID.err_last);

	PID.err_last = PID.err;

    return PID.voltage;
}

//  ARR = PID.voltage/vold(π©µÁµÁ—π) * ARR;