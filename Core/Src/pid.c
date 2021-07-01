#include "base.h"
#include "pid.h"

S_PID PID;

/*
void PID_Init(float Kp,float Kd,float Ki, float max)
{
    PID.Kp = Kp;
    PID.Kd = Kd;
    PID.Ki = Ki;
    PID.integral_MAX = max;

    PID.setVol      = 0;
    PID.samplingVol = 0;
    PID.err         = 0;
    PID.err_last    = 0;
    PID.integral    = 0;
}


float PID_Realize(float vi,float vt)
{
	PID.samplingVol = vi;
	PID.setVol      = vt;
	PID.err         = PID.setVol - PID.samplingVol;
	PID.integral   += PID.err;

	PID.voltage  = PID.Kp * PID.err +
			       PID.Ki * PID.integral +
			       PID.Kp * PID.Kd *(PID.err + PID.err_last);

	PID.err_last = PID.err;

	//TODO:·ÀÖ¹¹ý´ó
	if(-PID.integral < -PID.integral_MAX && PID.integral > PID.integral_MAX)
	{
		if( PID.integral >  PID.integral_MAX) PID.integral = PID.integral_MAX;
		if(-PID.integral > -PID.integral_MAX) PID.integral = PID.integral_MAX;
	}

    return PID.voltage;
}
*/

void PID_Init(float Kp,float Ki,float Kd, float max)
{
    PID.Kp = Kp;
    PID.Ki = Ki;
    PID.Kd = Kd;
    PID.limit = max;

    PID.setVol      = 0;
    PID.samplingVol = 0;
    PID.err         = 0;
    PID.lerr        = 0;
    PID.perr        = 0;
}

float PID_Realize(float vi,float vt)
{
	PID.samplingVol = vi;
	PID.setVol      = vt;
	PID.err         = PID.setVol - PID.samplingVol;

	PID.err = vt - vi;

	PID.inc = PID.Kp * (PID.err - PID.lerr)
		    + PID.Ki *  PID.err
		    + PID.Kd * (PID.err - 2*PID.lerr + PID.perr);

	PID.perr = PID.lerr;
	PID.lerr = PID.err;

	if( PID.inc < -PID.limit) PID.inc = -PID.limit;
	if( PID.inc >  PID.limit) PID.inc =  PID.limit;

	return PID.inc;
}






