#ifndef __PID_H__
#define __PID_H__

#define INCREASE

#ifdef POSITION
typedef struct {
	float setVol;       //设定值
	float samplingVol;  //实际值
	float err,err_last; //偏差值
	float Kp,Ki,Kd;     //比例、积分、微分系数
	float integral;     //积分值
    float voltage;      //输出
    float integral_MAX; //过度保护
} S_PID;
extern S_PID PID;
#endif

#ifdef INCREASE
typedef struct {
	float setVol;       //设定值
	float samplingVol;  //实际值
	float err,lerr,perr;//偏差值
	float Kp,Ki,Kd;     //比例、积分、微分系数
    float inc;          //输出
    float limit;        //过度保护
} S_PID;
extern S_PID PID;
#endif

void PID_Init(float Kp,float Ki,float Kd, float max);
float PID_Realize(float vi,float vt);

#endif
