#ifndef __PID_H__
#define __PID_H__

#define SAMPLING_TIME 5

struct {
	float setVol;       //设定值
	float samplingVol;  //实际值
	float err,err_last; //偏差值
	float Kp,Ki,Kd;     //比例、积分、微分系数
	float integral;     //积分值
    float voltage;      //输出
} PID;


void PID_Init(void);
float PID_Realize(float vol);


#endif
