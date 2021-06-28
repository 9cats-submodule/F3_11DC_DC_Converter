#ifndef __PID_H__
#define __PID_H__

#define SAMPLING_TIME 5

struct {
	float setVol;       //�趨ֵ
	float samplingVol;  //ʵ��ֵ
	float err,err_last; //ƫ��ֵ
	float Kp,Ki,Kd;     //���������֡�΢��ϵ��
	float integral;     //����ֵ
    float voltage;      //���
} PID;


void PID_Init(void);
float PID_Realize(float vol);


#endif
