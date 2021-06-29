#ifndef __PID_H__
#define __PID_H__

#define SAMPLING_TIME 5

typedef struct {
	float setVol;       //�趨ֵ
	float samplingVol;  //ʵ��ֵ
	float err,err_last; //ƫ��ֵ
	float Kp,Ki,Kd;     //���������֡�΢��ϵ��
	float integral;     //����ֵ
  float voltage;      //���
} S_PID;
extern S_PID PID;


void PID_Init(float Kp,float Kd,float Ki);
float PID_Realize(float vol);


#endif
