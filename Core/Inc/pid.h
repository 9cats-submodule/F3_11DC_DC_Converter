#ifndef __PID_H__
#define __PID_H__

#define INCREASE

#ifdef POSITION
typedef struct {
	float setVol;       //�趨ֵ
	float samplingVol;  //ʵ��ֵ
	float err,err_last; //ƫ��ֵ
	float Kp,Ki,Kd;     //���������֡�΢��ϵ��
	float integral;     //����ֵ
    float voltage;      //���
    float integral_MAX; //���ȱ���
} S_PID;
extern S_PID PID;
#endif

#ifdef INCREASE
typedef struct {
	float setVol;       //�趨ֵ
	float samplingVol;  //ʵ��ֵ
	float err,lerr,perr;//ƫ��ֵ
	float Kp,Ki,Kd;     //���������֡�΢��ϵ��
    float inc;          //���
    float limit;        //���ȱ���
} S_PID;
extern S_PID PID;
#endif

void PID_Init(float Kp,float Ki,float Kd, float max);
float PID_Realize(float vi,float vt);

#endif
