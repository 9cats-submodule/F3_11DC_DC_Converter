#ifndef __PID_H__
#define __PID_H__

#define SAMPLING_TIME 5

typedef struct {
	void (*init)(void);
	void (*realize)(void);
	unsigned char mode;     //模式-0:关闭，1:恒流充电，2:恒压充电，3:放电
	unsigned int  CCR;

	float vol;
	float last_vol;
	float Kp,Kd,Ki;

	unsigned short Next_Ui;
	unsigned short Next_Uo;
	unsigned short Next_Ud;
	struct {
		unsigned short Ui;  //充电电流
		unsigned short Uo;  //充电电压
		unsigned short Ud;  //放电电压
	} samplingVal[SAMPLING_TIME];
} PID;

typedef struct {
	float SetVol;
	float SamplingVol;
	float err,err_last;
	float Kp,Ki,Kd;
	float integral;
} PID;


void PWM_PID_Realize(void);
void PWM_PID_Init(void);


#endif
