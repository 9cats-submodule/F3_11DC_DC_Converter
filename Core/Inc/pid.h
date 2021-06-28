#ifndef __PID_H__
#define __PID_H__

#define SAMPLING_TIME 5

typedef struct {
	void (*init)(void);
	void (*realize)(void);
	unsigned char mode;     //ģʽ-0:�رգ�1:������磬2:��ѹ��磬3:�ŵ�
	unsigned int  CCR;

	float vol;
	float last_vol;
	float Kp,Kd,Ki;

	unsigned short Next_Ui;
	unsigned short Next_Uo;
	unsigned short Next_Ud;
	struct {
		unsigned short Ui;  //������
		unsigned short Uo;  //����ѹ
		unsigned short Ud;  //�ŵ��ѹ
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
