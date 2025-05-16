#ifndef __CONTROL_H
#define __CONTROL_H
//�����趨
#include "zf_common_headfile.h"

typedef struct
{
	float Error;                          //�ٶ�ƫ��
	float StanX;
	float StanY;
	float lowest;                    //�ٶ��趨��Сֵ
	float highest;
	float L_front_ControlIntegral;
	float R_front_ControlIntegral;
	float Rear_ControlIntegral;
	float L_front_Bigeest;
	float R_front_Bigeest;
	float Rear_Bigeest;
} Speed_struct;
extern Speed_struct speed;


typedef struct
{
    void (*PIDInit)();
    void (*SpeedInit)();
    void (*MotorControl)(void);
    void (*RelativePositionControl)(void);
    void (*PositionControl)(void);
    void (*Anglecontrol)(void);
    void (*target_approach_control)(void);

} CONTROL_CLASS;
extern CONTROL_CLASS control;

typedef struct
{
    float P;
    float I;
    float D;
} PID_CLASS;

typedef struct
{

    float EN;
    float Error;
    float ErrorFifo[10];
    float ErrorInteger;//����ֵ
    float ErrorDt;
    float feedback[10];
    float ErrorTemp[4];
    float ErrorDtTemp[4];
    float FeedbackTemp[4];
    float Differential;
    float InOut;
    float Integ;
    float OutPut;
    float OutTemp[4];
    float outputspeed;
    
}Ctrl_class;

float Limit_float(float  value, float max);
float Limit1_float(float  value, float max, float min);
extern rt_sem_t control_sem;//�ź���
extern uint8 anglePIDFlag;
extern uint8 positionPIDFlag;
extern uint8 stopFlag;
extern uint8 singlePos;
extern float angleSpeed;
extern PID_CLASS ServPID;
extern PID_CLASS MotorPID;
extern PID_CLASS Left_front_MotorPID, Right_front_MotorPID, Rear_MotorPID, AnglePID, PosPID_X, PosPID_Y;
extern float  L_front_SpeedControlOutUpdata;                 //��ǰ�������ռ�ձ�
extern float  R_front_SpeedControlOutUpdata;                 //��ǰ�������ռ�ձ�
extern float  Rear_SpeedControlOutUpdata;                    //���ָ���ռ�ձ�
extern float  l_front_setspeed, r_front_setspeed, rear_setspeed;
extern uint8 leftdrift_flag,rightdrift_flag;
extern Ctrl_class Position_X, Position_Y,Angle;
void Speed_calculate(void);
void Set_Angle(void);
void speed_set(void);
int controlInit(void);
void SinglePos_control(void);
void angleCorrection(void);
void lowSpeedPID(void);
void hingSpeedPID(void);
void Angle_adapt(void);
#endif
