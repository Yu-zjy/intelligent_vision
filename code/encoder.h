#ifndef _encoder_h
#define _encoder_h

#include "zf_common_headfile.h"

#define SPEED_F 1000.0f // �ٶȻ�ȡƵ�ʣ���Ӧ1ms��ʱ����
#define L_front_QD_UNIT 35000.0f // ������һ�׵ļ�������ǰ��
#define R_front_QD_UNIT 35000.0f // ������һ�׵ļ�������ǰ��
#define Rear_QD_UNIT 35000.0f // ������һ�׵ļ���������

#define L_frontQTIMER            QTIMER1_ENCODER1
#define R_frontQTIMER            QTIMER2_ENCODER1
#define RearQTIMER               QTIMER2_ENCODER2 

#define L_front_QD_phaseA         QTIMER1_ENCODER1_CH1_C0
#define L_front_QD_phaseB         QTIMER1_ENCODER1_CH2_C1

#define R_front_QD_phaseA         QTIMER2_ENCODER1_CH1_C3
#define R_front_QD_phaseB         QTIMER2_ENCODER1_CH2_C4

#define Rear_QD_phaseA            QTIMER2_ENCODER2_CH1_C5
#define Rear_QD_phaseB            QTIMER2_ENCODER2_CH2_C25

typedef struct
{
    float X;
    float Y;
} Coordinate;

extern rt_sem_t encoder_sem;
extern Coordinate encoder;
extern float Left_front_CarSpeed;
extern float Right_front_CarSpeed;
extern float Rear_CarSpeed; // �����ٶ�
extern int16 Left_front_count;
extern int16 Right_front_count;
extern int16 Rear_count; // ���ּ���

void qtimerInit(void);
void GetSpeed_position(void);
int encoder_init(void);
void counterCoefficient(Coordinate encoder, const Coordinate targetPoint);

#endif