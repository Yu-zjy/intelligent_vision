#include "encoder.h"
#include <math.h>

#define USE_CONFICINET

// �����������������
int16 Left_front_count;
int16 Right_front_count;
int16 Rear_count; 
// �����ٶȱ���
float Left_front_CarSpeed = 0;
float Right_front_CarSpeed = 0;
float Rear_CarSpeed = 0; // �����ٶ�

// ����λ�ñ���
Coordinate encoder = {0.0f, 0.0f}; // ��ʼ�����
rt_sem_t encoder_sem; // �ź���

// ����ϵ��
float coefficiencient = 1.0;

void encoder_entry(void *parameter)
{
    while (1)
    {
        rt_sem_take(encoder_sem, RT_WAITING_FOREVER);
        GetSpeed_position(); // ��ȡ�ٶȺ�λ��
        rt_sem_release(control_sem);
    }
}

#define RPOLY4                  0.166676916020084
#define RPOLY3                  -0.546165629714961
#define RPOLY2                  0.392587999505110
#define RPOLY1                  1.19354576275417
#define RPOLY_CONSTANT          -0.0100891764733760
#define RPOLY4_2                0.00307550812041141
#define RPOLY3_2                -0.0458095263387465
#define RPOLY2_2                0.252084562531021
#define RPOLY1_2                0.404194428671441
#define RPOLY_CONSTANT_2        0.646639620477122

AT_ITCM_SECTION_INIT(void counterCoefficient(Coordinate encoder, const Coordinate targetPoint))
{
    double encoderDistance;
    float distance = my_sqrt((encoder.X - targetPoint.X) * (encoder.X - targetPoint.X) + (encoder.Y - targetPoint.Y) * (encoder.Y - targetPoint.Y));
    if (distance < 2.0)
        encoderDistance = RPOLY4 * pow(distance, 4.0) + RPOLY3 * pow(distance, 3.0) + RPOLY2 * pow(distance, 2.0) + RPOLY1 * distance + RPOLY_CONSTANT;
    else
        encoderDistance = RPOLY4_2 * pow(distance, 4.0) + RPOLY3_2 * pow(distance, 3.0) + RPOLY2_2 * pow(distance, 2.0) + RPOLY1_2 * distance + RPOLY_CONSTANT_2;
    coefficiencient = distance / encoderDistance;
    return;
}

int encoder_init(void)
{
    rt_thread_t tid;

    // ��ʼ����ʱ��
    qtimerInit();

    // �����ź���
    encoder_sem = rt_sem_create("encoder_sem", 0, RT_IPC_FLAG_FIFO);

    // �����߳�
    tid = rt_thread_create("encoder", encoder_entry, RT_NULL, 1024, 7, 10);

    // �����߳�
    if (RT_NULL != tid)
    {
        rt_thread_startup(tid);
    }
    else
    {
        rt_kprintf("encoder thread create ERROR.\n");
        return -1;
    }

    return 0;
}

void qtimerInit(void)
{
    // ��ʼ������������
    encoder_dir_init(R_frontQTIMER, R_front_QD_phaseA, R_front_QD_phaseB);
    encoder_dir_init(L_frontQTIMER, L_front_QD_phaseA, L_front_QD_phaseB);
    encoder_dir_init(RearQTIMER, Rear_QD_phaseA, Rear_QD_phaseB); // ��ʼ�����ֱ�����

    // �������
    encoder_clear_count(R_frontQTIMER);
    encoder_clear_count(L_frontQTIMER);
    encoder_clear_count(RearQTIMER); // ������ּ���
}

void GetSpeed_position(void)
{
    float dx, dy;
    static int16 l_f_filter[4], r_f_filter[4], rear_filter[4];
    int i;

    // ��ȡ����������
    Right_front_count = encoder_get_count(R_frontQTIMER);
    encoder_clear_count(R_frontQTIMER);
    r_f_filter[0] = Right_front_count;
    for (i = 3; i > 0; i--)
        r_f_filter[i] = r_f_filter[i - 1];

    Left_front_count = encoder_get_count(L_frontQTIMER);
    encoder_clear_count(L_frontQTIMER);
    l_f_filter[0] = Left_front_count;
    for (i = 3; i > 0; i--)
        l_f_filter[i] = l_f_filter[i - 1];

    Rear_count = encoder_get_count(RearQTIMER); // ��ȡ���ּ���
    encoder_clear_count(RearQTIMER);
    rear_filter[0] = Rear_count;
    for (i = 3; i > 0; i--)
        rear_filter[i] = rear_filter[i - 1];

    // �����ٶ�
    Left_front_CarSpeed = (float)(Left_front_count * SPEED_F / L_front_QD_UNIT);
    Right_front_CarSpeed = (float)(Right_front_count * SPEED_F / R_front_QD_UNIT);
    Rear_CarSpeed = (float)(Rear_count * SPEED_F / Rear_QD_UNIT); // ��������ٶ�

    // ����λ��
    ///dy= (float)(Left_rear_count/L_rear_QD_UNIT  +  Right_rear_count/R_rear_QD_UNIT  +  Left_front_count/L_front_QD_UNIT + Right_front_count/R_front_QD_UNIT) *0.250f;
    ///dx= (float)(-Left_rear_count/L_rear_QD_UNIT  +  Right_rear_count/R_rear_QD_UNIT  +  Left_front_count/L_front_QD_UNIT - Right_front_count/R_front_QD_UNIT)*0.250f;
    
    dy = (float)(Left_front_count/1.732 / L_front_QD_UNIT - Right_front_count/1.732 / R_front_QD_UNIT);
    dx = (float)(2*Left_front_count / L_front_QD_UNIT + 2*Right_front_count / R_front_QD_UNIT- Rear_count/ Rear_QD_UNIT)/3;
    //dy = (float)(Left_front_count*1.732 / L_front_QD_UNIT - Right_front_count*1.732 / R_front_QD_UNIT)/2;
    //dx = (float)(Left_front_count / L_front_QD_UNIT + Right_front_count / R_front_QD_UNIT- Rear_count/ Rear_QD_UNIT/2);
              //l_front_setspeed = (float)(carYSpeed/1.732 + carXSpeed/3 + Angle.outputspeed/3); 
              //r_front_setspeed = (float)(-carYSpeed/1.732 + carXSpeed/3 + Angle.outputspeed/3);
              //rear_setspeed = (float)(-2*carXSpeed/3 + Angle.outputspeed/3);
///    encoder.Y+=(float)(dy*cos(PI*gyro.TurnAngle_Integral/180)-dx*sin(PI*gyro.TurnAngle_Integral/180));
///    encoder.X+=(float)(dx*cos(PI*gyro.TurnAngle_Integral/180)+dy*sin(PI*gyro.TurnAngle_Integral/180)); 

    encoder.Y += (float)(dy * cos(PI * gyro.TurnAngle_Integral / 180) - dx * sin(PI * gyro.TurnAngle_Integral / 180));
    encoder.X += (float)(dx * cos(PI * gyro.TurnAngle_Integral / 180) + dy * sin(PI * gyro.TurnAngle_Integral / 180));
}