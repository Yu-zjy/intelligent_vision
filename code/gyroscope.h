#ifndef _GYROSCOPE_H_
#define _GYROSCOPE_H_
#include "zf_common_headfile.h"
extern float Angle_Integral;
typedef struct
{
  float ExYawratetemp;
  float ExStandUpAngle;
  float ExStandUpAngleFifo[4];
  float ExStandUpOldAgnle;
  float ExStandUpAgnleLimit;
  float ExGyroPitchrate;
  float ExYawrate;

  float Omega;
  float AccAngle;
  float MachineAngle1;
  float MachineAngle2;
  float Kangle;
  float Komega;
  float Pitch;
  float Pitchrate;
  float Yaw;
  float Yawrate;
  float Roll;
  float Rollrate;
  float TureYawrate;
  float Turn_Angle;
  float PitchNew;
  float yawNew;
  float rollNew;
  float Yaw_Avarage;
  float TureYawrate_Fifo[100];
}CarAgnleInfotypedef;

typedef struct
{
    float lpf_1;
    float out;
}_lf_t;

typedef struct
{
    float x;
    float y;
    float z;
} _xyz_f_st;

typedef struct
{
    float w;//q0;
    float x;//q1;
    float y;//q2;
    float z;//q3;

    _xyz_f_st x_vec;
    _xyz_f_st y_vec;
    _xyz_f_st z_vec;

    _xyz_f_st a_acc;
    _xyz_f_st w_acc;

    float rol;
    float pit;
    float yaw;
    float inter_rol;
    float inter_pit;
    float inter_yaw;
} _imu_st;//计算四元数所需要的参数，四元数最终等于wxyz



typedef struct
{
    float ACC_Angle;
    float Gravity_Angle;
    float Car_Angle;
    float CarY_Angle;
    float Angle_Speed;
    float Turn_Speed;
    float TurnAngle_Integral;
    uint8 speed_err_flag;
    float PitchAngle_Integral;
    float offset;
} GYRO_CLASS;

extern rt_sem_t gyroscope_sem;//陀螺仪的信号量
extern float faiXTest;
extern float faiYTest;
extern _xyz_f_st sensorGyro;
extern _xyz_f_st sensorACC;
extern float speed_X;
extern float speed_Y;
extern float Distance_X;
extern float Distance_Y;
extern GYRO_CLASS gyro;
extern CarAgnleInfotypedef CarAngle;
extern _imu_st imu_data;
extern int read_buff[6];
extern float Acc_X, Acc_Y, Acc_Z, Gyro_X, Gyro_Y, Gyro_Z;
void gyroscopeInit(void);//总初始化程序
void Get_Attitude(void);
void Kalman_Filter(float angle_m, float gyro_m);
float my_sqrt(float number);

int16 Get_X_Acc();
int16 Get_Y_Acc();
int16 Get_Z_Acc();
int16 Get_X_Gyro();
int16 Get_Y_Gyro();
int16 Get_Z_Gyro();

void Spi_Init(void);
void Icm20602_init_spi(void);
void Icm20602_self3_check(void);
void get_icm20602_accdata_spi(void);
void get_icm20602_gyro_spi(void);
void LPF_1_db(float hz, float time, float in, float *out);
#define IMUConverter 0.000532f
//#define PI 3.1415926535f



#endif
