#include "zf_common_headfile.h"
#include "math.h"
#include "gyroscope.h"


//陀螺仪读取及数据处理
//目前只用到了初始化函数以及Get_Attitude()获取航向角，本文件中的卡尔曼滤波和交叉融合算法是用于求俯仰角的，不能滤偏航角(往年直立的代码)

#define my_pow(a) ((a)*(a))
#define halfT 0.0005f             // 采样周期的一半，用于求解四元数微分方程时计算角增量
void IMUupdate(float gx, float gy, float gz, float ax, float ay, float az,_imu_st *imu);

float Acc_X, Acc_Y, Acc_Z, Gyro_X, Gyro_Y, Gyro_Z;
float GRYX, GRYY, GRYZ, ACCX, ACCY, ACCZ;
int read_buff[6];
float angle_speed[4] = {0};
extern int chukuflag;
float Angle_Integral=0;

float speed_X=0;
float speed_Y=0;

float faiXTest=0;
float faiYTest=0;
float faiZTest=0;

float Distance_X=0;
float Distance_Y=0;

double yrateMax,yrateMin,yrateaverage;

rt_sem_t gyroscope_sem;//陀螺仪的信号量

GYRO_CLASS gyro;
CarAgnleInfotypedef CarAngle;
_imu_st imu_data =
{1,0,0,0,
{0,0,0},
{0,0,0},
{0,0,0},
{0,0,0},
{0,0,0},
0,0,0};

//陀螺仪线程
void gyroscope_entry(void *parameter)
{
  while(1)
  {
    rt_sem_take(gyroscope_sem, RT_WAITING_FOREVER);//2ms的PIT中断释放信号量
    Get_Attitude(); 
  }
}

float my_sqrt(float number)
{
  long i;
  float x, y;
  const float f = 1.5F;
  x = number * 0.5F;
  y = number;
  i = * ( long * ) &y;
  i = 0x5f3759df - ( i >> 1 );

  y = * ( float * ) &i;
  y = y * ( f - ( x * y * y ) );
  y = y * ( f - ( x * y * y ) );
  return number * y;
}
void LPF_1_db(float hz, float time, float in, float *out)//一阶IIR低通滤波
{
    *out += ( 1 / ( 1 + 1 / ( hz * 6.28f * time ) ) ) * ( in - *out );
}

//float sum=0,avr,max=-10,min=10;
//int32 num=0;
void Get_Attitude(void)
{
/**********************获取角速度Z**************************************/ 
/**********************低通滤波器处理陀螺仪中高频信号**************************************/
    imu963ra_get_gyro();
    LPF_1_db(35, 500, (imu963ra_gyro_transition(imu963ra_gyro_z)), &GRYZ);  
    
//    CarAngle.Yawrate =  0.7*CarAngle.Yawrate+0.3*GRYZ;//均值滤波
    CarAngle.Yawrate =  GRYZ;//均值滤波

        
    //角速度积分得到偏航角，以下代码是窗口滤波对零飘做处理    
    if(CarAngle.Yawrate<yrateMax&&CarAngle.Yawrate>yrateMin)
    {
      CarAngle.Yawrate=0;
      gyro.offset=0;
    }
    else         
    {     
      gyro.offset=yrateaverage;//零飘符合统计规律，长时间在小车静止的情况下读取数据以求得平均值，转动较快的情况下（大于0.6）将减掉漂移平均值。
    }
      gyro.TurnAngle_Integral -=   (CarAngle.Yawrate - gyro.offset )/ 500;//500hz采样频率，换算为 度每2ms再积分
    if(gyro.TurnAngle_Integral>360) gyro.TurnAngle_Integral-=360;
    else if(gyro.TurnAngle_Integral<-360) gyro.TurnAngle_Integral+=360;

//      CarAngle.Yawrate = -(float)((int)(CarAngle.Yawrate * 100 ) / 100);//符号修正了方向
//      gyro.TurnAngle_Integral +=   (CarAngle.Yawrate )/ 500;//500hz采样频率，换算为 度每2ms再积分
    
}




//float Q_angle=0.50, Q_gyro=0.070, R_angle=4;
//float Q_angle=0.2, Q_gyro=0.1, R_angle=0.3;
//float Q_angle=0.02, Q_gyro=0.01, R_angle=0.03;
//float Q_angle=0.01, Q_gyro=0.003, R_angle=10;
//float Q_angle=0.001, Q_gyro=0.1, R_angle=10;
//float Q_angle=0.01, Q_gyro=0.1, R_angle=10;
static  float Q_angle = 0.001, Q_gyro = 0.003, R_angle = 10, dt = 0.005;
//Q增大，动态响应增大
static float Pk[2][2] = { {1, 0}, {0, 1 }};

static float Pdot[4] = {0, 0, 0, 0};

static float q_bias = 0, angle_err, PCt_0, PCt_1, E, K_0, K_1, t_0, t_1;

//卡尔曼滤波算法

//-------------------------------------------------------
void Kalman_Filter(float angle_m, float gyro_m)
{
    gyro.Car_Angle += (gyro_m - q_bias) * dt; ///预测值
    Pdot[0] = Q_angle - Pk[0][1] - Pk[1][0];
    Pdot[1] = - Pk[1][1];
    Pdot[2] = - Pk[1][1];
    Pdot[3] = Q_gyro;
    Pk[0][0] += Pdot[0] * dt;
    Pk[0][1] += Pdot[1] * dt;
    Pk[1][0] += Pdot[2] * dt;
    Pk[1][1] += Pdot[3] * dt;
    angle_err = angle_m - gyro.Car_Angle; ///测量值-预测值
    PCt_0 =  Pk[0][0];
    PCt_1 =  Pk[1][0];
    E = R_angle + PCt_0;
    K_0 = PCt_0 / E; ///卡尔曼增益
    K_1 = PCt_1 / E;
    t_0 = PCt_0;
    t_1 = Pk[0][1];
    Pk[0][0] -= K_0 * t_0;
    Pk[0][1] -= K_0 * t_1;
    Pk[1][0] -= K_1 * t_0;
    Pk[1][1] -= K_1 * t_1;
    gyro.Car_Angle += K_0 * angle_err; ///最优角度=预测值+卡尔曼增益*(测量值-预测值)
    q_bias	+= K_1 * angle_err;
    gyro.Angle_Speed = gyro_m - q_bias;
}


void gyroscopeGetParameter(uint16 num)
{
  double  max=-1000,min=1000,sum=0;
  for(long int i = 0; i < num* 4; i++)
  {
    imu963ra_get_gyro();
    LPF_1_db(35, 500, (imu963ra_gyro_transition(imu963ra_gyro_z)), &GRYZ);     
    CarAngle.Yawrate =  GRYZ;
    if(max<CarAngle.Yawrate)
      max=CarAngle.Yawrate;
    if(min>CarAngle.Yawrate)
      min=CarAngle.Yawrate;    
    sum+=CarAngle.Yawrate;
  }
  
  yrateMax = max;
  yrateMin = min;
  yrateaverage = sum/num/4;  
}
void gyroscopeInit(void)//总初始化程序
{
  rt_thread_t tid;
  
  imu963ra_init();//硬件初始化
  
  gyroscopeGetParameter(65535);//窗口滤波的最大值，最小值与均值测量    //调试代码，用于计算从上电开始零飘的平均值和最大最小值，使用此代码时需要让小车静止不动
  
  //创建信号量，在2ms中断中被释放
  gyroscope_sem = rt_sem_create("gyroscope_sem", 0, RT_IPC_FLAG_FIFO);
  
  //创建线程 优先级6
  tid = rt_thread_create("gyroscope", gyroscope_entry, RT_NULL, 1024, 6, 10);
  
  //启动显示线程
  if(RT_NULL != tid)
  {
      rt_thread_startup(tid);
  }
  else													// 线程创建失败
  {
      rt_kprintf("encoder thread create ERROR.\n");
      return ;
  }
}

