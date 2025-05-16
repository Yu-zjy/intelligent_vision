#include "control.h"
//PID参数  4速度 1角度 2位置 两层PID串联
PID_CLASS Left_front_MotorPID, Right_front_MotorPID, Rear_MotorPID;        //电机的参数
PID_CLASS AnglePID;
PID_CLASS PosPID_X,PosPID_Y;
PID_CLASS SinglePosPID;
//标志位
uint8 anglePIDFlag=1;
uint8 positionPIDFlag=1;
uint8 stopFlag=0;
uint8 singlePos = 0;
//计算存储空间？
Ctrl_class Position_X={0},Position_Y={0},Position_r={0};
Ctrl_class Angle={0};
Ctrl_class SinglePos={0};
//速度控制参数
Speed_struct speed;

uint8 buf[3]={0xaa,0x55,0};
//速度设定值
float SetX=0.0f,SetY=0.0f;
float SetAngle;
extern findline_TypeDef Findline;
extern uint8 angle_correct;
extern uint8 move_state;
extern uint8 roundabout_type,road_type;
//PWM占空比
float L_front_SpeedControlOutUpdata;                 //左前电机更新占空比
float R_front_SpeedControlOutUpdata;                 //右前电机更新占空比
float Rear_SpeedControlOutUpdata;                    //后轮更新占空比

int TestMotorPwm[6];

void PIDInit(void);//PID参数初始化
void SpeedInit(void);//速度参数初始化（移植上届全向留下的，实际上没什么用）
void MotorControl(void);//电机速度控制(整个控制最底层)
void Speed_calculate(void);//由期望速度进行PID计算得到PWM占空比，被MotorControl调用
void speed_set(void);//根据目标位置和期望姿态解算四个轮子期望速度
void relative_position_control(void);//巡线相对跑道位置
void Position_control(void);//坐标位置控制，让小车运动到期望坐标，输出是期望的分速度
void Angle_control(void);//角度控制，保持小车不发生转动（为了求位置准确率，希望减少机体坐标旋转产生的误差）
void Set_Angle(void);//设定目标角度，让小车运行时不发生转动
void Angle_adapt(void);
void target_approach_control();
float angleSet=0;//期望角度

//速度限制
float HIGHEST_SPEEDX=2.5f;
float HIGHEST_SPEEDY=3.5f;
float angleSpeed = 3.5f;

CONTROL_CLASS control =
{
    &PIDInit,
    &SpeedInit,
    &MotorControl,
    &target_approach_control,
    &relative_position_control,
    &Position_control,
    &Angle_control,
};


//PID参数整定
void PIDInit()
{
	Left_front_MotorPID.P = 5.0; //3.5PID
	Left_front_MotorPID.I = 0.0;
	Left_front_MotorPID.D = 0.1;

	Right_front_MotorPID.P = 5.0;
	Right_front_MotorPID.I = 0.0;
	Right_front_MotorPID.D = 0.1;

	Rear_MotorPID.P = 5.0;
	Rear_MotorPID.I = 0.0;
	Rear_MotorPID.D = 0.1;

	PosPID_X.P = 0.1;
	PosPID_X.I = 0;
	PosPID_X.D = 0.0;

	PosPID_Y.P = 0.1; //3.5y
	PosPID_Y.I = 0;
	PosPID_Y.D = 0.0;

	SinglePosPID.P = 1.0;
	SinglePosPID.I = 0;
	SinglePosPID.D = 0;

	AnglePID.P = 3; //1.5w
	AnglePID.I = 0;
	AnglePID.D = 0.2;
}
void lowSpeedPID(void)
{
	Left_front_MotorPID.P = 5.0; //3.5PID
	Left_front_MotorPID.I = 0.0;
	Left_front_MotorPID.D = 0.1;

	Right_front_MotorPID.P = 5.0;
	Right_front_MotorPID.I = 0.0;
	Right_front_MotorPID.D = 0.1;

	Rear_MotorPID.P = 5.0;
	Rear_MotorPID.I = 0.0;
	Rear_MotorPID.D = 0.1;

	PosPID_X.P = 0.1;
	PosPID_X.I = 0;
	PosPID_X.D = 0.0;

	PosPID_Y.P = 0.1; //3.5y
	PosPID_Y.I = 0;
	PosPID_Y.D = 0;
        
        	SinglePosPID.P = 1.0;
	SinglePosPID.I = 0;
	SinglePosPID.D = 0;

	AnglePID.P = 3; //1.5w
	AnglePID.I = 0;
	AnglePID.D = 0.2;
}
void hingSpeedPID(void) //加强X位置环，在原地转弯时使用
{
	PosPID_X.P = 12.0; //3.5y
	PosPID_X.I = 0;
	PosPID_X.D = 8.0;
}

void SpeedInit()
{
	speed.StanX = 0;
	speed.StanY = 0;
	speed.lowest = (float)0.95 * MIN(speed.StanX, speed.StanY);
	speed.highest = (float)1.6 * MAX(speed.StanX, speed.StanY);

	speed.L_front_Bigeest = 6000;
	speed.R_front_Bigeest = 6000;
	speed.Rear_Bigeest = 6000; //防止电流过大造成复位
}

//线程入口
extern void Set_position(void);
void control_entry(void *parameter)
{
  while(1)
  {
    rt_sem_take(control_sem, RT_WAITING_FOREVER);
    Set_position();//top logic
    
    if(anglePIDFlag==1)
    {
      Set_Angle();
      control.Anglecontrol();
    }
    if(positionPIDFlag==1)
    {
     control.PositionControl();//绝对位置控制，用于摄像头发送箱子位置信息控制
    }
    else if(positionPIDFlag==2)
    {
    target_approach_control();
    }
    else
    {
      control.RelativePositionControl();//巡线运动控制
    }
    control.MotorControl();//计算出期望速度、速度环
    
    //电机测试（平时注释掉）  在debug下改数组值为不为零的数值即可。注意测试时数组内最好只能给一个1，其余全零。
    //MotorPwmFlash(TestMotorPwm);

  }
}
rt_sem_t control_sem;//信号量
//初始化函数 
int controlInit(void)
{
    rt_thread_t tid;
    
    control.PIDInit();                  //PID参数初始化
    control.SpeedInit();                //速度参数初始化
    
    //创建信号量，在encoder_entry中被释放
    control_sem = rt_sem_create("control_sem", 0, RT_IPC_FLAG_FIFO);
    
  //创建线程 优先级7
  tid = rt_thread_create("control", control_entry, RT_NULL, 1024, 7, 10);
  //启动显示线程
  if(RT_NULL != tid)
  {
      rt_thread_startup(tid);
  }
  else													// 线程创建失败
  {
      rt_kprintf("control thread create ERROR.\n");
      return -1;
  }
      
  return 0;
}


//三个电机的期望速度，作为PID串联的中间值
float l_front_setspeed = 0, r_front_setspeed = 0, rear_setspeed = 0; //左前，右前，后辅助


/**************************电机的控制*****************************************************************************************************************************************/
void MotorControl(void)
{
      if(anglePIDFlag==1||positionPIDFlag==1)
        speed_set();
      Speed_calculate();
}
// 新增的控制函数，基于x_error和h_error控制运动
void target_approach_control(void)
{
    uint8 i;

    // 处理x方向的误差，类似relative_position_control中的X方向处理
    Position_X.Error = x_error;

    for (i = 9; i > 0; i--)
        Position_X.ErrorFifo[i] = Position_X.ErrorFifo[i - 1];
    Position_X.ErrorFifo[0] = Position_X.Error;

    Position_X.ErrorInteger += Position_X.ErrorFifo[0];
    Position_X.ErrorInteger = Limit_float(Position_X.ErrorInteger, 1.4);

    for (i = 9; i > 0; i--)
        Position_X.feedback[i] = Position_X.feedback[i - 1];
    Position_X.feedback[0] = Position_X.Error;

    for (i = 3; i > 0; i--)
        Position_X.ErrorDtTemp[i] = Position_X.ErrorDtTemp[i - 1];
    Position_X.ErrorDtTemp[0] = Position_X.feedback[0] - Position_X.feedback[3];

    // PID计算速度，位置式PID一般是PD控制，防止积分饱和，系数KI为0
    Position_X.InOut = PosPID_X.P * Position_X.ErrorFifo[0] + PosPID_X.I * Position_X.ErrorInteger +
                       PosPID_X.D * (Position_X.ErrorDtTemp[0] * 0.6 + Position_X.ErrorDtTemp[1] * 0.3 + Position_X.ErrorDtTemp[2] * 0.1);

    for (i = 3; i > 0; i--)
        Position_X.OutTemp[i] = Position_X.OutTemp[i - 1];
    Position_X.OutTemp[0] = Position_X.InOut;

    Position_X.OutPut = Position_X.OutTemp[0] * 0.8 + Position_X.OutTemp[1] * 0.2;

    Position_X.OutPut = Limit_float(Position_X.OutPut, MAXYSPEED);
    Position_X.outputspeed = Position_X.OutPut;

    // 处理h方向的误差，这里简单假设h_error影响Y方向速度
    Position_Y.Error = h_error;

    for (i = 9; i > 0; i--)
        Position_Y.ErrorFifo[i] = Position_Y.ErrorFifo[i - 1];
    Position_Y.ErrorFifo[0] = Position_Y.Error;

    Position_Y.ErrorInteger += Position_Y.ErrorFifo[0];
    Position_Y.ErrorInteger = Limit_float(Position_Y.ErrorInteger, 1.4);

    for (i = 9; i > 0; i--)
        Position_Y.feedback[i] = Position_Y.feedback[i - 1];
    Position_Y.feedback[0] = Position_Y.Error;

    for (i = 3; i > 0; i--)
        Position_Y.ErrorDtTemp[i] = Position_Y.ErrorDtTemp[i - 1];
    Position_Y.ErrorDtTemp[0] = Position_Y.feedback[0] - Position_Y.feedback[3];

    Position_Y.InOut = PosPID_Y.P * Position_Y.ErrorFifo[0] + PosPID_Y.I * Position_Y.ErrorInteger +
                       PosPID_Y.D * (Position_Y.ErrorDtTemp[0] * 0.6 + Position_Y.ErrorDtTemp[1] * 0.3 + Position_Y.ErrorDtTemp[2] * 0.1);

    for (i = 3; i > 0; i--)
        Position_Y.OutTemp[i] = Position_Y.OutTemp[i - 1];
    Position_Y.OutTemp[0] = Position_Y.InOut;

    Position_Y.OutPut = Position_Y.OutTemp[0] * 0.8 + Position_Y.OutTemp[1] * 0.2;

    Position_Y.OutPut = Limit_float(Position_Y.OutPut, MAXYSPEED);
    Position_Y.outputspeed = Position_Y.OutPut;

    // 根据运动状态调整速度
    if (move_state == 0)
    {
        if (road_type == STRAIGHT)
            Position_Y.outputspeed = 1.6;
        else if (roundabout_type > 5)
            Position_Y.outputspeed = 1;
        else if (road_type == BLOCK)
            Position_Y.outputspeed = 1;
        else
            Position_Y.outputspeed = 0.8;
    }
    else if (move_state == 3)
    {
        Position_Y.outputspeed = -1;
    }
    else if (move_state == 4)
    {
        Position_Y.outputspeed = 0.9;
    }
    else
    {
        if (roundabout_type)
            Position_Y.outputspeed = Position_X.outputspeed + 0.2;
        else
            Position_Y.outputspeed = Position_X.outputspeed;
        if (move_state == 1)
            Position_X.outputspeed = -1;
        else
            Position_X.outputspeed = 1;
    }
}
/**************************巡线相对位置闭环控制*********************************************************************************************************************************/
void relative_position_control(void)
{
        uint8 i;
        Position_r.Error=Findline.err[0];
        for(i=9;i>0;i--)
          Position_r.ErrorFifo[i]=Position_r.ErrorFifo[i-1];
        Position_r.ErrorFifo[0] = Position_r.Error;

        Position_r.ErrorInteger += Position_r.ErrorFifo[0];//积分量
        //Angle.ErrorInteger -=  Angle.ErrorFifo[29];
        Position_r.ErrorInteger=Limit_float(Position_r.ErrorInteger,1.4);//积分量限制以防超量
        for(i=9;i>0;i--)
            Position_r.feedback[i] = Position_r.feedback[i-1];
        Position_r.feedback[0] = Position_r.Error;


        for(i=3;i>0;i--)
            Position_r.ErrorDtTemp[i] = Position_r.ErrorDtTemp[i-1];
        Position_r.ErrorDtTemp[0] = Position_r.feedback[0]-Position_r.feedback[3];//微分量

        //PID计算输出速度，位置式PID一般采用PD控制以防止超调，此处积分系数KI为0
        Position_r.InOut = PosPID_X.P * Position_r.ErrorFifo[0] +PosPID_X.I*Position_r.ErrorInteger+
                PosPID_X.D * (Position_r.ErrorDtTemp[0] * 0.6 + Position_r.ErrorDtTemp[1] * 0.3 + Position_r.ErrorDtTemp[2] * 0.1);

        for(i=3;i>0;i--)
            Position_r.OutTemp[i] = Position_r.OutTemp[i-1];
        Position_r.OutTemp[0] = Position_r.InOut;

        Position_r.OutPut = Position_r.OutTemp[0]*0.8 + Position_r.OutTemp[1]*0.2;

        Position_r.OutPut =Limit_float(Position_r.OutPut, 0.4);
        Position_X.outputspeed=Position_r.OutPut;
        Findline.err[0]=Findline.err[0]-0.002*Position_X.outputspeed;
        
        if(move_state==0)
        {
         //Position_Y.outputspeed = 2.5;
         if(road_type==STRAIGHT) Position_Y.outputspeed = 1.6;
         else if(roundabout_type>5) Position_Y.outputspeed=1;
         else if(road_type==BLOCK) Position_Y.outputspeed=1;
         else Position_Y.outputspeed=1.2;
//          if(angle_correct) Position_Y.outputspeed = 0;
        }
      else if(move_state==3) 
        {
          Position_Y.outputspeed=-1;
        }
        else if(move_state==4)
        {
          Position_Y.outputspeed = 0.9;
        }
        else
        {
          if(roundabout_type) Position_Y.outputspeed=Position_X.outputspeed+0.2;
          else Position_Y.outputspeed=Position_X.outputspeed;
          if(move_state==1) Position_X.outputspeed=-1;
          else Position_X.outputspeed=1;
          Findline.err[0]=Findline.err[0]-0.002*Position_Y.outputspeed;
        }
}
/**************************位置闭环控制*****************************************************************************************************************************************/

void Position_control(void)
{ 
        uint8 i;
  //X方向Pid
        Position_X.Error=(SetX-encoder.X)*cos(PI*gyro.TurnAngle_Integral/180)-(SetY-encoder.Y)*sin(PI*gyro.TurnAngle_Integral/180);//求偏差，这里的encoder.X实际距离是编码器（里程计）累加出的距离，实际不大准，需要后续结合惯导优化

        for(i=9;i>0;i--)
            Position_X.ErrorFifo[i] = Position_X.ErrorFifo[i-1];//软件实现移位寄存器，进行数据更新 深度为9
        Position_X.ErrorFifo[0] = Position_X.Error;

        Position_X.ErrorInteger += Position_X.ErrorFifo[0];//积分量
        //Angle.ErrorInteger -=  Angle.ErrorFifo[29];
        Position_X.ErrorInteger=Limit_float(Position_X.ErrorInteger,1.4);//积分量限制以防超量
        for(i=9;i>0;i--)
            Position_X.feedback[i] = Position_X.feedback[i-1];
        Position_X.feedback[0] = Position_X.Error;


        for(i=3;i>0;i--)
            Position_X.ErrorDtTemp[i] = Position_X.ErrorDtTemp[i-1];
        Position_X.ErrorDtTemp[0] = Position_X.feedback[0]-Position_X.feedback[3];//微分量

        //PID计算输出速度，位置式PID一般采用PD控制以防止超调，此处积分系数KI为0
        Position_X.InOut = PosPID_X.P * Position_X.ErrorFifo[0] +PosPID_X.I*Position_X.ErrorInteger+
                PosPID_X.D * (Position_X.ErrorDtTemp[0] * 0.6 + Position_X.ErrorDtTemp[1] * 0.3 + Position_X.ErrorDtTemp[2] * 0.1);

        for(i=3;i>0;i--)
            Position_X.OutTemp[i] = Position_X.OutTemp[i-1];
        Position_X.OutTemp[0] = Position_X.InOut;

        Position_X.OutPut = Position_X.OutTemp[0]*0.8 + Position_X.OutTemp[1]*0.2;

        Position_X.OutPut =Limit_float(Position_X.OutPut, MAXYSPEED);
        Position_X.outputspeed=Position_X.OutPut;
        
        
        //Y方向Pid
        Position_Y.Error=(SetX-encoder.X)*sin(PI*gyro.TurnAngle_Integral/180)+(SetY-encoder.Y)*cos(PI*gyro.TurnAngle_Integral/180);;

        for(i=9;i>0;i--)
            Position_Y.ErrorFifo[i] = Position_Y.ErrorFifo[i-1];
        Position_Y.ErrorFifo[0] = Position_Y.Error;

        Position_Y.ErrorInteger += Position_Y.ErrorFifo[0];
        //Angle.ErrorInteger -=  Angle.ErrorFifo[29];
        Position_Y.ErrorInteger=Limit_float(Position_Y.ErrorInteger,1.4);
        for(i=9;i>0;i--)
            Position_Y.feedback[i] = Position_Y.feedback[i-1];
        Position_Y.feedback[0] = Position_Y.Error;


        for(i=3;i>0;i--)
            Position_Y.ErrorDtTemp[i] = Position_Y.ErrorDtTemp[i-1];
        Position_Y.ErrorDtTemp[0] = Position_Y.feedback[0]-Position_Y.feedback[3];

        Position_Y.InOut = PosPID_Y.P * Position_Y.ErrorFifo[0] +PosPID_Y.I*Position_Y.ErrorInteger+
                PosPID_Y.D * (Position_Y.ErrorDtTemp[0] * 0.6 + Position_Y.ErrorDtTemp[1] * 0.3 + Position_Y.ErrorDtTemp[2] * 0.1);

        for(i=3;i>0;i--)
            Position_Y.OutTemp[i] = Position_Y.OutTemp[i-1];
        Position_Y.OutTemp[0] = Position_Y.InOut;

        Position_Y.OutPut = Position_Y.OutTemp[0]*0.8 + Position_Y.OutTemp[1]*0.2;

        Position_Y.OutPut =Limit_float(Position_Y.OutPut, MAXYSPEED);
        Position_Y.outputspeed=Position_Y.OutPut;

 
        
}


/**************************角度闭环控制*****************************************************************************************************************************************/
void Angle_control(void)
{        
    
  uint8 i;
  //与位置闭环类似，同样采用PD控制
    Angle.Error=(SetAngle-gyro.TurnAngle_Integral)/100;

        for(i=9;i>0;i--)
            Angle.ErrorFifo[i] = Angle.ErrorFifo[i-1];
        Angle.ErrorFifo[0] = Angle.Error;

        Angle.ErrorInteger += Angle.ErrorFifo[0];
        //Angle.ErrorInteger -=  Angle.ErrorFifo[29];
        Angle.ErrorInteger=Limit_float(Angle.ErrorInteger,1.4);
        for(i=9;i>0;i--)
            Angle.feedback[i] = Angle.feedback[i-1];
        Angle.feedback[0] = Angle.Error;


        for(i=3;i>0;i--)
            Angle.ErrorDtTemp[i] = Angle.ErrorDtTemp[i-1];
        Angle.ErrorDtTemp[0] = Angle.feedback[0]-Angle.feedback[3];


        Angle.InOut = AnglePID.P * Angle.ErrorFifo[0] +AnglePID.I*Angle.ErrorInteger+
                AnglePID.D * (Angle.ErrorDtTemp[0] * 0.6 + Angle.ErrorDtTemp[1] * 0.3 + Angle.ErrorDtTemp[2] * 0.1);

        for(i=3;i>0;i--)
            Angle.OutTemp[i] = Angle.OutTemp[i-1];
        Angle.OutTemp[0] = Angle.InOut;

        Angle.OutPut = Angle.OutTemp[0]*0.8 + Angle.OutTemp[1]*0.2;

        Angle.OutPut =Limit_float(Angle.OutPut, angleSpeed);
        Angle.outputspeed=Angle.OutPut;
}
 

/**************************设置目的角度*****************************************************************************************************************************************/
//顶层应用函数
void Set_Angle(void)
{ 
  if(fabs(gyro.TurnAngle_Integral-angleSet)>180)
  {
    if(gyro.TurnAngle_Integral>0)
    {
      gyro.TurnAngle_Integral -= 360;
    }
    else
    {
      gyro.TurnAngle_Integral += 360;
    }
  }
  SetAngle=angleSet; 
}

void Angle_adapt(void)
{
  if(angleSet>180) angleSet-=360;
  else if(angleSet<=-180) angleSet+=360;
}

/**************************期望速度计算*****************************************************************************************************************************************/
void speed_set(void)
{
	//由全向轮的运动学逆推得到
	//实际整个控制系统是一个位置+角度并级PID后再与速度PID串级
	//Position_Y.outputspee是惯性坐标系下的期望速度 ，l_front_setspeed是轮子的速度，需要在载体坐标系下计算
	float carYSpeed = 0, carXSpeed = 0;//惯性坐标系的速度
        
	carYSpeed = Position_Y.outputspeed;
	carXSpeed = Position_X.outputspeed;

	if (stopFlag == 0)
	{
//		if (singlePos < 2)//==0)//全向走法
//		{
//                  l_front_setspeed = (float)(carYSpeed/1.732 + carXSpeed + Angle.outputspeed); 
//                   r_front_setspeed = (float)(-carYSpeed/1.732 + carXSpeed + Angle.outputspeed);
//                  rear_setspeed = (float)(-carXSpeed + Angle.outputspeed);
//		}
//		else//载体坐标Y轴行动
//		{
//			l_front_setspeed = (float)(carYSpeed/1.732 );       r_front_setspeed = (float)(-carYSpeed/1.732);
//			rear_setspeed = (float)(0);
//		}
              l_front_setspeed = (float)(carYSpeed*1.732/2 + 0.5*carXSpeed + Angle.outputspeed); 
              r_front_setspeed = (float)(-carYSpeed*1.732/2 + 0.5*carXSpeed + Angle.outputspeed);
              rear_setspeed = (float)(-carXSpeed + Angle.outputspeed);
              //l_front_setspeed = (float)(carYSpeed/1.732 + carXSpeed/3 + Angle.outputspeed/3); 
              //r_front_setspeed = (float)(-carYSpeed/1.732 + carXSpeed/3 + Angle.outputspeed/3);
              //rear_setspeed = (float)(-2*carXSpeed/3 + Angle.outputspeed/3);
	}
	else { l_front_setspeed = 0; r_front_setspeed = 0; rear_setspeed = 0; }//轮子刹车

	//给固定期望速度 (平时注释掉)
//	  l_front_setspeed=1.0;r_front_setspeed=1.0;rear_setspeed=1.0;

}


/*******************************************************************************************************************************************************************/
void Speed_calculate()
{
	//速度PID计算
	static float  L_F_PreError[50] = { 0 };
	static float  R_F_PreError[50] = { 0 };
	static float  Rear_PreError[50] = { 0 };

	uint8 i;
	/*************************************前两轮的PID********************************************************/

	L_F_PreError[0] = (l_front_setspeed - Left_front_CarSpeed) * 1000;
	speed.L_front_ControlIntegral += L_F_PreError[0];
	speed.L_front_ControlIntegral = PWM_Limit(speed.L_front_ControlIntegral, 3000);
	for (i = 19; i > 0; i--)
		L_F_PreError[i] = L_F_PreError[i - 1];

	R_F_PreError[0] = (r_front_setspeed - Right_front_CarSpeed) * 1000;
	speed.R_front_ControlIntegral += R_F_PreError[0];
	speed.R_front_ControlIntegral = PWM_Limit(speed.R_front_ControlIntegral, 3000);
	for (i = 19; i > 0; i--)
		R_F_PreError[i] = R_F_PreError[i - 1];

	//左前电机占空比更新///减小了最大值
	L_front_SpeedControlOutUpdata = Left_front_MotorPID.P * L_F_PreError[0] + Left_front_MotorPID.I * speed.L_front_ControlIntegral + Left_front_MotorPID.D * (L_F_PreError[0] - L_F_PreError[2]);  //闭环输出
	L_front_SpeedControlOutUpdata = PWM_Limit(L_front_SpeedControlOutUpdata, 6000);
	//右前电机占空比更新
	R_front_SpeedControlOutUpdata = Right_front_MotorPID.P * R_F_PreError[0] + Right_front_MotorPID.I * speed.R_front_ControlIntegral + Right_front_MotorPID.D * (R_F_PreError[0] - R_F_PreError[2]);
	R_front_SpeedControlOutUpdata = PWM_Limit(R_front_SpeedControlOutUpdata, 6000);
	if (L_front_SpeedControlOutUpdata >= 0)
	{
		MotorPwm[0] = (int16)(L_front_SpeedControlOutUpdata > speed.L_front_Bigeest ? speed.L_front_Bigeest : L_front_SpeedControlOutUpdata);
		MotorPwm[1] = 0;
	}
	if (L_front_SpeedControlOutUpdata < 0)
	{
		MotorPwm[0] = 0;
		MotorPwm[1] = (int16)(-L_front_SpeedControlOutUpdata > speed.L_front_Bigeest ? speed.L_front_Bigeest : -L_front_SpeedControlOutUpdata);
	}
	if (R_front_SpeedControlOutUpdata >= 0)
	{
		MotorPwm[2] = (int16)(R_front_SpeedControlOutUpdata > speed.R_front_Bigeest ? speed.R_front_Bigeest : R_front_SpeedControlOutUpdata);
		MotorPwm[3] = 0;
	}
	if (R_front_SpeedControlOutUpdata < 0)
	{
		MotorPwm[2] = 0;
		MotorPwm[3] = (int16)(-R_front_SpeedControlOutUpdata > speed.R_front_Bigeest ? speed.R_front_Bigeest : -R_front_SpeedControlOutUpdata);
	}

	/*************************************后轮的PID********************************************************/

	Rear_PreError[0] = (rear_setspeed - Rear_CarSpeed) * 1000;
	speed.Rear_ControlIntegral += Rear_PreError[0];
	speed.Rear_ControlIntegral = PWM_Limit(speed.Rear_ControlIntegral, 3000);
	for (i = 19; i > 0; i--)
		Rear_PreError[i] = Rear_PreError[i - 1];

	//后轮占空比更新
	Rear_SpeedControlOutUpdata = Rear_MotorPID.P * Rear_PreError[0] + Rear_MotorPID.I * speed.Rear_ControlIntegral + Rear_MotorPID.D * (Rear_PreError[0] - Rear_PreError[2]);  //闭环输出
	Rear_SpeedControlOutUpdata = PWM_Limit(Rear_SpeedControlOutUpdata, 6000);
	if (Rear_SpeedControlOutUpdata >= 0)
	{
		MotorPwm[4] = (int16)(Rear_SpeedControlOutUpdata > speed.Rear_Bigeest ? speed.Rear_Bigeest : Rear_SpeedControlOutUpdata);
		MotorPwm[5] = 0;
	}
	if (Rear_SpeedControlOutUpdata < 0)
	{
		MotorPwm[4] = 0;
		MotorPwm[5] = (int16)(-Rear_SpeedControlOutUpdata > speed.Rear_Bigeest ? speed.Rear_Bigeest : -Rear_SpeedControlOutUpdata);
	}
	//更新PWM占空比
	MotorPwmFlash(MotorPwm);
}

/*******************************************************************************************************************************************************************/


