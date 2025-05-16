#include "functions.h"

int8 buf12[36] = { 0x03, 0xfc,//帧头,使用山外上位机
                   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  //发送数据，如果是float类型，则一个通道采用4字节
                   0xfc, 0x03//帧尾
                 };
uint8 uSendBuf[UartDataNum * 2] = {0};
uint8 FreeCarsDataNum = UartDataNum * 2; //它是通道数 UartDataNum*2(因为数据是Int16型的)


//一些常用函数
/*******************************************************************************/

void uint2Byte(float *target, int8 *buf, int8 beg4);//实际上是将float类型转化为4个字节，用于串口发送数据（float在标准数据格式中是32位）
void virtual_osc (void);//发送数据到上位机
float Limit_float(float  value, float max);
float Limit1_float(float  value, float max, float min);
float Slope_Calculate(uint8 begin,uint8 end,int16 *p);
//float variance(uint8 begin,uint8 end,int16 *p);//方差，往届一些摄像头判定时使用的
int16 abs16 (int16 x);

/*
  freeCar上位机
*/
void push_pc( uint8 chanel, uint16 data )   //原来是uint16 data
{
    uSendBuf[chanel * 2] = data / 256;
    uSendBuf[chanel * 2 + 1] = data % 256;
}
void sendDataToScope( void )    //每次调用该函数一次就发送完一帧的数据，上位机示波器就推进一格。
{
    uint8 i, sum = 0;
    //使用轮询的方式发送数据，当数据未发送，程序停在此处直到发送完成
    uart_write_byte(UART_8, 251);
    uart_write_byte(UART_8, 109);
    uart_write_byte(UART_8, 37);
    sum += ( 251 ); //全部数据加入校验
    sum += ( 109 );
    sum += ( 37 );
    for( i = 0; i < FreeCarsDataNum; i++ )
    {
        uart_write_byte( UART_8, uSendBuf[i] );
        sum += uSendBuf[i]; //全部数据加入校验
    }
    uart_write_byte( UART_8, sum );
}
//
//void freeCars_osc(void)
//{
//    push_pc(0, (int16)(Left_rear_CarSpeed*100));
//    push_pc(1, (int16)(Right_rear_CarSpeed*100));
//    push_pc(2, (int16)(Left_front_CarSpeed*100));
//    push_pc(3, (int16)(Right_front_CarSpeed*100));
//    push_pc(4, (int16)(l_rear_setspeed*100));
//    push_pc(5, (int16)(r_rear_setspeed*100));
//    push_pc(6, (int16)(l_front_setspeed*100));
//    push_pc(7, (int16)(r_front_setspeed*100));
//    sendDataToScope();
//}
/**************************实现函数********************************************
*函数原型: void virtual_osc (void)
*功　　能:    传输数据至虚拟示波器
输入参数：  可以将自己定义的全局变量放进去
输出参数：   buf12内的数据更新并从串口发送
*******************************************************************************/

//void virtual_osc (void)
//{
//    static float float_out = 0;
////    float_out = (float)Angle.outputspeed;
////    uint2Byte(&float_out, buf12, 18);    
////    float_out = (float)gyro.TurnAngle_Integral;
////    uint2Byte(&float_out, buf12, 22);
////    float_out = (float)Left_rear_CarSpeed;//Left_rear_CarSpeed
////    uint2Byte(&float_out, buf12, 2);
////    float_out = (float)Right_rear_CarSpeed;//Right_rear_CarSpeed
////    uint2Byte(&float_out, buf12, 6);
////    float_out = (float)Left_front_CarSpeed;//Left_front_CarSpeed
////    uint2Byte(&float_out, buf12, 10);
////    float_out = (float) Right_front_CarSpeed;   //Right_front_CarSpeed
////    uint2Byte(&float_out, buf12, 14);
//    float_out = (float)Left_rear_CarSpeed;//Left_rear_CarSpeed  编码器测得速度
//    uint2Byte(&float_out, buf12, 2);
//    float_out = (float)Right_rear_CarSpeed;//Right_rear_CarSpeed
//    uint2Byte(&float_out, buf12, 6);
//    float_out = (float)Left_front_CarSpeed;//Left_front_CarSpeed
//    uint2Byte(&float_out, buf12, 10);
//    float_out = (float) Right_front_CarSpeed;   //Right_front_CarSpeed
//    uint2Byte(&float_out, buf12, 14);
//    float_out = (float)l_rear_setspeed;// 各个轮子的期望速度
//    uint2Byte(&float_out, buf12, 18);    
//    float_out = (float)r_rear_setspeed;
//    uint2Byte(&float_out, buf12, 22);
//    float_out = (float)l_front_setspeed;
//    uint2Byte(&float_out, buf12, 26);    
//    float_out = (float)r_front_setspeed;
//    uint2Byte(&float_out, buf12, 30);
////修改时注意buf12的长度，目前的长度是8个通道
//
////    float_out = (float)0;
////    uint2Byte(&float_out, buf12, 26);
////    float_out = (float)0;
////    uint2Byte(&float_out, buf12, 30);
//    for(uint8 buf12_num = 0; buf12_num < 35; buf12_num++)
//    {
//        uart_write_byte(UART_8, buf12[buf12_num]);
//    }
//}

/**************************实现函数********************************************
*函数原型:  void uint2Byte(float *target, int8 *buf, int8 beg4)
*功　　能:    类型转换函数 将float转成字节发送（并不涉及数据类型的转换，只是存储空间的转换，便于发送）
输入参数：    要发送的浮点数，转换后的存储数组，转换后存储数组的存储起始点（偏移量）
输出参数：     四个字节的数组
*******************************************************************************/
void uint2Byte(float *target, int8 *buf, int8 beg4)
{
    int8 *point;
    point = (int8 *)target;
    buf[beg4] = point[0];
    buf[beg4 + 1] = point[1];
    buf[beg4 + 2] = point[2];
    buf[beg4 + 3] = point[3];
}
/**************************实现函数********************************************
*函数原型:   float Limit_float(float  value, float max)
*功　　能:    浮点数限幅
输入参数：       被限幅变量，限制范围绝对值最大值
输出参数：      限幅后数值
*******************************************************************************/
float Limit_float(float  value, float max)
{
    if(value > -max && value < max)
        return value;
    else if( value >=  max)
        value = max;
    else
        value = -max;
    return value;
}
/**************************实现函数********************************************
*函数原型:   float Limit_float(float  value, float max,  float min)
*功　　能:    浮点数限幅
输入参数：       被限幅变量，限制范围绝对值最大值，限制范围绝对值最小值
输出参数：      限幅后数值
*******************************************************************************/
float Limit1_float(float  value, float max, float min)
{
    if((value > min && value < max)||(value > -max && value < -min))
        return value;
    else if( value >=  max)
        value = max;
    else if( value <=  -max)
        value = -max;
    else if( (value >=  0) && (value <= min))
        value = min;
    else if( (value <=  0) && (value >= -min))
        value = -min;  
    return value;
}

/**************************实现函数********************************************
*函数原型:    float Slope_Calculate(uint8 begin,uint8 end,float *p)
*功　　能:    利用最小二乘法算斜率
输入参数：        数组的起始位置和数组的终点位置还有数组地址
输出参数：        斜率
*******************************************************************************/
float Slope_Calculate(uint8 begin,uint8 end,int16 *p)    //最小二乘法拟合斜率
{
   float xsum=0,ysum=0,xysum=0,x2sum=0;
   uint8 i=0;
   float result=0;
   static float resultlast;
   p=p+begin;
   for(i=begin;i<end;i++)
   {
       xsum+=i;
       ysum+=((float)(*p));
       xysum+=i*((float)(*p));
       x2sum+=i*i;
       p=p+1;
   }
  if((end-begin)*x2sum-xsum*xsum) //判断除数是否为零
  {
    result=(float)((end-begin)*xysum-xsum*ysum)/(float)((end-begin)*x2sum-xsum*xsum);
    resultlast=result;
  }
  else
  {
   result=resultlast;
  }
  return result;
}
/**************************实现函数********************************************
*函数原型:    int16 abs (int16 x);
*功　　能:    求绝对值
输入参数：
输出参数：
*******************************************************************************/

int16 abs16 (int16 x)
{
if(x>=0)
    return x;
else
    return -x;
}


/**************************实现函数********************************************
*函数原型:    float variance(uint8 begin,uint8 end,float *p)
*功　　能:    求数组中数的方差
输入参数：   数组起点、数组终点、数组地址
输出参数：   方差
*******************************************************************************/
//float variance(uint8 begin,uint8 end,int16 *p)
//{
//float EX=0,EX2=0;
//int16 ex_=0,ex_2=0;
//p=p+begin;
//for(uint8 i=begin;i<end;i++)
//    {
//        ex_+=*p;
//        ex_2+=(*p)*(*p);
//        p=p+1;
//    }
//    EX=(float)(ex_)/(end-begin);
//    EX2/=(float)(ex_2)/(end-begin);
//    return (EX*EX-EX2);
//}
















