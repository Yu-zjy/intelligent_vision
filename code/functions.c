#include "functions.h"

int8 buf12[36] = { 0x03, 0xfc,//֡ͷ,ʹ��ɽ����λ��
                   0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,  //�������ݣ������float���ͣ���һ��ͨ������4�ֽ�
                   0xfc, 0x03//֡β
                 };
uint8 uSendBuf[UartDataNum * 2] = {0};
uint8 FreeCarsDataNum = UartDataNum * 2; //����ͨ���� UartDataNum*2(��Ϊ������Int16�͵�)


//һЩ���ú���
/*******************************************************************************/

void uint2Byte(float *target, int8 *buf, int8 beg4);//ʵ�����ǽ�float����ת��Ϊ4���ֽڣ����ڴ��ڷ������ݣ�float�ڱ�׼���ݸ�ʽ����32λ��
void virtual_osc (void);//�������ݵ���λ��
float Limit_float(float  value, float max);
float Limit1_float(float  value, float max, float min);
float Slope_Calculate(uint8 begin,uint8 end,int16 *p);
//float variance(uint8 begin,uint8 end,int16 *p);//�������һЩ����ͷ�ж�ʱʹ�õ�
int16 abs16 (int16 x);

/*
  freeCar��λ��
*/
void push_pc( uint8 chanel, uint16 data )   //ԭ����uint16 data
{
    uSendBuf[chanel * 2] = data / 256;
    uSendBuf[chanel * 2 + 1] = data % 256;
}
void sendDataToScope( void )    //ÿ�ε��øú���һ�ξͷ�����һ֡�����ݣ���λ��ʾ�������ƽ�һ��
{
    uint8 i, sum = 0;
    //ʹ����ѯ�ķ�ʽ�������ݣ�������δ���ͣ�����ͣ�ڴ˴�ֱ���������
    uart_write_byte(UART_8, 251);
    uart_write_byte(UART_8, 109);
    uart_write_byte(UART_8, 37);
    sum += ( 251 ); //ȫ�����ݼ���У��
    sum += ( 109 );
    sum += ( 37 );
    for( i = 0; i < FreeCarsDataNum; i++ )
    {
        uart_write_byte( UART_8, uSendBuf[i] );
        sum += uSendBuf[i]; //ȫ�����ݼ���У��
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
/**************************ʵ�ֺ���********************************************
*����ԭ��: void virtual_osc (void)
*��������:    ��������������ʾ����
���������  ���Խ��Լ������ȫ�ֱ����Ž�ȥ
���������   buf12�ڵ����ݸ��²��Ӵ��ڷ���
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
//    float_out = (float)Left_rear_CarSpeed;//Left_rear_CarSpeed  ����������ٶ�
//    uint2Byte(&float_out, buf12, 2);
//    float_out = (float)Right_rear_CarSpeed;//Right_rear_CarSpeed
//    uint2Byte(&float_out, buf12, 6);
//    float_out = (float)Left_front_CarSpeed;//Left_front_CarSpeed
//    uint2Byte(&float_out, buf12, 10);
//    float_out = (float) Right_front_CarSpeed;   //Right_front_CarSpeed
//    uint2Byte(&float_out, buf12, 14);
//    float_out = (float)l_rear_setspeed;// �������ӵ������ٶ�
//    uint2Byte(&float_out, buf12, 18);    
//    float_out = (float)r_rear_setspeed;
//    uint2Byte(&float_out, buf12, 22);
//    float_out = (float)l_front_setspeed;
//    uint2Byte(&float_out, buf12, 26);    
//    float_out = (float)r_front_setspeed;
//    uint2Byte(&float_out, buf12, 30);
////�޸�ʱע��buf12�ĳ��ȣ�Ŀǰ�ĳ�����8��ͨ��
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

/**************************ʵ�ֺ���********************************************
*����ԭ��:  void uint2Byte(float *target, int8 *buf, int8 beg4)
*��������:    ����ת������ ��floatת���ֽڷ��ͣ������漰�������͵�ת����ֻ�Ǵ洢�ռ��ת�������ڷ��ͣ�
���������    Ҫ���͵ĸ�������ת����Ĵ洢���飬ת����洢����Ĵ洢��ʼ�㣨ƫ������
���������     �ĸ��ֽڵ�����
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
/**************************ʵ�ֺ���********************************************
*����ԭ��:   float Limit_float(float  value, float max)
*��������:    �������޷�
���������       ���޷����������Ʒ�Χ����ֵ���ֵ
���������      �޷�����ֵ
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
/**************************ʵ�ֺ���********************************************
*����ԭ��:   float Limit_float(float  value, float max,  float min)
*��������:    �������޷�
���������       ���޷����������Ʒ�Χ����ֵ���ֵ�����Ʒ�Χ����ֵ��Сֵ
���������      �޷�����ֵ
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

/**************************ʵ�ֺ���********************************************
*����ԭ��:    float Slope_Calculate(uint8 begin,uint8 end,float *p)
*��������:    ������С���˷���б��
���������        �������ʼλ�ú�������յ�λ�û��������ַ
���������        б��
*******************************************************************************/
float Slope_Calculate(uint8 begin,uint8 end,int16 *p)    //��С���˷����б��
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
  if((end-begin)*x2sum-xsum*xsum) //�жϳ����Ƿ�Ϊ��
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
/**************************ʵ�ֺ���********************************************
*����ԭ��:    int16 abs (int16 x);
*��������:    �����ֵ
���������
���������
*******************************************************************************/

int16 abs16 (int16 x)
{
if(x>=0)
    return x;
else
    return -x;
}


/**************************ʵ�ֺ���********************************************
*����ԭ��:    float variance(uint8 begin,uint8 end,float *p)
*��������:    �����������ķ���
���������   ������㡢�����յ㡢�����ַ
���������   ����
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
















