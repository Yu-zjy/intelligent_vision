#include "position.h" 

//����Ŀ��λ�úͺ���ǵ��趨���� ��control.c�ļ�λ�û������б�����
/**********debug�ĺ궨��***********/
//#define DEBUGIMAGEPARAMETER
//#define DSPLAYRESULT
//#define DEBUGANGLEPLAN
#define USE_CONFICINET

uint8 IsCon_get_l=0,IsCon_get_r=0;//���ֳɹ����
uint8 startCarFlag = 0;//ȷ��������޴�����
uint8 reTrySend = 0;
CARSTATUS_enum stateTop=PUSH_LEFT;//��ʼ״̬PUSH_LEFT FINDLINE
uint8 move_state;//0�����ߣ�1�������ߣ�2�������ߣ�3����
float before_locpicx,before_locpicy;
uint8 start_categorynum;//ʶ����Ŀ�㹻��ֱ�ӽ���
uint8 element_num;
uint8 element_pic_num;
uint8 num_next=0;
extern uint8 ELECTROMAGNET_STATE[5][2],other_picnum[3];

extern uint8 mainCategory;
float back_x,back_y;
/**************************����Ŀ������*****************************************************************************************************************************************/
//����Ӧ�ú���
extern float SetX,SetY;
extern float angleSet;
extern findline_TypeDef Findline;
extern uint8 road_type;
extern uint8 stopFlag;
int terify_flag=1;
uint8 round_finish=1;
extern uint8 imageProcess_flag,imageProcessFinish_flag,smotorProcessFinish_flag,imageProcess_found,roundabout_type;
extern uint8 angle_correct,angle_correct_finish,cross_task,letter_num_Find,pic_clear,uart_derection;
extern uint8 back_finish;
extern float HIGHEST_SPEEDX,HIGHEST_SPEEDY;
extern uint8 corretedImageFlag;
extern uint8 startline_stop_flag;


uint8 debugStr[80] ={0};
uint8 numPosition=0;//��̬�滮��  ��ǰĿ����������


void Set_position(void)
{
//  static int turnsFlag=0;
  switch(stateTop)
  {
  case START:
    if(terify_flag)
    {
      terify_flag=0;
      stopFlag=1;
      positionPIDFlag=1;
      lowSpeedPID();
      reTrySend = 0;
      timeTest_5ms=0;
    }
    else if( reTrySend == 0)
    {
      rt_mb_send(buzzer_mailbox, 50);
      //����opneart����
      uart_write_byte(OPENMINIUART_L, 0x01);
      uart_write_byte(OPENMINIUART_R, 0x01);
      timeTest_5ms = 0;
      reTrySend = 1;
//      identifyFlag=1;
//      ackFlag=1;
    }
    else if(timeTest_5ms%100==0 && (IsCon_get_l==0||IsCon_get_r==0) && reTrySend==1)//��ʱ�ش����� ��δ�յ�ȷ��֡
    {
      rt_mb_send(buzzer_mailbox, 50);
      //�ط�����opneart����
      uart_write_byte(OPENMINIUART_L, 0x01);
      uart_write_byte(OPENMINIUART_R, 0x01);
    }
    else if(IsCon_get_l&&IsCon_get_r)
    {
      SetX=0;
      SetY=0.3;
      stopFlag=0;
      if(encoder.Y>0.2)
      {
        IsCon_get_l=0;
        IsCon_get_r=0;
        terify_flag=1;
        stateTop=FINDLINE;
        startline_stop_flag=1;
        ackFlag=0;
      }
    }
    break;
  case FINDLINE:
    if(terify_flag)
    {
//      terify_flag=0;
      stopFlag = 0;
      lowSpeedPID();
      angleSpeed=1.0f;
      positionPIDFlag=0;// �ر�λ�� PID������ͼ���ƫ��
      
      move_state=0;// �˶�ģʽ��0-������ʻ
      timeTest_5ms=0;
      imageProcess_flag=1;// ����ͼ������������ͷ�ɼ�����ͼ��ͼ������ͨ�� road_type ����
      find_pic=0;// ����Ŀ��ʶ���־
      curve_pic=0;// ���������־
    }
    else if(startline_stop_flag==2)
    {
      stopFlag = 1;
      find_pic=0;
      terify_flag=1;
      stateTop=BACKFORSTART;
      start_categorynum=0;
      encoder.Y=0;
      encoder.X=0;
    }
//    else if(roundabout_type==4&&round_finish)//��⵽������roundabout_type==4��ʱ�����뻷��Ŀ��ʶ��״̬
//    {
//      stopFlag=1;// ͣ��
//      imageProcess_flag=0;// �ر�ͼ����
//      round_finish=0;
//      terify_flag=1;
//      stateTop=FINDROUNDPIC;
//      if(road_type==ROUNDABOUTL||cross_task==1) 
//      {
//        find_pic=1;
//        carry_flag=1;
//      }
//      else 
//      {
//        find_pic=2;
//        carry_flag=2;
//      }
//    }
//    else if(find_pic!=0&&startline_stop_flag==1)//������ͷʶ��Ŀ�꣨�����֡���־��ʱ��find_pic ������Ϊ����ֵ������״̬�л��� FACEPIC��
//    {
//      terify_flag=1;
//      stateTop=FACEPIC; // �л�������Ŀ��״̬
//      angleSpeed=0.3; // ���ͽ��ٶȣ���ϸ������
//      if(find_pic==2) 
//      {
//        uart_write_byte(OPENMINIUART_R, 0x06);// ������ͷʶ��ָ��
//        carry_flag=2;
//      }
//      else 
//      {
//        uart_write_byte(OPENMINIUART_L, 0x06);// ������ͷʶ��ָ��
//        carry_flag=1;
//      }
//    }
//    else if(road_type==LOSE)//�� road_type == LOSE��������ʧ��������������������ͣ����
//    {
//      stopFlag=1;
//      rt_mb_send(buzzer_mailbox, 50);
//    }
    if(roundabout_type==0&&round_finish==0&&timeTest_5ms>100) round_finish=1;
    break;
    case APPROACH_BOX:
    if (terify_flag) {
        terify_flag = 0;
//        positionPIDFlag = 2;
        anglePIDFlag = 1;
        timeTest_5ms = 0;
    }     
    else if (zero_flag)//30,100
    {
      // ����Ŀ�긽��
        // ֹͣ��������̬
      approachBoxState = 2;
      positionPIDFlag=1;
      stopFlag=1;
//        angleSet = 0; // ������ͷ����
//        SetX = encoder.X;
//        SetY = encoder.Y;
        stateTop = ADJUST_POSE;
        terify_flag=1;
        
        timeTest_5ms = 0; // ���ó�ʱ��ʱ��
    }
    break;
case ADJUST_POSE:
    if (terify_flag) {
        terify_flag = 0;
      //stopFlag = 0;
        positionPIDFlag = 1;
        angleSet = 0; // ������ͷ����
        SetX = encoder.X;
        SetY = encoder.Y;
        // �ȴ�������ͷ���ݻ�ʱ
        timeTest_5ms = 0;
       back_x=encoder.X;
       back_y = encoder.Y;
       angleSet=0;
        
    } else if (timeTest_5ms > 2000) { // ��ʱ10�루����5ms���ڣ�
        // Ĭ��������
      stopFlag = 0;
        stateTop = PUSH_LEFT;
        terify_flag = 1;
    }
    break;
// ---------------------- PUSH_LEFT �ֲ�ʵ�� -----------------
case PUSH_LEFT:
    if (terify_flag) {
        terify_flag = 0;
        // ��һ�׶Σ������Һ����ƶ�
        SetX = encoder.X +0.8;  // ����ƫ��0.5m
        SetY = encoder.Y;        // Y�ᱣ��
        angleSet=0;
         if (fabs(encoder.X - SetX) < 0.5) { // ��ȷ�������Ŀ��
        terify_flag = 1; // ����״̬�л�
        stateTop = MOVE_RIGHT;   // ��������ƶ�״̬
        timeTest_5ms = 0;
    }

        }
    break;
case MOVE_RIGHT:
    if (terify_flag) {
        terify_flag = 0;
        // �ڶ��׶Σ�����ǰ�����ƶ�
        angleSet=0;
        SetY = encoder.Y + 0.8;  // �����ƶ�0.5m
        SetX = encoder.X;
        angleSet=0;
        stateTop = MOVE_FORWARD_PUSH;
        timeTest_5ms = 0;
    }
    break;
case MOVE_FORWARD_PUSH:
    if (terify_flag) {
        terify_flag = 0;
        // �����׶Σ���������
        SetX = encoder.X - 1.2;  // ����0.6m
        stateTop = PUSH_LEFT_STEP;
        timeTest_5ms = 0;
    } else if (fabs(encoder.Y - SetY) < 0.4) { // ��ȷ��������Ŀ��
        terify_flag = 1;
    }
    break;
case PUSH_LEFT_STEP:
    if (terify_flag) {
        terify_flag = 0;
        // ���ر���λ��
        SetX = back_x;
        SetY = back_y;
        stateTop = RETURN_TRACK;
    } else if (fabs(encoder.X - SetX) < 0.7) {
        terify_flag = 1;
    }
    break;
case PUSH_RIGHT:
    if (terify_flag) {
        terify_flag = 0;
        // �����ƶ�0.5�ף�X���򣩣�����ǰ�˶�0.5�ף�Y���򣩣������������
        SetX = encoder.X - 0.5;  // ������PUSH_LEFT�෴
        SetY = encoder.Y + 0.5;  // ������ǰ�ƶ���ͬ����
        timeTest_5ms = 0;
    } else if (encoder.X <= SetX) {  // ����Ƿ񵽴����Ŀ��λ�ã�X��С����
        // �����������������
        SetY = encoder.Y;          // ����Y��λ��
        SetX += 0.6;               // �������䣨������PUSH_LEFT�෴��
        terify_flag=1;
        stateTop = RETURN_TRACK;   // ���뷵������״̬
    }
    break;
    case RETURN_TRACK:
    if (terify_flag) {
        terify_flag = 0;
        // ���ر��ݵĳ�ʼλ��
        SetX = back_x;
        SetY = back_y;
    } else if ((fabs(encoder.X - SetX)<0.3 )&&(fabs(encoder.Y - SetY)< 0.3)) {
        // �ָ�Ѳ��
        stateTop = FINDLINE;
        approachBoxState = 0;
    }
    break;
               
  case FACEPIC:
    if(terify_flag)
    {
      terify_flag=0;
      angleSpeed=0.2;
      timeTest_5ms=0;
      imageProcessFinish_flag=0;
      imageProcess_found=0;
      if(find_pic==1) uart_write_byte(OPENMINIUART_L, 0x06);
      else uart_write_byte(OPENMINIUART_R, 0x06);
    }
    else if((imageProcess_found)||(road_type==CURVE&&timeTest_5ms>600)||(road_type==STRAIGHT&&timeTest_5ms>400))//imageProcessFinish_flag&&
    {
      terify_flag=1;
      if(imageProcess_found) stateTop=FINDPIC;
      else 
      {
        stateTop=FINDLINE;
        find_pic=0;
        curve_pic=0;
      }
      imageProcessFinish_flag=0;
      imageProcess_found=0;
    }
    if(curve_pic)
    {
      if(timeTest_5ms>200&&move_state!=4) 
      {
        move_state=4;
        timeTest_5ms=0;
      }
    }
    else 
    {
      move_state=4;
    }
    letter_num_Find=1;
    break;
    
  case FINDPIC:
    if(terify_flag)
    {
      terify_flag=0;
      imageProcess_flag=0;
      angleSet=gyro.TurnAngle_Integral;
      imageProcessFinish_flag=0;
      imageProcess_found=0;
      SetX=encoder.X;
      SetY=encoder.Y;
      positionPIDFlag=1;
      stopFlag=0;
      if(find_pic==1) uart_write_byte(OPENMINIUART_L, 0x06);
      else uart_write_byte(OPENMINIUART_R, 0x06);
      timeTest_5ms=0;
    }
    else if(my_sqrt((encoder.X-SetX)*(encoder.X-SetX)+(encoder.Y-SetY)*(encoder.Y-SetY))<0.02&&imageProcessFinish_flag)
    {
      ackFlag=0;
      rt_mb_send(buzzer_mailbox, 50);
      uart_write_byte(OPENMINIUART_L, 0x5A);
      uart_write_byte(OPENMINIUART_R, 0x5A);
      imageProcessFinish_flag=0;
      terify_flag=1;
      if(imageProcess_found) stateTop=TARGET_IDENTIFY;
      else 
      {
        stateTop=FINDLINE;
        find_pic=0;
        curve_pic=0;
      }
    }
    else if(timeTest_5ms%15 == 0 && fabs(gyro.TurnAngle_Integral-angleSet)<1)//����ͼ����Ƶ��
    {    
      letter_num_Find=1;
    }
    break;
    
  case TARGET_IDENTIFY:
    if(terify_flag)
    {
      uart_write_byte(OPENMINIUART_L, 0x5A);
      uart_write_byte(OPENMINIUART_R, 0x5A);
      terify_flag=0;
      positionPIDFlag=1;
      SetX=encoder.X;
      SetY=encoder.Y;
      stopFlag=1;
      timeTest_5ms=0;
      stopFlag = 1;
      reTrySend = 0;
      identifyFlag=0;
    }
    else if(reTrySend == 0)
    {
      rt_mb_send(buzzer_mailbox, 50);
      //����opneart����
      if(roundabout_type==0) //ֱ��ʶ��
      {
        if(find_pic==2) uart_write_byte(OPENMINIUART_R, 0x02);
        else uart_write_byte(OPENMINIUART_L, 0x02);
      }
      else //Ԫ��ʶ��
      {
        if(find_pic==2) uart_write_byte(OPENMINIUART_R, 0x05);
        else uart_write_byte(OPENMINIUART_L, 0x05);
      }
      timeTest_5ms = 501;
      reTrySend = 1;
      ackFlag=0;
    }
    else if(timeTest_5ms%100==0 && identifyFlag==0 && reTrySend==1)//��ʱ�ش����� ��δ�յ�ȷ��֡
    {
      rt_mb_send(buzzer_mailbox, 50);
      //�ط�����opneart����
      if(roundabout_type==0) //ֱ��ʶ��
      {
        if(find_pic==2) uart_write_byte(OPENMINIUART_R, 0x02);
        else uart_write_byte(OPENMINIUART_L, 0x02);
      }
      else //Ԫ��ʶ��
      {
        if(find_pic==2) uart_write_byte(OPENMINIUART_R, 0x05);
        else uart_write_byte(OPENMINIUART_L, 0x05);
      }
    }
    else if(identifyFlag&&ackFlag)
    {
      identifyFlag=0;
      ackFlag=0;
      terify_flag=1;
      stateTop=TARGET_CARRY;
      uart_write_byte(OPENMINIUART_L, 0x5A);
      uart_write_byte(OPENMINIUART_R, 0x5A);
    }
    break;
    
  case TARGET_CARRY:
    if(terify_flag)
    {
      terify_flag=0;
      timeTest_5ms=0;
      rt_sem_release(smotor_sem);
    }
    else if(smotorProcessFinish_flag)
    {
      smotorProcessFinish_flag=0;
      terify_flag=1;
      if(roundabout_type==0&&cross_task==0) 
      {
        stateTop=FINDLINE;
        find_pic=0;
        curve_pic=0;
        carry_flag=0;
      }
      else stateTop=CORRECT_PIC;
      mainCategory=0;
    }
    break;
    
  case FINDROUNDPIC:
    if(terify_flag)
    {
      terify_flag=0;
      timeTest_5ms=0;
      imageProcessFinish_flag=0;
      imageProcess_found=0;
    }
    else if((imageProcess_found||imageProcessFinish_flag)&&timeTest_5ms>100)
    {
      ackFlag=0;
      terify_flag=1;
      if(imageProcess_found) stateTop=FINDAGAIN_PIC;
      else 
      {
        stateTop=FINDLINE;
        uart_write_byte(OPENMINIUART_R, 0x5A);
        uart_write_byte(OPENMINIUART_L, 0x5A);
        if(road_type==CROSSING) 
        {
          road_type=CURVE;
          roundabout_type=0;
          cross_task=0;
        }
        find_pic=0;
      }
      imageProcess_found=0;
      imageProcessFinish_flag=0;
    }
    if(find_pic==1) uart_write_byte(OPENMINIUART_L, 0x07);
    else uart_write_byte(OPENMINIUART_R, 0x07);
    break;
    
  case FINDAGAIN_PIC:
    if(terify_flag)
    {
      terify_flag=0;
      positionPIDFlag=1;
      angleSet=gyro.TurnAngle_Integral;
      angleSpeed=1.0;
      imageProcess_flag=0;
      imageProcessFinish_flag=0;
      SetX = encoder.X;
      SetY = encoder.Y;
      timeTest_5ms=0;
      stopFlag=0;
      element_pic_num=0;
    }
    else if(my_sqrt((encoder.X-SetX)*(encoder.X-SetX)+(encoder.Y-SetY)*(encoder.Y-SetY))<0.02&&imageProcessFinish_flag)
    {
      rt_mb_send(buzzer_mailbox, 50);
      ackFlag=0;
      terify_flag=1;
      if(imageProcess_found) 
      {
        roundabout_type=6;
        stateTop=TARGET_IDENTIFY;
      }
      else stateTop=ROUND_BACK;
      imageProcessFinish_flag=0;
      uart_write_byte(OPENMINIUART_R, 0x5A);
      uart_write_byte(OPENMINIUART_L, 0x5A);
      imageProcess_found=0;
    }
    if(timeTest_5ms%15 == 0 && fabs(gyro.TurnAngle_Integral-angleSet)<1)
    {
      if(find_pic==1) uart_write_byte(OPENMINIUART_L, 0x07);
      else uart_write_byte(OPENMINIUART_R, 0x07);
      letter_num_Find=1;
    }
    break;
    
  case ROUND_BACK:
    if(terify_flag)
    {
      terify_flag=0;
      positionPIDFlag=1;
      angleSpeed=1.0;
      imageProcess_flag=0;
      imageProcessFinish_flag=0;
      SetX = before_locpicx;
      SetY = before_locpicy;
      timeTest_5ms=0;
      stopFlag=0;
    }
    else if(my_sqrt((encoder.X-SetX)*(encoder.X-SetX)+(encoder.Y-SetY)*(encoder.Y-SetY))<0.02)
    {
      ackFlag=0;
      terify_flag=1;
      imageProcessFinish_flag=0;
      if(roundabout_type==6)
      {
        stateTop=CROSSING_FIRST;
        element_num=0;
//        if(road_type==CROSSING) stateTop=CROSSING_FIRST;
//        else stateTop=ROUND_LINE;
      }
      else stateTop=FINDLINE;
      round_finish=0;
      imageProcess_found=0;
    }
    break;
    
  case CORRECT_PIC:
    if(terify_flag)
    {
      element_pic_num++;
      uart_write_byte(OPENMINIUART_L, 0x5A);
      uart_write_byte(OPENMINIUART_R, 0x5A);
      terify_flag=0;
      timeTest_5ms=0;
      reTrySend = 0;
    }
    else if(reTrySend == 0)
    {
      rt_mb_send(buzzer_mailbox, 50);
      //����opneart����
      if(find_pic==2) uart_write_byte(OPENMINIUART_R, 0x07);
      else uart_write_byte(OPENMINIUART_L, 0x07);
      ackFlag=0;
      imageProcess_found=0;
      timeTest_5ms = 501;
      reTrySend = 1;
    }
    else if(timeTest_5ms%500==0 && ackFlag==0 && reTrySend==1)//��ʱ�ش����� ��δ�յ�ȷ��֡
    {
      rt_mb_send(buzzer_mailbox, 50);
      //�ط�����opneart����
      if(find_pic==2) uart_write_byte(OPENMINIUART_R, 0x07);
      else uart_write_byte(OPENMINIUART_L, 0x07);
    }
    else if(ackFlag&&reTrySend==1)
    {
      terify_flag=1;
      ackFlag=0;
      uart_write_byte(OPENMINIUART_L, 0x5A);
      uart_write_byte(OPENMINIUART_R, 0x5A);
      if(imageProcess_found&&element_pic_num<10) stateTop=TARGET_IDENTIFY;
      else 
      {
        stateTop=ROUND_BACK;
        round_finish=0;
      }
      imageProcess_found=0;
    }
    break;
    
  case CROSSING_FIRST://���ƻ���Ψһ��������ʼ��ʱȡ��
    if(terify_flag)
    {
      terify_flag=0;
      imageProcess_flag=1;
      positionPIDFlag=0;
      angleSpeed=0.4;
      move_state=0;
      timeTest_5ms=0;
      rt_mb_send(buzzer_mailbox, 50);
      imageProcess_found=0;
      if(find_pic==2) uart_write_byte(OPENMINIUART_L, 0x08);
      else uart_write_byte(OPENMINIUART_R, 0x08);
      letter_num_Find=1;
      stopFlag=0;
    }
    else if(imageProcess_found)
    {
      ackFlag=0;
      terify_flag=1;
      imageProcess_flag=0;
      stateTop=CLASSIFY_LOC;
    }
    else if(roundabout_type==0||element_num==5)//���⿴�����������
    {
      ackFlag=0;
      round_finish=0;
      terify_flag=1;
      find_pic=0;
      carry_flag=0;
      stateTop=FINDLINE;
      uart_write_byte(OPENMINIUART_L, 0x5A);
      uart_write_byte(OPENMINIUART_R, 0x5A);
      other_picnum[0]+=ELECTROMAGNET_STATE[4][1];
      other_picnum[1]+=ELECTROMAGNET_STATE[3][1];
      other_picnum[2]+=ELECTROMAGNET_STATE[2][1];
      for(int16 i=0;i<5;i++)
      {
        ELECTROMAGNET_STATE[i][1]=0;
        ELECTROMAGNET_STATE[i][0]=0;
      }
    }
    break;
    
  case ROUND_LINE:
    if(terify_flag)
    {
      terify_flag=0;
      imageProcess_flag=1;
      positionPIDFlag=0;
      angleSpeed=0.4;
      move_state=0;
      if(find_pic==2) uart_write_byte(OPENMINIUART_L, 0x08);
      else uart_write_byte(OPENMINIUART_R, 0x08);
      timeTest_5ms=0;
      rt_mb_send(buzzer_mailbox, 50);
      imageProcess_found=0;
      letter_num_Find=1;
      stopFlag=0;
    }
    else if(imageProcess_found&&timeTest_5ms>100)
    {
      ackFlag=0;
      terify_flag=1;
      imageProcess_flag=0;
      stateTop=CLASSIFY_LOC;
      if(find_pic==2) uart_write_byte(OPENMINIUART_L, 0x08);
      else uart_write_byte(OPENMINIUART_R, 0x08);
    }
    else if(roundabout_type==0||element_num==5)
    {
      ackFlag=0;
      round_finish=0;
      terify_flag=1;
      find_pic=0;
      uart_write_byte(OPENMINIUART_L, 0x5A);
      uart_write_byte(OPENMINIUART_R, 0x5A);
      stateTop=FINDLINE;
      other_picnum[0]+=ELECTROMAGNET_STATE[4][1];
      other_picnum[1]+=ELECTROMAGNET_STATE[3][1];
      other_picnum[2]+=ELECTROMAGNET_STATE[2][1];
      for(int16 i=0;i<5;i++)
      {
        ELECTROMAGNET_STATE[i][1]=0;
        ELECTROMAGNET_STATE[i][0]=0;
      }
    }
    letter_num_Find=1;
    imageProcess_found=0;
    break;
    
  case CLASSIFY_LOC:
    if(terify_flag)
    {
      if(find_pic==2) 
      {
        carry_flag=1;
        uart_write_byte(OPENMINIUART_L, 0x08);
      }
      else 
      {
        carry_flag=2;
        uart_write_byte(OPENMINIUART_R, 0x08);
      }
      terify_flag=0;
      positionPIDFlag=1;
      angleSpeed=1.0;
      SetX=encoder.X;
      SetY=encoder.Y;
      before_locpicx=encoder.X;
      before_locpicy=encoder.Y;
      timeTest_5ms=0;
      angleSet=gyro.TurnAngle_Integral;
      reTrySend = 0;
      imageProcessFinish_flag=0;
      stopFlag=0;
    }
    else if(reTrySend == 0)
    {
      rt_mb_send(buzzer_mailbox, 50);
      //����opneart����
      if(find_pic==2) uart_write_byte(OPENMINIUART_L, 0x08);
      else uart_write_byte(OPENMINIUART_R, 0x08);
      timeTest_5ms = 501;
      reTrySend = 1;
      ackFlag=0;
    }
    else if(timeTest_5ms%100==0 && ackFlag==0 && reTrySend==1)//��ʱ�ش����� ��δ�յ�ȷ��֡
    {
      rt_mb_send(buzzer_mailbox, 50);
      //�ط�����opneart����
      if(find_pic==2) uart_write_byte(OPENMINIUART_L, 0x08);
      else uart_write_byte(OPENMINIUART_R, 0x08);
    }
    else if(imageProcessFinish_flag||(!imageProcess_found&&timeTest_5ms>700))
    {
      ackFlag=0;
      uart_write_byte(OPENMINIUART_L, 0x5A);
      uart_write_byte(OPENMINIUART_R, 0x5A);
      imageProcessFinish_flag=0;
      terify_flag=1;
      stopFlag=1;
      if(startline_stop_flag<2) 
      {
        if(imageProcess_found) stateTop=ELEMENT_IDENTIFY;
        else stateTop=CROSSING_FIRST;
      }
      else 
      {
        if(imageProcess_found) stateTop=START_IDENTIFY;
        else 
        {
          stateTop=START_BACK;
          find_pic=0;
        }
      }
    }
    if(timeTest_5ms%20==0&&fabs(gyro.TurnAngle_Integral-angleSet)<1) 
    {
      letter_num_Find=1;
      imageProcess_found=0;
    }
    break;
  
  case ELEMENT_IDENTIFY:
    if(terify_flag)
    {
      element_num++;
      uart_write_byte(OPENMINIUART_L, 0x5A);
      uart_write_byte(OPENMINIUART_R, 0x5A);
      terify_flag=0;
      timeTest_5ms=0;
      stopFlag = 1;
      reTrySend = 0;
      identifyFlag=0;
      ackFlag=0;
    }
    else if(reTrySend == 0)
    {
      rt_mb_send(buzzer_mailbox, 50);
      //����opneart����
      if(find_pic==2) uart_write_byte(OPENMINIUART_L, 0x04);
      else uart_write_byte(OPENMINIUART_R, 0x04);
      timeTest_5ms = 501;
      reTrySend = 1;
      ackFlag=0;
    }
    else if(timeTest_5ms%100==0  && reTrySend==1&&identifyFlag==0)//��ʱ�ش����� ��δ�յ�ȷ��֡  && ackFlag==0
    {
      rt_mb_send(buzzer_mailbox, 50);
      //�ط�����opneart����
      if(find_pic==2) uart_write_byte(OPENMINIUART_L, 0x04);
      else uart_write_byte(OPENMINIUART_R, 0x04);
    }
    else if((identifyFlag&&ackFlag)||timeTest_5ms>700)
    {
      identifyFlag=0;
      ackFlag=0;
      terify_flag=1;
      stateTop=TARGET_LAY;
      uart_write_byte(OPENMINIUART_L, 0x5A);
      uart_write_byte(OPENMINIUART_R, 0x5A);
    }
    break;
    
  case TARGET_LAY:
    if(terify_flag)
    {
      terify_flag=0;
      timeTest_5ms=0;
      rt_sem_release(smotor_sem);
    }
    else if(smotorProcessFinish_flag)
    {
      smotorProcessFinish_flag=0;
      terify_flag=1;
      if(pic_clear) 
      {
        stateTop=ELEMENTBACK;
        mainCategory=0;
      }
      else stateTop=TARGET_LAY;
      pic_clear=0;
    }
    break;
    
  case ELEMENTBACK:
    if(terify_flag)
    {
      terify_flag=0;
      positionPIDFlag=1;
      angleSpeed=1.0;
      imageProcess_flag=0;
      imageProcessFinish_flag=0;
      SetX = before_locpicx;
      SetY = before_locpicy;
      timeTest_5ms=0;
      stopFlag=0;
    }
    else if(my_sqrt((encoder.X-SetX)*(encoder.X-SetX)+(encoder.Y-SetY)*(encoder.Y-SetY))<0.02)
    {
      ackFlag=0;
      terify_flag=1;
      imageProcessFinish_flag=0;
      if(find_pic==2) uart_write_byte(OPENMINIUART_L, 0x08);
      else uart_write_byte(OPENMINIUART_R, 0x08);
      stateTop=ROUND_LINE;
      round_finish=0;
      imageProcess_found=0;
    }
    break;
    
  case BACKFORSTART:
    if(terify_flag)
    {
      terify_flag=0;
      imageProcess_flag=0;
      positionPIDFlag=1;
      angleSpeed=1.0;
      SetX=encoder.X;
      SetY=-2.0;
      timeTest_5ms=0;
      angleSet=gyro.TurnAngle_Integral;
      reTrySend = 0;
      imageProcessFinish_flag=0;
      stopFlag=0;
    }
    else if(my_sqrt((encoder.X-SetX)*(encoder.X-SetX)+(encoder.Y-SetY)*(encoder.Y-SetY))<0.02)
    {
      terify_flag=1;
      stateTop=START_FINDRECT;
      stopFlag=1;
    }
    break;
    
  case START_FINDRECT:
    if(terify_flag)
    {
      uart_write_byte(OPENMINIUART_L, 0x08);
      uart_write_byte(OPENMINIUART_R, 0x08);
      terify_flag=0;
      lowSpeedPID();
      angleSpeed=0.5;
      positionPIDFlag=0;
      move_state=0;
      timeTest_5ms=0;
      imageProcess_flag=1;
      imageProcessFinish_flag=0;
      imageProcess_found=0;
      uart_derection=0;
      Findline.angle_err=0;
      stopFlag = 0;
    }
    else if(encoder.Y>0.5||start_categorynum==3)
    {
      terify_flag=1;
      stateTop=STOPFOREND;
    }
    else if(imageProcess_found&&(!num_next||timeTest_5ms>150))
    {
      stopFlag=1;
      imageProcess_flag=0;
      if(uart_derection==2)
      {
        uart_write_byte(OPENMINIUART_L, 0x5A);
        find_pic=1;
      }
      else
      {
        find_pic=2;
        uart_write_byte(OPENMINIUART_R, 0x5A);
      }
      terify_flag=1;
      stateTop=CLASSIFY_LOC;
    }
    letter_num_Find=1;
    imageProcess_found=0;
    uart_derection=0;
    break;
    
  case START_FINDRECTL:
    if(terify_flag)
    {
      uart_write_byte(OPENMINIUART_L, 0x08);
      uart_write_byte(OPENMINIUART_R, 0x08);
      terify_flag=0;
      lowSpeedPID();
      angleSpeed=0.5;
      positionPIDFlag=0;
      move_state=0;
      timeTest_5ms=0;
      imageProcess_flag=1;
      imageProcessFinish_flag=0;
      imageProcess_found=0;
      uart_derection=0;
      Findline.angle_err=0;
      stopFlag = 0;
    }
    else if(encoder.Y>0.5||start_categorynum==3)
    {
      terify_flag=1;
      stateTop=STOPFOREND;
    }
    else if(imageProcess_found&&(uart_derection!=2||timeTest_5ms>150))
    {
      stopFlag=1;
      imageProcess_flag=0;
      if(uart_derection==2)
      {
        uart_write_byte(OPENMINIUART_L, 0x5A);
        find_pic=1;
      }
      else
      {
        find_pic=2;
        uart_write_byte(OPENMINIUART_R, 0x5A);
      }
      terify_flag=1;
      stateTop=CLASSIFY_LOC;
      if(timeTest_5ms<100) num_next=1;
    }
    letter_num_Find=1;
    imageProcess_found=0;
    uart_derection=0;
    break;
    
  case START_FINDRECTR:
    if(terify_flag)
    {
      uart_write_byte(OPENMINIUART_L, 0x08);
      uart_write_byte(OPENMINIUART_R, 0x08);
      terify_flag=0;
      lowSpeedPID();
      angleSpeed=0.5;
      positionPIDFlag=0;
      move_state=0;
      timeTest_5ms=0;
      imageProcess_flag=1;
      imageProcessFinish_flag=0;
      imageProcess_found=0;
      uart_derection=0;
      Findline.angle_err=0;
      stopFlag = 0;
    }
    else if(encoder.Y>0.5||start_categorynum==3)
    {
      terify_flag=1;
      stateTop=STOPFOREND;
      uart_write_byte(OPENMINIUART_L, 0x5A);
      uart_write_byte(OPENMINIUART_R, 0x5A);
    }
    else if(imageProcess_found&&(uart_derection!=1||timeTest_5ms>150))
    {
      stopFlag=1;
      imageProcess_flag=0;
      if(uart_derection==2)
      {
        uart_write_byte(OPENMINIUART_L, 0x5A);
        find_pic=1;
      }
      else
      {
        find_pic=2;
        uart_write_byte(OPENMINIUART_R, 0x5A);
      }
      terify_flag=1;
      stateTop=CLASSIFY_LOC;
      if(timeTest_5ms<100) num_next=1;
    }
    letter_num_Find=1;
    imageProcess_found=0;
    uart_derection=0;
    break;
  
  case START_IDENTIFY:
    if(terify_flag)
    {
      uart_write_byte(OPENMINIUART_L, 0x5A);
      uart_write_byte(OPENMINIUART_R, 0x5A);
      terify_flag=0;
      timeTest_5ms=0;
      stopFlag = 1;
      reTrySend = 0;
      identifyFlag=0;
      ackFlag=0;
    }
    else if(reTrySend == 0)
    {
      rt_mb_send(buzzer_mailbox, 50);
      //����opneart����
      if(find_pic==2) uart_write_byte(OPENMINIUART_L, 0x03);
      else uart_write_byte(OPENMINIUART_R, 0x03);
      timeTest_5ms = 501;
      reTrySend = 1;
      ackFlag=0;
    }
    else if(timeTest_5ms%100==0  && reTrySend==1&&identifyFlag==0)//��ʱ�ش����� ��δ�յ�ȷ��֡  && ackFlag==0
    {
      rt_mb_send(buzzer_mailbox, 50);
      //�ط�����opneart����
      if(find_pic==2) uart_write_byte(OPENMINIUART_L, 0x03);
      else uart_write_byte(OPENMINIUART_R, 0x03);
    }
    else if(identifyFlag&&ackFlag)
    {
      start_categorynum++;
      identifyFlag=0;
      ackFlag=0;
      terify_flag=1;
      stateTop=START_LAY;
      uart_write_byte(OPENMINIUART_L, 0x5A);
      uart_write_byte(OPENMINIUART_R, 0x5A);
    }
    break;
    
  case START_LAY:
    if(terify_flag)
    {
      terify_flag=0;
      timeTest_5ms=0;
      rt_sem_release(smotor_sem);
    }
    else if(smotorProcessFinish_flag)
    {
      smotorProcessFinish_flag=0;
      terify_flag=1;
      if(pic_clear) 
      {
        stateTop=START_BACK;
        mainCategory=0;
      }
      else stateTop=START_LAY;
      pic_clear=0;
    }
    break;
    
  case START_BACK:
     if(terify_flag)
    {
      terify_flag=0;
      positionPIDFlag=1;
      angleSpeed=1.0;
      imageProcess_flag=0;
      imageProcessFinish_flag=0;
      SetX = before_locpicx;
      SetY = before_locpicy;
      timeTest_5ms=0;
      stopFlag=0;
    }
    else if(my_sqrt((encoder.X-SetX)*(encoder.X-SetX)+(encoder.Y-SetY)*(encoder.Y-SetY))<0.02)
    {
      ackFlag=0;
      terify_flag=1;
      imageProcessFinish_flag=0;
      imageProcess_found=0;
      stopFlag=1;
      if(num_next) find_pic=0;
      switch(find_pic)
      {
      case 2:
        stateTop=START_FINDRECTR;
        break;
        
      case 1:
        stateTop=START_FINDRECTL;
        break;
        
      case 0:
        stateTop=START_FINDRECT;
        break;

      default:
        stateTop=STOPFOREND;
        break;
      }
      uart_derection=0;
      uart_write_byte(OPENMINIUART_L, 0x08);
      uart_write_byte(OPENMINIUART_R, 0x08);
    }
    break;
    
  case STOPFOREND:
    if(terify_flag)
    {
//      terify_flag=0;
//      stopFlag = 0;
//      lowSpeedPID();
//      angleSpeed=0.8;
//      positionPIDFlag=0;
//      move_state=0;
//      timeTest_5ms=0;
//      imageProcess_flag=1;
      terify_flag=0;
      imageProcess_flag=0;
      positionPIDFlag=1;
      angleSpeed=1.0;
      SetX=encoder.X;
      SetY=1.0;
      timeTest_5ms=0;
      angleSet=gyro.TurnAngle_Integral;
      reTrySend = 0;
      imageProcessFinish_flag=0;
      stopFlag=0;
    }
    else if(encoder.Y>0.7)
    {
      rt_mb_send(buzzer_mailbox, 50);
      stopFlag=1;
    }
    uart_write_byte(OPENMINIUART_L, 0x5A);
    uart_write_byte(OPENMINIUART_R, 0x5A);
    SetY=1.0;
    break;
   
  case DEBUG_STATUS:
    if(terify_flag)
    {
      stopFlag = 0;
      SetX=1;
      SetY=1;
    }
  break;
    
  default :;

  }
}



