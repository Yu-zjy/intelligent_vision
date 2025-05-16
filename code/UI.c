#include "UI.h"
unsigned char ui_page0[8][22] =
{
    "   pageChange        ",
    "   left_rear_pid     ",
    "   right_rear_pid    ",
    "   left_front_pid    ",
    "   right_front_pid   ",
    "   angle_PID         ",
    "   positonX_PID      ",
    "   positonY_PID      ",
};
unsigned char ui_page1[8][22] =
{
    "   pageChange        ",
    "   Set1              ",
    "   Set2              ",
    "   Set3              ",
    "   Set4              ",
    "   Set5              ",
    "   positonX_PID      ",
    "   positonY_PID      ",
};
unsigned char ui_PID[8][16] =
{
    "   PID        ",
    "   P:         ",
    "   I:         ",
    "   D:         ",
    "   speed_set  ",
    "   speed_m    ",
    "   count      ",
    "              "
};
unsigned char ui_anglePID[8][16] =
{
    "   PID        ",
    "   P:         ",
    "   I:    ",
    "   D:         ",
    "   set or not ",
    "              ",
    "             ",
    "              "
};
unsigned char ui_positionPID[8][16] =
{
    "   PID        ",
    "   P:         ",
    "   I:    ",
    "   D:         ",
    "   set or not ",
    "   encoder.X  ",
    "   encoder.Y  ",
    "   stopFlag   "
};
unsigned char ui_SET[8][16] =
{
    "   pageChange  ",
    "   SETX:      ",
    "   SETY:      ",
    "              ",
    "              ",
    "              ",
    "              ",
    "              ",
};
unsigned char ui_page2[8][22] =
{
    "   pageChange        ",
    "                     ",
    "                     ",
    "                     ",
    "                     ",
    "                     ",
    "                     ",
    "                     "
};

unsigned char ui_page3[8][22] =
{
    "   pageChange        ",
    "                     ",
    "                     ",
    "                     ",
    "                     ",
    "                     ",
    "                     ",
    "                     "
};

unsigned char ui_encoder[8][16] =
{
    "    YAWangle  ",
    "              ",
    "turncounter   ",
    "              ",
    "   garage     ",
    "              ",
    "  get width   ",
    "              "
};

//unsigned char ui_PID[8][16] =
//{
//    "   BaseKP:    ",
//    "   BaseKD:    ",
//    "   curveP:    ",
//    "   curveD:    ",
//    "   huanP:     ",
//    "   huanD:     ",
//    "   near:      ",
//    "   far:       "
//};

unsigned char ui_Send[8][16] =
{
    "              ",
    "              ",
    "              ",
    "              ",
    "              ",
    "              ",
    "              ",
    "              "
};
uint16 fps_cnt[3] = {0, 0, 0};
/*
typedef struct
{
    void (*Disp)(void);
    int cursor[4], level, enter;
} UI_CLASS;
*/
UI_CLASS ui =
{
    &UI_Disp,
    {0, 0, 0, 0}, 0, -1
};//修改进入默认界面

static void UI_DispUIStrings22(uint8 strings[8][22])
{
    int i;
    for (i = 0; i < 8; i++)
    {
        if (i == ui.cursor[ui.level])
            strings[i][2] = CURSOR_POS;
        else
            strings[i][2] = ' ';
        if(i == 7)
        {
            strings[i][16] = 'p';
            strings[i][17] = 'a';
            strings[i][18] = 'g';
            strings[i][19] = 'e';
            strings[i][20] = (char)(ui.level + '0');
        }
        MyOLED_P6x8Str(0, i, strings[i]);
    }
}

static void UI_DispUIStrings16(uint8 strings[8][16])
{
    int i;
    for (i = 0; i < 8; i++)
    {
        if (i == ui.cursor[ui.level])
            strings[i][2] = CURSOR_POS;
        else
            strings[i][2] = ' ';
        MyOLED_P6x8Str(0, i, strings[i]);
    }
}


void UI_Disp(void)
{//三级菜单的实现
    switch(ui.level)
    {
    case 0:
        if(ui.enter == -1)//默认页面
        {
            UI_DispUIStrings22(ui_page0);
        }
        else
        {
            if(ui.enter == 0)
            {
                    UI_DispUIStrings22(ui_page0);
            }
            if(ui.enter == 1)
            {
                  UI_DispUIStrings16(ui_PID);
                MyOLED_P6x8Str(81, 0, (uint8 *)"L_R:");
                MyOLED_PrintFloatValue(84, 1, Left_rear_MotorPID.P);
                MyOLED_PrintFloatValue(84, 2, Left_rear_MotorPID.I);
                MyOLED_PrintFloatValue(84, 3, Left_rear_MotorPID.D);
                MyOLED_PrintFloatValue(84, 4, l_rear_setspeed);
                MyOLED_PrintFloatValue(84, 5, Left_rear_CarSpeed);
                MyOLED_PrintFloatValue(84, 6, Left_rear_count);
//                Showimage();
//                MyOLED_P6x8Str(81, 0, (uint8 *)"fps:");
//                MyOLED_Print_Num1(86, 1, fps_cnt[0]);
//                MyOLED_P6x8Str(81, 2, (uint8 *)"err:");
//                MyOLED_Print_Num1(86, 3, DirOutter.Curvature_Final);
//                MyOLED_P6x8Str(81, 4, (uint8 *)"L:");
//                MyOLED_Print_Num1(86, 4, leftcurve_flag);     
//                MyOLED_P6x8Str(81, 5, (uint8 *)"R:");
//                MyOLED_Print_Num1(86, 5, rightcurve_flag);
            }
            else if(ui.enter == 2)
            {
                  UI_DispUIStrings16(ui_PID);
                MyOLED_P6x8Str(81, 0, (uint8 *)"R_R:");
                MyOLED_PrintFloatValue(84, 1, Right_rear_MotorPID.P);
                MyOLED_PrintFloatValue(84, 2, Right_rear_MotorPID.I);
                MyOLED_PrintFloatValue(84, 3, Right_rear_MotorPID.D);
                MyOLED_PrintFloatValue(84, 4, r_rear_setspeed);
                MyOLED_PrintFloatValue(84, 5, Right_rear_CarSpeed);
                MyOLED_PrintFloatValue(84, 6, Right_rear_count);
//                MyOLED_Print_Num1(82, 0, gyro.TurnAngle_Integral);
//                MyOLED_Print_Num1(82, 2, youhuandao_flag);
//                MyOLED_Print_Num1(82, 4, zuohuandao_flag); 
//                MyOLED_Print_Num1(82, 6, gyro.PitchAngle_Integral);
            }
            else if(ui.enter == 3)
            {
                  UI_DispUIStrings16(ui_PID);
                  MyOLED_P6x8Str(81, 0, (uint8 *)"L_F:");
//                MyOLED_PrintFloatValue(84, 0, baseP);
                MyOLED_PrintFloatValue(84, 1, Left_front_MotorPID.P);
                MyOLED_PrintFloatValue(84, 2, Left_front_MotorPID.I);
                MyOLED_PrintFloatValue(84, 3, Left_front_MotorPID.D);
                MyOLED_PrintFloatValue(84, 4, l_front_setspeed);
                MyOLED_PrintFloatValue(84, 5, Left_front_CarSpeed);
                MyOLED_PrintFloatValue(84, 6, Left_front_count);
//                MyOLED_PrintFloatValue(84, 5, huandaoD);
//                MyOLED_PrintFloatValue(84, 6, near_sight);
//                MyOLED_PrintFloatValue(84, 7, far_sight);                
            }
            else if(ui.enter == 4)
            {
                  UI_DispUIStrings16(ui_PID);
                  MyOLED_P6x8Str(81, 0, (uint8 *)"R_F:");
                MyOLED_PrintFloatValue(84, 1, Right_front_MotorPID.P);
                MyOLED_PrintFloatValue(84, 2, Right_front_MotorPID.I);
                MyOLED_PrintFloatValue(84, 3, Right_front_MotorPID.D);
                MyOLED_PrintFloatValue(84, 4, r_front_setspeed);
                MyOLED_PrintFloatValue(84, 5, Right_front_CarSpeed);
                MyOLED_PrintFloatValue(84, 6, Right_front_count);
//			  	MyOLED_Print_Num1(60, 0, Findp.lenth);
                    //MyOLED_Print_Num1(60, 0, r_setspeed*100);
//				MyOLED_Print_Num1(60, 1, l_setspeed*100);
//                                MyOLED_Print_Num1(84, 2, r_sendspeed*100);
//                                MyOLED_Print_Num1(84, 3, l_sendspeed*100);
//                                MyOLED_Print_Num1(84, 5, recrightData*100);
//				MyOLED_Print_Num1(84, 6, recleftData*100);
            }
            else if(ui.enter == 5)
            {
                  UI_DispUIStrings16(ui_anglePID);
                  MyOLED_P6x8Str(81, 0, (uint8 *)"angle");
                MyOLED_PrintFloatValue(84, 1, AnglePID.P);
                MyOLED_PrintFloatValue(84, 2, AnglePID.I);
                MyOLED_PrintFloatValue(84, 3, AnglePID.D);
                MyOLED_PrintFloatValue(84, 4, anglePIDFlag);
//			  	MyOLED_Print_Num1(60, 0, Findp.lenth);
                    //MyOLED_Print_Num1(60, 0, r_setspeed*100);
//				MyOLED_Print_Num1(60, 1, l_setspeed*100);
//                                MyOLED_Print_Num1(84, 2, r_sendspeed*100);
//                                MyOLED_Print_Num1(84, 3, l_sendspeed*100);
//                                MyOLED_Print_Num1(84, 5, recrightData*100);
//				MyOLED_Print_Num1(84, 6, recleftData*100);
            }
            else if(ui.enter == 6)
            {
                  UI_DispUIStrings16(ui_positionPID);
                  MyOLED_P6x8Str(81, 0, (uint8 *)"Xposition");
                MyOLED_PrintFloatValue(84, 1, PosPID_X.P);
                MyOLED_PrintFloatValue(84, 2, PosPID_X.I);
                MyOLED_PrintFloatValue(84, 3, PosPID_X.D);
                MyOLED_PrintFloatValue(84, 4, positionPIDFlag);
                MyOLED_PrintFloatValue(84, 5, encoder.X);
                MyOLED_PrintFloatValue(84, 6, encoder.Y);
                MyOLED_PrintFloatValue(84, 7, stopFlag);                
//			  	MyOLED_Print_Num1(60, 0, Findp.lenth);
                    //MyOLED_Print_Num1(60, 0, r_setspeed*100);
//				MyOLED_Print_Num1(60, 1, l_setspeed*100);
//                                MyOLED_Print_Num1(84, 2, r_sendspeed*100);
//                                MyOLED_Print_Num1(84, 3, l_sendspeed*100);
//                                MyOLED_Print_Num1(84, 5, recrightData*100);
//				MyOLED_Print_Num1(84, 6, recleftData*100);
            }
            else if(ui.enter == 7)
            {
                  UI_DispUIStrings16(ui_positionPID);
                  MyOLED_P6x8Str(81, 0, (uint8 *)"Yposition");
                MyOLED_PrintFloatValue(84, 1, PosPID_Y.P);
                MyOLED_PrintFloatValue(84, 2, PosPID_Y.I);
                MyOLED_PrintFloatValue(84, 3, PosPID_Y.D);
                MyOLED_PrintFloatValue(84, 4, positionPIDFlag);
                MyOLED_PrintFloatValue(84, 5, encoder.X);
                MyOLED_PrintFloatValue(84, 6, encoder.Y);
                MyOLED_PrintFloatValue(84, 7, stopFlag); 
//			  	MyOLED_Print_Num1(60, 0, Findp.lenth);
                    //MyOLED_Print_Num1(60, 0, r_setspeed*100);
//				MyOLED_Print_Num1(60, 1, l_setspeed*100);
//                                MyOLED_Print_Num1(84, 2, r_sendspeed*100);
//                                MyOLED_Print_Num1(84, 3, l_sendspeed*100);
//                                MyOLED_Print_Num1(84, 5, recrightData*100);
//				MyOLED_Print_Num1(84, 6, recleftData*100);
            }
            
        }
        break;
    case 1:
        if(ui.enter == -1)//默认页面
        {
            UI_DispUIStrings22(ui_page1);
        }
        else if(ui.enter == 0)
        {
            UI_DispUIStrings22(ui_page1);
        }
        else if(ui.enter == 1)
        {
                UI_DispUIStrings16(ui_SET);
                MyOLED_P6x8Str(81, 0, (uint8 *)"Set1:");
                MyOLED_PrintFloatValue(84, 1, xSet1);
                MyOLED_PrintFloatValue(84, 2, ySet1);
        }
        else if(ui.enter == 2)
        {
                UI_DispUIStrings16(ui_SET);
                MyOLED_P6x8Str(81, 0, (uint8 *)"Set2:");
                MyOLED_PrintFloatValue(84, 1, xSet2);
                MyOLED_PrintFloatValue(84, 2, ySet2);
        }
        else if(ui.enter == 3)
        {
                UI_DispUIStrings16(ui_SET);
                MyOLED_P6x8Str(81, 0, (uint8 *)"Set3:");
                MyOLED_PrintFloatValue(84, 1, xSet3);
                MyOLED_PrintFloatValue(84, 2, ySet3);
        }
        else if(ui.enter == 4)
        {
                UI_DispUIStrings16(ui_SET);
                MyOLED_P6x8Str(81, 0, (uint8 *)"Set4:");
                MyOLED_PrintFloatValue(84, 1, xSet4);
                MyOLED_PrintFloatValue(84, 2, ySet4);
        }
        else if(ui.enter == 5)
        {
                UI_DispUIStrings16(ui_SET);
                MyOLED_P6x8Str(81, 0, (uint8 *)"Set5:");
                MyOLED_PrintFloatValue(84, 1, xSet5);
                MyOLED_PrintFloatValue(84, 2, ySet5);
        }
        break;
    case 2:
        UI_DispUIStrings22(ui_page2);
        break;
    case 3:
        UI_DispUIStrings22(ui_page3);
        break;
    }
}

void num_add(float* num, float step)
{
  	*num += step;
}
