#ifndef _MYOLED_h
#define _MYOLED_h

#include "headfile.h"



void MyOLED_Init(void);
void MyOLED_WrCmd(uint8 cmd);
void MyOLED_WrDat(uint8 data);
void MyOLED_Set_Pos(uint8 x, uint8 y);
void MyOLED_Fill(uint8 bmp_data);
void MyOLED_P6x8Str(uint8 x, uint8 y, uint8 ch[]);
void MyOLED_PrintFloatValue(unsigned char x, unsigned char y, float data);
void MyOLED_Print_Num1(uint8 x, uint8 y, int16 num);
void MyOLED_HEXACSII(uint16 hex, uint8 *Print);
void MyOLED_Put6x8Char(unsigned char x, unsigned char y, unsigned char ch);

      
#endif