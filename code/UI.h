#ifndef __UI_H
#define __UI_H
#include "common.h"
#include "headfile.h"
typedef struct
{
    void (*Disp)(void);
    int cursor[4], level, enter;
} UI_CLASS;

#define CURSOR_POS  ('z' + 1)
void UI_Disp(void);
static void UI_DispUIStrings22(uint8 strings[8][22]);
static void UI_DispUIStrings16(uint8 strings[8][16]);
extern UI_CLASS ui;
extern uint16 fps_cnt[3];
void num_add(float* num, float step);
#endif
