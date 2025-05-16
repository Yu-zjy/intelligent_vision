#include "myOled.h"

#define OLED_DC(x)   gpio_set (OLED_DC_PIN	, x)
#define OLED_RST(x)  gpio_set (OLED_RST_PIN	, x)
#define OLED_CS(x)   gpio_set (OLED_CS_PIN  , x)
uint32 oledBund;
uint8 recData;
void MyOLED_Init(void)
{
      oled_init();
}

void MyOLED_WrCmd(uint8 cmd)
{
  oled_wrcmd(cmd);
}

void MyOLED_WrDat(uint8 data)
{
        oled_wrdat(data);
}

void MyOLED_Set_Pos(uint8 x, uint8 y)
{
    MyOLED_WrCmd(0xb0 + y);
    MyOLED_WrCmd(((x & 0xf0) >> 4) | 0x10);
    MyOLED_WrCmd((x & 0x0f) | 0x00);
}

void MyOLED_Fill(uint8 bmp_data)
{
    uint8 y, x;
    for(y = 0; y < 8; y++)
    {
        MyOLED_WrCmd(0xb0 + y);
        MyOLED_WrCmd(0x01);
        MyOLED_WrCmd(0x10);
        for(x = 0; x < 128; x++)	MyOLED_WrDat(bmp_data);
    }
}

void MyOLED_P6x8Str(uint8 x, uint8 y, uint8 ch[])
{
    uint8 c = 0, i = 0, j = 0;
    while (ch[j] != '\0')
    {
        c = ch[j] - 32;
        if(x > 126)
        {
            x = 0;
            y++;
        }
        MyOLED_Set_Pos(x, y);
        for(i = 0; i < 6; i++)	MyOLED_WrDat(F6x8[c][i]);
        x += 6;
        j++;
    }
}

void MyOLED_PrintFloatValue(unsigned char x, unsigned char y, float data)
{
	int temp;

	temp = (int)data;
	MyOLED_Put6x8Char(x, y, ((temp % 1000) / 100) + 48);
	MyOLED_Put6x8Char(x + 6, y, ((temp % 100) / 10) + 48);
	MyOLED_Put6x8Char(x + 12, y, (temp % 10) + 48);

	MyOLED_Put6x8Char(x + 18, y, '.');

	data = data * 100000 + 1;
	temp = (int)data;
	temp %= 100000;
	MyOLED_Put6x8Char(x + 24, y, (temp / 10000) + 48);
	MyOLED_Put6x8Char(x + 30, y, ((temp % 10000) / 1000) + 48);
	MyOLED_Put6x8Char(x + 36, y, ((temp % 1000) / 100) + 48);
}

void MyOLED_Print_Num1(uint8 x, uint8 y, int16 num)
{
    uint8_t *ch1, ch[7];
    if(num < 0)
    {
        num = -num;
        MyOLED_P6x8Str(x, y, (uint8 *)"-");
    }
    else         MyOLED_P6x8Str(x, y, (uint8 *)" ");
    x += 6;
    MyOLED_HEXACSII(num, ch);
    ch1 = &ch[1];
    while(*ch1 != '\0')
    {
        MyOLED_P6x8Str(x, y, ch1);	//ÏÔÊ¾Êý×Ö
        x += 6;
        ch1++;
    }
}
void MyOLED_HEXACSII(uint16 hex, uint8 *Print)
{
    uint8 hexcheck ;
    uint8 TEMP ;
    TEMP = 6 ;
    Print[TEMP ] = '\0';
    while(TEMP)
    {
        TEMP -- ;
        hexcheck  =  hex % 10 ;
        hex   /= 10 ;
        Print[TEMP ]  = hexcheck + 0x30 ;
    }
}

void MyOLED_Put6x8Char(unsigned char x, unsigned char y, unsigned char ch)
{
	unsigned char c = 0, i = 0;

	c = ch - 32;

	if(x > 122)
	{
		x = 0;
		y++;
	}
	MyOLED_Set_Pos(x, y);

	for(i = 0 ; i < 6 ; i++)
		MyOLED_WrDat(F6x8[c][i]);
}

