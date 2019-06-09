#include "MPC5606B.h"

#define RS 			96	// 
#define E			97	// 

#define DT4 		98	// 
#define DT5			99	// 
#define DT6			100	// 
#define DT7			101	// 

volatile uint32_t dly, lly;

void Delay(uint32_t count);

void commandCLCD(unsigned char command);
void dataCLCD(char data);
void initCLCD(void);
void clearCLCD(void);
void shift_display(unsigned char dir, unsigned char x);
void set_coordinate(unsigned char x, unsigned char y);

void writeWord(unsigned char x, unsigned char y, char data);
void writeString(unsigned char x, unsigned char y, char *str);

void writeNum1000(unsigned char x, unsigned char y, unsigned int num);
void writeNum100(unsigned char x, unsigned char y, unsigned int num);
void writeNum10(unsigned char x, unsigned char y, unsigned char num);
void writeNum1(unsigned char x, unsigned char y, unsigned char num);
void writeHex(unsigned char x, unsigned char y, unsigned char hex);

void writeSNum1000(unsigned char x, unsigned char y, int snum);
void writeSNum100(unsigned char x, unsigned char y, int snum);

void Delay(uint32_t count)
{
  for(dly=0;dly<count;dly++) {}
}

void commandCLCD(unsigned char command)
{
	SIU.GPDO[RS].B.PDO = 0;
	
	// High 4Bit
	SIU.GPDO[DT4].B.PDO = (command & 0x10) >> 4;
	SIU.GPDO[DT5].B.PDO = (command & 0x20) >> 5;
	SIU.GPDO[DT6].B.PDO = (command & 0x40) >> 6;
	SIU.GPDO[DT7].B.PDO = (command & 0x80) >> 7;
	SIU.GPDO[E].B.PDO = 1;
	Delay(10);
	SIU.GPDO[E].B.PDO = 0;
	Delay(10);
	
	// Low 4Bit
	SIU.GPDO[DT4].B.PDO = (command & 0x01);
	SIU.GPDO[DT5].B.PDO = (command & 0x02) >> 1;
	SIU.GPDO[DT6].B.PDO = (command & 0x04) >> 2;
	SIU.GPDO[DT7].B.PDO = (command & 0x08) >> 3;
	SIU.GPDO[E].B.PDO = 1;
	Delay(10);
	SIU.GPDO[E].B.PDO = 0; 
	Delay(10);
	
	Delay(7000);
}

void dataCLCD(char data)		/* display a character on text LCD */
{	
	SIU.GPDO[RS].B.PDO = 1;
	
	// High 4Bit
	SIU.GPDO[DT4].B.PDO = (data & 0x10) >> 4;
	SIU.GPDO[DT5].B.PDO = (data & 0x20) >> 5;
	SIU.GPDO[DT6].B.PDO = (data & 0x40) >> 6;
	SIU.GPDO[DT7].B.PDO = (data & 0x80) >> 7;
	SIU.GPDO[E].B.PDO = 1;
	Delay(10);
	SIU.GPDO[E].B.PDO = 0;
	Delay(10);
	
	// Low 4Bit
	SIU.GPDO[DT4].B.PDO = (data & 0x01);
	SIU.GPDO[DT5].B.PDO = (data & 0x02) >> 1;
	SIU.GPDO[DT6].B.PDO = (data & 0x04) >> 2;
	SIU.GPDO[DT7].B.PDO = (data & 0x08) >> 3;
	SIU.GPDO[E].B.PDO = 1;
	Delay(10);
	SIU.GPDO[E].B.PDO = 0; 
	Delay(10);
	
	Delay(7000);
}

void initCLCD(void)
{	
	SIU.PCR[DT4].B.OBE = 1; // PC0 DATA
	SIU.PCR[DT5].B.OBE = 1;	// PC1 DATA
	SIU.PCR[DT6].B.OBE = 1;	// PC2 DATA
	SIU.PCR[DT7].B.OBE = 1;	// PC3 DATA
	SIU.PCR[RS].B.OBE = 1;	// PC14 RS
	SIU.PCR[E].B.OBE = 1;	// PC15 E
	
	Delay(20000);
	commandCLCD(0x28);			// 0010 1000b function set(funcion, 4 bit, 2 line, 5x8 dot)
	Delay(15000);
	commandCLCD(0x28);			// function set(funcion, 4 bit, 2 line, 5x8 dot)
	Delay(15000);
	commandCLCD(0x0C);			// display control(display ON, cursor OFF, blink OFF)
	Delay(15000);
	commandCLCD(0x06);			// entry mode set(increment, not shift)
	Delay(15000);
	commandCLCD(0x01);			// clear display
	Delay(16000);
}

void clearCLCD(void)
{
	commandCLCD(0x01);			// clear display
	Delay(20000);
}

void shift_display(unsigned char dir, unsigned char x)
{
	unsigned char i;
	for(i=0;i<x;i++)
	{
		if(dir == 0)
			commandCLCD(0x10 | 0x0C); 	//¿À¸¥ÂÊ
		else if(dir == 1)
			commandCLCD(0x10 | 0x08);  	// ¿ÞÂÊ
	}
}

void set_coordinate(unsigned char x, unsigned char y)		/* characterÀÇ ÁÂÇ¥¼³Á¤*/
{ 										// DDRAM Address Setting
	if (y == 0)
	{
		commandCLCD(0x80 + x); 			// 1Â°ÁÙ ½ÃÀÛ ÁÂÇ¥ + »ç¿ëÀÚ ÁÂÇ¥
	}
	else if (y == 1)
	{
		commandCLCD(0x80 + 0x40 + x); 	// 2Â°ÁÙ ½ÃÀÛ ÁÂÇ¥ + »ç¿ëÀÚ ÁÂÇ¥
	}
	else if (y == 2)
	{
		commandCLCD(0x80 + 0x14 + x);	// 3Â°ÁÙ ½ÃÀÛ ÁÂÇ¥ + »ç¿ëÀÚ ÁÂÇ¥
	}
	else if (y == 3)
	{
		commandCLCD(0x80 + 0x54 + x);	// 4Â°ÁÙ ½ÃÀÛ ÁÂÇ¥ + »ç¿ëÀÚ ÁÂÇ¥
	}
}

void writeWord(unsigned char x, unsigned char y, char data)
{
	set_coordinate(x, y); 				//ÁÂÇ¥¼³Á¤
	dataCLCD(data); 					// µ¥ÀÌÅÍ ¾²±â
}

void writeString(unsigned char x, unsigned char y, char *str)
{
	set_coordinate(x, y); 				//ÁÂÇ¥¼³Á¤
	while(*str) dataCLCD(*str++); 		// µ¥ÀÌÅÍ ¾²±â
}

void writeNum1000(unsigned char x, unsigned char y, unsigned int num)
{
	char temp[5];
	
	temp[0] = (char)(num / 1000 + '0');
	num = num % 1000;

	temp[1] = (char)(num / 100 + '0');
	num = num % 100;

	temp[2] = (char)(num / 10 + '0');
	num = num % 10;

	temp[3] = (char)(num + '0');

	temp[4] = 0;

	writeString(x, y, temp);
}

void writeNum100(unsigned char x, unsigned char y, unsigned int num)
{
	char temp[4];
	
	temp[0] = (char)(num / 100 + '0');
	num = num % 100;

	temp[1] = (char)(num / 10 + '0');
	num = num % 10;

	temp[2] = (char)(num + '0');

	temp[3] = 0;

	writeString(x, y, temp);
}

void writeNum10(unsigned char x, unsigned char y, unsigned char num)
{
	char temp[3];
	
	temp[0] = (char)(num / 10 + '0');
	num = num % 10;

	temp[1] = (char)(num + '0');

	temp[2] = 0;

	writeString(x, y, temp);
}

void writeNum1(unsigned char x, unsigned char y, unsigned char num)
{
	char temp[2];

	temp[0] = (char)(num + '0');

	temp[1] = 0;

	writeString(x, y, temp);
}

void writeHex(unsigned char x, unsigned char y, unsigned char hex)
{
	unsigned char hexa_h, hexa_l;
	char temp_hexa[3];

	hexa_h = hex >> 4;
	if (hexa_h >= 10) 
	{
		temp_hexa[0] = (char)(hexa_h - 10 + 'A');
	}
	else
	{			
		temp_hexa[0] = (char)(hexa_h + '0');
	}

	hexa_l = hex & 0x0F;
	if (hexa_l >= 10) 
	{
		temp_hexa[1] = (char)(hexa_l - 10 + 'A');
	}
	else			
	{
		temp_hexa[1] = (char)(hexa_l + '0');
	}

	temp_hexa[2]=0;

	writeString(x, y,temp_hexa);
}

void writeSNum1000(unsigned char x, unsigned char y, int snum)
{
	char temp[6];
	unsigned int num;
	
	if(snum >= 0)
	{
		num = (unsigned int)snum;
		temp[0] = '+';
	}
	else
	{
		num = (unsigned int)(-snum);
		temp[0] = '-';
	}
	
	temp[1] = (char)(num / 1000 + '0');
	num = num % 1000;

	temp[2] = (char)(num / 100 + '0');
	num = num % 100;

	temp[3] = (char)(num / 10 + '0');
	num = num % 10;

	temp[4] = (char)(num + '0');

	temp[5] = 0;				// NULL

	writeString(x, y, temp);
}

void writeSNum100(unsigned char x, unsigned char y, int snum)
{
	char temp[5];
	unsigned int num;
	
	if(snum >= 0)
	{
		num = (unsigned int)snum;
		temp[0] = '+';
	}
	else
	{
		num = (unsigned int)(-snum);
		temp[0] = '-';
	}
	
	temp[1] = (char)(num / 100 + '0');
	num = num % 100;

	temp[2] = (char)(num / 10 + '0');
	num = num % 10;

	temp[3] = (char)(num + '0');

	temp[4] = 0;				// NULL

	writeString(x, y, temp);
}
