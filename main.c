#include "MPC5606B.h"
#include "IntcInterrupts.h"

#include "INIT.h"
#include "CLCD.h"
#include "LS7366.h"

#define F_CPU 	64000000
#define T_PIT	0.00001					// 0.00001 = 10us		

#define OFF		0
#define ON		1

#define LINE_B	0
#define LINE_R	1
#define LINE_L	2
#define LINE_N	3

#define DC_MAX	990

#define BRAKE	700
#define RUN		1200

#define PSD_CNTMAX 	7
#define US_CNTMAX  	7

int i = 0;								// for count i
int j = 0;								// for count j

volatile char flagDisp;

volatile char data_btrx;
volatile char flagBTBrake;
volatile char flagBTSend;

volatile uint32_t CPeriod;
///////////////////////////////////////////////////////////////////////////////
volatile unsigned int timerRest = 0;
volatile uint8_t flagCamOnOff = 0;
volatile uint8_t flagTX = 0;
volatile unsigned int flagTimer = 0;
volatile uint8_t flagComplete = 0;
/////////////////////////////////////////////////////////////////////////////////
volatile uint8_t dataPot = 0;			// pot adc value
volatile uint8_t dataBatt = 0;
/////////////////////////////////////////////////////////////////////////////////
volatile uint8_t flagBrakeOnOff;
volatile uint8_t flagBrakeOnOffPrev;
/////////////////////////////////////////////////////////////////////////////////
double contrast = 0;

volatile uint8_t num_line;
volatile uint16_t cam_temp;
volatile uint16_t cam_rawR[130];
volatile uint16_t cam_rawL[130];

uint8_t cam_binR[128];
uint8_t cam_binL[128];
uint8_t cam_binU[256];

uint16_t minR;
uint16_t minL;

uint16_t maxR;
uint16_t maxL;

volatile uint16_t idxMaxR;
volatile uint16_t idxMaxL;
volatile uint16_t idxMinR;
volatile uint16_t idxMinL;

uint16_t cam_thrR;
uint16_t cam_thrL;

volatile uint8_t idxRCC;			// Right Cam Side Center
volatile uint8_t idxLCC;			// Left Cam Side Cetner

volatile uint8_t idxRCS;			// Right Cam Start
volatile uint8_t idxLCE;			// Left Cam End
volatile uint8_t idxRLC;			// Right Left Cam Cross Point idxRCC - idxRCS

volatile uint8_t idxRCL;
volatile uint8_t idxLCL;

volatile uint8_t idxRCL_def;
volatile uint8_t idxLCL_def;

volatile uint8_t idxUC;

volatile uint8_t diffRL;							// Right Left Cam Differnce
volatile uint16_t sizeUnite;
/////////////////////////////////////////////////////////////////////////
uint8_t idxStartLineR;
uint8_t idxEndLineR;

uint8_t idxStartLineL;
uint8_t idxEndLineL;

uint8_t cntBinR;
uint8_t cntBinL;
uint8_t cntBinU;

uint8_t cntR;
uint8_t cntL;
uint8_t cntU;

uint8_t idxLineR[40];
uint8_t idxLineL[40];

uint8_t idxSLineR[10];
uint8_t idxSLineL[10];

uint8_t flagLine_now;
uint8_t flagLine_pre;

int MyPos_now;
int MyPos_pre;
int MyPos_old;

double T_cycle;
////////////////////////Steering Servo///////////////////////////////////////////
double errServo_now;
double errServo_pre;
double errServo_sum;

double P_Servo;
double I_Servo;
double Isat_Servo;
double D_Servo;

uint16_t dutyServo_Center;
uint16_t dutyServo_Max;
uint16_t dutyServo_Min;

uint16_t dutyServo;
/////////////////////////////////////////////////////////////////////////////////
uint16_t cntDesire;
uint16_t cntDesire_def;
//////////////////////DC1 -> Right///////////////////////////////////////////////
uint16_t cntTgt1;

uint32_t cntDC1_now;
uint32_t cntDC1_pre;

long cntDC1;

double errDC1_now;
double errDC1_pre;
double errDC1_sum;

double P_DC1;
double I_DC1;
double Isat_DC1;
double D_DC1;

double rpmDC1;
double velDC1;
//////////////////////DC2 -> LEft/////////////////////////////////////////////////
uint16_t cntTgt2;

uint32_t cntDC2_now;
uint32_t cntDC2_pre;
long cntDC2;

double errDC2_now;
double errDC2_pre;
double errDC2_sum;

double P_DC2;
double I_DC2;
double Isat_DC2;
double D_DC2;

double rpmDC2;
double velDC2;
//////////////////////////////////////////////////////////////////////////
long cntDCAvg;

uint8_t flagDCOnOff;

int dutyMax;
int dutyDC1;
int dutyDC2;
//////////////////////////////////////////////////////////////////////////
double LPF(double alpha, double yPre, double xNow);
uint16_t MAF(uint8_t arraySize, volatile uint16_t* arrayData, uint16_t newData);

// Interrupt Service Routine
void ISR_LIN1_UART_RECV(void);
void ISR_PIT0(void);

void GetPixeldata(void);
void GetPSDdata(void);

void DistanceSensor(void);
void TransformImage(void);
void DetectLine(void);
void SetPosition(void);

void ControlServo(void);
void ControlBrake(void);
void ControlDC(void);

void InitVar(void);
void MainLoop(void);
void SensingOnOff(void);
void BehaviorBTData(void);

void Button1(void);
void Button2(void);
void Button3(void);
void Button4(void);

void DispData1(void);
void DispData2(void);
void DispData3(void);
void DispData4(void);

void BTSendData(void);

//double LPF(double alpha, double yPre, double xNow)
//{
//	return yPre + alpha * (xNow - yPre);
//}
//
//uint16_t MAF(uint8_t arraySize, volatile uint16_t* arrayData, uint16_t newData)
//{
//	uint16_t avgData = 0;	
// 
//	for(i = arraySize - 1; i > 0; i--)
//	{
//		arrayData[i] = arrayData[i - 1];
//		avgData += arrayData[i];
//	}
//	arrayData[0] = newData;
//	avgData += arrayData[0];
// 
//	avgData = avgData / arraySize;
// 
//	return avgData;
//}

void ISR_LIN1_UART_RECV(void)	// Data Reception Flag is Set
{
	data_btrx = (uint8_t)LINFLEX_1.BDRM.B.DATA4;
	
	/* clear the DRF and RMB flags by writing 1 to them */
	LINFLEX_1.UARTSR.B.DRF = 1;
}

void ISR_PIT0(void)		// Camera CLK Period * 519 + Post Processing Period = 
{	
	timerRest = flagTimer % 4;
	if(timerRest == 0)							// 0, 4, 8, 12, 18,...,500, 504, 508, 512, 516
	{
		if(flagTimer == 0)
		{
			SIU.GPDO[1].B.PDO = 1;				// SI1 High
			SIU.GPDO[3].B.PDO = 1;				// SI2 High
		}
	}
	else if(timerRest == 1)						// 1, 5, 9, 13, 19,...,501, 505, 509, 513x, 517x
	{
		if(flagTimer < 513)
		{
			SIU.GPDO[0].B.PDO = 1;				// CLK1 High
			SIU.GPDO[2].B.PDO = 1;				// CLK2 High
		}
	}
	else if(timerRest == 2)						// 2, 6, 10, 14, 20,...,502, 506, 510, 514x, 518x
	{											// 0, 1, 2,  3,  4, ...,125, 126, 127, 128x, 129x
		if(flagTimer == 2)
		{
			SIU.GPDO[1].B.PDO = 0;				// SI1 Low
			SIU.GPDO[3].B.PDO = 0;				// SI2 Low
		}
		else if(flagTimer == 518)
		{
			flagComplete = 1;
			PIT.CH[0].LDVAL.R = (uint32_t) (448000 - 1);	// 519 Cycle Period Set
		}										// 384000 -> 0.006, 448000 -> 0.007, 512000 -> 0.008, 1088000 -> 0.017, 1152000 -> 0.018, 1280000 -> 0.020
		GetPixeldata();							// Get Pixel Data
		num_line++;
	}
	else if(timerRest == 3)						// 3, 7, 11, 15, 19,...,503, 507, 511, 515x, 519x
	{											// 0, 1, 2,  3,  4, ...,125, 126, 127, 128x, 129x
		if(flagTimer < 515)
		{
			SIU.GPDO[0].B.PDO = 0;				// CLK1 Low
			SIU.GPDO[2].B.PDO = 0;				// CLK2 Low
		}
	}
	
	if(flagTimer == 519)						// Post Processing & Control Cycle
	{
		// Change ADC Channel
		ADC_0.NCMR0.R = 0;
		ADC_0.NCMR0.R |= (uint32_t)(0x00000001 << 0);	// Channel 0 Pot
		ADC_0.NCMR0.R |= (uint32_t)(0x00000001 << 4);	// Channel 4 Battery
		
		ADC_0.MCR.B.NSTART = 1;
		/* Wait for last scan to complete */
		while (ADC_0.MSR.B.NSTART == 1) {};
		
		dataPot = (uint8_t)(ADC_0.CDR[0].B.CDATA >> 2);			// POT 8bit ADC
		dataBatt = (uint8_t)(ADC_0.CDR[4].B.CDATA >> 2);		// Battery 8bit ADC
		
		TransformImage();
		DetectLine();
		SetPosition();
		
		if(SIU.GPDI[105].B.PDI == 0x00)			// Toggle 4
		{
			ControlServo();
		}
		
		cntDC1_now = GetEncoderPulse(1);
		cntDC2_now = GetEncoderPulse(2);
		
		cntDC1 = (int)(cntDC1_now - cntDC1_pre);
		cntDC2 = (int)(cntDC2_pre - cntDC2_now);
		
		cntDC1_pre = cntDC1_now;
		cntDC2_pre = cntDC2_now;
		
		if(flagDCOnOff == ON)
		{
			ControlDC();
		}
		
		BTSendData();
		
		flagTimer = 0;
		num_line = 0;
		
		CPeriod = PIT.CH[0].CVAL.R;
		
		// Change ADC Channel
		ADC_0.NCMR0.R = 0;  
		ADC_0.NCMR0.R |= (uint32_t)(0x00000001 << 2);	// Channel 2 Cam Left
		ADC_0.NCMR0.R |= (uint32_t)(0x00000001 << 3);	// Channel 3 Cam Right
		
		PIT.CH[0].LDVAL.R = (uint32_t) (640 - 1);			// 0 ~ 518 Cycle Period Set 10us 
															// 10usec => 640, 20usec => 1280
															// (LDVAL + 1) / 64000000 sec 
	}
	else
	{
		flagTimer++;
	}
	PIT.CH[0].TFLG.R=1;
}

void GetPixeldata(void)
{
	ADC_0.MCR.B.NSTART = 1;
	/* Wait for last scan to complete */
	while (ADC_0.MSR.B.NSTART == 1) {};
	
	cam_rawL[num_line] = (uint8_t)(ADC_0.CDR[2].B.CDATA >> 2);		// CamLeft 8bit ADC
	cam_rawR[num_line] = (uint8_t)(ADC_0.CDR[3].B.CDATA >> 2);		// CamRight 8bit ADC
}

//void GetPSDdata(void)
//{
//	ADC_0.MCR.B.NSTART = 1;
//	/* Wait for last scan to complete */
//	while (ADC_0.MSR.B.NSTART == 1) {};
//
////	psdB_now = (uint16_t)(ADC_0.CDR[4].B.CDATA);		// Bottom PSD 10bit ADC
//	dataPsdC_now = (uint16_t)(ADC_0.CDR[5].B.CDATA);		// Center PSD 10bit ADC
//}

//void DistanceSensor(void)
//{
//	// Change ADC Channel
//	ADC_0.NCMR0.R = 0;
//	ADC_0.NCMR0.R |= (uint32_t)(0x00000001 << 5);	// Channel 5 PSD Center
//	GetPSDdata();
//	
//	if(countPSD < PSD_CNTMAX - 1)				// ex) 0, 1, 2, 3, 4, 5, 6, 7, 8
//	{
//		dataPsdC[countPSD] = dataPsdC_now;
//		countPSD++;
//	}
//	else			// countPSD == PSD_CNTMAX - 1
//	{
//		// Moving Average Filter
//		avgPsdC = MAF(PSD_CNTMAX, &dataPsdC[0], dataPsdC_now);
//		
//		// Low Pass Filter
//		xPsdC_now = avgPsdC;
//		if(flagPSDFirst == 0)
//		{
//			yPsdC_pre = xPsdC_now;
//			flagPSDFirst = 1;
//		}		
//		yPsdC_now = LPF(LPF_PSD, yPsdC_pre, xPsdC_now);
//		yPsdC_pre = yPsdC_now;
//		
////		if(yPsdC_now > PSDThresC) { flagBrakeOnOff = ON; }
////		else { flagBrakeOnOff = OFF; }
//	}
//	
//	dataUS_now = cntEcho;
//	if(countUS < US_CNTMAX - 1)
//	{
//		dataUS[countUS] = dataUS_now;
//		countUS++;
//	}
//	else
//	{
//		// Moving Average Filter
//		avgUS = MAF(US_CNTMAX, &dataUS[0], dataUS_now);
//		
//		// Low Pass Filter
//		xUS_now = avgUS;
//		if(flagUSFirst == 0)
//		{
//			yUS_pre = xUS_now;
//			flagUSFirst = 1;
//		}
//		yUS_now = LPF(LPF_US, yUS_pre, xUS_now);
//		yUS_pre = yUS_now;
//	}
//}

void TransformImage(void)
{
	// Contrast Change
	if(SIU.GPDI[104].B.PDI == 0x00)		// Toggle 3
	{
		contrast = 1.0 / dataPot;
		for(i = 0; i < 128; i++)
		{
			cam_temp = (uint16_t)(contrast * cam_rawR[i] * cam_rawR[i]);
			if(cam_temp > 255){cam_temp = 255;}
			cam_rawR[i] = cam_temp;
			
			cam_temp = (uint16_t)(contrast * cam_rawL[i] * cam_rawL[i]);
			if(cam_temp > 255){cam_temp = 255;}
			cam_rawL[i] = cam_temp;
		}
	}
	
	// Get Max & Min Value
//	maxL = cam_rawL[7];
//	minL = cam_rawL[7];
//	maxR = cam_rawR[7];
//	minR = cam_rawR[7];
//	for(i = 8; i <= 120; i++)
//	{
//		if(cam_rawR[i] > maxR)
//		{
//			maxR = cam_rawR[i];
//			idxMaxR = i;
//		}
//
//		if(cam_rawR[i] < minR)
//		{
//			minR = cam_rawR[i];
//			idxMinR = i;
//		}
//
//		if(cam_rawL[i] > maxL)
//		{
//			maxL = cam_rawL[i];
//			idxMaxL = i;
//		}
//		
//		if(cam_rawL[i] < minL)
//		{
//			minL = cam_rawL[i];
//			idxMinL = i;
//		}
//	}
	
	maxR = cam_rawR[idxLCC];
	minR = cam_rawR[idxLCC];
	for(i = idxLCC + 1; i <= 120; i++)
	{
		if(cam_rawR[i] > maxR)
		{
			maxR = cam_rawR[i];
			idxMaxR = i;
		}

		if(cam_rawR[i] < minR)
		{
			minR = cam_rawR[i];
			idxMinR = i;
		}		
	}
	
	maxL = cam_rawL[idxRCC];
	minL = cam_rawL[idxRCC];
	for(i = idxRCC + 1; i >= 7; i--)
	{
		if(cam_rawL[i] > maxL)
		{
			maxL = cam_rawL[i];
			idxMaxL = i;
		}
		
		if(cam_rawL[i] < minL)
		{
			minL = cam_rawL[i];
			idxMinL = i;
		}
	}
	
	cam_thrR = (maxR + minR) / 4;
	cam_thrL = (maxL + minL) / 4;
	
	cntBinL = 0;
	cntBinR = 0;
	
	// Binary Transform
	for(i = 7; i <= 120; i++)
	{
		if(cam_rawL[i] > cam_thrL)
		{
			cam_binL[i] = 255;	
		}
		else
		{
			cam_binL[i] = 0;
		}
		
		if(cam_rawR[i] > cam_thrR)
		{
			cam_binR[i] = 255;
		}
		else
		{
			cam_binR[i] = 0;
		}
	}
	
	// Unite 2 Camera Binary Data
	for(i = 0; i < 128; i++)
	{
		if(i >= idxRCS && i <= idxRCC)
		{
			cam_binU[i - idxRCS] = cam_binR[i];				// idxRCS ~ idxRCC
		}
		
		if(i >= idxLCC && i <= idxLCE)
		{
			cam_binU[(i - idxRCS) + diffRL] = cam_binL[i];	// idxLCC ~ idxLCE
		}
	}
}

void DetectLine(void)
{
	cntBinU = 0;
	cntL = 0;
	cntR = 0;
	cntU = 0;
	
	idxStartLineR = 255;
	idxEndLineR = 255;
	idxStartLineL = 255;
	idxEndLineL = 255;
	
	for(i = 0; i < sizeUnite; i++)
	{
		if(cam_binU[i] == 0)
		{
			cntBinU++;
		}
	}
	
	// Camera R
	for(i = idxUC; i >= 0; i--)
	{
//		if(idxStartLineR == 255 && cam_binU[i] == 0)
//		{
//			idxStartLineR = (uint8_t)i;
//		}
//		else if(idxStartLineR != 255 && i < idxStartLineR && cam_binU[i] == 255)
//		{
//			idxEndLineR = (uint8_t)i;
//			
//			idxLineR[cntR] = idxStartLineR;
//			idxStartLineR = 255;
//			cntR++;
//		}
		
		if(idxStartLineR == 255 && cam_binU[i] == 0)
		{
			idxStartLineR = (uint8_t)i;
			idxLineR[cntR] = idxStartLineR;
			cntR++;
			
			break;
		}
	}
	
	// Camera L
	for(i = idxUC; i < sizeUnite; i++)
	{
//		if(idxStartLineL == 255 && cam_binU[i] == 0)
//		{
//			idxStartLineL = (uint8_t)i;
//		}
//		else if(idxStartLineL != 255 && i > idxStartLineL && cam_binU[i] == 255)
//		{
//			idxEndLineL = (uint8_t)i;
//			
//			idxLineL[cntL] = idxStartLineL;
//			idxStartLineL = 255;
//			cntL++;
//		}
		
		if(idxStartLineL == 255 && cam_binU[i] == 0)
		{
			idxStartLineL = (uint8_t)i;
			idxLineL[cntL] = idxStartLineL;
			cntL++;
			
			break;
		}
	}
	cntU = cntL + cntR;
	
	if(cntR != 0 && cntL != 0 && (idxStartLineL - idxStartLineR) < 50)
	{
		if(flagLine_now == LINE_R)
		{
			cntL = 0;
			
			idxStartLineL = 255;
		}
		else if(flagLine_now == LINE_L)
		{
			cntR = 0;
			
			idxStartLineR = 255;
		}
	}
	if(cntR != 0 && cntL == 0 && flagLine_now == LINE_L && idxSLineL[0] - idxStartLineR < 50)
	{
		idxStartLineR = 255;
		idxStartLineL = idxUC;
		
		idxLineL[0] = idxStartLineL;
		
		cntR = 0;
		cntL = 1;
	}
	if(cntR == 0 && cntL != 0 && flagLine_now == LINE_R && idxStartLineL - idxSLineR[0] < 50)
	{
		idxStartLineR = idxUC;
		idxStartLineL = 255;	
		
		idxLineR[0] = idxStartLineR;
		
		cntR = 1;
		cntL = 0;
	}
	
	// Collect index Line Now to Before 10 Cycle
	for(i = 8; i >= 0; i--)
	{
		idxSLineR[i + 1] = idxSLineR[i];
		idxSLineL[i + 1] = idxSLineL[i];
	}
	
	if(cntR != 0)
	{
		idxSLineR[0] = idxLineR[0];
	}
	else
	{
		idxSLineR[0] = 255;
	}
	
	if(cntL != 0)
	{
		idxSLineL[0] = idxLineL[0];
	}
	else
	{
		idxSLineL[0] = 255;
	}
	
//	for(i = 110; i >= 7; i--)
//	{
//		if(cam_binR[i] == 0)
//		{
//			idxSLineR[0] = i;
//			break;
//		}
//	}
//	
//	for(i = 7; i <= 120; i++)
//	{
//		if(cam_binL[i] == 0)
//		{
//			idxSLineL[0] = i;
//			break;
//		}
//	}
}

void SetPosition(void)
{
	MyPos_old = MyPos_pre;
	MyPos_pre = MyPos_now;
	
	if(cntL != 0 && cntR != 0)		// Detect Both Side Line
	{
		if(idxSLineR[0] < 2)		// Previous Steering Servo Turn Left
		{
			MyPos_now = idxSLineR[0] - idxRCL;
		}
		else if(idxSLineL[0] > sizeUnite - 2)	// Previous Steering Servo Turn Right
		{
			MyPos_now = idxSLineL[0] - idxLCL;
		}
		else
		{
			MyPos_now = ((idxSLineL[0] + idxSLineR[0]) / 2) - idxUC;
		}
		flagLine_pre = flagLine_now;
		flagLine_now = LINE_B;
	}
	else if(cntL != 0 && cntR == 0)	// Detect Left Side Line
	{
		MyPos_now = idxSLineL[0] - idxLCL;
		flagLine_pre = flagLine_now;
		flagLine_now = LINE_L;	
	}
	else if(cntL == 0 && cntR != 0)	// Detect Right Side Line
	{
		MyPos_now = idxSLineR[0] - idxRCL;
		flagLine_pre = flagLine_now;
		flagLine_now = LINE_R;			
	}
	else								// Present Detect Line None
	{
		if(flagLine_pre == LINE_B)			// Previous Detect 2 Side
		{
			MyPos_now = 0;
		}
		else if(flagLine_pre == LINE_L)		// Previous Detect Line Left Side
		{
			MyPos_now = MyPos_pre;
		}
		else if(flagLine_pre == LINE_R)		// Previous Detect Line Right Side
		{
			MyPos_now = MyPos_pre;
		}
		else if(flagLine_pre == LINE_N)		// Previous Detect Line None
		{
			MyPos_now = 0;
		}
		
		flagLine_pre = flagLine_now;
		flagLine_now = LINE_N;
	}
}

void ControlServo(void)
{	
	errServo_now = 0 - MyPos_now;
	if(flagDCOnOff == ON)
	{
		errServo_sum += errServo_now;
	}
	
	if((int)errServo_sum > (int)Isat_Servo)
	{
		errServo_sum = Isat_Servo;
	}
	else if((int)errServo_sum < (int)(Isat_Servo * -1))
	{
		errServo_sum = Isat_Servo * -1;
	}
	
	dutyServo = dutyServo_Center 
			+ P_Servo * errServo_now 
			+ I_Servo * errServo_sum // * T_cycle
			+ D_Servo * (errServo_now - errServo_pre); // / T_cycle;
	
//	dutyServo = P_Servo * (errServo_now + I_Servo * errServo_sum * T_cycle
//			+ D_Servo * (errServo_now - errServo_pre) / T_cycle);
	
	errServo_pre = errServo_now;
	
//	if(cntDC1 >= 450 || cntDC2 >= 450)
//	{
//		dutyServo = dutyServo * 1.25;
//	}
	
//	cntDCAvg = (cntDC1 + cntDC2) / 2;
//	dutyServo = dutyServo * (1 + (cntDCAvg / 400.0));
	
//	dutyServo = (dutyServo - dutyServo_Center) * 1.5 + dutyServo_Center; 
	
	if(dutyServo > dutyServo_Max)
	{
		dutyServo = dutyServo_Max;
	}
	else if(dutyServo < dutyServo_Min)
	{
		dutyServo = dutyServo_Min;
	}
		
	EMIOS_0_OPWM_Duty(4, dutyServo);
}

//void ControlBrake(void)
//{
//	if(flagBrakeOnOff == ON && flagBrakeOnOffPrev == OFF)
//	{
//		if(cntDesire != 0)
//		{
//			cntDesire_def = cntDesire;
//			cntDesire = 0;
//		}
//		EMIOS_0_OPWM_Duty(5, BRAKE);
//		
//		flagBrakeOnOffPrev = ON;
//	}
//	else if(flagBrakeOnOff == OFF && flagBrakeOnOffPrev == ON)
//	{
//		cntDesire = cntDesire_def;
//		EMIOS_0_OPWM_Duty(5, RUN);
//		
//		flagBrakeOnOffPrev = OFF;
//	}
//}

void ControlDC(void)
{
	double dutyDiff;
	double dutyRatioInc;
	double dutyRatioDec;
	
//	if(MyPos_now < 0)
	if(dutyServo > dutyServo_Center)		// Right Handle
	{
//		dutyDiff = -MyPos_now;		
		dutyDiff = dutyServo - dutyServo_Center;
		
		// 1st Order Differentier
//		dutyRatioInc = dutyDiff / 450.0;
//		dutyRatioDec = dutyDiff / 900.0;
//		
//		cntTgt1 = cntDesire * (1.0 - dutyRatioDec);
//		cntTgt2 = cntDesire * (1.0 + dutyRatioInc);
		
		// 2nd Order Differentier
		dutyRatioInc = dutyDiff / 750.0;
		dutyRatioDec = dutyDiff / 1100.0;
		
		cntTgt1 = cntDesire * (1.0 - dutyRatioDec) * (1.0 - dutyRatioDec);
		cntTgt2 = cntDesire * (1.0 + dutyRatioInc) * (1.0 + dutyRatioInc);
		
		// 1st Order Decrease
//		dutyRatioInc = dutyDiff / 800.0;
//		dutyRatioDec = dutyDiff / 400.0;
//		
//		cntTgt1 = cntDesire * (1.0 - dutyRatioDec);
//		cntTgt2 = cntDesire * (1.0 - dutyRatioInc);
		
		// 2nd Order DC Motor Decrease
//		dutyRatioInc = dutyDiff / 1000.0;
//		dutyRatioDec = dutyDiff / 1000.0;
//		
//		cntTgt1 = cntDesire * (1.0 - dutyRatioDec) * (1.0 - dutyRatioDec);
//		cntTgt2 = cntDesire * (1.0 - dutyRatioInc) * (1.0 - dutyRatioInc);
		
//		cntTgt1 = cntDesire;
//		cntTgt2 = cntDesire;
	}
//	else if(MyPos_now > 0)
	else if(dutyServo < dutyServo_Center)	// Left Handle
	{
//		dutyDiff = -MyPos_now;
		dutyDiff = dutyServo_Center - dutyServo;
		
		// 1st Order Differentier
//		dutyRatioInc = dutyDiff / 450.0;
//		dutyRatioDec = dutyDiff / 900.0;
//		
//		cntTgt1 = cntDesire * (1.0 + dutyRatioInc);
//		cntTgt2 = cntDesire * (1.0 - dutyRatioDec);
		
		// 2nd Order Differentier
		dutyRatioInc = dutyDiff / 750.0;
		dutyRatioDec = dutyDiff / 1100.0;
		
		cntTgt1 = cntDesire * (1.0 + dutyRatioInc) * (1.0 + dutyRatioInc);
		cntTgt2 = cntDesire * (1.0 - dutyRatioDec) * (1.0 - dutyRatioDec);
		
		// 1st Order Decrease
//		dutyRatioInc = dutyDiff / 800.0;
//		dutyRatioDec = dutyDiff / 400.0;
//		
//		cntTgt1 = cntDesire * (1.0 - dutyRatioInc);
//		cntTgt2 = cntDesire * (1.0 - dutyRatioDec);
		
		// 2nd Order DC Motor Decrease
//		dutyRatioInc = dutyDiff / 1000.0;
//		dutyRatioDec = dutyDiff / 1000.0;
//		
//		cntTgt1 = cntDesire * (1.0 - dutyRatioInc) * (1.0 - dutyRatioInc);
//		cntTgt2 = cntDesire * (1.0 - dutyRatioDec) * (1.0 - dutyRatioDec);	
		
//		cntTgt1 = cntDesire;
//		cntTgt2 = cntDesire;
	}
	else
	{
		cntTgt1 = cntDesire;
		cntTgt2 = cntDesire;
	}
	
//	cntTgt1 = cntDesire;
//	cntTgt2 = cntDesire;

/////////////////////////////////////////////////
	errDC1_now = cntTgt1 - cntDC1;
	errDC1_sum += errDC1_now * T_cycle;
	if(errDC1_sum > Isat_DC1 || errDC1_sum < Isat_DC1 * -1.0)
	{
		errDC1_sum = 0;
	}
	
	dutyDC1 += P_DC1 * errDC1_now 
			+ I_DC1 * errDC1_sum
			+ D_DC1 * (errDC1_now - errDC1_pre);
	if(dutyDC1 > dutyMax)
	{
		dutyDC1 = dutyMax;
	}
	else if(dutyDC1 < -dutyMax)
	{
		dutyDC1 = -dutyMax;
	}
	
	errDC1_pre = errDC1_now;
/////////////////////////////////////////////////	
	errDC2_now = cntTgt2 - cntDC2;
	errDC2_sum += errDC2_now * T_cycle;
	if(errDC2_sum > Isat_DC2 || errDC2_sum < Isat_DC2 * -1.0)
	{
		errDC2_sum = 0;
	}
	
	dutyDC2 += P_DC2 * errDC2_now 
			+ I_DC2 * errDC2_sum
			+ D_DC2 * (errDC2_now - errDC2_pre);
	if(dutyDC2 > dutyMax)
	{
		dutyDC2 = dutyMax;
	}
	else if(dutyDC2 < -dutyMax)
	{
		dutyDC2 = -dutyMax;
	}
	
	errDC2_pre = errDC2_now;
/////////////////////////////////////////////////
	if(dutyDC1 >= 0)	// Forward Direction
	{
		EMIOS_0_OPWM_Duty(24, (uint16_t)dutyDC1);
		EMIOS_0_OPWM_Duty(25, 0);
	}
	else				// Reverse Direction
	{
		EMIOS_0_OPWM_Duty(24, 0);
		EMIOS_0_OPWM_Duty(25, (uint16_t)(-dutyDC1));	
	}
	
	if(dutyDC2 >= 0)	// Forward Direction
	{
		EMIOS_0_OPWM_Duty(26, (uint16_t)dutyDC2);	
		EMIOS_0_OPWM_Duty(27, 0);
	}
	else				// Reverse Direction
	{
		EMIOS_0_OPWM_Duty(26, 0);	
		EMIOS_0_OPWM_Duty(27, (uint16_t)(-dutyDC2));		
	}
}

void BTSendData(void)
{
	if(flagBTSend == '1')
	{
		putChar1('@');
		putChar1((uint8_t)(MyPos_now + 128));
		putChar1('#');
	}
	else if(flagBTSend == '2')
	{
		putChar1('@');
		putChar1((uint8_t)cntU);
		putChar1('#');
	}
	else if(flagBTSend == '3')
	{
		putChar1('@');
		putChar1((uint8_t)cntR);
		putChar1('#');
	}
	else if(flagBTSend == '4')
	{
		putChar1('@');
		putChar1((uint8_t)cntL);
		putChar1('#');
	}
	else if(flagBTSend == '5')
	{
		putChar1('@');
		putChar1((uint8_t)idxSLineL[0]);
		putChar1('#');
	}
	else if(flagBTSend == '6')
	{
		putChar1('@');
		putChar1((uint8_t)idxSLineR[0]);
		putChar1('#');
	}
	else if(flagBTSend == '7')
	{
		putChar1('@');
		putChar1(cntDC1);
		putChar1(cntDC1 >> 8);
		putChar1(cntDC2);
		putChar1(cntDC2 >> 8);
		putChar1('#');
	}
	else if(flagBTSend == '8')
	{
		putChar1('@');

		putChar1('#');
	}
	else if(flagBTSend == '9')
	{
		putChar1('@');
		putChar1(MyPos_now + 128);
		putChar1((uint8_t)idxSLineR[0]);
		putChar1((uint8_t)idxSLineL[0]);
		putChar1(dutyServo);
		putChar1(dutyServo >> 8);
		putChar1(cntDC1);
		putChar1(cntDC1 >> 8);
		putChar1(cntDC2);
		putChar1(cntDC2 >> 8);
		putChar1('#');
	}
	// flagBTSend == 0  -> Stop Send Data
}

int main(void) 
{
	initModesAndClock();
	initPeriClkGen();
	disableWatchdog();
	
	initPads();
	SIU.GPDO[80].B.PDO = 0;							// DC Motor Disable
	
	InitVar();
	
	initEMIOS_0();
	initEMIOS_0_MCB(23, 1000 - 1);					// Counter Bus A, 1000usec
	initEMIOS_0_OPWM(24, 0x00, 0);					// PG10, DCMotor1, 0~1000
	initEMIOS_0_OPWM(25, 0x00, 0);					// PG11, DCMotor1, 0~1000
	initEMIOS_0_OPWM(26, 0x00, 0);					// PG12, DCMotor2, 0~1000
	initEMIOS_0_OPWM(27, 0x00, 0);					// PG13, DCMotor2, 0~1000
	
	initEMIOS_0_MCB(0, 20000 - 1);					// Counter Bus B, 20000usec
	initEMIOS_0_OPWM(4, 0x01, dutyServo_Center);	// PA4, Steering Servo, 1465 ~ 2065
	initEMIOS_0_OPWM(5, 0x01, RUN);					// PA5, Brake Servo, 700, 2300
	
	initADC0();
	
	initLINFlex_0();
	initLINFlex_1();
	
	initDSPI();
	initLS7366();
	
	INTC_InitINTCInterrupts();
	INTC_InstallINTCInterruptHandler(ISR_PIT0, 59, 1);
//	INTC_InstallINTCInterruptHandler(ISR_LIN1_UART_RECV, 99, 2);
	enableIrq();
	
	PIT.PITMCR.R = 0x00000001;       			/* Enable PIT and configure timers to stop in debug mode */
	PIT.CH[0].LDVAL.R = (uint32_t) (640 - 1);			// 0 ~ 518 Cycle Period Set 10us 
														// 10usec => 640, 20usec => 1280
	
	initCLCD();
	writeString(0, 0, "BT SCAN");
	
	putStr1("AT+BTSCAN");
	putChar1(BCR); 
	Delay(10000000);

	clearCLCD();
	Delay(10000000);
	
	while(1)
	{
		MainLoop();
	}
}

void InitVar(void)
{
	flagDisp = 0;
	
	data_btrx = 0;
	flagBTBrake = OFF;
	flagBTSend = 0;
	/////////////////////////////////////////
	for(i = 0; i < 256; i++)
	{
		cam_binU[i] = 125;
	}
	
	idxRCS = 7;
	idxLCE = 120;
	
	idxRCC = 87;			// Right Cam Side Center
	idxLCC = 41;			// Left Cam Side Cetner
	
	idxUC = idxRCC - idxRCS;	// Index Unite Center
	
	idxRCL = 4;				// Right Cam Side Line
	idxLCL = 158;			// Left Cam Side Line
	
	idxRCL_def = 4;
	idxLCL_def = 158;
	
//	idxRCC = 80;			// Right Cam Side Center
//	idxLCC = 46;			// Left Cam Side Cetner
//	
//	idxUC = idxRCC - idxRCS;	// Index Unite Center
//	
//	idxRCL = 17;			// Right Cam Side Line
//	idxLCL = 134;			// Left Cam Side Line
	
	diffRL = idxRCC - idxLCC + 1;
	sizeUnite = (idxRCC - idxRCS + 1) + (idxLCE - idxLCC + 1);
	
	flagLine_now = LINE_B;
	flagLine_pre = LINE_B;
	
	MyPos_now = 0;
	MyPos_pre = 0;
	MyPos_old = 0;
	/////////////////////////////////////////
	T_cycle = 0.00519 + 0.007;		// 10us * 519 = 0.00519, Calculate Period + 0.00519
	/////////////////////////////////////////
	errServo_now = 0;
	errServo_pre = 0;
	errServo_sum = 0;
	
	// Best Setting
//	P_Servo = 7.0;
//	I_Servo = 0.0;
//	Isat_Servo = 50.0;			// Original 100
//	D_Servo = 0.3;
	
	P_Servo = 7.5;
	I_Servo = 0.0;
	Isat_Servo = 200.0;			// Original 100
	D_Servo = 16.5;
	
	dutyServo_Center = 1800;
	dutyServo_Min = dutyServo_Center - 320;
	dutyServo_Max = dutyServo_Center + 320;
	/////////////////////////////////////////
	cntDesire = 200;
	
//	P_DC1 = 1.7;
//	I_DC1 = 0.0;
//	Isat_DC1 = 1000.0;
//	D_DC1 = 15.0;
//	
//	P_DC2 = 1.7;
//	I_DC2 = 0.0;
//	Isat_DC2 = 1000.0;
//	D_DC2 = 15.0;
	
	P_DC1 = 1.8;
	I_DC1 = 0.0;
	Isat_DC1 = 1000.0;
	D_DC1 = 4.0;
	
	P_DC2 = 1.8;
	I_DC2 = 0.0;
	Isat_DC2 = 1000.0;
	D_DC2 = 4.0;
	/////////////////////////////////////////
	flagDCOnOff = OFF;
	dutyMax = DC_MAX;
	dutyDC1 = 0;
	dutyDC2 = 0;
}

void MainLoop(void)
{
	BehaviorBTData();
	
	if(flagDisp == 0)
	{
		DispData1();
		Button1();
	}
	else if(flagDisp == 1)
	{
		DispData2();
		Button2();
	}
	else if(flagDisp == 2)
	{
		DispData3();
		Button3();
	}
//	else if(flagDisp == 3)
//	{
//		DispData4();
//		Button4();
//	}
	
	if(SIU.GPDI[102].B.PDI == 0x00)		// Toggle 1
	{
		Button1();
	}
	else
	{
		Button1();
	}
	
//	if(SIU.GPDI[103].B.PDI == 0x00)		// Toggle 2
//	{
//
//	}
//	else
//	{
//
//	}
//	
//	if(SIU.GPDI[104].B.PDI == 0x00)		// Toggle 3
//	{
//
//	}
//	else
//	{
//		
//	}
}

void Button1(void)
{
	if(SIU.GPDI[85].B.PDI == 0x00)			// No 4
	{
		while(SIU.GPDI[85].B.PDI == 0x00) {};

//		P_DC1 += 0.1;
//		P_DC2 += 0.1;
		
		Delay(100000);
		while(SIU.GPDI[64].B.PDI == 0x00) {};
		
		SensingOnOff();
	}
	if(SIU.GPDI[88].B.PDI == 0x00)			// No 8
	{
		while(SIU.GPDI[88].B.PDI == 0x00) {};
		
//		P_DC1 -= 0.1;
//		P_DC2 -= 0.1;
		
		Delay(100000);
		while(SIU.GPDI[65].B.PDI == 0x00) {};
	
		if(flagDCOnOff == ON)
		{
			flagDCOnOff = OFF;
			
			dutyDC1 = 0;
			dutyDC2 = 0;
			cntDesire = 0;
			SIU.GPDO[80].B.PDO = 0;
		}
		else if(flagDCOnOff == OFF)
		{
			SIU.GPDO[80].B.PDO = 1;
			flagDCOnOff = ON;
		}
	}
	
	if(SIU.GPDI[84].B.PDI == 0x00)			// No 3
	{
		while(SIU.GPDI[84].B.PDI == 0x00) {};

//		D_DC1 += 0.1;
//		D_DC2 += 0.1;
		
		flagDisp++;
		if(flagDisp > 2)
		{
			flagDisp = 0;
		}
	}
	if(SIU.GPDI[89].B.PDI == 0x00)			// No 7
	{
		while(SIU.GPDI[89].B.PDI == 0x00) {};
		
//		D_DC1 -= 0.1;
//		D_DC2 -= 0.1;
		
		clearCLCD();
	}
	
	if(SIU.GPDI[83].B.PDI == 0x00)			// No 2
	{
		while(SIU.GPDI[83].B.PDI == 0x00) {};
		
		cntDesire += 10;
	}
	if(SIU.GPDI[87].B.PDI == 0x00)			// No 6
	{
		while(SIU.GPDI[87].B.PDI == 0x00) {};

		cntDesire -= 10;
	}
	
	if(SIU.GPDI[82].B.PDI == 0x00)			// No 1
	{
		while(SIU.GPDI[82].B.PDI == 0x00) {};
		
		cntDesire += 100;
	}
	if(SIU.GPDI[86].B.PDI == 0x00)			// No 5
	{
		while(SIU.GPDI[86].B.PDI == 0x00) {};
		
		cntDesire -= 100;
	}
	////////////////////////////////////////////////////////	
	if(SIU.GPDI[64].B.PDI == 0x00)
	{

	}
	
	if(SIU.GPDI[65].B.PDI == 0x00)
	{

	}
	
	if(SIU.GPDI[66].B.PDI == 0x00)
	{
		while(SIU.GPDI[66].B.PDI == 0x00) {};

		flagDisp++;
		if(flagDisp > 2)
		{
			flagDisp = 0;
		}
	}
	
	if(SIU.GPDI[67].B.PDI == 0x00)
	{
		while(SIU.GPDI[67].B.PDI == 0x00) {};
		
		clearCLCD();
	}
}

void Button2(void)
{
	if(SIU.GPDI[85].B.PDI == 0x00)			// No 4
	{
		while(SIU.GPDI[85].B.PDI == 0x00) {};

//		P_DC1 += 0.1;
//		P_DC2 += 0.1;
		
		Delay(100000);
		while(SIU.GPDI[64].B.PDI == 0x00) {};
		
		SensingOnOff();
	}
	if(SIU.GPDI[88].B.PDI == 0x00)			// No 8
	{
		while(SIU.GPDI[88].B.PDI == 0x00) {};
		
//		P_DC1 -= 0.1;
//		P_DC2 -= 0.1;
		
		Delay(100000);
		while(SIU.GPDI[65].B.PDI == 0x00) {};
	
		if(flagDCOnOff == ON)
		{
			flagDCOnOff = OFF;
			
			dutyDC1 = 0;
			dutyDC2 = 0;
			cntDesire = 0;
			SIU.GPDO[80].B.PDO = 0;
		}
		else if(flagDCOnOff == OFF)
		{
			SIU.GPDO[80].B.PDO = 1;
			flagDCOnOff = ON;
		}
	}
	
	if(SIU.GPDI[84].B.PDI == 0x00)			// No 3
	{
		while(SIU.GPDI[84].B.PDI == 0x00) {};

//		D_DC1 += 0.1;
//		D_DC2 += 0.1;
		
		flagDisp++;
		if(flagDisp > 2)
		{
			flagDisp = 0;
		}
	}
	if(SIU.GPDI[89].B.PDI == 0x00)			// No 7
	{
		while(SIU.GPDI[89].B.PDI == 0x00) {};
		
//		D_DC1 -= 0.1;
//		D_DC2 -= 0.1;
		
		clearCLCD();
	}
	
	if(SIU.GPDI[83].B.PDI == 0x00)			// No 2
	{
		while(SIU.GPDI[83].B.PDI == 0x00) {};
		
		cntDesire += 10;
	}
	if(SIU.GPDI[87].B.PDI == 0x00)			// No 6
	{
		while(SIU.GPDI[87].B.PDI == 0x00) {};

		cntDesire -= 10;
	}
	
	if(SIU.GPDI[82].B.PDI == 0x00)			// No 1
	{
		while(SIU.GPDI[82].B.PDI == 0x00) {};
		
		cntDesire += 100;
	}
	if(SIU.GPDI[86].B.PDI == 0x00)			// No 5
	{
		while(SIU.GPDI[86].B.PDI == 0x00) {};
		
		cntDesire -= 100;
	}
	////////////////////////////////////////////////////////	
	if(SIU.GPDI[64].B.PDI == 0x00)
	{

	}
	
	if(SIU.GPDI[65].B.PDI == 0x00)
	{

	}
	
	if(SIU.GPDI[66].B.PDI == 0x00)
	{
		while(SIU.GPDI[66].B.PDI == 0x00) {};

		flagDisp++;
		if(flagDisp > 2)
		{
			flagDisp = 0;
		}
	}
	
	if(SIU.GPDI[67].B.PDI == 0x00)
	{
		while(SIU.GPDI[67].B.PDI == 0x00) {};
		
		clearCLCD();
	}
}
void Button3(void)
{
	if(SIU.GPDI[85].B.PDI == 0x00)			// No 4
	{
		while(SIU.GPDI[85].B.PDI == 0x00) {};

//		P_DC1 += 0.1;
//		P_DC2 += 0.1;
		
		Delay(100000);
		while(SIU.GPDI[64].B.PDI == 0x00) {};
		
		SensingOnOff();
	}
	if(SIU.GPDI[88].B.PDI == 0x00)			// No 8
	{
		while(SIU.GPDI[88].B.PDI == 0x00) {};
		
//		P_DC1 -= 0.1;
//		P_DC2 -= 0.1;
		
		Delay(100000);
		while(SIU.GPDI[65].B.PDI == 0x00) {};
	
		if(flagDCOnOff == ON)
		{
			flagDCOnOff = OFF;
			
			dutyDC1 = 0;
			dutyDC2 = 0;
			cntDesire = 0;
			SIU.GPDO[80].B.PDO = 0;
		}
		else if(flagDCOnOff == OFF)
		{
			SIU.GPDO[80].B.PDO = 1;
			flagDCOnOff = ON;
		}
	}
	
	if(SIU.GPDI[84].B.PDI == 0x00)			// No 3
	{
		while(SIU.GPDI[84].B.PDI == 0x00) {};

//		D_DC1 += 0.1;
//		D_DC2 += 0.1;
		
		flagDisp++;
		if(flagDisp > 2)
		{
			flagDisp = 0;
		}
	}
	if(SIU.GPDI[89].B.PDI == 0x00)			// No 7
	{
		while(SIU.GPDI[89].B.PDI == 0x00) {};
		
//		D_DC1 -= 0.1;
//		D_DC2 -= 0.1;
		
		clearCLCD();
	}
	
	if(SIU.GPDI[83].B.PDI == 0x00)			// No 2
	{
		while(SIU.GPDI[83].B.PDI == 0x00) {};
		
//		cntDesire += 10;
		
		idxRCL = idxSLineR[0];
		idxLCL = idxSLineL[0];
	}
	if(SIU.GPDI[87].B.PDI == 0x00)			// No 6
	{
		while(SIU.GPDI[87].B.PDI == 0x00) {};

//		cntDesire -= 10;
		
		idxRCL = idxRCL_def;
		idxLCL = idxLCL_def;
	}
	
	if(SIU.GPDI[82].B.PDI == 0x00)			// No 1
	{
		while(SIU.GPDI[82].B.PDI == 0x00) {};
		
//		cntDesire += 100;
	}
	if(SIU.GPDI[86].B.PDI == 0x00)			// No 5
	{
		while(SIU.GPDI[86].B.PDI == 0x00) {};
		
//		cntDesire -= 100;
	}
	////////////////////////////////////////////////////////	
	if(SIU.GPDI[64].B.PDI == 0x00)
	{

	}
	
	if(SIU.GPDI[65].B.PDI == 0x00)
	{

	}
	
	if(SIU.GPDI[66].B.PDI == 0x00)
	{
		while(SIU.GPDI[66].B.PDI == 0x00) {};

		flagDisp++;
		if(flagDisp > 2)
		{
			flagDisp = 0;
		}
	}
	
	if(SIU.GPDI[67].B.PDI == 0x00)
	{
		while(SIU.GPDI[67].B.PDI == 0x00) {};
		
		clearCLCD();
	}
}
void Button4(void)
{
	if(SIU.GPDI[85].B.PDI == 0x00)			// No 4
	{
		while(SIU.GPDI[85].B.PDI == 0x00) {};

//		P_DC1 += 0.1;
//		P_DC2 += 0.1;
		
		Delay(100000);
		while(SIU.GPDI[64].B.PDI == 0x00) {};
		
		SensingOnOff();
	}
	if(SIU.GPDI[88].B.PDI == 0x00)			// No 8
	{
		while(SIU.GPDI[88].B.PDI == 0x00) {};
		
//		P_DC1 -= 0.1;
//		P_DC2 -= 0.1;
		
		Delay(100000);
		while(SIU.GPDI[65].B.PDI == 0x00) {};
	
		if(flagDCOnOff == ON)
		{
			flagDCOnOff = OFF;
			
			dutyDC1 = 0;
			dutyDC2 = 0;
			cntDesire = 0;
			SIU.GPDO[80].B.PDO = 0;
		}
		else if(flagDCOnOff == OFF)
		{
			SIU.GPDO[80].B.PDO = 1;
			flagDCOnOff = ON;
		}
	}
	
	if(SIU.GPDI[84].B.PDI == 0x00)			// No 3
	{
		while(SIU.GPDI[84].B.PDI == 0x00) {};

//		D_DC1 += 0.1;
//		D_DC2 += 0.1;
		
		flagDisp++;
		if(flagDisp > 2)
		{
			flagDisp = 0;
		}
	}
	if(SIU.GPDI[89].B.PDI == 0x00)			// No 7
	{
		while(SIU.GPDI[89].B.PDI == 0x00) {};
		
//		D_DC1 -= 0.1;
//		D_DC2 -= 0.1;
		
		clearCLCD();
	}
	
	if(SIU.GPDI[83].B.PDI == 0x00)			// No 2
	{
		while(SIU.GPDI[83].B.PDI == 0x00) {};
		
		cntDesire += 10;
	}
	if(SIU.GPDI[87].B.PDI == 0x00)			// No 6
	{
		while(SIU.GPDI[87].B.PDI == 0x00) {};

		cntDesire -= 10;
	}
	
	if(SIU.GPDI[82].B.PDI == 0x00)			// No 1
	{
		while(SIU.GPDI[82].B.PDI == 0x00) {};
		
		cntDesire += 100;
	}
	if(SIU.GPDI[86].B.PDI == 0x00)			// No 5
	{
		while(SIU.GPDI[86].B.PDI == 0x00) {};
		
		cntDesire -= 100;
	}
	
	if(SIU.GPDI[64].B.PDI == 0x00)
	{

	}
	
	if(SIU.GPDI[65].B.PDI == 0x00)
	{

	}
	
	if(SIU.GPDI[66].B.PDI == 0x00)
	{
		while(SIU.GPDI[66].B.PDI == 0x00) {};

		flagDisp++;
		if(flagDisp > 2)
		{
			flagDisp = 0;
		}
	}
	
	if(SIU.GPDI[67].B.PDI == 0x00)
	{
		while(SIU.GPDI[67].B.PDI == 0x00) {};
		
		clearCLCD();
	}
}

void DispData1(void)
{
	writeString(0, 0, "1:");	
	// 1
	
	writeNum100(2, 0, idxSLineL[0]);
	writeNum100(6, 0, idxSLineR[0]);
	
	writeNum10(10, 0, cntL);
	writeNum10(13, 0, cntR);
	writeNum10(16, 0, cntU);
//========================================================//
	// 2
	
	writeNum100(2, 1, maxL);
	writeNum100(6, 1, maxR);
	
	writeNum10(10, 1, cntBinL);
	writeNum10(13, 1, cntBinR);
	writeNum10(16, 1, cntBinU);
//========================================================//	
	// 3
	
	writeNum100(2, 2, minL);
	writeNum100(6, 2, minR);
	
	writeNum100(10, 2, dataBatt * 0.506);
	
	writeNum100(14, 2, sizeUnite);
	writeNum10(18, 2, idxUC);
//========================================================//	
	// 4
	
	writeNum100(2, 3, cam_thrL);
	writeNum100(6, 3, cam_thrR);
	
	writeSNum100(10, 3, MyPos_now);
	writeNum10(15, 3, CPeriod / 64000.0);
	if(flagDCOnOff == ON)
	{
		writeString(17, 3, "ON ");
	}
	else
	{
		writeString(17, 3, "OFF");
	}
}

void DispData2(void)
{
	writeString(0, 0, "2:");	
	// 1
	writeNum10(2, 0, (uint16_t)(P_DC1 * 10));
	writeNum10(5, 0, (uint16_t)(P_DC2 * 10));
	
	writeNum100(8, 0, (uint16_t)(D_DC1 * 10));
	writeNum100(12, 0, (uint16_t)(D_DC2 * 10));
//========================================================//
	// 2
	writeSNum100(2, 1, cntDC1);
	writeSNum100(6, 1, cntDC2);
	
	writeNum100(11, 1, (uint16_t)cntDesire);
//========================================================//	
	// 3
	writeNum1000(2, 2, dutyServo);
	writeSNum100(8, 2, errServo_sum);
	
	writeNum100(15, 2, dataBatt * 5.06);
	writeWord(19, 2, data_btrx);
//========================================================//	
	// 4
	writeSNum100(10, 3, MyPos_now);
	writeNum10(15, 3, CPeriod / 64000.0);
	if(flagDCOnOff == ON)
	{
		writeString(17, 3, "ON ");
	}
	else
	{
		writeString(17, 3, "OFF");
	}
}

void DispData3(void)
{
	writeString(0, 0, "3:");	
	// 1
	writeNum100(2, 0, maxL);
	writeNum100(6, 0, maxR);

	writeNum100(11, 0, idxSLineL[0]);
	writeNum100(15, 0, idxSLineR[0]);
//========================================================//
	// 2
	writeNum100(2, 1, idxMaxL);
	writeNum100(6, 1, idxMaxR);
	
	writeNum100(11, 1, idxLCL);
	writeNum100(15, 1, idxRCL);
//========================================================//	
	// 3
	writeNum100(2, 2, minL);
	writeNum100(6, 2, minR);

	writeNum100(11, 2, idxLCL_def);
	writeNum100(15, 2, idxRCL_def);
//========================================================//	
	// 4
	writeNum100(2, 3, idxMinL);
	writeNum100(6, 3, idxMinR);
	
	writeSNum100(10, 3, MyPos_now);
	writeNum10(15, 3, CPeriod / 64000.0);
	if(flagDCOnOff == ON)
	{
		writeString(17, 3, "ON ");
	}
	else
	{
		writeString(17, 3, "OFF");
	}	
}

void DispData4(void)
{
	writeString(0, 0, "4:");	// 1
	

//========================================================//
// 2


//========================================================//	
// 3
	

//========================================================//	
// 4
	
	writeSNum100(10, 3, MyPos_now);
	writeNum10(15, 3, CPeriod / 64000.0);
	if(flagDCOnOff == ON)
	{
		writeString(17, 3, "ON ");
	}
	else
	{
		writeString(17, 3, "OFF");
	}	
}

void SensingOnOff(void)
{
	if(flagCamOnOff == OFF)				// Cam On
	{
		flagCamOnOff = ON;

		dutyServo = dutyServo_Center;
		EMIOS_0_OPWM_Duty(4, dutyServo);
		
		flagTimer = 0;
		num_line = 0;
		
		errServo_sum = 0;
		errDC1_sum = 0;
		errDC2_sum = 0;
		
		PIT.CH[0].LDVAL.R = (uint32_t) (640 - 1);		// 0 ~ 518 Cycle Period 10us
		PIT.CH[0].TCTRL.R = 0x000000003; 	// Enable PIT interrupt & make PIT active to count
	}
	else if(flagCamOnOff == ON)		// Cam Off
	{
		flagCamOnOff = OFF;
		
		PIT.CH[0].TCTRL.R = 0x000000000; 	// PIT Disable
		PIT.CH[0].LDVAL.R = (uint32_t) (640 - 1);		// 0 ~ 518 Cycle Period 10us
		
		SIU.GPDO[0].B.PDO = 0;				// CLK1 Low
		SIU.GPDO[1].B.PDO = 0;				// SI1 Low
		SIU.GPDO[2].B.PDO = 0;				// CLK2 Low
		SIU.GPDO[3].B.PDO = 0;				// SI2 Low
		
		SIU.PGPDO[2].R = 0x05000000;
		
		dutyServo = dutyServo_Center;
		EMIOS_0_OPWM_Duty(4, dutyServo);
		EMIOS_0_OPWM_Duty(5, RUN);
	}
}

void BehaviorBTData(void)
{
	////////////////////// Free Stop, Brake ///////////////////////////////
	if(data_btrx == 'p')
	{
		flagBTBrake = OFF;
		
		dutyDC1 = 0;
		dutyDC2 = 0;
		
		EMIOS_0_OPWM_Duty(24, (uint16_t)dutyDC1);
		EMIOS_0_OPWM_Duty(25, (uint16_t)dutyDC1);
		EMIOS_0_OPWM_Duty(26, (uint16_t)dutyDC2);	
		EMIOS_0_OPWM_Duty(27, (uint16_t)dutyDC2);
		
		cntDesire = 0;
		EMIOS_0_OPWM_Duty(5, RUN);
		flagDCOnOff = OFF;
		
		putChar1('P');
		
		data_btrx = 0;
	}
	else if(data_btrx == 'b')
	{
		flagBTBrake = ON;
		
//		dutyDC1 = 0;
//		dutyDC2 = 0;
		
//		EMIOS_0_OPWM_Duty(24, (uint16_t)dutyDC1);
//		EMIOS_0_OPWM_Duty(25, (uint16_t)dutyDC1);
//		EMIOS_0_OPWM_Duty(26, (uint16_t)dutyDC2);	
//		EMIOS_0_OPWM_Duty(27, (uint16_t)dutyDC2);
		
		cntDesire = 0;
//		EMIOS_0_OPWM_Duty(5, BRAKE);
//		flagDCOnOff = OFF;
		
		putChar1('B');
		
		data_btrx = 0;
	}
	else if(data_btrx == 'g')
	{
		if(flagDCOnOff == ON)
		{
			flagDCOnOff = OFF;
			
			dutyDC1 = 0;
			dutyDC2 = 0;
			cntDesire = 0;
			SIU.GPDO[80].B.PDO = 0;
		}
		else if(flagDCOnOff == OFF)
		{
			SIU.GPDO[80].B.PDO = 1;
			flagDCOnOff = ON;
		}
		
		data_btrx = 0;
	}
	///////////////////// Velocity ///////////////////////////
	else if(data_btrx == '1')
	{
		flagBTSend = data_btrx;
		
		data_btrx = 0;
	}
	else if(data_btrx == '2')
	{
		flagBTSend = data_btrx;
		
		data_btrx = 0;
	}
	else if(data_btrx == '3')
	{
		flagBTSend = data_btrx;
		
		data_btrx = 0;
	}
	else if(data_btrx == '4')
	{
		flagBTSend = data_btrx;
		
		data_btrx = 0;
	}
	else if(data_btrx == '5')
	{
		flagBTSend = data_btrx;
		
		data_btrx = 0;
	}
	else if(data_btrx == '6')
	{
		flagBTSend = data_btrx;
		
		data_btrx = 0;
	}
	else if(data_btrx == '7')
	{
		flagBTSend = data_btrx;
		
		data_btrx = 0;
	}
	else if(data_btrx == '8')
	{
		flagBTSend = data_btrx;
		
		data_btrx = 0;
	}
	else if(data_btrx == '9')
	{
		flagBTSend = data_btrx;
		
		data_btrx = 0;
	}
	else if(data_btrx == '0')
	{
		flagBTSend = data_btrx;
		
		data_btrx = 0;
	}
	/////////////////////////////////////////////////////
	else if(data_btrx == 'q')
	{
		P_DC1 += 0.1;
		P_DC2 += 0.1;
		
		putChar1((uint8_t)(P_DC1 * 10) + '0');
		
		data_btrx = 0;
	}
	else if(data_btrx == 'w')
	{
		P_DC1 -= 0.1;
		P_DC2 -= 0.1;
				
		putChar1((uint8_t)(P_DC1 * 10) + '0');
		
		data_btrx = 0;
	}
	else if(data_btrx == 'e')
	{
		D_DC1 += 0.1;
		D_DC2 += 0.1;
		
		data_btrx = 0;
	}
	else if(data_btrx == 'r')
	{
		D_DC1 -= 0.1;
		D_DC2 -= 0.1;
		
		data_btrx = 0;
	}
	else if(data_btrx == 't')
	{

		
		data_btrx = 0;
	}
	else if(data_btrx == 'y')
	{

		
		data_btrx = 0;
	}
	/////////////////////////////////////////////////////
	else if(data_btrx == 'a')
	{

		data_btrx = 0;
	}
	else if(data_btrx == 's')
	{

		data_btrx = 0;
	}
	else if(data_btrx == 'd')
	{
		
		data_btrx = 0;
	}
	else if(data_btrx == 'f')
	{

		data_btrx = 0;
	}
	//////////////////////// Desire Pulse /////////////////////////
	else if(data_btrx == 'z')
	{
		cntDesire += 10;
		if(cntDesire >= 1000 && cntDesire < 60000)
		{
			cntDesire = 999;
		}
		
		putChar1((uint8_t)(cntDesire / 100) + '0');
		
		data_btrx = 0;	
	}
	else if(data_btrx == 'x')
	{
		cntDesire -= 10;
		if(cntDesire >= 60000)
		{
			cntDesire = 0;
		}

		putChar1(cntDesire / 100 + '0');
		
		data_btrx = 0;
	}
	else if(data_btrx == 'c')
	{
		cntDesire += 100;
		if(cntDesire >= 1000 && cntDesire < 60000)
		{
			cntDesire = 999;
		}
		
		putChar1((uint8_t)(cntDesire / 100) + '0');
		
		data_btrx = 0;		
	}
	else if(data_btrx == 'v')
	{
		cntDesire -= 100;
		if(cntDesire >= 60000)
		{
			cntDesire = 0;
		}

		putChar1(cntDesire / 100 + '0');
		
		data_btrx = 0;	
	}
}
