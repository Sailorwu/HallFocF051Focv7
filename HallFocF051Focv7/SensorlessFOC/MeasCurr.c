#include "Meascurr.h"

tMeasCurrParm MeasCurrParm;

void ReadADC0( tReadADCParm* pParm )
{
	int32_t Ls_Data;
	Ls_Data=ADC_Tab[SP_Channl]*NOMINALSPEED_ELECTR;
	pParm->qADValue=Ls_Data/4096;			//12bit to 15bit
}

void MeasCompCurr(tParkParm* Iab)			//2013.7.21 MARK
{
	Iab->qIa=2*(MeasCurrParm.iOffsetLa-(int16_t)ADC_Tab[IA_Channl]*8);
	Iab->qIb=2*(MeasCurrParm.iOffsetLb-(int16_t)ADC_Tab[IB_Channl]*8);
}

void InitMeasCompCurr( short Offset_a, short Offset_b )
{
	MeasCurrParm.iOffsetLa=Offset_a;
	MeasCurrParm.iOffsetLb=Offset_b;
}
