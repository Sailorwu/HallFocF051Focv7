#ifndef MeasCurr_H
#define MeasCurr_H
#include "system_define.h"

typedef struct 
{
    short   qKa;        // 1.15 
    short   iOffsetLa;
    short   iOffsetHa;

    short   qKb;        // 1.15 
    short   iOffsetLb;
    short   iOffsetHb;

} tMeasCurrParm;

typedef struct 
{
    short   qK;         // 1.15 
    short   qADValue;   // 1.15
    short   qAnRef;		// 1.15
} tReadADCParm;

extern tReadADCParm ReadADCParm;
extern tMeasCurrParm MeasCurrParm;   
//------------------  C API for MeasCurr routines ---------------------

void MeasCurr( void );
void InitMeasCompCurr( short iOffset_a, short iOffset_b );

void ReadSignedADC0( tReadADCParm* pParm ); // Returns signed value -2*iK -> 2*iK
void ReadADC0( tReadADCParm* pParm );
#endif

