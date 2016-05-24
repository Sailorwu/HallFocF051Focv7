#ifndef UserParms_H
#define UserParms_H

#include "system_define.h"

#define ID_KP 0x17FF			//Q15(0.05)  666
#define ID_KI	0x0183			//Q15(0.01)
#define ID_KD 0x0100			//
#define ID_KC 0x7FFF			//Q15(0.99999)
#define ID_MAX 0x7FFE			//Q15(0.99999)

#define IQ_KP 0x2FF0			//Q15(0.01)  147
#define IQ_KI	0x0200			//Q15(0.005)
#define IQ_KD 0x017F			//
#define IQ_KC 0x7FFF			//Q15(0.99999)
#define IQ_MAX 0x7FFE			//Q15(0.99999)

#define SP_KP 0x7F00			//Q15(0.05) 
#define SP_KI	0x04F0			//Q15(0.01)	
#define SP_KD 0x01A0			//
#define SP_KC 0x7FFF			//Q15(0.99999)
#define SP_MAX 0x7E20			//Q15(20000)
#define SP_MIN (100)		//200R/min

//************** Start-Up Parameters **************
#define POLEPAIRS      		2       // Number of pole pairs
#define NOMINALSPEEDINRPM 2500	// Make sure NOMINALSPEEDINRPM generates a MAXOMEGA < 1.0
#define MINSPEEDINRPM			300	// Minimum speed in RPM. Closed loop will operate at this

#define PWMFREQUENCY		15000		// PWM Frequency in Hertz
#define DEADTIMESEC			0.0000015	// Deadtime in seconds

//************** Derived Parameters ****************
#define DFCY        SystemCoreClock		// Instruction cycle frequency (Hz)
#define DTCY        (1.0/DFCY)		// Instruction cycle period (sec)
#define DDEADTIME   (unsigned short)(DEADTIMESEC*DFCY)	// Dead time in dTcys



#endif



