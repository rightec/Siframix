/**
@file		Main.h
@brief		CPUMODULI Main file
					
@author		
@date		18/01/2011
@version	01.00
*/

#ifndef _MAIN_H_
#define _MAIN_H_

#include "global.h"
#include "swtim.h"
#include "alloc.h"
#include "new.h"
#include "strclass.h"
#include "i2c_sw.h"
#include "I2Ceeprom.h"
#include "sioSG2.h"
#include "adc.h"
//#include "sound.h"
#include "Leds_panel.h"
#include "QueueUart.h"
#include "channels.h"
#include "protocolSIFRA.h"
#include "CPUSIFRA_Manager.h"
#include "pwm.h"

//--------------------------------------------------------//
// Definizione dei tipi
//--------------------------------------------------------//

//--------------------------------------------------------//
// Definizione delle classi
//--------------------------------------------------------//

//--------------------------------------------------------//
// Definizione delle funzioni
//--------------------------------------------------------//

#ifdef __cplusplus
extern "C" {
#endif

void TestMem(void);         // Test per la memoria esterna ram tamponata con orologio

#ifdef __cplusplus
}
#endif


//--------------------------------------------------------//
// Definizione delle variabili
//--------------------------------------------------------//
extern const char sAppRevision[];
extern const char sAppDate[];
extern const char sAppName[];
extern const char sHwVersion[];
extern const int sAppNameLength;
extern const int sAppRevisionLength;
extern const int sAppDateLength;
extern const int sHwVersionLength;

#endif

