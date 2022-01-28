/**
@file		Leds_panel.h
@brief		Functions to manage comunication with leds panel (serial communication).

@author		
@date		06/12/2012
@version	01.0
*/

#ifndef _LEDS_PANEL_H_
#define _LEDS_PANEL_H_

#include "global.h"
#include "swtim.h"
#include "i2c_sw.h"
#include "I2Ceeprom.h"
#include "sioSG2.h"

//--------------------------------------------------------//
// Definizione dei tipi
//--------------------------------------------------------//

enum LED_DRIVER_COMMAND
		{
			TURN_ALL_OFF	= 0xFFFF,
			TURN_OFF		= 0x0000,
			ALL_RED		= 0x1F87,
			ALL_GREEN		= 0x1F78,
			LED_VIA1 		= 0x0100,
			LED_VIA2 		= 0x0200,
			LED_VIA3		= 0x0300,
			LED_VIA4		= 0x0400,
			LED_VIA5		= 0x0500,
			LED_VIA6		= 0x0600,
			LED_VIA7		= 0x0700,
			LED_VIA8		= 0x0800,
			LED_ROSSO		= 0x0087,	//LED_ROSSO		= 0x0078,
			LED_VERDE		= 0x0078,	//LED_VERDE		= 0x0087,
			DECODE_MODE 	= 0x0900,
			SCANLIMIT		= 0x0B04,	// solo 5 led per M3100
			INTENSITY 		= 0x0A0F,
			SHOUTDOWN 	= 0x0C00,
			NORMAL_OP		= 0x0C01,
			TEST_LED		= 0x0F00,
		};

//--------------------------------------------------------//
// Definizione delle classi
//--------------------------------------------------------//

//--------------------------------------------------------//
// Definizione delle funzioni
//--------------------------------------------------------//

#ifdef __cplusplus
extern "C" {
#endif

void LedDrivers_init();		// init interface to panel led drivers
bool LedDrivers_sendData(short data);
void LedDrivers_clearFifo();
void switchon_red_leds();
void switchon_green_leds();
void switchon_leds();
void switchoff_leds();
void ManageLedsInCalibStatus(byte cmd);
void LedDrivers_DisplayTest();

#ifdef __cplusplus
}
#endif


//--------------------------------------------------------//
// Definizione delle variabili
//--------------------------------------------------------//


#endif
