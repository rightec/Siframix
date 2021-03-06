/**
@file		Leds_panel.cpp
@brief		Functions to manage comunication with leds panel (serial communication).

@author		
@date		06/12/2012
@version	01.0
*/


#include "Leds_panel.h"

CSmallRingBuf <word, 8> DISPLAY_tx_fifo;

/**
Initialization of comunication to the external panel led driver (MAX7219/7221)
*/
void LedDrivers_init()		// init interface to panel led drivers
{
	sio_init_ch3();
	
	PinCS_LDD = 1;		// #CS (LOAD) led driver
	ioPnCS_LDD = 0;
	PinCS_LDD = 0;
	CB3TX = 0x0C01;		// inizializzazione
	while( CB3STR & 0x80 );
	PinCS_LDD = 1;
}

void LedDrivers_clearFifo()
{
	DISPLAY_tx_fifo.clear();
}

/**
Send new 16bit-data to led driver
*/
bool LedDrivers_sendData(short data)
{
	PinCS_LDD = 0;
	CB3TX = data;
	while( CB3STR & 0x80 );
	PinCS_LDD = 1;
	return 1;
}

/**
*/
void switchon_red_leds()
{
	byte i;
	short data = 0x0000;
	
	for(i = 1; i <= NUM_MAX_LINE; i++)
	{
		data = (i << 8) | LED_ROSSO;
		LedDrivers_sendData(data);
	}
}

/**
*/
void switchon_green_leds()
{
byte i;
short data = 0x0000;
	for(i = 1; i <= NUM_MAX_LINE; i++)
	{
		data = (i << 8) | LED_VERDE;
		LedDrivers_sendData(data);
	}
}

/**
*/
void switchon_leds()
{
	byte i;
	short data = 0x0000;
	for(i = 1; i <= NUM_MAX_LINE; i++)
	{
		data = (i << 8) | LED_VERDE | LED_ROSSO;
		LedDrivers_sendData(data);
	}
}

/**
*/
void switchoff_leds()
{
byte i;
short data = 0x0000;
	for(i = 1; i <= NUM_MAX_LINE; i++)
	{
		data = (i << 8) | TURN_OFF;
		LedDrivers_sendData(data);
	}
}

void LedDrivers_DisplayTest()
{
	LedDrivers_sendData(0x017F);
	LedDrivers_sendData(0x0100);
	LedDrivers_sendData(0x0107);
	LedDrivers_sendData(0x09FF);
	LedDrivers_sendData(0x0101);
	LedDrivers_sendData(0x010F);
	LedDrivers_sendData(0x0107);
}