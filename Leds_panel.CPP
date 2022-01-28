/**
@file		Leds_panel.cpp
@brief	Functions to manage comunication with leds panel (serial communication).

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

/**
Manages the lines led during calibration test
*/
void ManageLedsInCalibStatus(byte cmd)
{
	word data_display;
	
	switch(cmd)
	{
		case 0x00:	// luci spente
			sio_disable_rx_interrupt_ch3();
			DISPLAY_tx_fifo.push_back(TURN_ALL_OFF);	// spegne tutto
			sio_enable_rx_interrupt_ch3(0x07);
			break;
		case 0x01://luce via 1: peso da 2Kg su via 1,
			sio_disable_rx_interrupt_ch3();
			data_display = 0x0100 | LED_VERDE;	// abilitazione via 1
			DISPLAY_tx_fifo.push_back(data_display);
			sio_enable_rx_interrupt_ch3(0x07);
			break;
		case 0x02:// luce via 2: peso da 2Kg su via 2
			sio_disable_rx_interrupt_ch3();
			data_display = 0x0200 | LED_VERDE;	// abilitazione via 1
			DISPLAY_tx_fifo.push_back(data_display);
			sio_enable_rx_interrupt_ch3(0x07);
			break;
		//case 0x03:	// luci via 3: peso da 2Kg su via 3
		case 0x04:	// luci via 3: peso da 2Kg su via 3
			sio_disable_rx_interrupt_ch3();
			data_display = 0x0300 | LED_VERDE;	// abilitazione via 1
			DISPLAY_tx_fifo.push_back(data_display);
			sio_enable_rx_interrupt_ch3(0x07);
			break;
		//case 0x04:					// luci via 4: peso da 2Kg su via 4
		case 0x08:					// luci via 4: peso da 2Kg su via 4
			sio_disable_rx_interrupt_ch3();
			data_display = 0x0400 | LED_VERDE;	// abilitazione via 1
			DISPLAY_tx_fifo.push_back(data_display);
			sio_enable_rx_interrupt_ch3(0x07);
			break;
		//case 0x05:					// luci via 5: peso da 2Kg su via 5
		case 0x10:					// luci via 5: peso da 2Kg su via 5
			sio_disable_rx_interrupt_ch3();
			data_display = 0x0500 | LED_VERDE;	// abilitazione via 1
			DISPLAY_tx_fifo.push_back(data_display);
			sio_enable_rx_interrupt_ch3(0x07);
			break;
		case 0x06:	// luci tutte accese
			sio_disable_rx_interrupt_ch3();
			data_display = 0x0100 | LED_VERDE;	// abilitazione via 1
			DISPLAY_tx_fifo.push_back(data_display);
			data_display = 0x0200 | LED_VERDE;	// abilitazione via 1
			DISPLAY_tx_fifo.push_back(data_display);
			data_display = 0x0300 | LED_VERDE;	// abilitazione via 1
			DISPLAY_tx_fifo.push_back(data_display);
			data_display = 0x0400 | LED_VERDE;	// abilitazione via 1
			DISPLAY_tx_fifo.push_back(data_display);
			data_display = 0x0500 | LED_VERDE;	// abilitazione via 8 (o 6 nel M3200)		
			DISPLAY_tx_fifo.push_back(data_display);
			sio_enable_rx_interrupt_ch3(0x07);
			break;
	}
}

void LedDrivers_DisplayTest()
{
	LedDrivers_sendData(0x017F);
	LedDrivers_sendData(0x0100);
	LedDrivers_sendData(0x0107);
	LedDrivers_sendData(0x09FF);		// no decide for digit
	LedDrivers_sendData(0x0101);
	LedDrivers_sendData(0x010F);
	LedDrivers_sendData(0x0107);
}