/**
@file		I2C_sw.h
@brief		Function to manage a software I2C.

			It's a virtual I2C that uses the uC ports to implements the communication.
					
@author		
@date		18/01/2011
@version	01.00
*/


#ifndef _I2C_SW_H_
#define _I2C_SW_H_


#include "global.h"

/**
Sets the data line as output.
*/
#define	I2C_DATA_OUTPUT()	(ioPinI2C_SDA = 0)		
/**
Sets the data line as input.
*/
#define	I2C_DATA_INPUT()	(ioPinI2C_SDA = 1)		
/**
Sets the data line high.
*/
#define	I2C_DATA_HIGH()		(PinI2C_SDA = 1)		
/**
Sets the data line low.
*/
#define	I2C_DATA_LOW()		(PinI2C_SDA = 0)		
/**
Reads the state of the data bus line.
*/
#define	I2C_DATA()			(PinI2C_SDA == 1)

/**
Sets the clock line as output
*/
#define	I2C_CLOCK_OUTPUT()	(ioPinI2C_SCL	= 0) 		
/**
Sets the clock line high.
*/
#define	I2C_CLOCK_HIGH()	(PinI2C_SCL = 1)
/**
Sets the clock line low.
*/
#define	I2C_CLOCK_LOW()		(PinI2C_SCL = 0)


#ifdef __cplusplus
extern "C" {
#endif				

							// common .text section

void i2c_init(void);

void i2c_delay(void);
void i2c_start(void);
void i2c_stop(void);
bool i2c_putc(int c);
byte i2c_getc(void);
void i2c_ack(int ackLevel);



#ifdef __cplusplus
}
#endif

#endif
//--------------------------------------------------------//
// Definizione delle variabili
//--------------------------------------------------------//






//@@-----------------------------------------------------------------------@@//
//@@                                                                             
//@@  $Source:  $
//@@  Note di revisione:
//@@
//@@  $Log:  $
//@@
//@@  $Author: junior $
//@@  $RCSfile: I2C_SW.H,v $
//@@  $Revision: 1.1 $
//@@  $State: Exp $
//@@  $Date: 2004/01/02 06:49:06 $
//@@  $Name:  $
//@@-----------------------------------------------------------------------@@//
//@@-----------------------------------------------------------------------@@//
//@@                                                                       @@//
//@@                            C O M E T A                                @@//
//@@                                                                       @@//
//@@-----------------------------------------------------------------------@@//

