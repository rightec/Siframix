/**
@file		I2C_sw.cpp 
@brief		Function to manage a software I2C.

			It's a virtual I2C that uses the uC ports to implements the communication.
					
@author		
@date		18/01/2011
@version	01.00
*/

#include "i2c_sw.h"


/**
Initialice the I2C software module.
*/
void i2c_init(void)
{
	I2C_DATA_HIGH();
	I2C_CLOCK_HIGH();

	I2C_DATA_OUTPUT();			// dato in output
	I2C_CLOCK_OUTPUT();			// clock in output

	i2c_stop();
}


/**
Delay function.
*/
void i2c_delay(void)
{
int i;
	#ifdef __APP_IN_RAM__
		for(i=0; i< 100; i++)
		{
			asm("nop");
		}
	#else
		for(i=0; i< 400; i++)
		{
			asm("nop");
		}
	#endif
}

/**
I2C start sequence.
*/
void i2c_start(void)
{
	I2C_DATA_HIGH();			// tira su se già non c'è
	i2c_delay();

	I2C_CLOCK_HIGH();			// tira su se già non c'è
	i2c_delay();

	I2C_DATA_LOW();				// giu' la linea dei dati
	i2c_delay();

	I2C_CLOCK_LOW();			// giu' la linea del clock
	i2c_delay();
}


/**
I2C stop sequence.
From CK = 0, DO = x.
To  CK = 1, DO = 1.
*/
void i2c_stop(void)
{
	I2C_CLOCK_HIGH();
	I2C_CLOCK_OUTPUT();			// dato in output
	i2c_delay();

	I2C_DATA_HIGH();
	I2C_DATA_OUTPUT();			// dato in output
	i2c_delay();
}

/**
Sends a byte and wait the acknowledge signal.
@param c byte to send
@return true it the byte was acknowledged, false otherwise.
*/
bool i2c_putc(int c)
{
int i, ackCnt;
int bitTest = 0x80;
long timeout;

	I2C_DATA_OUTPUT();						// bus in output

	for(i=0; i<8; i++)						// sempre e comunque 8 shift
	{
		if( c & bitTest )
			I2C_DATA_HIGH();				// nuovo livello bit
		else
			I2C_DATA_LOW();

		i2c_delay();		//...NB... scommentare se necessita di un maggior delay

		I2C_CLOCK_HIGH();
		i2c_delay();
		
		I2C_CLOCK_LOW();
//		i2c_delay();		// ...NB... scommentare se necessita di un maggior delay

		bitTest >>= 1;
	}

	I2C_DATA_INPUT();						// bus in input
	i2c_delay();

	I2C_CLOCK_HIGH();						// su di nuovo il clock
	i2c_delay();
	i2c_delay();

	ackCnt = 0;
											// attesa ack a fine scrittura...
	for(timeout=0; timeout < 100; timeout++)
	{
		if( !I2C_DATA() )					// se il dispositivo da l'ack 
			ackCnt++;
		else
			ackCnt = 0;

		if( ackCnt > 10 )
			break;
	}

	I2C_CLOCK_LOW();

	I2C_DATA_LOW();							// blocchiamo il bus prima che vada a spasso
	I2C_DATA_OUTPUT();						// bus in output
	i2c_delay();

	if( ackCnt > 10 )
		return( True );

	return( False );
}
	
/**
Reads a byte
@return the byte read.
*/
byte i2c_getc(void)
{
int i;
byte c;

	I2C_DATA_INPUT();						// bus in input
	i2c_delay();							// dannato ritardo .....

	for(c=0, i=0; i<8; i++)
	{
		I2C_CLOCK_HIGH();
		i2c_delay();

		c <<= 1;

		if( I2C_DATA() )
			if( I2C_DATA() )
				if( I2C_DATA() )
					c |= 0x01;

		I2C_CLOCK_LOW();
		i2c_delay();
	}

	return( c );
}

/**
Acknowledge sequence.
@param ackLevel 1 data acknowledged, 0 NOT acknowledged
*/
void i2c_ack(int ackLevel)
{
	if( ackLevel )
		I2C_DATA_HIGH();
	else
		I2C_DATA_LOW();

	I2C_DATA_OUTPUT();				// bus in output
	i2c_delay();

	I2C_CLOCK_HIGH();
	i2c_delay();

	I2C_CLOCK_LOW();
	i2c_delay();
}


//@@-----------------------------------------------------------------------@@//
//@@
//@@  $Source:  $
//@@  Note di revisione:
//@@
//@@  $Revision: 1.2 $
//@@  $Date: 2002/01/08 00:00:00 $
//@@  			XXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXXX
//@@
//@@  $Log:  $
//@@
//@@  $Author: junior $
//@@  $RCSfile: I2C_SW.CPP,v $
//@@  $Revision: 1.1 $
//@@  $State: Exp $
//@@  $Date: 2002/12/16 00:00:00 $
//@@  $Name:  $
//@@-----------------------------------------------------------------------@@//
//@@-----------------------------------------------------------------------@@//
//@@                                                                       @@//
//@@                            C O M E T A                                @@//
//@@                                                                       @@//
//@@-----------------------------------------------------------------------@@//

