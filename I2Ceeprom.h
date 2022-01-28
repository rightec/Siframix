/**
@file		I2Ceeprom.h 
@brief		Driver for I2C EEPROM

			Chip: M24C01
			
@author		Nicola Molinazzi
@date		18/01/2011
@version	01.00
*/

#ifndef _I2CEEPROM_H_
#define _I2CEEPROM_H_

#include "global.h"
#include "i2c_sw.h"
/**
EEPROM write address
*/
#define EEPROM_ADDRESS_nWR 0xA0  
/**
EEPROM read address
*/
#define EEPROM_ADDRESS_RD 0xA1

/**
EEPROM size in byte
*/
#define EEPROM_SIZE 128 
/**
EEPROM page size in byte
*/
#define EEPROM_PAGE_SIZE 16

/**
Enables write operations
*/
#define EE_WR_PROTECT_LOW() (PinWReeprom = 0)
/**
Disables write operations
*/
#define EE_WR_PROTECT_HIGH() (PinWReeprom = 1)

/**
If this value is written whitin the eeprom at the address ADDRESS_ENTER_LOAD_MODE the program enter in loader mode
*/
#define ENTER_LOAD_MODE		0xAABBCCDD
/**
reset value 
*/
#define RST_ENTER_LOAD_MODE 0x00000000

/**
Address to check if stay in loader mode
*/
#define ADDRESS_ENTER_LOAD_MODE		0

/**
Addresses for weight channels calibration values
*/
#define ADDRESS_CHAN_ARE_MEM_CALIB 	16		// nei primi due indirizzi (16 e 17 per il canale 1) vengono salvati il type_of_calibration_offset ed il type_of_calibration_gain
#define ADDRESS_FACTORY_GAIN       		(ADDRESS_CHAN_ARE_MEM_CALIB + 2)		// 18
#define ADDRESS_FACTORY_OFFSET     		(ADDRESS_FACTORY_GAIN + sizeof(float))		// 22
#define ADDRESS_GAIN      				(ADDRESS_FACTORY_OFFSET + sizeof(short))	// 24
#define ADDRESS_OFFSET    				(ADDRESS_GAIN + sizeof(float))				// 28
#define ADDRESS_2Kg_READ				(ADDRESS_OFFSET + sizeof(short))			// 30
#define ADDRESS_CALIB					(ADDRESS_2Kg_READ + sizeof(short))			// 32
#define BASESTRUCTADDR				17	// numero di byte da scrivere per ogni canale
/*
* Canale 1: indirizzi dal 16 al 32
* Canale 2: indirizzi dal 33 al 49
* Canale 3: indirizzi dal 50 al 66
* Canale 4: indirizzi dal 67 al 83
* Canale 5: indirizzi dal 84 al 100
*/

/**
Addresses of string of last calibration date
*/
#define ADDRESS_DATE_OF_CALIB		(ADDRESS_CHAN_ARE_MEM_CALIB + EEPROM_PAGE_SIZE*6 )		// 112

void EE_init();

bool EE_byte_write(byte address, byte data);
bool EE_read_current_address(byte *address );
bool EE_random_byte_read(byte address, byte *data);
bool EE_sequential_current_read(byte *data, byte num_data_to_read = EEPROM_PAGE_SIZE);
bool EE_sequential_random_read(byte address, byte *data, byte num_data_to_read = EEPROM_PAGE_SIZE);
bool EE_write(void* data, byte num_data, byte address);
word EE_read_word(byte address);
int EE_read_int(byte address);
float EE_read_float(byte address);

#endif

