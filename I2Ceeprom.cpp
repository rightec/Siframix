/**
@file		I2Ceeprom.cpp 
@brief		Driver for I2C EEPROM

			Chip: M24C01
			
@author		Nicola Molinazzi
@date		18/01/2011
@version	01.00
*/


#include "I2Ceeprom.h"


static bool EE_page_write(byte address, byte *data, byte num_data = EEPROM_PAGE_SIZE);

/**
EEPROM initialization
*/
void EE_init()
{
	ioPinWReeprom = 0;
	EE_WR_PROTECT_HIGH();
}

/**
Writes a byte to the EEPROM

@param address write address
@param data byte to write
@return true if the write process ends correctly, false otherwise
*/
bool EE_byte_write(byte address, byte data)
{
	bool rw_ok = True;
	int i;
	
	EE_WR_PROTECT_LOW();
	for (i = 0; i < 5; i++)
		i2c_delay();
	
	i2c_start();
	if (!i2c_putc(EEPROM_ADDRESS_nWR))
		rw_ok = false;
	if (!i2c_putc(address))
		rw_ok = false;
	if (!i2c_putc(data))
		rw_ok = false;

	
	i2c_stop();
	
	EE_WR_PROTECT_HIGH();
	
	for (i = 0; i < 10; i++) 
			i2c_delay();
	return rw_ok;
}

bool EE_page_write(byte address, byte *data, byte num_data)
{
	bool rw_ok = True;
	int i;
	
	if (num_data > EEPROM_PAGE_SIZE) // non posso scrivere più di una pagina
		return false;
	if (((address & (EEPROM_PAGE_SIZE-1)) +  num_data) > EEPROM_PAGE_SIZE) // se sforo la pagina riscivo ciclicamente
		return false;
	
	for (i = 0; i < 5; i++) 
		i2c_delay();
	
	i2c_start();
	
	if (!i2c_putc(EEPROM_ADDRESS_nWR))
		rw_ok = false;
	if (!i2c_putc(address))
		rw_ok = false;
	
	for (i = 0; i < num_data; i++)
	{
		if (!i2c_putc(data[i]))
			rw_ok = false;
	}
	
	i2c_stop();
	
	for (i = 0; i < 10; i++) 
		i2c_delay();

	return rw_ok;
}

/**
Reads a byte from the current address loaded into the EEPROM

@param address byte read
@return true if the read process ends correctly, false otherwise
*/
bool EE_read_current_address(byte *address )
{
	bool rw_ok = True;
	int i;
	i2c_start();

	if (!i2c_putc(EEPROM_ADDRESS_RD))
		rw_ok = false;

	*address = i2c_getc();
	i2c_ack(1);
	
	i2c_stop();

	for (i = 0; i < 10; i++) 
		i2c_delay();
	
	return rw_ok;
}

/**
Reads a byte from the EEPROM

@param address read address
@param data byte read
@return true if the read process ends correctly, false otherwise
*/
bool EE_random_byte_read(byte address, byte *data)
{
	bool rw_ok = True;
	int i;
	i2c_start();
	
	if (!i2c_putc(EEPROM_ADDRESS_nWR))
		rw_ok = false;
	if (!i2c_putc(address))
		rw_ok = false;

	i2c_start();
	
	if (!i2c_putc(EEPROM_ADDRESS_RD))
		rw_ok = false;

	*data = i2c_getc();
	i2c_ack(1);
	
	i2c_stop();

	for (i = 0; i < 10; i++) 
		i2c_delay();
	return rw_ok;

}

/**
Reads bytes from current address loaded into the EEPROM

@param data pointer to the read buffer
@param num_data_to_read number of byte to read
@return true if the read process ends correctly, false otherwise
*/
bool EE_sequential_current_read(byte *data, byte num_data_to_read)
{
	bool rw_ok = True;
	int i;
	
	i2c_start();
	if (!i2c_putc(EEPROM_ADDRESS_RD))
		rw_ok = false;

	for (i=0; i< num_data_to_read -1; i++)
	{
		data[i] = i2c_getc();
		i2c_ack(0);
	}
	
	data[i] = i2c_getc();
	i2c_ack(1);
	
	i2c_stop();
	return rw_ok;
}

/**
Reads bytes from the EEPROM

@param address start read EEPROM address
@param data pointer to the read buffer
@param num_data_to_read number of byte to read
@return true if the read process ends correctly, false otherwise
*/
bool EE_sequential_random_read(byte address, byte *data, byte num_data_to_read)
{
	bool rw_ok = True;
	int i;
	
	i2c_start();
	
	if (!i2c_putc(EEPROM_ADDRESS_nWR))
		rw_ok = false;
	if (!i2c_putc(address))
		rw_ok = false;

	i2c_start();

	if (!i2c_putc(EEPROM_ADDRESS_RD))
		rw_ok = false;
		
	for (i=0; i< num_data_to_read -1; i++)
	{
		data[i] = i2c_getc();
		i2c_ack(0);
	}
	
	data[i] = i2c_getc();
	i2c_ack(1);

	
	i2c_stop();
	return rw_ok;

}

/**
Writes bytes to the EEPROM

@param data pointer to the wite buffer
@param num_data number of byte to write
@param address start EEPROM write address
@return true if the write process ends correctly, false otherwise
*/
bool EE_write(void* data, byte num_data, byte address)
{
	bool rw_ok = True;
	int i;
	
	byte num_full_page_to_write;
	byte num_odd_byte_to_write_1 ;
	byte num_odd_byte_to_write_2;

	for (i = 0; i < 10; i++)
		i2c_delay();
	EE_WR_PROTECT_LOW();
	for (i = 0; i < 10; i++)
		i2c_delay();
	
	num_odd_byte_to_write_1 = EEPROM_PAGE_SIZE - (address % EEPROM_PAGE_SIZE);
	if (num_odd_byte_to_write_1 > num_data)
	{
		num_odd_byte_to_write_1 = num_data;
		num_odd_byte_to_write_2 = 0;
		num_full_page_to_write = 0;
		
	}else
	{
		num_full_page_to_write = (num_data - num_odd_byte_to_write_1) / EEPROM_PAGE_SIZE;
		num_odd_byte_to_write_2 = (num_data - num_odd_byte_to_write_1) % EEPROM_PAGE_SIZE;
		
	}
	if (num_odd_byte_to_write_1 != 0)
		rw_ok = EE_page_write(address, (byte*)data, num_odd_byte_to_write_1);
	
	i = 0;
	while (i< num_full_page_to_write && rw_ok)
	{
		rw_ok = EE_page_write(address + num_odd_byte_to_write_1 + (i * EEPROM_PAGE_SIZE), (byte*)((byte*)data + num_odd_byte_to_write_1 + (i * EEPROM_PAGE_SIZE)), EEPROM_PAGE_SIZE);
		i++;
	}
	if (num_odd_byte_to_write_2 != 0 && rw_ok)
		rw_ok = EE_page_write(address + num_odd_byte_to_write_1 + (num_full_page_to_write * EEPROM_PAGE_SIZE), (byte*)((byte*)data+ num_odd_byte_to_write_1 + (num_full_page_to_write * EEPROM_PAGE_SIZE)), num_odd_byte_to_write_2);

	EE_WR_PROTECT_HIGH();
	
	for (i = 0; i < 10; i++) 
		i2c_delay();
	
	return rw_ok;
}

/**
Reads a word from the EEPROM

@param address start EEPROM read address
@return the read integer
*/
word EE_read_word(byte address)
{
	byte data[2];
	int i;
	byte num_odd_byte_to_read_1 ;
	byte num_odd_byte_to_read_2;

	
	num_odd_byte_to_read_1 = EEPROM_PAGE_SIZE - (address % EEPROM_PAGE_SIZE);
	if (num_odd_byte_to_read_1 > 2)
	{
		num_odd_byte_to_read_1 = 2;
		num_odd_byte_to_read_2 = 0;
		
	}else
	{
		num_odd_byte_to_read_2 = 2 - num_odd_byte_to_read_1;
	}
	
	if(num_odd_byte_to_read_1 != 0)
		EE_sequential_random_read(address, data, num_odd_byte_to_read_1);
	
	if(num_odd_byte_to_read_2 != 0)
		EE_sequential_random_read(address + num_odd_byte_to_read_1, (byte*)((byte*)data + num_odd_byte_to_read_1), num_odd_byte_to_read_2);

	for (i = 0; i < 10; i++) 
		i2c_delay();

	return *((word*)data);	
}

/**
Reads a integer from the EEPROM

@param address start EEPROM read address
@return the read integer
*/
int EE_read_int(byte address)
{
	byte data[4];
	int i;
	byte num_odd_byte_to_read_1 ;
	byte num_odd_byte_to_read_2;

	
	num_odd_byte_to_read_1 = EEPROM_PAGE_SIZE - (address % EEPROM_PAGE_SIZE);
	if (num_odd_byte_to_read_1 >4)
	{
		num_odd_byte_to_read_1 = 4;
		num_odd_byte_to_read_2 = 0;
		
	}else
	{
		num_odd_byte_to_read_2 = 4 - num_odd_byte_to_read_1;
	}
	
	if(num_odd_byte_to_read_1 != 0)
		EE_sequential_random_read(address, data, num_odd_byte_to_read_1);
	
	if(num_odd_byte_to_read_2 != 0)
		EE_sequential_random_read(address + num_odd_byte_to_read_1, (byte*)((byte*)data + num_odd_byte_to_read_1), num_odd_byte_to_read_2);

	for (i = 0; i < 10; i++) 
		i2c_delay();

	return *((int*)data);	
}

/**
Reads afloat from the EEPROM

@param address start EEPROM read address
@return the read float
*/
float EE_read_float(byte address)
{
	byte data[sizeof(float)];
	int i;
	byte num_odd_byte_to_read_1 ;
	byte num_odd_byte_to_read_2;


	memset((void*)&data[0],0,sizeof(float));
	
	num_odd_byte_to_read_1 = EEPROM_PAGE_SIZE - (address % EEPROM_PAGE_SIZE);
	if (num_odd_byte_to_read_1 >sizeof(float))
	{
		num_odd_byte_to_read_1 = sizeof(float);
		num_odd_byte_to_read_2 = 0;
		
	}else
	{
		num_odd_byte_to_read_2 = sizeof(float) - num_odd_byte_to_read_1;
	}
	
	if(num_odd_byte_to_read_1 != 0)
		EE_sequential_random_read(address, data, num_odd_byte_to_read_1);
	
	if(num_odd_byte_to_read_2 != 0)
		EE_sequential_random_read(address + num_odd_byte_to_read_1, data + num_odd_byte_to_read_1, num_odd_byte_to_read_2);

	for (i = 0; i < 10; i++) 
		i2c_delay();
	
	return *((float*)data);
}

