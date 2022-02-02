/**
@file		ieetest.cpp
@brief		Functions to manage IEE Test.

@see 		ieetest.h
@author		
@date		28/01/2022
@version	01.00
*/


#include "ieeTest.h"

byte numchan = 0;  // 
	
float FactoryGain = 10.5;
float Gain = 9.5;

int adc2Kg = 2001;
int adc2Kgdx = 2004;
byte type_of_calibration_test = CHAN_CALIB_CHECK_FACTORY_VAL;


bool testBase ()
{

	/** TEST BASE
	First Read for EEPROM location
	Second Compare with data test
	Third Write Data Test
	Fourth Re Read what has been written
	*/

	bool bWriteDone = false;
	bool bReadDone = false;
	
	byte address;
	byte data;

	float ReadFactoryGain = 0.0;
	float ReadGain = 0.0;
	int Readadc2Kg = 0;
	int Readadc2Kgdx = 0;

	/**READ FROM EEPROM **/
	bReadDone  = read_factory_gain_param(numchan, &ReadFactoryGain, &ReadGain, &Readadc2Kg, &Readadc2Kgdx);
	if (ReadFactoryGain == FactoryGain){
		// Read Gain - Float 
		if (ReadGain == Gain){
			// Read adc2Kg
			if (Readadc2Kg == adc2Kg){
				// Read adc2Kgdx
				if (Readadc2Kgdx ==adc2Kgdx) {
					// Read type of calib
					address = ADDRESS_CHAN_ARE_MEM_CALIBRATE + 1 + numchan*BASESTRUCTADDR;
					bReadDone = EE_random_byte_read(address,&data); 
					if (data == type_of_calibration_test){
						// Reading completed OK. The 1st time will always file
						bReadDone = true;
					} else {
						//
					}
				} else {
					//
				}
			} else {
				//
			}
		} else {
			//
		}
	} else {
		//
	}


	/** WRITE TO IEEE*/
	
	bWriteDone = backup_factory_gain_param(numchan, FactoryGain, adc2Kg, adc2Kgdx);
	if (bWriteDone == true){
		// Re - read what has been written
		// Reset var
		ReadFactoryGain = 0.0;
	       ReadGain = 0.0;
	       Readadc2Kg = 0;
	       Readadc2Kgdx = 0;
		 data = 0xAF;
		   
		/**READ FROM EEPROM **/
		bReadDone  = read_factory_gain_param(numchan, &ReadFactoryGain, &ReadGain, &Readadc2Kg, &Readadc2Kgdx);
		if (ReadFactoryGain == FactoryGain){
			// Read Gain - Float 
			if (ReadGain == Gain){
				// Read adc2Kg
				if (Readadc2Kg == adc2Kg){
					// Read adc2Kgdx
					if (Readadc2Kgdx ==adc2Kgdx) {
						// Read type of calib
						address = ADDRESS_CHAN_ARE_MEM_CALIBRATE + 1 + numchan*BASESTRUCTADDR;
						bReadDone = EE_random_byte_read(address,&data); 
						if (data == type_of_calibration_test){
							// Reading completed OK. The 1st time will always file
							bReadDone = true;
						} else {
							//
						}
					} else {
						//
					}
				} else {
					//
				}	
			} else {
				//
			}
		} else {
			//
		}		
	} else {
		// Write error
		return false;	
	}

  return bReadDone;
}


bool read_factory_gain_param(byte numchan, float *FactoryGain, float *Gain, int *adc2Kg, int *adc2Kgdx)
{
	// Read Factory Gain - Float 
	byte address = ADDRESS_FACTORY_GAIN + numchan*BASESTRUCTADDR;
	float dataFloat = EE_read_float(address); 
	*FactoryGain = dataFloat;

	// Read Gain - Float 
	address = ADDRESS_GAIN + numchan*BASESTRUCTADDR;
	dataFloat = EE_read_float(address); 
	*Gain = dataFloat;

	// Read adc2Kg
	address = ADDRESS_2Kg_READ + numchan*BASESTRUCTADDR;
	int dataInt = EE_read_int(address); 
	*adc2Kg = dataInt;

	// Read adc2Kgdx
	address = ADDRESS_2KgDx_READ + numchan*BASESTRUCTADDR;
	dataInt = EE_read_int(address); 
	*adc2Kgdx = dataInt;

	// Read type of calib
	address = ADDRESS_CHAN_ARE_MEM_CALIBRATE + 1 + numchan*BASESTRUCTADDR;
	byte data;
	bool bReadDone = EE_random_byte_read(address,&data); 

	return bReadDone;
	
	

}










