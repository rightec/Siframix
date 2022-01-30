/**
@file		ieeTest.h
@brief		Functions to manage test on IEE PROM.
@see 		ieeTest.cpp
@author		
@date		28/01/2022
@version	01.00
*/


#ifndef _IEETEST_H_
#define _IEETEST_H_


#include "global.h"

bool testBase ();
bool read_factory_gain_param(byte numchan, float *FactoryGain, float *Gain, int *adc2Kg, int *adc2Kgdx);



#endif // _IEETEST_H_



























