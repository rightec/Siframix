/**
@file		Sound.cpp
@brief		Functions to manage board's buzzer.

@author		
@date		18/01/2011
@version	01.0
*/


#include "sound.h"


static	CICALINO				BUZZERdata;


void setPWM(word frq, byte cycle);	 
void buzzer_start();
void buzzer_stop();

/**
Initializes the buzzer module.
*/
void buzzer_init(void)
{	
	//-- Counter frequency  --> fc=fxx/8
	//-- Bits in TP0CTL0 register
	_TP0CKS0 = 1;
	_TP0CKS1 = 1;
	_TP0CKS2 = 0;

	//-- Timer Mode --> PWM output mode
	//-- Bits in TP0CTL1 register
	_TP0MD0 = 0;
	_TP0MD1 = 0;
	_TP0MD2 = 1;
	_TP0EEE = 0;	// disable operation with external event count input
	//_TP0EST = 0;
	
	//-- TP0	IOC0
	_TP0OL0 = 0;	// low level when disable
	_TP0OE0 = 1;
	_TP0OL1 = 0;
	_TP0OE1 = 0; // l'uscita TPM01 è dedicata ad altro
	
	TP0CCR0 = 304; // Frequenza di interrupt --> fi =  fc / (TP0CCR0 + 1)
	TP0CCR1 = 152;
	//-- Pin setting ---
	PinBUZ = 0;
	_PF32= 0;		// manual pag 103
	_PFCE32  = 1;		//manual pag 102
	_PFC32  = 1;		//manual pag 102
	_PMC32  = 1;		//manual pag 100
	ioPinBUZ = 0;
		
	PWM_MOT1 = 0;	
	ioPWM_MOT1 = 0;	
	
	_TP0CE = 0; // TMP0 disable
	
	// Il timer viene inizializzato nel main insieme agli altri timer
	BUZZERdata.isPlaying = false;
	BUZZERdata.pwmCycle = 50;
	BUZZERdata.pwmFrq   = 8200;
	BUZZERdata.sound   = NO_SOUND;
	BUZZERdata.frqBase  = 20e+6/8.0;
	BUZZERdata.buzTimer.Stop();
	BUZZERdata.alarmTimer.Stop();
	BUZZERdata.length = 200;
	BUZZERdata.nota = _DO;
	setPWM(BUZZERdata.pwmFrq, BUZZERdata.pwmCycle);
}

/**
Starts the buzzer
@param SoundToPlay sound to play
*/
void goBuzzer(KindOfSound SoundToPlay)
{

	if (!BUZZERdata.isPlaying)
	{
		BUZZERdata.sound = SoundToPlay;
		switch (BUZZERdata.sound){
			case KEY_LIKE :
				BUZZERdata.length = 200; 
				break;
			
			case LONG_LIKE :
				BUZZERdata.length = 600;	
				break;

			case ALARM_LIKE :
				BUZZERdata.length = 500;
				break;
			default :
				BUZZERdata.length = 500; 
				break;
		}
		buzzer_start();
	}
}


/**
Call this function in the main loop. manage the buzzer module.
*/
void buzzerManager(void)
{
	switch (BUZZERdata.sound)
	{
		case KEY_LIKE :
		case LONG_LIKE:
			if (BUZZERdata.buzTimer.Match())
			{
				buzzer_stop();	 		
			}
			break;
		case ALARM_LIKE:
			if (BUZZERdata.buzTimer.Match())
			{
				buzzer_stop();	 	
				BUZZERdata.isPlaying = true;
				BUZZERdata.sound = ALARM_LIKE;
				BUZZERdata.alarmTimer.Preset(800);
			}
			if(BUZZERdata.alarmTimer.Match())
			{
				buzzer_start();
			}
			break;
		default:
			if (BUZZERdata.buzTimer.Match())
			{
				buzzer_stop();	 		
			}
		break;
	}	
}	

/**
stops the buzzer
*/
void buzzer_stop()
{
	BUZZERdata.isPlaying = false;
	BUZZERdata.sound = NO_SOUND;
	BUZZERdata.alarmTimer.Stop();
	BUZZERdata.buzTimer.Stop();
	_TP0CE = 0; // TMP4 stop
}

/**
Starts the buzzer.
*/
void buzzer_start()
{
	//BUZZERdata.isPlaying = false;
	setPWM(BUZZERdata.pwmFrq, BUZZERdata.pwmCycle);
	BUZZERdata.buzTimer.Preset(BUZZERdata.length);
	_TP0CE = 1; // TMP4 start
}

/**
@return true if the buzzer is playing, false otherwise
*/
bool buzzer_isPlaying()
{
	return BUZZERdata.isPlaying;
}

/**
@return the last sound played
*/
KindOfSound	buzzerSound()
{
	return BUZZERdata.sound;
}

/**
Stops the buzzer
*/
void shutBuzzer()
{
	buzzer_stop();
}

/**
Sets PWM frequency and duty cycle.
*/
void setPWM(word frq, byte cycle)
{
	word regVal;

	regVal = (BUZZERdata.frqBase/((float) BUZZERdata.pwmFrq)) - 1;
	TP0CCR0 = regVal; // Frequenza pwm --> fi =  fc / (TP0CCR0 + 1)
	regVal = (regVal * BUZZERdata.pwmCycle);
	regVal = regVal /100;
	TP0CCR1 = regVal;
}







///////////////////////////////////////////////////////////////////////////////
//	1)
//	A causa della variabilità della lunghezza del giro di programma specie
//	con i cambi delle videate, non e possibile basare lo STOP del suono solo
//	sul giro do programma, ma bisogna usare gli INTERRUPTS associati agli
//	altri TIMERS.
//	2)
//	Per ora vado in PWM. Prossimo obiettivo SQUARE WAVE GENERATION
//
//////////////////////////////////////////////////////////////////////////////

























