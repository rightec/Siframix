/**
@file		PWM.h
@brief		Functions to manage pwm signals.

			This module manages the pwm signals of the device:
			- pwm which moves pump motor 
			- pwm which controls EV enable
			- states of the state machines are set in the CPUSIFRA MANAGER
			
@author		Fregni Francesco
@date		18/01/2011
@version	01.00
*/

#include "pwm.h"

//debug
int conta = 0;
my_pwm pwm_mot[_NUM_PUMPS];
bool coverState_open = False;
int stato_pompa[_NUM_PUMPS], pre_stato_pompa[_NUM_PUMPS];
DecTimer PWM_timer[_NUM_PUMPS];
ENCODER_Motor_Status	 MotorStatus[_NUM_PUMPS];

void pwm_motors_reset()
{
	int i;
	
	for(i = 0; i < NUM_MAX_LINE; i++)
	{
		setPwmMotCycle(i, 0);
	}
}

/**
Init of the PWM1 port
This pwm signal controls speed of the pump motor
*/
void init_PWM_MOT1()
{
word regVal;

	TP0CTL0 = 0x03;	//-- Counter frequency  --> fc=fxx/8, PWM disable _TP0CE = 0
	TP0CTL1 = 0x04;	//-- Timer Mode --> PWM output mode, operation with internal clock counter
	TP0IOC0 = 0x04;	// low level when disable, l'uscita TPM00 è disabilitata
	//-- Pin setting ---
	PWM_MOT1 = 0;
	ioPWM_MOT1 = 0;
	_PF33 = 0;		// manual pag 103
	_PFC33  = 1;		//manual pag 102
	_PMC33  = 1;		//manual pag 100

	ioDirMOT1  = 0;
	DirMOT1 = 0;	// non utilizzato nel M3100
	
	pwm_mot[ 0 ].pwmCycle = 0;		// 10% corrisponde a 0,5V di SET_MOTORE (a valle del filtro RC) --> motore fermo pronto a partire
	pwm_mot[ 0 ].pwmFrq   = 8200;		// freq iniziale del pwm in Hz
 	pwm_mot[ 0 ].frqBase  = 20e+6/8.0;	// freq del clock disistema (legato al valore settato in TP0CTL0) in Hz

	regVal = (pwm_mot[ 0 ].frqBase/((float) pwm_mot[ 0 ].pwmFrq)) - 1;
	TP0CCR0 = regVal; // Frequenza pwm --> fpwm =  fc / (TP0CCR0 + 1)
	regVal = (regVal * pwm_mot[ 0 ].pwmCycle) / 100;
	TP0CCR1 = regVal;		// pwmCycle * (TO0CCR0 + 1) / 100
	asm("di");
	pwm_mot[ 0 ].enable = False;	// disable_PWM(PWM_ElettrValv, pwm_ev);
	asm("ei");

}

void init_PWM_MOT2()
{
word regVal;
	TP1CTL0 = 0x03;	//-- Counter frequency  --> fc=fxx/8, PWM disable _TP1CE = 0
	TP1CTL1 = 0x04;	//-- Timer Mode --> PWM output mode, operation with internal clock counter
	TP1IOC0 = 0x04;	// low level when disable, l'uscita TPM00 è disabilitata
	//-- Pin setting ---
	PWM_MOT2 = 0;
	ioPWM_MOT2 = 0;
	_PF35 = 0;		// manual pag 103
	_PFC35  = 1;		//manual pag 102
	_PMC35  = 1;		//manual pag 100

	pwm_mot[ 1 ].pwmCycle = 0;		// 10% corrisponde a 0,5V di SET_MOTORE (a valle del filtro RC) --> motore fermo pronto a partire
	pwm_mot[ 1 ].pwmFrq   = 8200;		// freq iniziale del pwm in Hz
 	pwm_mot[ 1 ].frqBase  = 20e+6/8.0;	// freq del clock disistema (legato al valore settato in TP0CTL0) in Hz

	regVal = (pwm_mot[ 1 ].frqBase/((float) pwm_mot[ 1 ].pwmFrq)) - 1;
	TP1CCR0 = regVal; // Frequenza pwm
	regVal = (regVal * pwm_mot[ 1 ].pwmCycle) / 100;
	TP1CCR1 = regVal;
	asm("di");
	pwm_mot[ 1 ].enable = False;
	asm("ei");
}

void init_PWM_MOT3()
{
word regVal;

	TP2CTL0 = 0x03;	//-- Counter frequency  --> fc=fxx/8, PWM disable _TP1CE = 0
	TP2CTL1 = 0x04;	//-- Timer Mode --> PWM output mode, operation with internal clock counter
	TP2IOC0 = 0x04;	// low level when disable, l'uscita TPM00 è disabilitata
	//-- Pin setting ---
	PWM_MOT3 = 0;
	ioPWM_MOT3 = 0;
	_PF96 = 0;		// manual pag 103
	_PMC96  = 1;		//manual pag 100
	_PFC96  = 1;		//manual pag 102
	_PFCE96 = 1;
	
	pwm_mot[ 2 ].pwmCycle = 0;		// 10% corrisponde a 0,5V di SET_MOTORE (a valle del filtro RC) --> motore fermo pronto a partire
	pwm_mot[ 2 ].pwmFrq   = 8200;		// freq iniziale del pwm in Hz
 	pwm_mot[ 2 ].frqBase  = 20e+6/8.0;	// freq del clock disistema (legato al valore settato in TP0CTL0) in Hz

	regVal = (pwm_mot[ 2 ].frqBase/((float) pwm_mot[ 2 ].pwmFrq)) - 1;
	TP2CCR0 = regVal; // Frequenza pwm
	regVal = (regVal * pwm_mot[ 2 ].pwmCycle) / 100;
	TP2CCR1 = regVal;
	asm("di");
	pwm_mot[ 2 ].enable = False;
	asm("ei");
}

void init_PWM_MOT4()
{
word regVal;

	TP3CTL0 = 0x03;	//-- Counter frequency  --> fc=fxx/8, PWM disable _TP1CE = 0
	TP3CTL1 = 0x04;	//-- Timer Mode --> PWM output mode, operation with internal clock counter
	TP3IOC0 = 0x04;	// low level when disable, l'uscita TPM00 è disabilitata
	//-- Pin setting ---
	PWM_MOT4 = 0;
	ioPWM_MOT4 = 0;
	_PF94 = 0;		// manual pag 103
	_PMC94  = 1;		//manual pag 100
	_PFC94  = 0;		//manual pag 102
	_PFCE94 = 1;

	pwm_mot[ 3 ].pwmCycle = 0;		// 10% corrisponde a 0,5V di SET_MOTORE (a valle del filtro RC) --> motore fermo pronto a partire
	pwm_mot[ 3 ].pwmFrq   = 8200;		// freq iniziale del pwm in Hz
 	pwm_mot[ 3 ].frqBase  = 20e+6/8.0;	// freq del clock disistema (legato al valore settato in TP0CTL0) in Hz

	regVal = (pwm_mot[ 3 ].frqBase/((float) pwm_mot[ 3 ].pwmFrq)) - 1;
	TP3CCR0 = regVal; // Frequenza pwm
	regVal = (regVal * pwm_mot[ 3 ].pwmCycle) / 100;
	TP3CCR1 = regVal;
	asm("di");
	pwm_mot[ 3 ].enable = False;
	asm("ei");
}

void init_PWM_MOT5()
{
word regVal;

	TP4CTL0 = 0x03;	//-- Counter frequency  --> fc=fxx/8, PWM disable _TP1CE = 0
	TP4CTL1 = 0x04;	//-- Timer Mode --> PWM output mode, operation with internal clock counter
	TP4IOC0 = 0x04;	// low level when disable, l'uscita TPM00 è disabilitata
	//-- Pin setting ---
	PWM_MOT5 = 0;
	ioPWM_MOT5 = 0;
	_PF92 = 0;		// manual pag 103
	_PMC92  = 1;		//manual pag 100
	_PFC92  = 0;		//manual pag 102
	_PFCE92 = 1;

	pwm_mot[ 4 ].pwmCycle = 0;		// 10% corrisponde a 0,5V di SET_MOTORE (a valle del filtro RC) --> motore fermo pronto a partire
	pwm_mot[ 4 ].pwmFrq   = 8200;		// freq iniziale del pwm in Hz
 	pwm_mot[ 4 ].frqBase  = 20e+6/8.0;	// freq del clock disistema (legato al valore settato in TP0CTL0) in Hz

	regVal = (pwm_mot[ 4 ].frqBase/((float) pwm_mot[ 4 ].pwmFrq)) - 1;
	TP4CCR0 = regVal; // Frequenza pwm
	regVal = (regVal * pwm_mot[ 4 ].pwmCycle) / 100;
	TP4CCR1 = regVal;
	asm("di");
	pwm_mot[ 4 ].enable = False;
	asm("ei");
}

/**
Init of the PWM2 port
This pwm signal controls enabling of the Electromagnetic valves
*/
void init_PWM_EVuC()
{
}

/**
manager for management of motors accelerations/decellerations
States of the state-machine are set in CPUSIFRA manager module
*/
void pwmManager()
{
	int m_line;
	
	// stati POMPA DISABILITATA, ABILITAZIONE, CONTROLLO, ARRESTO
	for( m_line = 0; m_line < _NUM_PUMPS; m_line++)
	{
		switch( stato_pompa[m_line])
		{
			case POMPA_DISABILITA:
				break;

			case ABILITAZIONE:
				if(getCoverPumpState())
				{
					pre_stato_pompa[m_line] = stato_pompa[m_line];
					stato_pompa[m_line] = COVER_APERTA;
				}
				else
				{
					setBlockRelay(m_line, 1);	//chiudo il relay
					enable_MOT(m_line);
					PWM_timer[m_line].Preset(32);
					stato_pompa[m_line]  = CONTROLLO;
				}
				break;

			 case CONTROLLO:
			 	if(PWM_timer[m_line].Match())
				{
				 	if(getCoverPumpState())
					{
						pre_stato_pompa[m_line] = stato_pompa[m_line];
						stato_pompa[m_line] = COVER_APERTA;
					}
					PWM_timer[m_line].Preset(20);
			 	}
				break;

			case ARRESTO:
				setBlockRelay(m_line, 0);		//cpld_pin_BLK1 = 0;	// apro il relay
				disable_PWM_MOT(m_line);
				setPwmMotCycle(m_line, 0);
				stato_pompa[m_line] = POMPA_DISABILITA;
				break;

			case COVER_APERTA:
				if(getCoverPumpState())
				{
					setBlockRelay(m_line, 0);		// apro il relay
					setPwmMotCycle(m_line, _PWM_CYCLE_0VEL);
				}
				else // se la pompa stava andando la facciamo riprendere
				{
					if((pre_stato_pompa[m_line] == ABILITAZIONE) || (pre_stato_pompa[m_line] == CONTROLLO))
					{
						stato_pompa[m_line] = ABILITAZIONE;
					}
				}
				break;
			
			default:
				break;
		}
	}
}

/**
manager for management of the electromagnetic valve
States of the state-machine are set in CPUSIFRA manager module
*/
void EVsManager()
{

}

byte changeEVstatus(byte newStatus)
{
	return 0;
}

/**
enabling of the PWM2 signal for electromagnetic valve
*/
void enable_PWM_EV()
{
}

/**
disabling of the PWM2 signal for electromagnetic valve
*/
void disable_PWM_EV()
{
}

/**
starts the PWM1 signal for pump motor
*/
void enable_PWM_MOT(int line)
{
	asm("di");
	pwm_mot[line].enable = True;
	pwm_mot[line].counter = 0;
	asm("ei");
	switch(line)
	{
		case 0:
			_TP0CE = 1;
			break;
		case 1:
			_TP1CE = 1;
			break;
		case 2:
			_TP2CE = 1;
			break;
		case 3:
			_TP3CE = 1;
			break;
		case 4:
			_TP4CE = 1;
			break;
	}
}

/**
disabling of the PWM1 signal for pump motor
*/
void disable_PWM_MOT(int line)
{
	asm("di");
	pwm_mot[line].enable = False;
	asm("ei");
	switch(line)
	{
		case 0:
			_TP0CE = 0;
			break;
		case 1:
			_TP1CE = 0;
			break;
		case 2:
			_TP2CE = 0;
			break;
		case 3:
			_TP3CE = 0;
			break;
		case 4:
			_TP4CE = 0;
			break;
	}
}

/**
set acceleration/decellaration of the pumps
*/
void reg_pump_vel(byte line, byte Command)
{
	pwm_mot[line].upper = Command;
}

/**
Attivazione/disattivazione reley alimentazione motore:
e = True --> attivo; e = False --> disattivo
*/
void setBlockRelay(int i, int e)
{
	switch( i )
	{
		case 0:
			cpld_pin_BLK1 = e;
			break;
		case 1:
			cpld_pin_BLK2 = e;
			break;
		case 2:
			cpld_pin_BLK3 = e;
			break;
		case 3:
			cpld_pin_BLK4 = e;
			break;
		case 4:
			cpld_pin_BLK5 = e;
			break;
	}
}

void enable_MOT(int i)
{
	switch( i )
	{
		case 0:
			init_PWM_MOT1();
			break;
		case 1:
			init_PWM_MOT2();
			break;
		case 2:
			init_PWM_MOT3();
			break;
		case 3:
			init_PWM_MOT4();
			break;
		case 4:
			init_PWM_MOT5();
			break;
	}
	enable_PWM_MOT( i );
}

void setRampPump(int i, byte command)
{
	pwm_mot[ i ].upper = command;
}

int getRampPump(int i)
{
	return pwm_mot[ i ].upper;
}

void setSpeedPump(int i, int command)
{
	pwm_mot[ i ].pwmCycle = command;
}

int getSpeedPump(int i)
{
int c = 0;
	c = pwm_mot[ i ].pwmCycle;
	if(c >= 0)
	{
		if(c <= 100)
		{
			return c;
		}
	}
	return 0;
}

void setStatePump(int i, int command)
{
	stato_pompa[ i ] = command;
}

int getStatePump(int i)
{
	return stato_pompa[ i ];
}

void setPwmMotCycleIncrease(int i, int increment)
{

	if(pwm_mot[ i ].pwmCycle <  _PWM_CYCLE_MAX)
	{
		pwm_mot[ i ].pwmCycle += increment;
		setPwmCycle( i , pwm_mot[ i ].pwmCycle);
	}
}

void setPwmMotCycleDecrease(int i, int decrement)
{

	if(pwm_mot[ i ].pwmCycle >  _PWM_CYCLE_LOW)
	{
		pwm_mot[ i ].pwmCycle -= decrement;
		setPwmCycle( i , pwm_mot[ i ].pwmCycle);
	}
}

void setPwmMotCycle(int i, int value)
{
	pwm_mot[ i ].pwmCycle = value;
	setPwmCycle( i , pwm_mot[ i ].pwmCycle);
}

void setPwmCycle( int pwm, int duty_cycle)
{
int regVal;
	switch( pwm )
	{
		case 0:
			regVal = TP0CCR0;
			regVal = (int)((regVal * duty_cycle) / 100);
			TP0CCR1 = regVal;
			break;
		case 1:
			regVal = TP1CCR0;
			regVal = (int)((regVal * duty_cycle) / 100);
			TP1CCR1 = regVal;
			break;
		case 2:
			regVal = TP2CCR0;
			regVal = (int)((regVal * duty_cycle) / 100);
			TP2CCR1 = regVal;
			break;
		case 3:
			regVal = TP3CCR0;
			regVal = (int)((regVal * duty_cycle) / 100);
			TP3CCR1 = regVal;
			break;
		case 4:
			regVal = TP4CCR0;
			regVal = (int)((regVal * duty_cycle) / 100);
			TP4CCR1 = regVal;
			break;
	}
}

bool getCoverPumpState()
{
	if(cpld_CoverPumpOpen)
	{
		coverState_open = True;
		return True;
	}
	else
	{
		coverState_open= False;
	}
	
	return False;
}

