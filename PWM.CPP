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
int enable_EVs;
bool coverState_open = False;
my_pwm pwm_ev, pwm_mot[_NUM_PUMPS];	// PWM controllo Elettrovalvole TP10 , pin P34
int stato_pompa[_NUM_PUMPS];
int pre_stato_pompa[_NUM_PUMPS];
byte state_EVs, previousstate_EVs;
DecTimer PWM_timer[_NUM_PUMPS];
DecTimer EV_timer;
ENCODER_Motor_Status	 MotorStatus;

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
	DirMOT1 = 1;

	pwm_mot[ 0 ].pwmCycle = 2;		// 10% corrisponde a 0,5V di SET_MOTORE (a valle del filtro RC) --> motore fermo pronto a partire
	pwm_mot[ 0 ].pwmFrq   = 8200;		// freq iniziale del pwm in Hz
 	pwm_mot[ 0 ].frqBase  = 20e+6/8.0;	// freq del clock di sistema (legato al valore settato in TP0CTL0) in Hz

	regVal = (pwm_mot[ 0 ].frqBase/((double) pwm_mot[ 0 ].pwmFrq)) - 1;
	TP0CCR0 = regVal; // Frequenza pwm --> fpwm =  fc / (TP0CCR0 + 1)
	regVal = (regVal * pwm_mot[ 0 ].pwmCycle) / 100;
	TP0CCR1 = regVal;		// pwmCycle * (TO0CCR0 + 1) / 100
	asm("di");
	pwm_mot[ 0 ].enable = false;	// disable_PWM(PWM_ElettrValv, pwm_ev);
	asm("ei");

}

void init_PWM_MOT2()
{
}

void init_PWM_MOT3()
{
}

void init_PWM_MOT4()
{
}

void init_PWM_MOT5()
{
}

/**
Init of the PWM2 port
This pwm signal controls enabling of the Electromagnetic valves
*/
void init_PWM_EVuC()
{
	word regVal;

	TP1CTL0 = 0x04;	//-- Counter frequency  --> fc=fxx/32
	TP1CTL1 = 0x04;

	_TP1OE0 = 1;	// enable TOP10
	_TP1OE1 = 0;	// disable TOP11

	PWM_EV = 0;
	_PF34 = 0;
	_PFC34 = 1;
	_PMC34 = 1;
	ioPWM_EV = 0;

	pwm_ev.pwmCycle = 10;
	pwm_ev.pwmFrq   = 8200;
	pwm_ev.frqBase  = 20e+6/16.0;	// fc = 1.25MHz
	EV_timer.Stop();
	regVal = (pwm_ev.frqBase/((double) pwm_ev.pwmFrq)) - 1;		// 151,44
	TP1CCR0 = regVal; // Frequenza pwm --> fpwm =  fc / (TP0CCR0 + 1)
	regVal = (regVal * pwm_ev.pwmCycle)  /100;						// 15.1			
	TP1CCR1 = regVal;		// pwmCycle * (TO0CCR0 + 1) / 100
	///duty cyle = TP1CCR1 / (TP1CCR0 + 1) = 
	asm("di");
	pwm_ev.enable = false;	// disable_PWM(PWM_ElettrValv, pwm_ev);
	asm("ei");
}

/**
set the duty cycle of the pwm, in variable mode
*/
void set_PWM_EVuC(byte set)
{
	word regVal;
	regVal = TP1CCR0;
	regVal = (regVal * (word)set)  /100;						// 15.1			
	TP1CCR1 = regVal;
}

/**
manager for management of motors accelerations/decellerations
States of the state-machine are set in CPUSIFRA manager module
*/
void pwmManager()
{
	byte m_line;
	// stati POMPA DISABILITATA, ABILITAZIONE, CONTROLLO, ARRESTO
	m_line = 0;

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
			else	{
				setStateCoverM3300Pump(True);
				setBlockRelay(m_line, 1);	//chiudo il relay
				enable_MOT(m_line);
				PWM_timer[m_line].Preset(20);
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
			disable_PWM_MOT(m_line);
			setBlockRelay(m_line, 0);		// apro il relay
			stato_pompa[m_line] = POMPA_DISABILITA;
			break;

		case COVER_APERTA:
			if(getCoverPumpState())
			{
				setBlockRelay(m_line, 0);		// apro il relay
				setPwmMotCycle(m_line, _PWM_CYCLE_0VEL);
				//stato_pompa[m_line] = POMPA_DISABILITA;	
				setStateCoverM3300Pump(True);
			}
			else // se la pompa stava andando la facciamo riprendere
			{
				setStateCoverM3300Pump(False);
				if((pre_stato_pompa[m_line] == ABILITAZIONE) || (pre_stato_pompa[m_line] == CONTROLLO))
					stato_pompa[m_line] = ABILITAZIONE;
			}
			break;
			
		case INDIETRO:		// valido solo per M3200 e M3300
			if(getCoverPumpState())
			{
				pre_stato_pompa[0] = stato_pompa[0];
				stato_pompa[0] = COVER_APERTA;
			}
			else	{
				//MotorStatus.On = True;
				setBlockRelay(0, 1);	//chiudo il relay
				enable_MOT(0);				
				DirMOT1 = ~DirMOT1;
				stato_pompa[0]  = CONTROLLO;
			}
			break;
	}

}

/**
manager for management of the electromagnetic valve
States of the state-machine are set in CPUSIFRA manager module
*/
void EVsManager()
{

	switch(state_EVs)
	{
		case DISABLE:
			break;

		case ENABLE:
			if(EV_timer.Match())	// infine, quando aperta, abbasso la tensione
			{
				EV_timer.Stop();
				init_PWM_EVuC();
				enable_PWM_EV();
				changeEVstatus(OPEN);
			}
			break;

		case OPEN:
			break;

		case CLOSE:
			disable_PWM_EV();
			PWM_EV = 0;	// abbasso il pin per togliere la 48V
			ioPWM_EV = 0;		// stoppare il PWM_EV
			_PMC34 = 0;	// P34 come IO
			changeEVstatus(DISABLE);
			break;
		default:
			break;
	}
}

//byte SIFRA_Manager::changeSIFRAstatus(byte newStatus)
byte changeEVstatus(byte newStatus)
{
	if(newStatus != state_EVs)
	{
		previousstate_EVs = state_EVs;
		state_EVs = newStatus;
	}
	return 0;
}

/**
enabling of the PWM2 signal for electromagnetic valve
*/
void enable_PWM_EV()
{
	asm("di");
	pwm_ev.enable = True;
	pwm_ev.counter = 0;
	asm("ei");
	_TP1CE = 1;
}

/**
disabling of the PWM2 signal for electromagnetic valve
*/
void disable_PWM_EV()
{
	asm("di");
	pwm_ev.enable = False;
	asm("ei");
	_TP1CE = 0;
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
		case 2:
		case 3:
		case 4:
		default:
			break;
	}
}

/**
enabling of the PWM1 signal for pump motor
*/
void enable_MOT(int i)
{
	switch( i )
	{
		case 0:
			init_PWM_MOT1();
			break;
		case 1:
		case 2:
		case 3:
		case 4:
		default:
			break;
	}
	enable_PWM_MOT( i );
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
		case 2:
		case 3:
		case 4:
		default:
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

bool getCoverPumpState()
{	// quando lo sportello è aperto il segnale è basso
	if(cpld_CoverPump == 0)	// aperto
	{
		return 1;	// ritorna 0 sportello chiuso
	}
	return 0;
}

void setBlockRelay(int i, int e)
{
	switch( i )
	{
		case 0:
			cpld_pin_BLK1 = e;
			break;
		case 1:
		case 2:
		case 3:
		case 4:
		default:
			break;
	}
}

void setRampPump(int i, byte command)
{
	pwm_mot[ i ].upper = command;
}

int getRampPump(int i)
{
	return pwm_mot[ i ].upper;
}

void setSpeedPump(int i, byte command)
{
	pwm_mot[ i ].pwmCycle = command;
}

int getSpeedPump(int i)
{
	return pwm_mot[ i ].pwmCycle;
}

void setPwmMotCycle(int i, int value)
{
	pwm_mot[ i ].pwmCycle = value;
	setPwmCycle( i , pwm_mot[ i ].pwmCycle);
}

void setPwmCycle( int pwm, byte duty_cycle)
{
int regVal;
	switch( pwm )
	{
		case 0:
			regVal = TP0CCR0;
			regVal = (regVal * duty_cycle) / 100;
			TP0CCR1 = regVal;
			break;
		case 1:
		case 2:
		case 3:
		case 4:
		default:
			break;
	}
}

/**
Sets state of the cover of the pump in M3300 module
*/
void setStateCoverM3300Pump(bool e)
{
	coverState_open = e;
}

/**
Gets state of the cover
*/
bool getStateCoverM3300Pump()
{
	return coverState_open;
}

/**
agisce sul basso livello per attivare l'elletrovalvola indicata
*/
void OpenLineEV(int i)
{
	PWM_EV = 1;	// prima alzo il pin per dare la 48V
	ioPWM_EV = 0;
	EV_timer.Preset(_T_OPEN_EV);
	switch(i)
	{
		case 0:
			if( PinCMD_EV1 == 1)	// poi comando l'EV
				PinCMD_EV1 = 0;
			break;
		case 1:
			if( PinCMD_EV2 == 1)	// poi comando l'EV
				PinCMD_EV2 = 0;
			break;
		case 2:
			if( PinCMD_EV3 == 1)	// poi comando l'EV
				PinCMD_EV3 = 0;
			break;
		case 3:
			if( PinCMD_EV4 == 1)	// poi comando l'EV
				PinCMD_EV4 = 0;
			break;
		case 4:
			if( PinCMD_EV5 == 1)	// poi comando l'EV
				PinCMD_EV5 = 0;
			break;
		case 5:
			if( PinCMD_EV6 == 1)	// poi comando l'EV
				PinCMD_EV6 = 0;
			break;
		case 6:
			if( PinCMD_EV7 == 1)	// poi comando l'EV
				PinCMD_EV7 = 0;
			break;
		case 7:
			if( PinCMD_EV8 == 1)	// poi comando l'EV
				PinCMD_EV8 = 0;
			break;
		default:
			break;
	}
	changeEVstatus(ENABLE);
}

/**
basso livello per la chiusura della elettrovalvola indicata
*/
void CloseLineEV(int i)
{
	switch(i)
	{
		case 0:
			if( PinCMD_EV1 == 0)	// poi disabilito l'EV
				PinCMD_EV1 = 1;
			break;
		case 1:
			if( PinCMD_EV2 == 0)	// poi disabilito l'EV
				PinCMD_EV2 = 1;
			break;
		case 2:
			if( PinCMD_EV3 == 0)	// poi disabilito l'EV
				PinCMD_EV3 = 1;
			break;
		case 3:
			if( PinCMD_EV4 == 0)	// poi disabilito l'EV
				PinCMD_EV4 = 1;
			break;
		case 4:
			if( PinCMD_EV5 == 0)	// poi disabilito l'EV
				PinCMD_EV5 = 1;
			break;
		case 5:
			if( PinCMD_EV6 == 0)	// poi disabilito l'EV
				PinCMD_EV6 = 1;
			break;
		case 6:
			if( PinCMD_EV7 == 0)	// poi disabilito l'EV
				PinCMD_EV7 = 1;
			break;
		case 7:
			if( PinCMD_EV8 == 0)	// poi disabilito l'EV
				PinCMD_EV8 = 1;		
			break;
		default:
			break;
	}

	changeEVstatus(CLOSE);
}

