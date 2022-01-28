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


#ifndef __PWM_H__
#define __PWM_H__
#include "global.h"
#include "swtim.h"


#define  _PWM_CYCLE_MAX	65		// deve corrispondere ad un set di 5V
#define _PWM_CYCLE_HALF	30
#define _PWM_CYCLE_LOW	10
#define _PWM_CYCLE_LOW_2	11
#define	_PWM_CYCLE_MIN	6		// dovrebbe corrospondere a 0,6V il motore comincia a muoversi a bassa velocità ma continuativamente senza scatti
#define _PWM_CYCLE_SPEED_ST 	_PWM_CYCLE_MAX
#define _PWM_CYCLE_0VEL	2
//#define _PWM_CYCLE_HIGH_SPEED 60
#define _PWM_CYCLE_MANUAL_SPEED	20
#define _PWM_CYCLE_CLEANING_SPEED	50
#define _PWM_CYCLE_LOW_MANUAL_SPEED 	12
#define _T_OPEN_EV 500
#define _T_OPEN_EV_DELAY 3000

typedef struct
{
	unsigned long period;
	unsigned long Ton;
	unsigned long Toff;
	unsigned long counter;
	bool enable;
}pwm;

typedef struct{
dword			counter;
byte					upper; // 1 aumenta velocità, 0 diminuisci
word				pwmFrq;
int					pwmCycle;
double 				frqBase;	// espresso in hertz
bool				enable;
}my_pwm;

/**
Generic Encoder data struct
*/
struct ENCODER_COUNTER_STATUS
{
	long step_motor_expected;		// valore calcolato che indica il numero dei passi attesi, dato un certo peso da erogare
	long step_motor_done;			// valore misurato dei passi pompa eseguiti con flag ON attivo
	word previus_step_motor;		// valore precedente misurato
};
typedef struct ENCODER_COUNTER_STATUS ENCODER_Motor_Status;

enum{	
	FERMA = 0,
	VELOCITA_MINIMA,
	ACCELLERA,
	PLATEAUX,
	DECELLERA,
	AVVICINAMENTO,
	MANTIENI
};

typedef enum 
{
	PWM_BUZZER = 0,
	PWM_MOTORE1,
	PWM_MOTORE2,
	PWM_ElettrValv,
	PWM_MOTORE3,
} PWM_TYPE;

/*typedef enum
{
	POMPA_DISABILITA = 0,
	ABILITAZIONE,
	CONTROLLO,
	ARRESTO,
	COVER_APERTA,
} STATO_POMPA;*/
enum
{
	POMPA_DISABILITA = 0,
	ABILITAZIONE,
	CONTROLLO,
	ARRESTO,
	COVER_APERTA,
	INDIETRO
};

enum{
	DISABLE = 0x00,
	ENABLE = 0x01,
	OPEN = 0x02,
	CLOSE = 0x03
};
/*
typedef enum{
	DISABLE = 0,
	ENABLE,
	OPEN,
	CLOSE
} STATE_EVS;
*/
void init_PWM_MOT1();
void init_PWM_MOT2();
void init_PWM_MOT3();
void init_PWM_MOT4();
void init_PWM_MOT5();
void init_PWM_EVuC();
void set_PWM_EVuC(byte set);
void pwmManager();
void EVsManager();
byte changeEVstatus(byte newStatus);
void enable_PWM_EV();
void disable_PWM_EV();
void disable_PWM_MOT(int line);
void reg_pump_vel(byte line, byte Command);
void setBlockRelay(int i, int e);
void setPwmCycle( int pwm, byte duty_cycle);
void enable_MOT(int i);
void setRampPump(int i, byte command);
int getRampPump(int i);
void setSpeedPump(int i, byte command);
int getSpeedPump(int i);
//void setPwmMotCycleIncrease(int i, int increment);
//void setPwmMotCycleDecrease(int i, int decrement);
void setPwmMotCycle(int i, int value);
bool getCoverPumpState();
void setStateCoverM3300Pump(bool e);
bool getStateCoverM3300Pump();
void OpenLineEV(int i);
void CloseLineEV(int i);

#endif






