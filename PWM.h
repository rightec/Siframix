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


#define _PWM_CYCLE_MAX	80	// vel max oltre alla quale le linee perdono in "rendimento" e non c'è più un rapporto lineare velocità/portata
#define _PWM_CYCLE_CORRECTION		1
#define _PWM_CYCLE_MANUAL_SPEED	30 	// 20
#define _PWM_CYCLE_LOW_MANUAL_SPEED 8
#define _PWM_CYCLE_SPEED_ST	60
#define _PWM_CYCLE_MIDDLE_LOW 50 		//25
#define _PWM_CYCLE_LOW_SPEED_RT 10	// deve vincere anche la resistenza dle tubo nuovo quindi duro, vedere se basta
#define _PWM_CYCLE_SPEED_RT	65		//40
#define _PWM_CYCLE_LOW	7
#define _PWM_CYCLE_END_LINE 5
#define _PWM_CYCLE_MIN		3			// dovrebbe corrosponder a 0,6V il motore comincia a muoversi
#define _PWM_CYCLE_0VEL	0
#define _COVER_TIMER_		500

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
int				pwmCycle;
float 				frqBase;	// espresso in hertz
bool				enable;
}my_pwm;

/**
Generic Encoder data struct
*/
struct ENCODER_COUNTER_STATUS
{
	//bool On;						// identifica la fase: motore attivo, motore inattivo
	//bool LastReading;
	//bool count_error;				// segnala una differenza fra conteggio effettivo e conteggio fatto con flag ON attivo --> verifica del mossa a mano	
	dword step_motor_expected;		// valore calcolato che indica il numero dei passi attesi, dato un certo peso da erogare
	dword step_motor_done;			// valore misurato dei passi pompa eseguiti con flag ON attivo
	dword previus_step_motor;	// valore precedente misurato
	long partial_step;			// valore usato per memorizzare i movimenti parziali dei motori e calcolare il flusso derivante
	//dword step_motor_really_done;	// valore misurato dei passi realmente fatti, comprensivo quindi della eventuale mossa a mano
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
void pwm_motors_reset();
void init_PWM_MOT1();
void init_PWM_MOT2();
void init_PWM_MOT3();
void init_PWM_MOT4();
void init_PWM_MOT5();
void init_PWM_EVuC();
void pwmManager();
void EVsManager();
byte changeEVstatus(byte newStatus);
void enable_PWM_EV();
void disable_PWM_EV();
void disable_PWM_MOT(int line);
void reg_pump_vel(byte line, byte Command);
void setBlockRelay(int i, int e);
void setPwmCycle( int pwm, int duty_cycle);
void enable_MOT(int i);
void setRampPump(int i, byte command);
int getRampPump(int i);
void setSpeedPump(int i, int command);
int getSpeedPump(int i);
void setStatePump(int i, int command);
int getStatePump(int i);
void setPwmMotCycleIncrease(int i, int increment);
void setPwmMotCycleDecrease(int i, int decrement);
void setPwmMotCycle(int i, int value);
bool getCoverPumpState();
//void setStateCoverM3300Pump(bool e);
//bool getStateCoverM3300Pump();

#endif






