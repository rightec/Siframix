/**
@file		Iodef.h
@brief		Board's pins definitions
					
@author		
@date		18/01/2011
@version	01.00
*/

#ifndef _IODEF_H_
#define _IODEF_H_
//--------------------------------------------------------//
// Definizione delle costanti
// Nota:
// 		InXXXX		-> bit su maschera ingressi
//		OutXXXX		-> bit su maschera uscite
//		PinXXXX		-> bit su periferica pin ingresso/uscita
//--------------------------------------------------------//

/* Selezione dispositivi di in-out e gestione moemoria
*/

#ifdef __SIMUL__
#define BASEREG	0xFFFFF000					// nuovi indirizzi per simulatore in fondo alla mem.
#else
#define BASEREG 0xFFFFF000					// indirizzi reali registri macchina
#endif

/**
Hardware revision 0
*/
#define __HW_REV_0__	0
/**
Current Hardware revision
*/
#define __HARDWARE_REV__ __HW_REV_0__


/**
Crystal frequency
*/
#define CRYSTAL_FREQ	20000000.0				/
/**
Sub crystal frequancy
*/
#define CRYST_SUBFRQ	32768.0			

								// costanti di base dei vari moduli
#define BASEMOD_SIOSG2	100		// prima i moduli HW
#define BASEMOD_DMASG2	200		// modulo dma
#define BASEMOD_IRQSG2	300		// modulo irq & vectors
#define BASEMOD_WTNMSG2	400		// Watch Timer
#define BASEMOD_WDTMSG2	500		// Watch Dog Timer
#define BASEMOD_TIMMSG2	600		// modulo timers
#define BASEMOD_SERIAL_ADC	700		// modulo ad conveters

#define BASEMOD_COMBUF	1100	// poi i moduli SW
#define BASEMOD_CSI		1200
#define BASEMOD_I2C		1300
#define BASEMOD_UART	1400
#define BASEMOD_ALLOC	1500
#define BASEMOD_MMCARD	1600
#define BASEMOD_FILSYS	1700
#define BASEMOD_CHRSET	1800
#define BASEMOD_RTCDRV	1900
#define BASEMOD_NVRDRV	2000
#define BASEMOD_DATAFLASH	2100

/**
Definition for the dynamic memory allocation
*/
#define ALLOC_ALIGNMENT		4
#define	RTC_DRV 		DRV_M48T35			// definizione rtc da utilizzare
#define	NVR_DRV 		DRV_M48T35			// definizione nvr da utilizzare
#define EEP_DRV            DRV_FM24C64    		// ram ferromagnetica
						
/**
All 8 pins set as input
*/
#define DDIR_INPUT		0xFF	
/**
All 8 pins set as output
*/
#define DDIR_OUTPUT		0x00			

/**
Pin set as output 
*/
#define PDIR_OUTPUT		0
/**
Pin set as input 
*/
#define PDIR_INPUT		1

//-------------------------------------------------------
//-------------- Hardware Revision 0 --------------------
//-------------------------------------------------------
/**
input signals from the CPLD
*/
#define cpld_reg_in 	 *((volatile word *)0x0400000)
#define cpld_pin_Air1 ((volatile struct bitf *)&cpld_reg_in)->bit00
#define cpld_pin_Air2 ((volatile struct bitf *)&cpld_reg_in)->bit01
#define cpld_pin_Air3 ((volatile struct bitf *)&cpld_reg_in)->bit02
#define cpld_pin_Air4 ((volatile struct bitf *)&cpld_reg_in)->bit03
#define cpld_pin_Air5 ((volatile struct bitf *)&cpld_reg_in)->bit04
#define cpld_pin_Air6 ((volatile struct bitf *)&cpld_reg_in)->bit05
#define cpld_pin_Air7 ((volatile struct bitf *)&cpld_reg_in)->bit06
#define cpld_pin_Air8 ((volatile struct bitf *)&cpld_reg_in)->bit07
#define cpld_FW_Vers0 ((volatile struct bitf *)&cpld_reg_in)->bit08
#define cpld_FW_Vers1 ((volatile struct bitf *)&cpld_reg_in)->bit09
#define cpld_CoverPump ((volatile struct bitf *)&cpld_reg_in)->bit11

/**
Output signal of the CPLD
*/
#define cpld_reg_out   *((volatile word *)0x0400010)
#define cpld_pin_led    ((volatile struct bitf *)&cpld_reg_out)->bit05
#define cpld_pin_BLK1 ((volatile struct bitf *)&cpld_reg_out)->bit00

/**
Counters of the CPLD for encoders cycles
*/
#define cpld_counter1 *(( volatile word *)0x0400100)

/**
indirizzi W/R su memoria sram tamponata, ora inclusa nel real time clock, 32Kb
*/
#define CS_TAMPRAM				(byte *)0x0180000
#define 	restart_command		*((volatile byte *)0x0180014)

/**
Addres for JUMP_TO_LOAD flag (in external non-volatile RAM o FRAM)
*/
#define	stay_in_loader_flag0  *(( volatile byte *)0x0180010)
#define	stay_in_loader_flag1  *(( volatile byte *)0x0180012)

/**
Addres for set the START STATE, in case of lack of currente
*/
#define start_on_course	*((volatile byte *)0x0180014)
#define reset_4_block	*((volatile byte *)0x018001A)


#define OFFSET_VAL_ENC	0x20	// posizione in cui si scrivono i valori del rapporto encoder
#define OFS_POINTS_SET 0x100	// posizione struttura backup in memoria


#if (__HARDWARE_REV__ == __HW_REV_0__)
//-- Buzzer --
#define PinBUZ			_P32	// Buzzer
#define ioPinBUZ			_PM32

//-- Led --
#define PinLED0_LIFE_M	_P10	// Led verde 
#define ioPinLED0_LIFE_M _PM10

//-- pin x debug --
#define FDLM0			_PDH5
#define ioFDLM0			_PMDH5

#define FDLM1			_PDL5
#define ioFDLM1			_PMDL5

#define DRST				_P05
#define ioDRST			_PM05

#define Pin_RX0			_P31
#define Pin_TX0			_P30

//-- Pin Analogici --
#define	PinAN_IN2		_P72		// conducibilità soluzione (M3800)
#define PinAN_IN0		_P70		// monitor tensioni scheda
#define PinAN_IN1		_P71		// temperatura soluzione (M3800)

//-- Bus e comunicazione CPLD --
#define	PinAD0 			_PDL0
#define	PinAD1 			_PDL1
#define	PinAD2 			_PDL2
#define	PinAD3 			_PDL3
#define	PinAD4 			_PDL4
#define	PinAD5 			_PDL5
#define	PinAD6 			_PDL6
#define	PinAD7 			_PDL7
#define	PinAD8 			_PDL8
#define	PinAD9 			_PDL9
#define	PinAD10 		_PDL10
#define	PinAD11			_PDL11
#define	PinAD12			_PDL12
#define	PinAD13  		_PDL13
#define	PinAD14			_PDL14
#define	PinA1D5	 		_PDL15

#define	PinA16			_PDH0
#define	PinA17			_PDH1
#define	PinA18			_PDH2
#define	PinA19			_PDH3
#define	PinA20			_PDH4

#define	PinWAIT			_PCM0
#define	PinCLKOUT		_PCM1
#define PinASTB			_PCT6
#define PinRD			_PCT4
#define PinWR1			_PCT1
#define PinWR0			_PCT0

#define PinCSBUS0		_P915	//	_P99
#define ioPinCSBUS0		_PM915	//	_PM99

#define PinCPLD_IO0		_P914
#define ioPinCPLD_IO0	_PM914

#define PinCPLD_IO1		_P04
#define ioPinCPLD_IO1	_PM04

//-- Definizioni richieste per eeprom, Real Time Clock e flash memory  --

#define PinI2C_SDA		_P38
#define ioPinI2C_SDA    	_PM38
#define PinI2C_SCL  		_P39
#define ioPinI2C_SCL    	_PM39

#define PinWReeprom		_PCM3
#define ioPinWReeprom	_PMCM3

// -- pin per ADC serialei	
#define PinCSadc1			_P36
#define ioPinCSadc1			_PM36

#define PinCSadc2			_P93
#define ioPinCSadc2			_PM93

#define PinCSadc3			_P37
#define ioPinCSadc3			_PM37

#define PinCSadc4			_P95
#define ioPinCSadc4			_PM95

#define PinCSadc5			_P50
#define ioPinCSadc5			_PM50

#define PinEnRS485_DE		_P51
#define ioPinEnRS485_DE   	_PM51

#define PinSPI1ready			_P97
#define PinSPI2ready			_P53
//-- Pin di controllo --
// -- Pin comando Elettrovalvole scheda M3300
//#define PinEnableElectroValve		_P35
//#define ioPinEnableElectroValve     _PM35
#define PinCMD_EV1	_P74		// usato in M3200 e M3300
#define ioPinCMD_EV1	_PM74
#define PinCMD_EV2	_P75		// usato in M3200 e M3300
#define ioPinCMD_EV2 _PM75
#define PinCMD_EV3	_P76		// usato in M3200 e M3300
#define ioPinCMD_EV3	_PM76
#define PinCMD_EV4	_P77		// usato in M3200 e M3300
#define ioPinCMD_EV4 _PM77
#define PinCMD_EV5	_P78		// usato in M3200 e M3300
#define ioPinCMD_EV5	_PM78
#define PinCMD_EV6	_P79		// usato in M3200 e M3300
#define ioPinCMD_EV6 _PM79
#define PinCMD_EV7	_P710		// usato in M3300
#define ioPinCMD_EV7	_PM710
#define PinCMD_EV8	_P711		// usato in M3300
#define ioPinCMD_EV8 _PM711

#define PWM_EV		_P34		// usato in M3200 e M3300
#define ioPWM_EV	_PM34

// --- pin Gestione Motori pompe ---- //
#define DirMOT1		_PCM2		// usato in M3300
#define ioDirMOT1	_PMCM2
#define PWM_MOT1	_P33		// usato in M3100, M3200, M3300
#define ioPWM_MOT1 _PM33
#define PWM_MOT2	_P35		// usato in M3100
#define ioPWM_MOT2 _PM35
#define PWM_MOT3	_P96		// usato in M3100
#define ioPWM_MOT3 _PM96
#define PWM_MOT4	_P94		// usato in M3100
#define ioPWM_MOT4 _PM94
#define PWM_MOT5	_P92		// usato in M3100
#define ioPWM_MOT5 _PM92

// comandi da e per altri moduli	//
#define PinzaEXT		_P11
#define ioPinzaEXT	_PM11

#define CMD_BLOCK	_P03
#define ioCMD_BLOCK	_PM03

#define CMD_EXT		_P06
#define ioCMD_EXT	_PM06

#define CMD_STOP	_P02
#define ioCMD_STOP	_PM02

#define PinCS_LDD	_P913
#define ioPnCS_LDD	_PM913

#endif
//-------------------------------------------------------

//-------------------------------------------------------
//-------------- Hardware Revision 1 --------------------
//-------------------------------------------------------

#if (__HARDWARE_REV__ == __HW_REV_1__)

#endif
//-------------------------------------------------------

//-- Definizioni per buffer IO mappati nella PLD --
#define SLEEP_CPLD   0
#define WAKE_UP_CPLD  1

#define ENABLE_ELECTRO_VALVE  	1
#define DISABLE_ELECTRO_VALVE	0

//--------------------------------------------------------//
// Definizione dei tipi
//--------------------------------------------------------//

//--------------------------------------------------------//
// Definizione delle classi
//--------------------------------------------------------//

//--------------------------------------------------------//
// Definizione delle funzioni
//--------------------------------------------------------//

//--------------------------------------------------------//
// Definizione delle variabili
//--------------------------------------------------------//

//--------------------------------------------------------//
// Inclusione altri files .....
//--------------------------------------------------------//

#endif


