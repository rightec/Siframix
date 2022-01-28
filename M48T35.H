//---------------------------------------------------------------------------//
//                                                                           //
//                           C O M E T A                                     //
//                                                                           //
//---------------------------------------------------------------------------//
//  
//  Progetto:                GENLIB
//  Nome File:               M48T35.H
//  Descrizione:             Definizioni rtc ST M48T35
//  Versione:                $Revision: 1.1 $
//  Data Versione:           $Date: 2003/01/23 00:00:00 $
//  
//---------------------------------------------------------------------------//

//#include "global.h"
//--------------------------------------------------------//
// Definizione delle costanti
//--------------------------------------------------------//
								// alias pin chip select
//#define	RTC_WR_LOCK()		(CS_PLD_WRR = 0)
//#define	RTC_WR_UNLOCK()		(CS_PLD_WRR = 85)
// queste due forse dovrei provare a rifarle per pld, cioè a generare un comando alla pld che a sua volta protegge(sprotegge il bit, 
// ma proprio per come lavora la pld credo di potermene fregare
								// alias porta di comunicazione
#define	RTC_CS				CS_TAMPRAM



#ifdef  __RTCDRV__		// registri ad uso e consumo del driver RTC

								// Indirizzi dei registri interni dell'orologio/datario dell'rtc
#define	RTC_REG_CENT			0x7FF7		// registro non esistente ...
#define	RTC_REG_CONTROL			0x7FF8
#define	RTC_REG_SEC				0x7FF9
#define	RTC_REG_MIN				0x7FFA
#define	RTC_REG_ORA				0x7FFB
#define	RTC_REG_GIORNO_SETT		0x7FFC
#define	RTC_REG_GIORNO			0x7FFD
#define	RTC_REG_MESE			0x7FFE
#define	RTC_REG_ANNO			0x7FFF

#define	RTC_REG_WATCHDOG		0x7FF6		// registro non esistente ...
#define	RTC_REG_ALARM_MESE		0x7FF5		// registro non esistente ...
#define	RTC_REG_ALARM_GIORNO	0x7FF4		// registro non esistente ...
#define	RTC_REG_ALARM_ORA		0x7FF3		// registro non esistente ...
#define	RTC_REG_ALARM_MIN		0x7FF2		// registro non esistente ...
#define	RTC_REG_ALARM_SEC		0x7FF1		// registro non esistente ...
#define	RTC_REG_FLAGS			0x7FF0		// registro non esistente ...
#define	RTC_REG_SQWAVE			0x7FF0		// registro non esistente ...


#endif


#ifdef  __NVRDRV__		// registri ad uso e consumo del driver NVR

#define	NVR_FIRST_ADDR			0x0000		// primo indirizzo area nvr (non volatile ram user)
#define	NVR_LAST_ADDR			0x7FEF		// ultimo indirizzo area nvr (non volatile ram user)

#endif


#define	RTC_WRBIT_MASK			0x80		// mascherina per bit di write nel primo byte di comando
#define	RTC_STBIT_MASK			0x20		// mascherina per bit di write nel primo byte di comando

//--------------------------------------------------------//
// Definizione dei tipi
//--------------------------------------------------------//

#ifdef  __RTCDRV__
								// Immagine dei registri dell'RTC M43T85
struct ClockM48T35 {
	byte        cent;
    byte        control;
	byte        sec;
	byte        min;
	byte        ora;
    byte        giorno_sett;
    byte        giorno;
    byte        mese;
    byte        anno;
};
 
typedef struct ClockM48T35 ClockM48T35;

#endif

//--------------------------------------------------------//
// Definizione delle classi 
//--------------------------------------------------------//

//--------------------------------------------------------//
// Definizione delle funzioni
//--------------------------------------------------------//

#ifdef __cplusplus
extern "C" {
#endif

#ifdef  __RTCDRV__
	#warning __RTCDRV__ active ....
static void drv_setupClock(void);
static void drv_readClock(SystemTime *__fulltime)	;				// fa leggere in blocco..
static void drv_writeClock(SystemTime *__fulltime);				// fa scrivere in blocco..
static void drv_readRegisters(ClockM48T35 *__rtcRegisters);
#endif

#ifdef  __NVRDRV__
//static void drv_setupMem(void);														// inizializza il driver low-lev per nvr
bool drv_readMem(void *__data_buf, int __count, int __start_ddr);			// legge dalla memoria dell'nvr
bool drv_writeMem(void *__data_buf, int __count, int __start_addr);			// scrive nella memoria dell'nvr
#endif


#ifdef __cplusplus
}
#endif


//--------------------------------------------------------//
// Definizione delle variabili
//--------------------------------------------------------//




//@@-----------------------------------------------------------------------@@//
//@@                                                                             
//@@  $Source:  $
//@@  Note di revisione:                
//@@
//@@  $Revision: 1.2 $
//@@  $Date: 2002/03/14 00:00:00 $
//@@ 	- Viene definita la costante "RTC_WRBIT_MASK" per le operazioni di
//@@	scrittura della memoria. 
//@@
//@@
//@@  $Log:  $
//@@
//@@  $Author: junior $
//@@  $RCSfile: M48T35.H,v $
//@@  $Revision: 1.1 $
//@@  $State: Exp $
//@@  $Date: 2001/09/07 06:49:06 $
//@@  $Name:  $
//@@-----------------------------------------------------------------------@@//
//@@-----------------------------------------------------------------------@@//
//@@                                                                       @@//
//@@                            C O M E T A                                @@//
//@@                                                                       @@//
//@@-----------------------------------------------------------------------@@//

