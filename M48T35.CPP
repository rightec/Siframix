//---------------------------------------------------------------------------//
//                                                                           //
//                           C O M E T A                                     //
//                                                                           //
//---------------------------------------------------------------------------//
//                                                                           //
//  Progetto:                DRVLIB                                          //
//  Nome File:               M48T35.CPP                                      //
//  Descrizione:             Routines gestione rtc ST M48T35                 //
//  Note:             		                                                 //
//							                                                 //
//  Supporto Fisico:         Cpu Siiramix
//  Versione:                $Revision: 1.2 $                                //
//  Data Versione:           $Date: 2002/03/22 00:00:00 $                    //
//  Descrizione Versione:    Stesura modificata per micro JG2                        //
//                                                                           //
//---------------------------------------------------------------------------//

// il driver non include niente (è lui stesso un header)


//-------------------------------------------------------
// Variabili globali utili a tutti i moduli
//-------------------------------------------------------

//-------------------------------------------------------
// Funzioni statiche utili solo a questo modulo
//-------------------------------------------------------


					///////////////////////////////////////////////////////////////
					// Funzioni di interesse escusivo per il driver RTC
					///////////////////////////////////////////////////////////////

#ifdef  __RTCDRV__

#ifdef __DEBUG__
	#warning __RTCDRV__M48T35__ active ....
#endif


static void drv_writeSingleReg(word __reg, byte __data);
static byte bcdToBin(byte __bcdvalue);
static byte binToBcd(byte __binvalue);

//-------------------------------------------------------
// Costanti statiche utili solo a questo modulo
//-------------------------------------------------------


//-------------------------------------------------------
// Variabili statiche utili solo a questo modulo
//-------------------------------------------------------

static ClockM48T35 rtcClock;			// ad uso interno...

//-------------------------------------------------------
// Corpo funzioni di questo modulo
//------------------------------------------------


/* Esegue l'inizializzazione dell'RTC per quanto sia necessario fare..
*/
void drv_setupClock(void)
{
	RTC_WR_UNLOCK();					// sblocca la scrittura in ram

    drv_writeSingleReg(RTC_REG_ALARM_ORA, 0x00);		// setup configurazione
    
	RTC_WR_LOCK();						// blocca la porta di ingresso (ma non quella di uscita..)
}

/* Legge tutti i registri dell'RTC e li riporta tali e quali
*/
void drv_readRegisters(ClockM48T35 *__rtcRegisters)
{
register byte *pc;
register byte *pr;
register word i;

	RTC_WR_LOCK();						// inizio trasferimento

	pr = RTC_CS + RTC_REG_CENT;					// Primo indirizzo: centesimi di secondo in lettura

	pc = (byte *) __rtcRegisters;		// parto dal primo elemento...

										// copia i registri dell'rtc	
	for(i=0; i<sizeof(rtcClock); i++)
		*pc++ = *pr++;
		
	RTC_WR_LOCK();						// fine trasferimento
}

/* Legge tutti i registri dell'RTC e li converte in un formato standardizzato
*/
void drv_readClock(SystemTime *__fulltime)
{
register byte *pc;
register byte *pr;
register word i;

	RTC_WR_LOCK();						// inizio trasferimento

	pr = RTC_CS + RTC_REG_CENT;			// Primo indirizzo: centesimi di secondo in lettura

	pc = (byte *) &rtcClock;			// parto dal primo elemento...

										// copia i registri dell'rtc	
	for(i=0; i<sizeof(rtcClock); i++)
		*pc++ = *pr++;
		
	RTC_WR_LOCK();						// fine trasferimento

										// converte nell'unita' di misura internazionale..
	__fulltime->year = bcdToBin(rtcClock.anno) + 2000;
	__fulltime->day = bcdToBin(rtcClock.giorno);
	__fulltime->month = bcdToBin(rtcClock.mese);

	__fulltime->hour = bcdToBin(rtcClock.ora & 0x3F);
	__fulltime->min = bcdToBin(rtcClock.min);
	__fulltime->sec = bcdToBin(rtcClock.sec & 0x7F);
	__fulltime->hund = bcdToBin(rtcClock.cent);

	__fulltime->dayofweek = bcdToBin(rtcClock.giorno_sett & 0x07);
}


/* Scrive tutti i registri dell'RTC che sono stati modificati dall'ultima lettura
   Occhio: eseguire sempre una lettura (readClock) prima di chiamare questa funzione !!!!
   In questo funzione + che l'efficienza viene privilegiata la sicurezza dei dati
   riscrivendo ogni dato modificato uno alla volta (no selezione unica del primo registro
   e poi carrellata di scritture)
*/
void drv_writeClock(SystemTime *__fulltime)
{

	if( __fulltime->year != bcdToBin(rtcClock.anno) + 2000 )			// se hanno cambiato l'anno..a partire dal 2000
	{
		rtcClock.anno = binToBcd(__fulltime->year - 2000);				// aggiorna l'ultimo dato scambiato
		drv_writeSingleReg(RTC_REG_ANNO, rtcClock.anno);					// aggiorna anche nei registri RTC
	}
	
	
	if( __fulltime->day != bcdToBin(rtcClock.giorno) )
	{
		rtcClock.giorno = binToBcd(__fulltime->day);
		drv_writeSingleReg(RTC_REG_GIORNO, rtcClock.giorno);				// aggiorna anche nei registri RTC
	}
		
	if( __fulltime->month != bcdToBin(rtcClock.mese) )
	{
		rtcClock.mese = binToBcd(__fulltime->month);
		drv_writeSingleReg(RTC_REG_MESE, rtcClock.mese);					// aggiorna anche nei registri RTC
	}

	if( __fulltime->hour != bcdToBin(rtcClock.ora & 0x3F) )
	{
		rtcClock.ora &= ~0x3F;											// ripulisce...
		rtcClock.ora |= binToBcd(__fulltime->hour) & 0x3F;				// aggiunge il richiesto
		drv_writeSingleReg(RTC_REG_ORA, rtcClock.ora);					// aggiorna anche nei registri RTC
	}

	if( __fulltime->min != bcdToBin(rtcClock.min) )
	{
		rtcClock.min = binToBcd(__fulltime->min);
		drv_writeSingleReg(RTC_REG_MIN, rtcClock.min);					// aggiorna anche nei registri RTC
	}

	if( __fulltime->sec != bcdToBin(rtcClock.sec & 0x7F) )
	{
		rtcClock.sec &= ~0xFF;											// ripulisce...
		rtcClock.sec |= binToBcd(__fulltime->sec) & 0x7F;				// aggiunge il richiesto
		drv_writeSingleReg(RTC_REG_SEC, rtcClock.sec);					// aggiorna anche nei registri RTC
	}

	if( __fulltime->hund != bcdToBin(rtcClock.cent) )
	{
		rtcClock.cent = binToBcd(__fulltime->hund);
		drv_writeSingleReg(RTC_REG_CENT, rtcClock.cent);					// aggiorna anche nei registri RTC
	}

	if( __fulltime->dayofweek != bcdToBin(rtcClock.giorno_sett & 0x07) )
	{
		rtcClock.giorno_sett &= ~0x07;										// ripulisce...
		rtcClock.giorno_sett |= binToBcd(__fulltime->dayofweek) & 0x07;		// aggiunge il richiesto
		drv_writeSingleReg(RTC_REG_GIORNO_SETT, rtcClock.giorno_sett);		// aggiorna anche nei registri RTC
	}

}


/* Scrive nella memoria dell'RTC un singolo registro
*/
static void drv_writeSingleReg(word __reg, byte __data)
{
  
	RTC_WR_UNLOCK();											// inizio trasferimento

	*(RTC_CS + RTC_REG_CONTROL) |= RTC_WRBIT_MASK;				// invia il dato al registro richiesto
	*(RTC_CS + RTC_REG_CONTROL) &= ~RTC_STBIT_MASK;				// invia il dato al registro richiesto

	*(RTC_CS + __reg) = __data;									// invia il dato al registro richiesto
    
	*(RTC_CS + RTC_REG_CONTROL) &= ~RTC_WRBIT_MASK;;			// invia il dato al registro richiesto

 	RTC_WR_LOCK();												// fine trasferimento
}


static byte bcdToBin(byte __bcdvalue)
{
byte __binvalue;

	__binvalue = __bcdvalue & 0x0F;			// la parte bassa resta tale e quale
	__binvalue += (__bcdvalue >> 4) * 10;	// la parte alta sono di sicuro delle decine

	return( __binvalue );
}


static byte binToBcd(byte __binvalue)
{
byte __bcdvalue;

	if( __binvalue > 99 )					// con due digit non possiamo rappresentare 
		__binvalue = 99;					// + della cifra 99!!
		
	__bcdvalue = __binvalue % 10;			// la parte bassa viene convertita in unità
	__bcdvalue += (__binvalue / 10) << 4;	// la parte alta viene convertita in decine

	return( __bcdvalue );
}
 
#endif  			// __RTCDRV__


					///////////////////////////////////////////////////////////////
					// Funzioni di interesse escusivo per il driver NVR
					///////////////////////////////////////////////////////////////

#ifdef  __NVRDRV__

#ifdef __DEBUG__
	#warning __NVRDRV__M48T35__ active ....
#endif

/* Esegue l'inizializzazione dell'NVR per quanto sia necessario fare..
*/
/*
void drv_setupMem(void)
{
	RTC_WR_LOCK();										// blocca la scrittura accidentale
}
*/
/* Legge dalla memoria dell'rtc nell'area "user" 
*/
bool drv_readMem(void *__data_buf, int __count, int __start_addr)
{
register byte *pc;
register byte *pr;
register int i;

	if( __start_addr > NVR_LAST_ADDR || __start_addr + 2*__count > NVR_LAST_ADDR )
		return(False);

	pc = (byte *) __data_buf;							// piglia il testimone...
	
	pr = CS_TAMPRAM + __start_addr + NVR_FIRST_ADDR;		// Primo indirizzo utente

	for(i=__count; i; i--)	{							// a candela!!		
		*pc++ = *pr;		//*pc++ = *pr++;// si può leggere anche a indirizzi dispari, ma le scritture sono eseguite solo sui pari
		pr+=2;
	}
	
	return(True);										// trasferimento ok !!
}


/* Scrive nella memoria dell'rtc nell'area "user" 
*/
bool drv_writeMem(void *__data_buf, int __count, int __start_addr)
{
register byte *pc;
register byte *pr;
register int i;

	if( __start_addr > NVR_LAST_ADDR || __start_addr + 2*__count > NVR_LAST_ADDR )
		return(False);

	pc = (byte *) __data_buf;							// piglia il testimone...

	pr = CS_TAMPRAM + __start_addr + NVR_FIRST_ADDR;		// Primo indirizzo utente

	for(i=__count; i; i--)								// a candela!!
	{
		*pr = *pc++;	// scrittura solo a indirizzi pari	//*pr++ = *pc++;
		pr+=2;
	}
	
	return(True);										// trasferimento ok !!
}

#endif


//@@-----------------------------------------------------------------------@@//
//@@
//@@  $Source:  $
//@@  Note di revisione:
//@@
//@@  $Revision: 1.2 $
//@@  $Date: 2002/03/14 00:00:00 $
//@@ 	- Viene aggiunta 
//@@	
//@@
//@@  $Log:  $
//@@
//@@  $Author: junior $
//@@  $RCSfile: M48T35.CPP,v $
//@@  $Revision: 1.2 $
//@@  $State: Exp $
//@@  $Date: 2002/12/16 00:00:00 $
//@@  $Name:  $
//@@-----------------------------------------------------------------------@@//
//@@-----------------------------------------------------------------------@@//
//@@                                                                       @@//
//@@                            C O M E T A                                @@//
//@@                                                                       @@//
//@@-----------------------------------------------------------------------@@//

