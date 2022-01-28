//---------------------------------------------------------------------------//
//                                                                           //
//                           C O M E T A                                     //
//                                                                           //
//---------------------------------------------------------------------------//
//
//  Progetto:                GENLIB
//  Nome File:               NVRDRV.CPP
//  Descrizione:             Modulo "conversazione" con ram non volatile 
//  						 generica (tipicamente NVR della ST)
//  Supporto Fisico:         None
//  Versione:                $Revision: 1.3 $
//  Data Versione:           $Date: 2004/02/02 00:00:00 $
//  Descrizione Versione:    Revisione con traccia errori 
//
//---------------------------------------------------------------------------//

#include "nvrdrv.h"

//-------------------------------------------------------
// Costanti locali utili solo a questo modulo..
//-------------------------------------------------------
#define	NVR_FIRST_USERADDR			(NVR_FIRST_ADDR + 0x0400)		// primo indirizzo (user)
#define	NVR_LAST_USERADDR			(NVR_LAST_ADDR - 0x400)			// ultimo indirizzo (user)

#define	NVR_FIRST_PAR1ADDR			(NVR_FIRST_ADDR + 0x0000)		// primo indirizzo (parameter block1)
#define	NVR_LAST_PAR1ADDR			(NVR_FIRST_ADDR + 0x03FF)		// ultimo indirizzo (parameter block1)

#define	NVR_FIRST_PAR2ADDR			(NVR_LAST_ADDR - 0x3FF)			// primo indirizzo (parameter block2)
#define	NVR_LAST_PAR2ADDR			(NVR_LAST_ADDR - 0x000)			// ultimo indirizzo (parameter block2)

//-------------------------------------------------------
// Variabili globali utili a tutti i moduli
//-------------------------------------------------------
word m_chk;
//-------------------------------------------------------
// Funzioni statiche utili solo a questo modulo
//-------------------------------------------------------

//-------------------------------------------------------
// Costanti statiche utili solo a questo modulo
//-------------------------------------------------------

//-------------------------------------------------------
// Variabili statiche utili solo a questo modulo
//-------------------------------------------------------
static byte deviceAddress;
static word readAddress;
static word writeAddress;
static NVRErrors nvrError;

//-------------------------------------------------------
// Corpo funzioni di questo modulo
//-------------------------------------------------------

void nvr_init(byte __deviceAddr)
{
	deviceAddress = __deviceAddr;				// salva l'indirizzo richiesto.
	readAddress = 0;
	writeAddress = 0;
	nvrError = NVR_ERR_NONE;
	//drv_setupMem();	// ci pensa la PLD ad evitare che il WR si attivi a sproposito
}

/* Ritorna l'errore corrente come "int" number generico
   e azzera l'errore attivo
*/
int nvr_errors(void)
{
int error = nvrError;

	nvrError = NVR_ERR_NONE;					// reset errore

	return( error );
}


void nvr_setDeviceAddress(byte __deviceAddr)
{
	deviceAddress = __deviceAddr;				// salva l'indirizzo richiesto.

	if( deviceAddress > 7 )
	{
		nvrError = NVR_ERR_DEV_ADDR;

#ifdef __DEBUG__
		error_trace("NVRDRV", "nvr_setDeviceAddress", nvrError);		// aspetta che lo vengano a prendere
#endif
	}
}


// legge dalla memoria "user" dell'eep
bool nvr_readUserMem(void *__data_buf, int __count, int __start_addr)
{
	if( __start_addr > NVR_LAST_USERADDR || __start_addr + __count > NVR_LAST_USERADDR )
	{
		nvrError = NVR_ERR_USERADDR;

#ifdef __DEBUG__
		error_trace("NVRDRV", "nvr_readUserMem", nvrError);		// aspetta che lo vengano a prendere
#endif
		return(False);
	}
	
		
	return( drv_readMem(__data_buf, __count, __start_addr + NVR_FIRST_USERADDR) );			// scrive sommando l'offset dell'area "user"
}

// scrive nella memoria "user" dell'eep
bool nvr_writeUserMem(void *__data_buf, int __count, int __start_addr)
{
	if( __start_addr > NVR_LAST_USERADDR || __start_addr + __count > NVR_LAST_USERADDR )	// rapido check
	{
		nvrError = NVR_ERR_USERADDR;

#ifdef __DEBUG__
		error_trace("NVRDRV", "nvr_writeUserMem", nvrError);		// aspetta che lo vengano a prendere
#endif
		return(False);
	}	

	return( drv_writeMem(__data_buf, __count, __start_addr + NVR_FIRST_USERADDR) );			// scrive sommando l'offset dell'area "user"
}

// legge i parametri dalla memoria dell'eep
bool nvr_readParam(void *__param_buf, int __count, int __start_addr, bool *__chkOk)
{
bool chkMatch;
bool readOk;
bool writeOk;
word chkBlock;
word chkBlock2;

	*__chkOk = False;					// reset flag...

	if( __start_addr > NVR_LAST_PAR1ADDR || __start_addr + __count > NVR_LAST_PAR1ADDR )	// rapido check
	{
		nvrError = NVR_ERR_PARADDR;		// indirizzi sbagliati, o abbiamo sforato
		return(False);
	}
										// verifica sul primo blocco
	chkBlock = nvr_chkRead(__start_addr + NVR_FIRST_PAR1ADDR, (2*__count));	// ricorda che scriviamo solo in indirizzi pari quindi impegnamo il doppio
	chkMatch = nvr_chkVerify(__start_addr + NVR_FIRST_PAR1ADDR, __count, chkBlock);
	
	if( chkMatch == True )				// se il chk è ok 
	{
		*__chkOk = True;

		readOk = drv_readMem(__param_buf, __count, __start_addr + NVR_FIRST_PAR1ADDR);		// legge sommando l'offset dell'area "parametri 1"
		writeOk = True;

		if( readOk == True )			// se la lettura è ok...
		{
										// verifica comunque sul secondo blocco
			chkBlock2 = nvr_chkRead(__start_addr + NVR_FIRST_PAR2ADDR, __count);
			chkMatch = nvr_chkVerify(__start_addr + NVR_FIRST_PAR2ADDR, __count, chkBlock2);
		
											// se nel secondo blocco il chk non è a posto .....
											// o comunque se i due chk non sono uguali
			if( !chkMatch || chkBlock2 != chkBlock )
			{
				writeOk = drv_writeMem(__param_buf, __count, __start_addr + NVR_FIRST_PAR2ADDR);	// ricopia sommando l'offset dell'area "parametri 2"
				writeOk &= nvr_chkWrite(__start_addr + NVR_FIRST_PAR2ADDR, __count, &chkBlock);		// ricopia pure il chk
			}
		}

		if( readOk == False || writeOk == False )
		{
			nvrError = NVR_ERR_DEV_IO;
		}			
		return( readOk );
	}
	else
	{
		nvrError = NVR_ERR_CHKPAR1;
	}
											// verifica sul secondo blocco
	chkBlock = nvr_chkRead(__start_addr + NVR_FIRST_PAR2ADDR, __count);
	chkMatch = nvr_chkVerify(__start_addr + NVR_FIRST_PAR2ADDR, __count, chkBlock);
	
	if( chkMatch == True )				// se il chk del secondo blocco è ok 
	{

		*__chkOk = True;

		readOk = drv_readMem(__param_buf, __count, __start_addr + NVR_FIRST_PAR2ADDR);			// legge sommando l'offset dell'area "parametri 2"
		writeOk = True;
		
		if( readOk )					// se legge ok !!
		{
			writeOk = drv_writeMem(__param_buf, __count, __start_addr + NVR_FIRST_PAR1ADDR);	// scrive sommando l'offset dell'area "parametri 1"
			writeOk &= nvr_chkWrite(__start_addr + NVR_FIRST_PAR1ADDR, __count, &chkBlock);		// ricopia pure il chk
		}

		if( readOk == False || writeOk == False )
		{
			nvrError = NVR_ERR_DEV_IO;
		}

		return( readOk );
	}

	nvrError = NVR_ERR_CHKPAR2;

	*__chkOk = False;

	return( False );					// non è andata proprio bene bene ....
}

// scrive i parametri nella memoria dell'eep
bool nvr_writeParam(void *__param_buf, int __count, int __start_addr, bool *__chkOk)
{
word chkBlock;
bool writeOk = True;

	*__chkOk = False;

	if( __start_addr > NVR_LAST_PAR1ADDR || __start_addr + __count > NVR_LAST_PAR1ADDR )	// rapido check
	{
		nvrError = NVR_ERR_PARADDR;

#ifdef __DEBUG__
		error_trace("NVRDRV", "nvr_writeParam", nvrError);		// aspetta che lo vengano a prendere
#endif
		return(False);
	}
		
	chkBlock = nvr_chkCalcBuf(__param_buf, __count); // calcola il chk della struttra da scrivere, non accede in mem		

	writeOk &= drv_writeMem(__param_buf, __count, __start_addr + NVR_FIRST_PAR1ADDR);	// scrive sommando l'offset dell'area "parametri 1"
	writeOk &= nvr_chkWrite(__start_addr + NVR_FIRST_PAR1ADDR, (2*__count), &chkBlock);		// aggiorna il chk

	writeOk &= drv_writeMem(__param_buf, __count, __start_addr + NVR_FIRST_PAR2ADDR);	// scrive sommando l'offset dell'area "parametri 2"
	writeOk &= nvr_chkWrite(__start_addr + NVR_FIRST_PAR2ADDR, (2*__count), &chkBlock);		// aggiorna il chk pure qui

	if( writeOk == False )
	{
		nvrError = NVR_ERR_DEV_IO;

#ifdef __DEBUG__
		error_trace("NVRDRV", "nvr_writeParam", nvrError);		// aspetta che lo vengano a prendere
#endif
	}

	*__chkOk = True;
		
	return( writeOk );
}


// legge un blocco di dati dalla memoria user dell'eep
bool nvr_readUserChkBlock(void *__param_buf, int __count, int __start_addr, bool *__chkOk)
{
bool chkMatch;
bool readOk;
word chkBlock;

	*__chkOk = False;

	if( __start_addr > NVR_FIRST_USERADDR || __start_addr + __count > NVR_LAST_USERADDR )	// rapido check
	{
		nvrError = NVR_ERR_USERADDR;
		
#ifdef __DEBUG__
		error_trace("NVRDRV", "nvr_readUserChkBlock", nvrError);		// aspetta che lo vengano a prendere
#endif
		return(False);
	}

										// verifica sul primo blocco
	chkBlock = nvr_chkRead(__start_addr + NVR_FIRST_USERADDR, __count);
	chkMatch = nvr_chkVerify(__start_addr + NVR_FIRST_USERADDR, __count, chkBlock);
	
	if( chkMatch == True )				// se il chk è ok 
	{
		*__chkOk = True;
		
		readOk = drv_readMem(__param_buf, __count, __start_addr + NVR_FIRST_USERADDR);		// legge sommando l'offset dell'area "parametri 1"

		if( readOk == False )
		{
			nvrError = NVR_ERR_DEV_IO;
	
#ifdef __DEBUG__
			error_trace("NVRDRV", "nvr_readUserChkBlock", nvrError);		// aspetta che lo vengano a prendere
#endif
		}

		return( readOk );
	}

	nvrError = NVR_ERR_CHKUSER;

#ifdef __DEBUG__
	error_trace("NVRDRV", "nvr_readUserChkBlock", nvrError);		// aspetta che lo vengano a prendere
#endif
	return( False );					// non è andata proprio bene bene ....
}

// scrive un blocco di dati nella memoria user dell'eep
bool nvr_writeUserChkBlock(void *__param_buf, int __count, int __start_addr, bool *__chkOk)
{
word chkBlock;
bool writeOk = True;

	*__chkOk = False;

	if( __start_addr > NVR_FIRST_USERADDR || __start_addr + __count > NVR_LAST_USERADDR )	// rapido check
	{
		nvrError = NVR_ERR_USERADDR;

#ifdef __DEBUG__
		error_trace("NVRDRV", "nvr_writeUserChkBlock", nvrError);		// aspetta che lo vengano a prendere
#endif
		return(False);
	}
		
	writeOk &= drv_writeMem(__param_buf, __count, __start_addr + NVR_FIRST_USERADDR);	// scrive sommando l'offset dell'area "user"
	writeOk &= nvr_chkWrite(__start_addr + NVR_FIRST_USERADDR, __count, &chkBlock);		// aggiorna il chk

	*__chkOk = True;

	if( writeOk == False )
	{
		nvrError = NVR_ERR_DEV_IO;
	
#ifdef __DEBUG__
		error_trace("NVRDRV", "nvr_writeUserChkBlock", nvrError);		// aspetta che lo vengano a prendere
#endif
	}

	return( writeOk );
}

// calcola il chk del blocco richiesto
word nvr_chkCalcBuf(void *__data_buf, int __count)
{
register word i;
register byte *pd = (byte *) __data_buf;
word chkCalc;

	chkCalc = 0xFFFF;
	
	for(i=__count; i; i--)                  	// somma tutto
	{
         	chkCalc += *pd++;
	}

	return(chkCalc);
}

// scrive il chk del blocco richiesto
bool nvr_chkWrite(word __blockAddr, word __blockSize, word *__chk)
{
	return( drv_writeMem( (byte *)__chk, 2, __blockAddr + __blockSize) );
}

// legge il chk dei blocco richiesto
word nvr_chkRead(word __blockAddr, word __blockSize)
{
word chkLocal = 0;				// 0 -> valore non ammesso

	drv_readMem((byte *) &chkLocal, 2, __blockAddr + __blockSize);	
	m_chk = chkLocal;
	return( chkLocal );
}

// calcola e verifica il chk dei parametri
bool nvr_chkVerify(word __blockAddr, word __blockSize, word __chk)
{
register int i;
register byte *pd;
word chkCalc;
byte buf[32];
word bufAddr;
word bufSize;

	chkCalc = 0xFFFF;
	bufAddr = 0;
	memset(buf, 0 , sizeof(buf));				// reset buffer....
	
	while( __blockSize )
	{
		if( __blockSize >= 32 )
			bufSize = 32;
		else
			bufSize = __blockSize;
	
		__blockSize -= bufSize;					// scala un attimo
		
												// ne legge un pochetto
		drv_readMem(buf, bufSize, __blockAddr + bufAddr);	
		
		bufAddr += 2*bufSize;						// avanti l'indice al blocco
		
		pd = buf;								// dall'inizio del buffer
		
		for(i=bufSize; i; i--)                  // somma tutto
	         chkCalc += *pd++;
	}
	m_chk = chkCalc;
	return( chkCalc == __chk );					// torna a casa margherita
}


												// include il driver giusto...
#if (NVR_DRV == DRV_M41T91)
	#include "m41t94.cpp"
#elif (NVR_DRV == DRV_EPSON4553)
	#include "e4553.cpp"
#elif (NVR_DRV == DRV_M48T35)
#ifdef __DEBUG__
	#warning m48t35 selected ....
#endif	
	#include "m48t35.cpp"
#endif


//@@-----------------------------------------------------------------------@@//
//@@
//@@  $Source:  $
//@@  Note di revisione:
//@@
//@@  $Revision: 1.2 $
//@@  $Date: 2004/01/28 00:00:00 $
//@@  - Viene aggiunto il controllo del secondo blocco  di parametri anche 
//@@	se il primo blocco è ok (funzione nvr_readParam)
//@@  - Viene modificato l'ordine di scrittura dei parametri e del crc 
//@@	(funzione nvr_writeParam)
//@@  - Viene bypassato il controllo del crc per decidere se eseguire la 
//@@	scrittura (funzione nvr_writeParam) -> riattivabile se si modifica
//@@	la routine di calcolo del crc.
//@@  - Aggiunte due funzioni per leggere e scrivere blocchi di dati nell'area 
//@@	"user" con il calcolo del chk: 
//@@	"nvr_writeUserChkBlock" & "nvr_readUserChkBlock"
//@@
//@@  $Revision: 1.3 $
//@@  $Date: 2004/02/02 00:00:00 $
//@@  - Aggiunta la var. di traccia degli errori del modulo "nvrErrors" + 
//@@	la traccia visiva in debug con la chiamata alla "error_trace" 
//@@  - Aggiunta la funzione "nvr_errors" per poter leggere ed azzerare 
//@@	l'errore interno al modulo
//@@
//@@  $Log:  $
//@@
//@@  $Author: junior $
//@@  $RCSfile: NVRDRV.CPP,v $
//@@  $Revision: 1.1 $
//@@  $State: Exp $
//@@  $Date: 2004/01/02 00:00:00 $
//@@  $Name:  $
//@@-----------------------------------------------------------------------@@//
//@@-----------------------------------------------------------------------@@//
//@@                                                                       @@//
//@@                            C O M E T A                                @@//
//@@                                                                       @@//
//@@-----------------------------------------------------------------------@@//
