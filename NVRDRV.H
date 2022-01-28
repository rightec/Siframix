//---------------------------------------------------------------------------//
//                                                                           //
//                           C O M E T A                                     //
//                                                                           //
//---------------------------------------------------------------------------//
//
//  Progetto:                GENLIB
//  Nome File:               NVRDRV.H
//  Descrizione:             Definizioni per gestione memoria non volatile
//  Versione:                $Revision: 1.3 $
//  Data Versione:           $Date: 2004/02/02 00:00:00 $
//
//---------------------------------------------------------------------------//

#include "global.h"
#include "error.h"
#ifndef  __NVRDRV__
#define  __NVRDRV__
#else
#error Macro __NVRDRV__ already def.!!
#endif

#include "m48t35.h"

//--------------------------------------------------------//
// Definizione delle costanti
//--------------------------------------------------------//

								// definizioni per la selezione del tipo di driver e di memoria
#define DRV_M41T91		4191
#define DRV_EPSON4553	4553
#define	DRV_M48T35		4835

#define NVR_BASE_ERR	BASEMOD_NVRDRV	// numero di partenza per gli errori


//--------------------------------------------------------//
// Definizione dei tipi
//--------------------------------------------------------//

enum NVRErrors{ NVR_ERR_NONE,
				NVR_ERR_DEV_ADDR=NVR_BASE_ERR,		// device address errato
				NVR_ERR_DEV_IO,						// problemi di input/output dal device
				NVR_ERR_USERADDR,					// inidirizzo memoria user errato
				NVR_ERR_PARADDR,					// inidirizzo memoria parametri errato
				NVR_ERR_CHKPAR1,					// crc errato area par1
				NVR_ERR_CHKPAR2,					// crc errato area par2
				NVR_ERR_CHKUSER						// crc errato area user
				};

typedef enum NVRErrors NVRErrors;


//--------------------------------------------------------//
// Definizione delle classi 
//--------------------------------------------------------//

//--------------------------------------------------------//
// Definizione delle funzioni
//--------------------------------------------------------//
#ifdef __cplusplus
extern "C" {
#endif

void nvr_init(byte __deviceAddr);
int  nvr_errors(void);
void nvr_setDeviceAddress(byte __deviceAddr);
bool nvr_readUserMem(void *__data_buf, int __count, int __start_ddr);		// legge dalla memoria dell'eep
bool nvr_writeUserMem(void *__data_buf, int __count, int __start_addr);		// scrive nella memoria dell'eep
bool nvr_readParam(void *__param_buf, int __count, int __start_ddr, bool *__chkOk);		// legge i parametri dalla memoria dell'eep
bool nvr_writeParam(void *__param_buf, int __count, int __start_addr, bool *__chkOk);	// scrive i parametri nella memoria dell'eep
bool nvr_readUserChkBlock(void *__param_buf, int __count, int __start_addr, bool *__chkOk);
bool nvr_writeUserChkBlock(void *__param_buf, int __count, int __start_addr, bool *__chkOk);
word nvr_chkCalcBuf(void *__data_buf, int __count);
bool nvr_chkWrite(word __blockAddr, word __blockSize, word *__chk);			// scrive il chk del blocco richiesto
word nvr_chkRead(word __blockAddr, word __blockSize);						// legge il chk dei blocco richiesto
bool nvr_chkVerify(word __blockAddr, word __blockSize, word __chk);			// calcola e verifica il chk dei parametri

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
//@@  $Date: 2004/01/28 00:00:00 $
//@@  - Aggiunta la definizione dei due funzioni per leggere e scrivere 
//@@	blocchi di dati nell'area "user" con il calcolo del chk: 
//@@	"nvr_writeUserChkBlock" & "nvr_readUserChkBlock"
//@@
//@@  $Revision: 1.3 $
//@@  $Date: 2004/02/02 00:00:00 $
//@@  - Aggiunto l'enum per la traccia degli errori  
//@@  - Aggiunta la definizione della funzione "nvr_errors" 
//@@
//@@
//@@  $Log:  $
//@@
//@@  $Author: junior $
//@@  $RCSfile: NVRDRV.H,v $
//@@  $Revision: 1.2 $
//@@  $State: Exp $
//@@  $Date: 2001/12/16 06:49:06 $
//@@  $Name:  $
//@@-----------------------------------------------------------------------@@//
//@@-----------------------------------------------------------------------@@//
//@@                                                                       @@//
//@@                            C O M E T A                                @@//
//@@                                                                       @@//
//@@-----------------------------------------------------------------------@@//
