/**
@file		CPUSIFRA_Manager.h
@brief		CPUSIFRA states machine.

			This class manage the machine states of the device:
			- receives messages from the communication protocol decoding classes 
			- manages the acquisition channels
			- .....
			
@author		Fregni Francesco
@date		18/01/2011
@version	01.00
*/

#include "global.h"
#include "nvrdrv.h"
#include "protocolSIFRA.h"
#include "pwm.h"
#include "adc.h"
#include "util.h"
#include "Leds_panel.h"
#include "Arith.h"

#define _LOAD_SYSTEM2_DEFAULT_	1.0

#define LIMITE_PESO					30 				// 25 decimi di grammo (2.5 grammi) che è già un'enormità in condizioni di pompe ferme
#define LIMITE_PESO_PRECISIONE		20 				// velocità settata in prossimità del target. quando mancano 15 decimi di grammo (1.5 grammi)
#define LIMITE_PESO_MINIMO			6
#define LIMITE_PESO_ENCODER		5
#define LIMITE_PESO_FINE_EROG		1
#define LIMITE_PESO_RT				15

/*
* Soglie utilizzate nell'algoritmo SampleManager per valutare l'errore di peso instabile.
* Il livello 1 è quello più sensibile alla variabilità del peso, il livello 4 quello più robusto a tale condizione.
*/
#define THREESHOLD_FILLING_INCR_LEVEL_1		22				// 22 decimi di grammo (2.2 grammi)
#define THREESHOLD_FILLING_INCR_LEVEL_2		25				// 25 decimi di grammo (2.5 grammi)
#define THREESHOLD_FILLING_INCR_LEVEL_3		35				// 35 decimi di grammo (3.5 grammi)
#define THREESHOLD_FILLING_INCR_LEVEL_4		50				// 50 decimi di grammo (5.0 grammi)

/*
* Soglie variazioni peso in pausa formula
*/
#define _MIN_STOP_WEIGHT_VARIATION_				5		// 0,5 grammi di variazione peso ammessa
#define _MAX_STOP_WEIGHT_DECREASE_				10		// 1 grammo di calo peso ammesso
#define _STOP_WEIGHT_VARIATION_BAG_CHANGE_		500		// una variazione di almeno 50 grammi la associo ad un cambio sacca o flacone

#define DEPR_TEST_PINZE			6				// 6 decimi di grammo medi che calano per l'asimmetria dei giri motore del test pinze

#define THRESHOLD_WEIGHT			10 				// 1g
#define T_NO_CALO_MANUAL			3500			// tempo di monitoraggio dell'allarme non cala con motore a basso regime
#define T_NO_CALO_INIT				3000			// tempo di monitoraggio dell'allarme non cala con motore a basso regime
#define T_NON_CALA					3000			// tempo di monitoraggio dell'allarme non cala
#define T_NON_CALA_ENC				5000
#define T_NON_CALA_RIFINITURA		8000
#define T_NO_CALO_MIN				1000				// tempo di monitoraggio minimo dell'allarme non cala con motore a velocità elevata
#define _WAIT_3_SEC					3000
#define VELOCITA_TEST_EV			25
#define SOGLIA_PARZ_TEST_PINZE 	3				// nel primo controllo sono molto stringente per escludere perdite dalle pinze, 0.3 mL
#define SOGLIA_TEST_PINZE			6				// se il primo mezzo giro ha rilevato variazione peso al secondo controllo sono meno stringente, facendo un altro mezzo giro se ci sono perdite sono sicuramente maggiori
#define PRIMA_VELOCITA				15				// primi 2ml a bassa velocità
#define SECONDA_VELOCITA			50				// poi 4 ml più forte
#define _VOL_RT_NEAR_TARGET		10

#define _SOGLIA_ENC_ 				500		// corrisponde ad un sedicesimo di giro
#define _SOGLIA_ENC_NON_GIRA		120		// valore calcolato empiricamente misurando il numero di impulsi encoder per giro di pompa calcolato a diverse velocità della pompa e poi mediato.
											//Misura fatta tramite funzione Debug_vel_pompe del supervisore
#define _PERC_SOGLIA_ENCODER		0.3		//Valore percentuale di errore ammesso sul calcolo teorico degli impulsi encoder in base alla velocità della pompa per generare l'errore di pompa che non gira

/* TEMPO PER funzionamento del watchdog timer che se scattato ferma tutti i motori */
#define WATCHDOG_TIME				7000			// 7 secondi di tempo prima che intervenga il blocco dei motori

#define PESO_MASSIMO				43000			// peso massimo ammissibile su M3300 = 4300g, + un minimo margine
#define _TIMEOUT_CHKHW_INIT_		30000			// ogni 5sec verifica se il valore letto dalla cella è indice di FS o rottura
#define _TIMEOUT_CHKHW			5000			// ogni 5sec verifica se il valore letto dalla cella è indice di FS o rottura
/* margine per gestire il vuoto ed evitare entrata di aria */
#define _MARGINE_SU_VUOTO			5				// 0.5ml di tolleranza
#define _TH_ERR_VUOTO_			15				// fattore moltiplicativo usato per segnalare errore vuoto per tempo in erogazione
/* definizione maschere bit per segnali encoder (word)*/
#define  _MASK_16BIT_				0x0000FFFF
#define _t_BLINK_OTHER_ERR 		200				// errore peso instabile o vuoto
#define _t_BLINK_AIR_ERROR 			800				// errore aria rilevata
#define _t_BLINK_FATAL_ERROR  		800				// errore definitivo di cella rotta o peso massimo
#define _t_BLINK_START				_t_BLINK_AIR_ERROR	// LAMPEGGIO All'avvio dela macchina che serve come checkup
#define _t_BLINK_STOP 				500				// stop di via abilitata
/*Definizione tempi gestione rampe motori*/
#define _t_RAMPA_RT					20 				// tempo aggiornamento velocità motore in rampa accelerazione riempitubi
#define _t_RAMPA_SV					40				// tempo aggtior. velocità motore in rampa stato service

/* delta adc di differenza accettabile sul valore di zero della cella */
#define _DELTA_ADC_1_PERC_FS_		656			// 1 % del fondo scala della cella di carico
#define _DELTA_ADC_0Kg				(_DELTA_ADC_1_PERC_FS_ * 3)	// 3% del fondo scala
#define _DELTA_ADC_0Kg_DEF			(_DELTA_ADC_1_PERC_FS_ * 5)	// 5% del fondo scala
//#define _DELTA_ADC_0Kg				2000		// x % del FS in adc

/* valori adc plausibili per verifica guadagno in calibrazione	*/
#define _ADC_2KG_MIN 				19000
#define _ADC_2KG_MAX 				23000
#define _DELTA_ADC_2Kg				655	// 1% del FS in adc

#define _ADULTI_					0	// settaggio linea adulti
/* volumi riempitubi a seconda della linea */
#define _VOL_PRIMOSTEP 				20	// 2 ml di primo step a bassa velocità
#define _VOL_RT_MAX					90
/* frequenza campionamento */
#define _SAMPLE_FREQ 				60.0		// frequenz adi campionamento effettivo degli ADC
/* adattamento velocità pompa -> protata per predizione calo peso reale */
#define Speed_to_flow				0.63		// misura di flusso empirica, (ml/(PWM *s))*10. Il *10 è dovuto alla gestione dei pesi in decimi di grammo e non in grammi. Per ottenere ml/s devo moltiplicare per la velocità della pompa espressa in PWM

#define _TIMER_INIT_READ_AIR 		5000		// timer per lettura aria che avviene quindi ogni 100msec
#define _TIMER_READ_AIR 			100		// timer per lettura aria che avviene quindi ogni 100msec
#define _TIMER_READ_AIR_CONTROL	30		// timer per lettura aria in caso di singola segnalazione pari a 30 ms
#define _TIMER_READ_AIR_ALERT		10		// timer per lettura aria in caso di possibile allarme aria, pari a 10 ms
#define _VOLUME_MAX				3.0		// 0,3ml di volume massimo tollerabile
#define _TEMPO_DI_ARIA				20		// numero di giri
#define _330_mSEC_					330
#define _500_mSEC_					500
#define _1_SEC_						1000
#define _2_SEC_						2000
#define _3_SEC_						3000
#define _4_SEC_						4000
#define _5_SEC_						5000
#define _20_SEC_						20000
#define TIME_PINCH_CONTROL			4000	// tempo di attesa esecuzione mezzo giro in avanti o indietro nella fase di test pinze
#define TIME_PINCH_WEIGHT_STABLE		2500	// temp di attesa stabilizzazione peso necessario a rilevare un eventuale errore test pinze
#define _TIME_x_BACKUP 				400
#define CONST_TEST_PINZE			0.67		// costante usata per impostare sbilanciamento rotazione pompa nel test pinze tra andata e ritorno

#define _SOGLIA_					0.3		// margine in percentuale per accettazione nuovo valore di rapporto flusso/peso rispetto il nominale
#define _NON_CALA_THRESHOLD		5	 	// margine in percentuale usato per il controllo errore non cala nella funzione controllo_errori_linea
#define _NON_CALA_THRESHOLD_ENC	3
#define SOGLIA_ERR_PRODOTTO		0.3		// soglia relativa all'errore sul volume erogato rispetto a quello impostato
#define _SOGLIA_CORR_BALANCE		0.04		// soglia correzione lettura pesi su vie più vicine a celle sinistre usata nel calcolo dell'errore prodotto
#define SOGLIA_RIEMPITUBI			30		//con una media empirica sulle linee abbiamo visto che il peso di liquido erogato tra l'imbocco del flacone e il sensore aria è di 2.55 grammi. Impostiamo la soglia più ampia a 3 grammi (30 dg)

#define _VOL_MIN_x_RESTART		2	// 0,2 ml volume min per provare a ripartire dopo interruzione in riempimento
#define _VOL_MIN_FOR_ERR_PROD	50	// volume minimo continuativo erogato di 5ml per verifica errore prodotto
#define _WAIT_M300_READING		2000	// secondi di attesa per fare leggere il cambio stato alla tastiera
#define _DELTA_MIN					7	// delta di 0.4ml per verifica "non cala"
#define _MILLE_						1000.0
#define _PESO_SPEC_MIN_			0.8
#define _PESO_SPEC_MAX_			1.3
#define _MAX_WORD_				65535

#define _500_MS_					500
#define _TIME_TEST_FW				20000

#define _TIMER_LED_FAST_			100
#define _TIMER_LED_MEDIUM_		200
#define _TIMER_LED_SLOW_			800

#define _ENC_STEP_EXPECTED			2050	// valore empirico

#define _PERC_ENC_CONTROL			0.1

#define _PAR_ENC_NOM_				445.0		// parametro conversione flusso/peso nominale, step encoder/decigrammi
#define _VOL_ST_MAX_				1500

#define _VIA_8_						0
#define _VIA_15_						7

typedef struct
{
	long this_sample;	// campione attuale
	long next_sample;	// valore stimato del prossimo campione
}ViaSample;

typedef CSmallRingBuf<ViaSample, 2> WEIGHT_SAMPLE;

/**
Board's states
*/
enum SIFRA_Status
{
	InitStatus = 0,					// inizializzazione sistema
	AttesaComando,					// stato di attesa comandi da tastiera, sistema inattivo
	StatoCalibrazione,				// calibrazione eseguibile da utente
	StatoRiempitubi,					// stato controllo riempitubi (allarme aria disab, led attivo, velocità fissa bassa poi alta
	StatoSvuotatubi,					// stato controllo svuotamento tubi per cambio linea (allarmi disabilitati in toto, led aria ttivo)
	StatoManuale, 			// 5		// avvio manuale della pompa (svuotamento/riempimento manuale, sfiato
	StatoRisciacquo,					// stato di riempitubi solo finalizzato al risciacquo linee M3300 (quindi per ora va solo su via 15)
	StatoSacca,						// qui si gestisce il riempimento della sacca, una via per volta, tutti gli allarmi attivi
	StatoErrore,						// stato di errore
	StopAllStatus,					// stato di allarme, ci si arriva per stop esterno
	StatoStop,				// 10	// stato in cui si gestisce lo stop di linee in erogazione, il backup le luci di allamre
	PausaErogazione,					// stato inserito per identificare lo stato di attesa conclusione esecuzione formula
	//PausaFormulaService,
	StatoTestPinze,					// stato di controllo della tenuta delle pinch e del posizionamento della linea
	StatoServiceEV,					// test ciclico sulle elettrovalvole, solo service
	StatoService,				// 15	// stato di service utile per i test di validazione o per le prove
	StatoTestEMC,					// stato usato per i test EMC
	StatoTestLuci,					// stato usato per verificare il corretto azionamento dei led sulla scheda luci
	StatoTestPompe,					// routine di test delle velocità pompe, per calcolo flussi, rapporti flusso peso, ecc.
	StatoTestFlussoPompa,			// routine di test per calcolo flusso pompa a diverse velocità
	StatoTestEvAperte,		// 20	// stato per test attivazione di 2 o più EV contemporaneamente
	RipresaMancanzaRete,			// stato necessario per distinguere la rilevazione degli errori al riavvio dopo mancanza di corrente
	TermineSequenza,				// stato usato per resettare le variabili di riempimento
	E_WAIT_ENTER_LOADER
};

enum STATUS_MSG_START
{
	E_RESET_SISTEMA		= 0x0081,	// abort + def_calib, ricevuto a fine sacca
	INIT_SISTEMA 			= 0x0001,	// abort per inizializzare lo stato sacca
	RESTART_SEQUENZA 		= 0x0002,	// solo start per riprendere una sacca interrotta
	NUOVA_SEQUENZA 		= 0x0003,
	DEBUG_ELETTROVALVOLE = 0x0005,	// tara + abort
	DEBUG_LUCI				= 0x0006,	// start + tara
	DEBUG_VEL_POMPE 		= 0x0007,	// abort+start+tara per provare la velocità pompa
	MANUALE_CON_OBIETTIVO = 0x0008,	//MANUALE_CON_OBIET_2 	= 0x000A,
	RISCIACQUO				= 0x0009,	// funzione di risciacquo linea da eseguire dopo abbandono sacca
	RISCIACQUO_DA_SUPE	= 0x004B,	// abort + start + manuale + all. aria disabil.
	SVUOTATUBI				= 0x000A,	// manuale
	DEBUG_SERVICE 			= 0x000B,	// 00001011 abort+start+manuale
	E_TEST_PKTS_DEBUG		= 0x000C,	// 00001100 manuale + tara
	E_TEST_OPEN_MORE_EV	= 0x000D,	// 00001101 manuale + tara + abort
	TEST_EMC				= 0X000F,	// 00001111 abort + start + tara + manuale
	MANAGER_LUCI_CALIBR 	= 0x0010,
	DEBUG_VEDI_ADC_VIE 	= 0x0013,	// abort+start+calibrazione
	E_TEST_FLOW_PUMP		= 0x0017,	// 00010111 calibrazione + tara + start + abort
	RIEMPITUBI 				= 0x0022,	// start + riempitubi
	TEST_PINZE				= 0x0023, 	// start + abort + riempitubi	
	DEFAULT_CALIBR 		= 0x0080,
	E_USR_CALIB_CMD		= 0x0082,	// start + calib. incond.
	E_VERIFY_CALIB_VAL_CMD = 0x0083,	// comando utilizzato per leggere in tempo reale i valori di calibrazione memorizzati nella EEPROM
	RESET_ADC_ON_BOARD	= 0x001B		// abort+start+manuale+calibrazione
};

/**
Filling Status states
*/
enum sifra_phase_t
{
	E_PHASE_INIT = 0,
	E_PHASE_IDLE,
	E_PHASE_FILLING,
	E_PHASE_PAUSE,
	E_PHASE_SERVICE,
	E_PHASE_CALIBRATION,
	E_PHASE_SVUOTATUBI,
	E_PHASE_END_SEQUENZA,
	E_PHASE_ERROR,
	E_PHASE_RIPRESA_NO_ALIM,
	E_PHASE_NUM
};

/*
* Enum usato per stabilire le soglie da utilizzare nell'algoritmo peso instabile in base a quanto inviato dalla tastiera
*/
enum
{
	E_LEVEL_UNSTABLE_NONE = 0,
	E_LEVEL_UNSTABLE_LOW,
	E_LEVEL_UNSTABLE_MED_1,
	E_LEVEL_UNSTABLE_MED_2,
	E_LEVEL_UNSTABLE_HIGH,
	E_LEVEL_UNSTABLE_NUM
};

/*
* Enum usato per stabilire lo stato di idle pompe da quello di erogazione
*/
enum
{
	INIT_DRIVER = 0,
	CONTROLL_DRIVER,
	E_WAIT_DRIVER
};

/**
* States of stability verify routine in SampleManager
*/
enum StabilityStates
{
	init_stab = 0,
	stabile,
	forse_instabile,
	instabile,
	wait_stabile,
	E_VOID_QUEUE
};

/**
* Pinch test status states, usato per test pinze e debug EV
*/
enum PINCHTESTSTATUS
{
	E_PINCH_TEST_IDLE = 0,
	E_PINCH_TEST_CONFIG,
	E_PINCH_TEST_ABIL_PUMP_CW,
	E_PINCH_TEST_SET_SPEED_CW,
	E_PINCH_TEST_MOVE_CW,
	E_PINCH_TEST_FIRST_CONTROL,
	E_PINCH_TEST_ERR_MOV_CW,
	E_PINCH_TEST_SET_SECOND_CW,
	E_PINCH_TEST_SECOND_MOVE_CW,
	E_PINCH_TEST_SECOND_CONTROL,
	E_PINCH_TEST_ABIL_PUMP_CCW,
	E_PINCH_TEST_SET_SPEED_CCW,
	E_PINCH_TEST_MOVE_CCW
};

enum ev_test_status_t
{
	E_EV_TEST_IDLE = 0,
	E_EV_TEST_OPEN,
	E_EV_TEST_CLOSE,
	E_EV_TEST_NUM
};

/*
* Stati utilizzati nella LedManager
*/
enum LED_STATUS_MANAGER
{
	DISPLAY_INIT = 0,
	DISPLAY_READY,
	CONTROLLO_LUCE,
	BLINK_LEDS,
	BLINK_LEDS_HW_ERROR,
	DISPLAY_IDLE,
	DISPLAY_RE_INIT_1,
	DISPLAY_RE_INIT_2,
};

/**
* Macchina a stati utilizzata nella AirInManager
*/
enum AIR_STATUS
{
	E_START_AIR = 0,	// l'algoritmo di rilevazione aria è inizializzato a questo stato dall'abort e dallo start (riempitubi o sacca)
	E_CONTROL_AIR,		// in questa fase viene verificata la presenza di aria durante lo start e se viene rilevata aria si passa allo stato alert
	E_ALERT_AIR,		// stato transitorio per evitare letture spurie isolate (o bollicine trascurabili); se l'aria viene confermata, passa in attivo
	E_ACTIVE_AIR,		// qui si esegue l'integrazione della bolla: se supera il volume minimo passa in allarme
	E_ALARM_AIR,		// qui rimane finchè c'è aria, altrimenti prima va in reset per verificare 
	E_RESET_AIR		// qui riverifica che sia veramente finita e in tal caso resetta lo stato e torna in controllo
};

/**
* Macchina a stati usata nel controllo errore non cala
*/
enum NON_CALA_STATUS
{
	no_allarme_non_cala = 0,		// nessuna anomalia legata al non cala
	alert_1_non_cala,					// forse non cala, da verificare
	allarme_non_cala				// allarme non cala confermato
};

/**
* Enum utilizzato per distinguere errore prodotto da errore di flusso
*/
enum ERRORE_VOLUME
{
	no_errore_volume = 0,
	errore_prodotto,			// errore compreso tra l'8 ed il 30 %
	errore_flusso,				// errore compreso tra il 30 ed il 98 %
	errore_generico
};

/**
* Enum utilizzato per identificare lo stato luci di ogni via (variabile stato_luci)
*/
enum
{
	IDLE_LUCE = 0,
	SPEGNI_LUCE,
	SETTA_LUCE,
	BLINKA_LUCE,
	ERR_VIA,
	ERR_CELLA,
	ERR_ARIA,
	ERR_INSTABILE
};

/**
* Calibration Status States
*/
enum CALIBSTATUS
{
	StartAcq = 0,
	ReadZeroInBothLines,
	ReadLoadCellSX,
	ReadLoadCellDX,
	E_END_CALIB
};

/*
* Enum usato per macchina a stati SaccaManager
*/
enum SACCA_STATUS
{
	SaccaIdle = 0,
	ConfigurazioneSequenza,		// settaggio via da eseguire e impostazione sequenza
	StartSequenza,				// si setta quale via della sequenza ora deve partire
	RipresaSequenza,		// si riprende una sequenza interrotta, qui si verifica la via interrotta e si caricano i dati backuppati
	StartErogazioneVia,		// fase che da il via al riempimento specifica per M3200 e M3300
	StatoStartPompa,	// minimo ritardo fra apertura elettrovalvola e start pompa
	StatoErogazione,		// stato riempimento sacca specifica per M3200 e M3300
	StatoControlloEncoder, 		// il riempimento è controllato a encoder
	StartRifinituraErogazione,
	RifinituraErogazione,
	ChiusuraErogazione,			// fine sacca
	FineErogazione,				// chiusura start sulla via in esecuzione
	FineSequenza				// chiusura della sequenza in esecuzione
};

/**
* Enum usato per fase manuale e svuotamento linee
*/
enum MANUALSPEEDSTATUS
{
	IdleManuale = 0,
	StartManuale,
	Apert_EV_Manuale,
	ControlloManuale,
	ControlloManuale_Fine,
	ChiusuraManuale
};

/**
* Enum usato per fase riempitubi e risciacquo
*/
enum HIGHSPEEDSTATUS
{
	IdleRiempitubi = 0,
	SetupRiempitubi,
	StartRiempitubi,
	StartPompaRiempitubi,
	SetVelocitaRiempitubi,
	ControlloFase1,			// 5
	ControlloFase2,
	ControlloFase3,
	ErogazioneAggiuntiva,
	AttendiUltimoPeso,
	ChiusuraRiempitubi
};

/**
* Service status states, usato in ServiceManager, TestEMC_Manager e TestPompe_Manager
*/
enum SERVICE_STATUS_MANAGER
{
	E_SERVICE_NONE = 0,
	setup_via,
	Start_pompe,
	Controllo_pompe,
	calcoli,
};

/**
* Service status states, usato in ServiceManager, TestEMC_Manager e TestPompe_Manager
*/
enum service_ev_test_t
{
	E_SERVICE_EV_START = 0,
	E_SERVICE_EV_OPEN,
	E_SERVICE_EV_STOP
};

/**
* Macchina a stati usata nella VerificaESD_Damage
*/
enum CheckEsdDamageMachineState
{
	verifica_contatore = 0,
	contatore_overflow,
	verifica_watchdog_timer,
	verifica_contatore_dopo_reset_1,
	verifica_contatore_dopo_reset_2,
	reset_scheda
};

/**
* Stati macchina di test della scheda luci
*/
enum LED_TEST
{
	E_LED_START = 0,
	E_LED_RED,
	E_LED_1_GREEN,
	E_LED_1_RED,
	E_LED_2_GREEN,
	E_LED_2_RED,
	E_LED_3_GREEN,
	E_LED_3_RED,
	E_LED_4_GREEN,
	E_LED_4_RED,
	E_LED_5_GREEN,
	E_LED_5_RED,
	E_LED_6_GREEN,
	E_LED_6_RED,
	E_LED_7_GREEN,
	E_LED_7_RED,
	E_LED_8_GREEN,
	E_LED_8_RED,
	E_LED_GREEN,
	E_LED_STOP,
	E_LED_NUM
};

enum type_of_filling_t
{
	E_TYPE_FORMULA = 0,
	E_TYPE_SERVICE
};

enum scale_phase_start_end_t
{
	E_SCALE_PHASE_START = 0,
	E_SCALE_PHASE_STOP
};

enum type_of_scales_phase_t
{
	E_TYPE_SCALE_VERIFICATION = 0,
	E_TYPE_SCALE_CALIB_MEM,
	E_TYPE_SCALE_CALIB_USR,
	E_TYPE_SCALE_NONE
};

enum test_pinch_exec_t
{
	E_TEST_PINCH_TO_DO = 0,
	E_TEST_PINCH_DONE
};

/**
Manage the machine state of board.

@see VoltageHwChanManager
@see CurrentHwChanManager
@see GenericHwChanManager
@see SifraProtocol
*/
class SIFRA_Manager
{
	public:
		
		//SIFRA_Manager(Keyboard *keyboard, SIFRAProtocol *SIFRAProtocol, UnisensorProtocol *uniProtocol);
		SIFRA_Manager(SIFRAProtocol *SifraProtocol);
		~SIFRA_Manager();

		void Manager();

		/**
		Sets the class that manages the power voltage samples.
		@param v poiter to the class VoltageHwChanManager
		@see VoltageHwChanManager
		*/
		void setVoltageHwChanManager(VoltageHwChanManager <word, _PWR_VOLTAGE_BUFFER_LENGTH_> *v){ m_powerVoltageChan = v;};
		//void setLoadChan(LoadHwChanManager <word, _LOAD_BUFFER_LENGTH_> *vol){ m_loadChan = vol;};
		/**
		Sets the class that manages the weight samples.
		@param w poiter to the class GenericHwChanManager
		@see setWeightChan
		*/
		void setWeightChan(GenericHwChanManager <dword, _WEIGHT_BUFFER_LENGTH_> *w){ m_weightChan = w;};
		
		/**
		Reads the hardware flags which indicate board hardware versione
		*/
		byte readHardwareVersion(){return (byte)(( cpld_reg_in >> 8) & 0x03);};
		/**
		Set a error occourred in eeprom reading
		*/
		void setErrorEEpromReading();
				
	protected:
		//---- Messages Handler ---
		void SIFRAcheck_new_command();
		int SIFRAMsg_setStopAllHandler();
		int SIFRAMsg_setStartHandler();
		int SIFRAMsg_statusAskedHandler(); 
		int SIFRAMsg_infoAskedHandler();
		int SIFRAMsg_debugViaAskedHandler();
		bool SIFRAMsg_debugStartAskedHandler();
		bool SIFRAMsg_debugStatusAskedHandler();
		bool SIFRAMsg_debugRamAskedHandler();
        	int SIFRAMsg_startAcquisitionHandler();
		int SIFRAMsg_resetApplicationHandler();
		int SIFRAMsg_hwErrorHandler();
		int SIFRAMsg_protocolErrorHandler();
		int SIFRAMsg_unknownErrorHandler();   
		//--------------

		/*
		* Metodi usati per resettare le variabili
		*/
		void SifraResetSystemParams();
		void SifraResetViaStatus();
		void SifraResetStatus();
		void SifraResetStartCmd();
		void SifraResetStateMachine();
		void SifraResetFillingData();
		void SifraResetServiceData();
		void SifraResetBackup();
		void SifraStopTimers();
		void SIFRA_resetStatusErrorStop();
		void SIFRA_resetStatusError();
		void Init_Variables();
		void DefaultSystemReset();
		void ResetMotorStepCounter();

		void FlashLed_hw_error();
		bool SetHardwareVersion();
		byte changeSIFRAstatus(byte newStatus);
		byte getSIFRAstatus();
		byte getSIFRApreviousStatus();
		byte getSIFRAphase();
		void SIFRAPhase_manager();
		
		/*
		* Metodi associati all'azionamento delle elettrovalvole
		*/
		void SIFRAOpenLineEV(int m_line);
		void SIFRACloseLineEV(int m_line);
		
		/*
		* Metodi associati alla lettura ed al calcolo dei pesi
		*/
		float GetSystemLoad(dword* samples);
		void Setta_filtro_peso(int val);
		word calc_next_theor_weight(int i);
		float calc_param_weight(ChannelsBackupParam *m_chan);
		
		/*
		* Metodi associati all'azionamento della pompa
		*/
		void encoder_updateCnt(void);
		void setStatePump(int i, int command);
		int getStatePump(int i);
		
		/*
		* Metodi usati in calibrazione
		*/
		bool readZeroInBothLine();
		bool readLoadInLine8();
		bool readLoadInLine15();
		
		/*
		* Metodi usati in algoritmo aria
		*/
		void set_air_alarm(int i);
		float calc_flusso_pompa();			// metodo usato per calcolare i volui delle bolle

		/*
		* Metodo usato per controllare la corretta movimentazione della pompa
		*/
		bool Verifica_encoder_motori();

		/*
		* Metodo usato per controllare che le celle di carico non siano guaste
		*/
		bool Verifica_celle();

		/*
		* Metodo usato per controllare che la rastrelliera non sia sovraccaricata
		*/
		bool Verifica_peso_massimo();

		/*
		* Metodo usato per controllare che non venga aperto lo sportello 
		*/
		bool Verifica_apertura_sportello();
		
		/*
		* Metodo usato per convertire i volumi in pesi
		*/
		float AdattaPesoSpecifico(word pes_spec);

		/*
		* Metodo usato per settare la capacità ad inizio di un'erogazione
		*/
		bool controllo_capacita_iniziale(word cap_passata);

		/*
		* Metodo usato per aggiornare il valore di capacità
		*/
		word aggiorna_capacita(word cap_ini, word peso_erogato, float peso_spec);

		/*
		* Metodo usato per aggiornare il valore di capacità
		*/
		void set_unstable_threshold(byte __level);

		/*
		* Metodo usati per l'accensione dei led sulla scheda luci
		*/
		void setta_led_pannello(int line, bool light);	//void setta_led_pannello(int line, word light);
		void reset_state_this_led(int line, bool state);

		/*
		* Metodi usati per salvare e leggere i dati in RAM tamponata associati alla struttura di backup
		*/
		bool Write_Param_Backup();
		bool Read_Param_Backup();

		/*
		* Metodi usati per salvare e leggere i dati in RAM tamponata associati alla struttura struct
		*/
		bool Write_Param_Encoder();
		bool Read_Param_Encoder();

		/*
		* Metodi usati per salvataggio e recupero struttura di backup in memoria
		*/
		void Backup_StatoStart();
		bool DownLoad_StatoStart();

		/*
		* Metodi usati per salvataggio e recupero valori nominali associati alla struttura struct
		*/
		void default_struttura_flussopeso();
		bool salva_rapporto_flussopeso(int m_line);
		float controllo_flussopeso(long step, word deltapeso, float reference);

		/**
		* Metodi manager di gestione
		*/
		
		void SIFRALed_Manager();
		bool SIFRASacca_Manager();
		bool SIFRAManual_Manager(int line);
		bool SIFRARampe_Manager( byte m_line, byte m_load);
		bool SIFRASample_Manager();
		bool SIFRAError_Manager();	
		void SIFRABreak_Manager();
		void SIFRAWeightControlRestart(char __typeOfFilling);
		//void SIFRAWeightControlRestartService();
		bool SIFRAStop_Manager();
		byte SIFRAAirIn_Manager();
		bool SIFRART_Manager(int line);
		bool SIFRAST_Manager(int line);
		bool SIFRAService_Manager();
		bool SIFRATestEMC_Manager();
		bool SIFRATestLed_Manager();				// usata per debug scheda luci
		bool SIFRATest_Pompe(int m_line);
		bool SIFRATest_FlussoPompa(int m_line);
		bool SIFRATest_OpenMoreEVs();
		bool SIFRARisciacquo_Manager(int line);
		bool SIFRACalibr_Manager();
		bool SIFRATestPinch();
		int SIFRASpeedPump_Manager( byte m_load);

		void SIFRASetScalesPhase(byte __startEnd, byte __phase);

		void SIFRASetScalesNotCalib();

		/*
		* Metodo usato per identificare quale comando di start è stato ricevuto dalla tastiera
		*/
		byte Decode_StartMsg();

		/*
		* Metodi usati alla ricezione di uno start per identificare le vie da eseguire
		*/
		void Set_START_Erogazione(int i);
		int ViaDaEseguire();
		byte Find_Num_of_Enabled_Lines(byte command);
		int Find_Enable_line(byte command);

		/*
		* Metodi usati per controllo errori durante l'erogazione
		*/
		bool check_conditions(int line);
		byte check_volume_error(int line);
		byte controllo_non_cala(int line, int delta);
		byte controllo_vuoto(int line);		

		/*
		* Metodi usati per test EMC
		*/
		void UpdateDebugStruct();
		void Encoder_counter();
		void EncoderPumpsControl();
		void Set_weight_threshold();
		void LoadControl();

		/**
		* Da attivare solamente per test, non in release
		*/
		void VerificaESD_Damage();

		/*
		* Non utilizzata, mantenuta per eventuale necessità conteggio step motori
		*/
		void read_encoder_motors(int m_mot);
			
	private:

		/*
		* Metodo usato per resettare i valori di calibrazione memorizzati per le celle di carico
		*/
		bool SIFRACalibResetData();
		
		/**
		Pointer to the class that manages the communication with the PC.
		*/
		SIFRAProtocol *m_SIFRAProtocol;
		/**
		Pointer to the instance of class that manages the power voltage samples.
		*/
		VoltageHwChanManager <word, _PWR_VOLTAGE_BUFFER_LENGTH_> *m_powerVoltageChan;
		/**
		Pointer to the instance of class that manages the weight samples.
		*/
		GenericHwChanManager <dword, _WEIGHT_BUFFER_LENGTH_> *m_weightChan;
		/**
		Pointer to the instance of class that manages the generic samples.
		*/
		GenericHwChanManager <word, _GENERIC_BUFFER_LENGTH_> *m_genericChan;
		
		/**
		Lean array to read the Weight samples from the class ImpHwChanManager
		*/
		dword m_WeightSample[_MAX_LOAD_CHAN_];

		ChannelsBackupParam m_control_cal_values[_MAX_LOAD_CHAN_];		// variabili usate per leggere i valori di calibrazione salvati in EEPROM
		

		/*
		* variabili usate per misurazione pesi e loro validazione
		*/
		WEIGHT_SAMPLE Via_Sample;		// template usato per salvataggio pesi letti dalle celle di carico
		float m_loadsystem2;					// variabile usata per il calcolo del peso letto sulla rastrelliera
		bool m_startLoadingSamples;
		bool m_dato_valido;

		/*
		* variabili associate alla pompa (velocità ed encoder)
		*/
		int m_this_step;
		int m_old_step;
		int m_waiting_step;
		byte m_vel;							// valore velocità pompa

		/*
		* variabili usata per identificare la configurazione hw della scheda CPU
		*/
		byte m_this_CPU_is;

		/*
		* variabili utilizzate per identificare la fase di controllo pesi in corso
		*/
		byte m_scales_phase;					// variabile usata per identificare un'eventuale fase di verifica bilance o calibrazione in corso
		bool m_verifica_bilance;				// flag necessario per segnalazione errore di peso instabile
		
		// variabili utilizzate nell'algoritmo di rilevazione aria
		word 	m_enc_prev_val;
		word 	m_enc_val;
		word  	m_enc_old_val;
		byte		m_count_aria[NUM_MAX_LINE];		//contatore dei giri dell'AirInManager in cui è stata rilevata aria
		float 	m_volume_aria[NUM_MAX_LINE];	// totalizzatore del volume di aria complessiva necessaria per scatenare l'allarme aria

		/*
		* Variabili gestione pesi
		*/
		long m_peso_attuale;					// peso attuale restituito dalla SampleManager, deve essere long perché a culla vuota il peso può essere negativo, mentre con tutti i flaconi il peso può superare i 4 Kg (40000 dg)

		/*
		* Variabile usata in macchina a stati test pinze e test pompe
		*/
		long m_peso1;

		/*
		* Variabile usata in FlashLed_hw_error
		*/
		bool m_blink;

		/*
		* Variabile usata in macchina a stati LedManager
		*/
		bool m_state_led;

		/*
		* Variabile di gestione riempimenti
		*/
		word m_enc_value;				// variabile usata per memorizzare numero passi encoder del motore
		bool m_restart;					// variabile usata per identificare uno stop durante una sequenza e valutare se eseguire il controllo prodotto
		bool m_RT_con_calib;				// variabile usata per stabilire se occorre eseguire il calcolo del rapporto flusso peso
		byte m_controllo_errore_volume;	// variabile usata per distinguere eventuale errore di prodotto da errore di flusso
		bool m_controllo_volume_encoder;	// variabile usata per controllare se devo fare il controllo volume quando arrivo nello stato StatoControlloEncoder
		bool m_test_pinch_alarm;			// usata per identificare un doppio giro in avanti della pompa durante il test pinze
		long m_giri_pinch_test;		// usata in test pinze per salvare in caso di doppio mezzo giro in avanti della pompa i giri fatti nel primo mezzo giro
		word m_peso_non_cala;				// variabile usata in algoritmo non cala
		word m_peso_non_cala_precedente;	// variabile usata in algoritmo non cala
		float m_volMaxST;					// variabile usata per interrompere svuotamento linee se ho già superato un volume massimo stabilito
		byte m_debug_EV;					// variabile usata in test EV
		word m_peso_parziale;				// variabile usata in riempimento per salvare il peso parziale erogato dallo Start
		bool m_openEV_pich_fail;

		/*
		* Variabili gestione macchine a stati
		*/
		byte m_air_manager[NUM_MAX_LINE];		// variabile usata in macchina a stati AirInManager
		byte m_stato_check_blocco;				// variabile usata per gestire la ESD_Damage
		byte m_test_led;							// variabile usata per verificare funzionamento comunicazione con scheda luci

		/**
		* Variabili usate per test EMC
		*/
		word m_weight_real_time;
		word m_init_weight;
		long m_init_load;
		long m_weight_lower_limit;
		long m_weight_upper_limit;
		word m_enc_value_control;
		word m_enc_movement;
		word m_enc_upper_limit;
		word m_enc_lower_limit;
		bool m_first_enc_control;

		/*
		* Variabili usate per test pompe
		*/
		dword m_startime;
		float m_time_total;

		/*
		* Variabile usata per misurazione tempo necessaria in algoritmo calc_next_weight
		*/
		float m_timeControl;

		/*
		* Variabile usata per memorizzare il livello di immunità al peso instabile fissato da tastiera
		*/
		byte m_weightUnstableLevel;

		/*
		* Variabile usata per test pinze di service
		*/
		byte m_testEVstate;

		/*
		* Variabili usate per identificare variazioni peso in pausa erogazione
		*/
		long m_peso_stop;
		short m_variazione_peso;
		bool m_void_source;
		word m_calo_volume_stop;

		/*
		* Variabile usata per test apertura di più EV contemporaneamente
		*/
		byte m_test_open_more_ev;
		
		/*
		* Timer utilizzati in riempimento o service e resettabili
		*/
		DecTimer m_restore_stability;			// timer gestito in autonomia da macchina a stati SampleManager
		DecTimer m_timerVerificaCalo;			// timer usato in algoritmo non cala
		DecTimer m_timerEndSeq;				// timer usato per gestione termine riempimento
		DecTimer m_timer_enc_measure;		// timer usato per controllo encoder in service
		DecTimer BackUpTimer;				// timer usato per salvataggio dati di backup
		DecTimer m_check_block_pump;		// timer usato per check blocco pompa
		DecTimer m_verify_enc_timer;			// time usato per test EMC
			
		/*
		* Timer da non resettare
		*/
		DecTimer m_timerLed;
		DecTimer m_rampa_timer;
		DecTimer m_watchDogTimer;
		DecTimer m_TimerChkHw;
		DecTimer m_TimerStopAll;
		DecTimer m_TimerAllarmeAria[NUM_MAX_LINE];
		DecTimer m_resetLedDrivers;			// ogni x secondi devo resettare il pannello luci per farlo ripigliare dopo stop per ESD
		DecTimer m_timer_openEV;
		DecTimer m_timerPinchTest;
		DecTimer m_timerServiceEv;
		DecTimer m_timer_test_fw_run;
		DecTimer m_test_led_timer;					// timer usato per debug scheda luci
		DecTimer m_timer_calib;						// timer usato in CalibrManager
		DecTimer m_timer_fineErog;
		DecTimer m_timer_flowPump;
		DecTimer m_timer_RT;
		DecTimer m_timer_test_nextWeight;
		DecTimer m_timer_enter_loader;				// timer usato all'ingresso in loader per aggiornamento firmware
		DecTimer m_timer_moreEVopened;				// timer usato in procedura usata per apertura di più pinze contemporaneamente
		DecTimer m_TimerMaxWeight;
		DecTimer m_timerControlCover;
		DecTimer m_timer_control_restart;

		/*
		* Variabili di debug aggiunte
		*/
		word m_num_sample_mgr;
		
};
