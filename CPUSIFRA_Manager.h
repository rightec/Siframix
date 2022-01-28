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

/* Definizione maschera di bit per segnali encoder (word)*/
#define  _MASK_16BIT_	0x0000FFFF

/*
* Definizioni per identificazione linea montata
*/
#define _ADULTI_	0			// settaggio linea adulti
#define _PEDIATRICA_	1		// settaggio linea pediatrica

/*
* Soglie utilizzate in SampleManager per verifica stabilità del peso
* Riferimento alla velocità massima delle pompe di 1000ml/min -> 0,28g/sample
*/
#define SOGLIA_ALTA_STAB	30			// 3  grammi
#define SOGLIA_BASSA_STAB	40			// 4 grammi

/*
* Soglie utilizzate in AirInManager
*/
#define _VOLUME_MAX			5.0		// 0,5ml di volume massimo tollerabile
#define _TEMPO_DI_ARIA			100		// numero di giri

/*
* Soglie variazioni peso in pausa formula
*/
#define _MIN_STOP_WEIGHT_VARIATION_				5		// 0,5 grammi di variazione peso ammessa
#define _MAX_STOP_WEIGHT_DECREASE_				70		// 7 grammi di calo peso ammessi
#define _STOP_WEIGHT_VARIATION_BAG_CHANGE_		500		// una variazione di almeno 50 grammi la associo ad un cambio sacca o flacone

/*
* Soglia utilizzata per rilevazione movimento pompa a mano in stop
*/
#define _SOGLIA_ENC_				500		// circa un sesto di giro, corrispondente a 0,3 grammi per linea pediatrica e 0,8 grammi per linea adulti

/*
* Utilizzati in Verifica_encoder_motori per rilevare eventuali blocchi della pompa
*/
#define _SOGLIA_ENC_MIN			10
#define _SOGLIA_ENC_NON_GIRA		80		// valore calcolato empiricamente							
#define _PERC_SOGLIA_ENCODER		0.2		//Valore percentuale di errore ammesso sul calcolo teorico degli impulsi encoder in base alla velocità della pompa

/*
* Soglia per segnalazione errore prodotto
*/
#define _PERC_SOGLIA_ERR_PRODOTTO	0.05		// valore percentuale di errore ammesso per generare errore prodotto

/*
* Velocità pompa impostata all'inizio del riempimento linee per ciascuna via
*/
#define _VOL_LOW_VEL_RT			20

/*
* Margine espresso in grammi usato per rilevare in tempo l'errore di sacca vuota
*/
#define MARGINE_SU_VUOTO			10

/*
* Valore minimo di volume da erogare per riprendere una via in pausa
*/
#define _VOL_RIPRESA_SEQ_MIN		4

/*
* Soglia peso minimo ammesso per calibrazione guadagno di ogni culla
*/
#define _MIN_PESO_GAIN_CALIB		15000	// 1500 g

/*
* Soglia valore ADC minimo per escludere danneggiamento o disconnessione cella di carico
*/
#define _TH_VAL_ADC_CELL_DIS_		2000	// valore ADC minimo sotto al quale identifico cella di carico scollegata o danneggiata

/*
* Soglia peso massimo ammesso per ogni culla
*/
#define PESO_MASSIMO		70000	// peso massimo ammissibile su M3100 = 7000 g

/* 65535 punti per 12Kg --> 0.183 g/punto */
/* delta adc di differenza accettabili in calibrazione delle celle di carico */
#define _DELTA_ADC_1_PERC_FS_			656			// l'1% del valore di fondo scala delle celle di carico (65535) è 656, variazione pari a 120 grammi
#define _DELTA_ADC_0Kg					(_DELTA_ADC_1_PERC_FS_ * 3)	// 3% del fondo scala
#define _DELTA_ADC_0Kg_DEF				(_DELTA_ADC_1_PERC_FS_ * 5)	// 5% del fondo scala

/* numero di giri di programma durante i quali l'adc non fornisce mai dati utili. Soglia usata per determinare la necessità del reset degli adc */
#define _COUNT_ADC		10000

/*
* Usate per calcolo dei pesi specifici
*/
#define _MILLE_				1000.0	// costante utilizzata per il calcolo del peso specifico
#define _PESO_SPEC_MIN		0.8
#define _PESO_SPEC_MAX		1.5

/*
* Frequenza campionamento
*/
#define _SAMPLE_FREQ 	30.0		// frequenza di campionamento effettivo degli ADC senza media mobile

/*
* Soglia usata in TEST EMC per verifica stabilità del peso nel periodo del test
*/
#define THRESHOLD_WEIGHT	20			// 2 g

/*
* Soglia usata in TEST EMC per verifica corretto conteggio passi encoder con pompe in movimento
*/
#define _ENC_STEP_EXPECTED			1650		// valore empirico
#define _PERC_ENC_CONTROL			0.50

// Gli offset riportati di seguito vengono sommati all'indirizzo 0x180000 corrispondente alla RAM tamponata mappata nella PLD (vedi define CS_TAMPRAM e RTS_CS)
#define OFS_RESTART_SET		0x0300	// posizione struttura restart in memoria
#define OFS_POINTS_SET			0x0200	// posizione struttura backup in memoria
#define OFFSET_VAL_ENC			0x0030	// posizione inizio struture degli encoder (5 vie con 4 float per via, 16byte per via, 80byte complessivamente, scritti solo a indirizzi pari fanno 160 byte)

/*
* Elenco timer
*/

/* Tempi gestione lampeggi led pannello, nelle varie situazioni */
#define _T_BLINK_START				800		// lampeggio all'avvio dela macchina, serve come check del corretto funzionamento
#define _T_BLINK_OTHER_ERR 		200		// errore peso instabile o vuoto
#define _T_BLINK_AIR_ERROR			800		// errore aria rilevata
#define _T_BLINK_STOP				500		// stop di via abilitata

/* Definizione tempi di gestione della rampa motori */
#define _T_RAMPA_RT_			50 		// tempo aggiornamento velocità motore in rampa accelerazione riempitubi
#define _T_RAMPA_SERVICE_		40		// tempo aggiornamento velocità motore in rampa stato service
#define _T_RAMPA_ST_			40		// tempo aggiornamento velocità motore in rampa modalità svuotamento linee
#define _T_RAMPA_MAN_			40		// tempo aggiornamento velocità motore in rampa modalità manuale

/*
* Timer utilizzati in algoritmo non cala
*/
#define T_NO_CALO 				3000
#define T_NO_CALO_FASE_2		4000

/*
* Timer usati in algoritmo aria
*/
#define _TIMER_READ_AIR		100		// timer per lettur aria che avviene quindi ogni 100msec
#define _TIME_READ_FAST_AIR	20  // timer per lettur aria che avviene quindi ogni 10msec
#define _TIMER_READ_LONG_AIR	1000  // timer lungo che lascia il tempo all'M3000 di leggere l'errore

#define WATCHDOG_TIME				7000	// 7 secondi, tempo prima che intervenga il blocco dei motori per comunicazione interrotta
#define _TIMEOUT_CHKHW			5000	// 5 secondi, verifica periodicamente se il valore letto dalle celle è indice di cella scollegata (valore di fondo scala) o rottura (valore nullo)
#define _TIMER_CALC_FLW			2000	// timer utilizzato per periodico aggiornamento del flusso misurato in base ai passi encoder fatti dai motori, usato per rilevazione errori prodotto
#define _100_mSEC_					100
#define _200_mSEC_					200
#define _330_mSEC_					330
#define _500_mSEC_					500
#define _800_mSEC_					800
#define _1_SEC_						1000
#define _2_SEC_						2000
#define _3_SEC_						3000
#define _4_SEC_						4000
#define _5_SEC_						5000
#define _10_SEC_						10000
#define _30_SEC_						30000

/**
* Communication error with the PC.
* This errors are raised from the PC communication protocol decode class.
*/
enum SIFRAComErrors
{
	SIFRAComError_noError = 0,
	SIFRAComError_timoutRx,
	SIFRAComError_timoutTx,
	SIFRAComError_opcodeUnknown,
	SIFRAComError_checkSumError,
	SIFRAComError_HwError,
	SIFRAComError_transmitBufferOverflowError,
	SIFRAComError_receptionBufferOverflowError,
	SIFRAComError_dataReceivedBufferOverflow,
	SIFRAComError_unknownError,		
};

enum vie_number_t
{
	E_VIA_1 = 0,
	E_VIA_2,
	E_VIA_3,
	E_VIA_4,
	E_VIA_5,
	E_VIA_NUM
};

/**
Board's states
*/
enum SIFRA_Status		// stati FSM principale che gestisce l'applicativo
{
	InitStatus = 0,			// inizializzazione sistema
	AttesaComando,			// stato di attesa comandi da tastiera, sistema inattivo
	StatoCalibrazione,		// calibrazione eseguibile da utente
	StatoRiempitubi,			// stato controllo alta velocità (allarme aria disab velocità fissa alta
	StatoSvuotatubi,			// stato di svuotamento manuale di una via (si vuole svuotare il tubo prima di smontare la linea)
	StatoManuale, 			// avvio manuale della pompa (svuotamento/riempimento manuale, sfiato
	StatoSacca, 				// qui si getsice il riempimento
	StatoErrore,				// stato di errore
	StopAllStatus,			// stato di allarme, ci si arriva per stop esterno
	StatoStop,
	PausaErogazione,			// stato inserito per identificare lo stato di attesa conclusione esecuzione formula
	//PausaFormulaService,		// stato inserito per identificare lo stato di attesa ripartenza di una formula di servizio
	RipresaMancanzaRete,	// stato necessario per distinguere la generazione degli errori in caso di riavvio del sistema
	StatoService,				// stato di service utile per i test di validazione o per le prove 
	StatoTestPompe,			// routine di test delle velocità pompe, per calcolo flussi, rapporti flusso peso...
	StatoTestFlussoPompe,	// routine di test del flusso pompe con movimentazione dei motori in base al numero di passi encoder
	StatoTestEmc,
	StatoTestLuci,			// stato usato per verificare il corretto azionamento dei led sulla scheda luci
	E_WAIT_ST_STABLE,
	E_WAIT_ENTER_LOADER,
	E_WAIT_ENTER_LOOP
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
	E_PHASE_ERROR,
	E_PHASE_SERVICE,
	E_PHASE_CALIBRATION,
	E_PHASE_SVUOTATUBI,
	E_PHASE_RESTART_NO_ALIM,
	E_PHASE_NUM
};

enum SACCA_STATUS	// stati FST che gestisce il riempimento in SACCA
{
	SaccaIdle = 0,
	ConfigurazioneSequenza,
	RipresaSequenza,
	LeggiTareVie,
	StartErogazioneVia,		// fase che da il via al riempimento
	StatoErogazione,		// stato riempimento sacca specifica per M3200 e M3300
	StartRifinituraErogazione,	// fase di fine riempimento dove le pompe sono controllate ad impulsi encoder
	RifinituraErogazione,
	ChiusuraErogazione,			// fine sacca
	FineErogazione				// chiusura sequenza start
};

/**
* States of stability verify routine
*/
enum StabilityStates
{
	init_stab = 0,
	stabile,
	forse_instabile,
	instabile,
	E_WAIT_STABILITY
};

/**
Calibration Status States
*/
enum CALIBSTATUS
{
	IdleCalib = 0,
	ReadZeroInBothLines,
	ReadZeroInThisLine,
	ReadLoadInThisLine,
};

/**
Manager fase riempitubi
*/
enum HIGHSPEEDSTATUS
{
	IdleRiempitubi,
	SetupRiempitubi,
	StartRiempitubi,
	ControlloFase1,
	ControlloFase2,
	E_ADD_EROG_RT,
	ChiusuraRiempitubi,
	E_END_RT
};

/**
Manager fase manuale
*/
enum MANUALSPEEDSTATUS
{
	IdleManuale = 0,
	StartManuale,
       ControlloManuale,
       ControlloManuale_Fine,
	ChiusuraManuale,
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
	E_LED_GREEN,
	E_LED_STOP,
	E_LED_NUM
};

enum LED_STATUS_MANAGER
{
	DISPLAY_INIT = 0,
	DISPLAY_READY,
	CONTROLLO_LUCE,
	BLINK_LEDS,
	BLINK_LEDS_HW_ERROR,
	idle,
	DISPLAY_RE_INIT_1,
	DISPLAY_RE_INIT_2,
	GO_TO_CONTROLL_DRIVER_STATUS
};

enum SERVICE_STATUS_MANAGER
{
	IdleService = 0,
	Start_pompe,
	Controllo_pompe,
	calcoli,
	uscita
};

enum test_emc_status
{
	E_EMC_NONE = 0,
	E_EMC_START_TEST,
	E_EMC_ENABLE_VIA_1,
	E_EMC_ENABLE_VIA_2,
	E_EMC_ENABLE_VIA_3,
	E_EMC_ENABLE_VIA_4,
	E_EMC_ENABLE_VIA_5,
	E_EMC_TEST_BEGIN,
	E_EMC_TEST_RUN,
	E_EMC_END_TEST,
	E_EMC_NUM
	
};

enum STATUS_MSG_START
{
	RESET_SISTEMA			= 0x81,	// abort + calibrazione default per re-inizializzare il sistema allo stato fabbrica
	INIT_SISTEMA			= 0x01,	// abort per inizializzare il sistema in condizione d'uso
	RESTART_SEQUENZA 		= 0x02,	// start  - ripresa sequenza sospesa
	NUOVA_SEQUENZA 		= 0x03,	// abort+start - start nuova sequenza
	E_TEST_PKT_ERR		= 0x05,
	E_RESET_RESTART		= 0x06,
	E_TEST_LED_CMD		= 0x08,
	USER_CALIBR 			= 0x10,	
	DEFAULT_CALIBR 		= 0x80,
	E_USR_CALIB_CMD		= 0x82,
	E_VERIFY_CALIB_VAL_CMD = 0x83,	// comando utilizzato per leggere in tempo reale i valori di calibrazione memorizzati nella EEPROM
	RIEMPITUBI 				= 0x22,	// manuale highspeed + start
	SVUOTATUBI				= 0x0A,	// svuotamento via manualmente
	MANUALE_CON_OBIETTIVO = 0x0B,	// manuale con obiettivo
	DEBUG_VEDI_ADC_VIE 	= 0x13,	// combinazione di ABORT+START+CALIBRAZIONE UTENTE
	DEBUG_SERVICE 			= 0x2B,	// abort+start+manuale+riempitubi = SERVICE 
	DEBUG_VEL_POMPE 		= 0x07,	// abort+start+tara per provare le velocità pompe
	TEST_FLUSSI_POMPE		= 0x17,	// abort+start+tara+calibrazione per provare flussi pompa al variare della velocità
	DEBUG_EMC				= 0x0F,	// abort+start+tara+manuale per provare gli encoder delle pompe
	RESET_ADC_ON_BOARD	= 0x1B	// abort+start+manuale+calibrazione
};

enum AIR_STATUS
{
	E_AIR_MGR_START = 0,	// la m.a.s. è inizializzata a questo stato dall'abort e dallo start (riempitubi o sacca)
	E_AIR_MGR_CONTROL,	// qui è la fase che verifica aria durante start e se c'è aria passa in alert
	E_AIR_MGR_ALERT,		// stato trasistorio per evitare letture spurie isolate (o bollicinetrascurabili), se l'aria è confermata, passa in attivo
	E_AIR_MGR_ACTIVE,		// qui si esegue l'integrazione della bolla: se supera il volume minimo passa in allarme
	E_AIR_MGR_ALARM,		// qui rimane finchè c'è aria, altrimenti prima va in reset per verificare 
	E_AIR_MGR_RESET,		// qui riverifica che sia veramente finita e in tal caso resetta lo stato e torna in contyrollo
};

enum CheckEsdDamageMachineState
{
	verifica_contatore = 0,
	contatore_overflow,
	verifica_watchdog_timer,
	verifica_contatore_dopo_reset_1,
	verifica_contatore_dopo_reset_2,
	reset_scheda
};

enum led_state_t
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

enum pump_state_t
{
	INIT_DRIVER = 0,
	CONTROLL_DRIVER,
	E_WAIT_DRIVER
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

typedef struct
{
	int this_sample;	// campione attuale
	int next_sample;	// valore stimato del prossimo campione
}ViaSample;
typedef CSmallRingBuf<ViaSample, 3> WEIGHT_SAMPLE;


/**
* Manages the machine state of board.
*
* @see VoltageHwChanManager
* @see CurrentHwChanManager
* @see GenericHwChanManager
* @see SifraProtocol
*/
class SIFRA_Manager
{
	public:
		
		SIFRA_Manager(SIFRAProtocol *SifraProtocol);
		~SIFRA_Manager();

		void Manager();

		/**
		Sets the class that manages the power voltage samples.
		@param v poiter to the class VoltageHwChanManager
		@see VoltageHwChanManager
		*/
		void setVoltageHwChanManager(VoltageHwChanManager <word, _PWR_VOLTAGE_BUFFER_LENGTH_> *v){ m_powerVoltageChan = v;};

		/**
		Sets the class that manages the weight samples.
		@param w poiter to the class GenericHwChanManager
		@see setWeightChan
		*/
		void setWeightChan(GenericHwChanManager <dword, _WEIGHT_BUFFER_LENGTH_> *w){ m_weightChan = w;};
		
		int SIFRALed_Manager();
		bool SIFRATestLed_Manager();
		bool SIFRASacca_Manager();
		bool SIFRARampe_Manager(int m_load, bool singola_via);
		void SIFRARampe_Manager_Test(int m_load);
		bool SIFRASample_Manager();
		bool SIFRAError_Manager();	
		void SIFRABreak_Manager();
		void SIFRAWeightControlRestart(char __typeOfFilling);
		bool SIFRACalib_Manager(int chan);
		void SIFRAAirIn_Manager();
		bool SIFRART_Manager(int line);
		bool SIFRAST_Manager(int line);
		bool SIFRAManual_Manager(int line);
		bool SIFRAStop_Manager();
		bool SIFRAService_Manager();
		bool SIFRATest_Pompe(int m_line);
		bool SIFRATest_FlussoPompe(int m_line);
		bool SIFRATest_Emc();
		void SIFRAFlow_manager();
		void SIFRAPhase_manager();

		/**
		Reads the hardware flags which indicate board hardware versione
		*/
		byte readHardwareVersion(){return (byte)(( cpld_reg_in >> 8) & 0x03);};

		/**
		Set a error occoursed in eeprom reading
		*/
		void setErrorEEpromReading();

		void SIFRA_resetStatusErrorStop();
		void SIFRA_resetStatusError();
				
	protected:
		//---- Messages Handler ---
		void SIFRAcheck_new_command();
		int SIFRAMsg_setStartHandler();
		int SIFRAMsg_statusAskedHandler(); 
		int SIFRAMsg_infoAskedHandler();
		int SIFRAMsg_debugAskedHandler();
		int SIFRAMsg_debugAskedVie_1_3_Handler();
		int SIFRAMsg_debugAskedVie_4_5_Handler();
		int SIFRAMsg_debugAskedStartHandler();
		int SIFRAMsg_debugAskedRamHandler();
        	int SIFRAMsg_startAcquisitionHandler();
		int SIFRAMsg_jumpToLoaderHandler();
		int SIFRAMsg_resetApplicationHandler();
		int SIFRAMsg_hwErrorHandler();
		int SIFRAMsg_protocolErrorHandler();
		int SIFRAMsg_unknownErrorHandler();

		/**
		* Sets board's hardware version, reading two bits of cpld
		*/
		bool SetHardwareVersion();

		/*
		* Metodo utilizzato per distinguere lampeggio led in caso di caricamento firmware non idoneo a configurazione switch della scheda CPU
		*/
		void FlashLed_hw_error(void);

		/*
		* Metodo utilizzati per identificare lo stato e la fase del sistema
		*/
		byte changeSIFRAstatus(byte newStatus);
		byte getSIFRAstatus();
		byte getSIFRAphase();

		/*
		* Metodo utilizzato per salvare i valori di riempimento parziali
		*/
		bool BackUp_StatoStart();

		/*
		* Metodo utilizzato per recuperare i valori di riempimento parziali
		*/
		bool Carica_StatoStart();

		/**
		* Metodi usati per reset delle variabili
		*/
		void SifraResetSystemParams();
		void SifraResetStatus();
		void SifraResetStartCmd();
		void SifraResetFillingData();
		void SifraResetViaStatus();
		void SifraResetBackup();
		void SifraResetStateMachine();
		void SifraStopTimers();
		void ResetMotorStepCounter();

		/**
		* Metodo usato per re inizializzare il sistema
		*/
		void DefaultSystemReset();

		/**
		* Metodo usato per impostare la fase di verifica bilance o calibrazione in corso
		*/
		void SIFRASetScalesPhase(byte __startEnd, byte __phase);

		void SIFRASetScalesNotCalibrated();

		/**
		* Metodo usati in fase di calibrazione
		*/
		bool readZeroInThisLine(int chan);
		bool readLoadInThisLine(int chan);

		/**
		* Metodi usati in algoritmo SampleManager
		*/
		int calc_next_theor_weight(int i);
		int calcola_variazione_peso(int t);

		/**
		* Metodi usati in algoritmo AirInManager
		*/
		void imposta_allarme_aria(int i);
		float calc_flusso_pompa(int m_line);		// metodo usato per misurare volume bolle d'aria

		/**
		* Metodi usati alla ricezione di un comando di Start
		*/
		byte Decode_StartMsg();
		int Find_Num_of_Enabled_Lines(byte command);
		int Find_Enable_line(byte command);

		/**
		* Metodi usati per controllo movimentazione motori
		*/
		void Verifica_encoder_motori();
		long encoder_updateCnt(int m_line);		// aggiorna il contatore dei giri della pompa puntata
		void setStatePump(int i, int command);
		int getStatePump(int i);

		/**
		* Metodi richiamati nella ErrorManager per controllare gli stati dello sportello e di errori nella lettura pesi
		*/
		bool Verifica_apertura_sportello();

		/**
		* Metodi usati per controllo led scheda luci
		*/
		void setta_led_pannello(int line, bool light);
		void reset_state_this_led(int line, bool state);
		void reset_state_leds(bool state);

		/**
		* Metodi usati per controllo errori vuoto e non cala
		*/
		byte controllo_errori_linea(int line, int delta);
		word calcolo_margine_tara(int line, word delta);

		/**
		* Metodi usati per controllo linea montata
		*/
		byte Verifica_linea_montata(int m_line, int peso_rif);

		void Aggiorna_pesi_erogati();
		void VerificaESD_Damage();
		bool Verifica_celle();
		bool Verifica_peso_massimo();
		void Setta_filtro_peso(int val);
		int SIFRASpeedPump_Manager( byte m_load);	
		void calcola_velocita_massime();

		/*
		* Metodi per caricamento parametri di flusso associati ad ogni pompa
		*/
		void default_struttura_flusso(int m_line);
		bool salva_rapporto_flusso(int m_line, int peso_rif, int peso_fin, byte linea);

		/*
		* Metodo per caricamento parametri di restart
		*/
		void default_struttura_restart();

		/*
		* Metodi utilizzati per salvare in ram tamponata i dati di backup aggiornati
		*/
		bool Read_Param_Backup();
		bool Write_Param_Backup();		
		bool Backup_Param_Encoder(FP_STRUCT *this_struct, int m_line);
		bool Carica_Param_Encoder(FP_STRUCT *this_struct, int m_line);
		bool Backup_Param_Restart(Restart_t *this_struct);
		bool Carica_Param_Restart(Restart_t *this_struct);
				
		float AdattaPesoSpecifico(word pes_spec);
		
		/**
		* Metodi aggiunti per debug e test EMC
		*/
		void UpdateDebugStruct();
		void Encoder_counter();
		void EncoderPumpsControl(byte __line);
		void LoadControl(byte __line);
		void Set_weight_threshold();
		
	
	private:

		/*
		* Metodo usato per resettare i valori di calibrazione memorizzati per le celle di carico
		*/
		bool SIFRACalibResetData(byte __line);
		
		/**
		* Pointer to the class that manages the communication with the PC.
		*/
		SIFRAProtocol *m_SIFRAProtocol;
		
		/**
		* Pointer to the instance of class that manages the power voltage samples.
		*/
		VoltageHwChanManager <word, _PWR_VOLTAGE_BUFFER_LENGTH_> *m_powerVoltageChan;
		
		/**
		* Pointer to the instance of class that manages the weight samples.
		*/
		GenericHwChanManager <dword, _WEIGHT_BUFFER_LENGTH_> *m_weightChan;
		
		/**
		* Pointer to the instance of class that manages the generic samples.
		*/
		GenericHwChanManager <word, _GENERIC_BUFFER_LENGTH_> *m_genericChan;

		byte m_this_CPU_is;

		// variabili necessarie per i calcoli di pesi, movimentazione dei motori, ecc.
		dword m_WeightSample[_MAX_LOAD_CHAN_];			// usata per lettura valori ADC e calcolo pesi
		WEIGHT_SAMPLE Via_Sample[_MAX_LOAD_CHAN_];		// usata per lettura valori ADC e calcolo pesi
		int vel_max[_NUM_PUMPS];								// usata per settare velocità pompe
		byte	m_count_aria[NUM_MAX_LINE];						//contatore dei giri di aria rilevata, usata nell'algoritmo AirInManager
		bool m_startLoadingSamples;							// usata per attivare campionamento ADC dei valori delle celle di carico
		bool m_dato_valido;									// usata per segnalare la presenza di un nuovo campione valido dagli ADC
		bool m_verifica_linea;								// usata per stabilire se occorre calcolare il flusso di una pompa in RT per stabilire la linea montata
		byte this_chan;										// usata in calibrazione per memorizzare la via su cui occorre effettuarla
		word m_prev_val[NUM_MAX_LINE];						// usata per memorizzare i valori encoder ad ogni giro di programma
		word m_timer_rampa;								// usata per settare i valori del timer di accelerazione delle pompe
		float m_volume_aria[NUM_MAX_LINE];						// totalizzatore del volume di aria complessiva
		dword m_old_step[NUM_MAX_LINE];						// passi encoder registrati al precedente giro di programma per ogni motore
		dword m_this_step[NUM_MAX_LINE];						// passi encoder registrati al presente giro di programma per ogni motore
		word m_waiting_step[NUM_MAX_LINE];					// passi encoder attesi per ogni motore
		bool m_state_led;									// variabile usata nell'algoritmo LedManager

		/*
		* Variabili utilizzate per identificare la fase di controllo pesi in corso
		*/
		bool m_verifica_bilance;
		byte m_scales_phase;

		/*
		* Variabili usate in BreakManager
		*/
		bool m_void_source[NUM_MAX_LINE];
		word m_peso_erog_by_hand[NUM_MAX_LINE];

		// variabili aggiunte per test EMC 2019
		word m_enc_value[NUM_MAX_LINE];
		word m_enc_movement[NUM_MAX_LINE];
		word m_enc_value_control[NUM_MAX_LINE];
		word m_enc_upper_limit[NUM_MAX_LINE];
		word m_enc_lower_limit[NUM_MAX_LINE];
		word m_vel_test_emc;
		word m_init_weight[NUM_MAX_LINE];		// variabile con peso già convertito per corretta visualizzazione da supervisore
		int m_init_load[NUM_MAX_LINE];			// variabile con peso misurato dal modulo pari a quello mostrato dal supervisore moltiplicato per 10 e usata per controllo stabilità del peso
		int m_weight_real_time[NUM_MAX_LINE];		// variabile con peso già convertito per corretta visualizzazione da supervisore
		long m_weight_lower_limit[NUM_MAX_LINE];
		long m_weight_upper_limit[NUM_MAX_LINE];
		bool m_first_enc_control[NUM_MAX_LINE];

		/**
		* Variabili gestione macchine a stati
		*/
		byte m_air_manager[NUM_MAX_LINE];			// variabile usata per gestire la macchina a stati del controllo aria
		byte stab_error[_MAX_LOAD_CHAN_];					// usata per memorizzare lo stato della macchina a stati per il peso instabile
		byte m_stato_check_blocco;				// variabile usata per controllo blocco lettura ADC, macchina a stati di VerificaESD_Damage
		byte m_stato_test_emc;					// variabile usata per gestire la macchina a stati nel test EMC
		byte m_test_led;							// variabile usata per verificare funzionamento comunicazione con scheda luci

		/**
		* Variabili usate all'interno delle macchine a stati e da resettare quando vado in AttesaComando
		*/
		byte m_num_enabled_lines;							// usata nei riempimenti per sapere quante vie devono essere attivate
		int m_enable_line;									// usata nei riempimenti come indice delle vie da azionare
		int m_valore_peso_stimato[_MAX_LOAD_CHAN_];		// usata nell'algoritmo per il peso instabile
		
		/**
		* Variabili usate per salvataggio pesi
		*/
		int m_peso_precedente[_NUM_PUMPS];
		int m_peso_attuale[_NUM_PUMPS];
		int m_peso_iniziale[_NUM_PUMPS];
		int m_peso_finale[_NUM_PUMPS];

		/*
		* Variabile usata per bloccare lo svuotamento linee dopo un volume svuotato presunto eccessivo
		*/
		float m_flussoPeso[NUM_MAX_LINE];

		ChannelsBackupParam m_control_cal_values[_MAX_LOAD_CHAN_];

		// timer da non stoppare
		DecTimer m_TimerChkHw[NUM_MAX_LINE];		// timer per verifica fondo scala celle
		DecTimer m_TimerMaxWeight[NUM_MAX_LINE];	// timer per verifica peso massimo sulle celle
		DecTimer m_TimeCalcFlow;						// timer usato per stima flussi erogati da pompe in base a passi encoder eseguiti
		DecTimer m_timerLed;							// timer gestione FSM led manager
		DecTimer m_timer_test_fw_run;				// timer di controllo dell'attività del firmware
		DecTimer m_stab_timer[_MAX_LOAD_CHAN_];		// timer usato nell'algoritmo di peso instabile
		DecTimer m_timer_board_reset;				// attivato in algoritmo VerificaESD_Damage
		DecTimer m_watchDogTimer;					// timer di controllo comunicazione attiva
		DecTimer m_timer_ultimo_peso;					// timer attivato in caso di ricezione stop da tastiera e disabilitato subito dopo nella StopManager
		DecTimer m_TimerStopAll;						// timer attivato in caso di ricezione stop da tastiera e disabilitato subito dopo nella StopManager
		DecTimer m_TimerAllarmeAria[NUM_MAX_LINE];	// timer utilizzato in AirInManager
		DecTimer m_resetLedDrivers;					// ogni x secondi devo resettare il pannello luci per farlo ripigliare dopo stop per ESD
		DecTimer m_test_led_timer;					// timer usato per debug scheda luci
		DecTimer m_enter_loader_timer;				// timer utilizzato per garantire ricezione risposta corretta della tastiera
		//DecTimer m_timer_control_restart[NUM_MAX_LINE];
		DecTimer m_timerControlCover;
		DecTimer m_timerSvuotamentoStop;

		// timer da stoppare quando chiudo una fase
		DecTimer m_timerEndSeq;						// timer usato in sacca, RT, manuale e service
		DecTimer	 m_emc_motor_timer;					// attivato solamente in procedura di service test EMC
		DecTimer m_verify_enc_timer;					// attivato solamente in procedura di service test EMC
		DecTimer m_timer_enc_measure;				// attivato solamente in procedura di service test flusso e test flusso pompe
		DecTimer BackUpTimer;						// usato per salvare in ram tamponata la struttura di backup
		
};
