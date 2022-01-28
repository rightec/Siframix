/**
@file		protocolSIFRA.h
@brief		Class to manage siframix protocol.

			This class manage the communication protocol through this module and the master module M3000 :
			- receives and decodes messages from the UART 
			- manages the answers to the messages
			- .....
			
@author		Fregni Francesco
@date		18/01/2011
@version	01.00
*/

#ifndef __PROTOCOL_SIFRA_H__
#define __PROTOCOL_SIFRA_H__


//#include "global.h"
#include "protocolStdUart.h"

//#include "sound.h"
#include "pwm.h"
#include "channels.h"
#include "adc.h"

// definizioni utli al modulo e al protocollo
#define _CONV_IN_DECIMI 	10.0	// converto in decimi di grado, con risoluzione +/-0.1g, cioè 0.2g
#define _OFFSET_SU_ZERO	10000 // sommo 20g per scongiurare il peso mostrato negativo

/**
* Lunghezza pacchetti trasmessi a modulo M3000.
* Per la composizione ed i dati contenuti nei pacchetti vedi le funzioni sendSIFRAStart, sendSIFRAStatus, sendSIFRAInfo e sendAcknowledge.
*/
#define _PKT_START_LENGTH		55
#define _PKT_STATUS_LENGTH	67
#define _PKT_INFO_LENGTH		31
#define _PKT_ACK_LENGTH		3

/**
* Lunghezza pacchetti con dati di debug trasmessi a modulo M3000.
* Per la composizione ed i dati contenuti nei pacchetti vedi le funzioni dedicate.
*/
#define _PKT_DBG_VIA_LENGTH				35
#define _PKT_DBG_START_LENGTH			54
#define _PKT_DBG_STATUS_LENGTH			53
#define _PKT_DBG_BACKUP_RAM_LENGTH		57

/*
* I dati utili per gestire le formule sono 34, dai 37 ricevuti si escludono il nodo (M3100), il tipo di pacchetto (START, 0x0B) e la checksum finale.
*/
#define _PKT_START_DATA_M3300		52

#define _PKT_WEIGHT_DATA_M3300	1
#define _PKT_SCALES_DATA_M3300	2


/*
* Struttura dati associata ad ogni via del modulo M3300.
*
* 22 byte per via escluso il DecTimer.
*
*/
struct LINES_STATUS
{
	bool abilitazione;		// linea abilita: on/off EV
	bool da_eseguire;
	bool eseguita;
	bool stato_led;
	byte stato_luci;
	byte fase_luci;
	word peso_da_erogare;	// peso obiettivo
	word capacita;			// capacità attuale
	word peso_erogato;
	word peso_gia_erogato;
	word peso_iniziale;
	word peso_finale;
	float peso_spec ;			// peso specifico passato da tastiera con 3 cifre decimali (valori compresi fra 0,900 e 1,300)
	DecTimer timer_led;
};
typedef struct LINES_STATUS LINES_Status;

/*
* Struttura dati associata al comando di start ricevuto.
*
* 51 byte
*
*/
struct STRUCTSTARTCMD
{
	word function;
	byte support;
	word peso_spec[NUM_MAX_LINE];	// peso specifico passato da tastiera intero (valori compresi fra 90 e 130)
	word peso_linea[NUM_MAX_LINE];
	word cap_linea[NUM_MAX_LINE];	// tara nel caso del M3100 
};

/*
* Struttura dati associata allo stato del modulo M3300.
*
* 21 byte.
*
*/
struct STRUCTSTATUSCMD
{	
	bool air_block_en;		// attivazione del blocco motore per aria
	bool m_FillingOnCourse;
	byte status;
	byte flagsErrorLine;		// segnalazione errore bloccante rilevato
	byte tot_enabled_lines;
	int next_line;
	/*
	statusChan è un vettore nel caso del modulo M3300 di 2 word.
	_ADC1_ e _ADC2_ fanno parte di un enum: _ADC1_ corrisponde a 0 mentre _ADC2_ corrisponde a 1.
	La prima word (statusChan[_ADC1_]) memorizza nei due byte che la costituiscono i vari errori che possono presentarsi sul  modulo ed elencati nell'enum ERRORSTATUS
	riportato sotto.
	La seconda word (statusChan[_ADC2_]) memorizza invece le vie (per l'M3300 sono 8) che hanno preentato gli errori di aria o prodotto.
	In particolare il primo byte della word più significativa (8 bit più a sinistra) ha settato a 1 le vie che hanno presentato errore prodotto: ad esempio se il primo byte è 01001001
	l'errore prodotto è stato rilevato sulle vie 8, 11 e 14 del modulo M3300 (le vie vanno da 8 a 15).
	Il secondo byte invece (8 bit più a destra) ha settato a 1 le vie che hanno presentato errore aria: ad esempio se il primo byte è 00100100 l'errore prodotto è stato rilevato
	sulle vie 10 e 13.
	*/
	byte statusAirSensors;
	word statusChan[_MAX_LOAD_CHAN_];
	word SIFRAComPacketsRx;
	word SIFRAComHwErrors;
	word SIFRAComProtocolErrors;
	word SIFRAComUnknownErrors;
	byte error_stop;
	short variazione_peso;
	byte test_pinch_done;
	byte error_monitor;
	byte state;
	byte prevState;
	byte phase;
};

/**
* Struttura contenente gli stati delle varie macchine a stati
*
* 9 byte
*
*/
struct structMachineState_t
{	
	byte sampleState;					// variabile usata in macchina a stati SampleManager per rilevazione peso instabile
	byte calibState;					// variabile usata in macchina a stati calibrazione
	byte saccaState;					// variabile usata in SaccaManager
	byte rtState;						// variabile usata in RTManager e RisciacquoManager
	byte stState;						// variabile usata in STManager e ManualManager
	byte pinchState;					// variabile usata in macchina a stati test pinze
	byte ledState;					// variabile usata per gestire la macchina a stati associata ai led della scheda luci
	byte driverState;					// variabile usata per stabilire lo stato di idle pompe da quello di erogazione
	byte serviceState;					// variabile usata in macchina a stati ServiceManager, TestEMC_Manager e TestPompe_Manager
	byte nonCalaState;				// variabile usata in macchina a stati algoritmo non cala
};

/**
* Struttura usata solamente per rendere disponibili alla protocolSifra variabili della CPU_Manager da inviare alla tastiera per debug
*
* 38 byte
*
*/
struct structStatusDebug_t
{	
	byte sifraStatus;
	byte sifraPrevStatus;
	byte sifraPhase;

	int peso_current_time_int;
	word peso_current_time_word;
	word sampleMngr_cycles;

	/*
	* Variabili da monitorare in SaccaManager
	*/
	byte sacca_manager_state;
	word peso_parziale;

	/*
	* Variabili da monitorare per test EMC
	*/
	word		enc_value;
	word		enc_value_control;
	word		enc_movement;
	word		enc_up_limit;
	word		enc_low_limit;
	word		weight;
	word		init_weight;
	byte		air_state[NUM_MAX_LINE];
	byte		cover_pump;
	byte		error_control;
};

/*
* Struttura dati associata al backup di un'erogazione in corso.
*
* 49 byte
*
*/
typedef struct
{
	int nextLine;
	byte num_vie_da_eseguire;
	byte vie_da_eseguire;
	byte vie_eseguite;
	long peso_stop;
	bool abilitazione[NUM_MAX_LINE];
	word peso_erogato[NUM_MAX_LINE];
	word capacita[NUM_MAX_LINE];
} BK_struct;

/*
* Struttura dati salvata in RAM tamponata.
*
* 9 byte
*
*/
typedef struct
{
	float tolleranza;
	float param_encoder;
	byte restart;
} FP_STRUCT;

struct structWeightCmd_t
{
	byte sensibility;
};

struct structRestartFormulaCmd_t
{
	byte type_of_formula;
};

struct structScalesPhaseCmd_t
{
	byte init_end;
	byte type_of_phase;
};

enum
{
	SIFRAMsg_noMsg = 0,
	SIFRAMsg_StopAll,
	SIFRAMsg_setStart,
	SIFRAMsg_statusAsked,
	SIFRAMsg_infoAsked,
	SIFRAMsg_debugViaAsked,
	SIFRAMsg_debugStartAsked,
	SIFRAMsg_debugStatusAsked,
	SIFRAMsg_debugRamAsked,
	SIFRAMsg_SetWeightSensibility,
	SIFRAMsg_restartWeightCoverControl,
	SIFRAMsg_restartFormula,
	SIFRAMsg_scalesPhase,
	SIFRAMsg_scalesNotCalib,
	SIFRAMsg_setZeroVerification,
	SIFRAMsg_setZeroCalib,
	SIFRAMsg_setGain8,
	SIFRAMsg_setGain15,
	SIFRAMsg_resetCalib,
	SIFRAMsg_jumpToLoader,
	SIFRAMsg_resetApplication,
	SIFRAMsg_startAcquisition,
	SIFRAMsg_stopAcquisition,

	/*
	* Errori legati al protocollo seriale
	*/
	SIFRAMsg_timoutRx,
	SIFRAMsg_timoutTx,
	SIFRAMsg_opcodeUnknown,
	SIFRAMsg_stateProtocolUnknown,
	SIFRAMsg_checkSumError,
	SIFRAMsg_transmitBufferOverflowError,
	SIFRAMsg_receptionBufferOverflowError,
	SIFRAMsg_dataReceivedBufferOverflow,

	/*
	* Errori legati all'hardware
	*/
	SIFRAMsg_HwError,

	/*
	* Errori sconosciuti
	*/
	SIFRAMsg_unknownError
};

enum SIFRA_HwErrors
{
	SIFRAUart_Hw_NoError 			= 0x00,
	SIFRAUart_Hw_ReceptionError 	= 0x01,
	SIFRAUart_Hw_ParityError    		= 0x02,
	SIFRAUart_Hw_FrameError 		= 0x04, 
	SIFRAUart_Hw_OverrunError 		= 0x08,
	SIFRAUart_Hw_NoDevice			= 0x10,
	SIFRAUart_Hw_ErrorUnknown		= 0x20,
};

enum sifraCmd_t
{
	SIFRA_SET_STOP_ALL = 0x00,			// stop generale

	SIFRA_SET_START_M3300 = 0x10,
	SIFRA_GET_STATUS_M3300 = 0x11,			// richiesta info stato
	SIFRA_GET_INFO_M3300 = 0x18,				// richiesta info sistema
	SIFRA_SET_WEIGHT_SENSIBILITY_M3300 = 0x20,		// richiesta settaggio filtro pesi
	SIFRA_ERROR_RESTART_CONTROL = 0x2B,
	SIFRA_GET_DEBUG_VIA_M3300	= 0x30,		// richiesta debug struttura via
	SIFRA_GET_DEBUG_START_M3300	= 0x31,		// richiesta debug struttura start
	SIFRA_GET_DEBUG_STATUS_M3300	= 0x32,		// richiesta debug struttura status
	SIFRA_GET_DEBUG_BACKUP_RAM_M3300	= 0x33,		// richiesta debug struttura ram tamponata

	SIFRA_GET_SCALES_STATE_M3300 = 0x40,		// comando ricevuto dalla tastiera per sapere quando sono in verifica bilance o calibrazione

	SIFRA_GET_SCALES_NOT_CALIB_M3300 = 0x50,		// comando ricevuto dalla tastiera per identificare vie scalibrate

	SIFRA_SETZERO_VERIFICATION = 0x1C,	// calibrazione di zero in verifica bilance
	SIFRA_RESET_CALIB_PARAMS = 0x0C, 		// reset dei valori di calibrazione salvati in EEPROM
	SIFRA_SETZERO_CELL8_CELL15 = 0x0D,	// calibr. zero cella 6-H20 
	SIFRA_SET_CAL_CELL8 = 0x0E,			// calibr. gain cella 6
	SIFRA_SET_CAL_CELL15 = 0x0F,			// calibr. gain cella H20
	SIFRA_JUMP_TO_LOADER = 0xA5,			// salta in loader per riprogrammazione
	SIFRA_M3000_STOP		= 0xA0,			// stop ricevuto dalla tastiera
	SIFRA_RESTART_FORMULA_M3300 = 0x2C	// comando ricevuto alla ripresa di una formula
};

enum ERRORSTATUS
{
	NO_ERRORS 							= 0x0000,	// situazione di reset
	ERR_LETTURA_ZERO 					= 0x0001,	// si verifica in calibrazione
	ERR_LETTURA_2Kg 					= 0x0002,	// si verifica in calibrazione		
	ERR_GENERIC 						= 0x0004,	// solo sulla via abilitata negli stati start di sacca e riempitubi: bloccante in start, solo segnalazione in riempitubi
	ERR_RILEV_VUOTO 					= 0x0008,	// solo sulla via abilitata negli stati start di sacca e riempitubi, bloccante in entrambi
	ERR_PESOINSTABILE 					= 0x0010,	// sempre attivo, segnalazione ma bloccante solo sulla via abilitata in start			
	ERR_NON_CALA 						= 0x0020,	// non cala
	ERR_TESTPINZE_KO					= 0x0040,	// una o più elettrovalvole non chiude bene o il tubo è fuori dalla clamp
	ERR_POMPA_NON_GIRA 				= 0x0080,	// è stato rilevato che la pompa non gira quanto impostato
	ERR_PRODOTTO 						= 0x0200,	// mismatch fra volume erogato pesato e volume erogato calcolato come passi encoder dovuto all'errore di prodotto
	ERR_PESO_MASSIMO					= 0x0400,	// cella sovraccaricata
	ERR_CELLA_ROTTA_SX				= 0x0800,	// peso letto a zero o fondoscala, ponte rotto o cablaggio compromesso cella sx	
	ERR_CELLA_ROTTA_DX				= 0x1000,	// peso letto a zero o fondoscala, ponte rotto o cablaggio compromesso cella dx
	ERR_COVERAPERTA					= 0x2000,	// sportello pompa aperto
	ERR_FLUSSO							= 0x4000,	// errore per strozzatura del cavo
	ERR_COMUN_INT						= 0x8000,	// errore mancata comunicazione oltre il tempo del watchdog

	// errori negati o combinazioni di errori
	ERR_CELLA_ROTTA_SX_NEGATO		= 0xF7FF,	// negazione dell'errore ERR_CELLA_ROTTA_SX
	ERR_CELLA_ROTTA_DX_NEGATO		= 0xEFFF,	// negazione dell'errore ERR_CELLA_ROTTA_DX
	ERR_PESO_MASSIMO_NEGATO			= 0xFBFF,	// negazione dell'errore ERR_PESO_MASSIMO
	ERR_PESOINSTABILE_NEGATO			= 0xFFEF,	// negazione dell'errore ERR_PESOINSTABILE
	ERR_TESTPINZE_KO_NEGATO			= 0xFFBF,	// negazione dell'errore ERR_TESTPINZE_KO
	ERR_COVERAPERTA_NEGATO			= 0xDFFF,	// negazione dell'errore ERR_COVERAPERTA
	ERR_POMPA_MOSSA_A_MANO_NEG	= 0xFEFF,
	ERR_STOP							= ERR_CELLA_ROTTA_SX | ERR_CELLA_ROTTA_DX | ERR_COMUN_INT | ERR_PRODOTTO | ERR_FLUSSO,
	ERR_END_FORMULA					= ERR_CELLA_ROTTA_SX | ERR_CELLA_ROTTA_DX | ERR_COMUN_INT
};

enum errorStop_t
{
	E_ERR_STOP_NONE						= 0,
	E_ERR_STOP_PUMP_BY_HAND				= 0x0001,
	E_ERR_STOP_EXCESSIVE_DECREASE		= 0x0002,
	E_ERR_STOP_WEIGHT_DECREASE			= 0x0004,
	E_ERR_STOP_WEIGHT_INCREASE			= 0x0008,
	E_ERR_STOP_BAG_CHANGED				= 0x0010,
	E_ERR_STOP_MAYBE_BAG_CHANGED		= 0x0020
};

enum errorStatus_2_t
{
	E_ERROR_NONE = 0,
	E_ERROR_ENCODER,
	E_ERROR_WEIGHT,
	ERR_EEPROM,
	ERR_RAM_READ_BK,
	ERR_RAM_READ_ENC,		// 5
	ERR_RAM_WRITE_BK,
	ERR_RAM_WRITE_ENC,
	ERR_SYSTEM,
	ERR_MSG_NOT_EXPECTED,
	ERR_OPCODE_UNKNOWN,		//10
	ERR_START_MESSAGE,
	ERR_START_NOT_HANDLED,
	ERR_ESD_DAMAGE,
	ERR_RESTORE_DATA,
	ERR_READ_EEPROM,			// 15
	ERR_CASE_SACCA_MNGR,
	ERR_CASE_MANUAL_MNGR,
	ERR_CASE_RT_MNGR,
	ERR_CASE_RISCIACQUO_MNGR,
	ERR_CASE_ST_MNGR,			// 20
	ERR_CASE_SAMPLE_MNGR,
	ERR_CASE_AIR_MNGR,
	ERR_CASE_CALIB_MNGR,
	ERR_CASE_SERVICE_MNGR,
	ERR_CASE_EMC_MNGR,		// 25
	ERR_CASE_POMPE_MNGR,
	ERR_CASE_LED_MNGR,
	ERR_CASE_PINCH_TEST_MNGR,
	ERR_CASE_VOL_CONTROL_RT,
	ERR_CASE_PESO_INIT_SACCA,	// 30
	ERR_CASE_PESO_FINALE_SACCA,
	ERR_CASE_PESO_PARZIALE_SACCA,
	ERR_CASE_CODA_SAMPLE_MNGR,
	ERR_MSG_VIA_DEBUG,
	ERR_CASE_VUOTO,				// 35
	ERR_CASE_STATO_EROGAZIONE_SACCA,
	ERR_CASE_STATO_ENCODER_SACCA,
	ERR_CASE_TEST_DEBUG,
	ERR_CASE_PESO_FINALE_RT_MNGR,
	ERR_CASE_PESO_EROGATO_RT_MNGR,	// 40
	ERR_CASE_PESO_DA_EROGARE_SACCA_MNGR,
	ERR_CASE_PESO_FINALE_SACCA_MNGR,
	ERR_CASE_PESO_PARZIALE_MANUAL_MNGR,
	ERR_CASE_CHIUSURA_MANUAL_MNGR,
	ERR_CASE_PESO_PARZIALE_RINSE_MNGR,	// 45
	ERR_CASE_CHIUSURA_RINSE_MNGR,
	ERR_CASE_TEST_FLOW_PUMP,
	ERR_CASE_VARIAZIONE_PESO,
	ERR_CASE_TOT_EN_LINES,
	ERR_CASE_SET_STABILITY_LEVEL,			// 50
	ERR_CASE_NON_CALA,
	ERR_CASE_FACTORY_CALIB_OFFSET_OUT_CELL_8,
	ERR_CASE_FACTORY_CALIB_OFFSET_OUT_CELL_15,
	ERR_CASE_FACTORY_CALIB_OFFSET_LOW,
	ERR_CASE_CALIB_OFFSET_OUT_CELL_8,				// 55
	ERR_CASE_CALIB_OFFSET_OUT_CELL_15,
	ERR_CASE_FACTORY_CALIB_GAIN_SX_OUT_CELL_8,
	ERR_CASE_FACTORY_CALIB_GAIN_SX_OUT_CELL_15,
	ERR_CASE_CALIB_GAIN_SX_OUT_CELL_8,
	ERR_CASE_CALIB_GAIN_SX_OUT_CELL_15,				// 60
	ERR_CASE_FACTORY_CALIB_GAIN_DX_OUT_CELL_8,
	ERR_CASE_FACTORY_CALIB_GAIN_DX_OUT_CELL_15,
	ERR_CASE_CALIB_GAIN_DX_OUT_CELL_8,
	ERR_CASE_CALIB_GAIN_DX_OUT_CELL_15,
	ERR_CASE_READ_ADC_VALUES,						// 65
	ERR_CASE_TEST_OPEN_MORE_EV,
	ERR_RESET_CALIB_DATA_FAIL,
	ERR_CASE_CALC_PUMP_NOT_MOVE,
	ERR_CASE_FACTORY_CALIB_DIVIDE_BY_ZERO,
	ERR_CASE_DOWNLOAD_STATE,
	ERR_CASE_START_FAILED,
	ERR_CASE_NUM
};

enum COMMANDSTATUS
{
	STATO_IDLE = 0x0001,
	STATO_ERRORE = 0x0002,
	STATO_CALIBRAZIONE_FABBRICA = 0x0004,
	STATO_STOP = 0x0008,
	STATO_START = 0x0010,
	STATO_FINE_SEQ = 0x0020,
	STATO_VERIFICA_BILANCE = 0x0040,
	STATO_CALIBRAZIONE_UTENTE = 0x0080
};

/**
Class to manage the serial communication between the board HRC-7000 and the PC.

When a packet is decode a messages is pushed in the messages' fifo as well as an error occured. 
The messages have to be popped from the board manager using the propriety AcqHrProtocol::getMsg().

The protocol packets are builded as follow:
- (A) STX = '\\n'  - byte
- (B) OPCODE	  - byte	
- (C) DATA LENGTH - byte 
- (D) DATA        - DATA LENGTH bytes
- (E) CHECKSUM = (B)+(C)+(D)  - byte

Data length can be also 0, in this case the next data received will be the checksum.
Data length is the length of the field data.

The uart communication specifications are the follows:
- Baud rate: 230400
- Data bits: 8
- Stop bits: 1
- Parity: None
- Flow Control: None
*/

class SIFRAProtocol: protected stdUartProtocolAbstraction
{
	public:
		
		SIFRAProtocol(UARTDevice device = UART1, int dim = 500);
		~SIFRAProtocol();
		
		void Manager();

		int sendSIFRAStart();
		int sendSIFRAStatus();
		int sendSIFRAInfo();
		int sendAcknowledge();
		int SIFRA_decodeStartCmd( byte *cmd_byte_buffer );
		int SIFRA_decodeWeightSensibilityCmd( byte *cmd_byte_buffer );
		int SIFRA_decodeRestartFormulaCmd( byte *cmd_byte_buffer);
		int SIFRA_decodeScalesPhaseCmd( byte *cmd_byte_buffer);
		int sendSIFRAViaDebug();
		bool sendSIFRAStartDebug();
		bool sendSIFRAStatusDebug();
		bool sendSIFRARamDebug();
		
		/**
		@return the number of bytes in the transmission buffer.
		*/
		int GetNumBytesInTransmissionBuffer(){return stdUartProtocolAbstraction::GetNumBytesInTransmissionBuffer();};
		
		long getLoadChan(){return total_weight;};
		long sendSIFRALoadSamples(float loadsystem);

		virtual int getMsg();

		int SIFRA_set_Stop_All();
		int SIFRA_getStatusCmdDecode();
		int SIFRA_setStart();
		int SIFRA_getInfoCmdDecode();
		int SIFRA_getDebugViaCmdDecode();
		int SIFRA_getDebugStartCmdDecode();
		int SIFRA_getDebugStatusCmdDecode();
		int SIFRA_getDebugRamCmdDecode();
		int SIFRA_setWeightSensibility();
		int SIFRA_restartWeightCoverControl();
		int SIFRA_restartFormula();
		int SIFRA_setScalesPhase();
		int SIFRA_setScalesNotCalib();
		int SIFRA_setZeroVerification();
		int SIFRA_setResetCalib();
		int SIFRA_setZeroCell8_Cell15();
		int SIFRA_setGainCell8();
		int SIFRA_setGainCell15();	
		int SIFRA_setJumpToLoader();
		int jumpToLoaderHandler();
		int resetApplicationHandler();

		byte getLastHwError(){return m_lastHwError;};
		void rstLastHwError(){m_lastHwError = SIFRAUart_Hw_NoError;};
		
		void setLocalNODEID(byte i){ SIFRAProtocol::setLocalNodeID(i);};
		byte getLocalNODEID(){ return SIFRAProtocol::m_localNodeID;}

		bool m_show_adcvalue;
		bool m_show_velpompe;		
		
	protected:	
		static bool SIFRADataAnalyse(unsigned short dataS, stdUartProtocolAbstraction *protocol);
		virtual int sendMsg(int i);		
		bool checkOpCode(byte data, int NodeID, int &numDataToReceive);
		bool checksum(byte receivedChk, byte opCode, byte nodeID);
		
		void setLocalNodeID(byte node);
		byte getLocalNodeID();
		void setLastRemoteNodeID(byte node);
		byte getLastRemoteNodeID();		
		
	private:		
		CSmallRingBuf<int , 20> m_message;
		word loadchan[ _MAX_LOAD_CHAN_ ];
		long total_weight;
		byte m_lastRemoteNodeID;
		byte m_localNodeID;
		byte last_opCode;
		byte m_lastHwError;

		word m_numStopPkts;
		word m_numStartPkts;

};

#endif
