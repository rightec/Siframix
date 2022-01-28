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

// definizioni utili al modulo e al protocollo
#define _CONV_IN_DECIMI 	10.0	// converto in decimi di grado
#define _CONV_X_PROTOCOLLO 	5.0	// converto in decimi di grado, con risoluzione +/-0.1g, cioè 0.2g
#define _OFFSET_SU_ZERO	10000  // sommo 20g per scongiurare il peso mostrato negativo

/**
* Lunghezza pacchetti trasmessi a modulo M3000.
* Per la composizione ed i dati contenuti nei pacchetti vedi le funzioni sendSIFRAStart, sendSIFRAStatus, sendSIFRAInfo, sendSIFRADebug e sendAcknowledge.
*/
#define _PKT_START_LENGTH		37
#define _PKT_STATUS_LENGTH	80
#define _PKT_INFO_LENGTH		31
#define _PKT_DEBUG_LENGTH		78
#define _PKT_DEBUG_VIE_1_3_LENGTH		72
#define _PKT_DEBUG_VIE_4_5_LENGTH		78
#define _PKT_DEBUG_START_LENGTH		45
#define _PKT_DEBUG_RAM_LENGTH		81
#define _PKT_ACK_LENGTH		3

/*
* I dati utili per gestire le formule sono 34, dai 37 ricevuti si escludono il nodo (M3100), il tipo di pacchetto (START, 0x0B) e la checksum finale.
*/
#define _PKT_START_DATA_M3100		34

/*
* Il dato utile per gestire il reset dei pesi massimi è uno solo
*/
#define _PKT_RST_MAX_WEIGHT_M3100		1

/*
* I dati utili per gestire la fase bilance in corso sono due
*/
#define _PKT_SCALES_PHASE_LEN_M3100		2

//---------------------------------------------------------------------------------------
//------------------------------ SIFRAProtocol ---------------------------------------------
//---------------------------------------------------------------------------------------

enum SIFRA_HwErrors_t
{
	SIFRAUart_Hw_NoError 			= 0x00,
	SIFRAUart_Hw_ReceptionError 	= 0x01,
	SIFRAUart_Hw_ParityError    		= 0x02,
	SIFRAUart_Hw_FrameError 		= 0x04, 
	SIFRAUart_Hw_OverrunError 		= 0x08,
	SIFRAUart_Hw_NoDevice			= 0x10,
	SIFRAUart_Hw_ErrorUnknown		= 0x20,
};

enum
{
	SIFRAMsg_noMsg = 0,
	SIFRAMsg_StopAll,			// il master impone lo stop a tutti i moduli
	SIFRAMsg_setStart,
	SIFRAMsg_statusAsked,
	SIFRAMsg_infoAsked,
	SIFRAMsg_debugAsked,
	SIFRAMsg_debugVie_1_3_Asked,
	SIFRAMsg_debugVie_4_5_Asked,
	SIFRAMsg_debugStartAsked,
	SIFRAMsg_debugRamAsked,
	SIFRAMsg_newLines,
	SIFRAMsg_maxWeightReset,
	SIFRAMsg_restartControl,
	SIFRAMsg_scalesPhase,
	SIFRAMsg_scalesNotCalib,
	SIFRAMsg_setZeroCalib,
	SIFRAMsg_setOffset1,
	SIFRAMsg_setOffset2,
	SIFRAMsg_setOffset3,
	SIFRAMsg_setOffset4,
	SIFRAMsg_setOffset5,
	SIFRAMsg_setGain1,
	SIFRAMsg_setGain2,
	SIFRAMsg_setGain3,
	SIFRAMsg_setGain4,
	SIFRAMsg_setGain5,
	SIFRAMsg_resetCalib,
	SIFRAMsg_jumpToLoader,
	E_SIFRAMsg_loopApplication,
	            
	// errori di comunicazione legati all'hardware
	SIFRAMsg_HwError,

	// errori di comunicazione legati al protocollo
	SIFRAMsg_timoutRx,
	SIFRAMsg_timoutTx,
	SIFRAMsg_opcodeUnknown,
	SIFRAMsg_protocolStateUnknown,
	SIFRAMsg_checkSumError,
	SIFRAMsg_transmitBufferOverflowError,
	SIFRAMsg_receptionBufferOverflowError,
	SIFRAMsg_dataReceivedBufferOverflow,

	// errori non identificati
	SIFRAMsg_unknownError
};

typedef enum
{
	SIFRA_SET_START_M3100 = 0x0B,		// seguito da 24 byte
	SIFRA_GET_STATUS_M3100 = 0x0C,			// richiesta info stato
	SIFRA_GET_INFO_M3100 = 0x17,				// richiesta info sistema
	SIFRA_GET_DEBUG_M3100 = 0x12,
	SIFRA_NEW_LINES_M3100 = 0x1A,
	SIFRA_RESET_MAX_WEIGHT = 0x2B,
	SIFRA_RESTART_CONTROL = 0x30,
	SIFRA_GET_SCALES_STATE_M3100 = 0x40,
	SIFRA_GET_CELLS_NOT_CALIB = 0x50,
	SIFRA_GET_DEBUG_VIE_1_3_M3100 = 0x2C,
	SIFRA_GET_DEBUG_VIE_4_5_M3100 = 0x2D,
	SIFRA_GET_DEBUG_START_M3100 = 0x2E,
	SIFRA_GET_DEBUG_RAM_M3100 = 0x2F,

	SIFRA_SETZERO_CELL1 = 0x01,	// calibr. zero cella 1
	SIFRA_SETZERO_CELL2 = 0x02,	// calibr. zero cella 2
	SIFRA_SETZERO_CELL3 = 0x03,	// calibr. zero cella 3
	SIFRA_SETZERO_CELL4 = 0x04,	// calibr. zero cella 4
	SIFRA_SETZERO_CELL5 = 0x05,	// calibr. zero cella 5
	SIFRA_SET_CAL_CELL1 = 0x06,			// calibr. gain cella 1 
	SIFRA_SET_CAL_CELL2 = 0x07,			// calibr. gain cella 2
	SIFRA_SET_CAL_CELL3 = 0x08,
	SIFRA_SET_CAL_CELL4 = 0x09,
	SIFRA_SET_CAL_CELL5 = 0x0A,
	SIFRA_RESET_CALIB_PARAMS = 0x1C,	// reset dei valori di calibrazione salvati in EEPROM
	SIFRA_ZERO_CELLS_M3100 = 0x1B,	// imponi zero alle celle del modulo M3100
	SIFRA_SET_STOP_ALL = 0x00,			// stop generale
	SIFRA_M3000_STOP = 0xA0,
	SIFRA_JUMP_TO_LOADER = 0xA5,		// SALTA in loader per riprogrammazione
	E_SIFRA_LOOP_CMD		= 0XAB		// entra in un ciclo infinito, usato per aggiornamento firmware di un altro modulo per evitare interferenze sulla rete 485
	}SIFRACmd_FromPcToBoard;

enum errorStatus_1_t
{
	NO_ERRORS 									= 0x0000,	// situaizone di reset
	ERR_LETTURA_ZERO 							= 0x0001,	// si verifica in calibrazione
	ERR_LETTURA_2Kg 							= 0x0002,	// si verifica in calibrazione		
	ERR_RILEV_ARIA 							= 0x0004,	// sollo sulla via abilitata negli stati start di sacca e riempitubi: bloccante in start, solo segnalazione in riempitubi
	ERR_RILEV_VUOTO 							= 0x0008,	// solo sulla via abilitata negli stati start di sacca e riempitubi, bloccante in entrambi
	ERR_PESOINSTABILE 							= 0x0010,	// sempre attivo, segnalazione ma bloccante solo sulla via abilitata in start			
	ERR_NON_CALA	 							= 0x0020,
	ERR_PESO_MASSIMO							= 0x0040,
	ERR_CELLA_ROTTA							= 0x0080,
	ERR_PRODOTTO								= 0x0200,
	ERR_IMP_TO_FLOW							= 0x0400,	// il valore calcolato di imp_to_flow per quella pompa si discosta troppo dal valore nominale
	ERR_POMPA_NON_GIRA 						= 0x0800,	// è stato rilevato che la pompa non gira
	ERR_VAR_PESI								= 0x1000,	// usato per controllo stabilità pesi durante test EMC
	ERR_VAR_ENCODER							= 0x2000,	// usato per controllo stabilità valori encoder durante test EMC
	
	ERR_RST_STOP 								= ERR_CELLA_ROTTA | ERR_PRODOTTO,		// resetto gli errori non critici e che al riavvio se ancora presenti vengono rigenerati
	ERR_END_FORMULA 							= ERR_CELLA_ROTTA,						// resetto tutti gli errori al termine di una formula esclusi quelli critici
};

enum errorStatus_2_t
{
	E_ERROR_NONE					= 0x0000,
	ERR_COVER_APERTA				= 0x0001,		// errore sportello copertura rulli aperto
	ERR_COMUN_INT					= 0x0002,		// errore mancata comunicazione oltre il tempo del watchdog
	ERR_EEPROM					= 0x0004,
	ERR_RAM_TAMPONATA			= 0x0008,
	ERR_SYSTEM					= 0x0010,

	E_ERRORS_RST_STOP			= ERR_COMUN_INT | ERR_EEPROM | ERR_RAM_TAMPONATA,
	
	ERR_SYSTEM_NEGATO			= 0xFFEF
};

enum errorStop_t
{
	E_ERR_STOP_NONE						= 0x0000,
	E_ERR_STOP_HAND_MOVE				= 0x0001,
	E_ERR_STOP_HAND_MOVE_FAIL			= 0x0002,		// errore mancata comunicazione oltre il tempo del watchdog
	E_ERR_STOP_WEIGHT_DECREASE			= 0x0004,
	E_ERR_STOP_EXCESSIVE_DECREASE		= 0x0008,
	E_ERR_STOP_WEIGHT_INCREASE			= 0x0010,
	E_ERR_STOP_BAG_CHANGED				= 0x0020,
	E_ERR_STOP_MAYBE_BAG_CHANGED		= 0x0040
};

enum error_log_t
{
	E_LOG_ERROR_NONE = 0,
	E_ERR_CALIB_MGR_STATE,
	E_ERR_PESO_INIT_RT_FASE_2,
	E_ERR_PESO_INIT_RT_ADD,
	E_ERR_RIPR_SEQ_BK,
	E_ERR_RIPR_SEQ_NUM_LINES,
	ERR_CASE_CODA_SAMPLE_MNGR,
	ERR_CASE_SAMPLE_MNGR,
	ERR_CASE_SAVE_BK_DATA,
	ERR_CASE_READ_BK_DATA,
	ERR_CASE_SAVE_RESTART_DATA,				// 10
	ERR_CASE_READ_RESTART_DATA,
	ERR_CASE_SAVE_ENC_DATA,
	ERR_CASE_READ_ENC_DATA,
	ERR_CASE_DECODE_MSG_STATUS,
	ERR_CASE_PKT_TEST_ERROR,
	ERR_CASE_READ_ADC_VALUES,
	ERR_CASE_FACTORY_CALIB_OFFSET_OUT,
	ERR_CASE_FACTORY_CALIB_OFFSET_LOW,
	ERR_CASE_CALIB_OFFSET_OUT,
	ERR_CASE_FACTORY_CALIB_GAIN_OUT,		// 20
	ERR_CASE_FACTORY_CALIB_GAIN_LOW,
	ERR_CASE_CALIB_GAIN_OUT,
	ERR_CASE_CALIB_GAIN_LOW,
	ERR_CASE_STATE_MGR,
	ERR_CASE_SACCA_MGR,
	ERR_CASE_MANUAL_MGR,
	ERR_CASE_RT_MGR,
	ERR_CASE_ST_MGR,
	ERR_CASE_SAMPLE_MGR,
	ERR_CASE_AIR_MGR,
	ERR_CASE_SERVICE_MGR,
	ERR_CASE_TEST_POMPE_MGR,
	ERR_CASE_TEST_FLUSSO_POMPE_MGR,
	ERR_CASE_DECODE_START_MSG,
	ERR_CASE_MSG_RECEIVED,
	ERR_CASE_OPCODE_UNKNOWN,
	E_ERR_CASE_RESET_CALIB_FAIL,
	E_ERR_CASE_CARICA_START,
	E_ERR_CASE_LINE_ST,
	E_ERR_CASE_MANUAL_EROG
};

/*
* Struttura dati associata ad ogni via.
*
* 26 byte per via (i DecTimer sono stati esclusi ed i float vengono passati con 2 byte e non 4)
*
*/
struct LINES_STATUS
{
	bool abilitazione;
	bool da_eseguire;
	bool eseguita;
	bool firstErogDone;
	bool rifinituraDone;
	bool restart_bag;
	bool stato_led;
	byte stato_luci;
	byte fase_luci;
	int vel;
	word peso_da_erogare;	// peso obiettivo
	word peso_erogato;
	word peso_gia_erogato;
	int peso_stop;
	int variazione_peso_stop;
	word tara;				// tara sacca su via
	float peso_spec ;			// peso specifico passato da tastiera con 3 cifre decimali (valori compresi fra 0,900 e 1,300)
	float flusso_pompa;		// volume erogato calcolato in funzione dei giri pompa, usato per controllare un eventuale errore prodotto
	DecTimer timer_led;		// timer che non deve essere mai disattivata, pena il non corretto azionamento dei led della scheda luci
	DecTimer timer_check_block_pump;
	DecTimer timer_rampa;
	DecTimer timerVerificaCalo;
};
typedef struct LINES_STATUS LINES_Status;

/*
* Struttura dati associata ai comandi di start ricevuti dalla tastiera.
*
* 33 byte
*
*/
struct STRUCTSTARTCMD
{
	word function;			// comando ricevuto
	byte support;			// vie da abilitare
	word peso_linea[ NUM_MAX_LINE];	// peso target
	word tara_linea[ NUM_MAX_LINE];	// tara
	word peso_spec[ NUM_MAX_LINE];	// peso specifico
};

struct max_weight_error_rst_t
{
	byte lines;
};

struct restart_control_cmd_t
{
	byte type_of_formula;
};

struct structScalesPhaseCmd_t
{
	byte init_end;
	byte type_of_phase;
};

struct structScalesNotCalibCmd_t
{
	byte scales_not_calib;
};

/*
* Struttura dati associata ad ogni via.
*
* 39 byte
*
*/
struct STRUCTSTATUSCMD
{
	bool led_aria_en;						// attivazione della segnalazione aria sui led della scheda luci
	bool air_block_en[ NUM_MAX_LINE];		// attivazione del blocco motore per aria
	bool fillingOnCourse;				// indicazione rimepimento in corso (motore e allarmi attivi)
	byte state;
	byte prevState;
	byte phase;
	byte status;							// stato del sistema
	byte flagsErrorLine;					// segnalazione errore bloccante rilevato
	word statusChan[_MAX_LOAD_CHAN_];	// variabile usata per registrare la tipologia di errore occorso
	word errors;							// maschera a 8 bit di errori aggiuntivi generici e non specifici di una via
	byte log_error;						// variabile usata per monitorare comportamenti anomali o inattesi del firmware
	byte error_stop[_MAX_LOAD_CHAN_];
	word commPacketsRx;
	word commErrorHw;
	word commErrorProtocol;
	word commErrorUnknown;
	bool cover_open;						// segnala lo stato della cover rulli
};

/*
* Struttura dati associata a test EMC
*
* 75 byte
*
*/
struct STRUCTDEBUGCMD
{
	word		peso_init[NUM_MAX_LINE];
	int		peso_real_time[NUM_MAX_LINE];
	word		enc_value[NUM_MAX_LINE];
	word		enc_value_control[NUM_MAX_LINE];
	word		enc_movement[NUM_MAX_LINE];
	word		enc_upper_limit[NUM_MAX_LINE];
	word		enc_lower_limit[NUM_MAX_LINE];
	byte		air_state[NUM_MAX_LINE];
};

/*
* Struttura dati associata a test EMC
*
* 7 byte
*
*/
struct machineStatus_t
{
	byte calibState;						// variabile usata per gestire la macchina a stati della calibrazione celle di carico
	byte rtState;					// variabile usata per gestire le macchine a stati di riempimento linee, svuotamento linee e riempimento manuale
	byte saccaState;
	byte manualState;
	byte driverState;				// variabile usata per gestione led in SIFRALed_Manager
	byte ledState;					// variabile usata per gestire i diversi lampeggii dei led
	byte serviceState;					// variabile usata per gestire le macchine a stati di service nel test di service, flusso pompe e flusso
};

/*
* Struttura dati per backup
*
* 29 byte
*
*/
typedef struct
{
	byte start_cmd;
	byte vie_da_eseguire;
	byte vie_eseguite;
	byte num_lines;
	bool abilitazione[NUM_MAX_LINE];
	word peso_erogato[NUM_MAX_LINE];
	int peso_stop[NUM_MAX_LINE];
} BK_struct;

/*
* Struttura dati motori
*
* 20 byte (i float vengono spediti come 2 byte)
*
*/
typedef struct {
	float tolleranza[_DIM_PAR_ENC];
	float param_encoder[_DIM_PAR_ENC];
} FP_STRUCT;

/*
* Struttura dati linea e restart
*
* 2 byte
*
*/
typedef struct
{
	byte tipo_linea;
	byte restart_bk;
} Restart_t;

/**
* System status states
*/

enum COMMANDSTATUS
{
	STATO_IDLE = 0x01,
	STATO_ERRORE = 0x02,
	STATO_CALIBRAZIONE_FABBRICA = 0x04,
	STATO_STOP = 0x08,
	STATO_START = 0x10,
	STATO_FINE_SEQ = 0x20,
	STATO_ST = 0x40,
	STATO_CALIBRAZIONE_UTENTE = 0x80			
};

/**
Class to manage the serial communication between the board HRC-7000 and the PC.

When a packet is decoded, a message is pushed in the messages' fifo as well as an error occured. 

The protocol packets are builded as follow:
- (A) ID del modulo  - 1 byte
- (B) OPCODE	  - 1 byte
- (C) DATA        - x bytes
- (D) CHECKSUM = (B)+(C) in complemento a 1  - 1 byte (LSB)

Data length can be also 0, in this case the next data received will be the checksum.

The uart communication specifications are the follows:
- Baud rate: 19200
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
		int sendSIFRADebug();
		int sendSIFRADebugVie_1_3();
		int sendSIFRADebugVie_4_5();
		int sendSIFRADebugStart();
		int sendSIFRADebugRam();
		int SIFRA_decodeStartCmd(byte *cmd_byte_buffer );
		int SIFRA_decodeResetMaxWeight(byte *cmd_byte_buffer );
		int SIFRA_decodeRestartControlCmd( byte *cmd_byte_buffer);
		int SIFRA_decodeScalesPhaseCmd( byte *cmd_byte_buffer);
		int SIFRA_decodeScalesNotCalibCmd( byte *cmd_byte_buffer);
		
		/**
		@return the number of bytes in the transmission buffer.
		*/
		int GetNumBytesInTransmissionBuffer(){return stdUartProtocolAbstraction::GetNumBytesInTransmissionBuffer();};
		int sendAcknowledge();
		long sendSIFRALoadSamples(dword samples, int chan);
		int sendSIFRALoadSamples(dword *loadsystem);

		virtual int getMsg();

		int SIFRA_set_Stop_All();
		int SIFRA_getStatusCmdDecode();
		int SIFRA_setStart();
		int SIFRA_getInfoCmdDecode();
		int SIFRA_getDebugCmdDecode();
		int SIFRA_getDebugVie_1_3_CmdDecode();
		int SIFRA_getDebugVie_4_5_CmdDecode();
		int SIFRA_getDebugStartCmdDecode();
		int SIFRA_getDebugRamCmdDecode();
		int SIFRA_setLineToIdentify();
		int SIFRA_resetMaxWeightError();
		int SIFRA_restartControl();
		int SIFRA_setScalesPhase();
		int SIFRA_setScalesNotCalib();
		int SIFRA_setZeroCell1();
		int SIFRA_setZeroCell2();
		int SIFRA_setZeroCell3();
		int SIFRA_setZeroCell4();
		int SIFRA_setZeroCell5();
		int SIFRA_setGainCell1();
		int SIFRA_setGainCell2();
		int SIFRA_setGainCell3();
		int SIFRA_setGainCell4();
		int SIFRA_setGainCell5();
		int SIFRA_setResetCalib();
		int SIFRA_setJumpToLoader();
		int SIFRA_setLoopApplication();
		int SIFRA_getCalLoadCmdDecode();
		int SIFRA_getCalCell1CmdDecode();
		int SIFRA_getCalCell2CmdDecode();
		int SIFRA_getCalCell3CmdDecode();
		int SIFRA_getCalCell4CmdDecode();
		int SIFRA_getCalCell5CmdDecode();
		int SIFRA_zeroLoadCmdDecode();

		byte getLastHwError(){return m_lastHwError;};
		void rstLastHwError(){m_lastHwError = SIFRAUart_Hw_NoError;};
		void setLocalNODEID(byte i){ SIFRAProtocol::setLocalNodeID(i);};
		byte getLocalNODEID(){ return SIFRAProtocol::m_localNodeID;}
		void setEnabledLine(byte i){m_EnabledLine = i;};
		byte getEnabledLine(){return m_EnabledLine;};
		void setNumOfEnabledLines(int i){m_NumOfEnabledLines = i;};
		int getNumOfEnabledLines(){return m_NumOfEnabledLines;};

		int getLoadChan(byte __chan){return loadchan[__chan];};

		bool m_show_adcvalue;
		
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
		int loadchan[ _MAX_LOAD_CHAN_ ];
		byte m_lastRemoteNodeID;
		byte m_localNodeID;
		byte last_opCode;
		byte m_lastHwError;		
		byte m_EnabledLine;
		int m_NumOfEnabledLines;
};

#endif
