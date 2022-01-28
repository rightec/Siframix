/**
@file		CPUSIFRA_Manager.cpp
@brief		CPUSIFRA states machine.

			This class manage the machine states of the device:
			- receives messages from the communication protocol decoding classes
			- manages the acquisition channels
			- .....

@author		Fregni Francesco
@date		18/01/2011
@version	01.00
*/

#include "CPUSIFRA_Manager.h"

void (*fp)(void);

extern LINES_Status Via[];
extern STRUCTSTARTCMD StartCmd;
extern max_weight_error_rst_t g_rstMaxWeight;
extern restart_control_cmd_t g_restartControlCmd;
extern structScalesPhaseCmd_t g_scalesPhaseCmd;
extern structScalesNotCalibCmd_t g_scalesNotCalibCmd;
extern STRUCTSTATUSCMD StatusCmd;
extern STRUCTDEBUGCMD DebugCmd;
extern machineStatus_t g_stateMachine;
extern BK_struct bk;
extern FP_STRUCT g_encStruct[NUM_MAX_LINE];
extern Restart_t g_restart;

extern word g_adc_real_value[ _MAX_LOAD_CHAN_ ];

extern ChannelsBackupParam Chan[];
extern int stato_pompa[];
extern ENCODER_Motor_Status	 MotorStatus[];

float param_encoder_nom[] = {65.0,170.0};		// parametro di flusso misurato empiricamente valido per ogni pompa, rappresenta gli step encoder necessari per erogara 0.1 ml di volume
int vol_controllo[] = {10, 6};					// volume in grammi per errore non cala, a seconda della linea
int vol_calcolo_enc[] = {200, 150};				// volume minimo in decimi di grammi di riempitubi accettabile per eseguire il calcolo del rapporto encoder grammi
int vol_PreTarget[] = {70, 25};					// volume di differenza dal target che fa fermare la rampa per proseguire a vel bassa costante
int vol_PreTargetManuale[] = {60, 30};			// volume di differenza dal target che fa rallenatre a scalino la pompa in manuale
//int vol_Frenata_RT[] = {80, 20};				// distanza dal target che impone la frenata rapida in RT quando siamo prossimi al target
int vol_Stop_RT[] = {40, 20};					// distanza dal target che impone lo stop in RT, perchè prossimissimi al target
int vol_ExtraVol_RT[] = {70, 50};				// volume extra massimo in più per provare ad aliminare eventuale aria rimasta a fine RT
int vol_svuotatubi[] = {6000.0, 3000.0};			// volume massimo in decimi di grammo da pompare in svuotatubi
int vol_margine[] = {7, 3};						// margine in decimi di grammo, usato come verifica target raggiunto
int vol_margine_RT[] = {8, 3};					// margine in decimi di grammo, usato come verifica target raggiunto in riempitubi
byte vol_soglia_err_prod[] = {150, 100};			// soglia minima per generazione errore prodotto
float vol_soglia_hand_move[] = {50.0, 25.0};		// soglia massima per forzare abbandono sacca in caso di movimento pompa a mano
float pendenza_frenata[] = {2.6, 6.5};			// pendenza della retta di frenata della rampa pompa
float m_tolleranza[] = {0.05, 0.15};				// i due valori di tolleranza usati per validare i volumi alla fine dei riempimenti

/**
* Class constructor.
* @param SIFRAProtocol class tha manage the communication protocol with PC
*/
SIFRA_Manager::SIFRA_Manager( SIFRAProtocol *SifraProtocol)
{
	byte i;
	byte result_EE_read;

	m_SIFRAProtocol = SifraProtocol;

	// resetto le variabili di fase e stato e gli errori di comunicazione
	StatusCmd.commErrorHw = 0;
	StatusCmd.commErrorProtocol = 0;
	StatusCmd.commErrorUnknown = 0;
	StatusCmd.state 			= InitStatus;
	StatusCmd.prevState 		= InitStatus;
	StatusCmd.phase			= E_PHASE_INIT;
	StatusCmd.status			= STATO_IDLE;
	m_startLoadingSamples		= False;

	DefaultSystemReset();						// Inizializzazione del sistema allo stato fabbrica

	/*
	* Se switch configurati diversamente dal firmware caricato il software entra in un ciclo infinito
	*/
	SetHardwareVersion();
	m_this_CPU_is = m_SIFRAProtocol->getLocalNODEID();
	if(m_this_CPU_is != M3100)		// firmware e set hardware non combaciano
	{
		while(1)					// si illuppa qui fino al riavvio
		{
			FlashLed_hw_error();	// getsione luci pannello e scheda
		}
	}

	/**
	* TODO: gestire correttamente salvataggio linea montata e utilizzo parametri per valutazione flussi
	*/
	m_verifica_linea = False;					// reset del parametro verifica linea, da aggiornare dopo il primo riempitubi
	m_verifica_bilance = False;

	result_EE_read = get_adc_param();
	switch(result_EE_read)
	{
		case E_READ_ERROR:
			setErrorEEpromReading();			// segnalare errore di lettura eeprom
			break;
		case E_READ_BK_FACTORY:
			/*
			* TODO: INSERIRE CONTROLLO SUI VALORI LETTI DALLA EEPROM
			*/
			break;
		case E_READ_NO_BK_FACTORY:
			break;
		default:
			StatusCmd.status = STATO_ERRORE;
			StatusCmd.errors = ERR_SYSTEM;
			StatusCmd.log_error = ERR_CASE_READ_ADC_VALUES;
			break;
	}
	
	if(!Carica_Param_Restart(&g_restart))		// carica dalla TAMP RAM i valori di restart
	{									// se torna false devo caricare i dati default
		default_struttura_restart();
		if(!Backup_Param_Restart(&g_restart))
		{
			//StatusCmd.flagsErrorLine = 0x01;
			//StatusCmd.errors = ERR_RAM_TAMPONATA;
			StatusCmd.log_error = ERR_CASE_SAVE_RESTART_DATA;
			//return;
		}
	}
	
	for(i = 0; i < NUM_MAX_LINE; i++)
	{
		if(!Carica_Param_Encoder(&g_encStruct[i], i))		// carica dalla TAMP RAM i valori di encoder
		{									// se torna false devo caricare i dati default
			default_struttura_flusso(i);		// setta con valori di default la struttura
			if(!Backup_Param_Encoder(&g_encStruct[i], i))
			{
				//StatusCmd.flagsErrorLine = 0x01;
				//StatusCmd.errors = ERR_RAM_TAMPONATA;
				StatusCmd.log_error = ERR_CASE_SAVE_ENC_DATA;
				//return;
			}
		}
		m_TimerAllarmeAria[i].Preset(_TIMER_READ_AIR);	// rilevamento aria temporizzato ogni 100msec
		Via[i].timer_led.Preset(_1_SEC_);		// inizializzazione dei timer necessaria per la corretta accensione dei led sulla scheda luci
		Via_Sample[i].clear();					// azzeramento coda dei campioni convertiti
	}

	g_stateMachine.driverState = INIT_DRIVER;			// variabile usata per controllo gestione led
	g_stateMachine.ledState = DISPLAY_INIT;			// variabile usata per gestire i diversi lampeggii dei led

	if(reset_4_block == 0xAA)					// ho ordinato il reset scheda causa blocco adc e pannello luci --> auto-reset
	{										// quindi deve eseguire il download subito senza aspettare il cmd abort da tastiera
											// e poi metto il sistema in errore: così la tastiera reagisce con uno stop
		reset_4_block = 0x55;					// resetto flag ripresa da auto-reset
		Carica_StatoStart();					// scarico il backup dello stato macchina prima dell'auto-reset
		for(i = 0; i < NUM_MAX_LINE; i++)		// devo settare lo stato di err instabile sulle vie abilitate e bloccate e ora riprese
		{
			if(Via[i].abilitazione == True)
			{
				Via[i].stato_led = ON;
				Via[i].fase_luci = 0;
				Via[i].stato_luci = ERR_INSTABILE;
				//StatusCmd.statusChan[i] |= ERR_PESOINSTABILE;	// set errore stabilità
			}
		}
		Setta_filtro_peso(_Fc_1Hz_);		// settaggio del filtro di media sul peso:  15Hz/15 = 1Hz
	}	
}

/**
Class destructor.
*/
SIFRA_Manager::~SIFRA_Manager()
{

}
/**
* Manages the machine state.
* It has to be called within the main cycle loop.
*/
void SIFRA_Manager::Manager()
{
	int line;

	UpdateDebugStruct();

	if(m_timer_test_fw_run.Match())
	{
		m_timer_test_fw_run.Preset(_30_SEC_);
		asm("nop");
	}

	m_SIFRAProtocol->Manager();					// si occupa della seriale di comunicazione, gestisce errori di comm e decodifica i comandi M3000

	//--- checking for SIFRAProtocol messages ----
	SIFRAcheck_new_command();					// analizza la coda dei messaggi gestiti dalla m_SIFRAProtocol->Manager() e verifica se ci fevono essere dei cambi stat

	m_dato_valido = SIFRASample_Manager();			// verifico la disponibilità di nuovi dati nelle fifo degli adc. se ci sono, converto in decimi di grammo e verifico la stabilità
	
	SIFRAFlow_manager();							// aggiorna i valori istantanei di flusso delle pompa
	SIFRAError_Manager();							// verifica le condizioni di errore della macchina CPU_MANAGER

	//---- SIFRAMIX state machine --------
	switch (getSIFRAstatus())
	{
		case AttesaComando:
			break;

		case InitStatus:
			SIFRAMsg_startAcquisitionHandler();
			changeSIFRAstatus(AttesaComando);
			break;

		case StatoSacca:
			if(SIFRASacca_Manager())				// gestisce il riempimento controllando le pompe
			{
				changeSIFRAstatus(AttesaComando);	// setta il nuovo stato
			}
			break;

		case StatoRiempitubi:
			if(SIFRART_Manager(m_enable_line))
			{
				changeSIFRAstatus(AttesaComando);
			}
			break;

		case StatoSvuotatubi:
			if(SIFRAST_Manager(m_enable_line))
			{
				SifraResetFillingData();
				SifraResetViaStatus();
				StatusCmd.status = STATO_STOP;
				changeSIFRAstatus(AttesaComando);
			}
			break;

		case StatoManuale:
			if(SIFRAManual_Manager(m_enable_line))	// gestisce il riempimento controllando le pompe
			{
			/*
				for(line=0;line<NUM_MAX_LINE;line++)
				{
					m_timer_control_restart[line].Preset(_2_SEC_);
				}
				*/
				changeSIFRAstatus(PausaErogazione);	// setta il nuovo stato
			}
			break;

		case StatoCalibrazione:
			if(m_dato_valido)						// se c'è una nuova tornata di dati validi (uno per ogni via)
			{
				m_dato_valido = False;
				if(SIFRACalib_Manager(this_chan))
				{
					reset_state_leds(OFF);
					changeSIFRAstatus(AttesaComando);
				}
				else
				{
					switchon_red_leds();
				}
			}
			break;

		case StatoErrore:
			break;

		case StopAllStatus:						// comando di stop generale, inteso come pausa
			// se c'è una o più vie abilitate allora qui la luce verde deve rimanere accesa, se invece c'è un errore in corso
			// deve lampeggiare la via in errore nel modo opportuno. In ogni caso se via abilitata deve andare in stato errore
			for(line = 0; line < NUM_MAX_LINE; line++)
			{
				setStatePump(line, ARRESTO);		// setta lo stato di arresto pompa, il pwm manager si occuperà del resto
				Via[line].timer_check_block_pump.Stop();
				Via[line].timerVerificaCalo.Stop();
			}
			StatusCmd.fillingOnCourse = False;
			SIFRA_resetStatusErrorStop();
			SifraStopTimers();
			if(g_restart.restart_bk == E_VALID_BACKUP)
			{
				m_TimerStopAll.Preset(_3_SEC_);		// setto un tempo massimo di attesa stabilizzazione
				changeSIFRAstatus(StatoStop);		// vado in stato errore ma solo per comodità, in realtà lo status è stop
			}
			else
			{
				if(StatusCmd.status == STATO_ST)
				{
					m_timerSvuotamentoStop.Preset(_3_SEC_);
					changeSIFRAstatus(E_WAIT_ST_STABLE);
					break;
				}
				for(line = 0; line < NUM_MAX_LINE; line++)
				{
					Via[line].peso_stop = m_peso_attuale[line];
				}
			 	changeSIFRAstatus(AttesaComando);
			}
			StatusCmd.status = STATO_STOP;
			break;

		case StatoStop:							// qui gestisce la fase di chiusura e salvatggio dello stato per poter riprendere alla ripartenza
			if(SIFRAStop_Manager() == True)
			{
			/*
				for(line=0;line<NUM_MAX_LINE;line++)
				{
					m_timer_control_restart[line].Preset(_2_SEC_);
				}
				*/
				BackUp_StatoStart();
				ResetMotorStepCounter();
				changeSIFRAstatus(PausaErogazione);
			}
			break;

		case PausaErogazione:
		case RipresaMancanzaRete:
			SIFRABreak_Manager();
			break;
/*
		case PausaFormulaService:
			SIFRABreak_ManagerService();
			break;
*/

		case StatoService:						// stato di service utile per i test di validazione o per le prove
			if(SIFRAService_Manager())
			{
				changeSIFRAstatus(AttesaComando);		// setta il nuovo stato
			}
			break;
		case StatoTestPompe:
			if(SIFRATest_Pompe(m_enable_line))
			{
				changeSIFRAstatus(AttesaComando);
			}
			break;
		case StatoTestFlussoPompe:
			if(SIFRATest_FlussoPompe(m_enable_line))
			{
				changeSIFRAstatus(AttesaComando);
			}
			break;
		case StatoTestEmc:
			SIFRATest_Emc();
			if(m_verify_enc_timer.Match())
			{
				m_verify_enc_timer.Preset(_330_mSEC_);
				Encoder_counter();
				for(line=0;line<NUM_MAX_LINE;line++)
				{
					EncoderPumpsControl(line);
				}
			}
			for(line=0;line<NUM_MAX_LINE;line++)
			{
				LoadControl(line);
			}
			break;
		case StatoTestLuci:
			if(SIFRATestLed_Manager())
			{
				m_test_led_timer.Stop();
				StatusCmd.status = STATO_STOP;
				changeSIFRAstatus(AttesaComando);
			}
			break;
		case E_WAIT_ST_STABLE:
			if(m_timerSvuotamentoStop.Match())
			{
				m_timerSvuotamentoStop.Stop();
				SifraResetFillingData();
				SifraResetViaStatus();
				StatusCmd.status = STATO_STOP;
				changeSIFRAstatus(AttesaComando);
			}
			break;
		case E_WAIT_ENTER_LOADER:
			if(m_enter_loader_timer.Match())
			{
				m_enter_loader_timer.Stop();
				SIFRAMsg_jumpToLoaderHandler();
			}
			break;
		case E_WAIT_ENTER_LOOP:
			PinLED0_LIFE_M = 0;
			cpld_pin_led = 0;
			while(1)
			{
			}
			break;
		default:
			StatusCmd.status = STATO_ERRORE;
			StatusCmd.errors = ERR_SYSTEM;
			StatusCmd.log_error = ERR_CASE_STATE_MGR;
			break;
	}

	SIFRALed_Manager();		// getsione luci pannello

	SIFRAPhase_manager();		// aggiornamento fase sistema

	if(m_watchDogTimer.Match())
	{
		StatusCmd.flagsErrorLine |= 0x01;	// setto un errore
		StatusCmd.errors |= ERR_COMUN_INT;	// comunicazione interrotta
	}
	
}

/**
Analizza la coda dei messaggi che che protocol_sifra ha inviato al sistema, per gestire dei comandi di cambio stato, o stop
*/
void SIFRA_Manager::SIFRAcheck_new_command()
{
	int msg;
	byte line;

	msg = m_SIFRAProtocol->getMsg();	// 	a seconda del messaggio ricevuto, risponde adeguatamente ed eventualmente cambia stato
	while (msg != SIFRAMsg_noMsg)
	{
		switch (msg)
		{
			case SIFRAMsg_noMsg:
				break;
			case SIFRAMsg_StopAll:	// stop generale, no risposta
				changeSIFRAstatus(StopAllStatus);
				break;
			case SIFRAMsg_setStart:	// da lo start con una serie di alternative di funzioni
				if((getSIFRAstatus() == AttesaComando) || (getSIFRAstatus() == PausaErogazione) || (getSIFRAstatus() == RipresaMancanzaRete)) //||(getSIFRAstatus() == PausaFormulaService))
				{
					Decode_StartMsg();			// prevedo di evitare la ricezione e decodicfica di uno start message si è in START o RIEMPITUBI
					SIFRAMsg_setStartHandler();
				}
				else
				{
					if((StatusCmd.status == STATO_START) || (StatusCmd.status == STATO_CALIBRAZIONE_UTENTE) ||
						(StatusCmd.status == STATO_CALIBRAZIONE_FABBRICA) || (StatusCmd.status == STATO_ST))
					{
						SIFRAMsg_setStartHandler();
					}
					else
					{
						if(StatusCmd.state == StatoManuale)
						{
							SIFRAMsg_setStartHandler();
						}
						else
						{
							StatusCmd.status = STATO_ERRORE;
							StatusCmd.errors = ERR_SYSTEM;
							StatusCmd.log_error = ERR_CASE_DECODE_MSG_STATUS;
						}
					}
				}
				break;
			case SIFRAMsg_statusAsked:	// risponde alla richiesta di stato del sistema
				SIFRAMsg_statusAskedHandler();
				m_watchDogTimer.Preset(WATCHDOG_TIME);
				break;
			case SIFRAMsg_infoAsked:
				SIFRAMsg_infoAskedHandler();	// invia il pacchetto di stato (versione HW e FW)
				break;
			case SIFRAMsg_debugAsked:
				SIFRAMsg_debugAskedHandler();
				break;
			case SIFRAMsg_debugVie_1_3_Asked:
				SIFRAMsg_debugAskedVie_1_3_Handler();
				break;
			case SIFRAMsg_debugVie_4_5_Asked:
				SIFRAMsg_debugAskedVie_4_5_Handler();
				break;
			case SIFRAMsg_debugStartAsked:
				SIFRAMsg_debugAskedStartHandler();
				break;
			case SIFRAMsg_debugRamAsked:
				SIFRAMsg_debugAskedRamHandler();
				break;
			case SIFRAMsg_newLines:
				m_SIFRAProtocol->sendAcknowledge();
				m_verifica_linea = True;
				break;
			case SIFRAMsg_maxWeightReset:
				m_SIFRAProtocol->sendAcknowledge();
				if(g_rstMaxWeight.lines > 0)
				{
					for(line=0;line<NUM_MAX_LINE;line++)
					{
						if(g_rstMaxWeight.lines & (0x01 << line))
						{
							m_TimerChkHw[line].Preset(_TIMEOUT_CHKHW);
							m_TimerMaxWeight[line].Preset(_3_SEC_);
						}
					}
				}
				else
				{
					for(line=0;line<NUM_MAX_LINE;line++)
					{
						m_TimerMaxWeight[line].Preset(_3_SEC_);
						m_TimerChkHw[line].Preset(_TIMEOUT_CHKHW);
					}
				}
				m_timerControlCover.Preset(_3_SEC_);
				break;
			case SIFRAMsg_restartControl:
				m_SIFRAProtocol->sendAcknowledge();
				SIFRAWeightControlRestart(g_restartControlCmd.type_of_formula);
				break;
			case SIFRAMsg_scalesPhase:
				m_SIFRAProtocol->sendAcknowledge();
				SIFRASetScalesPhase(g_scalesPhaseCmd.init_end, g_scalesPhaseCmd.type_of_phase);
				break;
			case SIFRAMsg_scalesNotCalib:
				m_SIFRAProtocol->sendAcknowledge();
				SIFRASetScalesNotCalibrated();
				break;
			case SIFRAMsg_setZeroCalib:	// invia risposta e comincia la lettura degli zeri uno di seguito all'altro
				changeSIFRAstatus(StatoCalibrazione);
				m_SIFRAProtocol->sendAcknowledge();	// questo va spedito subito per "sbloccare" la tastiera, poi eseguiamo il comando
				g_stateMachine.calibState = ReadZeroInBothLines;
				break;
			case SIFRAMsg_setOffset1:
				this_chan = _ADC1_;
				changeSIFRAstatus(StatoCalibrazione);
				m_SIFRAProtocol->sendAcknowledge();	// questo va spedito subito per "sbloccare" la tastiera, poi eseguiamo il comando
				g_stateMachine.calibState = ReadZeroInThisLine;
				break;
			case SIFRAMsg_setOffset2:
				this_chan = _ADC2_;
				changeSIFRAstatus(StatoCalibrazione);
				m_SIFRAProtocol->sendAcknowledge();	// questo va spedito subito per "sbloccare" la tastiera, poi eseguiamo il comando
				g_stateMachine.calibState = ReadZeroInThisLine;
				break;
			case SIFRAMsg_setOffset3:
				this_chan = _ADC3_;
				changeSIFRAstatus(StatoCalibrazione);
				m_SIFRAProtocol->sendAcknowledge();	// questo va spedito subito per "sbloccare" la tastiera, poi eseguiamo il comando
				g_stateMachine.calibState = ReadZeroInThisLine;
				break;
			case SIFRAMsg_setOffset4:
				this_chan = _ADC4_;
				changeSIFRAstatus(StatoCalibrazione);
				m_SIFRAProtocol->sendAcknowledge();		// questo va spedito subito per "sbloccare" la tastiera, poi eseguiamo il comando
				g_stateMachine.calibState = ReadZeroInThisLine;
				break;
			case SIFRAMsg_setOffset5:
				this_chan = _ADC5_;
				changeSIFRAstatus(StatoCalibrazione);
				m_SIFRAProtocol->sendAcknowledge();	// questo va spedito subito per "sbloccare" la tastiera, poi eseguiamo il comando
				g_stateMachine.calibState = ReadZeroInThisLine;
				break;
			case SIFRAMsg_setGain1:
				this_chan = _ADC1_;
				changeSIFRAstatus(StatoCalibrazione);
				m_SIFRAProtocol->sendAcknowledge();	// questo va spedito solo a lettura eseguita
				g_stateMachine.calibState = ReadLoadInThisLine;
				break;
			case SIFRAMsg_setGain2:
				this_chan = _ADC2_;
				changeSIFRAstatus(StatoCalibrazione);
				m_SIFRAProtocol->sendAcknowledge();	// questo va spedito solo a lettura eseguita
				g_stateMachine.calibState = ReadLoadInThisLine;
				break;
			case SIFRAMsg_setGain3:
				this_chan = _ADC3_;
				changeSIFRAstatus(StatoCalibrazione);
				m_SIFRAProtocol->sendAcknowledge();	// questo va spedito solo a lettura eseguita
				g_stateMachine.calibState = ReadLoadInThisLine;
				break;
			case SIFRAMsg_setGain4:
				this_chan = _ADC4_;
				changeSIFRAstatus(StatoCalibrazione);
				m_SIFRAProtocol->sendAcknowledge();	// questo va spedito solo a lettura eseguita
				g_stateMachine.calibState = ReadLoadInThisLine;
				break;
			case SIFRAMsg_setGain5:
				this_chan = _ADC5_;
				changeSIFRAstatus(StatoCalibrazione);
				m_SIFRAProtocol->sendAcknowledge();	// questo va spedito solo a lettura eseguita
				g_stateMachine.calibState = ReadLoadInThisLine;
				break;
			case SIFRAMsg_resetCalib:					// invia risposta e resetta i parametri di calibrazione in memoria
				m_SIFRAProtocol->sendAcknowledge();
				if(!SIFRACalibResetData(NUM_MAX_LINE))
				{
					StatusCmd.status = STATO_ERRORE;
					StatusCmd.errors = ERR_SYSTEM;
					StatusCmd.log_error = E_ERR_CASE_RESET_CALIB_FAIL;
				}
				break;
			case SIFRAMsg_jumpToLoader:
				m_SIFRAProtocol->sendAcknowledge();	// questo va spedito solo a lettura eseguita
				changeSIFRAstatus(E_WAIT_ENTER_LOADER);
				m_watchDogTimer.Stop();
				m_enter_loader_timer.Preset(_5_SEC_);
				break;
			case E_SIFRAMsg_loopApplication:
				m_SIFRAProtocol->sendAcknowledge();	// questo va spedito solo a lettura eseguita
				changeSIFRAstatus(E_WAIT_ENTER_LOOP);
				break;
			case SIFRAMsg_HwError:
				SIFRAMsg_hwErrorHandler();
				break;
			case SIFRAMsg_timoutRx:
			case SIFRAMsg_timoutTx:
			case SIFRAMsg_opcodeUnknown:
			case SIFRAMsg_protocolStateUnknown:
			case SIFRAMsg_checkSumError:
			case SIFRAMsg_transmitBufferOverflowError:
			case SIFRAMsg_receptionBufferOverflowError:
			case SIFRAMsg_dataReceivedBufferOverflow:
				SIFRAMsg_protocolErrorHandler();
				break;
			case SIFRAMsg_unknownError:
				SIFRAMsg_unknownErrorHandler();
				break;
			default:
				StatusCmd.status = STATO_ERRORE;
				StatusCmd.errors = ERR_SYSTEM;
				StatusCmd.log_error = ERR_CASE_MSG_RECEIVED;
				break;
		}
		msg = m_SIFRAProtocol->getMsg();
	}
}

/**
* Manager dello stato sacca: può uscire per una situazione di errore oppure perchè ha finito.
* Se la procedura viene portata a termine torna True, altrimenti torna False.
*/
bool SIFRA_Manager::SIFRASacca_Manager()
{
	byte vie_abilitate = 0x00;	// maschera di bit
	byte m_line;
	int num_vie_abilitate = 0;
	int verifica_peso;
	int peso_rimanente;
	word peso_abs;
	float soglia;
	word calo_peso = 0;
	word threshold_vuoto;
	word peso_stimato;
	float flusso_stimato;

	switch( g_stateMachine.saccaState )
	{
		case SaccaIdle:
			break;
		case ConfigurazioneSequenza:
			for(m_line = 0; m_line < NUM_MAX_LINE; m_line++)	// scorrendo sulle possibili vie, qui va bene NUM_MAX_LINE
			{
				if(Via[m_line].da_eseguire == True) 	// se questa via è abilitata
				{
					Via[m_line].abilitazione = True;
					Via[m_line].peso_da_erogare = StartCmd.peso_linea[m_line];
					Via[m_line].tara = StartCmd.tara_linea[m_line];
					Via[m_line].peso_spec = AdattaPesoSpecifico(StartCmd.peso_spec[m_line]);
					Via[m_line].peso_stop = m_peso_attuale[m_line];
					setta_led_pannello(m_line, ON);
					num_vie_abilitate++;					// conto le sole via abilitate a partire (non quelle abilitate da comando)
					vie_abilitate |= (0x01 << m_line);
				}
			}
			if(num_vie_abilitate > 0)
			{
				m_SIFRAProtocol->setNumOfEnabledLines(num_vie_abilitate);
				m_SIFRAProtocol->setEnabledLine(vie_abilitate);
				for(m_line = 0; m_line < NUM_MAX_LINE; m_line++)	// scorrendo sulle possibili vie, qui va bene NUM_MAX_LINE
				{
					if(Via[m_line].abilitazione == True)
					{
						if((cpld_reg_in & 0x001F) & (0x01 << m_line))
						{
							StatusCmd.flagsErrorLine = 0x01;
							StatusCmd.statusChan[m_line] |= ERR_RILEV_ARIA;
						}
						else
						{
							StatusCmd.air_block_en[m_line] = True;				// attivazione blocco x aria, l'allarme aria è bloccante e gestisce il led
						}
						m_air_manager[m_line] = E_AIR_MGR_START;
					}
				}
				StatusCmd.fillingOnCourse = True;		// si attiva il controllo motori in caso di errore: i motori sono spenti
				StatusCmd.led_aria_en = True;				// il led rosso se c'è aria può lampeggiare
				g_restart.restart_bk = E_VALID_BACKUP;					// setto flag che mi allerta in caso di mancanza corrente che devo recuperare il backup
				if(!Backup_Param_Restart(&g_restart))
				{
					//StatusCmd.flagsErrorLine = 0x01;
					//StatusCmd.errors = ERR_RAM_TAMPONATA;
					StatusCmd.log_error = ERR_CASE_SAVE_RESTART_DATA;
					//break;
				}
				BackUpTimer.Preset(_330_mSEC_);					// d'ora in avanti un backup al secondo
				g_stateMachine.saccaState = LeggiTareVie;
			}
			else
			{
				m_timerEndSeq.Preset(_1_SEC_);				// esco subito ma devo dare il tempo al padrone di leggere il mio stato start
				g_stateMachine.saccaState = ChiusuraErogazione;			// nessuna via, per qualche motivo, quindi fine sequenza
			}
			break;

		case LeggiTareVie:
			if(m_dato_valido)								// solo quando c'è un dato valido di adc su cui lavorare
			{
				m_dato_valido = False;
				for(m_line = 0; m_line < NUM_MAX_LINE; m_line++)
				{
					if(Via[m_line].abilitazione == True)
					{
						threshold_vuoto = calcolo_margine_tara(m_line, MARGINE_SU_VUOTO);		// calcolo il margine sulla segnalazione dell'errore vuoto
						if(m_peso_attuale[m_line] < threshold_vuoto)
						{
							StatusCmd.flagsErrorLine = 0x01;	// setto un errore
							StatusCmd.statusChan[m_line] |= ERR_RILEV_VUOTO;	// non c'è abbastanza liquido
							m_void_source[m_line] = True;
						}
						else
						{
							m_peso_precedente[m_line] = m_peso_iniziale[m_line] = m_peso_attuale[m_line];	// converte il valore intero in decimi di grammo
							m_peso_finale[m_line] = (int)(m_peso_iniziale[m_line] - Via[m_line].peso_da_erogare);
						}
					}
				}		
				g_stateMachine.saccaState = StartErogazioneVia;
			}
			break;

		case RipresaSequenza:
			if(m_dato_valido)								// solo quando c'è un dato valido di adc su cui lavorare
			{
				m_dato_valido = False;
				num_vie_abilitate = 0;
				for(m_line = 0; m_line < NUM_MAX_LINE; m_line++)	// scorrendo sulle possibili vie, qui va bene NUM_MAX_LINE
				{
						//m_timer_control_restart[m_line].Stop();
						StatusCmd.error_stop[m_line] = E_ERR_STOP_NONE;
						m_void_source[m_line] = False;
						Via[m_line].tara = StartCmd.tara_linea[m_line];
						Via[m_line].peso_da_erogare = StartCmd.peso_linea[m_line];
						Via[m_line].eseguita = (bk.vie_eseguite & (0x01 << m_line));
						Via[m_line].peso_erogato = bk.peso_erogato[m_line];	// recupero i dati del backup precedente
						Via[m_line].peso_spec = AdattaPesoSpecifico(StartCmd.peso_spec[m_line]);
						//Via[m_line].peso_stop = m_peso_attuale[m_line];
						if(Via[m_line].eseguita == False)
						{
							m_peso_precedente[m_line] = m_peso_iniziale[m_line] = m_peso_attuale[m_line];			// converte il valore intero in decimi di grammo
							Via[m_line].peso_gia_erogato = Via[m_line].peso_erogato + m_peso_erog_by_hand[m_line];
							peso_rimanente = Via[m_line].peso_da_erogare - Via[m_line].peso_gia_erogato;
							m_peso_finale[m_line] = m_peso_iniziale[m_line] -peso_rimanente;
							if(peso_rimanente < _VOL_RIPRESA_SEQ_MIN)
							{
								Via[m_line].eseguita = True;
								Via[m_line].abilitazione = False;
							}
							else
							{
								Via[m_line].restart_bag = True;
								Via[m_line].abilitazione = True;
								setta_led_pannello(m_line, ON);
								vie_abilitate |= (0x01 << m_line);		// setto il bit relativo alla via ri-abilitata
								num_vie_abilitate++;			// conto le sole via abilitate a partire (non quelle abilitate da comando
							}
						}
						m_peso_erog_by_hand[m_line] = 0;
				}
				if(num_vie_abilitate > 0)
				{
					m_SIFRAProtocol->setNumOfEnabledLines(num_vie_abilitate);
					m_SIFRAProtocol->setEnabledLine(vie_abilitate);
					for(m_line = 0; m_line < NUM_MAX_LINE; m_line++)	// scorrendo sulle possibili vie, qui va bene NUM_MAX_LINE
					{
						if(Via[m_line].abilitazione == True)
						{
							if((cpld_reg_in & 0x001F) & (0x01 << m_line))
							{
								StatusCmd.flagsErrorLine = 0x01;
								StatusCmd.statusChan[m_line] |= ERR_RILEV_ARIA;
							}
							else
							{
								StatusCmd.air_block_en[m_line] = True;				// attivazione blocco x aria, l'allarme aria è bloccante e gestisce il led
							}
							m_air_manager[m_line] = E_AIR_MGR_START;
						}
					}
					StatusCmd.fillingOnCourse = True;	// si attiva il controllo motori in caso di errore: i motori sono spenti
					StatusCmd.led_aria_en = True;			// il led rosso se c'è aria può lampeggiare
					BackUpTimer.Preset(_330_mSEC_);				// d'ora in avanti un backup al secondo
					g_stateMachine.saccaState = StartErogazioneVia;
				}
				else
				{
					m_timerEndSeq.Preset(_1_SEC_);			// esco subito ma devo dare il tempo al padrone di leggere il mio stato start
					g_stateMachine.saccaState = ChiusuraErogazione;		// nessuna via, per qualche motivo, quindi fine sequenza
				}
			}
			break;

		case StartErogazioneVia:
			calcola_velocita_massime();						// se c'è più di una via in sequenza, allora le velocità massime devono essere proporzionali
			m_timer_rampa = _T_RAMPA_RT_;
			for(m_line = 0; m_line < NUM_MAX_LINE; m_line++)
			{
				if((Via[m_line].abilitazione ==  True) && (Via[m_line].eseguita == False))
				{	
					setStatePump(m_line, ABILITAZIONE);	// fa partire la pompa dedicata o quella unica
					Via[m_line].timer_check_block_pump.Preset(_2_SEC_);
					Via[m_line].timer_rampa.Preset(m_timer_rampa);
					Via[m_line].timerVerificaCalo.Preset(T_NO_CALO);
				}
			}
			g_stateMachine.saccaState = StatoErogazione;
			break;

		case StatoErogazione:
			if(m_dato_valido)								// se c'è una nuova tornata di dati validi (uno per ogni via)
			{
				m_dato_valido = False;
				for(m_line = 0; m_line < NUM_MAX_LINE; m_line++)	// al pirmo allarme aria, blocca tutto, backuppa i dati e mette in pausa
				{
					if(Via[m_line].abilitazione == True)
					{
						if(m_peso_attuale[m_line] <= m_peso_iniziale[m_line])
						{
							Via[m_line].peso_erogato = ((word)(m_peso_iniziale[m_line] - m_peso_attuale[m_line])) + Via[m_line].peso_gia_erogato;
						}

						if( SIFRARampe_Manager(m_line, False))	// gestisce rampa e riempimento: se ha finito, comanda stop motore e fine via
						{
							Via[m_line].firstErogDone = True;
							Via[m_line].timer_check_block_pump.Stop();
							Via[m_line].timerVerificaCalo.Stop();
						}
						// la funzione errori torna un byte diverso da zero se c'è errore, con il bit a 1 che identifica la/le via/e in errore
						StatusCmd.flagsErrorLine |= controllo_errori_linea(m_line, vol_controllo[g_restart.tipo_linea]);	// verifica delle situazioni di non cala e vuoto, viene settato
					}
					else
					{
						Via[m_line].firstErogDone = True;
					}
				}
				if((Via[E_VIA_1].firstErogDone) && (Via[E_VIA_2].firstErogDone) && (Via[E_VIA_3].firstErogDone) &&
					(Via[E_VIA_4].firstErogDone) && (Via[E_VIA_5].firstErogDone))
				{
					g_stateMachine.saccaState = StartRifinituraErogazione;
					m_timerEndSeq.Preset(_2_SEC_);			// esco subito ma devo dare il tempo al padrone di leggere il mio stato start
				}
			}
			break;

		case StartRifinituraErogazione:
			if(m_dato_valido)	// se c'è una nuova tornata di dati validi (uno per ogni via)
			{
				m_dato_valido = False;
				if(m_timerEndSeq.Match())
				{	// ciclando si tutte le vie abilitate e non eseguita
					Aggiorna_pesi_erogati();
					num_vie_abilitate = 0;
					for(m_line = 0; m_line < NUM_MAX_LINE; m_line++)
					{
						if(Via[m_line].abilitazione ==  True)
						{
							if(Via[m_line].eseguita == False)
							{
								if(m_peso_attuale[m_line] > m_peso_finale[m_line])
								{
									m_peso_precedente[m_line] = m_peso_attuale[m_line];
									setPwmMotCycle(m_line, _PWM_CYCLE_LOW);	// imposto una vel bassa costante(vel 5 = 60ml/min = 1ml/sec)
									Via[m_line].timer_check_block_pump.Preset(_2_SEC_);
									Via[m_line].timerVerificaCalo.Preset(T_NO_CALO_FASE_2); // resetto il timer di no_calo
									num_vie_abilitate++;
								}
								else
								{
									Via[m_line].eseguita = True;
									Via[m_line].rifinituraDone = True;
								}
							}
						}
						else
						{
							Via[m_line].rifinituraDone = True;
						}
					}
					m_SIFRAProtocol->setNumOfEnabledLines(num_vie_abilitate);		// aggiorno il num di vie abilitate
					g_stateMachine.saccaState = RifinituraErogazione;	// cambio stato
					break;
				}	
			}
			break;

		case RifinituraErogazione:
			// qui arrivo quando tutte le vie coinvolte hanno raggiunto i -7cc dall'obiettivo
			// da qui in poi procedo a vel costante sotto il controllo dell'encoder
			// quindi a motori fermi leggo lo stato dei contatori; dal peso mancante calcolo i passi da eseguire e lancio i motori
			// continuo fino a che non ho raggiunto il num passi desiderato.
			if(m_dato_valido)	// se c'è una nuova tornata di dati validi (uno per ogni via)
			{
				m_dato_valido = False;
				for(m_line = 0; m_line < NUM_MAX_LINE; m_line++)	// al pirmo allarme aria, blocca tutto, backuppa i dati e mette in pausa
				{
					if((Via[m_line].abilitazione ==  True) && (Via[m_line].eseguita == False))
					{
						if(m_peso_attuale[m_line] < m_peso_iniziale[m_line])	// margine di un decimo di grammo
						{
							Via[m_line].peso_erogato = ((word)(m_peso_iniziale[m_line] - m_peso_attuale[m_line])) + Via[m_line].peso_gia_erogato;
						}
						if((m_peso_attuale[m_line] - m_peso_finale[m_line]) < vol_margine[g_restart.tipo_linea])	// ho per ora eliminato il controllo a encoder
						{
							Via[m_line].eseguita = True;
							setStatePump(m_line, ARRESTO);		// setta lo stato di arresto pompa, il pwm manager si occuperà del resto
							setPwmMotCycle(m_line, 0);			// setta il pwm a zero
							setBlockRelay(m_line, 0);
							Via[m_line].rifinituraDone = True;
							Via[m_line].timer_check_block_pump.Stop();
							Via[m_line].timerVerificaCalo.Stop();
						}
						// la funzione errori torna un byte diverso da zero se c'è errore, con il bit a 1 che identifica la/le via/e in errore
						StatusCmd.flagsErrorLine |= controllo_errori_linea(m_line, (vol_controllo[g_restart.tipo_linea]/2));	// verifica delle situazioni di non cala e vuoto, viene settato
					}
				}
				if((Via[E_VIA_1].rifinituraDone) && (Via[E_VIA_2].rifinituraDone) && (Via[E_VIA_3].rifinituraDone) &&
					(Via[E_VIA_4].rifinituraDone) && (Via[E_VIA_5].rifinituraDone))
				{
					m_timerEndSeq.Preset(_3_SEC_);	// piccolo timer per assestare il peso effettivo
					g_stateMachine.saccaState = ChiusuraErogazione;
				}
			}
			break;

		case ChiusuraErogazione:
			if(m_dato_valido)	// se c'è una nuova tornata di dati validi (uno per ogni via)
			{
				m_dato_valido = False;
				Aggiorna_pesi_erogati();	// sulle vie abilitate viene aggiornato il peso erogato (e il suo backup)
			}
			if(m_timerEndSeq.Match())
			{
				for(m_line = 0; m_line < NUM_MAX_LINE; m_line++)
				{
					Via[m_line].abilitazione = False;	// questa via ha finito oppure è in errore non recuperabile
					if(m_SIFRAProtocol->getEnabledLine() & (0x01 << m_line))	// se la via è abilitata: non esegue se la via è non abilit. oppure se ha finito oppure se è in errore
					{
						if(!Via[m_line].restart_bag)
						{
							calo_peso = (word)(m_peso_iniziale[m_line] - m_peso_attuale[m_line]);
							if(calo_peso > vol_soglia_err_prod[g_restart.tipo_linea])	// delta peso minimo di 10 g continuativi per poter valutare l'errore di volume
							{
								//peso_stimato = (word)(Via[m_line].flusso_pompa * Via[m_line].peso_spec);
								flusso_stimato = (float)MotorStatus[m_line].step_motor_done/g_encStruct[m_line].param_encoder[g_restart.tipo_linea];
								peso_stimato = (word)(flusso_stimato * Via[m_line].peso_spec);
								verifica_peso = calo_peso - peso_stimato;
								soglia = _PERC_SOGLIA_ERR_PRODOTTO * (float)calo_peso;
								peso_abs = abs(verifica_peso);
								if((float)peso_abs > soglia)			// se il peso misurato è sensibilmente diverso da quello calcolato dai passi encoder
								{
									StatusCmd.statusChan[m_line] |= ERR_PRODOTTO;
								}
							}
						}
					}
					Via[m_line].flusso_pompa = 0;
				}
				g_stateMachine.saccaState = FineErogazione;
			}
			break;

		case FineErogazione:
			g_stateMachine.saccaState = SaccaIdle;
			for(m_line = 0; m_line < NUM_MAX_LINE; m_line++)	// al pirmo allarme aria, blocca tutto, backuppa i dati e mette in pausa
			{
				if(m_SIFRAProtocol->getEnabledLine() & (0x01 << m_line))	// se la via è abilitata: non esegue se la via è non abilit. oppure se ha finito oppure se è in errore
				{
					Via[m_line].peso_erogato = ((word)(m_peso_iniziale[m_line] - m_peso_attuale[m_line])) + Via[m_line].peso_gia_erogato;
				}
			}
			reset_state_leds(OFF);					//spegni_leds_pannello;
			StatusCmd.status = STATO_FINE_SEQ;
			StatusCmd.fillingOnCourse = False;
			return True;
			break;
		default:
			StatusCmd.status = STATO_ERRORE;
			StatusCmd.errors = ERR_SYSTEM;
			StatusCmd.log_error = ERR_CASE_SACCA_MGR;
			break;
	}
	
	if(BackUpTimer.Match())
	{
		BackUpTimer.Preset(_330_mSEC_);
		BackUp_StatoStart();
	}
	
	return False;
}


/**
* Manager della fase di riempimento manuale: per qualche motivo, solitamente un allarme aria, la fase di sacca è stata bloccata. Le vie non hanno finito
* il loro riempimento e una o più sono in allarme. L'utente può comandare l'avanzamento manuale di una sola di queste (per esempio per sbollare),
* comandando l'avanzamnto. Deve essere tenuto conto del volume erogato gestendo l'obiettivo, si legge l'allarme aria per gestire lo sbollamneto ma non
* è bloccante. gli altri errori sono attivi. La pompa si ferma se raggiunto l'obiettivo ma anche appena il pulsante è mollato, che corrisponde poi ad uno stop
*/
bool SIFRA_Manager::SIFRAManual_Manager(int m_line)
{
	switch(g_stateMachine.manualState)
	{
		case IdleManuale:
			break;
		case StartManuale:
			if(m_dato_valido)
			{
				m_dato_valido = False;
				Via[m_line].peso_da_erogare = StartCmd.peso_linea[m_line];
				Via[m_line].tara = StartCmd.tara_linea[m_line];
				Via[m_line].peso_erogato = bk.peso_erogato[m_line];	// recupero i dati del backup precedente
				m_peso_precedente[m_line] = m_peso_attuale[m_line];
				if(((int)Via[m_line].peso_da_erogare - (int)Via[m_line].peso_erogato) < vol_margine[g_restart.tipo_linea])	// se c'è meno di un grammo da erogare
				{
					Via[m_line].eseguita = True;
					m_timerEndSeq.Preset(_500_mSEC_);
					g_stateMachine.manualState = ChiusuraManuale;
					break;
				}
				else
				{
					BackUpTimer.Preset(_330_mSEC_);
					Via[m_line].peso_gia_erogato = Via[m_line].peso_erogato + m_peso_erog_by_hand[m_line];
					m_peso_iniziale[m_line] = m_peso_attuale[m_line];		// aggiorno la tara al peso attuale
					m_peso_finale[m_line] = (int)(m_peso_iniziale[m_line] - (Via[m_line].peso_da_erogare - Via[m_line].peso_gia_erogato));				
					Via[m_line].abilitazione = True;
					StatusCmd.fillingOnCourse = True;		// si attiva il controllo motori in caso di errore: i motori sono spenti
					StatusCmd.led_aria_en = True;				// il led rosso se c'è aria può lampeggiare
					StatusCmd.air_block_en[m_line] = False;			// attivazione blocco x aria, l'allarme aria è bloccante e gestisce il led
					setta_led_pannello(m_line, ON);
					ResetMotorStepCounter();					// azzeramento encoder
					setStatePump(m_line, ABILITAZIONE);			// fa partire la pompa dedicata o quella unica
					Via[m_line].vel = _PWM_CYCLE_MANUAL_SPEED;		// velocità intermedia massima da ragiungere
					m_timer_rampa = _T_RAMPA_MAN_;
					Via[m_line].timer_rampa.Preset(m_timer_rampa);
					Via[m_line].timer_check_block_pump.Preset(_2_SEC_);
					Via[m_line].timerVerificaCalo.Preset(T_NO_CALO);
					g_stateMachine.manualState = ControlloManuale;
					m_peso_erog_by_hand[m_line] = 0;
				}
			}
			break;

		case ControlloManuale:
			if(m_dato_valido)									// se c'è una nuova tornata di dati validi (uno per ogni via)
			{
				m_dato_valido = False;
				if(m_peso_attuale[m_line] < m_peso_iniziale[m_line])
				{
					Via[m_line].peso_erogato = ((word)(m_peso_iniziale[m_line] - m_peso_attuale[m_line])) + Via[m_line].peso_gia_erogato;	// qui non ho rifatto la tara
				}
				//m_vel = SIFRASpeedPump_Manager( m_line);		// gestione rampa di accelerazione (non c'è frenata)
				if((m_peso_attuale[m_line] - m_peso_finale[m_line]) < vol_PreTargetManuale[g_restart.tipo_linea] )	// gestisce rampa e riempimento
				{
					Via[m_line].vel = _PWM_CYCLE_LOW_MANUAL_SPEED;		// velocità intermedia massima da ragiungere
					g_stateMachine.manualState = ControlloManuale_Fine;
				}
				if(Via[m_line].timerVerificaCalo.Match())
				{
					if(m_peso_attuale[m_line] < (m_peso_precedente[m_line] - (vol_controllo[g_restart.tipo_linea]/2)))
					{
						Via[m_line].timerVerificaCalo.Preset(T_NO_CALO);
						m_peso_precedente[m_line] = m_peso_attuale[m_line];
					}
					else
					{
						StatusCmd.flagsErrorLine |= (0x01 << m_line);	// setto il bit dell'errore sulla via m_line-esima
						StatusCmd.statusChan[m_line] |= ERR_NON_CALA;
					}
				}
				SIFRASpeedPump_Manager( m_line);		// gestione rampa di accelerazione (non c'è frenata)
			}
			break;
		case ControlloManuale_Fine:
			if(m_dato_valido)									// se c'è una nuova tornata di dati validi (uno per ogni via)
			{
				m_dato_valido = False;
				if(m_peso_attuale[m_line] < m_peso_iniziale[m_line])
				{
					Via[m_line].peso_erogato = ((word)(m_peso_iniziale[m_line] - m_peso_attuale[m_line])) + Via[m_line].peso_gia_erogato;	// qui non ho rifatto la tara
				}
				if((m_peso_attuale[m_line] - m_peso_finale[m_line]) <  vol_margine_RT[g_restart.tipo_linea])	// gestisce rampa e riempimento
				{
					setBlockRelay(m_line, 0);					// ferma immediatamente il motore
					Via[m_line].timer_check_block_pump.Stop();
					Via[m_line].timerVerificaCalo.Stop();
					setStatePump(m_line, ARRESTO);
					m_timerEndSeq.Preset(_3_SEC_);
					g_stateMachine.manualState = ChiusuraManuale;
					break;
				}
				// la funzione errori torna un byte diverso da zero se c'è errore, con il bit a 1 che identifica la/le via/e in errore
				if(Via[m_line].timerVerificaCalo.Match())
				{
					if(m_peso_attuale[m_line] < (m_peso_precedente[m_line] - (vol_controllo[g_restart.tipo_linea]/2)))
					{
						Via[m_line].timerVerificaCalo.Preset(T_NO_CALO);
						m_peso_precedente[m_line] = m_peso_attuale[m_line];
					}
					else
					{
						StatusCmd.flagsErrorLine |= (0x01 << m_line);	// setto il bit dell'errore sulla via m_line-esima
						StatusCmd.statusChan[m_line] |= ERR_NON_CALA;
					}
				}
				SIFRASpeedPump_Manager( m_line);			// gestione rampa di accelerazione (non c'è frenata)
			}
			break;

		case ChiusuraManuale:
			if(m_peso_attuale[m_line] < m_peso_iniziale[m_line])
			{
				Via[m_line].peso_erogato = ((word)(m_peso_iniziale[m_line] - m_peso_attuale[m_line])) + Via[m_line].peso_gia_erogato;	// qui non ho rifatto la tara
			}
			if(m_timerEndSeq.Match())
			{
				if((m_peso_attuale[m_line] - m_peso_finale[m_line]) < _VOL_RIPRESA_SEQ_MIN)
				{
					Via[m_line].eseguita = True;
					Via[m_line].da_eseguire = False;
					Via[m_line].abilitazione = False;	// questa via ha finito oppure è in errore non recuperabile
				}							             // essendo cambiato il peso attuale rispetto al peso dello stop
				Via[m_line].peso_stop = m_peso_attuale[m_line];
				ResetMotorStepCounter();
				StatusCmd.fillingOnCourse = False;
				return True;
			}
			break;
		default:
			StatusCmd.status = STATO_ERRORE;
			StatusCmd.errors = ERR_SYSTEM;
			StatusCmd.log_error = ERR_CASE_MANUAL_MGR;
			break;
	}

	if(BackUpTimer.Match())
	{
		BackUpTimer.Preset(_330_mSEC_);
		BackUp_StatoStart();
	}

	return False;
}

/**
* Manager della fase riempitubi: una sola via alla volta settata in decode_start_msg e indicata dalla variabile m_enable_line
*/
bool SIFRA_Manager::SIFRART_Manager(int m_line)
{
	//int vel_mot;
	int m_vel;									// velocità attuale motore
	word m_primo_step_RT;
	bool ret;

	switch(g_stateMachine.rtState)
	{
		case IdleRiempitubi:
			break;
		case SetupRiempitubi:
			if(m_dato_valido)
			{
				m_dato_valido = False;
				Via[m_line].tara = StartCmd.tara_linea[m_line];
				Via[m_line].peso_da_erogare = StartCmd.peso_linea[m_line];
				Via[m_line].peso_spec = AdattaPesoSpecifico(StartCmd.peso_spec[m_line]);
				Via[m_line].abilitazione = True;
				Via[m_line].da_eseguire = True;
				m_peso_precedente[m_line] = m_peso_iniziale[m_line] = m_peso_attuale[m_line];	// converte il valore intero in decimi di grammo
				m_peso_finale[m_line] = (int)(m_peso_iniziale[m_line] - Via[m_line].peso_da_erogare);
				if((m_peso_finale[m_line] >= m_peso_iniziale[m_line]) || (m_peso_finale[m_line] >= m_peso_attuale[m_line])) 	// se ho già erogato tutto, in caso di rirpesa da stop
				{
					m_timerEndSeq.Preset(_1_SEC_);
					g_stateMachine.rtState = ChiusuraRiempitubi;
					break;
				}
				m_SIFRAProtocol->setEnabledLine((byte)(0x01 << m_line));
				StatusCmd.fillingOnCourse = True;				// si attiva il controllo motori in caso di errore: i motori sono spenti
				StatusCmd.led_aria_en = True;
				StatusCmd.air_block_en[m_line] = False;		// attivazione blocco x aria, l'allarme aria non è bloccante e gestisce il led
				setta_led_pannello(m_line, ON);				// il led rosso se c'è aria ora può lampeggiare
				ResetMotorStepCounter();
				setStatePump(m_line, ABILITAZIONE);					// fa partire la pompa dedicata o quella unica
				g_stateMachine.rtState = StartRiempitubi;				// stato interno allo stato del riempitubi
			}
			break;

		case StartRiempitubi:
			m_vel = _PWM_CYCLE_LOW_SPEED_RT;
			setPwmMotCycle(m_line, m_vel);
			m_timer_rampa = _T_RAMPA_SERVICE_;
			Via[m_line].timer_check_block_pump.Preset(_2_SEC_);
			Via[m_line].timerVerificaCalo.Preset(T_NO_CALO_FASE_2);			// intervallo ampio per verificare solo eventuali tubi chiusi
			g_stateMachine.rtState = ControlloFase1;
			m_primo_step_RT = _VOL_LOW_VEL_RT;
			break;

		case ControlloFase1:
			if(m_dato_valido)										// se c'è una nuova tornata di dati validi (uno per ogni via)
			{
				m_dato_valido = False;
				if((m_peso_iniziale[m_line] - m_peso_attuale[m_line]) > 0)
				{
					Via[m_line].peso_erogato = (word)(m_peso_iniziale[m_line] - m_peso_attuale[m_line]);
				}
				else
				{
					Via[m_line].peso_erogato = 0;
				}
				StatusCmd.flagsErrorLine |= controllo_errori_linea(m_line, vol_controllo[g_restart.tipo_linea]);	// verifica delle situazioni di non cala e vuoto, viene settato
				if(Via[m_line].peso_erogato > m_primo_step_RT)
				{
					m_vel = _PWM_CYCLE_SPEED_RT;				// velocità alta
					setPwmMotCycle(m_line, m_vel);
					Via[m_line].timer_rampa.Preset(m_timer_rampa);
					g_stateMachine.rtState = ControlloFase2;				// esco subito per passare alla fase a velocità alta
				}
			}
			break;

		case ControlloFase2:
			if(m_dato_valido)										// se c'è una nuova tornata di dati validi (uno per ogni via)
			{
				m_dato_valido = False;
				if((m_peso_iniziale[m_line] - m_peso_attuale[m_line]) > 0)
				{
					Via[m_line].peso_erogato = (word)(m_peso_iniziale[m_line] - m_peso_attuale[m_line]);
				}
				StatusCmd.flagsErrorLine |= controllo_errori_linea(m_line, vol_controllo[g_restart.tipo_linea]);	// verifica delle situazioni di non cala e vuoto, viene settato

				if(m_peso_attuale[m_line]  < m_peso_finale[m_line] + vol_Stop_RT[g_restart.tipo_linea])
				{												// se non c'è aria oppure se, anche se in presenza di aria, ha comunque erogato Xcc oltre al target
					if((StatusCmd.statusChan[m_line] & ERR_RILEV_ARIA) == 0)
					{
						Via[m_line].timer_check_block_pump.Stop();
						Via[m_line].timerVerificaCalo.Stop();
						setStatePump(m_line, ARRESTO);
						m_timerEndSeq.Preset(_3_SEC_);
						g_stateMachine.rtState = ChiusuraRiempitubi;
					}
					else
					{
						m_peso_finale[m_line] = m_peso_finale[m_line] - vol_ExtraVol_RT[g_restart.tipo_linea];
						g_stateMachine.rtState = E_ADD_EROG_RT;
					}
				}
			}
			break;
			
		case E_ADD_EROG_RT:
			if(m_dato_valido)										// se c'è una nuova tornata di dati validi (uno per ogni via)
			{
				m_dato_valido = False;
				if((m_peso_iniziale[m_line] - m_peso_attuale[m_line]) > 0)
				{
					Via[m_line].peso_erogato = (word)(m_peso_iniziale[m_line] - m_peso_attuale[m_line]);
				}
				else
				{
					StatusCmd.status = STATO_ERRORE;
					StatusCmd.errors = ERR_SYSTEM;
					StatusCmd.log_error = E_ERR_PESO_INIT_RT_ADD;
				}
				StatusCmd.flagsErrorLine |= controllo_errori_linea(m_line, vol_controllo[g_restart.tipo_linea]);	// verifica delle situazioni di non cala e vuoto, viene settato

				if(m_peso_attuale[m_line] < m_peso_finale[m_line] + vol_Stop_RT[g_restart.tipo_linea])
				{
					Via[m_line].timer_check_block_pump.Stop();
					Via[m_line].timerVerificaCalo.Stop();
					setStatePump(m_line, ARRESTO);
					m_timerEndSeq.Preset(_3_SEC_);
					g_stateMachine.rtState = ChiusuraRiempitubi;
				}
			}
			break;
			
		case ChiusuraRiempitubi:
			if((m_peso_iniziale[m_line] - m_peso_attuale[m_line]) > 0)
			{
				Via[m_line].peso_erogato = (word)(m_peso_iniziale[m_line] - m_peso_attuale[m_line]);
			}
			if(m_timerEndSeq.Match())
			{
				// individuazione linea montata
				if(m_verifica_linea)								// se è la prima via di questa sessione di RT
				{
					m_verifica_linea = False;								// resetto il flag, la verifica linea la faccio solo analizzando la prima via riempita
					g_restart.tipo_linea = Verifica_linea_montata(m_line, m_peso_iniziale[m_line]);	// verifica la linea montata calcolando rapporto fra passi encoder e delta peso
					if(!Backup_Param_Restart(&g_restart))	// salvo in memoria tamponata
					{
						//StatusCmd.flagsErrorLine = 0x01;
						//StatusCmd.errors = ERR_RAM_TAMPONATA;
						StatusCmd.log_error = ERR_CASE_SAVE_RESTART_DATA;
						//return 0;
					}
				}
				
				// calibrazione linea
				ret = salva_rapporto_flusso(m_line, m_peso_iniziale[m_line], m_peso_attuale[m_line], g_restart.tipo_linea);

				if(ret)		// calcola il rapporto passi_encoder/flusso e lo salva in ram tamponata
				{
					if(!Backup_Param_Encoder(&g_encStruct[m_line], m_line))
					{
						//StatusCmd.flagsErrorLine = 0x01;
						//StatusCmd.errors = ERR_RAM_TAMPONATA;
						StatusCmd.log_error = ERR_CASE_SAVE_ENC_DATA;
						//return False;
					}
				}
				else
				{
					StatusCmd.statusChan[m_line] |= ERR_IMP_TO_FLOW;		// segnalo una errata calibrazione, verificabile dal pannello dettagli
				}
				if((StatusCmd.statusChan[m_line] & ERR_RILEV_ARIA) == 0)
				{
					setta_led_pannello(m_line, OFF);
				}
				else
				{
					Via[m_line].stato_luci = ERR_CELLA;
				}
				g_stateMachine.rtState = E_END_RT;
			}
			break;
			
		case E_END_RT:
			Via[m_line].abilitazione = False;						// questa via ha finito oppure è in errore non recuperabile
			StatusCmd.status = STATO_FINE_SEQ;
			StatusCmd.fillingOnCourse = False;
			SifraResetFillingData();
			SifraResetStartCmd();
			SifraResetStateMachine();
			ResetMotorStepCounter();
			return True;
			break;
			
		default:
			StatusCmd.status = STATO_ERRORE;
			StatusCmd.errors = ERR_SYSTEM;
			StatusCmd.log_error = ERR_CASE_RT_MGR;
			break;
	}

	return False;
}

/**
* Manager della fase di svuotamento manuale: l'utente dopo aver sconnesso la via dalla sacca sulla culla, vuole svuotare la linea stessa per evitare il bagno
* durante lo smontaggio linea. allora via tastiera comanda l'avanzamento manuale della pompa a suo piacimento. c'è solo un limite massimo dipendente dalla linea
* per fermare comunque prima o poi (300 linea ped e 600 linea adu)
*/
bool SIFRA_Manager::SIFRAST_Manager(int m_line)
{
	switch(g_stateMachine.manualState)
	{
		case IdleManuale:
			break;
		case StartManuale:
			Via[m_line].abilitazione = True;
			StatusCmd.led_aria_en = True;			// il led rosso se c'è aria ora può lampeggiare
			StatusCmd.air_block_en[m_line] = False;		// attivazione blocco x aria, l'allarme aria è bloccante e gestisce il led
			setta_led_pannello(m_line, ON);
			ResetMotorStepCounter();				// azzeramento encoder
			setStatePump(m_line, ABILITAZIONE);		// fa partire la pompa dedicata o quella unica
			Via[m_line].vel = _PWM_CYCLE_SPEED_ST;		// velocità intermedia massima da ragiungere
			Via[m_line].timer_rampa.Preset(_T_RAMPA_ST_);
			Via[m_line].timer_check_block_pump.Preset(_2_SEC_);
			g_stateMachine.manualState = ControlloManuale;
			break;

		case ControlloManuale:
			SIFRASpeedPump_Manager( m_line);		// gestione rampa di accelerazione (non c'è frenata)
			m_flussoPeso[m_line] = (float)MotorStatus[m_line].step_motor_done / g_encStruct[m_line].param_encoder[g_restart.tipo_linea];	// in decimi di grammi
			// qui c'è il controllo peso massimo
			if(m_flussoPeso[m_line] > vol_svuotatubi[g_restart.tipo_linea])	// gestisce rampa e svuotamento
			{									// metto un limite, con tanti cc la linea è già vuota di sicuro
				setBlockRelay(m_line, 0);			// ferma immediatamente il motore
				Via[m_line].timer_check_block_pump.Stop();
				setStatePump(m_line, ARRESTO);
				g_stateMachine.manualState = IdleManuale;
				return True;
			}
			break;
			
		default:
			StatusCmd.status = STATO_ERRORE;
			StatusCmd.errors = ERR_SYSTEM;
			StatusCmd.log_error = ERR_CASE_ST_MGR;
			break;
	}
	return False;
}

/**
* It manages the filling of m_line, using line pump m_load, controlling increasing and decreasing velocity ramp
* Return 1 if target reached or a error occurred;
*/

bool SIFRA_Manager::SIFRARampe_Manager(int m_line, bool singola_via)
{
	word p_m;
	int vel_mot;
	int t_vel;

	if(m_peso_attuale[m_line] >= m_peso_finale[m_line])
	{
		p_m = (word)(m_peso_attuale[m_line] - m_peso_finale[m_line]);	// peso ancora da erogare (decimi di grammo)
	}
	else
	{
		p_m = 0;
	}
			

	if( p_m < vol_margine[g_restart.tipo_linea])
	{
		setBlockRelay(m_line, 0);	// ferma immediatamente il motore
		Via[m_line].timer_check_block_pump.Stop();
		setStatePump(m_line, ARRESTO);		// setta lo stato di arresto pompa, il pwm manager si occuperà del resto
		Via[m_line].timer_rampa.Stop();
		return True;
	}
	else
	{
		if(singola_via == False)									// ci sono più vie: le faccio fermare tutte, per farle finire assieme a vel costate
		{
			if( p_m < vol_PreTarget[g_restart.tipo_linea])	// mi sto avvicinando al target, procederò piano
			{	// fermo la pompa, perchè l'ultimo tratto sarà realizzato con controllo a encoder
				setPwmMotCycle(m_line, 0);			// setta il pwm a zero
				Via[m_line].timer_rampa.Stop();
				return True;
			}
		}
		
		// controllo accelerazione e decelerazione
		t_vel = (int)(pendenza_frenata[g_restart.tipo_linea] * (float)(p_m / 10));
		
		// questo conto dipende dal tipo di linea. La linea pediatrica permette di andare + avanti, avando flussi istantanei minori
		// se 100 è vel massima, allora questa riga impone come 40g limite per cominciare la decellerazione con la linea dulti e 25g per la linea pediatrica
		if(t_vel > vel_max[m_line]) 	// per ogni via è stato calcolato il massimo da raggiungere, così quando ci sono + vie assieme tendono a finire assieme
		{
			Via[m_line].vel = vel_max[m_line];
		}
		else	
		{
			if(t_vel < _PWM_CYCLE_LOW)
			{
				Via[m_line].vel = _PWM_CYCLE_LOW;
			}
			else
			{
				Via[m_line].vel = t_vel;
			}
		}
		vel_mot = getSpeedPump(m_line);
		if(vel_mot < Via[m_line].vel)
		{
			if(Via[m_line].timer_rampa.Match())
			{
				Via[m_line].timer_rampa.Preset(_T_RAMPA_RT_);
				vel_mot++;
			}
			else
			{
				return False;
			}
		}
		else
		{
			vel_mot = Via[m_line].vel;
		}
		setPwmMotCycle(m_line, vel_mot);

	}
	
	return False;
	
}

/**
* It manages the filling of m_line controlling increasing of speed ramp
*/

void SIFRA_Manager::SIFRARampe_Manager_Test(int m_line)
{
	int p_m;
	int vel_mot;
	int t_vel;
	float weight;

	p_m = m_peso_attuale[m_line];

	// controllo accelerazione e decelerazione (linea adulti)
	weight = (float)p_m / 10;
	t_vel = (int)(pendenza_frenata[0] * weight);
		
	// questo conto dipende dal tipo di linea. La linea pediatrica permette di andare + avanti, avando flussi istantanei minori
	if(t_vel > vel_max[m_line]) 	// per ogni via è stato calcolato il massimo da raggiungere, così quando ci sono + vie assieme tendono a finire assieme
	{
		Via[m_line].vel = vel_max[m_line];
	}
	else	
	{
		if(t_vel < _PWM_CYCLE_LOW)
		{
			Via[m_line].vel = _PWM_CYCLE_LOW;
		}
		else
		{
			Via[m_line].vel = t_vel;
		}
	}
	
	vel_mot = getSpeedPump(m_line);
	
	if(vel_mot < Via[m_line].vel)
	{
		if(Via[m_line].timer_rampa.Match())
		{
			Via[m_line].timer_rampa.Preset(_T_RAMPA_RT_);
			vel_mot++;
		}
	}
	else
	{
		vel_mot = Via[m_line].vel;
	}
	setPwmMotCycle(m_line, vel_mot);
	
}

/**
* Gestisce la rampa di sola accelerazione nella fase riempitubi
*/
int SIFRA_Manager::SIFRASpeedPump_Manager( byte m_line)
{
int vel_mot;

	vel_mot = getSpeedPump(m_line);
	if(vel_mot < Via[m_line].vel)
	{
		if(Via[m_line].timer_rampa.Match())
		{
			Via[m_line].timer_rampa.Preset(m_timer_rampa);
			vel_mot++;
			setPwmMotCycle(m_line, vel_mot);
		}
	}
	else
	{
		vel_mot = Via[m_line].vel;
		setPwmMotCycle(m_line, vel_mot);
	}
	return vel_mot;
}

/**
Get new samples from FIFO's weight channel and give them to system manager.
If stability verify is enable for the channel, matches new sample with previus: difference has to be lower than threshold sets by system (it
dipends to status system)
If difference is lower or higher of the threshold, it set instability error of the channel

se interviene l'errore, verifico anche il successivo, per scongiurare misure spurie.
Se anche il successivo è fuori dal range previsto, allora salvo l'ultimo valore stabile (volendo posso usare tutti i campioni nella fifo per calcolare
il vero ultimo valore stabile, perchè l'ultimo in realtà potrebbe già essere affetto da stabilità ma ancora dentro per un pelo, ma questo dipende
dai limiti settati) in una variabile statica.
continuo ad inserire i nuovi valori che confronto con il new_sample come facevo prima. intanto avendo l'ultimo stabil e conoscendo istante per
istante la velocità pompa, posso ogni volta aggiornare il valore teorico del peso. prima o poi l'instabilità si risolverà (meglio mettere comunque
un timeout per prevedere l'instabile causa rottura). una volta stabilizzato posso confrontare il valore attuale con quello stimato e verificare se la
differenza sta dentro un certo limite (+-2g).
se sta dentro, ok, se sta fuori, devo verificare anche la pompa mossa a mano. se questa non torna, allora considerato che non è possibile il cambio flacone durante
l'errore instabile, devi supporre che il sistema abbia preso una nova tara e quindi la saca deve essere abortita
*/
/**
modifiche apportate 10/08/2016
in calc_next_theor_weight(i), modificato il tipo di dato PWMCYCLE ora passato ad tipo INT per uniformità
controllo sul valore letto del PWMCYCLE, limitato tra 0 e 100
*/
bool SIFRA_Manager::SIFRASample_Manager()
{
	int peso_misurato;
	int next_delta_stim;
	int m_val_stima;
	ViaSample m_via_sample;
	int i;

	if( weightChan->popAdcData(m_WeightSample) )					// carica i dati (word) di tutti i canali fisici  nell'array m_WeightSample
	{															// il peso attuale è indicato in centesimi di ml, per migliorare gli arrotondamenti
		/*
		if(m_SIFRAProtocol->m_show_adcvalue == True)	// funzione di debug: si invia il valore dell'adc (filtrato) non convertito in grammi,
		{
			m_SIFRAProtocol->sendSIFRALoadSamples(m_WeightSample);
		}
		*/

		m_SIFRAProtocol->sendSIFRALoadSamples(m_WeightSample);

		for(i = 0; i < _MAX_LOAD_CHAN_; i++)
		{														// qui invece c'è la conversione in valori fisici mediante i dati di calibrazione
			peso_misurato = (int)m_SIFRAProtocol->sendSIFRALoadSamples(m_WeightSample[i], i);
			switch(stab_error[i])									// allo startup posto a STABILE
			{
				case init_stab:										// inizializzazione, la coda è vuota
					m_via_sample.this_sample = peso_misurato;
					m_via_sample.next_sample = peso_misurato;	// - next_delta_stim;// il prossimo valore stimato è calcolato dall'attuale meno la stima
					m_peso_attuale[i] = peso_misurato;
					m_weight_real_time[i] = m_SIFRAProtocol->getLoadChan(i);
					if(!Via_Sample[i].full())								// se non è piena, aggiungo un dato
					{
						Via_Sample[i].push_back(m_via_sample);
					}
					if(Via_Sample[i].numItem() >= _NUM_MIN_DATA_in_FIFO)   // se ci sono almeno 2 campioni
					{
						stab_error[i] = stabile;							// allora posso iniziare il controllo stabilità	
					}
					break;
				case stabile:											// se arrivo da init ho sicuramente la coda con 2 campioni		
					if(!Via_Sample[i].empty())							// se la FIFO non è vuota
					{
						next_delta_stim = calc_next_theor_weight(i);				// calcola il teorico prossimo peso, in base alla velocità della pompa
						Via_Sample[i].readLastPushed(m_via_sample);		// leggo l'ultimo inserito senza rimuoverlo,che è il campione precedente all'attuale
						Via_Sample[i].clear();							// poi svuoto la FIFO
						// eseguo il confronto per la verifica della stabilità
						m_val_stima = m_via_sample.next_sample;		// valore centrale della forchetta di accettazione
						if((peso_misurato > (m_val_stima - SOGLIA_BASSA_STAB))
										&& (peso_misurato < (m_val_stima + SOGLIA_ALTA_STAB)))
						{	// se è dentro forchetta, calcolo il next_sample del nuov campione e lo metto in FIFO (che così avrà due valori stabili)
							m_via_sample.this_sample = peso_misurato;
							m_via_sample.next_sample = peso_misurato - next_delta_stim;// il prossimo valore stimato è calcolato dall'attuale meno la stima					
							Via_Sample[i].push_back(m_via_sample);		// inserisco il nuovo valore in FIFO
							m_peso_attuale[i] = peso_misurato;				// rilascio il nuovo peso attuale come quello misurato e "approvato"
							m_weight_real_time[i] = m_SIFRAProtocol->getLoadChan(i);
							stab_error[i] = stabile;						// non cambio stato: ho almeno 2 campioni in FIFO
						}
						else
						{	// se è fuori forchetta, calcolo il next sample a partire dall'ultimo stabile, che qui è il penultimo campione letto
							m_via_sample.this_sample = m_val_stima;
							m_via_sample.next_sample =- next_delta_stim;// il prossimo valore è al massimo l'attuale meno un decimo	
							Via_Sample[i].push_back(m_via_sample);		// inserisco il nuovo valore in FIFO
							stab_error[i] = forse_instabile;				// cambio stato con almeno 2 campioni in FIFO
						}
						
					}
					else												// la FIFO era vuota
					{	// aggiungo un campione senza curare la stabilità
					/*
						m_via_sample.this_sample = peso_misurato;
						m_via_sample.next_sample = peso_misurato - next_delta_stim;	// il prossimo valore è al massimo l'attuale meno un decimo					
						Via_Sample[i].push_back(m_via_sample);			// inserisco il nuovo valore in FIFO
						m_peso_attuale[i] = peso_misurato;					// rilascio il nuovo peso attuale come quello misurato e "approvato"
						m_weight_real_time[i] = m_SIFRAProtocol->getLoadChan(i);
						stab_error[i] = stabile;							// non cambio stato: 1 solo campione in FIFO
						*/
					}
					break;
				case forse_instabile:	// stato in cui a fronte di un valore instabile, si verifica se è stato uno spurio oppure se c'è una instabilità vera		
					if(!Via_Sample[i].empty())							// se la FIFO non è vuota
					{
						next_delta_stim = calc_next_theor_weight(i);				// calcola il teorico prossimo peso, in base alla velocità della pompa
						Via_Sample[i].readLastPushed(m_via_sample);		// leggo l'ultimo inserito senza rimuoverlo,che è il campione precedente all'attuale
						Via_Sample[i].clear();							// poi svuoto la FIFO
						// eseguo il confronto per la verifica della stabilità
						m_val_stima = m_via_sample.next_sample;		// valore centrale della forchetta di accettazione
						if((peso_misurato > (m_val_stima - SOGLIA_BASSA_STAB))
										&& (peso_misurato < (m_val_stima + SOGLIA_ALTA_STAB)))
						{
							// se è dentro forchetta, calcolo il next_sample del nuov campione e lo metto in FIFO (che così avrà due valori stabili)	
							m_via_sample.this_sample = peso_misurato;
							m_via_sample.next_sample = peso_misurato - next_delta_stim;	// il prossimo valore stimato, costruito partendo da un valore reale					
							Via_Sample[i].push_back(m_via_sample);		// inserisco il nuovo valore in FIFO
							m_peso_attuale[i] = peso_misurato;
							m_weight_real_time[i] = m_SIFRAProtocol->getLoadChan(i);
							stab_error[i] = stabile;						// cambio stato: ho 2 campioni in FIFO
						}
						else
						{
							//if(!m_verifica_bilance)
							if((!m_verifica_bilance) && (StatusCmd.status != STATO_ST))
							{
								if(!(StatusCmd.statusChan[i] & ERR_CELLA_ROTTA))	// se non è già attivo l'errore di peso massimo
								{
									StatusCmd.statusChan[i] |= ERR_PESOINSTABILE;		// set errore stabilità
								}
								if(Via[i].abilitazione == True)				// da considerare solo sulle vie abilitate
								{
									StatusCmd.flagsErrorLine |= (0x01 << i);	// setto un errore nella via m_chan solo se abilitata
								}
								m_stab_timer[i].Preset(_2_SEC_);			// avvio timer per verifica stabilizzazione
								m_via_sample.this_sample = m_via_sample.next_sample;
								m_via_sample.next_sample =- next_delta_stim;// prima un dato stimato, quindi teoricamente stabile
								Via_Sample[i].push_back(m_via_sample);		// inserisco il valore stimato stabile in FIFO farà ds rifeirmento					
								stab_error[i] = instabile;
							}
							else
							{
								m_via_sample.this_sample = peso_misurato;
								m_via_sample.next_sample = peso_misurato - next_delta_stim;	// il prossimo valore stimato, costruito partendo da un valore reale					
								Via_Sample[i].push_back(m_via_sample);		// inserisco il nuovo valore in FIFO
								m_peso_attuale[i] = peso_misurato;
								m_weight_real_time[i] = m_SIFRAProtocol->getLoadChan(i);
								stab_error[i] = stabile;						// cambio stato: ho 2 campioni in FIFO
							}
						}
					}
					else												// la FIFO era vuota
					{	// aggiungo un campione senza curare la stabilità e ritorno allo stato STABILE
						/*
						m_via_sample.this_sample = peso_misurato;
						m_via_sample.next_sample = peso_misurato - next_delta_stim;	// il prossimo valore è al massimo l'attuale meno un decimo					
						Via_Sample[i].push_back(m_via_sample);			// inserisco il nuovo valore in FIFO
						m_peso_attuale[i] = peso_misurato;					// rilascio il nuovo peso attuale come quello misurato e "approvato"
						m_weight_real_time[i] = m_SIFRAProtocol->getLoadChan(i);
						stab_error[i] = stabile;							// non cambio stato: 1 solo campione in FIFO
						*/
					}						
					break;
				case instabile:				
						// eseguo il confronto per la verifica della stabilità
						/*
						m_val_stima = m_via_sample.next_sample;	// valore centrale della forchetta di accettazione
						if((peso_misurato < (m_val_stima - SOGLIA_BASSA_STAB))	// eseguo il confronto per la verifica della stabilità
							|| (peso_misurato > (m_val_stima + SOGLIA_ALTA_STAB)))  
						{	// continuano i valori instabili, è in corso uno spostamento
							m_stab_timer[i].Preset(_2_SEC_);		// avvio timer per verifica stabilizzazione
							stab_error[i] = instabile;
						}
						else
						{
						*/
						if(m_stab_timer[i].Match())									// se scatta questo allora vuol dire che da un tot di campioni tutti sono entro la forchetta
						{
							Via_Sample[i].clear();
							if(Via[i].abilitazione == True)
							{
								m_stab_timer[i].Preset(_4_SEC_);
								stab_error[i] = E_WAIT_STABILITY;
							}
							else
							{
								StatusCmd.statusChan[i] &= ~ERR_PESOINSTABILE;		// resetto l'errore
								m_stab_timer[i].Stop();
								/*
								m_via_sample.this_sample = peso_misurato;
								m_via_sample.next_sample = peso_misurato - next_delta_stim;	// il prossimo valore è al massimo l'attuale meno un decimo
								Via_Sample[i].push_back(m_via_sample);			// aggiungo il nuovo valore letto
								m_peso_attuale[i] = peso_misurato;
								*/
								m_weight_real_time[i] = m_SIFRAProtocol->getLoadChan(i);
								stab_error[i] = init_stab;
							}
						}
						//}
					break;
				case E_WAIT_STABILITY:
					if(m_stab_timer[i].Match())
					{
						m_stab_timer[i].Stop();
						StatusCmd.statusChan[i] &= ~ERR_PESOINSTABILE;
						stab_error[i] = init_stab;
					}
					break;
				default:
					StatusCmd.status = STATO_ERRORE;
					StatusCmd.errors = ERR_SYSTEM;
					StatusCmd.log_error = ERR_CASE_SAMPLE_MGR;
					break;
			}
		}
		return True;
	}
	else
	{
		return False;
	}
}

/**
* Manager della fase di calibrazione: alla ricezione del comando opportuno, va nello stato e se ci sono dati validi esegue la calibrazione specifica.
* Se la calibrazione fallisce torna errore.
*/
bool SIFRA_Manager::SIFRACalib_Manager(int chan)
{
	bool calib_status_error = False;
	byte i;
	
	switch(g_stateMachine.calibState)
	{
		case IdleCalib:
			break;
			
		case ReadZeroInThisLine:	// valido solo per M3100, una via alla volta
			if(readZeroInThisLine(chan) == False)	// legge lo zero della cella, eventualmente confronta con il valore di rif e salva
			{
				StatusCmd.flagsErrorLine = 0x01;
				StatusCmd.statusChan[chan] |= ERR_LETTURA_ZERO;
				Chan[chan].AreCalibrate = False;
				backup_calib_state((byte)chan, (byte)Chan[chan].AreCalibrate);
				/*
				if(!SIFRACalibResetData(chan))
				{
					StatusCmd.flagsErrorLine = 0x01;
					StatusCmd.errors = ERR_SYSTEM;
					StatusCmd.log_error = E_ERR_CASE_RESET_CALIB_FAIL;
				}
				*/
				return False;
			}
			else
			{
				stab_error[chan] = init_stab;		// COSì QUANDO RIENTRA NELLA sample_manager non va in errore instabile (essendo cambiato lo zero)
				StatusCmd.status = STATO_STOP;
				return True;
			}
			break;

		case ReadLoadInThisLine:
			if(readLoadInThisLine(chan) == False)	// legge l'adc con 2kg, calcola il guadagno e salva in eeprom
			{
				StatusCmd.flagsErrorLine = 0x01;
				StatusCmd.statusChan[chan] |= ERR_LETTURA_2Kg;
				Chan[chan].AreCalibrate = False;
				backup_calib_state((byte)chan, (byte)Chan[chan].AreCalibrate);
				/*
				if(!SIFRACalibResetData(chan))
				{
					StatusCmd.flagsErrorLine = 0x01;
					StatusCmd.errors = ERR_SYSTEM;
					StatusCmd.log_error = E_ERR_CASE_RESET_CALIB_FAIL;
				}
				*/
				return False;
			}
			else
			{
				Chan[chan].AreCalibrate = True;
				backup_calib_state((byte)chan, (byte)Chan[chan].AreCalibrate);
				stab_error[chan] = init_stab;		// COSì QUANDO RIENTRA NELLA sample_manager non va in errore instabile (essendo cambiato lo zero)
				StatusCmd.status = STATO_STOP;
				return True;
			}
			break;

		case ReadZeroInBothLines:	// esegue gli zeri di tute le vie in una sola botta
			for(i=0; i<NUM_MAX_LINE; i++)
			{
				if(readZeroInThisLine(i) == False)	// legge lo zero della cella, eventualmente confronta con il valore di rif e salva
				{
					StatusCmd.statusChan[i] |= ERR_LETTURA_ZERO;
					calib_status_error = True;
					Chan[chan].AreCalibrate = False;
					backup_calib_state((byte)chan, (byte)Chan[chan].AreCalibrate);
					/*
					if(!SIFRACalibResetData(chan))
					{
						StatusCmd.flagsErrorLine = 0x01;
						StatusCmd.errors = ERR_SYSTEM;
						StatusCmd.log_error = E_ERR_CASE_RESET_CALIB_FAIL;
					}
					*/
				}
				else
				{
					stab_error[i] = init_stab;	// COSì QUANDO RIENTRA NELLA sample_manager non va in errore instabile (essendo cambiato lo zero)
				}
			}
			if(!calib_status_error)
			{
				StatusCmd.status = STATO_STOP;
				return True;
			}
			else
			{
				StatusCmd.flagsErrorLine = 0x01;
				return False;
			}
			break;
			
		default:
			StatusCmd.status = STATO_ERRORE;
			StatusCmd.errors = ERR_SYSTEM;
			StatusCmd.log_error = E_ERR_CALIB_MGR_STATE;
			break;
	}

	return False;
	
}

/**
* Manages the air-sensor signals. Sets the third bit of statusChan[num_via] flag byte to 1.
* La funzione controlla tutte le linee continuamente e ne setta il parametro StatusCmd.statusChan[i], quando è rilevata aria. L'informazione serve
* anche nella fase di stop alla tastiera. Se è settato il flag quando è abilitata la variale m_FillingStatus, settando l'eventuale errore anche sulle vie disablitate
* il sistema stoppa tutto appena è rilevato un fronte (quindi anche una bolla isolata) in una delle vie abilitate.
*/
void SIFRA_Manager::SIFRAAirIn_Manager()
{
	byte error_air_flag = (cpld_reg_in & 0x001F);	// rapporto diretto ALL1-ALL5 bit 1-bit 5 con cablaggio nuovo e scheda UABD M3300
	byte i;
	float flusso_attuale; 	// in decimi di ml

	for(i = 0; i < NUM_MAX_LINE; i++)
	{
		if(m_TimerAllarmeAria[i].Match())
		{
			switch(m_air_manager[i])
			{
				case E_AIR_MGR_START:			// se la verifica aria è abilitata, da qui comincia l'analisi dell'eventuale aria
					if(error_air_flag & (0x01 << i))	// per scheda con cablaggi tipo nuovo sull'UABD ver 2013
					{
						m_TimerAllarmeAria[i].Preset(_TIME_READ_FAST_AIR);	// rilevamento aria temporizzato ogni 10msec
					}
					else
					{
						m_TimerAllarmeAria[i].Preset(_TIMER_READ_AIR);	// rilevamento aria temporizzato ogni 100msec
					}
					m_air_manager[i] = E_AIR_MGR_CONTROL;
					break;
				case E_AIR_MGR_CONTROL:
					if(error_air_flag & (0x01 << i))	// per scheda con cablaggi tipo nuovo sull'UABD ver HW 1.00.00	//if( error_air_flag & (0x80 >> i))	// per scheda con cablaggi tipo vecchio sull'UABD ver HW 1.00.01
					{
						m_TimerAllarmeAria[i].Preset(_TIME_READ_FAST_AIR);	// rilevamento aria di rifare subito per ferificare se è vero o lettura spuria
						m_count_aria[i] = 0;					// resetto il contatore dei giri di aria rilevata
						m_volume_aria[i] = 0;				// resetto il totalizzatore del volume di aria compplessiva
						m_air_manager[i] = E_AIR_MGR_ALERT;			// cambio stato: sono in un episodio di aria da analizzare
					}
					else
					{
						m_TimerAllarmeAria[i].Preset(_TIMER_READ_AIR);	// rilevamento aria temporizzato ogni 100msec
					}
					break;
				case E_AIR_MGR_ALERT:
					m_TimerAllarmeAria[i].Preset(_TIME_READ_FAST_AIR);	// rilevamento aria di rifare subito
					if(error_air_flag & (0x01 << i))	// c'è ancora aria, allora forse è seria
					{
						m_count_aria[i]++;
						flusso_attuale = calc_flusso_pompa(i);	// questo calcolato era un flusso su 10msec, ritorna decimi di ml
						m_volume_aria[i] += flusso_attuale;
						m_air_manager[i] = E_AIR_MGR_ACTIVE;
					}
					else		// se non c'è aria allora era una bollicina iniqua o un errore di elttura
					{
						m_air_manager[i] = E_AIR_MGR_CONTROL;
					}
					break;
				case E_AIR_MGR_ACTIVE:	// qui c'è aria, ma devo integrare e settare akllarme solo per bolle sopra il ml oppure per bolle lunghe + di un secondo con pompa che va
					if(error_air_flag & (0x01 << i))		// ancora aria, integro
					{
						m_count_aria[i]++;
						flusso_attuale = calc_flusso_pompa(i);		// questo calcolato era un flusso su 10msec, ritorna decimi di ml
						m_volume_aria[i] += flusso_attuale;			// incremento il contatore dei decimi di ml del volume della bolla d'aria
						if((m_volume_aria[i] > _VOLUME_MAX) || (m_count_aria[i] > _TEMPO_DI_ARIA)) 	// la bolla supera il volume di soglia oppure è passato 1 sec di aria
						{								// ho rilevato aria
							StatusCmd.statusChan[i] |= ERR_RILEV_ARIA;		// setto il bit relativo all'allarme aria
							imposta_allarme_aria(i);		// setta il flag per la tasteira, il flag per accendere la luce e l'errore di sistema, in base alle condizioni
							m_TimerAllarmeAria[i].Preset(_TIMER_READ_LONG_AIR);	// rilevamento aria di rifare dopo un pò
							m_air_manager[i] = E_AIR_MGR_ALARM;
						}
						else
						{
							m_TimerAllarmeAria[i].Preset(_TIME_READ_FAST_AIR);
						}
					}
					else									// sembra finita ma per sicurezza rileggo a breve distanza
					{
						m_TimerAllarmeAria[i].Preset(_TIME_READ_FAST_AIR);	// rilevamento aria di rifare subito
						m_air_manager[i] = E_AIR_MGR_ALERT;
					}
					break;
				case E_AIR_MGR_ALARM:
					m_TimerAllarmeAria[i].Preset(_TIMER_READ_AIR);	// rilevamento aria di rifare dopo un pò
					//if( error_air_flag & (0x80 >> i))		// per scheda con cablaggi tipo vecchio sull'UABD ver HW 1.00.01
					if(!(error_air_flag & (0x01 << i)))		// c'è ancora aria, sarà stata una lettura spuria
					{
						m_air_manager[i] = E_AIR_MGR_RESET;
					}
					else
					{
						StatusCmd.statusChan[i] |= ERR_RILEV_ARIA;		// setto il bit relativo all'allarme aria
					}
					break;
				case E_AIR_MGR_RESET:
					StatusCmd.statusChan[i] &= ~ERR_RILEV_ARIA;	// resetto il bit relativo all'alalrme aria
					m_TimerAllarmeAria[i].Preset(_TIMER_READ_AIR);	// rilevamento aria di rifare dopo un pò
					Via[i].stato_luci = IDLE_LUCE;
					m_air_manager[i] = E_AIR_MGR_START;
					break;
				default:
					StatusCmd.status = STATO_ERRORE;
					StatusCmd.errors = ERR_SYSTEM;
					StatusCmd.log_error = ERR_CASE_AIR_MGR;
					break;
			}
		}
	}
}

/**
* Manages of all type of error in CPU_SIFRA state machine, like block, air alarm, ...
*/
bool SIFRA_Manager::SIFRAError_Manager()
{
	int m_line;

	/*
	* Il metodo VerificaESD_Damage() deve essere decommentato solo per prove e mai in release.
	* Interviene ad esempio sulle variabili di ogni via associate ai led della scheda luci. Se non commentata i led non vengono accesi.
	*/
	//VerificaESD_Damage();	// verifica lo stato del continous conversion mode degli ADC, se si è interrotto li resetto e accendo l'allarme instabile
	
	if((StatusCmd.status != STATO_CALIBRAZIONE_FABBRICA) && (StatusCmd.status != STATO_CALIBRAZIONE_UTENTE))
	{
		Verifica_celle();			// ogni _TIMEOUT_CHKHW verifica l'eventuale peso massimo o celle rotte
		Verifica_peso_massimo();
	}

	Verifica_encoder_motori();	// gestisce contatori encoder motori e verifica errori di movimento

	Verifica_apertura_sportello();

	SIFRAAirIn_Manager();		// verifica aria nelle linee di riempimento

	if(StatusCmd.flagsErrorLine != NO_ERRORS)	// se è settato un errore generico sulla linea abilitata
	{
		StatusCmd.flagsErrorLine = 0x00;
		if((StatusCmd.fillingOnCourse == True))
		{
			for(m_line = 0; m_line < NUM_MAX_LINE; m_line++)
			{
				setBlockRelay(m_line, 0);		// lo anticipo in via del tutto eccezionale per evitare erogazioni ulteriori dopo l'errore
				setStatePump(m_line, ARRESTO);
				Via[m_line].timer_check_block_pump.Stop();
				Via[m_line].timerVerificaCalo.Stop();
			}
		}
		for(m_line = 0; m_line < NUM_MAX_LINE; m_line++)
		{
			if(Via[m_line].abilitazione == True)
			{
		 		if(StatusCmd.statusChan[m_line] & ERR_PESOINSTABILE)		// priorità più alta
		 		{
					Via[m_line].stato_luci = ERR_INSTABILE;	// così per questa vi comincia la gestione del lampeggio indipendente dalle altre
		 		}
				else
				{
					if(StatusCmd.statusChan[m_line] & ERR_RILEV_ARIA)		// priorità due
					{
						Via[m_line].stato_luci = ERR_ARIA;
					}
					else
					{
						if(StatusCmd.statusChan[m_line] & ERR_RILEV_VUOTO)		// priorità tre
						{
							Via[m_line].stato_luci = ERR_VIA;
						}
						else
						{
							if(StatusCmd.statusChan[m_line] & ERR_NON_CALA)		// priorità 4
							{
								Via[m_line].stato_luci = ERR_VIA;
							}
						}
					}
				}
			}
		}
		// se interviene un errore quando sono già in stop (cioè ho ricevuto il cmd di stop dalla tastiera), non devo cambiare stato
		if(getSIFRAphase() != E_PHASE_PAUSE)
		{	// altrimenti mi metto in attesa del comando di stop della tastiera.
			StatusCmd.status = STATO_ERRORE;
			changeSIFRAstatus(StatoErrore);		// in attesa dell'intervento dell'utilizzatore
		}
		return True;
	}

	return False;
}

/**
* Manager della fase di stop durante una formula, dove si controllano eventuali movimenti manuali delle pompe.
*/
void SIFRA_Manager::SIFRABreak_Manager()
{
	byte line;
	float soglia_calo_peso;

	for(line = 0;line<NUM_MAX_LINE;line++)
	{
		if(Via[line].da_eseguire)
		{
			// N.B. per il momento escludo movimenti all'indietro, solo rotazioni in avanti
			if(MotorStatus[line].step_motor_done > _SOGLIA_ENC_)			// encoder rileva movimento rotore a motore fermo
			{
				soglia_calo_peso = (float)MotorStatus[line].step_motor_done/g_encStruct[line].param_encoder[g_restart.tipo_linea];
				if((StartCmd.function & 0x00FF) != RIEMPITUBI)
				{
					if(soglia_calo_peso >= vol_soglia_hand_move[g_restart.tipo_linea])
					{
						StatusCmd.error_stop[line] |= E_ERR_STOP_HAND_MOVE_FAIL;	// eccessiva variazione peso dovuta a movimento pompa a mano
					}
					else
					{
						StatusCmd.error_stop[line] |= E_ERR_STOP_HAND_MOVE;		// variazione peso accettabile dovuta a movimento pompa a mano
						m_peso_erog_by_hand[line] = (word)soglia_calo_peso;
					}
				}
			}
		}
	}
}

/**
* Manager della ripresa di una formula, con controllo di conformità del peso con l'ultimo misurato prima dello stop.
*/
void SIFRA_Manager::SIFRAWeightControlRestart(char __typeOfFilling)
{
	byte line;
	//float soglia_calo_peso;

	switch(__typeOfFilling)
	{
		case E_TYPE_FORMULA:
			for(line = 0;line<NUM_MAX_LINE;line++)
			{
				if(Via[line].da_eseguire)
				{
				/*
					if(m_timer_control_restart[line].Match())
					{
						m_timer_control_restart[line].Preset(_1_SEC_);
						*/
					Via[line].variazione_peso_stop = Via[line].peso_stop - m_peso_attuale[line];
					if(Via[line].variazione_peso_stop > _MIN_STOP_WEIGHT_VARIATION_)		// rilevazione di un calo peso che potrebbe essere andato in sacca
					{
						if(Via[line].variazione_peso_stop > _MAX_STOP_WEIGHT_DECREASE_)		// rilevazione di un calo peso eccessivo che potrebbe essere andato in sacca
						{
							StatusCmd.error_stop[line] |= E_ERR_STOP_EXCESSIVE_DECREASE;
						}
						else
						{
							StatusCmd.error_stop[line] |= E_ERR_STOP_WEIGHT_DECREASE;		// rilevazione di un calo peso accettabile che potrebbe essere andato in sacca
						}
					}
					else if(Via[line].variazione_peso_stop < -(_MIN_STOP_WEIGHT_VARIATION_))	// rilevazione di un aumento del peso da giustificare
					{
						if(Via[line].variazione_peso_stop < -(_STOP_WEIGHT_VARIATION_BAG_CHANGE_))
						{
							if(m_void_source[line])
							{
								StatusCmd.error_stop[line] |= E_ERR_STOP_BAG_CHANGED;	// aumento peso anticipato da errore vuoto, per cui sicuramente dovuto a cambio sacca
							}
							else
							{
								StatusCmd.error_stop[line] |= E_ERR_STOP_MAYBE_BAG_CHANGED;	// aumento peso imputabile a cambio sacca
							}
						}
						else
						{
							StatusCmd.error_stop[line] |= E_ERR_STOP_WEIGHT_INCREASE;		// aumento peso non motivato
						}
					}
					//}
				}
			}
			break;
			
		case E_TYPE_SERVICE:
			for(line = 0;line<NUM_MAX_LINE;line++)
			{
				if(Via[line].da_eseguire)
				{
				/*
					if(m_timer_control_restart[line].Match())
					{
						m_timer_control_restart[line].Preset(_1_SEC_);
						*/
					Via[line].variazione_peso_stop = Via[line].peso_stop - m_peso_attuale[line];
					if(Via[line].variazione_peso_stop < -(_STOP_WEIGHT_VARIATION_BAG_CHANGE_))
					{
						if(m_void_source[line])
						{
							StatusCmd.error_stop[line] |= E_ERR_STOP_BAG_CHANGED;	// aumento peso anticipato da errore vuoto, per cui sicuramente dovuto a cambio sacca
						}
						else
						{
							StatusCmd.error_stop[line] |= E_ERR_STOP_MAYBE_BAG_CHANGED;	// aumento peso imputabile a cambio sacca
						}
					}
					//}
				}
			}
			break;
			
		default:
			break;
	}
}

/**
* Manager gestione chiusura o interruzione erogazione. Viene chiamata quando per errore occorso al sistema oppure per scelta dell'utilizzatore,
* viene mandato uno stop al sistema qquando è in START (sacca, riempitubi, manuale). Questa routine deve gestire il salvataggio dello stato attuale
* delle vie, per consentire la ripartenza;
* molto delicato è la getsione della via eventualmente in errore instabile con il caso che se il valore ri-stabilizzato è troppo diverso da
* quello previsto, allora il sistema si chiude con un errore inaccettabile --> abort della sacca
*/
bool SIFRA_Manager::	SIFRAStop_Manager()
{
	byte line;

	/*
	if(m_dato_valido)	// se c'è una nuova tornata di dati validi (uno per ogni via)
	{
		m_dato_valido = False;
		for(line = 0; line < NUM_MAX_LINE; line++)
		{
			if(Via[line].abilitazione == True)	// se abilit. continuo ad aggiornare i pesi, in attesa della completa stabilizzazioe
			{
				if((m_peso_iniziale[line] - m_peso_attuale[line]) > 0)
					Via[line].peso_erogato = (word)(m_peso_iniziale[line] - m_peso_attuale[line]) + Via[line].peso_gia_erogato;
			}
		}
	}
	*/
	/*
	if(m_timer_ultimo_peso.Match())	// è passato il minimo ritardo per considerare stabilizzato il peso
	{	// quindi backuppo lo stato delle vie abilitate e vado effettivamente in stop, intanto i leds lampeggiano
		m_timer_ultimo_peso.Stop();
		StatusCmd.fillingOnCourse = False;
		for(line = 0; line < NUM_MAX_LINE; line++)
		{
			if((Via[line].abilitazione == True) && ((StatusCmd.statusChan[line] & ERR_PESOINSTABILE) > 0))// questa via non è in stato di peso stabile
			{	// c'è ancora una via in instabile, non si può uscire da qui
				StatusCmd.fillingOnCourse = True;	// non è ancora il momento di chiudere
			}
		}
		if(StatusCmd.fillingOnCourse == False)		// se invece pocco chiudere il sistema
		{
			return True;
		}
	}
	*/
	if(m_TimerStopAll.Match())		// se scatta questo bisogna uscire e salvare lo stato, abbiamo aspettato troppo
	{
		for(line = 0; line < NUM_MAX_LINE; line++)
		{
			if(Via[line].abilitazione == True)		// se abilit. continuo ad aggiornare i pesi, in attesa della completa stabilizzazioe
			{
				if((m_peso_iniziale[line] - m_peso_attuale[line]) > 0)
				{
					Via[line].peso_erogato = (word)(m_peso_iniziale[line] - m_peso_attuale[line]) + Via[line].peso_gia_erogato;
				}
				Via[line].abilitazione = False;
				Via[line].peso_stop = m_peso_attuale[line];
			}
		}
		BackUp_StatoStart();
		SifraResetFillingData();
		SifraResetStateMachine();
		m_TimerStopAll.Stop();
		return True;
	}
	
	return False;
}

/**
* Manager dello stato sacca: può uscire per una situaizone di errore oppure perchè ha finito, tornando True.
* Altrimenti torna False
*/
bool SIFRA_Manager::SIFRAService_Manager()
{
int m_line;

	switch(g_stateMachine.serviceState)
	{
		case IdleService:
			break;
		case Start_pompe:
			// avvio motori
			// accensione luci verdi vengono accese in decode_startmsg
			// start campionamento pesi
			// avvio watchdog timer
			// cambio stato
			// devo andare in stato start per attivare l'errore di stabilità
			ResetMotorStepCounter();// reset dei contatori encoder
			m_timer_rampa = _T_RAMPA_SERVICE_;
			for(m_line = 0; m_line < NUM_MAX_LINE; m_line++)
			{
				if( Via[m_line].abilitazione )
				{
					setStatePump(m_line, ABILITAZIONE);	// fa partire la pompa dedicata o quella unica
					Via[m_line].vel = _PWM_CYCLE_MAX;
					Via[m_line].timer_rampa.Preset(m_timer_rampa);
				}
			}
			g_stateMachine.serviceState = Controllo_pompe;
			break;

		case Controllo_pompe:
			// gestione rampa motori fino alla velocità massima
			for(m_line = 0; m_line < NUM_MAX_LINE; m_line++)
			{
				if( Via[m_line].abilitazione )
				{
					SIFRASpeedPump_Manager( m_line);
				}
			}
			/*
			if(timer_check_block_pump.Match())
			{
				timer_check_block_pump.Preset(_2_SEC_);
				for(m_line = 0; m_line < NUM_MAX_LINE; m_line++)
				{
					if( Via[m_line].abilitazione )
					{
						vel_mot = getSpeedPump(m_line);
						m_old_step[m_line] = m_this_step[m_line];
						m_this_step[m_line] = (int)MotorStatus[m_line].step_motor_done;
						if(m_waiting_step[m_line] > 0)
						{
							if((m_this_step[m_line] - m_old_step[m_line]) < (m_waiting_step[m_line] * 0.4))
							{
								g_stateMachine.serviceState = uscita;
							}
						}
						m_waiting_step[m_line] = vel_mot * 120;
					}
				}
			}
			*/
			// lettura pesi con verifica stabilità : margine 0.5g
				// in caso di errore
				// stop motori
				// gestione luci rosse e verdi
					// le verdi lampeggiano
					// luce/i rossa/e sulla via/e instabile/i
				// cambio stato allarme_pompe
			break;
		case uscita:
			/*for(m_line = 0; m_line < NUM_MAX_LINE; m_line++)
			{
				if( Via[m_line].abilitazione )
				{
					setStatePump(m_line, ARRESTO);		// setta lo stato di arresto pompa, il pwm manager si occuperà del resto
					setBlockRelay(m_line, 0);
					timer_check_block_pump[m_line].Stop();
					Via[m_line].abilitazione = False;
				}
			}
			*/
			break;
		default:
			StatusCmd.status = STATO_ERRORE;
			StatusCmd.errors = ERR_SYSTEM;
			StatusCmd.log_error = ERR_CASE_SERVICE_MGR;
			break;
	}
	return False;
}

/**
* RoUtine di prova velocità pompe per calcolare il flusso peso e il rapporto giri motore flusso pompa
*/
bool SIFRA_Manager::SIFRATest_Pompe(int m_line)
{
	short m_vel;
	int peso_erog = 0;
	dword m_deltatime;
	static dword m_startime, m_stoptime;
	
	switch(g_stateMachine.serviceState)
	{
		case IdleService:
			break;
		case Start_pompe:
			if(m_dato_valido)	// solo quando c'è un dato valido di adc su cui lavorare
			{
				Via[0].peso_erogato = 0;
				Via[1].peso_erogato = 0;
				Via[2].peso_erogato = 0;
				Via[3].peso_erogato = 0;
				Via[4].peso_erogato = 0;
				m_dato_valido = False;
				m_peso_iniziale[m_line] = m_peso_attuale[m_line];
				//m_vel = 10;
				m_vel = StartCmd.peso_linea[m_line];	// passato in unità 1-10 e convertito in decine 10-100 come valori percentuali
				setta_led_pannello(m_line, ON);
				ResetMotorStepCounter();// reset dei contatori encoder
				setBlockRelay(m_line, 1);	//chiudo il relay
				enable_MOT(m_line);
				setPwmMotCycle(m_line, m_vel);
				m_timerEndSeq.Preset(10000);	// 10 secondi di test
				m_timer_enc_measure.Preset(2000);
				m_startime = globalTimer.getMsec();	//m_startime = globalTimer.getTime();
				g_stateMachine.serviceState = Controllo_pompe;
			}
			break;

		case Controllo_pompe:
			// gestione rampa motori fino alla velocità massima
			if(m_dato_valido)	// solo quando c'è un dato valido di adc su cui laborare
			{
				m_dato_valido = False;
				peso_erog = (int)(m_peso_iniziale[m_line] - m_peso_attuale[m_line]);
				if(peso_erog >= 10)
				{
					Via[0].peso_erogato = (word)(peso_erog);			// peso erogato fino a qui in grammi
					m_deltatime = globalTimer.getMsec() - m_startime;	// millisecondi passati dallo start (in decimale)
					Via[1].peso_erogato = (word)((peso_erog*60000)/m_deltatime);		// peso al minuto g/min attuale
					Via[2].peso_erogato = (word)(MotorStatus[m_line].step_motor_done / peso_erog);		// rapporto flusso/pompa (impulsi enc/g)
					Via[3].peso_erogato = (word)(MotorStatus[m_line].step_motor_done * 120 * 10 / m_deltatime);		// giri/min della pompa
				}
/*** ricorda che c'è un diviso 10 nel supervisore, quindi faccio un *10)  ***/
				// giri assoluti = (impulsi assoluti / 500)
				// giri al minuto = giri assoluti / min = (giri assoluti / msec)*(msec/min)
			}
			if(m_timer_enc_measure.Match())
			{
				m_timer_enc_measure.Preset(2000);
				Via[4].peso_erogato = (word)MotorStatus[m_line].step_motor_done;
			}
			if(m_timerEndSeq.Match())
			{
				setBlockRelay(m_line, 0);		//chiudo il relay
				m_stoptime = globalTimer.getMsec();
				disable_PWM_MOT(m_line);
				setPwmMotCycle(m_line, 0);
				m_timerEndSeq.Preset(_4_SEC_);	// tempo di assestamento
				m_timer_enc_measure.Stop();
				g_stateMachine.serviceState = calcoli;
			}
			break;
		case calcoli:
			if(m_timerEndSeq.Match())
			{
				if(m_dato_valido)	// solo quando c'è un dato valido di adc su cui laborare
				{
					m_dato_valido = False;
					peso_erog = (int)(m_peso_iniziale[m_line] - m_peso_attuale[m_line]);
					Via[0].peso_erogato = (word)(peso_erog);			// peso erogato fino a qui in decimi di grammo
					m_deltatime = m_stoptime - m_startime;	// millisecondi passati dallo start (in decimale)
					Via[1].peso_erogato = 10 * (word)(peso_erog * 6);	// flusso dg/min attuale
					Via[2].peso_erogato = 10 * (word)(MotorStatus[m_line].step_motor_done / peso_erog); // rapporto flusso/pompa (impulsi enc/decimi di g) poi moltiplicato per 10
					Via[3].peso_erogato = 10 * (word)(MotorStatus[m_line].step_motor_done * 3 / 500);	// giri motore/min poi moltiplicato per 10
					Via[4].peso_erogato = (word)MotorStatus[m_line].step_motor_done;
					setta_led_pannello(m_line, OFF);
					return True;
				}
			}
			break;
		default:
			StatusCmd.status = STATO_ERRORE;
			StatusCmd.errors = ERR_SYSTEM;
			StatusCmd.log_error = ERR_CASE_TEST_POMPE_MGR;
			break;
	}
	return False;
}

/**
* Routine di prova verificare il flusso peso in base agli step imposti sul motore
* Per compiere un giro completo del rullo occorre settare uno spostamento di 3000 step encoder
* Impostando uno spostamento di 700000 step encoder si ottiene una movimentazione di una pompa di circa 1 minuto
*/
bool SIFRA_Manager::SIFRATest_FlussoPompe(int m_line)
{
	short m_vel;
	int peso_erog = 0;
	dword m_deltatime;
	static dword m_startime, m_stoptime;
	byte k;
	
	switch(g_stateMachine.serviceState)
	{
		case IdleService:
			break;
		case Start_pompe:
			if(m_dato_valido)	// solo quando c'è un dato valido di adc su cui lavorare
			{
				Via[0].peso_erogato = 0;
				Via[1].peso_erogato = 0;
				Via[2].peso_erogato = 0;
				Via[3].peso_erogato = 0;
				Via[4].peso_erogato = 0;
				for(k=0;k< NUM_MAX_LINE;k++)
				{
					vel_max[k] = _PWM_CYCLE_MAX;
				}
				m_dato_valido = False;
				m_peso_iniziale[m_line] = m_peso_attuale[m_line];
				m_vel = StartCmd.peso_linea[m_line];	// passato in unità 1-10 e convertito in decine 10-100 come valori percentuali
				setta_led_pannello(m_line, ON);
				ResetMotorStepCounter();		// reset dei contatori encoder
				setBlockRelay(m_line, 1);		//chiudo il relay
				enable_MOT(m_line);
				setPwmMotCycle(m_line, m_vel);
				m_timer_enc_measure.Preset(_1_SEC_);
				Via[m_line].timer_rampa.Preset(_T_RAMPA_RT_);		//da inserire per simulare un aumento di velocità delle pompe
				m_startime = globalTimer.getMsec();	//m_startime = globalTimer.getTime();
				g_stateMachine.serviceState = Controllo_pompe;
			}
			break;

		case Controllo_pompe:
			// gestione rampa motori fino alla velocità massima
			if(m_dato_valido)	// solo quando c'è un dato valido di adc su cui laborare
			{
				m_dato_valido = False;
				peso_erog = (int)(m_peso_iniziale[m_line] - m_peso_attuale[m_line]);
			}
			if(m_timer_enc_measure.Match())
			{
				m_timer_enc_measure.Preset(_1_SEC_);
				Via[0].peso_erogato = (word)(MotorStatus[m_line].step_motor_done);
			}
			SIFRARampe_Manager_Test(m_line);	//da inserire per simulare un aumento di velocità delle pompe
			if(MotorStatus[m_line].step_motor_done >= 700000)		// da verificare se 3200 step 
			{
				setBlockRelay(m_line, 0);		//chiudo il relay
				m_stoptime = globalTimer.getMsec();
				disable_PWM_MOT(m_line);
				setPwmMotCycle(m_line, 0);
				m_timerEndSeq.Preset(_2_SEC_);	// tempo di assestamento
				g_stateMachine.serviceState = calcoli;
			}
			break;
			
		case calcoli:
			if(m_timerEndSeq.Match())
			{
				if(m_dato_valido)	// solo quando c'è un dato valido di adc su cui laborare
				{
					m_dato_valido = False;
					peso_erog = (int)(m_peso_iniziale[m_line] - m_peso_attuale[m_line]);
					m_deltatime = m_stoptime - m_startime;	// millisecondi passati dallo start (in decimale)
					Via[0].peso_erogato = (word)(MotorStatus[m_line].step_motor_done);		// passi motore fatti
					Via[1].peso_erogato = (word)(peso_erog);			// peso erogato fino a qui in decimi di grammo
					Via[2].peso_erogato = (word)(m_deltatime);		// tempo di durata del test in ms
					setta_led_pannello(m_line, OFF);
					return True;
				}
			}
			break;
		default:
			StatusCmd.status = STATO_ERRORE;
			StatusCmd.errors = ERR_SYSTEM;
			StatusCmd.log_error = ERR_CASE_TEST_FLUSSO_POMPE_MGR;
			break;
	}
	return False;
}


/**
* Routine di prova
*/
bool SIFRA_Manager::SIFRATest_Emc()
{
	byte line;
	
	switch(m_stato_test_emc)
	{
		case E_EMC_NONE:
			break;
		case E_EMC_START_TEST:
			/*
			for(line=0;line<NUM_MAX_LINE;line++)
			{
				Via[line].abilitazione = True;		// riga da attivare per controllo peso instabile
			}
			*/
			ResetMotorStepCounter();	// reset dei contatori encoder
			for(line=0;line < NUM_MAX_LINE;line++)
			{
				//setBlockRelay(line, 1);	//chiudo i relay dei 5 motori
				m_init_weight[line] = (word)m_weight_real_time[line];
				m_init_load[line] = m_peso_attuale[line];
			}
			Set_weight_threshold();
			StatusCmd.status = STATO_START;
			m_stato_test_emc = E_EMC_ENABLE_VIA_1;
			m_emc_motor_timer.Preset(_1_SEC_);
			break;

		case E_EMC_ENABLE_VIA_1:
			if(m_emc_motor_timer.Match())
			{
				m_emc_motor_timer.Preset(_1_SEC_);
				setBlockRelay(0, 1);
				enable_MOT(0);
				setPwmMotCycle(0, m_vel_test_emc);
				m_first_enc_control[0] = True;
				m_stato_test_emc = E_EMC_ENABLE_VIA_2;
			}
			break;

		case E_EMC_ENABLE_VIA_2:
			if(m_emc_motor_timer.Match())
			{
				m_emc_motor_timer.Preset(_1_SEC_);
				setBlockRelay(1, 1);
				enable_MOT(1);
				setPwmMotCycle(1, m_vel_test_emc);
				m_first_enc_control[1] = True;
				m_stato_test_emc = E_EMC_ENABLE_VIA_3;
			}
			break;

		case E_EMC_ENABLE_VIA_3:
			if(m_emc_motor_timer.Match())
			{
				m_emc_motor_timer.Preset(_1_SEC_);
				setBlockRelay(2, 1);
				enable_MOT(2);
				setPwmMotCycle(2, m_vel_test_emc);
				m_first_enc_control[2] = True;
				m_stato_test_emc = E_EMC_ENABLE_VIA_4;
			}
			break;

		case E_EMC_ENABLE_VIA_4:
			if(m_emc_motor_timer.Match())
			{
				m_emc_motor_timer.Preset(_1_SEC_);
				setBlockRelay(3, 1);
				enable_MOT(3);
				setPwmMotCycle(3, m_vel_test_emc);
				m_first_enc_control[3] = True;
				m_stato_test_emc = E_EMC_ENABLE_VIA_5;
			}
			break;

		case E_EMC_ENABLE_VIA_5:
			if(m_emc_motor_timer.Match())
			{
				setBlockRelay(4, 1);
				enable_MOT(4);
				setPwmMotCycle(4, m_vel_test_emc);
				m_first_enc_control[4] = True;
				m_stato_test_emc = E_EMC_TEST_BEGIN;
			}
			break;

		case E_EMC_TEST_BEGIN:
			m_emc_motor_timer.Stop();
			m_verify_enc_timer.Preset(_2_SEC_);
			m_stato_test_emc = E_EMC_TEST_RUN;
			break;

		case E_EMC_TEST_RUN:
			break;
	}
	
	return False;
}

/**
* Manager of panel leds,
* Control of lights red and green of every line; lights can blink, stay off or stay on, but they can't be switched simultaneously
*/
int SIFRA_Manager::SIFRALed_Manager()
{
	byte m_line;
	word data = 0x0000;


	if(m_resetLedDrivers.Match())	// il timer per il reset pannello luci è scattato, devo fare la routine di reset. Qui entro se sono entrato nella gestione del riavvio della VerificaESD_Damage
	{
		m_resetLedDrivers.Stop();
		if(g_stateMachine.driverState != INIT_DRIVER)
		{
			g_stateMachine.driverState = INIT_DRIVER;
			g_stateMachine.ledState = DISPLAY_RE_INIT_1;
		}
	}

 	if(g_stateMachine.driverState == INIT_DRIVER)
 	{
 		if(m_timerLed.Match())
		{
			switch(g_stateMachine.ledState)
			{
				case DISPLAY_INIT:
					LedDrivers_sendData(INTENSITY);	// spegne il test
					LedDrivers_sendData(TEST_LED);		// digits 0, 1, 2, 3, 4 displayed
					LedDrivers_sendData(SCANLIMIT);		// no decide for digit
					LedDrivers_sendData(DECODE_MODE);
					m_timerLed.Preset(_100_mSEC_);
					g_stateMachine.ledState = DISPLAY_READY;
					break;
				case DISPLAY_READY:
					m_timerLed.Preset(_100_mSEC_);
					switchoff_leds();	// spegni tutte le luci
					m_state_led = ON;
					g_stateMachine.ledState = BLINK_LEDS;
					break;
				case BLINK_LEDS:
					m_timerLed.Preset(_800_mSEC_);
					if(m_state_led)
					{
						m_state_led = OFF;
						switchon_red_leds();	// accende tutti i led rossi e solo i rossi
					}
					else{
						m_state_led = ON;
						switchon_green_leds();	// accende tutti i led verdi e solo i verdi
					}
					break;
				case DISPLAY_RE_INIT_1:	// periodicamente reinizializza il driver luci
					m_timerLed.Preset(_200_mSEC_);
					LedDrivers_sendData(NORMAL_OP);	// inizializzazione
					g_stateMachine.ledState = DISPLAY_RE_INIT_2;
					break;
				case DISPLAY_RE_INIT_2:	// periodicamente reinizializza il driver luci
					m_timerLed.Preset(_100_mSEC_);
					LedDrivers_sendData(INTENSITY);	// spegne il test
					LedDrivers_sendData(SCANLIMIT);		// no decide for digit
					LedDrivers_sendData(DECODE_MODE);
					switchoff_leds();	// spegni tutte le luci
					g_stateMachine.ledState = idle;
					g_stateMachine.driverState = CONTROLL_DRIVER;
					break;
				case GO_TO_CONTROLL_DRIVER_STATUS:
					m_timerLed.Preset(_100_mSEC_);
					switchoff_leds();	// spegni tutte le luci
					reset_state_leds(OFF);	// setta in idle lo stato di tutti i led
					g_stateMachine.ledState = idle;
					g_stateMachine.driverState = CONTROLL_DRIVER;
					break;
				case idle:
					break;
				default:
					m_timerLed.Preset(_100_mSEC_);
					g_stateMachine.ledState = DISPLAY_RE_INIT_1;
					break;
			}
		}
	}
	else if (g_stateMachine.driverState == E_WAIT_DRIVER)
	{
		;
	}
	else			// g_stateMachine.driverState == CONTROLL_DRIVER
	{
		for(m_line = 0; m_line < NUM_MAX_LINE; m_line++)
		{
			if(Via[m_line].timer_led.Match())		// timer relativo a questa via
			{
				if(Via[m_line].stato_led == OFF)	// qui il led è spento, via disabilitata
				{
					Via[m_line].timer_led.Preset(_T_BLINK_STOP);
					if(Via[m_line].stato_luci == SETTA_LUCE)	// via abilitata, luce verde accesa, poi va in IDLE
					{
						data = ((m_line + 1) << 8) | LED_VERDE;
						reset_state_this_led(m_line, ON);	// setta in idle lo stato di tutti i led
						Via[m_line].stato_luci = IDLE_LUCE;
					}
					else
						if(Via[m_line].stato_luci == ERR_CELLA)	// la cella risulta rotta o staccata, lampeggio del verde con il rosso acceso
						{
							data = ((m_line + 1) << 8) | LED_VERDE | LED_ROSSO;
							reset_state_this_led(m_line, ON);	// setta in idle lo stato di tutti i led
							Via[m_line].stato_luci = ERR_CELLA;
						}
						else
						{
							data = ((m_line + 1) << 8) | TURN_OFF;
						}
					LedDrivers_sendData(data);
				}
				else
				{
					asm("nop");
					switch(Via[m_line].stato_luci)
					{
						case IDLE_LUCE:
						case BLINKA_LUCE:
						case SETTA_LUCE:
							Via[m_line].timer_led.Preset(_T_BLINK_START);
							data = ((m_line + 1) << 8) | LED_VERDE;
							break;
						case SPEGNI_LUCE:									// spegne la/le luci e va in idle
							Via[m_line].timer_led.Preset(_T_BLINK_START);
							data = ((m_line + 1) << 8) | TURN_OFF;
							reset_state_this_led(m_line, OFF);					// setta in idle lo stato di tutti i led
							break;
						case ERR_CELLA:										// errore di cella rotta, rosso fisso e verde blink con T = 800msec
							Via[m_line].timer_led.Preset(_T_BLINK_AIR_ERROR);
							if(Via[m_line].fase_luci == 0)
							{
								data = ((m_line + 1) << 8) | LED_VERDE | LED_ROSSO;
								Via[m_line].fase_luci = 1;
							}
							else
							{
								data = ((m_line + 1) << 8) | LED_ROSSO;
								Via[m_line].fase_luci = 0;
							}
							break;
						case ERR_ARIA:										// allarme aria, verde fisso e rosso blink con T = 800msec
						case ERR_VIA:										// allarme non cala o vuoto, verde fisso e rosso blink con T = 800msec
							Via[m_line].timer_led.Preset(_T_BLINK_AIR_ERROR);
							if(Via[m_line].fase_luci == 0)
							{
								data = ((m_line + 1) << 8) | LED_VERDE | LED_ROSSO;
								Via[m_line].fase_luci = 1;
							}
							else
							{
								data = ((m_line + 1) << 8) | LED_VERDE;
								Via[m_line].fase_luci = 0;
							}
							break;
						case ERR_INSTABILE:									//allarme instabile, rosso blink con T  = 200msec e verde blink con T =  800msec
							Via[m_line].timer_led.Preset(_T_BLINK_OTHER_ERR);
							switch(Via[m_line].fase_luci)
							{
								case 0:
									data = ((m_line + 1) << 8) | LED_VERDE | LED_ROSSO;
									Via[m_line].fase_luci++;
									break;
								case 1:
									data = ((m_line + 1) << 8) | TURN_OFF;
									Via[m_line].fase_luci++;
									break;
								case 2:
									data = ((m_line + 1) << 8) | LED_ROSSO;
									Via[m_line].fase_luci++;
									break;
								case 3:
									data = ((m_line + 1) << 8) | TURN_OFF;
									Via[m_line].fase_luci = 0;
									break;
								default:
									Via[m_line].fase_luci = 0;
									break;
							}
							break;
						default:
							Via[m_line].timer_led.Preset(_T_BLINK_AIR_ERROR);
							break;
					}
					LedDrivers_sendData(data);	// valido per tutti gli stati
				}
			}
		}
	}
	return 0;
}

/**
* Identifica la fase del sistema in base alla variabile StatusCmd.state
*/
void SIFRA_Manager::SIFRAPhase_manager()
{
	switch(getSIFRAstatus())
	{
		case InitStatus:
			StatusCmd.phase = E_PHASE_INIT;
			break;
		case AttesaComando:
			StatusCmd.phase = E_PHASE_IDLE;
			break;
		case StatoCalibrazione:
			StatusCmd.phase = E_PHASE_CALIBRATION;
			break;
		case StatoRiempitubi:
		case StatoManuale:
		case StatoSacca:
			StatusCmd.phase = E_PHASE_FILLING;
			break;
		case StatoSvuotatubi:
			StatusCmd.phase = E_PHASE_SVUOTATUBI;
			break;
		case StatoErrore:
			StatusCmd.phase = E_PHASE_ERROR;
			break;
		case StatoStop:
		case StopAllStatus:
		case PausaErogazione:
			StatusCmd.phase = E_PHASE_PAUSE;
			break;
		case StatoService:
		case StatoTestEmc:
		case StatoTestFlussoPompe:
		case StatoTestPompe:
		case StatoTestLuci:
			StatusCmd.phase = E_PHASE_SERVICE;
			break;
		case RipresaMancanzaRete:
			StatusCmd.phase = E_PHASE_RESTART_NO_ALIM;
			break;
		default:
			break;
	}
}

/**
aggiorna i valori istantanei di flusso delle pompa (calcolati du un deltaT di 100msec)
*/
void SIFRA_Manager::SIFRAFlow_manager()
{
	float	partial_flow;

	if(m_TimeCalcFlow.Match())
	{
		m_TimeCalcFlow.Preset(_TIMER_CALC_FLW);

		for(int i = 0;i < NUM_MAX_LINE;i++)
		{
			if((Via[i].abilitazione ==  True) && (Via[i].eseguita == False))	
			{
				partial_flow = (float)MotorStatus[i].partial_step/g_encStruct[i].param_encoder[g_restart.tipo_linea];
				Via[i].flusso_pompa = Via[i].flusso_pompa + partial_flow;
			}
		}
	}
}

/**
* Decoding of parameters recieved in last START MESSAGE from M3000
*/
byte SIFRA_Manager::Decode_StartMsg()
{
	int m_line;
	word i;
	byte line_to_restart = 0;
	word cmd = 0x0000;
	byte data;		// variabile di appoggio dei valori letti dalla EEPROM

	cmd = StartCmd.function & 0x00FF;	// maschera di bit
	switch(cmd)
	{
		case RESET_SISTEMA: 				// re-init di strutture dati e backup memoria
			DefaultSystemReset();			// Inizializzazione del sistema allo stato di avvio
			SifraResetBackup();
			SIFRA_resetStatusError();
			g_restart.restart_bk = E_NO_BACKUP;
			if(!Write_Param_Backup())
			{
				//StatusCmd.errors= ERR_RAM_TAMPONATA;
				//StatusCmd.flagsErrorLine = 0x01;
				StatusCmd.log_error= ERR_CASE_SAVE_BK_DATA;
				//break;
			}
			if(!Backup_Param_Restart(&g_restart))
			{
				//StatusCmd.errors = ERR_RAM_TAMPONATA;
				//StatusCmd.flagsErrorLine = 0x01;
				StatusCmd.log_error = ERR_CASE_SAVE_RESTART_DATA;
				//break;
			}
			changeSIFRAstatus(AttesaComando);
			break;

		case INIT_SISTEMA:				// start con solo abort reset sistema e spegnimento luci
			SifraResetStatus();
			SifraResetFillingData();
			ResetMotorStepCounter();
			SifraResetStateMachine();
			g_stateMachine.driverState = CONTROLL_DRIVER;
			switchoff_leds();
			Setta_filtro_peso(_Fc_1Hz_);		// settaggio del filtro di media sul peso:  15Hz/15 = 1Hz
			if(g_restart.restart_bk == E_VALID_BACKUP)		// lettura flag riempimento attivo prima di mancanza rete
			{	// c'è un backup pre mancanza rete
				if(Carica_StatoStart())					// scarico il backup dello stato macchina prima dell'auto-reset
				{
					StatusCmd.status = STATO_STOP;
					changeSIFRAstatus(RipresaMancanzaRete);
				}
				else
				{
					StatusCmd.status = STATO_ERRORE;
					StatusCmd.errors = ERR_SYSTEM;
					StatusCmd.log_error = E_ERR_CASE_CARICA_START;
				}
			}
			else
			{
				StatusCmd.status = STATO_IDLE;
				changeSIFRAstatus(AttesaComando);
			}
			#ifdef __DEBUG_ADC
				acquisition = True;
			#endif
			break;

		case NUOVA_SEQUENZA:	// start nuova sequenza
			SifraResetViaStatus();
			SifraResetFillingData();
			ResetMotorStepCounter();
			StatusCmd.status = STATO_START;
			changeSIFRAstatus(StatoSacca);
			m_num_enabled_lines = Find_Num_of_Enabled_Lines(StartCmd.support);	// numero di vie abilitate dal comando ricevuto
			Setta_filtro_peso(_Fc_30Hz_);
			if(m_num_enabled_lines > 0)										// legge il numero di vie da eseguire
			{				
				g_stateMachine.driverState = CONTROLL_DRIVER;
				reset_state_leds(OFF);
				g_stateMachine.saccaState = ConfigurazioneSequenza;
			}
			else
			{
				m_timerEndSeq.Preset(_800_mSEC_);					// esco subito ma devo dare il tempo al padrone di leggere il mio stato start
				g_stateMachine.saccaState = ChiusuraErogazione;
			}
			break;

		case RESTART_SEQUENZA:	// START di una sequenza sospesa, non può far ripartire tutte le vie, ma solo quelle
			if(g_restart.restart_bk == E_VALID_BACKUP)
			{
				StatusCmd.status = STATO_START;
				changeSIFRAstatus(StatoSacca);
				for(m_line = 0; m_line < NUM_MAX_LINE; m_line++)	// scorrendo sulle possibili vie, qui va bene NUM_MAX_LINE
				{
					Via[m_line].abilitazione = False;
				}
				m_num_enabled_lines = Find_Num_of_Enabled_Lines(StartCmd.support);				// numero di vie abilitate nel comando
				Setta_filtro_peso(_Fc_30Hz_);
				if (m_num_enabled_lines > 0)							// se c'è un backup attivo e vie abilitate, altrimenti per ora fine sequenza
				{
					if(m_num_enabled_lines == bk.num_lines)
					{
						g_stateMachine.driverState = CONTROLL_DRIVER;
						reset_state_leds(OFF);
						g_stateMachine.saccaState = RipresaSequenza;
					}
					else
					{
						StatusCmd.status = STATO_ERRORE;
						StatusCmd.errors = ERR_SYSTEM;
						StatusCmd.log_error = E_ERR_RIPR_SEQ_NUM_LINES;
					}
				}
				else
				{
					m_timerEndSeq.Preset(_800_mSEC_);	// esco subito ma devo dare il tempo al padrone di leggere il mio stato start
					g_stateMachine.saccaState = ChiusuraErogazione;
				}
			}
			else
			{
				StatusCmd.status = STATO_ERRORE;
				StatusCmd.errors = ERR_SYSTEM;
				StatusCmd.log_error = E_ERR_RIPR_SEQ_BK;
			}
			break;

		case USER_CALIBR:	// calibrazione UTENTE: COMUNICO AL MONDO LO STATO E ACCENDO LUCI
			g_stateMachine.driverState = CONTROLL_DRIVER;	// i led rispondono al tipo di comando
			StatusCmd.status = STATO_CALIBRAZIONE_UTENTE;
			Setta_filtro_peso(_Fc_1Hz_);		// settaggio del filtro di media sul peso:  15Hz/15 = 1Hz
			if(StartCmd.support == 0x00)
			{
				reset_state_leds(OFF);
			}
			else
			{
				for(m_line = 0; m_line < NUM_MAX_LINE; m_line++)	// scorrendo sulle possibili vie, qui va bene NUM_MAX_LINE
				{
					if(StartCmd.support & (0x01 << m_line)) 	// se questa via è abilitata
					{
						setta_led_pannello(m_line, ON);
					}
				}
			}
			break;

		case DEFAULT_CALIBR:	// calibr. incondizionata
			g_stateMachine.driverState = CONTROLL_DRIVER;	// i led rispondono al tipo di comando
			StatusCmd.status = STATO_CALIBRAZIONE_FABBRICA;
			Setta_filtro_peso(_Fc_1Hz_);		// settaggio del filtro di media sul peso:  15Hz/15 = 1Hz
			if(StartCmd.support == 0x00)
			{
				reset_state_leds(OFF);
			}
			else
			{
				for(m_line = 0; m_line < NUM_MAX_LINE; m_line++)	// scorrendo sulle possibili vie, qui va bene NUM_MAX_LINE
				{
					if(StartCmd.support & (0x01 << m_line)) 	// se questa via è abilitata
						setta_led_pannello(m_line, ON);
				}
			}
			break;

		case E_USR_CALIB_CMD:
			g_stateMachine.driverState = CONTROLL_DRIVER;	// i led rispondono al tipo di comando
			StatusCmd.status = STATO_CALIBRAZIONE_UTENTE;
			Setta_filtro_peso(_Fc_1Hz_);		// settaggio del filtro di media sul peso:  15Hz/15 = 1Hz
			reset_state_leds(OFF);
			switchon_green_leds();
			break;

		case E_VERIFY_CALIB_VAL_CMD:
			for(m_line=0;m_line<_MAX_LOAD_CHAN_;m_line++)
			{
				EE_random_byte_read((ADDRESS_CHAN_ARE_MEM_CALIB + m_line*BASESTRUCTADDR), &data);
				m_control_cal_values[m_line].typeOfOffsetCal = data;
				EE_random_byte_read((ADDRESS_CHAN_ARE_MEM_CALIB + 1 + m_line*BASESTRUCTADDR), &data);
				m_control_cal_values[m_line].typeOfGainCal = data;
				m_control_cal_values[m_line].WeightFactoryGain = EE_read_float(ADDRESS_FACTORY_GAIN + m_line*BASESTRUCTADDR);
				m_control_cal_values[m_line].WeightFactoryOffset = EE_read_word(ADDRESS_FACTORY_OFFSET+ m_line*BASESTRUCTADDR);
				m_control_cal_values[m_line].Weightgain = EE_read_float(ADDRESS_GAIN + m_line*BASESTRUCTADDR);
				m_control_cal_values[m_line].Weightoffset = EE_read_word(ADDRESS_OFFSET + m_line*BASESTRUCTADDR);
				m_control_cal_values[m_line].AdcOf2Kg = EE_read_word(ADDRESS_2Kg_READ + m_line*BASESTRUCTADDR);
			}
			asm("nop");
			break;

		case RIEMPITUBI:					// RIEMPITUBI - UNA VIA ALLA VOLTA
			SifraResetViaStatus();									// nuova sequenza, reset di tutte le vie
			SifraResetFillingData();
			SifraResetStateMachine();
			ResetMotorStepCounter();
			StatusCmd.status = STATO_START;
			g_stateMachine.driverState = CONTROLL_DRIVER;				// i led rispondono al tipo di comando
			changeSIFRAstatus(StatoRiempitubi);
			m_enable_line = Find_Enable_line(StartCmd.support);
			Setta_filtro_peso(_Fc_30Hz_);		// campionamento più veloce con conseguente aggiornamento pesi più immediato
			if (m_enable_line > -1)	// una via è abilitata
			{
				//m_timer_control_restart[m_enable_line].Stop();
				Via[m_enable_line].peso_stop = m_peso_attuale[m_enable_line];
				m_void_source[m_enable_line] = False;
				StatusCmd.error_stop[m_line] = E_ERR_STOP_NONE;
				g_stateMachine.rtState = SetupRiempitubi;
			}
			else
			{
				g_stateMachine.rtState = E_END_RT;
			}
			break;

		case SVUOTATUBI:	// SVUOTATUBI - UNA VIA ALLA VOLTA
			m_enable_line = Find_Enable_line(StartCmd.support);
			if (m_enable_line > -1)	// se la via passata è impossibile, non cambia neppure stato e rimane in STOP
			{
				SifraResetViaStatus();
				StatusCmd.status = STATO_ST;
				changeSIFRAstatus(StatoSvuotatubi);
				g_stateMachine.driverState = CONTROLL_DRIVER;				// i led rispondono al tipo di comando
				g_stateMachine.manualState = StartManuale;
			}
			else
			{
				StatusCmd.status = STATO_ERRORE;
				StatusCmd.errors = ERR_SYSTEM;
				StatusCmd.log_error = E_ERR_CASE_LINE_ST;
			}
			break;

		case MANUALE_CON_OBIETTIVO:					// continuazione manuale del riempimento con abilitazione di una sola via
			m_enable_line = Find_Enable_line(StartCmd.support);
			if (m_enable_line > -1)							// se c'è almeno una via abilitata sulle cinque
			{
				StatusCmd.fillingOnCourse = False;
				g_stateMachine.driverState = CONTROLL_DRIVER;		// i led rispondono al tipo di comando
				Setta_filtro_peso(_Fc_30Hz_);		// settaggio del filtro di media sul peso:  15Hz/3 = 5Hz
				g_stateMachine.manualState = StartManuale;		// primo stato della macchina di gestione del manuale
				changeSIFRAstatus(StatoManuale);
			}
			else
			{
				StatusCmd.status = STATO_ERRORE;
				StatusCmd.errors = ERR_SYSTEM;
				StatusCmd.log_error = E_ERR_CASE_MANUAL_EROG;
			}
			break;
			
		case DEBUG_VEDI_ADC_VIE:	// funzione di debug: inviando il comando TARA il peso inviato al M3000 non è in grammi ma in ADC (filtrato)
			if(m_SIFRAProtocol->m_show_adcvalue == True)	// rispedendo lo stesso comando, si ritorna allo stato normale (invio dei pesi in grammi)
				m_SIFRAProtocol->m_show_adcvalue = False;
			else
				m_SIFRAProtocol->m_show_adcvalue = True;
			break;
			
		case DEBUG_SERVICE:								// abort+start+manuale = SERVICE
			if(StartCmd.support != 0x00)
			{
				reset_state_leds(OFF);						// spengo quelli accessi, perchè alla ripartenza rivengono settati
				g_stateMachine.driverState = CONTROLL_DRIVER;	// i led rispondono al tipo di comando
				SifraResetViaStatus();						// nuova sequenza, reset di tutte le vie
				for(m_line = 0; m_line < NUM_MAX_LINE; m_line++)	// scorrendo sulle possibili vie, qui va bene NUM_MAX_LINE
				{
					if(StartCmd.support & (1 << m_line)) 	// se questa via è abilitata
					{
						Via[m_line].abilitazione = True;
						StatusCmd.air_block_en[m_line] = False;			// disattivazione blocco x aria
						setta_led_pannello(m_line, ON);
						line_to_restart = line_to_restart | (0x01 << m_line);		// setto il bit relativo alla via ri-abilitata
					}
				}
			}
			m_SIFRAProtocol->setEnabledLine(line_to_restart);
			StatusCmd.status = STATO_START;
			g_stateMachine.serviceState = Start_pompe;
			changeSIFRAstatus(StatoService);
			break;
		case DEBUG_VEL_POMPE:
			if((m_enable_line = _1on8_to_int(StartCmd.support)) > 0)
			{
				m_enable_line--;							// le vie vanno da 0 a 5 e non da 1 a 6
				Setta_filtro_peso(_Fc_15Hz_);
				g_stateMachine.serviceState = Start_pompe;
				changeSIFRAstatus(StatoTestPompe);
			}
			break;
		case TEST_FLUSSI_POMPE:
			m_enable_line = _1on8_to_int(StartCmd.support);
			if(m_enable_line > 0)
			{
				m_enable_line--;							// le vie vanno da 0 a 5 e non da 1 a 6
				Setta_filtro_peso(_Fc_30Hz_);
				g_stateMachine.serviceState = Start_pompe;
				changeSIFRAstatus(StatoTestFlussoPompe);
			}
			break;
		case DEBUG_EMC:
			SifraResetStatus();
			m_stato_test_emc = E_EMC_NONE;
			m_first_enc_control[i] = False;
			if((StartCmd.peso_linea[0] > 0) && (StartCmd.peso_linea[0] <= 100))
			{
				m_vel_test_emc = StartCmd.peso_linea[0];
			}
			else
			{
				m_vel_test_emc = 50;
			}
			Setta_filtro_peso(_Fc_30Hz_);
			for(i = 0; i < _MAX_LOAD_CHAN_;i++)
			{
				m_TimerAllarmeAria[i].Stop();
				Via[i].timer_check_block_pump.Stop();
			}
			m_enc_value_control[0] = cpld_counter1 & _MASK_16BIT_;
			m_enc_value_control[1] = cpld_counter2 & _MASK_16BIT_;
			m_enc_value_control[2] = cpld_counter3 & _MASK_16BIT_;
			m_enc_value_control[3] = cpld_counter4 & _MASK_16BIT_;
			m_enc_value_control[4] = cpld_counter5 & _MASK_16BIT_;
			m_stato_test_emc = E_EMC_START_TEST;
			changeSIFRAstatus(StatoTestEmc);
			break;
		case E_TEST_LED_CMD:
			m_timerLed.Stop();
			for(m_line = 0; m_line < _MAX_LOAD_CHAN_;m_line++)
			{
				m_TimerAllarmeAria[i].Stop();
			}
			g_stateMachine.driverState = E_WAIT_DRIVER;
			m_test_led_timer.Preset(_1_SEC_);
			StatusCmd.status = STATO_START;
			m_test_led = E_LED_START;
			changeSIFRAstatus(StatoTestLuci);
			break;
		case RESET_ADC_ON_BOARD:						// reset manuale della scheda, con recupero dello stato al riavvio
			reset_4_block = 0x55;							// setto il flag in eeprom, che al riavvio mi dice di fare un ripristino
			SIFRAMsg_resetApplicationHandler();
			break;
		case E_TEST_PKT_ERR:						// reset manuale della scheda, con recupero dello stato al riavvio
			StatusCmd.status = STATO_ERRORE;
			StatusCmd.errors = ERR_SYSTEM;
			StatusCmd.log_error = ERR_CASE_PKT_TEST_ERROR;
			break;
		case E_RESET_RESTART:
			default_struttura_restart();
			if(!Backup_Param_Restart(&g_restart))
			{
				//StatusCmd.flagsErrorLine = 0x01;
				//StatusCmd.errors = ERR_RAM_TAMPONATA;
				StatusCmd.log_error = ERR_CASE_SAVE_RESTART_DATA;
			}
			break;
		default:
			StatusCmd.status = STATO_ERRORE;
			StatusCmd.errors = ERR_SYSTEM;
			StatusCmd.log_error = ERR_CASE_DECODE_START_MSG;
			break;
	}
	return 1;
}

int SIFRA_Manager::SIFRAMsg_setStartHandler()
{
	m_SIFRAProtocol->sendSIFRAStart();
	return 1;
}

int SIFRA_Manager::SIFRAMsg_statusAskedHandler()
{
	m_SIFRAProtocol->sendSIFRAStatus();
	return 1;
}

int SIFRA_Manager::SIFRAMsg_infoAskedHandler()
{
	m_SIFRAProtocol->sendSIFRAInfo();	// invia il pacchetto di stato (versione HW e FW)
	return 1;
}

int SIFRA_Manager::SIFRAMsg_debugAskedHandler()
{
	m_SIFRAProtocol->sendSIFRADebug();
	return 1;
}

int SIFRA_Manager::SIFRAMsg_debugAskedVie_1_3_Handler()
{
	m_SIFRAProtocol->sendSIFRADebugVie_1_3();
	return 1;
}

int SIFRA_Manager::SIFRAMsg_debugAskedVie_4_5_Handler()
{
	m_SIFRAProtocol->sendSIFRADebugVie_4_5();
	return 1;
}

int SIFRA_Manager::SIFRAMsg_debugAskedStartHandler()
{
	m_SIFRAProtocol->sendSIFRADebugStart();
	return 1;
}

int SIFRA_Manager::SIFRAMsg_debugAskedRamHandler()
{
	m_SIFRAProtocol->sendSIFRADebugRam();
	return 1;
}

int SIFRA_Manager::SIFRAMsg_startAcquisitionHandler()
{
	m_startLoadingSamples = True;
	return 1;
}

/**
* Manager of the command board jump to loader.

* The command is received and decoded from the class AcqHrProtocol which sends a message to the board manager.
* @return always 1
*/
int SIFRA_Manager::SIFRAMsg_jumpToLoaderHandler()
{
	DecTimer wait;

	wait.Preset(500);
	while(!wait.Match());

	while (m_SIFRAProtocol->GetNumBytesInTransmissionBuffer() > 0)
	{
		wait.Preset(_100_mSEC_);
		while(!wait.Match());
	}
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");

	stay_in_loader_flag0 = 0xA5;
	stay_in_loader_flag1 = 0xA5;
	start_on_course = 0x55;		// reset del flag di mancanza di corrente

	fp = (void (*)()) LOADER_FLASH_START_ADDRESS;				// inizio dell'applicone
    	(*fp)();

	return 1;
}

/**
* Reset application and restarts system automatically
* @return always 1
*/
int SIFRA_Manager::SIFRAMsg_resetApplicationHandler()
{
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");

	restart_command = 0xA5;	// salvo in ram tamponata il comando di reset ricevuto da tastiera

	fp = (void (*)()) __APPLICATION_START_ADDRESS__;	// inizio dell'applicazione
    	(*fp)();

	return 1;
}

int SIFRA_Manager::SIFRAMsg_hwErrorHandler()
{
	StatusCmd.commErrorHw++;
	return 1;
}

int SIFRA_Manager::SIFRAMsg_protocolErrorHandler()
{
	StatusCmd.commErrorProtocol++;
	return 1;
}

int SIFRA_Manager::SIFRAMsg_unknownErrorHandler()
{
	StatusCmd.commErrorUnknown++;
	return 1;
}

/**
* Resetto le strutture e variabili di sistema
*/
void SIFRA_Manager::SifraResetSystemParams()
{
	SifraResetStatus();
	SifraResetStartCmd();
	SifraResetFillingData();
	SifraResetViaStatus();
	SifraResetStateMachine();
	ResetMotorStepCounter();
	SifraStopTimers();
}

/**
* Resetto le strutture Via del sistema
*/
void SIFRA_Manager::SifraResetViaStatus()
{
	byte i;
	
	for(i = 0; i < NUM_MAX_LINE; i++)
	{
		Via[i].abilitazione = False;
		Via[i].da_eseguire = False;
		Via[i].eseguita = False;
		Via[i].firstErogDone = False;
		Via[i].rifinituraDone = False;

		Via[i].stato_led = OFF;
		Via[i].fase_luci = 0;
		Via[i].stato_luci = IDLE_LUCE;
		
		Via[i].peso_da_erogare = 0;
		Via[i].peso_erogato = 0;
		Via[i].peso_gia_erogato = 0;
		Via[i].peso_stop = m_peso_attuale[i];
		Via[i].tara = 0;
		Via[i].peso_spec = 0;
		Via[i].flusso_pompa = 0;
		Via[i].restart_bag = False;
		
		Via[i].vel = _PWM_CYCLE_LOW;
		
		Via[i].timer_rampa.Stop();
		Via[i].timer_check_block_pump.Stop();
		Via[i].timerVerificaCalo.Stop();
	}
}

/**
* Resetto lo stato del sistema e spengo i led della scheda luci
*/
void SIFRA_Manager::SifraResetStatus()
{
	byte i;

	LedDrivers_clearFifo();
	switchoff_leds();		// spengo i led

	// resetto la struttura StatusCmd
	StatusCmd.status = STATO_IDLE;		// resetto stato del sistema
	StatusCmd.led_aria_en = False;				// in caso di aria, il led rosso non deve lampeggiare
	StatusCmd.fillingOnCourse = False;
	for(i = 0; i < _MAX_LOAD_CHAN_;i++)
	{
		StatusCmd.air_block_en[i]  = False;		// disattivazione blocco x aria
	}
	StatusCmd.cover_open = False;

}

/**
* Resetto la struttura StartCmd del sistema
*/
void SIFRA_Manager::SifraResetStartCmd()
{
	byte i;

	StartCmd.function = 0;
	StartCmd.support = 0;

	for(i = 0; i < _MAX_LOAD_CHAN_;i++)
	{
		StartCmd.tara_linea[i] = 0;
		StartCmd.peso_linea[i] = 0;
		StartCmd.peso_spec[i] = 0;
	}

}

/**
* Resetto le variabili che gestiscono le varie macchine a stati presenti nel sistema
*/
void SIFRA_Manager::SifraResetStateMachine()
{
	byte i;
	
	for(i = 0; i < _MAX_LOAD_CHAN_;i++)
	{
		m_air_manager[i] = E_AIR_MGR_START;					// variabile usata per controllo allarmi aria
		setStatePump( i, POMPA_DISABILITA);	//variabile usata per controllo stato pompe
		stab_error[i] = init_stab;				//variabile usata nell'algoritmo del peso instabile
	}
	
	m_stato_check_blocco = verifica_contatore;	// variabile usata per controllo blocco lettura ADC
	m_stato_test_emc = E_EMC_NONE;			// variabile usata per gestire la macchina a stati nel test EMC
	g_stateMachine.calibState = IdleCalib;					// variabile usata per gestire la macchina a stati in calibrazione
	g_stateMachine.rtState = IdleRiempitubi;			// variabile usata per gestire la macchina a stati in svuotamento linee, riempimento linee e riempimento manuale
	g_stateMachine.saccaState = SaccaIdle;					// variabile usata per gestire la macchina a stati in sacca
	g_stateMachine.serviceState = IdleService;				// variabile usata per gestire la macchina a stati in service per i test di flusso, flusso pompe e service
	m_test_led = E_LED_STOP;

}

/**
* Resetto le variabili associate ad un riempimento automatico
*/
void SIFRA_Manager::SifraResetFillingData()
{

	byte i;

	// variabili usate in SaccaManager
	m_enable_line = -1;

	// variabili usate in algoritmo peso instabile
	for(i=0;i<_MAX_LOAD_CHAN_;i++)
	{
		m_valore_peso_stimato[i] = 0;
		//m_void_source[i] = False;
		m_peso_erog_by_hand[i] = 0;
	}
	
}

/**
* Resetto la struttura dati di backup
*/
void SIFRA_Manager::SifraResetBackup()
{
	byte i;

	bk.start_cmd = 0;
	bk.vie_da_eseguire = bk.vie_eseguite =  0x0000;
	bk.num_lines = 0;

	for(i = 0; i < NUM_MAX_LINE; i++)
	{
		bk.abilitazione[i] = False;
		bk.peso_erogato[i] = 0;
	}	
}

/**
* Resetto i timer associati a controlli fatti in sacca o in service
*/
void SIFRA_Manager::SifraStopTimers()
{
	m_timerEndSeq.Stop();
	m_emc_motor_timer.Stop();
	m_verify_enc_timer.Stop();
	m_timer_enc_measure.Stop();
}

/**
* It reads adc value when cell is not charged. With this value, it will calculate offset for this adc, then saved in EEPROM.
* It works in two way. In CALIBRAZIONE DI FABBRICA, it reads, calculates and backups if difference of value calibration is below 5% of previous factory value.
* In CALIBRAZIONE UTENTE calculated value is matched with factory value: in case of difference greater of 3% it sets error.
*/
bool SIFRA_Manager::readZeroInThisLine(int chan)
{
	if(StatusCmd.status == STATO_CALIBRAZIONE_FABBRICA)
	{
		if((int)m_WeightSample[chan] > _DELTA_ADC_1_PERC_FS_)	// ci deve essere qualcosa sulla cella, metto soglia pari al 1% del fondo scala
		{
			if(Chan[chan].AreCalibrate)
			{
				if(((int)m_WeightSample[chan] > ((int)Chan[chan].WeightFactoryOffset - _DELTA_ADC_0Kg_DEF)) &&
						((int)m_WeightSample[chan] < ((int)Chan[chan].WeightFactoryOffset + _DELTA_ADC_0Kg_DEF)))	
				{
					weightChan->setOffset(chan, (word)m_WeightSample[chan]);
					Chan[chan].WeightFactoryOffset = Chan[chan].Weightoffset = (word)m_WeightSample[chan];
					backup_factory_offset_value(chan, Chan[chan].Weightoffset);
					return True;
				}
				else
				{
					StatusCmd.log_error = ERR_CASE_FACTORY_CALIB_OFFSET_OUT;
				}
			}
			else
			{
				weightChan->setOffset(chan, (word)m_WeightSample[chan]);
				Chan[chan].WeightFactoryOffset = Chan[chan].Weightoffset = (word)m_WeightSample[chan];
				backup_factory_offset_value(chan, Chan[chan].Weightoffset);
				return True;
			}
		}
		else
		{
			StatusCmd.log_error = ERR_CASE_FACTORY_CALIB_OFFSET_LOW;
		}
	}
	else
	{
		if(((int)m_WeightSample[chan] > ((int)Chan[chan].WeightFactoryOffset - _DELTA_ADC_0Kg)) &&
						((int)m_WeightSample[chan] < ((int)Chan[chan].WeightFactoryOffset + _DELTA_ADC_0Kg)))	
		{
			weightChan->setOffset(chan, (word)m_WeightSample[chan]);	//weightChan->setOffset(chan, (long)m_WeightSample[chan]);
			Chan[chan].Weightoffset = (word)m_WeightSample[chan];
			backup_new_offset_value(chan, Chan[chan].Weightoffset);
			return True;
		}
		else
		{
			StatusCmd.log_error = ERR_CASE_CALIB_OFFSET_OUT;
		}
	}
	
	return False;	// non è riuscito a calibrare (valore troppo sballato)
	
}

/**
* It reads adc value when user charges with 2kg sample weight. With this value, it will be calculated gain for this adc, then saved in EEPROM.
* It works in two way. In CALIBRAZIONE DI FABBRICA, it reads, calculates and backups if difference of value calibration is below 5% of previous factory value.
* In CALIBRAZIONE UTENTE calculated value is matched with factory value: in case of difference greater of 3% it sets error.
*/
bool SIFRA_Manager::readLoadInThisLine(int chan)
{
	float this_gain;

	this_gain = (2000.0 /(float)((int)m_WeightSample[chan] - (int)weightChan->getOffset(chan)));	

	if(StatusCmd.status == STATO_CALIBRAZIONE_FABBRICA)
	{
		if((int)m_WeightSample[chan] > _DELTA_ADC_1_PERC_FS_)	// ha misurato qualcosa di diverso da zero, come soglia mettiamo l'1% del fondo scala della cella di carico
		{
			if(Chan[chan].AreCalibrate)
			{
				if(((int)m_WeightSample[chan] > ((int)Chan[chan].AdcOf2Kg - _DELTA_ADC_0Kg_DEF)) &&
						((int)m_WeightSample[chan] < ((int)Chan[chan].AdcOf2Kg + _DELTA_ADC_0Kg_DEF)))
				{
					Chan[chan].WeightFactoryGain = Chan[chan].Weightgain = this_gain;
					Chan[chan].AdcOf2Kg = (word)m_WeightSample[chan];
					weightChan->setGain(chan, this_gain);
					weightChan->set2KgValue(chan, (long)m_WeightSample[chan]);
					backup_factory_gain_param(chan, (float)this_gain, (word)m_WeightSample[chan]);
					return True;
				}
				else
				{
					StatusCmd.log_error = ERR_CASE_FACTORY_CALIB_GAIN_OUT;
				}
			}
			else
			{
				Chan[chan].WeightFactoryGain = Chan[chan].Weightgain = this_gain;
				Chan[chan].AdcOf2Kg = (word)m_WeightSample[chan];
				weightChan->setGain(chan, this_gain);
				weightChan->set2KgValue(chan, (long)m_WeightSample[chan]);
				backup_factory_gain_param(chan, (float)this_gain, (word)m_WeightSample[chan]);
				return True;
			}
		}
		else
		{
			StatusCmd.log_error = ERR_CASE_FACTORY_CALIB_GAIN_LOW;
		}
	}
	else
	{
		if(m_SIFRAProtocol->sendSIFRALoadSamples(m_WeightSample[chan], chan) > _MIN_PESO_GAIN_CALIB)	// ha misurato qualcosa di superiore a 1500g
		{
			if(((int)m_WeightSample[chan] > ((int)Chan[chan].AdcOf2Kg - _DELTA_ADC_0Kg_DEF)) &&
					((int)m_WeightSample[chan] < ((int)Chan[chan].AdcOf2Kg + _DELTA_ADC_0Kg_DEF)))
			{
				Chan[chan].Weightgain = this_gain;
				//Chan[chan].AdcOf2Kg = (word)m_WeightSample[chan];
				weightChan->setGain(chan, this_gain);
				//weightChan->set2KgValue(chan, (long)m_WeightSample[chan]);
				backup_new_gain_value(chan, (float)this_gain);
				return True;
			}
			else
			{
				StatusCmd.log_error = ERR_CASE_CALIB_GAIN_OUT;
			}
		}
		else
		{
			StatusCmd.log_error = ERR_CASE_CALIB_GAIN_LOW;
		}
	}
	
	return False;
	
}


/**
* Sets board's hardware version, reading two bits of cpld
*/
bool SIFRA_Manager::SetHardwareVersion()
{
	// fa una lettura sulla PLD per vedere quale valore è settato
	// 0x01 = M3100; 0x11 = M3300
	byte hw_set = 0;

	hw_set = SIFRA_Manager::readHardwareVersion();
	if( hw_set != 0x00)
	{
		switch(hw_set)
		{
			case 0x01:
				m_SIFRAProtocol->setLocalNODEID(M3100);
				break;
			case 0x03:
				m_SIFRAProtocol->setLocalNODEID(M3300);
				break;
		}
		return True;
	}

	return False;
	
}

/**
* Set a error occourred in eeprom reading
*/
void SIFRA_Manager::setErrorEEpromReading()
{
	// per ora non gestito, mi limito a settare un errore generico 
	StatusCmd.errors = ERR_EEPROM;
}


/**
* Esegue il backup continuo dello stato in start (o in stop o errore durante un riempimento).
* Salvando i dati nella ram tamponata, che può essere scritta senza limiti di massima.
* 3600scritture/h x 6h/d x 6d/w x 45w/h x 10h
*/
bool SIFRA_Manager::BackUp_StatoStart()
{
	byte i;

	bk.start_cmd = StartCmd.support;
	bk.vie_da_eseguire = bk.vie_eseguite =  0x0000;

	for(i = 0; i < NUM_MAX_LINE; i++)
	{
		if(Via[i].da_eseguire == True)
		{
			bk.vie_da_eseguire |= (0x0001 << i);

		}
		if(Via[i].eseguita == True)
		{
			bk.vie_eseguite |= (0x0001 << i);
		}
		bk.abilitazione[i] = Via[i].abilitazione;
		bk.peso_erogato[i] = Via[i].peso_erogato;
		bk.peso_stop[i] = Via[i].peso_stop;
	}

	bk.num_lines = m_num_enabled_lines;

	if(!Write_Param_Backup())
	{
		//StatusCmd.flagsErrorLine = 0x01;
		//StatusCmd.errors = ERR_RAM_TAMPONATA;
		StatusCmd.log_error = ERR_CASE_SAVE_BK_DATA;
		//return False;
	}

	return True;
	
}

/**
* In caso di ripresa dopo mancanza rete, scarica il backup dello stato pre-stop che abbinato ai dati salvati dalla tastiera, permettono di riprendere
*/
bool SIFRA_Manager::Carica_StatoStart()
{
	byte i;
	byte enable_line = 0;
	
	if(!Read_Param_Backup())
	{
		//StatusCmd.flagsErrorLine = 0x01;
		//StatusCmd.errors = ERR_RAM_TAMPONATA;
		StatusCmd.log_error = ERR_CASE_READ_BK_DATA;
		//return False;
	}

	for(i = 0 ; i < NUM_MAX_LINE; i++)
	{
		if((bk.vie_da_eseguire & (0x0001 << i)) > 0)
		{
			Via[i].da_eseguire = True;
			enable_line++;
		}
		if((bk.vie_eseguite & (0x0001 << i)) > 0)
		{
			Via[i].eseguita = True;
		}
		Via[i].abilitazione 		= bk.abilitazione[i];
		Via[i].peso_erogato 	= bk.peso_erogato[i];
		Via[i].peso_stop		= bk.peso_stop[i];
	}
	m_num_enabled_lines = enable_line;
	StatusCmd.status = STATO_STOP;

	return True;
	
}

/**
* Salva in ram tamponata i valori della struttura backup.
* Se la scrittura non va, restituisce false.
*/
bool SIFRA_Manager::Write_Param_Backup()
{
	bool chkOk;
	bool writeOk;

	writeOk = nvr_writeParam(&bk, sizeof(BK_struct), OFS_POINTS_SET, &chkOk);
	if( writeOk == False )          // se non ce la fa
	{
		writeOk = nvr_writeParam(&bk, sizeof(BK_struct), OFS_POINTS_SET, &chkOk);
		if( writeOk == False )          // se non ce la fa
		{
			return False;
		}
	}

	return True;

}

/**
* Legge dalla ram tamponata i valori memorizzatti della struttura backup.
* Se la lettura fallisce 2 volte (problemi di lettura o checksum incoerente) torna False.
*/
bool SIFRA_Manager::Read_Param_Backup()
{
	bool readOk;
	bool chkOk;

	readOk = nvr_readParam(&bk, sizeof(BK_struct), OFS_POINTS_SET, &chkOk);
   	if( readOk == False )               // se non ci sono calibrazioni valide per questo canale
    	{
    		readOk = nvr_readParam(&bk, sizeof(BK_struct), OFS_POINTS_SET, &chkOk);
   		if( readOk == False )               // se non ci sono calibrazioni valide per questo canale
   		{
			return False;
   		}
   	}

	return True;
	
}

/**
* Salva in ram tamponata i valori user dei parametri flusso e tolleranza.
* Se la scrittura non va, restituisce false. Il dato attuale sarà usabile solo in questa sessione e al riavvio sucessivo saranno caricati i valori nominali
*/
bool SIFRA_Manager::Backup_Param_Encoder(FP_STRUCT *this_struct, int m_line)
{
	bool writeOk, chkOk;
	word offset_for_this_struct;	// si sposta di 2 volte la dimensione della struttura + 2 volte la dimensione del CHK

	offset_for_this_struct = OFFSET_VAL_ENC + 2*(sizeof(FP_STRUCT) + sizeof(word))*m_line;

	writeOk = nvr_writeParam(this_struct, sizeof(FP_STRUCT),offset_for_this_struct, &chkOk);
	if( writeOk == False )          // se non ce la fa
	{
		writeOk = nvr_writeParam(this_struct, sizeof(FP_STRUCT),offset_for_this_struct, &chkOk);
		if( writeOk == False )          // se non ce la fa
		{
			return False;
		}
	}
	
	return True;
	
}

/**
* Legge dalla ram tamponata i valori memorizzatti dei rapporti flusso per i due tipi di linea per ogni pompa.
* Se la lettura fallisce 2 volte (problemi di lettura o checksum incoerente) torna False: in tal caso vengono caricati i dati di default
*/
bool SIFRA_Manager::Carica_Param_Encoder(FP_STRUCT *this_struct, int m_line)
{
bool readOk, chkOk;
int offset_for_this_struct;	// si sposta di 2 volte la dimensione della struttura + 2 volte la dimensione del CHK

	offset_for_this_struct = OFFSET_VAL_ENC + 2*(sizeof(FP_STRUCT) + sizeof(word))*m_line;

	readOk = nvr_readParam(this_struct, sizeof(FP_STRUCT), offset_for_this_struct, &chkOk); // scrive nei 180byte successivi a 0x018002A
	if( readOk == False )               // se ho fallito in lettura, ci riprovo almeno un'altra volta
	{
		readOk = nvr_readParam(this_struct, sizeof(FP_STRUCT), offset_for_this_struct, &chkOk);
	   	if( readOk == False )               // se non ci sono calibrazioni valide per questo canale
	   	{
			return False;
	   	}
	}
	
	return True;
	
}

/**
* Salvo i parametri di ultimo tipo di linea caricata (adulti o pediatrica) e variabile che mi indica se c'è una sacca sospesa
*/
bool SIFRA_Manager::Backup_Param_Restart(Restart_t *this_struct)
{
	bool writeOk, chkOk;

	writeOk = nvr_writeParam(this_struct, sizeof(Restart_t),OFS_RESTART_SET, &chkOk);
	if( writeOk == False )          // se non ce la fa
	{
		writeOk = nvr_writeParam(this_struct, sizeof(Restart_t),OFS_RESTART_SET, &chkOk);
		if( writeOk == False )          // se non ce la fa
		{
			return False;
		}
	}
	
	return True;
	
}

/**
* Legge dalla ram tamponata i valori memorizzatti di linea e spegnimento con sacca in corso.
* Se la lettura fallisce 2 volte (problemi di lettura o checksum incoerente) torna False: in tal caso vengono caricati i dati di default
*/
bool SIFRA_Manager::Carica_Param_Restart(Restart_t *this_struct)
{
	bool readOk, chkOk;

	readOk = nvr_readParam(this_struct, sizeof(Restart_t), OFS_RESTART_SET, &chkOk); // scrive nei 180byte successivi a 0x018002A
	if( readOk == False )               // se ho fallito in lettura, ci riprovo almeno un'altra volta
	{
		readOk = nvr_readParam(this_struct, sizeof(Restart_t), OFS_RESTART_SET, &chkOk);
	   	if( readOk == False )               // se non ci sono calibrazioni valide per questo canale
	   	{
			return False;
	   	}
	}
	
	return True;
	
}

/**
* Utilizzata per stabilire quale linea è montata sul modulo M3100. Come parametri utilizza il numero di passi encoder ed il volume erogato in quest'ultima sessione.
* La differenza fra i rapporti encoder/decimi di ml fra le due linee è tale che mi basta mettere una soglia a metà fra i due valori per discriminare
* Si parte dalla prima via eseguita nel RT del M3100.
* Se tutto va bene:
	1 - calcolo rapporto peso erogato / passi encoder
	2 - confronto con i valori nominali
	3 - scelta linea --> eventuale modifica del valore tipo_linea
* Al termine dell'esecuzione della/delle eventuali vie aggiutnive, si ripete il calcolo e si esegue una verifica.
* In caso di mismatch, si accetta il risultato calcolato dalla via con peso_erogato maggiore.
*/
byte SIFRA_Manager::Verifica_linea_montata(int m_line, int peso_rif)
{
	word delta_vol;
	float cont;

	if(peso_rif >= m_peso_attuale[m_line])
	{
		delta_vol = (word)(peso_rif - m_peso_attuale[m_line]);	// delta peso riferito a questo specifico step_motor_done
	}
	else
	{
		delta_vol = 0;
	}

	cont = (float)MotorStatus[m_line].step_motor_done / (float)delta_vol;				// calcolo il rapporto
	
	if( cont > ((param_encoder_nom[_ADULTI_] + param_encoder_nom[_PEDIATRICA_]) / 2)) 		// uso i valori nominali per le linee
	{
		return _PEDIATRICA_;					// setto linea pediatrica
	}
	else
	{
		return _ADULTI_;					// setto linea adulti
	}
}

/**
* Calcola il rapporto passi_encoder/delta peso e lo salva in ram tamponata ma solo se il delta peso ( e quindi anche il delta encoder) è superiore ad una soglia,
* per evitare che i piccoli errori siano troppo influenti
* linea = 0 per linea adulti; linea = 1 per linea pediatrica
*/
bool SIFRA_Manager::salva_rapporto_flusso(int m_line, int peso_rif, int peso_fin, byte linea)
{
	word delta_peso;
	float calc;
	float soglia = 0;

	delta_peso = (word)(peso_rif - peso_fin);			// differenza fra peso iniziale e finale espresso in decigrammi, solitamente quindi un numero fra 400 e 500
	
	if(delta_peso > vol_calcolo_enc[linea])		// se i valori sono riferimento di un riempitubi completo, ok, altrimenti valori troppo piccoli
	{
		calc = (float)((float)MotorStatus[m_line].step_motor_done / (float)delta_peso);		// aggiorno il valore del rapporto, circa 27200
		soglia = param_encoder_nom[linea] * _PERC_ENC_CONTROL;					// soglia massima del 35% di differenza (imputabile alla tolleranza della linea)
		if(abs(param_encoder_nom[linea] - calc) < soglia) 							// rapporto differisce meno del 35% dal nominale
		{	// per evitare di calibrare con la linea sbagliata
			g_encStruct[m_line].param_encoder[linea] = calc;							// lo posso considerare valido e aggiorno il valore
			return True;
		}
	}
		
	return False;
	
}

/**
* Se è il caso, inizializza la struttura parametri flusso peso con i valori nominali e di default
* Chiamata all'avvio del programma se fallisce il recupero dei dati dalla RAM tamponata
*/
void SIFRA_Manager::default_struttura_flusso(int m_line)
{
	byte i;
	
	for(i = 0; i < _DIM_PAR_ENC; i++)
	{
		g_encStruct[m_line].param_encoder[i] = param_encoder_nom[i]; // carico i valori nominali
		g_encStruct[m_line].tolleranza[i] = m_tolleranza[_ALTA_TOLL];		// tolleranza del 15% sulla verifica del flusso peso
	}
}

/**
* Se è il caso, inizializza la struttura parametri restart con i valori nominali e di default
* Chiamata all'avvio del programma se fallisce il recupero dei dati dalla RAM tamponata
*/
void SIFRA_Manager::default_struttura_restart()
{
	g_restart.tipo_linea = LINEA_ADULTI;
	g_restart.restart_bk = E_NO_BACKUP;
}

byte SIFRA_Manager::changeSIFRAstatus(byte newStatus)
{
	if(newStatus != StatusCmd.state)
	{
		StatusCmd.prevState = StatusCmd.state;
		StatusCmd.state = newStatus;
	}
	return 0;
}

byte SIFRA_Manager::getSIFRAstatus()
{
	return StatusCmd.state;
}

byte SIFRA_Manager::getSIFRAphase()
{
	return StatusCmd.phase;
}

/**
* Durante il riempimento si verifica continuamente lo stato del flacone per proteggersi dal vuoto e l'effettivo calo del peso
* per verificare ostruzioni della linea
*/
byte SIFRA_Manager::controllo_errori_linea(int line, int delta)
{
	byte error = 0x00;
	int threshold;

	threshold = calcolo_margine_tara(line, MARGINE_SU_VUOTO);	//calcolo il margine per segnalare in tempo l'errore di flacone VUOTO
	if(m_peso_attuale[line] < threshold)
	{
		error |= (0x01 << line);	// setto un errore
		StatusCmd.statusChan[line] |= ERR_RILEV_VUOTO;	// non c'è abbastanza liquido
		m_void_source[line] = True;
	}
	if(Via[line].timerVerificaCalo.Match())
	{
		if(m_peso_attuale[line] < (m_peso_precedente[line] - delta))
		{
			Via[line].timerVerificaCalo.Preset(T_NO_CALO);
			m_peso_precedente[line] = m_peso_attuale[line];
		}
		else
		{
			error |= (0x01 << line);	// setto il bit dell'errore sulla via m_line-esima
			StatusCmd.statusChan[line] |= ERR_NON_CALA;
		}
	}
	
	return error;
	
}

/*
* Calcolo il margine relativo alla tara al di sotto della quale deve scattare l'allarme VUOTO
*/
word SIFRA_Manager::calcolo_margine_tara(int line, word delta)
{
	word threshold = 0;

	threshold = (10 * Via[line].tara) + (10 * delta);			// sommo al valore di tara impostato un margine sempre in decimi di grammo
		
	return threshold;
	
}

/**
* scandaglia il comando start ricevuto da tastiera per settare le vie abilitate
* ogni via abilitata ha "da_eseguire" settato e peso_già_erogato resettato
* sono contate le vie complessivamente abilitate, escluse però quelle con errore continuo in corso
*/
int SIFRA_Manager::Find_Num_of_Enabled_Lines(byte command)
{
int i;
int m_enable = 0;								// azzero il numero delle vie da eseguire

	for(i = 0; i < NUM_MAX_LINE; i++)			// scorrendo sulle possibili vie, qui va bene NUM_MAX_LINE
	{
		if(command & (1 << i)) // se questa via è abilitata
		{
			Via[i].da_eseguire = True;	// la via fa parte della sequenza
			m_enable++;		// incremento il numero delle vie da eseguire
		}
		else
		{
			Via[i].da_eseguire = False;	// la via non fa parte della sequenza
		}
	}
	
	if(m_enable > NUM_MAX_LINE)
	{
		m_enable = NUM_MAX_LINE;	// numero massimo 8
	}
	
	return m_enable;
}

/**
* cerca il numero della prima via trovata abilitata,
* ritorna numero che va da 0 a 7 se almeno una via abilitata
* ritorna -1 se nessuna via abilitata
*/
int SIFRA_Manager::Find_Enable_line(byte command)
{
int i;
	for(i = 0; i < NUM_MAX_LINE; i++)	// scorrendo sulle possibili vie, qui va bene NUM_MAX_LINE
		if(command & (1 << i)) // se questa via è abilitata
		{
			return i;	// ritorna il numero della via abilitata da 0 a (NUM_MAX_LINE - 1)
		}
	return -1;	// se non trova linee abilitate
}

/**
* Invia i dati aggiornati di peso erogato e al tempo stesso aggiorna la variabile che deve essere backuppata periodiocamente
* Esegue solo sulle vie abilitate
*/
void SIFRA_Manager::Aggiorna_pesi_erogati()
{
	byte m_line;

	for(m_line = 0; m_line < NUM_MAX_LINE; m_line++)	// al pirmo allarme aria, blocca tutto, backuppa i dati e mette in pausa
	{
		if(Via[m_line].abilitazione == True)
		{
			if((m_peso_iniziale[m_line] - m_peso_attuale[m_line]) > 0)
			{
				Via[m_line].peso_erogato = ((word)(m_peso_iniziale[m_line] - m_peso_attuale[m_line])) + Via[m_line].peso_gia_erogato;
			}
		}
	}
}

/**
* Verifica periodica dello stato delle celle.
* La variabile m_WeightSample con cella collegata e scarica (solo la culla vuota presente) assume per le 5 vie valori compresi tra 4500 e 12000.
* Quando la cella è scollegata il valore di m_WeightSample è 65535.
* Se la cella è rotta il valore di m_WeightSample può assumere valore 0x0000 (0) oppure 0xFFFF (65535).
*/
bool SIFRA_Manager::Verifica_celle()
{
	byte line;
	bool result = False;

	for(line = 0; line < _MAX_LOAD_CHAN_; line++)
	{
		if(m_TimerChkHw[line].Match())
		{
			// verifica della rottura delle celle o del distacco dei cavi cella
			if((g_adc_real_value[line] == 0x0000) || (g_adc_real_value[line] == 0xFFFF))
			{
				StatusCmd.statusChan[line] |= ERR_CELLA_ROTTA;		// set errore cella rotta (a zero o a fondoscala)
				StatusCmd.flagsErrorLine |= (0x01 << line);		// setto un errore nella via m_chan
				Via[line].stato_luci = ERR_CELLA;
				m_TimerChkHw[line].Stop();
				m_TimerMaxWeight[line].Stop();
				result = True;
			}
			else
			{
				if(g_adc_real_value[line] < _TH_VAL_ADC_CELL_DIS_)
				{
					StatusCmd.statusChan[line] |= ERR_CELLA_ROTTA;		// set errore cella rotta (a zero o a fondoscala)
					StatusCmd.flagsErrorLine |= (0x01 << line);		// setto un errore nella via m_chan
					Via[line].stato_luci = ERR_CELLA;
					m_TimerChkHw[line].Stop();
					m_TimerMaxWeight[line].Stop();
					result = True;
				}
				else
				{
					m_TimerChkHw[line].Preset(_TIMEOUT_CHKHW);
				}
			}
		}
	}
	
	return result;
	
}

/**
* Verifica periodica del peso massimo caricato sulle culle.
*/
bool SIFRA_Manager::Verifica_peso_massimo()
{
	byte line;
	bool result = False;
	
	for(line = 0; line < _MAX_LOAD_CHAN_; line++)
	{	
		// verifica della rottura delle celle o del distacco dei cavi cella
		if(m_TimerMaxWeight[line].Match())
		{
			// se la cella è scollegata o non funzionante non eseguo il controllo, è già scattato o scatterà l'allarme di cella guasta
			if((g_adc_real_value[line] == 0x0000) || (g_adc_real_value[line] == 0xFFFF))
			{
				m_TimerMaxWeight[line].Preset(_3_SEC_);
				return False;
			}

			if(g_adc_real_value[line] < _TH_VAL_ADC_CELL_DIS_)
			{
				m_TimerMaxWeight[line].Preset(_3_SEC_);
				return False;
			}
			
			// veririfca del sovraccarico sulla cella
			if(m_peso_attuale[line] > PESO_MASSIMO)
			{
				StatusCmd.statusChan[line] |= ERR_PESO_MASSIMO;
				StatusCmd.flagsErrorLine = 0x01;
				Via[line].stato_luci = ERR_CELLA;
				m_TimerMaxWeight[line].Stop();
				m_TimerChkHw[line].Stop();
				result = True;
			}
			else
			{
				m_TimerMaxWeight[line].Preset(_3_SEC_);
			}
		}
	}

	return result;
	
}

/**
* Aggiorna i contatori degli encoder motori.
* Se i motori sono in moto (se il timer timer_check_block_pump sta contando) allora verifica anche eventuali blocchi.
* Se riscontra un blocco per ora attiva l'allarme pompa non gira.
* un giro pompa == 6 giri motore == 3000 impulsi encoder
*/
void SIFRA_Manager::Verifica_encoder_motori()
{
	int vel_mot;
	byte i;
	word delta;
	word soglia_step;

	m_enc_value[0] = cpld_counter1 & _MASK_16BIT_;
	m_enc_value[1] = cpld_counter2 & _MASK_16BIT_;
	m_enc_value[2] = cpld_counter3 & _MASK_16BIT_;
	m_enc_value[3] = cpld_counter4 & _MASK_16BIT_;
	m_enc_value[4] = cpld_counter5 & _MASK_16BIT_;

	for(i = 0; i < _MAX_LOAD_CHAN_; i++)
	{
		MotorStatus[i].partial_step = encoder_updateCnt(i);						// aggiorna valore passi encoder
		if(Via[i].timer_check_block_pump.Match())			// temporizzazione per periodicizzare il controllo
		{
			Via[i].timer_check_block_pump.Preset(_2_SEC_);
			m_old_step[i] = m_this_step[i];			// calcolo il delta passi fra i due controlli.
			m_this_step[i] = MotorStatus[i].step_motor_done;
			if(m_this_step[i] > m_old_step[i])
			{
				delta = m_this_step[i] - m_old_step[i];
			}
			else
			{
				delta = 0;
			}
			if(Via[i].abilitazione)					// la verifica è fatta solo sulle vie abilitate, cioè durante le esecuzioni
			{
				vel_mot = getSpeedPump(i);		// si legge la velocità impostata alle pompa
				m_waiting_step[i] = (word)(vel_mot * _SOGLIA_ENC_NON_GIRA);
				if(m_waiting_step[i] > 0)				// valore atteso di delta passi data una certa velocità pompa
				{					
					soglia_step = (word)((float)m_waiting_step[i] * _PERC_SOGLIA_ENCODER); // soglia minima per scongiurare un blocco o un inatteso freno al motore
					if(delta < soglia_step)			// se questa condizione è vera, il motore è bloccato o frenato, oppure c'è un guasto elettrico
					{
						StatusCmd.flagsErrorLine |= (0x01 << i);	// setto il bit dell'errore sulla via m_line-esima
						StatusCmd.statusChan[i] |= ERR_POMPA_NON_GIRA;
					}
				}
			}
			else	// se il motore dovrebbe essere spento, se vedo variazioni lo blocco
			{
				if(abs(delta) > _SOGLIA_ENC_MIN)			// il motore si sta muovendo anche se non abilitato
				{
					setStatePump(i, ARRESTO);		// setta lo stato di arresto pompa, il pwm manager si occuperà del resto
					setBlockRelay(i, 0);
					setPwmMotCycle(i, 0);			// setta il pwm a zero
				}
			}
		}
	}
}

void SIFRA_Manager::Setta_filtro_peso(int val)
{
byte i;
	for(i = 0; i < _MAX_LOAD_CHAN_; i++)	// scorrendo sulle possibili vie, qui va bene NUM_MAX_LINE
		weightChan->set_filter_value(i, val);		// settaggio del filtro di media sul peso: 5 sample --> 60Hz/5 = circa 12Hz
}

/**
* Setta le velocità massime delle vie in sacca, nelle sequenze multi via, in modo che le pompe vadano assieme anche a fronte di volumi molto differenti fra loro.
* In questo modo, regolando la velocità massima di ogni via in funzione della via con massimo volume, tutte le vie dovrebbero arrivare
* alla fine più o meno nello stesso momento, garantendo il miscelamento ottimale della soluzione.
*/
void SIFRA_Manager::calcola_velocita_massime()
{
	word p_m[NUM_MAX_LINE];
	word p_m_max = 0;
	byte i;
	byte i_max = 0;
	float rapporto_vel[NUM_MAX_LINE];

	for(i = 0; i < NUM_MAX_LINE; i++)
	{
		Via[i].vel = _PWM_CYCLE_LOW;
		if((Via[i].abilitazione ==  True) && (Via[i].eseguita == False))	
		{
			if(m_peso_attuale[i] >= m_peso_finale[i])
			{
				p_m[i] = (word)(m_peso_attuale[i] - m_peso_finale[i]);	// peso ancora da erogare (decimi di grammo)
			}
			else
			{
				p_m[i] = 0;
			}
			if(p_m[i] > p_m_max)	// peso ancora da erogare (decimi di grammo)
			{
				p_m_max = p_m[i];
				i_max = i;
			}
		}
		else
		{
			p_m[i] = 0;
		}
	}
	if(i_max < NUM_MAX_LINE)	// cioè solo se il conto sopra è andato a buon fine
	{
		for(i = 0; i < NUM_MAX_LINE; i++)
		{
			if((Via[i].abilitazione ==  True) && (Via[i].eseguita == False))	// && (Via[m_line].obiettivo == False))
			{
				if(i == i_max)
				{
					vel_max[i] = _PWM_CYCLE_MAX;
				}
				else
				{
					if(p_m[i] > 0)
					{
						if(p_m_max > 0)
						{
							rapporto_vel[i] = ((float)p_m[i])/((float)p_m_max);
							vel_max[i] = (int)(rapporto_vel[i] * _PWM_CYCLE_MAX);
						}
						// ora un fattore di correzione: calo la velocità in funzione di quanto è diversa dalla massima
						if(vel_max[i] > _PWM_CYCLE_CORRECTION)
						{
							vel_max[i] = vel_max[i] -_PWM_CYCLE_CORRECTION;		//vel_max[i] -= (vel_max[i] / 10);
						}
						else
						{
							vel_max[i] = 0;
						}
						if(vel_max[i] < _PWM_CYCLE_LOW)
						{
							vel_max[i] = _PWM_CYCLE_LOW;
						}
					}
					else
					{
						vel_max[i] = 0;	// se non è da eseguire, ci assicurazimo di resettare il valore
					}
				}
			}
			else
			{
				vel_max[i] = 0;
			}
		}
	}
}

/**
* Frequenza encoder massima = 10,4KHz
* Si basa sull'uso di contatori a 14bit (16000 punti) perchè i due bit + significativi (15 e 16) sono usati per gestire il giro
*/
long SIFRA_Manager::encoder_updateCnt(int m_line)
{
	register word 	encCnt;
	register word  oldCnt;
	register long   deltaCnt;

	oldCnt = (word)MotorStatus[m_line].previus_step_motor;

	switch(m_line)		// copio il valore attuale del contatore encoder associato alla via
	{
		case 0:
			encCnt = cpld_counter1 & _MASK_16BIT_;
			break;
		case 1:
			encCnt = cpld_counter2 & _MASK_16BIT_;
			break;
		case 2:
			encCnt = cpld_counter3 & _MASK_16BIT_;
			break;
		case 3:
			encCnt = cpld_counter4 & _MASK_16BIT_;
			break;
		case 4:
			encCnt = cpld_counter5 & _MASK_16BIT_;
			break;
		default:
			encCnt = 0;
			break;
	}

	/*
	if(m_line == 0)
		encCnt = cpld_counter2 & _MASK_16BIT_;		// copio il valore attuale del contatore
	else if(m_line == 1)
			encCnt = cpld_counter3 & _MASK_16BIT_;		// copio il valore attuale del contatore
		else if(m_line == 2)
				encCnt = cpld_counter4 & _MASK_16BIT_;		// copio il valore attuale del contatore
			else if(m_line == 3)
					encCnt = cpld_counter5 & _MASK_16BIT_;		// copio il valore attuale del contatore
				else //if(m_line == 4)
						encCnt = cpld_counter1 & _MASK_16BIT_;		// copio il valore attuale del contatore
	*/

	// magari ripeti + volte la lettura e confronta le due per escludere letture spurie
	if( encCnt != oldCnt )			// se si muove ..
  	{
  		if( (encCnt & 0x8000) == (oldCnt & 0x8000) )  // bit msb non cambiato
  		{
  			deltaCnt = encCnt - oldCnt;
      			MotorStatus[m_line].step_motor_done += deltaCnt;
  		}
  		else
  		{
  			if( (encCnt & 0x8000) != 0 )		// possiamo avere caso 0x7f00 -> 0x8000 o 0x0000 -> 0xFF00
  			{
  				if( (encCnt & 0x4000) == 0 )	// sicuramente 0x7f00-> 0x8000
  				{							// 0x80
          				deltaCnt = encCnt - oldCnt;					// count up
              			MotorStatus[m_line].step_motor_done += deltaCnt;
  				}
  				else	// è andato all'indietro?	// sicuramente 0x00-> 0xFF
  				{							// 0xC0
          				deltaCnt = 0x10000;		// count down
          				deltaCnt -= encCnt;
          				deltaCnt += oldCnt;
              			MotorStatus[m_line].step_motor_done -= deltaCnt;
  				}
  			}
  			else							// possiamo avere caso 0x8000 -> 0x7f00 o 0xFF00 -> 0x0000
  			{
  				if( (encCnt & 0x4000) == 0 )	// sicuramente 0xFF00-> 0x0000
  				{							// 0x00
	          			deltaCnt = 0x10000;
	          			deltaCnt -= oldCnt;		// count up
	          			deltaCnt += encCnt;
	              		MotorStatus[m_line].step_motor_done += deltaCnt;
  				}
  				else						// sicuramente 0x8000-> 0x7f00
  				{							// 0x40
	          			deltaCnt = oldCnt;		// count down
	          			deltaCnt -= encCnt;
	              		MotorStatus[m_line].step_motor_done -= deltaCnt;
  				}
  			}
  		}

		MotorStatus[m_line].previus_step_motor = encCnt;				// aggiorna l'ultima lettura... in caso di variazioni
	}
	else
	{
		deltaCnt = 0;
	}

	return deltaCnt;
}

/*
* Questa funzione è chiamata nella AirInManager periodicamente allo scadere del timer m_TimerAllarmeAria, cioè ogni volta che leggo lo stato dell'aria.
* Calcolo la derivata dello stato encoder e da quello il flusso pompa stimato.
* Ritorna il flusso in decimi di millilitro.
*/
float SIFRA_Manager::calc_flusso_pompa(int m_line)
{
	float flow = 0;
	register word 		encval;
	register word  	oldval;
	long   deltaval = 0;

	// costruisco l'indirizzo all'interno della cPLD per andare a leggere il contatore a seconda della via in oggetto. L'indirizzo 0x0400100 corrisponde a cpld_counter1
	encval = (*((word *)(0x0400100 + 2*m_line))) & _MASK_16BIT_;

	oldval = m_prev_val[m_line];	// richiamo il valore encoder del giro precedente

	if( encval != oldval )			// se si muove ..
  	{
  		if( (encval & 0x8000) == (oldval & 0x8000) )  // bit msb non cambiato
  		{
  			deltaval = encval - oldval;
  		}
  		else
  		{
  			if( (encval & 0x8000) != 0 )		// possiamo avere caso 0x7f00 -> 0x8000 o 0x0000 -> 0xFF00
  			{
  				if( (encval & 0x4000) == 0 )	// sicuramente 0x7f00-> 0x8000
  				{							// 0x80
          				deltaval = encval - oldval;					// count up
  				}
  				else	// è andato all'indietro?	// sicuramente 0x00-> 0xFF
  				{							// 0xC0
          				deltaval = 0x10000;		// count down
          				deltaval -= encval;
          				deltaval += oldval;
  				}
  			}
  			else							// possiamo avere caso 0x8000 -> 0x7f00 o 0xFF00 -> 0x0000
  			{
  				if( (encval & 0x4000) == 0 )	// sicuramente 0xFF00-> 0x0000
  				{							// 0x00
	          			deltaval = 0x10000;
	          			deltaval -= oldval;		// count up
	          			deltaval += encval;
  				}
  				else						// sicuramente 0x8000-> 0x7f00
  				{							// 0x40
	          			deltaval = oldval;		// count down
	          			deltaval -= encval;
  				}
  			}
  		}
	}
	
	flow = ((float)deltaval) / g_encStruct[m_line].param_encoder[g_restart.tipo_linea];	// (impulsi) / (impulsi / decimo di ml) = decimi di ml
	m_prev_val[m_line] = encval;		// aggiorno il valore che mi servirà al prossimo giro

	return flow;	// ritorna il flusso attuale calcolato in decimi di millilitri
	
}


/**
* Reset of the encoder data struct of all the motors
*/
void SIFRA_Manager::ResetMotorStepCounter()
{
	int i;

	MotorStatus[0].previus_step_motor = cpld_counter1 & _MASK_16BIT_;
	MotorStatus[1].previus_step_motor = cpld_counter2 & _MASK_16BIT_;
	MotorStatus[2].previus_step_motor = cpld_counter3 & _MASK_16BIT_;
	MotorStatus[3].previus_step_motor = cpld_counter4 & _MASK_16BIT_;
	MotorStatus[4].previus_step_motor = cpld_counter5 & _MASK_16BIT_;

	for(i = 0; i < _NUM_PUMPS; i++)
	{
		MotorStatus[i].step_motor_done = 0;
		MotorStatus[i].step_motor_expected = 0;
		m_old_step[i] = 0;
		m_this_step[i] = 0;
		m_waiting_step[i] = 0;
	}
}

void SIFRA_Manager::setStatePump(int i, int command)
{
	stato_pompa[ i ] = command;
}

int SIFRA_Manager::getStatePump(int i)
{
	return stato_pompa[ i ];
}

/**
* Tool di verifica di eventuali blocchi delle comunicazioni verso il driver pannello luci e verso gli adc, entrambi mappati su seriale sincrona e
* particolarmente colpiti dagli effetti delle ESD.
* L'algoritmo se riconosce un blocco del contatore che gira nella FSM degli adc o una o più celle a fondoscala, esegue una routine, che prevede prima il
* tenativo di ripristinare la comunicazione e se fallisce, allora impone il reset scheda, il tutto in modo che la tastiera si accorge solo di un
* allarme instabile. alla fine la scheda si dovrebbe ritrovare in stato errore isntabile, pompe ferme e pesi validi relativi all'ultimo backup
*/
void SIFRA_Manager::VerificaESD_Damage()
{
	int line;
	switch(m_stato_check_blocco)
	{
		case verifica_contatore:	// verifico il contatore innestato nella macchina a stati della comunicazione con gli ADC che incrementa in caso di assenza di dati utili dagli adc stessi
			for(line = 0; line < NUM_MAX_LINE; line++)
			{
				Via[line].stato_led = OFF;
				Via[line].stato_luci = SPEGNI_LUCE;
			}
			if(CS5530_checkBlockComunication(_COUNT_ADC) || CS5530_checkBlockCells(m_WeightSample)) 	// se il contatore supera la soglia o se gli adc forniscono il valore di fondo scala, torna True
			{
				m_stato_check_blocco = contatore_overflow;
			}
			break;
		case contatore_overflow:		// il contatore è andato in overflow oppure i pesi letti sono a fondo scala.
			for(line = 0; line < NUM_MAX_LINE; line++)
			{
			/*
				if(Via[line].abilitazione == True)	// da considerare solo sulle vie abilitate
				{
				*/
				Via[line].stato_led = ON;
				Via[line].stato_luci = ERR_ARIA;
				StatusCmd.flagsErrorLine |= (0x01 << line);	// setto un errore nella via m_chan solo se abilitata
				StatusCmd.statusChan[line] |= ERR_PESOINSTABILE;	// set errore stabilità
				//}
			}
			CS5530_setWatchdogAdcTimer(_5_SEC_);	// imposto il timer dopo il quale vado a resettare gli adc
			CS5530_setCounterWaitingData();		// resetto il contatore
			load_stopSampling();					//chiusura della seriale verso gli adc
			m_stato_check_blocco = verifica_watchdog_timer;
			break;
		case verifica_watchdog_timer:	// qui attende x secondi dal rilevamento blocco, prima di reinizializzare la comunicazione
			if(CS5530_getWatchdogAdcTimer())	// è scattato il watchdog inizializzato dopo l'errore
			{
				CS5530_resetAdcComunication();		// resetto la comunicazione
				m_resetLedDrivers.Preset(10);		// allla prima esecuzione della led manager deve fare il reset
				for(line = 0; line < NUM_MAX_LINE; line++)
				{
					Via[line].stato_luci = SPEGNI_LUCE;
					StatusCmd.statusChan[line] &= ~ERR_PESOINSTABILE;	// reset errore stabilità
				}
				CS5530_setWatchdogAdcTimer(_2_SEC_);		// un minimo ritardo per attendere il ripristino della comunicazione con gli adcs e procedere a verificare che sia tutto ok
				m_stato_check_blocco =  verifica_contatore_dopo_reset_1;
			}
			break;
		case verifica_contatore_dopo_reset_1:
			if(CS5530_getWatchdogAdcTimer())	// è scattato anche il secondo watchdog, è ora di ricontrollare la comunicazione con gli adc
			{
				if(CS5530_checkBlockComunication(0))	// se si incrementa, quindi se funziona
				{
					CS5530_setWatchdogAdcTimer(_10_SEC_);
					m_stato_check_blocco =  verifica_contatore_dopo_reset_2;
					for(line = 0; line < NUM_MAX_LINE; line++)
					{
						Via[line].stato_luci = ERR_ARIA;
					}
				}
				else
				{
					for(line = 0; line < NUM_MAX_LINE; line++)
					{
						Via[line].stato_luci = SETTA_LUCE;
					}
					m_timer_board_reset.Preset(_5_SEC_);
					m_stato_check_blocco = reset_scheda;	//	cambio stato reset_scheda
				}
			}
			break;
		case verifica_contatore_dopo_reset_2:
			if(CS5530_checkBlockCells(m_WeightSample)) // se almeno uno degli adc dà il valore di fondo scala, torna True
			{
				for(line = 0; line < NUM_MAX_LINE; line++)
				{
					Via[line].stato_luci = SETTA_LUCE;
				}
				m_timer_board_reset.Preset(_5_SEC_);
				m_stato_check_blocco = reset_scheda;	//	cambio stato reset_scheda
			}
			else			// il contatore si incrementa ma non sappiamo ancora se va bene tutto
			{
				if(CS5530_getWatchdogAdcTimer())	// è scattato il terzo watchdog, la comunicazione con gli adc è stata ripristinata
				{
					m_stato_check_blocco = verifica_contatore;	// sembra che tutto va bene, ritorno allo stato iniziale
				}
				else
				{
					if(CS5530_checkBlockComunication(_COUNT_ADC)) // il contatore incrementa ma il campionamento non è partito, non ci sono dati validi
					{
						m_stato_check_blocco = contatore_overflow;
					}
				}
			}
			break;
		case reset_scheda:
			if(m_timer_board_reset.Match())
			{
				m_timer_board_reset.Stop();
				reset_4_block = 0xAA;	// setto il flag in eeprom, che al riavvio mi dice di fare un ripristino
				SIFRAMsg_resetApplicationHandler();	// impongo reset alla scheda
				m_stato_check_blocco = verifica_contatore;
			}
			break;
	}
}

/**
* Set in red, in green or off the 'line' diode of the panel
*/
void SIFRA_Manager::setta_led_pannello(int line, bool light)
{
	Via[line].stato_led = ON;
	if(light == ON)
	{
		Via[line].stato_luci = SETTA_LUCE;
	}
	else
	{
		Via[line].stato_luci = SPEGNI_LUCE;
	}
}

/**
* Set off all the  diodes of the panel
*/
void SIFRA_Manager::reset_state_leds(bool state)
{
int i;
	for(i = 0; i < NUM_MAX_LINE; i++)
	{
		reset_state_this_led(i, state);	// una alla volta le setta tutte, in base al valore di "state"
		Via[i].timer_led.Preset(_T_BLINK_OTHER_ERR);
	}
}

void SIFRA_Manager::reset_state_this_led(int line, bool state)
{
	Via[line].stato_led = state;
	Via[line].fase_luci = 0;
	Via[line].stato_luci = IDLE_LUCE;
}

/**
* Usata solo nella air_manager, serve per settare in maniera condizionata dallo stato attuale, le condizone e i flag di errore
* Se non c'è un allarme instabile sulla via e se è attivo il flag air_detect_en, allora accende la luce rossa lampegginate lenta
* Se la via è abilitata ed è attivo il flag air_block_en, allora setta l'errore macchina per il blocco motori
* In ogni caso setta lo status chan così la tastiera vede la via in allarme e accende il flag rosso nella schermata service, anche nelle fasi di idle
*/
void SIFRA_Manager::imposta_allarme_aria(int i)
{
	//StatusCmd.statusChan[ i ] |= ERR_RILEV_ARIA;	// setto il bit relativo all'allarme aria
	if(Via[i].abilitazione == True)
	{
		if(StatusCmd.air_block_en[i] == True)
		{
			StatusCmd.flagsErrorLine |= 0x03;		// se errore aria sulla via abilitata, devo bloccare e lampeggiare il led rosso
			StatusCmd.air_block_en[i] = False;		// resetto il flag per evitare un loop errore/stop/errore/stop
		}
	}
	if((StatusCmd.statusChan[ i ] & ERR_PESOINSTABILE) == 0)	// se non c'è un allarme instabile attivo, che ha priorità
	{
		if(StatusCmd.led_aria_en == True)			// se può lampeggiare, allora setta la macchina a stati della led_manager
		{
			Via[i].stato_luci = ERR_ARIA;
		}
	}
}

/**
* Inizializzaizone della classe e di tutte le strutture dell'applicazione
*/
void SIFRA_Manager::DefaultSystemReset()
{
	byte m_line;

	StatusCmd.status	= STATO_IDLE;

	for(m_line=0;m_line<_MAX_LOAD_CHAN_;m_line++)
	{
		m_control_cal_values[m_line].typeOfOffsetCal = 0;
		m_control_cal_values[m_line].typeOfGainCal = 0;
		m_control_cal_values[m_line].WeightFactoryOffset = 0;
		m_control_cal_values[m_line].WeightFactoryGain = 0;
		m_control_cal_values[m_line].Weightoffset = 0;
		m_control_cal_values[m_line].Weightgain = 0;
		m_control_cal_values[m_line].AdcOf2Kg = 0;
	}

	/*
	* Reset di tutte le variabili
	*/
	SifraResetSystemParams();

	/*
	* Impostazione dei timer di controllo sempre attivi
	*/
	m_TimeCalcFlow.Preset(_TIMER_CALC_FLW);
	m_timerLed.Preset(_1_SEC_);
	m_timer_test_fw_run.Preset(_5_SEC_);	
}

/**
* Flash board's led in case of hardware and firmware conflict
*/
void SIFRA_Manager::FlashLed_hw_error(void)
{
static bool blink = False;

	if(m_timerLed.Match())
	{
		if(	blink == True )
		{
			PinLED0_LIFE_M  = 1;				// toggle solo P63 cosi' p64 rimane	libero per la comunicazione
			cpld_pin_led = 1;
		}							// e/o la segnalazione del vpp
		else
		{
			PinLED0_LIFE_M = 0;				// toggle solo P63 cosi' p64 rimane	libero per la comunicazione
			cpld_pin_led = 0;
		}

		switch(g_stateMachine.ledState)
		{
			case DISPLAY_INIT:
				LedDrivers_sendData(INTENSITY);	// spegne il test
				LedDrivers_sendData(TEST_LED);		// digits 0, 1, 2, 3, 4 displayed
				LedDrivers_sendData(SCANLIMIT);		// no decide for digit
				LedDrivers_sendData(DECODE_MODE);
				m_timerLed.Preset(_100_mSEC_);
				g_stateMachine.ledState = DISPLAY_READY;
				break;
			case DISPLAY_READY:
				m_timerLed.Preset(_100_mSEC_);
				switchoff_leds();	// spegni tutte le luci
				g_stateMachine.ledState = BLINK_LEDS_HW_ERROR;
				break;
			case BLINK_LEDS_HW_ERROR:	// lampeggio continuo dei leds per segnalare errore bloccante
				m_timerLed.Preset(_800_mSEC_);
				if(blink == True)
					switchon_leds();		// accende tutti i led
				else
					switchoff_leds();		// spegne tutti i led
				break;
		}
		blink = ~blink;
	}
}

/**
* In base alla velocità pompa, alla frequenza di campionamento, calcola il delta peso stimato al prossimo campione ritornando un valore in decimi di grammo
*/
int SIFRA_Manager::calc_next_theor_weight(int i)
{
	int t_velsu100, t_freq, t_stim;
	float t_vel;

	t_velsu100 = (int)(getSpeedPump(i));			// percentuale della portata 0-1000ml/min
	if(t_velsu100 == 0)
		return 0;								// un decimo di ml
	if(t_velsu100 > _PWM_CYCLE_MAX)
		t_velsu100 = _PWM_CYCLE_MAX;				// a 65 per ora si ottine la velocità massima, oltre non va
	t_vel = calcola_variazione_peso(t_velsu100);		// converte la velocità in percentuale, in decimi di ml al secondo
	t_freq = weightChan->get_filter_value(i);			// numero campioni filtro secondario			
	t_stim = (int)(t_vel * (float)t_freq / _SAMPLE_FREQ);	// delta peso stimato fra questo e i prossimo campione
	if(t_stim < 1)
		t_stim = 1;								// delta minimo di un decimo di ml

	return t_stim;
}

/**
* Converte il valore di velocità in percentuale, in decimi di ml, a partire dai valori di calibrazione flusso pompa raccolti nei test di 
* inizio 2014: ml/min = 9.3*vel%-73,6
*/
int SIFRA_Manager::calcola_variazione_peso(int t)
{
	int MltoMin = 9.3 * t - 7.4;	// prende la vel in percentuale, la converte in ml/min
	if(MltoMin > 0)
	{
		int deciMLtoMin = MltoMin*10;
		int deciMltoSec = deciMLtoMin/60;
		return deciMltoSec;
	}
	else 
		return 0;
}

void SIFRA_Manager::UpdateDebugStruct()
{
	byte line;

	for(line=0;line<NUM_MAX_LINE;line++)
	{
		DebugCmd.peso_init[line] = m_init_weight[line];
		DebugCmd.peso_real_time[line] = m_weight_real_time[line];
		DebugCmd.enc_value[line] = m_enc_value[line];
		DebugCmd.enc_value_control[line] = m_enc_value_control[line];
		DebugCmd.enc_movement[line] = m_enc_movement[line];
		DebugCmd.enc_upper_limit[line] = m_enc_upper_limit[line];
		DebugCmd.enc_lower_limit[line] = m_enc_lower_limit[line];
	}
	DebugCmd.air_state[0] = cpld_pin_Air1;
	DebugCmd.air_state[1] = cpld_pin_Air2;
	DebugCmd.air_state[2] = cpld_pin_Air3;
	DebugCmd.air_state[3] = cpld_pin_Air4;
	DebugCmd.air_state[4] = cpld_pin_Air5;
}

/*
* Metodo usato in test EMC per calcolare la variazione dei valori encoder con motori in movimento
*/
void SIFRA_Manager::Encoder_counter()
{
	byte line;
	word enc_value_temp;

	for(line=0;line<NUM_MAX_LINE;line++)
	{
		if(m_enc_value[line] >= m_enc_value_control[line])
		{
			m_enc_movement[line] = m_enc_value[line] - m_enc_value_control[line];
		}
		else
		{
			enc_value_temp = MAX_VALUE_16_BIT - m_enc_value_control[line];
			m_enc_movement[line] = m_enc_value[line] + enc_value_temp;
		}
		m_enc_value_control[line] = m_enc_value[line];
	}
}

/*
* Metodo usato in test EMC per verificare la non variabilità dei valori encoder dei motori
*/
void SIFRA_Manager::EncoderPumpsControl(byte __line)
{
	word range;
	word upper_limit;
	word lower_limit;

	range = (word)(_ENC_STEP_EXPECTED * _PERC_ENC_CONTROL);
	lower_limit = (word)(_ENC_STEP_EXPECTED - range);
	upper_limit = (word)(_ENC_STEP_EXPECTED + range);

	m_enc_lower_limit[__line] = lower_limit;
	m_enc_upper_limit[__line] = upper_limit;

	if(m_first_enc_control[__line])
	{
		m_first_enc_control[__line] = False;
	}
	else
	{
		if((m_enc_movement[__line] < m_enc_lower_limit[__line]) || (m_enc_movement[__line] > m_enc_upper_limit[__line]))
		{
			StatusCmd.flagsErrorLine |= (0x01 << __line);	// setto un errore nella via m_chan solo se abilitata
			StatusCmd.statusChan[__line] |= ERR_VAR_ENCODER;	// set errore stabilità
			m_verify_enc_timer.Stop();
		}
	}
}

/*
* Metodo usato in test EMC che imposta la forbice di variazione di valori ammessi
*/
void SIFRA_Manager::Set_weight_threshold()
{
	byte line;

	for(line=0;line<NUM_MAX_LINE;line++)
	{
		m_weight_lower_limit[line] = m_init_load[line] - THRESHOLD_WEIGHT;
		m_weight_upper_limit[line] = m_init_load[line] + THRESHOLD_WEIGHT;
	}
}

/*
* Metodo usato in test EMC per verificare la non variabilità dei pesi
*/
void SIFRA_Manager::LoadControl(byte __line)
{
	if((m_peso_attuale[__line] < m_weight_lower_limit[__line]) || (m_peso_attuale[__line] > m_weight_upper_limit[__line]))
	{
		StatusCmd.flagsErrorLine |= (0x01 << __line);	// setto un errore nella via m_chan solo se abilitata
		StatusCmd.statusChan[__line] |= ERR_VAR_PESI;	// set errore stabilità
		m_verify_enc_timer.Stop();
	}
}

/*
* Reset di tutti gli errori escluso quello di cella danneggiata.
*/
void SIFRA_Manager::SIFRA_resetStatusErrorStop()
{
	byte num_via;
	
	for( num_via = 0; num_via < (_MAX_LOAD_CHAN_ ); num_via++)
	{
		StatusCmd.statusChan[num_via] &= ERR_RST_STOP;
	}
	StatusCmd.errors &= E_ERRORS_RST_STOP;
	StatusCmd.log_error = E_LOG_ERROR_NONE;
}

/*
* Reset di tutti gli errori escluso quello di cella danneggiata.
*/
void SIFRA_Manager::SIFRA_resetStatusError()
{
	byte num_via;
	
	for( num_via = 0; num_via < (_MAX_LOAD_CHAN_ ); num_via++)
	{
		StatusCmd.statusChan[num_via] &= ERR_END_FORMULA;
		StatusCmd.error_stop[num_via] = E_ERR_STOP_NONE;
	}
	StatusCmd.errors &= E_ERRORS_RST_STOP;
	StatusCmd.log_error = E_LOG_ERROR_NONE;
}

/**
* A partire dal peso specifico inviato da tastiera, estrae il valore float adattato.
* La tastiera invia un numero intero in migliaia, è da convertire in un numero con 3 cifre decimali (p.e. 1,254).
* Se il valore calcolato non è coerente, torna 1.0 (peso specifico acqua).
*/
float SIFRA_Manager::AdattaPesoSpecifico(word pes_spec)
{
	float spec = 0;
	
	spec = (float)pes_spec / _MILLE_;
	
	if((spec > _PESO_SPEC_MIN) && (spec < _PESO_SPEC_MAX))
	{
		return spec;
	}
	else
	{
		return 1.0;
	}
}

/**
* Metodo utilizzato per resettare i parametri di calibrazione memorizzati in EEPROM
*/
bool SIFRA_Manager::SIFRACalibResetData(byte __line)
{
	byte i;
	bool result = True;
	byte type_of_cal;

	type_of_cal = CHAN_CALIB_RESET_VAL;

	switch(__line)
	{
		case _VIA_1_:
		case _VIA_2_:
		case _VIA_3_:
		case _VIA_4_:
		case _VIA_5_:
			if(!reset_backup_factory_values(__line, type_of_cal))
			{
				result = False;
			}
			else
			{
				Chan[__line].AreCalibrate = False;
				backup_calib_state(__line, (byte)Chan[__line].AreCalibrate);
			}
			break;
		case NUM_MAX_LINE:
			for(i=0;i<_NUM_OF_WEIGHT_CHAN_;i++)
			{
				if(!reset_backup_factory_values(i, type_of_cal))
				{
					result = False;
				}
				else
				{
					Chan[i].AreCalibrate = False;
					backup_calib_state(i, (byte)Chan[i].AreCalibrate);
				}
			}
			break;
		default:
			result = False;
			break;
	}

	return result;
	
}

/**
* Verifica apertura sportello della pompa.
*/
bool SIFRA_Manager::Verifica_apertura_sportello()
{
	bool result = False;

	StatusCmd.cover_open = getCoverPumpState();

	if(m_timerControlCover.Match())
	{
		m_timerControlCover.Preset(_1_SEC_);
		if(getCoverPumpState())	// se è aperto lo sportello, blocca tutto e genera errore
		{
			StatusCmd.flagsErrorLine = 0x01;
			StatusCmd.errors |= ERR_COVER_APERTA;
			m_timerControlCover.Stop();
			result = True;
		}
	}

	return result;

}

/**
* Imposta la fase di verifica bilance o calibrazione in corso
*/
void SIFRA_Manager::SIFRASetScalesPhase(byte __startEnd, byte __phase)
{
	if(__startEnd == E_SCALE_PHASE_START)
	{
		m_verifica_bilance = True;
		switch(__phase)
		{
			case E_TYPE_SCALE_VERIFICATION:
				m_scales_phase = E_TYPE_SCALE_VERIFICATION;
				break;
			case E_TYPE_SCALE_CALIB_MEM:
				m_scales_phase = E_TYPE_SCALE_CALIB_MEM;
				break;
			case E_TYPE_SCALE_CALIB_USR:
				m_scales_phase = E_TYPE_SCALE_CALIB_USR;
				break;
			default:
				m_verifica_bilance = False;
				m_scales_phase = E_TYPE_SCALE_NONE;
				break;
		}
	}
	else
	{
		m_verifica_bilance = False;
		m_scales_phase = E_TYPE_SCALE_NONE;
		changeSIFRAstatus(AttesaComando);
	}
}

/**
* Imposta le vie non calibrate identificate dall'applicazione.
*/
void SIFRA_Manager::SIFRASetScalesNotCalibrated()
{
	byte i;

	for(i=0;i<NUM_MAX_LINE;i++)
	{
		if(g_scalesNotCalibCmd.scales_not_calib & (0x01 << i))
		{
			Chan[i].AreCalibrate = False;
			backup_calib_state(i, (byte)Chan[i].AreCalibrate);
		}
	}

	g_scalesNotCalibCmd.scales_not_calib = 0;
}

/**
* Metodo per verificare il funzionamento della scheda luci. Accensione progressiva di tutti i led di ciascuna via.
*/
bool SIFRA_Manager::SIFRATestLed_Manager()
{
	if(m_test_led_timer.Match())
	{
		switch(m_test_led)
		{
			case E_LED_START:		// spengo tutte le luci
				switchoff_leds();
				m_test_led_timer.Preset(_1_SEC_);
				m_test_led = E_LED_RED;
				break;
			case E_LED_RED:		// accendo tutti i led rossi
				m_test_led_timer.Preset(_1_SEC_);
				LedDrivers_sendData(LED_VIA1 | LED_ROSSO);
				LedDrivers_sendData(LED_VIA2 | LED_ROSSO);
				LedDrivers_sendData(LED_VIA3 | LED_ROSSO);
				LedDrivers_sendData(LED_VIA4 | LED_ROSSO);
				LedDrivers_sendData(LED_VIA5 | LED_ROSSO);
				LedDrivers_sendData(LED_VIA6 | LED_ROSSO);
				LedDrivers_sendData(LED_VIA7 | LED_ROSSO);
				LedDrivers_sendData(LED_VIA8 | LED_ROSSO);
				m_test_led = E_LED_1_GREEN;
				break;
			case E_LED_1_GREEN:		// accendo il led verde di via 1
				m_test_led_timer.Preset(_1_SEC_);
				switchoff_leds();
				LedDrivers_sendData(LED_VIA1 | LED_VERDE);
				m_test_led = E_LED_1_RED;
				break;
			case E_LED_1_RED:		// accendo il led rosso di via 1
				m_test_led_timer.Preset(_1_SEC_);
				LedDrivers_sendData(LED_VIA1 | TURN_OFF);
				LedDrivers_sendData(LED_VIA1 | LED_ROSSO);
				m_test_led = E_LED_2_GREEN;
				break;
			case E_LED_2_GREEN:		// accendo il led verde di via 2
				m_test_led_timer.Preset(_1_SEC_);
				LedDrivers_sendData(LED_VIA1 | TURN_OFF);
				LedDrivers_sendData(LED_VIA2 | LED_VERDE);
				m_test_led = E_LED_2_RED;
				break;
			case E_LED_2_RED:		// accendo il led rosso di via 2
				m_test_led_timer.Preset(_1_SEC_);
				LedDrivers_sendData(LED_VIA2 | TURN_OFF);
				LedDrivers_sendData(LED_VIA2 | LED_ROSSO);
				m_test_led = E_LED_3_GREEN;
				break;
			case E_LED_3_GREEN:		// accendo il led verde di via 3
				m_test_led_timer.Preset(_1_SEC_);
				LedDrivers_sendData(LED_VIA2 | TURN_OFF);
				LedDrivers_sendData(LED_VIA3 | LED_VERDE);
				m_test_led = E_LED_3_RED;
				break;
			case E_LED_3_RED:		// accendo il led rosso di via 3
				m_test_led_timer.Preset(_1_SEC_);
				LedDrivers_sendData(LED_VIA3 | TURN_OFF);
				LedDrivers_sendData(LED_VIA3 | LED_ROSSO);
				m_test_led = E_LED_4_GREEN;
				break;
			case E_LED_4_GREEN:		// accendo il led verde di via 4
				m_test_led_timer.Preset(_1_SEC_);
				LedDrivers_sendData(LED_VIA3 | TURN_OFF);
				LedDrivers_sendData(LED_VIA4 | LED_VERDE);
				m_test_led = E_LED_4_RED;
				break;
			case E_LED_4_RED:		// accendo il led rosso di via 4
				m_test_led_timer.Preset(_1_SEC_);
				LedDrivers_sendData(LED_VIA4 | TURN_OFF);
				LedDrivers_sendData(LED_VIA4 | LED_ROSSO);
				m_test_led = E_LED_5_GREEN;
				break;
			case E_LED_5_GREEN:		// accendo il led verde di via 5
				m_test_led_timer.Preset(_1_SEC_);
				LedDrivers_sendData(LED_VIA4 | TURN_OFF);
				LedDrivers_sendData(LED_VIA5 | LED_VERDE);
				m_test_led = E_LED_5_RED;
				break;
			case E_LED_5_RED:		// accendo il led rosso di via 5
				m_test_led_timer.Preset(_1_SEC_);
				LedDrivers_sendData(LED_VIA5 | TURN_OFF);
				LedDrivers_sendData(LED_VIA5 | LED_ROSSO);
				m_test_led = E_LED_GREEN;
				break;
			case E_LED_GREEN:		// accendo tutti i led verdi
				m_test_led_timer.Preset(_1_SEC_);
				LedDrivers_sendData(LED_VIA5 | TURN_OFF);
				LedDrivers_sendData(LED_VIA1 | LED_VERDE);
				LedDrivers_sendData(LED_VIA2 | LED_VERDE);
				LedDrivers_sendData(LED_VIA3 | LED_VERDE);
				LedDrivers_sendData(LED_VIA4 | LED_VERDE);
				LedDrivers_sendData(LED_VIA5 | LED_VERDE);
				m_test_led = E_LED_STOP;
				break;
			case E_LED_STOP:		// spengo tutti i led
				switchoff_leds();
				return True;
				break;
			default:
				m_test_led_timer.Preset(_1_SEC_);
				m_test_led = E_LED_START;
				break;
		}
	}	

	return False;
}

