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

void (*fpp)(void);

extern LINES_Status Via[];
extern STRUCTSTARTCMD StartCmd;
extern STRUCTSTATUSCMD StatusCmd;
extern structStatusDebug_t StatusDebug;
extern structMachineState_t g_stateMachine;
extern structWeightCmd_t WeightCmd;
extern structRestartFormulaCmd_t g_restartFormulaCmd;
extern structScalesPhaseCmd_t g_scalesPhaseCmd;
extern DecTimer EV_timer;
extern ChannelsBackupParam Chan[];
extern int stato_pompa[];
extern ENCODER_Motor_Status	 MotorStatus;

extern BK_struct bk;
extern FP_STRUCT structEnc;

float m_tolleranza[] = {0.05, 0.05};	// per il momento impostiamo valori uguali
//float m_tolleranza[] = {0.065, 0.15};	// VALORI ORIGINALI

/**
* Class constructor.
* @param SIFRAProtocol class tha manage the communication protocol with PC
*/
SIFRA_Manager::SIFRA_Manager( SIFRAProtocol *SifraProtocol)
{
	byte i;
	byte result_EE_read;
	byte data_calib;

#ifdef _DEBUG_CALIB_VALUES_
	byte numchan;
#endif

	m_SIFRAProtocol = SifraProtocol;
	
	SetHardwareVersion();					// verifica dell'HW in base ai due switch settabili dal collaudatore

	StatusCmd.state 			= InitStatus;
	StatusCmd.prevState		= InitStatus;
	StatusCmd.phase			= E_PHASE_INIT;

	m_this_CPU_is = 0;
	m_weightUnstableLevel = THREESHOLD_FILLING_INCR_LEVEL_2;
	m_verifica_bilance = False;
	m_openEV_pich_fail = False;

	g_stateMachine.driverState = INIT_DRIVER;			// variabile usata per controllo gestione led

	m_startLoadingSamples = False;			// inizializzo la variabile che permette la lettura dei pesi forniti dagli ADC
	m_loadsystem2 = 0;
	WeightCmd.sensibility = E_LEVEL_UNSTABLE_NONE;
	StatusCmd.SIFRAComHwErrors = 0;
	StatusCmd.SIFRAComProtocolErrors = 0;
	StatusCmd.SIFRAComUnknownErrors = 0;
	StatusCmd.test_pinch_done = E_TEST_PINCH_TO_DO;

	DefaultSystemReset();
	m_TimerChkHw.Preset(_TIMEOUT_CHKHW);	// timer per verifica cella rotta, da fare molto diluito nel tempo
	
	m_this_CPU_is = m_SIFRAProtocol->getLocalNODEID();
	if(m_this_CPU_is != M3300)		// firmware e set hardware non combaciano
	{
		while(1)		// errore bloccante, se hw e fw non corrispondono si blocca qui lampeggiando tutti i led
		{	
			FlashLed_hw_error();	// getsione luci pannello e scheda
		}
	}

#ifdef _DEBUG_CALIB_VALUES_
	for(numchan = 0; numchan < _MAX_LOAD_CHAN_; numchan++)
	{
		if(EE_random_byte_read((ADDRESS_CHAN_ARE_MEM_CALIBRATE + numchan*BASESTRUCTADDR), &data_calib))
		{
			if (data_calib == CHAN_CALIB_CHECK_FACTORY_VAL)
			{
				Chan[numchan].WeightFactoryOffset = EE_read_word(ADDRESS_FACTORY_OFFSET+ numchan*BASESTRUCTADDR);
				Chan[numchan].Weightoffset = EE_read_word(ADDRESS_OFFSET+ numchan*BASESTRUCTADDR);
			}
			else
			{
				Chan[numchan].WeightFactoryOffset = EE_read_word(ADDRESS_FACTORY_OFFSET+ numchan*BASESTRUCTADDR);
				Chan[numchan].Weightoffset = EE_read_word(ADDRESS_OFFSET+ numchan*BASESTRUCTADDR);
			}
		}
		else
		{
			asm("nop");
		}

		if(EE_random_byte_read((ADDRESS_CHAN_ARE_MEM_CALIBRATE + 1 +numchan*BASESTRUCTADDR), &data_calib))
		{
			if (data_calib == CHAN_CALIB_CHECK_FACTORY_VAL)
			{
				Chan[numchan].WeightFactoryGain = EE_read_float(ADDRESS_FACTORY_GAIN + numchan*BASESTRUCTADDR);
				Chan[numchan].Weightgain = EE_read_float(ADDRESS_GAIN + numchan*BASESTRUCTADDR);
				Chan[numchan].AdcTo2Kg = EE_read_int(ADDRESS_2Kg_READ + numchan*BASESTRUCTADDR);	// può essere occasionalmente negatio quelo del canale 2
				Chan[numchan].AdcTo2Kg_dx = EE_read_int(ADDRESS_2KgDx_READ + numchan*BASESTRUCTADDR);
			}
			else
			{
				Chan[numchan].WeightFactoryGain = EE_read_float(ADDRESS_FACTORY_GAIN + numchan*BASESTRUCTADDR);
				Chan[numchan].Weightgain = EE_read_float(ADDRESS_GAIN + numchan*BASESTRUCTADDR);
				Chan[numchan].AdcTo2Kg = EE_read_int(ADDRESS_2Kg_READ + numchan*BASESTRUCTADDR);	// può essere occasionalmente negatio quelo del canale 2
				Chan[numchan].AdcTo2Kg_dx = EE_read_int(ADDRESS_2KgDx_READ + numchan*BASESTRUCTADDR);
			}
		}
		else
		{
			asm("nop");
		}
	}
#endif

	result_EE_read = get_adc_param();
	switch(result_EE_read)
	{
		case E_READ_ERROR:
			setErrorEEpromReading();		// segnalare errore di lettura eeprom
			break;
		case E_READ_BK_FACTORY:
			if(EE_random_byte_read(ADDRESS_CALIB, &data_calib))
			{
				if (data_calib > 0)
				{
					Chan[_ADC1_].AreCalibrate = True;
				}
				else
				{
					Chan[_ADC1_].AreCalibrate = False;
				}
			}
			if(EE_random_byte_read(ADDRESS_CALIB + BASESTRUCTADDR, &data_calib))
			{
				if (data_calib > 0)
				{
					Chan[_ADC2_].AreCalibrate = True;
				}
				else
				{
					Chan[_ADC2_].AreCalibrate = False;
				}
			}
			/*
			* TODO: INSERIRE CONTROLLO SUI VALORI LETTI DALLA EEPROM
			*/
			break;
		case E_READ_NO_BK_FACTORY:
			Chan[_ADC1_].AreCalibrate = False;
			Chan[_ADC2_].AreCalibrate = False;
			backup_calib_state(_ADC1_, (byte)Chan[_ADC1_].AreCalibrate);
			backup_calib_state(_ADC2_, (byte)Chan[_ADC2_].AreCalibrate);
			break;
		default:
			StatusCmd.status = STATO_ERRORE;
			StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
			StatusCmd.error_monitor = ERR_CASE_READ_ADC_VALUES;
			break;
	}

	m_timeControl = globalTimer.getTime();
	m_timer_test_nextWeight.Preset(_5_SEC_);

	if(!Read_Param_Encoder())		// carica dalla TAMP RAM i valori di encoder
	{	// se torna false devo caricare i dati di default
		default_struttura_flussopeso();	// setta con valori di defaulti la struttura
		StatusCmd.error_monitor = ERR_RAM_READ_ENC;
		//StatusCmd.flagsErrorLine = 0x01;
		//StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
	}

	SIFRAMsg_startAcquisitionHandler();
	m_loadsystem2 = calc_param_weight(Chan);

	/*
	* Qui entro solamente se ho avuto un blocco nella lettura degli ADC
	*/
	if(reset_4_block == 0xAA)	// ho ordinato il reset scheda causa blocco adc e pannello luci --> auto-reset
	{	// quindi deve eseguire il download subito senza aspettare il cmd abort da tastiera
		// e poi metto il sistema in errore: così la tastiera reagisce con uno stop
		reset_4_block = 0x55;		// resetto flag ripresa da auto-reset
		DownLoad_StatoStart();		// scarico il backup dello stato macchina prima dell'auto-reset
		StatusCmd.status = STATO_ERRORE;	// setto stato errore per avvertire la tastiera che mi deve mandare uno stop ORA
		for(i = 0; i < NUM_MAX_LINE; i++)	// devo settare lo stato di err instabile sulle vie abilitate e bloccate e ora riprese
		{
			if(Via[i].abilitazione == True)
			{
				Via[i].stato_led = ON;
				Via[i].fase_luci = 0;
				Via[i].stato_luci = ERR_INSTABILE;
				StatusCmd.error_monitor = ERR_ESD_DAMAGE;
				StatusCmd.flagsErrorLine = 0x01;
				StatusCmd.statusChan[_ADC1_] |= ERR_PESOINSTABILE;	// set errore stabilità
				//changeSIFRAstatus(StatoErrore);
			}
		}
	}	
}

/**
* Class destructor
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
	if(m_timer_test_fw_run.Match())
	{
		m_timer_test_fw_run.Preset(_TIME_TEST_FW);
		asm("nop");
	}
	
	m_SIFRAProtocol->Manager();	// si occupa della seriale di comunicazione, gestisce errori di comm e decodifica i comandi M3000

	SIFRAcheck_new_command();	// analizza la coda dei messaggi gestiti dalla m_SIFRAProtocol->Manager() e verifica se ci fevono essere dei cambi stat

	m_dato_valido = SIFRASample_Manager();	// verifico la disponibilità di nuovi dati nelle fifo degli adc. se ci sono, converto in decimi di grammo e verifico la stabilità
	
	SIFRAError_Manager();		// verifica le condizioni di errore della macchina CPU_MANAGER

	//---- SIFRAMIX state machine --------
	switch (getSIFRAstatus())
	{
		case AttesaComando:
			break;		
		case InitStatus:			// stato di inizializzazione, non devo fare niente fino a che non ricevo un comando dalla tastiera
			break;
		case StatoSacca:
			if(SIFRASacca_Manager())		// gestisce il riempimento controllando le pompe
			{
				changeSIFRAstatus(TermineSequenza);	// setta il nuovo stato
			}
			break;
		case StatoCalibrazione:
			if(SIFRACalibr_Manager())
			{
				switchoff_leds();
				LedDrivers_clearFifo();
				StatusCmd.status = STATO_STOP;
				changeSIFRAstatus(AttesaComando);	// setta il nuovo stato
			}
			break;
		case StatoErrore:
			asm("nop");
			break;
		case StopAllStatus:		// comando di stop generale
			setStatePump(0, ARRESTO);		// setta lo stato di arresto pompa, il pwm manager si occuperà del resto
			m_check_block_pump.Stop();
			if(StatusCmd.next_line >= 0)
			{
				SIFRACloseLineEV(StatusCmd.next_line);	// chiusura della ev abilitata
			}
			StatusCmd.status = STATO_STOP;
			SIFRA_resetStatusErrorStop();
			if(StatusCmd.m_FillingOnCourse == True)
			{
				if(structEnc.restart == 0xAA)
				{
					BackUpTimer.Stop();
					Backup_StatoStart();
					m_TimerStopAll.Preset(_WAIT_3_SEC);		// setto un tempo massimo di attesa stabilizzazione
					changeSIFRAstatus(StatoStop);				// vado in stato stop
				}
				else
				{
					m_peso_stop = m_peso_attuale;
					changeSIFRAstatus(TermineSequenza);
				}
			}
			else
			{
				m_peso_stop = m_peso_attuale;
				changeSIFRAstatus(TermineSequenza);
			}
			break;
		case StatoStop:	// qui gestisce la fase di chiusura e salvatggio dello stato per poter riprendere alla ripartenza
			if(SIFRAStop_Manager() == True)
			{
				m_peso_stop = m_peso_attuale;
				if(structEnc.restart == 0xAA)
				{
					Backup_StatoStart();
				}
				ResetMotorStepCounter();
				//m_timer_control_restart.Preset(_2_SEC_);
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
		case StatoRiempitubi:
			if(SIFRART_Manager(StatusCmd.next_line))
			{
				changeSIFRAstatus(TermineSequenza);
			}
			break;
		case StatoSvuotatubi:
			if(SIFRAST_Manager(StatusCmd.next_line))
			{
				g_stateMachine.stState = IdleManuale;
				changeSIFRAstatus(AttesaComando);
			}
			break;
		case StatoManuale:
			if(SIFRAManual_Manager(StatusCmd.next_line))		// gestisce il riempimento controllando le pompe
			{
				g_stateMachine.stState = IdleManuale;
				//m_timer_control_restart.Preset(_2_SEC_);
				changeSIFRAstatus(PausaErogazione);	// setta il nuovo stato
			}
			break;
		case StatoRisciacquo:
			if(SIFRARisciacquo_Manager(StatusCmd.next_line))
			{
				changeSIFRAstatus(TermineSequenza);	// setta il nuovo stato
			}
			break;
		case StatoTestPinze:			
			if(SIFRATestPinch())	// Procedura di TEST ELETTROPINZE
			{
				StatusCmd.test_pinch_done = E_TEST_PINCH_DONE;
				StatusCmd.status = STATO_FINE_SEQ;
				StatusCmd.next_line = -1;
				SifraResetViaStatus();
				ResetMotorStepCounter();	// RESETTO stato contatori per evitare errori nel passo successivo
				changeSIFRAstatus(AttesaComando);		// Test Concluso con successo, proseguo con il riempimento
				m_enc_value = cpld_counter1 & _MASK_16BIT_;
			}	
			break;
		case StatoServiceEV:		// a rotazione apri e chiudi ogni elettropinza, con un secondo in apertura e uno in chiusura
			switch(m_testEVstate)	// si esce solo alla ricezione di uno stop
			{
				case E_EV_TEST_OPEN:	// qui si apre la valvola
					if(m_timerServiceEv.Match())
					{
						SIFRAOpenLineEV(m_debug_EV);
						m_timerServiceEv.Preset(_1_SEC_);
						m_testEVstate = E_EV_TEST_CLOSE;
					}
					break;
				case E_EV_TEST_CLOSE:
					if(m_timerServiceEv.Match())
					{ 
						SIFRACloseLineEV(m_debug_EV);
						if(m_debug_EV < (NUM_MAX_LINE - 1))
						{
							m_debug_EV++;
						}
						else	
						{
							m_debug_EV = 0;
						}
						m_timerServiceEv.Preset(_1_SEC_);
						m_testEVstate = E_EV_TEST_OPEN;
					}
					break;
			}
			break;
		case StatoService:	// stato di service utile per i test di validazione o per le prove
			if(SIFRAService_Manager())
			{
				changeSIFRAstatus(AttesaComando);	// setta il nuovo stato
			}
			break;
		case StatoTestEMC:	// stato usato per le prove EMC
			SIFRATestEMC_Manager();	
			if(m_verify_enc_timer.Match())
			{
				m_verify_enc_timer.Preset(_330_mSEC_);
				Encoder_counter();
				EncoderPumpsControl();
			}
			if(g_stateMachine.serviceState >= Controllo_pompe)
			{
				LoadControl();
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
		case StatoTestPompe:
			if(SIFRATest_Pompe(StatusCmd.next_line))
			{
				StatusCmd.status = STATO_STOP;
				m_SIFRAProtocol->m_show_velpompe = False;
				changeSIFRAstatus(AttesaComando);
			}
			break;
		case StatoTestFlussoPompa:
			if(SIFRATest_FlussoPompa(StatusCmd.next_line))
			{
				StatusCmd.status = STATO_STOP;
				changeSIFRAstatus(AttesaComando);
			}
			break;
		case StatoTestEvAperte:
			if(SIFRATest_OpenMoreEVs())
			{
				changeSIFRAstatus(AttesaComando);
			}
			break;
		case TermineSequenza:
			/*
			if(((StartCmd.function & 0x00FF) == RIEMPITUBI) || ((StartCmd.function & 0x00FF) == RISCIACQUO))
			{
				m_timer_control_restart.Preset(_1_SEC_);
				changeSIFRAstatus(PausaFormulaService);
			}
			else
			{
				changeSIFRAstatus(AttesaComando);
			}
			*/
			changeSIFRAstatus(AttesaComando);
			SIFRA_resetStatusErrorStop();
			SifraResetStartCmd();
			SifraResetStatus();
			SifraResetStateMachine();
			SifraResetServiceData();
			SifraResetFillingData();
			SifraStopTimers();
			break;
		case E_WAIT_ENTER_LOADER:
			if(m_timer_enter_loader.Match())
			{
				m_timer_enter_loader.Stop();
				m_SIFRAProtocol->jumpToLoaderHandler();
			}
			break;
		default:
			break;
	}
			
	SIFRALed_Manager();		// getsione luci pannello

	SIFRAPhase_manager();		// aggiornamento fase sistema

	UpdateDebugStruct();		// compila la struttura di debug

	if(m_watchDogTimer.Match())
	{
		StatusCmd.flagsErrorLine = 0x01;					// setto un errore
		StatusCmd.statusChan[_ADC1_] |= ERR_COMUN_INT;	// comunicazione interrotta
	}
}

/**
* Analizza la coda dei messaggi che protocol_sifra ha inviato al sistema, per gestione dei comandi ricevuti
*/
void SIFRA_Manager::SIFRAcheck_new_command()
{
	int msg;

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
				if((getSIFRAstatus() == AttesaComando) || (getSIFRAstatus() == PausaErogazione) ||		//	(getSIFRAstatus() == PausaFormulaService) ||
					(getSIFRAstatus() == InitStatus) || (getSIFRAstatus() == RipresaMancanzaRete))
				{
					Decode_StartMsg();
					SIFRAMsg_setStartHandler();
				}
				else
				{
					if((StatusCmd.status == STATO_START) || (StatusCmd.status == STATO_CALIBRAZIONE_UTENTE) ||
						(StatusCmd.status == STATO_CALIBRAZIONE_FABBRICA))
					{
						SIFRAMsg_setStartHandler();
					}
					else
					{
						StatusCmd.status = STATO_ERRORE;
						StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
						StatusCmd.error_monitor = ERR_CASE_START_FAILED;
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
			case SIFRAMsg_debugViaAsked:
				SIFRAMsg_debugViaAskedHandler();
				break;
			case SIFRAMsg_debugStartAsked:
				SIFRAMsg_debugStartAskedHandler();
				break;
			case SIFRAMsg_debugStatusAsked:
				SIFRAMsg_debugStatusAskedHandler();
				break;
			case SIFRAMsg_debugRamAsked:
				SIFRAMsg_debugRamAskedHandler();
				break;
			case SIFRAMsg_SetWeightSensibility:
				m_SIFRAProtocol->sendAcknowledge();
				if((WeightCmd.sensibility > E_LEVEL_UNSTABLE_NONE) && (WeightCmd.sensibility < E_LEVEL_UNSTABLE_NUM))
				{
					set_unstable_threshold(WeightCmd.sensibility);
				}
				else
				{
					set_unstable_threshold(E_LEVEL_UNSTABLE_MED_1);
				}
				break;
			case SIFRAMsg_restartWeightCoverControl:
				m_SIFRAProtocol->sendAcknowledge();
				m_TimerMaxWeight.Preset(_3_SEC_);
				m_timerControlCover.Preset(_3_SEC_);
				break;
			case SIFRAMsg_restartFormula:
				m_SIFRAProtocol->sendAcknowledge();
				SIFRAWeightControlRestart(g_restartFormulaCmd.type_of_formula);
				break;
			case SIFRAMsg_scalesPhase:
				m_SIFRAProtocol->sendAcknowledge();
				SIFRASetScalesPhase(g_scalesPhaseCmd.init_end, g_scalesPhaseCmd.type_of_phase);
				break;
			case SIFRAMsg_scalesNotCalib:
				m_SIFRAProtocol->sendAcknowledge();
				SIFRASetScalesNotCalib();
				break;
			case SIFRAMsg_setZeroVerification:	// invia risposta e comincia la lettura degli zeri
				changeSIFRAstatus(StatoCalibrazione);
				m_SIFRAProtocol->sendAcknowledge();	// questo va spedito solo a lettura eseguita
				g_stateMachine.calibState = ReadZeroInBothLines;
				switchoff_leds();
				LedDrivers_sendData(LED_VIA1 | LED_ROSSO);
				LedDrivers_sendData(LED_VIA8 | LED_ROSSO);
				break;
			case SIFRAMsg_setZeroCalib:	// invia risposta e comincia la lettura degli zeri
				changeSIFRAstatus(StatoCalibrazione);
				m_SIFRAProtocol->sendAcknowledge();	// questo va spedito solo a lettura eseguita
				g_stateMachine.calibState = ReadZeroInBothLines;
				switchoff_leds();
				LedDrivers_sendData(LED_VIA1 | LED_ROSSO);
				LedDrivers_sendData(LED_VIA8 | LED_ROSSO);
				break;
			case SIFRAMsg_setGain8:
				changeSIFRAstatus(StatoCalibrazione);
				m_SIFRAProtocol->sendAcknowledge();	// questo va spedito solo a lettura eseguita
				g_stateMachine.calibState = ReadLoadCellSX;
				switchoff_leds();
				LedDrivers_sendData(LED_VIA1 | LED_ROSSO);
				break;
			case SIFRAMsg_setGain15:
				changeSIFRAstatus(StatoCalibrazione);
				m_SIFRAProtocol->sendAcknowledge();	// questo va spedito solo a lettura eseguita
				g_stateMachine.calibState = ReadLoadCellDX;
				switchoff_leds();
				LedDrivers_sendData(LED_VIA8 | LED_ROSSO);
				break;
			case SIFRAMsg_resetCalib:					// invia risposta e resetta i parametri di calibrazione in memoria
				m_SIFRAProtocol->sendAcknowledge();
				if(!SIFRACalibResetData())
				{
					StatusCmd.status = STATO_ERRORE;
					StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
					StatusCmd.error_monitor = ERR_RESET_CALIB_DATA_FAIL;
				}
				break;
			case SIFRAMsg_jumpToLoader:
				m_SIFRAProtocol->sendAcknowledge();	// questo va spedito solo a lettura eseguita
				changeSIFRAstatus(E_WAIT_ENTER_LOADER);
				m_watchDogTimer.Stop();
				m_timer_enter_loader.Preset(_5_SEC_);
				break;
			case SIFRAMsg_resetApplication:
				changeSIFRAstatus(AttesaComando);
				m_SIFRAProtocol->resetApplicationHandler();
				break;
			case SIFRAMsg_HwError:
				SIFRAMsg_hwErrorHandler();
				break;
			case SIFRAMsg_opcodeUnknown:
			case SIFRAMsg_stateProtocolUnknown:
			case SIFRAMsg_timoutRx:
			case SIFRAMsg_timoutTx:
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
				StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
				StatusCmd.error_monitor = ERR_MSG_NOT_EXPECTED;
				break;
		}
		msg = m_SIFRAProtocol->getMsg();
	}
}

/**
* Manager dello stato sacca: può uscire per una situaizone di errore oppure perchè ha finito, tornando True.
* Altrimenti torna False.
*/
bool SIFRA_Manager::SIFRASacca_Manager()
{
	word peso_controllo;
	byte air_errors;

	switch( g_stateMachine.saccaState )
	{
		case SaccaIdle:
			break;
		case ConfigurazioneSequenza:	// le vie da eseguire sono state settate, qui si cerca la prima da eseguire e si settano i dettagli
			bk.nextLine = StatusCmd.next_line = ViaDaEseguire();					// setta la via più bassa della sequenza ancora da eseguire
			if(StatusCmd.next_line < 0)// nessuna via da eseguire, ho finito la sequenza
			{
				m_timerEndSeq.Preset(_WAIT_M300_READING);		// esco subito ma devo dare il tempo al padrone di leggere il mio stato start	
				g_stateMachine.saccaState = FineSequenza;
			}
			else
			{
				m_peso_stop = m_peso_attuale;
				if((cpld_reg_in & 0x00FF) & (0x01 << StatusCmd.next_line))
				{
					StatusCmd.flagsErrorLine = 0x01;
					StatusCmd.statusChan[_ADC2_] |= (0x0001 << StatusCmd.next_line);
					Via[StatusCmd.next_line].stato_luci = ERR_ARIA;
					m_air_manager[StatusCmd.next_line] = E_START_AIR;
				}
				else
				{
					g_stateMachine.saccaState = StartSequenza;						// do avvio alla sequenza
					StatusCmd.m_FillingOnCourse = True;	// si attiva il controllo motori in caso di errore: i motori sono spenti
					StatusCmd.air_block_en = True;			// attivazione blocco x aria, l'allarme aria è bloccante e gestisce il led
				}
			}
			break;
		case StartSequenza:									// avvio della sequenza, o inizio rimepimento vie successive
			if (m_dato_valido)
			{
				m_dato_valido = False;
				Set_START_Erogazione(StatusCmd.next_line);			// esegue tutte le impostazioni idonee per lo start pompa
				Via[StatusCmd.next_line].peso_iniziale = (word)m_peso_attuale;
				g_stateMachine.saccaState = StartErogazioneVia;
			}
			break;
		case RipresaSequenza:
			if(m_dato_valido)	// solo quando c'è un dato valido di adc su cui lavorare
			{
				m_dato_valido = False;
				//m_timer_control_restart.Stop();
				StatusCmd.error_stop = E_ERR_STOP_NONE;
				StatusCmd.next_line = bk.nextLine;				// setta la via più bassa della sequenza ancora da eseguire
				Via[StatusCmd.next_line].peso_iniziale = (word)m_peso_attuale;
				if(!m_void_source)
				{
					if(!controllo_capacita_iniziale(StartCmd.cap_linea[StatusCmd.next_line]))		// controlla la capacità passata da tastiera
					{
						break;
					}
				}
				m_void_source = False;
				StatusCmd.m_FillingOnCourse = True;
				if(StatusCmd.next_line >= 0)
				{
					if((cpld_reg_in & 0x00FF) & (0x01 << StatusCmd.next_line))
					{
						if((Via[StatusCmd.next_line].peso_da_erogare - Via[StatusCmd.next_line].peso_gia_erogato) > _VOL_MIN_x_RESTART)
						{
							StatusCmd.flagsErrorLine = 0x01;
							StatusCmd.statusChan[_ADC2_] |= (0x0001 << StatusCmd.next_line);
							Via[StatusCmd.next_line].stato_luci = ERR_ARIA;
							m_air_manager[StatusCmd.next_line] = E_START_AIR;
						}
						else
						{	// troppo poco per ri-partire, non la abilito neppure
							m_timerEndSeq.Preset(_WAIT_M300_READING);		// esco subito ma devo dare il tempo al padrone di leggere il mio stato start
							g_stateMachine.saccaState = ChiusuraErogazione;					// nessuna via, per qualche motivo, quindi fine sequenza
						}
					}
					else
					{
						if(Via[StatusCmd.next_line].peso_da_erogare > Via[StatusCmd.next_line].peso_gia_erogato)
						{
							if((Via[StatusCmd.next_line].peso_da_erogare - Via[StatusCmd.next_line].peso_gia_erogato) <= _VOL_MIN_x_RESTART)		// se c'è meno del minimo erogabile
							{
								m_timerEndSeq.Preset(_WAIT_M300_READING);		// esco subito ma devo dare il tempo al padrone di leggere il mio stato start
								g_stateMachine.saccaState = ChiusuraErogazione;					// nessuna via, per qualche motivo, quindi fine sequenza
							}
							else		// se devo riprendere il riempimento interrotto
							{
								if(Via[StatusCmd.next_line].peso_da_erogare >= Via[StatusCmd.next_line].peso_gia_erogato)
								{
									Via[StatusCmd.next_line].peso_da_erogare = Via[StatusCmd.next_line].peso_da_erogare - Via[StatusCmd.next_line].peso_gia_erogato;
								}
								else
								{
									//StatusCmd.status = STATO_ERRORE;
									StatusCmd.status = STATO_ERRORE;
									StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
									StatusCmd.error_monitor = ERR_CASE_PESO_DA_EROGARE_SACCA_MNGR;
									//changeSIFRAstatus(StatoErrore);
									break;
								}
								if(Via[StatusCmd.next_line].peso_iniziale >= Via[StatusCmd.next_line].peso_da_erogare)
								{
									Via[StatusCmd.next_line].peso_finale = Via[StatusCmd.next_line].peso_iniziale - Via[StatusCmd.next_line].peso_da_erogare;
								}
								else
								{
									StatusCmd.status = STATO_ERRORE;
									StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
									StatusCmd.error_monitor = ERR_CASE_PESO_FINALE_SACCA_MNGR;
									//changeSIFRAstatus(StatoErrore);
									break;
								}
								StatusCmd.air_block_en = True;
								Set_START_Erogazione(StatusCmd.next_line);			// esegue tutte le impostazioni idonee per lo start pompa
								SIFRAOpenLineEV(StatusCmd.next_line);
								m_restart = True;								// serve per escludere il controllo errore volume
								g_stateMachine.saccaState = StatoStartPompa;
							}
						}
						else
						{	// troppo poco per ri-partire, non la abilito neppure
							m_timerEndSeq.Preset(_WAIT_M300_READING);		// esco subito ma devo dare il tempo al padrone di leggere il mio stato start
							g_stateMachine.saccaState = ChiusuraErogazione;					// nessuna via, per qualche motivo, quindi fine sequenza
						}
					}
				}
				else		// se non ci sono vie da riprendere
				{
					m_timerEndSeq.Preset(_WAIT_M300_READING);		// esco subito ma devo dare il tempo al padrone di leggere il mio stato start
					g_stateMachine.saccaState = ChiusuraErogazione;					// nessuna via, per qualche motivo, quindi fine sequenza
				}
			}
			break;
			
		case StartErogazioneVia:
			if(m_dato_valido)	// solo quando c'è un dato valido di adc su cui lavorare
			{
				m_dato_valido = False;
				if(!controllo_capacita_iniziale(StartCmd.cap_linea[StatusCmd.next_line]))		// controlla la capacità passata da tastiera
				{
					break;
				}
				if(Via[StatusCmd.next_line].peso_iniziale > Via[StatusCmd.next_line].peso_da_erogare)
				{
					Via[StatusCmd.next_line].peso_finale = Via[StatusCmd.next_line].peso_iniziale - Via[StatusCmd.next_line].peso_da_erogare;
				}
				else
				{
					StatusCmd.status = STATO_ERRORE;
					StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
					StatusCmd.error_monitor = 	ERR_CASE_PESO_FINALE_SACCA;
					break;
				}
				ResetMotorStepCounter();							// reset dei contatori encoder
				//MotorStatus.step_motor_expected = (long)((float)Via[StatusCmd.next_line].peso_da_erogare * structEnc.param_encoder / Via[StatusCmd.next_line].peso_spec);
				SIFRAOpenLineEV(StatusCmd.next_line);
				g_stateMachine.saccaState = StatoStartPompa;
			}	
			break;
			
		case StatoStartPompa:
			if(m_timer_openEV.Match())		// attendo il tempo per l'apertura della EV
			{
				setStatePump(0, ABILITAZIONE);	// fa partire la pompa dedicata o quella unica			
				m_check_block_pump.Preset(_1_SEC_);
				if(Via[StatusCmd.next_line].peso_da_erogare <= LIMITE_PESO)	// procedo con controllo a encoder
				{
					m_timerVerificaCalo.Preset(T_NON_CALA_ENC);
					m_rampa_timer.Preset(_t_RAMPA_SV);
					setPwmMotCycle(0, _PWM_CYCLE_LOW_2);
					g_stateMachine.saccaState = StatoControlloEncoder;	// l'encoder controlla, il peso verifica
				}
				else
				{
					m_vel = _PWM_CYCLE_MIN;
					m_timerVerificaCalo.Preset(T_NO_CALO_INIT);
					m_rampa_timer.Preset(PRIMA_VELOCITA);
					g_stateMachine.saccaState = StatoErogazione;		// possiamo partire
				}
			}
			break;
			
		case StatoErogazione:
			if(m_dato_valido)	// se c'è una nuova tornata di dati validi
			{
				m_dato_valido = False;
				if(Via[StatusCmd.next_line].peso_iniziale >= m_peso_attuale)
				{
					m_peso_parziale = Via[StatusCmd.next_line].peso_iniziale - (word)m_peso_attuale;
				}
				else
				{
					m_peso_parziale = 0;
				}
				Via[StatusCmd.next_line].peso_erogato = m_peso_parziale + Via[StatusCmd.next_line].peso_gia_erogato;
				Via[StatusCmd.next_line].capacita = aggiorna_capacita(StartCmd.cap_linea[StatusCmd.next_line],  m_peso_parziale, Via[StatusCmd.next_line].peso_spec);
				StatusCmd.flagsErrorLine |= controllo_vuoto(StatusCmd.next_line);
				StatusCmd.flagsErrorLine |= controllo_non_cala(StatusCmd.next_line, _NON_CALA_THRESHOLD);	// verifica delle situazioni di non cala e vuoto, viene settato
			}
			if(SIFRARampe_Manager(StatusCmd.next_line, 0))	// gestisce rampa e riempimento, se torna true allora deve cambiare stato
			{
				m_check_block_pump.Stop();
				m_timerVerificaCalo.Stop();
				setStatePump(0, ARRESTO);
				//SIFRACloseLineEV(StatusCmd.next_line);		// chiusura elettrovalvola
				//m_timerEndSeq.Preset(T_NO_CALO_MIN);			// attesa prima di chiudere la EV, magari dobbiamo ripartire
				g_stateMachine.saccaState = StartRifinituraErogazione;
				break;
			}								
			break;

		case StatoControlloEncoder:
			if(m_dato_valido)	// se c'è una nuova tornata di dati validi (uno per ogni via)
			{
				m_dato_valido = False;
				if(m_peso_attuale <= 0)
				{
					StatusCmd.status = STATO_ERRORE;
					StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
					StatusCmd.error_monitor = 	ERR_CASE_STATO_ENCODER_SACCA;
				}
				if(Via[StatusCmd.next_line].peso_iniziale >= m_peso_attuale)
				{
					m_peso_parziale = Via[StatusCmd.next_line].peso_iniziale - (word)m_peso_attuale;
				}
				else
				{
					m_peso_parziale = 0;
				}
				Via[StatusCmd.next_line].peso_erogato = m_peso_parziale + Via[StatusCmd.next_line].peso_gia_erogato;
				Via[StatusCmd.next_line].capacita = aggiorna_capacita(StartCmd.cap_linea[StatusCmd.next_line],  m_peso_parziale, Via[StatusCmd.next_line].peso_spec);
				StatusCmd.flagsErrorLine |= controllo_vuoto(StatusCmd.next_line);
				StatusCmd.flagsErrorLine |= controllo_non_cala(StatusCmd.next_line, _NON_CALA_THRESHOLD);	// verifica delle situazioni di non cala e vuoto, viene settato
			}
			if(m_peso_attuale >= Via[StatusCmd.next_line].peso_finale)
			{
				peso_controllo = (word)m_peso_attuale - Via[StatusCmd.next_line].peso_finale;
			}
			if(SIFRARampe_Manager(StatusCmd.next_line, 0))	// gestisce rampa e riempimento, se torna true allora deve cambiare stato
			{
				m_check_block_pump.Stop();
				m_timerVerificaCalo.Stop();
				setStatePump(0, ARRESTO);
				//SIFRACloseLineEV(StatusCmd.next_line);		// chiusura elettrovalvola
				//m_timerEndSeq.Preset(T_NO_CALO_MIN);			// attesa prima di chiudere la EV, magari dobbiamo ripartire
				g_stateMachine.saccaState = StartRifinituraErogazione;
				break;
			}	
			break;

		case StartRifinituraErogazione:
			setStatePump(0, ABILITAZIONE);	// fa partire la pompa dedicata o quella unica			
			m_check_block_pump.Preset(_2_SEC_);
			m_timerVerificaCalo.Preset(T_NON_CALA_RIFINITURA);
			setPwmMotCycle(0, _PWM_CYCLE_MIN);
			g_stateMachine.saccaState = RifinituraErogazione;	// l'encoder controlla, il peso verifica
			break;

		case RifinituraErogazione:
			if(m_dato_valido)	// se c'è una nuova tornata di dati validi (uno per ogni via)
			{
				m_dato_valido = False;
				if(Via[StatusCmd.next_line].peso_iniziale >= m_peso_attuale)
				{
					m_peso_parziale = Via[StatusCmd.next_line].peso_iniziale - (word)m_peso_attuale;
				}
				else
				{
					m_peso_parziale = 0;
				}
				Via[StatusCmd.next_line].peso_erogato = m_peso_parziale + Via[StatusCmd.next_line].peso_gia_erogato;
				Via[StatusCmd.next_line].capacita = aggiorna_capacita(StartCmd.cap_linea[StatusCmd.next_line],  m_peso_parziale, Via[StatusCmd.next_line].peso_spec);
				StatusCmd.flagsErrorLine |= controllo_vuoto(StatusCmd.next_line);
				StatusCmd.flagsErrorLine |= controllo_non_cala(StatusCmd.next_line, _NON_CALA_THRESHOLD);	// verifica delle situazioni di non cala e vuoto, viene settato
			}
			if(m_peso_attuale >= Via[StatusCmd.next_line].peso_finale)
			{
				peso_controllo = (word)m_peso_attuale - Via[StatusCmd.next_line].peso_finale;
				if(peso_controllo <= LIMITE_PESO_FINE_EROG)
				{
					m_check_block_pump.Stop();
					m_timerVerificaCalo.Stop();
					setStatePump(0, ARRESTO);
					SIFRACloseLineEV(StatusCmd.next_line);		// chiusura elettrovalvola
					m_timerEndSeq.Preset(T_NO_CALO_MIN);
					g_stateMachine.saccaState = ChiusuraErogazione;
				}
			}
			else
			{
				m_check_block_pump.Stop();
				m_timerVerificaCalo.Stop();
				setStatePump(0, ARRESTO);
				SIFRACloseLineEV(StatusCmd.next_line);		// chiusura elettrovalvola
				m_timerEndSeq.Preset(T_NO_CALO_MIN);
				g_stateMachine.saccaState = ChiusuraErogazione;
			}
			break;

		case ChiusuraErogazione:	// credo di aver raggiunto il target, aspetto la stabilizzazione del peso e procedo con la chiusra
			if(m_dato_valido)								// se c'è una nuova tornata di dati validi (uno per ogni via)
			{
				m_dato_valido = False;
				if(Via[StatusCmd.next_line].peso_iniziale >= m_peso_attuale)
				{
					m_peso_parziale = Via[StatusCmd.next_line].peso_iniziale - (word)m_peso_attuale;
				}
				else
				{
					m_peso_parziale = 0;
					/*
					StatusCmd.flagsErrorLine = 0x01;
					StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
					StatusCmd.error_monitor = 	ERR_CASE_PESO_PARZIALE_SACCA;
					*/
				}
				Via[StatusCmd.next_line].peso_erogato = m_peso_parziale + Via[StatusCmd.next_line].peso_gia_erogato;
				Via[StatusCmd.next_line].capacita = aggiorna_capacita(StartCmd.cap_linea[StatusCmd.next_line],  m_peso_parziale, Via[StatusCmd.next_line].peso_spec);
			}
			if(m_timerEndSeq.Match())
			{
				Via[StatusCmd.next_line].eseguita = True;
				Via[StatusCmd.next_line].abilitazione = False;	// questa via ha finito
				setta_led_pannello(StatusCmd.next_line, OFF);	// spengo la luce relativa alla via conclusa
				// VERIFICA ERRORE DI PRODOTTO //
				if(check_conditions(StatusCmd.next_line))					// verifica se ci sono le condizioni per verificare l'errore relativo di volume
				{
					check_volume_error(StatusCmd.next_line); // verifica di errore flusso peso e/o prodotto non corrispondente
				}
				m_timerEndSeq.Preset(_t_BLINK_STOP);			// secondi di attesa prima di passare alla via successiva in sequenza
				g_stateMachine.saccaState = FineErogazione;	
			}
			break;
		case FineErogazione:
			m_restart = False;
			if(m_timerEndSeq.Match())
			{
				if(StatusCmd.tot_enabled_lines > 0)
				{
					StatusCmd.tot_enabled_lines--;			// decremento le vie da fare
				}
				else
				{
					StatusCmd.status = STATO_ERRORE;
					StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
					StatusCmd.error_monitor = 	ERR_CASE_TOT_EN_LINES;
				}
				bk.num_vie_da_eseguire = StatusCmd.tot_enabled_lines;
				bk.nextLine = StatusCmd.next_line = ViaDaEseguire();		// setta la via più bassa della sequenza ancora da eseguire
				if((StatusCmd.tot_enabled_lines < 1) || (StatusCmd.next_line < 0))		// nessuna via da eseguire, ho finito la sequenza	
				{
					m_timerEndSeq.Preset(_t_BLINK_STOP);			// tempo di attesa prima di chiudere la sequenza
					g_stateMachine.saccaState = FineSequenza;		// ho finito, vado allo stato di chiusra sequenza
				}
				else
				{
					g_stateMachine.saccaState = StartSequenza;		// ritorno all'avvio riempimento, sulla nuova via
				}
			}		
			break;
		case FineSequenza:
			if(m_timerEndSeq.Match())
			{
				StatusCmd.status = STATO_FINE_SEQ;
				StatusCmd.m_FillingOnCourse = False;
				Backup_StatoStart();
				BackUpTimer.Stop();
				return True;
			}
			break;
		default:
			StatusCmd.status = STATO_ERRORE;
			StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
			StatusCmd.error_monitor = 	ERR_CASE_SACCA_MNGR;
			break;
	}

	if(BackUpTimer.Match())		// messo qui il backup si fa solo se siamo in sacca: se stop o errore non c'è backup
	{
		BackUpTimer.Preset(_TIME_x_BACKUP);
		Backup_StatoStart();
	}

	air_errors = StatusCmd.statusChan[_ADC2_] & 0xFF;
	if(air_errors > 0)
	{
		if((air_errors & (0x01 << StatusCmd.next_line)) > 0)		// se c'è aria sulla via in corso deve andare in errore
		{
			StatusCmd.flagsErrorLine = 0x01;
		}
	}
	
	return False;
}

/**
Manager della fase riempitubi: una sola via alla volta settata in decode_start_msg e indicata dalla variabile line
*/
bool SIFRA_Manager::SIFRAManual_Manager(int line)
{

switch(g_stateMachine.stState)
{
	case IdleManuale:
			break;
	case StartManuale:
		if(m_dato_valido)
		{
			m_dato_valido = False;
			Via[line].peso_da_erogare = StartCmd.peso_linea[line];
			Via[line].peso_spec = AdattaPesoSpecifico(StartCmd.peso_spec[line]);
			Via[line].peso_erogato = bk.peso_erogato[line];
			if(Via[ line ].peso_da_erogare >= Via[ line ].peso_erogato)
			{
				if((Via[ line ].peso_da_erogare - Via[ line ].peso_erogato) < _VOL_MIN_x_RESTART)	// se c'è meno di un grammo da erogare
				{
					m_timerEndSeq.Preset(_WAIT_M300_READING);
					g_stateMachine.stState = ChiusuraManuale;
					break;
				}
				else
				{
					if(!controllo_capacita_iniziale(StartCmd.cap_linea[StatusCmd.next_line]))		// controlla la capacità passata da tastiera
					{
						break;
					}
					Via[line].peso_gia_erogato = Via[line].peso_erogato;
					Via[line].peso_iniziale = (word)m_peso_attuale;
					if(Via[line].peso_da_erogare >= Via[line].peso_gia_erogato)
					{
						Via[line].peso_finale = Via[line].peso_iniziale - (Via[line].peso_da_erogare - Via[line].peso_gia_erogato);
					}
					else
					{
						m_timerEndSeq.Preset(_WAIT_M300_READING);
						g_stateMachine.stState = ChiusuraManuale;
						break;
					}
					Via[ line ].abilitazione = True;
					StatusCmd.m_FillingOnCourse = True;						// è un riempimento a tutti gli fetti
					StatusCmd.air_block_en = False;							// disattivazione blocco x aria
					setta_led_pannello(line, ON);
					SIFRAOpenLineEV(line);
					g_stateMachine.stState = Apert_EV_Manuale;
				}
			}
			else
			{
				m_timerEndSeq.Preset(_WAIT_M300_READING);
				g_stateMachine.stState = ChiusuraManuale;
				break;
			}
		}	
		break;
	case Apert_EV_Manuale:
		if(m_timer_openEV.Match())
		{				
			ResetMotorStepCounter();// reset dei contatori encoder
			setStatePump(0, ABILITAZIONE);	// fa partire la pompa dedicata o quella unica
			m_vel = _PWM_CYCLE_MANUAL_SPEED;
			m_rampa_timer.Preset(_TIMER_READ_AIR);	
			m_check_block_pump.Preset(_WAIT_M300_READING);
			m_timerVerificaCalo.Preset(T_NO_CALO_MANUAL);
			g_stateMachine.stState = ControlloManuale;	// possiamo partire
		}
		break;
	case ControlloManuale:
		if(m_dato_valido)									// se c'è una nuova tornata di dati validi (uno per ogni via)
		{
			m_dato_valido = False;
			if(Via[line].peso_iniziale >= m_peso_attuale)
			{
				m_peso_parziale = Via[line].peso_iniziale - (word)m_peso_attuale;
			}
			else
			{
				m_peso_parziale = 0;
			}
			Via[line].peso_erogato = m_peso_parziale + Via[line].peso_gia_erogato;
			Via[line].capacita = aggiorna_capacita(StartCmd.cap_linea[line],  m_peso_parziale, Via[line].peso_spec);
			if(Via[line].peso_erogato < Via[line].peso_da_erogare)
			{
				if((Via[line].peso_da_erogare - Via[line].peso_erogato) < LIMITE_PESO)				// gestisce rampa e riempimento
				{	
					m_vel = _PWM_CYCLE_LOW;		// velocità intermedia massima da ragiungere
					g_stateMachine.stState = ControlloManuale_Fine;
					break;
				}
				StatusCmd.flagsErrorLine |= controllo_vuoto(line);
				StatusCmd.flagsErrorLine |= controllo_non_cala(line, _NON_CALA_THRESHOLD);	// verifica delle situazioni di non cala e vuoto, viene settato
			}
			else
			{
				m_check_block_pump.Stop();
				m_timerVerificaCalo.Stop();
				setStatePump(0, ARRESTO);
				SIFRACloseLineEV(line);							
				m_timerEndSeq.Preset(_WAIT_M300_READING);
				g_stateMachine.stState = ChiusuraManuale;
				break;
			}
			SIFRASpeedPump_Manager(0);					// gestione rampa di accelerazione (non c'è frenata)
		}	
		break;	
	case ControlloManuale_Fine:
		if(m_dato_valido)	// se c'è una nuova tornata di dati validi (uno per ogni via)
		{
			m_dato_valido = False;
			if(Via[line].peso_iniziale >= m_peso_attuale)
			{
				m_peso_parziale = Via[line].peso_iniziale - (word)m_peso_attuale;
			}
			else
			{
				m_peso_parziale = 0;
			}
			Via[line].peso_erogato = m_peso_parziale + Via[line].peso_gia_erogato;
			Via[line].capacita = aggiorna_capacita(StartCmd.cap_linea[line],  m_peso_parziale, Via[line].peso_spec);
			if(Via[line].peso_erogato >= (Via[line].peso_da_erogare - LIMITE_PESO_ENCODER))	// gestisce rampa e riempimento
			{
				m_check_block_pump.Stop();
				m_timerVerificaCalo.Stop();
				setStatePump(0, ARRESTO);
				SIFRACloseLineEV(line);							
				m_timerEndSeq.Preset(_WAIT_M300_READING);
				g_stateMachine.stState = ChiusuraManuale;
				break;
			}
			StatusCmd.flagsErrorLine |= controllo_vuoto(line);
			StatusCmd.flagsErrorLine |= controllo_non_cala(line, _NON_CALA_THRESHOLD);	// verifica delle situazioni di non cala e vuoto, viene settato
		}
		SIFRASpeedPump_Manager(0);						// gestione rampa di accelerazione (non c'è frenata)
		break;
		case ChiusuraManuale:
			if(Via[line].peso_iniziale >= m_peso_attuale)
			{
				m_peso_parziale = Via[line].peso_iniziale - (word)m_peso_attuale;
			}
			else
			{
				m_peso_parziale = 0;
			}
			Via[line].peso_erogato = m_peso_parziale + Via[line].peso_gia_erogato;
			Via[line].capacita = aggiorna_capacita(StartCmd.cap_linea[line],  m_peso_parziale, Via[line].peso_spec);
			if(m_timerEndSeq.Match())
			{
				Via[line].eseguita = True;
				Via[line].abilitazione = False;
				m_peso_stop = m_peso_attuale;
				StatusCmd.statusChan[_ADC2_] &= ~(0x0001 << line);
				StatusCmd.flagsErrorLine = 0;
				Via[line].stato_luci = IDLE_LUCE;
				m_air_manager[line] = E_CONTROL_AIR;
				ResetMotorStepCounter();
				StatusCmd.m_FillingOnCourse = False;
				StatusCmd.status = STATO_STOP;
				return True;
			}
			break;
		default:
			StatusCmd.status = STATO_ERRORE;
			StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
			StatusCmd.error_monitor = 	ERR_CASE_MANUAL_MNGR;
			break;
	}

	if(BackUpTimer.Match())		// messo qui il backup si fa solo se siamo in sacca: se stop o errore non c'è backup
	{
		BackUpTimer.Preset(_TIME_x_BACKUP);
		Backup_StatoStart();
	}

	return False;
}

/**
* Manager dello stato riempitubi: può uscire per una situaizone di errore oppure perchè ha finito, tornando True.
* Altrimenti torna False
* Nel RT la rilevazione dell'aria è attiva e gestisce il led rosso, ma non è bloccante. Però se al raggiunto target di peso, c'è ancora aria, allora avanza per ulteriore
* quantità nel tentativo di eliminare l'aria davanti al sensore.
* Nel riempitubi di 15, è eseguita anche la calibrazione del rapporto flussopeso: per questo motivo un eventuale stop durante la via 15, fa resettare il peso erogato,
* e alla ripresa, riparte da 0, questo per poter fare un conto di calibrazione sempre su una quantità sufficiente da poter trascurare la tolleranza dei tubi.
*/
bool SIFRA_Manager::SIFRART_Manager(int line)
{
	int set_velocita = 0;

	switch(g_stateMachine.rtState)
	{
		case IdleRiempitubi:
			break;
		case SetupRiempitubi:
			if(m_dato_valido)
			{
				m_dato_valido = False;
				ResetMotorStepCounter();
				StatusCmd.next_line = line;
				Via[line].da_eseguire = True;
				Via[line].capacita = Via[line].capacita - m_calo_volume_stop;
				Via[line].peso_da_erogare = StartCmd.peso_linea[line];
				Via[line].peso_spec = AdattaPesoSpecifico(StartCmd.peso_spec[line]);
				Via[line].abilitazione = True;
				if(line == (NUM_MAX_LINE-1))		// se si tratta di via 15 devo calcolare il rapporto flusso peso
				{
					m_RT_con_calib = True;
				}
				Via[line].peso_iniziale = (word)m_peso_attuale;
				if(Via[line].peso_iniziale >= Via[line].peso_da_erogare)
				{
					Via[line].peso_finale = Via[line].peso_iniziale - Via[line].peso_da_erogare;
				}
				else
				{
					StatusCmd.status = STATO_ERRORE;
					StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
					StatusCmd.error_monitor = 	ERR_CASE_PESO_FINALE_RT_MNGR;
					break;
				}
				m_calo_volume_stop = 0;
				StatusCmd.m_FillingOnCourse = True;						// si attiva il controllo motori in caso di errore: i motori sono spenti..
				setta_led_pannello(line, ON);
				g_stateMachine.rtState = StartRiempitubi;						// stato interno allo stato del riempitubi
			}
			break;
		case StartRiempitubi:
			if(!controllo_capacita_iniziale(StartCmd.cap_linea[line]))		// controlla la capacità passata da tastiera
			{
				break;
			}
			SIFRAOpenLineEV(line);
			g_stateMachine.rtState = StartPompaRiempitubi;			
			break;
		case StartPompaRiempitubi:
			if(m_timer_openEV.Match())
			{
				setStatePump(0, ABILITAZIONE);									// fa partire la pompa dedicata o quella unica
				//m_vel = _PWM_CYCLE_MIN;	
				m_timer_RT.Preset(_TIMER_LED_MEDIUM_);				
				g_stateMachine.rtState = SetVelocitaRiempitubi;						// possiamo partire
			}
			break;
		case SetVelocitaRiempitubi:
			if(m_timer_RT.Match())
			{
				m_timer_RT.Stop();
				set_velocita = PRIMA_VELOCITA;
				setPwmMotCycle(0, set_velocita);
				m_check_block_pump.Preset(_WAIT_M300_READING);
				m_timerVerificaCalo.Preset(T_NO_CALO_INIT);
				g_stateMachine.rtState = ControlloFase1;
			}
			break;
		case ControlloFase1:
			if(Via[line].peso_erogato > _VOL_PRIMOSTEP)
			{
				set_velocita = SECONDA_VELOCITA;
				setPwmMotCycle(0, set_velocita);
				g_stateMachine.rtState = ControlloFase2;
				break;											// esco subito per passare alla fase a velocità alta
			}
			if(m_dato_valido)										// se c'è una nuova tornata di dati validi (uno per ogni via)
			{
				m_dato_valido = False;
				if(Via[line].peso_iniziale >= m_peso_attuale)
				{
					Via[line].peso_erogato = Via[line].peso_iniziale - (word)m_peso_attuale;
				}
				else
				{
					Via[line].peso_erogato = 0;
				}
				Via[line].capacita = aggiorna_capacita(StartCmd.cap_linea[line],  Via[line].peso_erogato, Via[line].peso_spec);
				StatusCmd.flagsErrorLine |= controllo_vuoto(line);
				StatusCmd.flagsErrorLine |= controllo_non_cala(line, _NON_CALA_THRESHOLD);	// verifica delle situazioni di non cala e vuoto, viene settato
			}
			break;	
		case ControlloFase2:
			if(Via[line].peso_erogato > (Via[line].peso_da_erogare - LIMITE_PESO_RT))
			{
				if((StatusCmd.statusChan[_ADC2_] & (0x0001 << line)) == 0)
				{
					m_check_block_pump.Stop();
					m_timerVerificaCalo.Stop();
					setStatePump(0, ARRESTO);
					SIFRACloseLineEV(line);
					g_stateMachine.rtState = AttendiUltimoPeso;
					m_timer_fineErog.Preset(_500_mSEC_);
					break;			// metto il break così se ho finito, non mi faccio condizionare da errori successivi
				}
				else
				{
					g_stateMachine.rtState = ErogazioneAggiuntiva;
					break;
				}
			}
			if(m_dato_valido)															// se c'è una nuova tornata di dati validi (uno per ogni via)
			{
				m_dato_valido = False;
				if(Via[line].peso_iniziale >= m_peso_attuale)
				{
					Via[line].peso_erogato = Via[line].peso_iniziale - (word)m_peso_attuale;
				}
				else
				{
					StatusCmd.status = STATO_ERRORE;
					StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
					StatusCmd.error_monitor = 	ERR_CASE_PESO_EROGATO_RT_MNGR;
					break;
				}
				Via[line].capacita = aggiorna_capacita(StartCmd.cap_linea[line],  Via[line].peso_erogato, Via[line].peso_spec);
				StatusCmd.flagsErrorLine |= controllo_vuoto(line);
				StatusCmd.flagsErrorLine |= controllo_non_cala(line, _NON_CALA_THRESHOLD);	// verifica delle situazioni di non cala e vuoto, viene settato
			}
			break;
		case ErogazioneAggiuntiva:
			if(Via[line].peso_erogato >= (Via[line].peso_da_erogare + _VOL_PRIMOSTEP))
			{
				m_check_block_pump.Stop();
				m_timerVerificaCalo.Stop();
				setStatePump(0, ARRESTO);
				SIFRACloseLineEV(line);
				g_stateMachine.rtState = AttendiUltimoPeso;
				m_timer_fineErog.Preset(_500_mSEC_);
				break;			// metto il break così se ho finito, non mi faccio condizionare da errori successivi
			}
			if(m_dato_valido)	// se c'è una nuova tornata di dati validi (uno per ogni via)
			{
				m_dato_valido = False;
				if(Via[line].peso_iniziale >= m_peso_attuale)
				{
					Via[line].peso_erogato = Via[line].peso_iniziale - (word)m_peso_attuale;
				}
				else
				{
					StatusCmd.status = STATO_ERRORE;
					StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
					StatusCmd.error_monitor = 	ERR_CASE_PESO_EROGATO_RT_MNGR;
					break;
				}
				Via[line].capacita = aggiorna_capacita(StartCmd.cap_linea[line],  Via[line].peso_erogato, Via[line].peso_spec);
			}
			StatusCmd.flagsErrorLine |= controllo_vuoto(line);
			StatusCmd.flagsErrorLine |= controllo_non_cala(line, _NON_CALA_THRESHOLD);	// verifica delle situazioni di non cala e vuoto, viene settato
			break;
		case AttendiUltimoPeso:
			if(Via[line].peso_iniziale >= m_peso_attuale)
			{
				Via[line].peso_erogato = Via[line].peso_iniziale - (word)m_peso_attuale;
			}
			else
			{
				StatusCmd.status = STATO_ERRORE;
				StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
				StatusCmd.error_monitor = 	ERR_CASE_PESO_EROGATO_RT_MNGR;
				break;
			}
			if(m_timer_fineErog.Match())
			{
				if(m_dato_valido)	// se c'è una nuova tornata di dati validi
				{
					m_timer_fineErog.Stop();
					m_dato_valido = False;
					Via[line].capacita = aggiorna_capacita(StartCmd.cap_linea[line],  Via[line].peso_erogato, Via[line].peso_spec);
					m_timerEndSeq.Preset(_500_mSEC_);
					g_stateMachine.rtState = ChiusuraRiempitubi;
				}
			}
			break;
		case ChiusuraRiempitubi:
			if(m_timerEndSeq.Match())
			{
				Via[line].abilitazione = False;	// questa via ha finito
				Via[line].eseguita = True;
				if (check_conditions(line))
				{
					if(m_RT_con_calib == True)	// se siamo alla via acqua
					{
						m_RT_con_calib = False;
						if(salva_rapporto_flussopeso(line))
						{
							if(!Write_Param_Encoder())
							{
								//StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
								//StatusCmd.flagsErrorLine = 0x01;
								StatusCmd.error_monitor = ERR_RAM_WRITE_ENC;
							}
						}
						else
						{
							StatusCmd.statusChan[_ADC1_] |= ERR_FLUSSO;
							StatusCmd.statusChan[_ADC2_] |= (0x0100 << line);
						}
					}
					/*	IN RIEMPIMENTO LINEE NON HA SENSO SOLLEVARE L'ERRORE PRODOTTO, ANCHE PER UNIFORMITA' CON MODULO M3100
					else
					{
						m_controllo_errore_volume = check_volume_error(line); 	// verifica di errore flusso peso e/o prodotto non corrispondente
					}
					*/
				}
				else
				{
					m_controllo_errore_volume = errore_generico;
				}
				if((StatusCmd.statusChan[_ADC2_] & (0x0001 << line)) == 0)
				{
					setta_led_pannello(line, OFF);
				}
				else		// se c'è ancora allarme aria lascio accesa la luce
				{
					setta_led_pannello(line, ON);
					Via[line].stato_luci = ERR_CELLA;
				}
				StatusCmd.status = STATO_FINE_SEQ;
				StatusCmd.m_FillingOnCourse = False;
				return True;
			}
			break;
		default:
			StatusCmd.status = STATO_ERRORE;
			StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
			StatusCmd.error_monitor = 	ERR_CASE_RT_MNGR;
			break;
	}
	
	return False;
}

/**
* Funzione gestione risciacquo dalla via di spinta.
* Rilevazione aria attivo e bloccante.
*/
bool SIFRA_Manager::SIFRARisciacquo_Manager(int line)
{
	switch(g_stateMachine.rtState)
	{
		case IdleRiempitubi:
			break;
		case SetupRiempitubi:
			if(m_dato_valido)
			{
				m_dato_valido = False;
				Via[line].peso_da_erogare = StartCmd.peso_linea[line];
				Via[line].peso_spec = AdattaPesoSpecifico(StartCmd.peso_spec[line]);
				Via[line].da_eseguire = True;
				Via[line].abilitazione = True;
				Via[line].peso_iniziale = (word)m_peso_attuale;
				Via[line].peso_finale = Via[line].peso_iniziale - Via[line].peso_da_erogare;
				if(Via[line].peso_finale >= m_peso_attuale) 	// se ho già erogato tutto, in caso di rirpesa da stop
				{
					m_timerEndSeq.Preset(_WAIT_M300_READING);
					g_stateMachine.rtState = ChiusuraRiempitubi;
				}
				else
				{	
					StatusCmd.m_FillingOnCourse = True;				// si attiva il controllo motori in caso di errore: i motori sono spenti..
					StatusCmd.air_block_en = True;						// e blocca in caso di aria per non riempire il tubo di aria
					setta_led_pannello(line, ON);	
					g_stateMachine.rtState = StartRiempitubi;				// stato interno allo stato del riempitubi
				}
			}
			break;
		case StartRiempitubi:
			if(!controllo_capacita_iniziale(StartCmd.cap_linea[line]))		// controlla la capacità passata da tastiera
			{
				break;
			}
			SIFRAOpenLineEV(line);
			g_stateMachine.rtState = StartPompaRiempitubi;			
			break;
		case StartPompaRiempitubi:
			if(m_timer_openEV.Match())
			{	
				StatusCmd.next_line = line;
				ResetMotorStepCounter();							// reset dei contatori encoder
				setStatePump(0, ABILITAZIONE);						// fa partire la pompa
				m_vel = _PWM_CYCLE_MANUAL_SPEED;
				setPwmMotCycle(0,m_vel);
				m_rampa_timer.Preset(_TIMER_READ_AIR);	
				m_check_block_pump.Preset(_WAIT_M300_READING);
				m_timerVerificaCalo.Preset(T_NO_CALO_INIT);
				g_stateMachine.rtState = ControlloFase1;					// possiamo partire
			}
			break;	
		case ControlloFase1:
			if(m_dato_valido)				// se c'è una nuova tornata di dati validi (uno per ogni via)
			{
				m_dato_valido = False;
				if(Via[line].peso_iniziale >= m_peso_attuale)
				{
					Via[line].peso_erogato = Via[line].peso_iniziale - (word)m_peso_attuale;
				}
				else
				{
					Via[line].peso_erogato = 0;
				}
				Via[line].capacita = aggiorna_capacita(StartCmd.cap_linea[line],  Via[line].peso_erogato, Via[line].peso_spec);
				StatusCmd.flagsErrorLine |= controllo_vuoto(line);
				StatusCmd.flagsErrorLine |= controllo_non_cala(line, _NON_CALA_THRESHOLD);	// verifica delle situazioni di non cala e vuoto, viene settato
				if(Via[line].peso_erogato > LIMITE_PESO)
				{
					m_vel = _PWM_CYCLE_CLEANING_SPEED;		// velocità massima da raggiungere
					setPwmMotCycle(0,m_vel);
					g_stateMachine.rtState = ControlloFase2;
				}
				break;
				/*
				if(m_peso_attuale >= Via[line].peso_finale)
				{
					if(((word)m_peso_attuale - Via[line].peso_finale) < LIMITE_PESO)	// gestisce rampa e riempimento
					{	
						m_vel = _PWM_CYCLE_MIN;		// velocità intermedia massima da raggiungere
						setPwmMotCycle(0,m_vel);
						g_stateMachine.rtState = ControlloFase2;
						break;
					}	
				}
				else
				{
					m_check_block_pump.Stop();
					m_timerVerificaCalo.Stop();
					setStatePump(0, ARRESTO);
					SIFRACloseLineEV(line);
					m_timerEndSeq.Preset(_WAIT_M300_READING);
					g_stateMachine.rtState = ChiusuraRiempitubi;
					break;
				}
				*/
			}
			SIFRASpeedPump_Manager(0);	// gestione rampa di accelerazione (non c'è frenata)
			break;
		case ControlloFase2:		// avanzo ancora un certo volume perchè c'è aria davanti al sensore			
			if(m_dato_valido)											// se c'è una nuova tornata di dati validi (uno per ogni via)
			{
				m_dato_valido = False;
				if(Via[line].peso_iniziale >= m_peso_attuale)
				{
					Via[line].peso_erogato = Via[line].peso_iniziale - (word)m_peso_attuale;
				}
				else
				{
					StatusCmd.status = STATO_ERRORE;
					StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
					StatusCmd.error_monitor = 	ERR_CASE_PESO_PARZIALE_RINSE_MNGR;
					break;
				}
				Via[line].capacita = aggiorna_capacita(StartCmd.cap_linea[line],  Via[line].peso_erogato, Via[line].peso_spec);
				StatusCmd.flagsErrorLine |= controllo_vuoto(line);
				StatusCmd.flagsErrorLine |= controllo_non_cala(line, _NON_CALA_THRESHOLD);	// verifica delle situazioni di non cala e vuoto, viene settato
				if(m_peso_attuale <= Via[line].peso_finale)
				{
					m_check_block_pump.Stop();
					m_timerVerificaCalo.Stop();
					SIFRACloseLineEV(line);
					setStatePump(0, ARRESTO);
					m_timerEndSeq.Preset(_WAIT_M300_READING);
					g_stateMachine.rtState = ChiusuraRiempitubi;
					break;
				}
			}
			//SIFRASpeedPump_Manager(0);								// gestione rampa di accelerazione (non c'è frenata)
			break;
		case ChiusuraRiempitubi:
			if(m_dato_valido)	// se c'è una nuova tornata di dati validi (uno per ogni via)
			{
				m_dato_valido = False;
				if(Via[line].peso_iniziale >= m_peso_attuale)
				{
					Via[line].peso_erogato = Via[line].peso_iniziale - (word)m_peso_attuale;
				}
				else
				{
					StatusCmd.status = STATO_ERRORE;
					StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
					StatusCmd.error_monitor = 	ERR_CASE_CHIUSURA_RINSE_MNGR;
					break;
				}
				Via[line].capacita = aggiorna_capacita(StartCmd.cap_linea[line],  Via[line].peso_erogato, Via[line].peso_spec);
			}	
			if(m_timerEndSeq.Match())
			{
				Via[line].abilitazione = False;	// questa via ha finito oppure è in errore non recuperabile
				Via[line].eseguita = True;
				setta_led_pannello(line, OFF);
				/*	IN LAVAGGIO PER ORA NON CONTROLLO ERRORE PRODOTTO O FLUSSO
				if(check_conditions(line))
				{
					m_controllo_errore_volume = check_volume_error(line); 	// verifica di errore flusso peso e/o prodotto non corrispondente
				}
				*/
				StatusCmd.status = STATO_FINE_SEQ;
				StatusCmd.m_FillingOnCourse = False;
				return True;
			}
			break;
		default:
			StatusCmd.status = STATO_ERRORE;
			StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
			StatusCmd.error_monitor = 	ERR_CASE_RISCIACQUO_MNGR;
			break;
	}
	return False;
}

/**
* Manager della fase di svuotamento manuale: l'utente dopo aver sconnesso le sacche sulle culle vuole svuotare la linea stessa per evitare perdite d'acqua
* durante lo smontaggio linea. La tastiera comanda l'avanzamento manuale della pompa a suo piacimento. C'è solo un limite massimo di azionamento continuo.
*/
bool SIFRA_Manager::SIFRAST_Manager(int line)
{
	switch(g_stateMachine.stState)
	{
		case IdleManuale:
			break;
		case StartManuale:
			Via[line].abilitazione = False;				// non serve gestire l'abilitazione, non ci sono pesi e/o backup da gestire	
			StatusCmd.flagsErrorLine = NO_ERRORS;				// nessun errore
			StatusCmd.m_FillingOnCourse = False;		// disattivo il controllo, così l'allarme instabile non blocca
			StatusCmd.air_block_en = False;			// diattivazione blocco x aria, l'allarme aria non è bloccante e gestisce il led	
			setta_led_pannello(line, ON);				// accensione led via
			SIFRAOpenLineEV(line);					// apertura ev
			g_stateMachine.stState = Apert_EV_Manuale;	
			break;
			
		case Apert_EV_Manuale:
			if(m_timer_openEV.Match())
			{				
				ResetMotorStepCounter();				// reset dei contatori encoder
				setStatePump(0, ABILITAZIONE);			// fa partire la pompa dedicata o quella unica
				m_vel = _PWM_CYCLE_SPEED_ST;	
				m_rampa_timer.Preset(_TIMER_READ_AIR);	
				m_check_block_pump.Preset(_1_SEC_);
				m_timerVerificaCalo.Preset(T_NO_CALO_INIT);
				g_stateMachine.stState = ControlloManuale;	// possiamo partire
			}
			break;

		case ControlloManuale:
			SIFRASpeedPump_Manager(0);	// gestione rampa di accelerazione (non c'è frenata)
			if(MotorStatus.step_motor_done > 0)
			{
				m_volMaxST = (word)(MotorStatus.step_motor_done / structEnc.param_encoder);	// in decimi di grammi
			}
			else
			{
				m_volMaxST = 0;
			}
			if(m_volMaxST > _VOL_ST_MAX_)	// gestisce rampa e svuotamento
			{	// metto un limite, con tanti cc la linea è già vuota di sicuro
				SIFRACloseLineEV(line);
				setStatePump(0, ARRESTO);		// ferma immediatamente il motore	
				m_check_block_pump.Stop();
				return 1;
			}		
			break;
		default:
			StatusCmd.status = STATO_ERRORE;
			StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
			StatusCmd.error_monitor = 	ERR_CASE_ST_MNGR;
			break;
	}
	return 0;
}

/**
* It manages the filling of m_line, using line pump m_load, controlling increasing and decreasing speed ramp
* In M3300 m_line is the line to fill, while m_load is always 0, because there is only a pump
* Return true if target reached or a error occurred.
*/
bool SIFRA_Manager::SIFRARampe_Manager( byte m_line, byte m_load)
{
	word p_m;
	byte vel_mot;
	int t_vel;

	if(m_peso_attuale > Via[m_line].peso_finale)
	{
		p_m = (word)m_peso_attuale - Via[m_line].peso_finale;		// peso ancora da erogare (decimi di grammo)
	}
	else
	{
		p_m = 0;
		/*
		StatusCmd.flagsErrorLine = 0x01;
		StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
		StatusCmd.error_monitor = 	ERR_CASE_VARIAZIONE_PESO;
		*/
		return True;
	}
	
	if(p_m < LIMITE_PESO_MINIMO)			// se abbiamo superato il limite o ci siamo proprio vicinissimi
	{
		m_check_block_pump.Stop();
		setPwmMotCycle(m_load, _PWM_CYCLE_0VEL);
		m_rampa_timer.Stop();
		return True;
	}	
	else	
	{
		if( p_m < LIMITE_PESO)		// mi sto avvicinando al target, procederò piano
		{
			if( p_m < LIMITE_PESO_PRECISIONE)
			{
				setPwmMotCycle(m_load, _PWM_CYCLE_MIN);
			}
			else
			{
				setPwmMotCycle(m_load, _PWM_CYCLE_LOW);
			}
		}
		else	
		{
		
			t_vel = (int)(7 * (float)(p_m /10));	// se 70 è vel massima, allora questa riga impone come 50g limite per cominciare la decelerazione
			if(t_vel > _PWM_CYCLE_MAX)
			{
				m_vel = _PWM_CYCLE_MAX;
			}
			else	
			{
				if(t_vel < _PWM_CYCLE_MIN)
				{
					m_vel = _PWM_CYCLE_MIN;
				}
				else
				{
					m_vel = (byte)t_vel;
				}
			}
			vel_mot = getSpeedPump(m_load);
			if(vel_mot < m_vel)
			{
				if(m_rampa_timer.Match())
				{
					m_rampa_timer.Preset(_TIMER_READ_AIR);
					vel_mot += 1;
					setPwmMotCycle(m_load, vel_mot);
				}
			}
			else
			{
				vel_mot = m_vel;
				setPwmMotCycle(m_load, vel_mot);
			}
			

		/*
			if(m_rampa_timer.Match())
			{
				m_rampa_timer.Preset(_TIMER_READ_AIR);
				vel_mot ++;
				setPwmMotCycle(m_load, vel_mot);
			}
			*/
		}
	}
	
	return False;
}

int SIFRA_Manager::SIFRASpeedPump_Manager( byte m_load)
{
	byte vel_mot;

	vel_mot = getSpeedPump(m_load);
	if(vel_mot < m_vel)
	{
		if(m_rampa_timer.Match())
		{
			m_rampa_timer.Preset(_t_RAMPA_RT);
			vel_mot++;
		}
	}
	else
	{
		vel_mot = m_vel;
	}
	setPwmMotCycle(m_load, vel_mot);
	
	return (int)vel_mot;
}

/**
* Get new samples from FIFO's weight channel and give them to system manager.
* If stability verify is enable for the channel, matches new sample with previous: difference has to be lower than threshold sets by system (it dipends on status system).
* If difference is lower or higher of threshold, instability error of the channel is set.

* Il valore m_peso_attuale viene aggiornato ad ogni nuova lettura dalla coda dei nuovi campioni m_WEIGHSAMPLE solo se stabile.
* Il valore letto (uguale al m_peso_attuale) è confrontato con la forchetta prevista di accettazione valore.
* Questa forchetta ha larghezza costante pari a 2*THREESHOLD_FILLING_INCR, ma il suo valore medio è invece dato dal valore calcolato al precedente accesso;
* questo valore calcolato è dato dal valore precedente meno un delta calcolato in base alla velocità istantanea della pompa; 
*/
bool SIFRA_Manager::SIFRASample_Manager()
{
	long m_val_stima;
	word next_delta_stim;
	ViaSample m_via_sample;
	
	long valore_letto = 0;
	byte i;

	if( weightChan->popAdcData(m_WeightSample) )		// carica i dati (word) di tutti i canali fisici
	{	// il peso attuale è indicato in centesimi di ml, per migliorare gli arrotondamenti
		if(!m_restore_stability.getStatus())
		{
			valore_letto = m_SIFRAProtocol->sendSIFRALoadSamples(GetSystemLoad(m_WeightSample));
			m_weight_real_time = (word)m_SIFRAProtocol->getLoadChan();
			m_num_sample_mgr++;
		}
		else
		{
			if(m_restore_stability.Match())
			{
				StatusCmd.statusChan[_ADC1_] &= ~ERR_PESOINSTABILE;		// resetto l'errore
				if(!(StatusCmd.statusChan[_ADC1_] & ERR_CELLA_ROTTA_SX))		// se non è già attivo l'errore di cella rotta
				{
					if(!(StatusCmd.statusChan[_ADC1_] & ERR_CELLA_ROTTA_DX))		// se non è già attivo l'errore di cella rotta
					{
						StatusCmd.flagsErrorLine = 0x00;
						for(i=0;i< NUM_MAX_LINE; i++)
						{
							Via[i].stato_luci = IDLE_LUCE;					// così per questa vi comincia la gestione del lampeggio indipendente dalle altre
						}
					}
				}
				g_stateMachine.sampleState = init_stab;
				Via_Sample.clear();
				valore_letto = m_SIFRAProtocol->sendSIFRALoadSamples(GetSystemLoad(m_WeightSample));
				m_weight_real_time = (word)m_SIFRAProtocol->getLoadChan();
				m_restore_stability.Stop();
			}
			else
			{
				return False;
			}
		}

		//m_num_sample_mgr++;
		next_delta_stim = calc_next_theor_weight(0);		// calcola il teorico deltapeso al prossimo campionamento, in base alla velocità della pompa
		
		switch(g_stateMachine.sampleState)
		{
			case init_stab:										// inizializzazione, la coda è vuota
				m_via_sample.this_sample = valore_letto;
				m_via_sample.next_sample = valore_letto - (long)next_delta_stim;	// il prossimo valore stimato è calcolato dall'attuale meno la stima
				m_peso_attuale = valore_letto;
				if(!Via_Sample.full())								// se non è piena, aggiungo un dato
				{
					Via_Sample.push_back(m_via_sample);
				}
				if(Via_Sample.numItem() >= _NUM_MIN_DATA_in_FIFO)   // se ci sono almeno 2 campioni
				{
					g_stateMachine.sampleState = stabile;							// allora posso iniziare il controllo stabilità
				}
				break;
			case stabile:	
				if(!Via_Sample.empty())							// se la FIFO non è vuota
				{
					Via_Sample.readLastPushed(m_via_sample);		// leggo l'ultimo inserito senza rimuoverlo,che è il campione precedente all'attuale
					Via_Sample.clear();							// poi svuoto la FIFO
					// eseguo il confronto per la verifica della stabilità
					m_val_stima = m_via_sample.next_sample;		// valore centrale della forchetta di accettazione
					if((valore_letto > (m_val_stima - m_weightUnstableLevel))
									&& (valore_letto < (m_val_stima + m_weightUnstableLevel)))
					{	// se è dentro forchetta, calcolo il next_sample del nuov campione e lo metto in FIFO (che così avrà due valori stabili)
						m_via_sample.this_sample = valore_letto;
						m_via_sample.next_sample = valore_letto - (long)next_delta_stim;	// il prossimo valore stimato è calcolato dall'attuale meno la stima
						Via_Sample.push_back(m_via_sample);		// inserisco il nuovo valore in FIFO
						m_peso_attuale = valore_letto;				// rilascio il nuovo peso attuale come quello misurato e "approvato"
						g_stateMachine.sampleState = stabile;						// non cambio stato: ho almeno 2 campioni in FIFO
						return True;
					}
					else
					{	// se è fuori forchetta, calcolo il next sample a partire dall'ultimo stabile, che qui è il penultimo campione letto
						m_via_sample.this_sample = m_val_stima;
						m_via_sample.next_sample = m_val_stima - (long)next_delta_stim;	// il prossimo valore stimato è calcolato dall'attuale meno la stima
						Via_Sample.push_back(m_via_sample);		// inserisco il nuovo valore in FIFO
						g_stateMachine.sampleState = forse_instabile;				// cambio stato con almeno 2 campioni in FIFO
					}
					
				}
				else		// la coda di campioni ViaSample non deve mai essere vuota
				{
					g_stateMachine.sampleState = E_VOID_QUEUE;
				}
				break;
			case forse_instabile:	// stato in cui a fronte di un valore instabile, si verifica se è stato uno spurio oppure se c'è una instabilità vera
				if(!Via_Sample.empty())							// se la FIFO non è vuota
				{
					Via_Sample.readLastPushed(m_via_sample);		// leggo l'ultimo inserito senza rimuoverlo,che è il campione precedente all'attuale
					Via_Sample.clear();							// poi svuoto la FIFO
					// eseguo il confronto per la verifica della stabilità
					m_val_stima = m_via_sample.next_sample;		// valore centrale della forchetta di accettazione
					if((valore_letto > (m_val_stima - m_weightUnstableLevel))
									&& (valore_letto < (m_val_stima + m_weightUnstableLevel)))
					{
						// se è dentro forchetta, calcolo il next_sample del nuov campione e lo metto in FIFO (che così avrà due valori stabili)	
						m_via_sample.this_sample = valore_letto;
						m_via_sample.next_sample = valore_letto - (long)next_delta_stim;	// il prossimo valore stimato è calcolato dall'attuale meno la stima
						Via_Sample.push_back(m_via_sample);		// inserisco il nuovo valore in FIFO
						m_peso_attuale = valore_letto;				
						g_stateMachine.sampleState = stabile;						// cambio stato: ho 2 campioni in FIFO
					}
					else
					{
						if(m_verifica_bilance)
						{
							Via_Sample.clear();
							g_stateMachine.sampleState = init_stab;
							//m_restore_stability.Preset(_2_SEC_);
						}
						else
						{
							if(!(StatusCmd.statusChan[_ADC1_] & ERR_CELLA_ROTTA_SX))	// se non è già attivo l'errore di peso massimo
							{
								if(!(StatusCmd.statusChan[_ADC1_] & ERR_CELLA_ROTTA_DX))	// se non è già attivo l'errore di peso massimo
								{
									StatusCmd.statusChan[_ADC1_] |= ERR_PESOINSTABILE;	// set errore stabilità
								}
							}
							if((StatusCmd.phase != E_PHASE_FILLING) && (StatusCmd.phase != E_PHASE_SERVICE))
							{
								Via_Sample.clear();
								m_restore_stability.Preset(_2_SEC_);
								g_stateMachine.sampleState = wait_stabile;
							}
							else
							{
								StatusCmd.flagsErrorLine = 0x01;	// setto un errore nella via m_chan solo se abilitata
								m_via_sample.this_sample = m_via_sample.next_sample;
								m_via_sample.next_sample = m_via_sample.this_sample;	// il prossimo valore stimato è calcolato dall'attuale meno la stima
								Via_Sample.push_back(m_via_sample);		// inserisco il valore stimato stabile in FIFO farà ds rifeirmento					
								g_stateMachine.sampleState = instabile;
							}
						}
					}
				}
				else		// la coda di campioni ViaSample non deve mai essere vuota
				{
					g_stateMachine.sampleState = E_VOID_QUEUE;
				}
				break;
			case instabile:
				if(!Via_Sample.empty())	
				{
					Via_Sample.clear();
					m_restore_stability.Preset(_WAIT_3_SEC);
					g_stateMachine.sampleState = wait_stabile;
				}
				else
				{
					g_stateMachine.sampleState = E_VOID_QUEUE;
				}
				break;
			case wait_stabile:
				break;
			case E_VOID_QUEUE:
				StatusCmd.status = STATO_ERRORE;
				StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
				StatusCmd.error_monitor = 	ERR_CASE_CODA_SAMPLE_MNGR;
				break;
			default:
				StatusCmd.status = STATO_ERRORE;
				StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
				StatusCmd.error_monitor = 	ERR_CASE_SAMPLE_MNGR;
				break;
		}
		return False;
	}
	else
	{
		return False;
	}
}

/**
* Manages the air-sensor signals. Sets the third bit of statusChan[_ADC1_] flag byte to 1.

* La funzione controlla tutte le linee continuamente e ne setta il parametro StatusCmd.statusChan[i], quando è rilevata aria. L'informazione serve
* anche nella fase di stop alla tastiera. Se è settato il flag quando è abilitata la variale m_FillingStatus, settando l'eventuale errore anche sulle vie disablitate
* il sistema stoppa tutto appena è rilevato un fronte (quindi anche una bolla isolata) in una delle vie abilitate. Ignora l'allarme nelle vie inattive.
*/
byte SIFRA_Manager::SIFRAAirIn_Manager()
{
	byte m_error_air_flag = (cpld_reg_in & 0x00FF);	// rapporto diretto ALL1-ALL8 bit 1-bit 8 con cablaggio nuovo e scheda UABD M3300
	byte i;
	float flusso_attuale; 	// in decimi di ml

	StatusCmd.statusAirSensors = (cpld_reg_in & 0x00FF);

	for(i = 0; i < NUM_MAX_LINE; i++)
	{	
		if(m_TimerAllarmeAria[i].Match())
		{
			m_TimerAllarmeAria[i].Preset(_TIMER_READ_AIR);	// rilevamento aria temporizzato ogni 100 msec
			switch(m_air_manager[i])
			{
				case E_START_AIR:			// se la verifica aria è abilitata, da qui comincia l'analisi dell'eventuale aria
					if(m_error_air_flag & (0x01 << i))	// per scheda con cablaggi tipo nuovo sull'UABD ver 2013
					{
						m_TimerAllarmeAria[i].Preset(_TIMER_READ_AIR_CONTROL);	// se rilevo già aria riduco il tempo di attesa per l'ulteriore verifica
					}
					m_air_manager[i] = E_CONTROL_AIR;	// cambio stato
					break;
				case E_CONTROL_AIR:
					if(m_error_air_flag & (0x01 << i))	// c'è ancora aria, allora forse è seria
					{
						m_TimerAllarmeAria[i].Preset(_TIMER_READ_AIR_CONTROL);	// rilevamento aria di rifare subito per ferificare se è vero o lettura spuria				
						m_count_aria[i] = 0;				// resetto il contatore dei giri di aria rilevata
						m_volume_aria[i] = 0;				// resetto il totalizzatore del volume di aria compplessiva
						m_air_manager[i] = E_ALERT_AIR;			// cambio stato: sono in un episodio di aria da analizzare
					}
					break;
				case E_ALERT_AIR:
					if(m_error_air_flag & (0x01 << i))	// c'è ancora aria, allora forse è seria
					{
						m_count_aria[i]++;
						if(Via[i].abilitazione == True)
						{
							flusso_attuale = calc_flusso_pompa();	// questo calcolato era un flusso su 10msec, ritorna decimi di ml
							m_volume_aria[i] += flusso_attuale;
						}
						m_air_manager[i] = E_ACTIVE_AIR;
						m_TimerAllarmeAria[i].Preset(_TIMER_READ_AIR_ALERT);
					}
					else		// se non c'è aria allora era una bollicina iniqua o un errore di elttura
					{
						m_TimerAllarmeAria[i].Preset(_TIMER_READ_AIR_CONTROL);
						m_air_manager[i] = E_CONTROL_AIR;
					}
					break;
				case E_ACTIVE_AIR:	// qui c'è aria, ma devo integrare e settare akllarme solo per bolle sopra il ml oppure per bolle lunghe + di un secondo con pompa che va
					if(m_error_air_flag & (0x01 << i))	// c'è ancora aria,integro
					{
						m_TimerAllarmeAria[i].Preset(_TIMER_READ_AIR_ALERT);
						if(Via[i].abilitazione == True)
						{
							m_count_aria[i]++;
							flusso_attuale = calc_flusso_pompa();		// questo calcolato era un flusso su 10msec, ritorna decimi di ml
							m_volume_aria[i] += flusso_attuale;			// incremento il contatore dei decimi di ml del volume della bolla d'aria
							if((m_volume_aria[i] > _VOLUME_MAX) || (m_count_aria[i] > _TEMPO_DI_ARIA)) // la bolla supera il volume di soglia oppure è passato 3 sec di aria
							{
								StatusCmd.statusChan[_ADC2_] |= (0x0001 << i);	// setto il bit relativo all'allarme aria
								set_air_alarm(i);		// setta il flag per la tasteira, il flag per accendere la luce e l'errore di sistema, in base alle condizioni
								m_air_manager[i] = E_ALARM_AIR;
							}
							/*
							else
							{
								if(m_count_aria[i] > _TEMPO_DI_ARIA)
								{
									StatusCmd.statusChan[_ADC2_] |= (0x0001 << i);		// setto il bit relativo all'allarme aria
									set_air_alarm(i);		// setta il flag per la tasteira, il flag per accendere la luce e l'errore di sistema, in base alle condizioni
									m_air_manager[i] = E_ALARM_AIR;
								}
							}
							*/
						}
					}
					else									// sembra finita ma per sicurezza rileggo a breve distanza
					{
						m_TimerAllarmeAria[i].Preset(_TIMER_READ_AIR_ALERT);		// rilevamento aria da rifare subito dopo 5msec
						m_count_aria[i] = 0;				// resetto il contatore dei giri di aria rilevata
						m_volume_aria[i] = 0;
						m_air_manager[i] = E_ALERT_AIR;
					}
					break;
				case E_ALARM_AIR:	// in questo stato rimango finché la condizione di aria non viene risolta
					if(!(m_error_air_flag & (0x01 << i)))
					{
						m_TimerAllarmeAria[i].Preset(_TIMER_READ_AIR_CONTROL);		// rilevamento aria da rifare dopo 10 msec
						m_air_manager[i] = E_RESET_AIR;
					}
					/*
					else
					{
						if(Via[i].abilitazione == True)
						{
							StatusCmd.statusChan[_ADC2_] |= (0x0001 << i);			// setto il bit relativo all'allarme aria
						}
					}
					*/
					break;
				case E_RESET_AIR:
					if(m_error_air_flag & (0x01 << i))	// c'è ancora aria, sarà stata una lettura spuria
					{	
						m_air_manager[i] = E_ALARM_AIR;			// quindi ritorna nello stato di allarme
					}
					else									// altrimenti è proprio finito l'allarme
					{
					/*
						StatusCmd.statusChan[_ADC2_] &= ~(0x0001 << i);	// resetto il bit relativo all'allarme aria	
						Via[i].stato_luci = IDLE_LUCE;
						*/
						m_air_manager[i] = E_CONTROL_AIR;
					}
					break;
				default:
					StatusCmd.status = STATO_ERRORE;
					StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
					StatusCmd.error_monitor = 	ERR_CASE_AIR_MNGR;
					break;
			}
		}
	}
	return 0;
}

/**
* Manages of all type of error in CPU_SIFRA state machine, like block, airalarm, ...
*/
bool SIFRA_Manager::SIFRAError_Manager()
{
	//VerificaESD_Damage();	// verifica lo stato del continous conversion mode degli ADC, se si è interrotto li resetto e accendo l'allarme instabile
	
	Verifica_celle();			// ogni _TIMEOUT_CHKHW verifica l'eventuale danneggiamento delle celle

	Verifica_peso_massimo();	// verifica il sovraccarico della rastrelliera
	
	encoder_updateCnt();

	Verifica_encoder_motori();

	Verifica_apertura_sportello();
	
	SIFRAAirIn_Manager();		// verifica aria nelle linee di riempimento
		
	if(StatusCmd.flagsErrorLine != NO_ERRORS)	// se è settato un errore generico sulla linea abilitata
	{
		StatusCmd.flagsErrorLine = 0x00;
		SIFRACloseLineEV(StatusCmd.next_line);	// chiusura della ev abilitata
		setStatePump(0, ARRESTO);		// setta lo stato di arresto pompa, il pwm manager si occuperà del resto
		m_check_block_pump.Stop();

 		if(Via[StatusCmd.next_line].abilitazione == True)
 		{
			if(StatusCmd.statusChan[_ADC1_] & ERR_PESOINSTABILE)		// priorità più alta
				Via[StatusCmd.next_line].stato_luci = ERR_INSTABILE;	// così per questa vi comincia la gestione del lampeggio indipendente dalle altre
			else
			{
				if((StatusCmd.statusChan[_ADC2_] & 0x00FF) >0)		// priorità due, errore aria
				{
					Via[StatusCmd.next_line].stato_luci = ERR_ARIA;
				}
				else
					if(StatusCmd.statusChan[_ADC1_] & ERR_RILEV_VUOTO)		// priorità tre
						Via[StatusCmd.next_line].stato_luci = ERR_VIA;
					else
						if(StatusCmd.statusChan[_ADC1_] & ERR_NON_CALA)		// priorità quattro
							{
								Via[StatusCmd.next_line].stato_luci = ERR_VIA;
							}
			}
		}

		if(getSIFRAphase() != E_PHASE_PAUSE)
		{
			StatusCmd.status = STATO_ERRORE;
			changeSIFRAstatus(StatoErrore);
		}
		
		return True;
		
	}

	return False;
}

/**
* Manager della fase di stop durante START, dove si gestisce i movimenti manuali delle pompe
*/
void SIFRA_Manager::SIFRABreak_Manager()
{
	float soglia_calo_volume;

	if(MotorStatus.step_motor_done > _SOGLIA_ENC_)			// encoder rileva movimento rotore a motore fermo
	{
		soglia_calo_volume = (float)MotorStatus.step_motor_done/structEnc.param_encoder;
		if(((StartCmd.function & 0x00FF) != RIEMPITUBI) && ((StartCmd.function & 0x00FF) != RISCIACQUO))
		{
			StatusCmd.error_stop |= E_ERR_STOP_PUMP_BY_HAND;	// sono già in stop e non vado in errore. La tastiera in stop deve monitorare questo flag
		}
		m_calo_volume_stop = (word)soglia_calo_volume;
	}
}

/**
* Manager della ripresa di una formula, con controllo di conformità del peso con l'ultimo misurato prima dello stop.
*/
void SIFRA_Manager::SIFRAWeightControlRestart(char __typeOfFilling)
{
	m_variazione_peso = (short)(m_peso_stop - m_peso_attuale);

	switch(__typeOfFilling)
	{
		case E_TYPE_FORMULA:
			if(m_variazione_peso > _MIN_STOP_WEIGHT_VARIATION_)		// rilevazione di un calo peso che potrebbe essere andato in sacca
			{
				if(m_variazione_peso > _MAX_STOP_WEIGHT_DECREASE_)		// rilevazione di un calo peso eccessivo che potrebbe essere andato in sacca
				{
					StatusCmd.error_stop |= E_ERR_STOP_EXCESSIVE_DECREASE;
				}
				else
				{
					StatusCmd.error_stop |= E_ERR_STOP_WEIGHT_DECREASE;		// rilevazione di un calo peso accettabile che potrebbe essere andato in sacca
				}
			}
			else if(m_variazione_peso < -(_MIN_STOP_WEIGHT_VARIATION_))	// rilevazione di un aumento del peso da giustificare
			{
				if(m_variazione_peso < -(_STOP_WEIGHT_VARIATION_BAG_CHANGE_))
				{
					if(m_void_source)
					{
						StatusCmd.error_stop |= E_ERR_STOP_BAG_CHANGED;	// aumento peso anticipato da errore vuoto, per cui sicuramente dovuto a cambio sacca
					}
					else
					{
						StatusCmd.error_stop |= E_ERR_STOP_MAYBE_BAG_CHANGED;	// aumento peso imputabile a cambio sacca
					}
				}
				else
				{
					StatusCmd.error_stop |= E_ERR_STOP_WEIGHT_INCREASE;		// aumento peso non motivato
				}
			}
			break;
			
		case E_TYPE_SERVICE:
			if(m_variazione_peso < -(_STOP_WEIGHT_VARIATION_BAG_CHANGE_))
			{
				if(m_void_source)
				{
					StatusCmd.error_stop |= E_ERR_STOP_BAG_CHANGED;	// aumento peso anticipato da errore vuoto, per cui sicuramente dovuto a cambio sacca
				}
				else
				{
					StatusCmd.error_stop |= E_ERR_STOP_MAYBE_BAG_CHANGED;	// aumento peso imputabile a cambio sacca
				}
			}
			break;
			
		default:
			break;
	}
	
	StatusCmd.variazione_peso = m_variazione_peso;
	
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
* Imposta la non calibrazione delle celle di carico
*/
void SIFRA_Manager::SIFRASetScalesNotCalib()
{
	Chan[_ADC1_].AreCalibrate = False;
	Chan[_ADC2_].AreCalibrate = False;
	backup_calib_state(_ADC1_, (byte)Chan[_ADC1_].AreCalibrate);
	backup_calib_state(_ADC2_, (byte)Chan[_ADC2_].AreCalibrate);
}

/**
* Manager gestione chiusura o interruzione erogazione. Viene chiamata quando per errore occorso al sistema oppure per scelta dell'utilizzatore,
* viene mandato uno stop al sistema che sta erogando. Questa routine deve gestire il salvataggio dello stato attuale delle vie, per consentire la ripartenza.
*/
bool SIFRA_Manager::SIFRAStop_Manager()
{
	if(m_TimerStopAll.Match())	// se scatta questo bisogna uscire e salvare lo stato, abbiamo aspettato troppo
	{
		if(Via[StatusCmd.next_line].peso_iniziale >= m_peso_attuale)
		{
			m_peso_parziale = Via[StatusCmd.next_line].peso_iniziale - (word)m_peso_attuale;
		}
		else
		{
			m_peso_parziale = 0;
		}
		Via[StatusCmd.next_line].peso_erogato = m_peso_parziale + Via[StatusCmd.next_line].peso_gia_erogato;
		Via[StatusCmd.next_line].capacita = aggiorna_capacita(StartCmd.cap_linea[StatusCmd.next_line],  m_peso_parziale, Via[StatusCmd.next_line].peso_spec);
		SifraStopTimers();
		SifraResetStartCmd();
		SifraResetFillingData();
		SifraResetStateMachine();
		SifraResetStatus();
		m_TimerStopAll.Stop();
		return True;
	}

	return False;
}

/**
* Macchina a stati gestione calibrazione
*/
bool SIFRA_Manager::SIFRACalibr_Manager()
{
	switch(g_stateMachine.calibState)
	{
		case StartAcq:
			break;
		case ReadZeroInBothLines:	// valido per M3200 e M3300, adc1 e adc2 in una sola volta (corrispondenti alle vie 6 e H20 (8 e 15))
			if(readZeroInBothLine() == False)	// legge lo zero delle celle 1 e 2, eventualmente confronta con il valore di rif e salva
			{
				StatusCmd.statusChan[_ADC1_] |= ERR_LETTURA_ZERO;
				StatusCmd.flagsErrorLine = 0x01;
				LedDrivers_sendData(LED_VIA1 | LED_ROSSO);
				LedDrivers_sendData(LED_VIA8 | LED_ROSSO);
				g_stateMachine.calibState = StartAcq;
				Chan[_ADC1_].AreCalibrate = False;
				Chan[_ADC2_].AreCalibrate = False;
				backup_calib_state(_ADC1_, Chan[_ADC1_].AreCalibrate);
				backup_calib_state(_ADC2_, Chan[_ADC2_].AreCalibrate);
				/*
				if(!SIFRACalibResetData())
				{
					StatusCmd.flagsErrorLine = 0x01;
					StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
					StatusCmd.error_monitor = ERR_RESET_CALIB_DATA_FAIL;
				}
				*/
			}
			else
			{
				LedDrivers_sendData(LED_VIA1 | LED_VERDE);
				LedDrivers_sendData(LED_VIA8 | LED_VERDE);
				m_timer_calib.Preset(_2_SEC_);
				g_stateMachine.calibState = E_END_CALIB;
			}
			break;
		case ReadLoadCellSX:
			if(readLoadInLine8() == False)	// legge lo zero delle celle 1 e 2, eventualmente confronta con il valore di rif e salva
			{
				StatusCmd.statusChan[_ADC1_] |= ERR_LETTURA_2Kg;
				StatusCmd.flagsErrorLine = 0x01;
				LedDrivers_sendData(LED_VIA1 | LED_ROSSO);
				g_stateMachine.calibState = StartAcq;
				Chan[_ADC1_].AreCalibrate = False;
				Chan[_ADC2_].AreCalibrate = False;
				backup_calib_state(_ADC1_, Chan[_ADC1_].AreCalibrate);
				backup_calib_state(_ADC2_, Chan[_ADC2_].AreCalibrate);
				/*
				if(!SIFRACalibResetData())
				{
					StatusCmd.flagsErrorLine = 0x01;
					StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
					StatusCmd.error_monitor = ERR_RESET_CALIB_DATA_FAIL;
				}
				*/
			}
			else
			{
				LedDrivers_sendData(LED_VIA1 | LED_VERDE);
				m_timer_calib.Preset(_2_SEC_);
				g_stateMachine.calibState = E_END_CALIB;
			}
			break;
		case ReadLoadCellDX:
			if(readLoadInLine15() == False) 	// legge lo zero delle celle 1 e 2, eventualmente confronta con il valore di rif e salva
			{
				StatusCmd.statusChan[_ADC1_] |= ERR_LETTURA_2Kg;
				StatusCmd.flagsErrorLine = 0x01;
				g_stateMachine.calibState = StartAcq;
				LedDrivers_sendData(LED_VIA8 | LED_ROSSO);
				Chan[_ADC1_].AreCalibrate = False;
				Chan[_ADC2_].AreCalibrate = False;
				backup_calib_state(_ADC1_, Chan[_ADC1_].AreCalibrate);
				backup_calib_state(_ADC2_, Chan[_ADC2_].AreCalibrate);
				/*
				if(!SIFRACalibResetData())
				{
					StatusCmd.flagsErrorLine = 0x01;
					StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
					StatusCmd.error_monitor = ERR_RESET_CALIB_DATA_FAIL;
				}
				*/
			}
			else
			{
				LedDrivers_sendData(LED_VIA8 | LED_VERDE);
				m_timer_calib.Preset(_2_SEC_);
				Chan[_ADC1_].AreCalibrate = True;
				Chan[_ADC2_].AreCalibrate = True;
				backup_calib_state(_ADC1_, Chan[_ADC1_].AreCalibrate);
				backup_calib_state(_ADC2_, Chan[_ADC2_].AreCalibrate);
				m_loadsystem2 = calc_param_weight(Chan);
				g_stateMachine.calibState = E_END_CALIB;
			}
			break;
		case E_END_CALIB:
			if(m_timer_calib.Match())
			{
				m_timer_calib.Stop();
				return True;
			}
			break;
		default:
			StatusCmd.status = STATO_ERRORE;
			StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
			StatusCmd.error_monitor = 	ERR_CASE_CALIB_MNGR;
			break;
	}
	
	return False;
	
}

/**
* Manager dello stato sacca: può uscire per una situaizone di errore oppure perchè ha finito, tornando True.
* Altrimenti torna False
*/
bool SIFRA_Manager::SIFRAService_Manager()
{
	switch(g_stateMachine.serviceState)
	{
		case setup_via:
			SIFRAOpenLineEV(StatusCmd.next_line);			// apertura elettrovalvola
			g_stateMachine.serviceState = Start_pompe;
			break;
		case Start_pompe:
			if(m_timer_openEV.Match())
			{
				ResetMotorStepCounter();				// reset dei contatori encoder
				setStatePump(0, ABILITAZIONE);			// fa partire la pompa dedicata o quella unica
				m_vel = _PWM_CYCLE_MAX;		
				m_check_block_pump.Preset(_1_SEC_);
				m_rampa_timer.Preset(_t_RAMPA_SV);											
				g_stateMachine.serviceState = Controllo_pompe;	
			}
			break;			
		case Controllo_pompe:		// lettura pesi con verifica stabilità : margine 0.5g 
			// gestione rampa motori fino alla velocità massima
			if( Via[StatusCmd.next_line].abilitazione )
			{
				SIFRASpeedPump_Manager( 0);
			}					
			break;
		default:
			StatusCmd.status = STATO_ERRORE;
			StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
			StatusCmd.error_monitor = 	ERR_CASE_SERVICE_MNGR;
			break;
	}
	
	return False;
}

/**
* Manager del test EMC: può uscire solo per una situazione di errore oppure per uno stop dell'operatore.
* Torna sempre False
*/
bool SIFRA_Manager::SIFRATestEMC_Manager()
{
	switch(g_stateMachine.serviceState)
	{
		case setup_via:
			SIFRAOpenLineEV(StatusCmd.next_line);			// apertura elettrovalvola
			g_stateMachine.serviceState = Start_pompe;
			break;
		case Start_pompe:
			if(m_timer_openEV.Match())
			{
				setStatePump(0, ABILITAZIONE);			// fa partire la pompa
				m_vel = _PWM_CYCLE_HALF;
				m_rampa_timer.Preset(_t_RAMPA_SV);
				m_verify_enc_timer.Preset(_1_SEC_);
				m_first_enc_control = True;
				m_init_weight = m_weight_real_time;
				m_init_load = m_peso_attuale;
				Set_weight_threshold();
				g_stateMachine.serviceState = Controllo_pompe;	
			}
			break;			
		case Controllo_pompe:
			// gestione rampa motori fino alla velocità massima
			/*
			if( Via[StatusCmd.next_line].abilitazione )
			{
			*/
				SIFRASpeedPump_Manager( 0);
			//}					
			break;
		default:
			StatusCmd.status = STATO_ERRORE;
			StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
			StatusCmd.error_monitor = 	ERR_CASE_EMC_MNGR;
			break;
	}
	
	return False;
}

/**
* Routine di prova velocità pompe per calcolare il flusso peso e il rapporto giri motore flusso pompa
*/
bool SIFRA_Manager::SIFRATest_Pompe(int m_line)
{
	int m_vel;
	long peso_erog = 0;
	dword m_deltatime;
	
	switch(g_stateMachine.serviceState)
	{
		case setup_via:
			if(m_timer_flowPump.Match())
			{
				g_stateMachine.serviceState = Start_pompe;
				m_timer_flowPump.Stop();
				Via[0].peso_erogato = 0;
				Via[1].peso_erogato = 0;
				Via[2].peso_erogato = 0;
				Via[3].peso_erogato = 0;
				Via[4].peso_erogato = 0;
				m_time_total = 0;
				m_startime = 0;
			}
			break;
		case Start_pompe:
			if(m_dato_valido)	// solo quando c'è un dato valido di adc su cui laborare
			{
				m_dato_valido = False;
				m_peso1 = m_peso_attuale;
				m_vel = StartCmd.peso_linea[m_line];
				setta_led_pannello(m_line, ON);
				ResetMotorStepCounter();// reset dei contatori encoder
				setBlockRelay(0, 1);	//chiudo il relay
				enable_MOT(0);
				setPwmMotCycle(0, m_vel);
				m_timerEndSeq.Preset(10000);	// 10 secondi di test
				m_timer_enc_measure.Preset(2500);
				m_startime = globalTimer.getMsec();	//m_startime = globalTimer.getTime();
				g_stateMachine.serviceState = Controllo_pompe;			
			}
			break;
			
		case Controllo_pompe:
			// gestione rampa motori fino alla velocità massima
			if(m_dato_valido)	// solo quando c'è un dato valido di adc su cui laborare
			{
				m_dato_valido = False;
				if(m_peso1 >= m_peso_attuale)
				{
					peso_erog = m_peso1 - m_peso_attuale;
				}
				else
				{
					peso_erog = 0;
				}
				if(peso_erog > 0)
				{
					Via[0].peso_erogato = (word)(peso_erog);			// peso erogato fino a qui in grammi
					m_deltatime = globalTimer.getMsec() - m_startime;	// millisecondi passati dallo start (in decimale)
					Via[1].peso_erogato = (word)((peso_erog*60000)/m_deltatime);		// peso al minuto g/min attuale
					Via[2].peso_erogato = (word)((float)MotorStatus.step_motor_done/(float)peso_erog);		// rapporto flusso/pompa (impulsi enc/g)
					Via[3].peso_erogato = (word)(MotorStatus.step_motor_done * 120 * 10 / m_deltatime);		// giri/min della pompa
				}
/*** ricorda che c'è un diviso 10 nel supervisore, quindi faccio un *10)  ***/
				// giri assoluti = (impulsi assoluti / 500)
				// giri al minuto = giri assoluti / min = (giri assoluti / msec)*(msec/min)
			}
			if(m_timer_enc_measure.Match())
			{
				m_timer_enc_measure.Preset(1000);
				Via[4].peso_erogato = (word)(MotorStatus.step_motor_done);	// impulsi encoder
			}
			if(m_timerEndSeq.Match())
			{
				m_timer_enc_measure.Stop();
				setBlockRelay(0, 0);		//chiudo il relay
				disable_PWM_MOT(0);
				setPwmMotCycle(0, 0);
				m_check_block_pump.Stop();
				m_timerEndSeq.Preset(3000);	// tempo di assestamento
				m_time_total = ((float)(globalTimer.getMsec() - m_startime)) / _MILLE_;	// secondi passati dallo start (in decimale)
				g_stateMachine.serviceState = calcoli;
			}
			break;
		case calcoli:
			if(m_timerEndSeq.Match())
			{
				Via[0].peso_erogato = (word)(peso_erog);			// peso erogato fino a qui in decimi di grammo
				Via[1].peso_erogato = 10 * (word)(peso_erog * 6);	// flusso dg/min attuale
				Via[2].peso_erogato = 10 * (word)(MotorStatus.step_motor_done / peso_erog); // rapporto flusso/pompa (impulsi enc/decimi di g) poi moltiplicato per 10
				Via[3].peso_erogato = 10 * (word)(MotorStatus.step_motor_done * 3 / 500);	// giri motore/min poi moltiplicato per 10
				Via[4].peso_erogato = (word)MotorStatus.step_motor_done;
				setta_led_pannello(m_line, OFF);
				SIFRACloseLineEV(m_line);
				m_timerEndSeq.Stop();
				return True;			
			}	
			if(m_dato_valido)	// solo quando c'è un dato valido di adc su cui laborare
			{
				m_dato_valido = False;
				if(m_peso1 >= m_peso_attuale)
				{
					peso_erog = m_peso1 - m_peso_attuale;
				}
				else
				{
					peso_erog = 0;
				}
				Via[0].peso_erogato = (word)(peso_erog);			// peso erogato fino a qui in decimi di grammo
				Via[1].peso_erogato = 10 * (word)(peso_erog * 6);	// flusso dg/min attuale
				Via[2].peso_erogato = 10 * (word)(MotorStatus.step_motor_done / peso_erog); // rapporto flusso/pompa (impulsi enc/decimi di g) poi moltiplicato per 10
				Via[3].peso_erogato = 10 * (word)(MotorStatus.step_motor_done * 3 / 500);	// giri motore/min poi moltiplicato per 10
				Via[4].peso_erogato = (word)MotorStatus.step_motor_done;
			}
			break;
		default:
			StatusCmd.status = STATO_ERRORE;
			StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
			StatusCmd.error_monitor = 	ERR_CASE_POMPE_MNGR;
			break;
	}
	return False;
}

/**
* Routine di prova per ricavare flusso pompa a diverse velocità
*/
bool SIFRA_Manager::SIFRATest_FlussoPompa(int m_line)
{
	int m_vel;
	
	switch(g_stateMachine.serviceState)
	{
		case E_SERVICE_NONE:
			break;
		case setup_via:
			ResetMotorStepCounter();
			m_num_sample_mgr = 0;
			g_stateMachine.serviceState = Start_pompe;
			m_timer_flowPump.Preset(_20_SEC_);
			m_vel = StartCmd.peso_linea[m_line];
			setBlockRelay(0, 1);
			enable_MOT(0);
			setPwmMotCycle(0, m_vel);
			break;
		case Start_pompe:
			if(m_timer_flowPump.Match())
			{
				setBlockRelay(0, 0);
				disable_PWM_MOT(0);
				setPwmMotCycle(0, 0);
				SIFRACloseLineEV(m_line);
				m_timer_flowPump.Preset(_5_SEC_);
				g_stateMachine.serviceState = calcoli;			
			}
			break;
		case Controllo_pompe:
			break;
		case calcoli:
			if(m_timer_flowPump.Match())
			{
				setta_led_pannello(m_line, OFF);
				m_timer_flowPump.Stop();
				return True;			
			}
			break;
		default:
			StatusCmd.status = STATO_ERRORE;
			StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
			StatusCmd.error_monitor = 	ERR_CASE_TEST_FLOW_PUMP;
			break;
	}
	
	return False;
}

/**
* Routine di prova per ricavare flusso pompa a diverse velocità
*/
bool SIFRA_Manager::SIFRATest_OpenMoreEVs()
{	
	byte line;

	switch(m_test_open_more_ev)
	{
		case E_SERVICE_EV_START:
			for(line=0;line<NUM_MAX_LINE;line++)
			{
				if(Via[line].abilitazione)
				{
					OpenLineEV(line);
				}
			}
			m_timer_moreEVopened.Preset(_2_SEC_);
			m_test_open_more_ev = E_SERVICE_EV_OPEN;
			break;
		case E_SERVICE_EV_OPEN:
			if(m_timer_moreEVopened.Match())
			{
				m_timer_moreEVopened.Stop();
				m_test_open_more_ev = E_SERVICE_EV_STOP;
			}
			break;
		case E_SERVICE_EV_STOP:
			for(line=0;line<NUM_MAX_LINE;line++)
			{
				CloseLineEV(line);
			}
			return True;
			break;
		default:
			StatusCmd.status = STATO_ERRORE;
			StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
			StatusCmd.error_monitor = 	ERR_CASE_TEST_OPEN_MORE_EV;
			break;
	}
	
	return False;
}

/**
* Manager of panel leds,
* Control of lights red and green of every line; lights can blink, stay off or stay on.
*
* Per il corretto controllo dei led della scheda luci all'avvio è necessario impostare i parametri di DECODE_MODE, INTENSITY, SCAN_LIMIT e NORMAL_OP.
* Vedere datasheet MAX7219 per dettagli interfacciamento con scheda luci.
*/
void SIFRA_Manager::SIFRALed_Manager()
{
	byte m_line;
	word data = 0x0000;

	if(m_resetLedDrivers.Match())	// il timer per il reset pannello luci è scattato, devo fare la routine di reset
	{
		m_resetLedDrivers.Stop();
		if(g_stateMachine.driverState != INIT_DRIVER)
		{
			g_stateMachine.driverState = INIT_DRIVER;
			g_stateMachine.ledState = DISPLAY_RE_INIT_1;
		}
	}
	
 	switch(g_stateMachine.driverState)
 	{
 		case INIT_DRIVER:
	 		if(m_timerLed.Match())
			{
				switch(g_stateMachine.ledState)
				{
					case DISPLAY_INIT:
						LedDrivers_sendData(DECODE_MODE);
						LedDrivers_sendData(INTENSITY);
						LedDrivers_sendData(SCANLIMIT);
						LedDrivers_sendData(NORMAL_OP);
						m_timerLed.Preset(_TIMER_LED_FAST_);
						g_stateMachine.ledState = DISPLAY_READY;				
						break;
					case DISPLAY_READY:
						m_timerLed.Preset(_TIMER_LED_FAST_);
						switchoff_leds();	// spegni tutte le luci
						m_state_led = ON;
						g_stateMachine.ledState = BLINK_LEDS;
						break;
					case BLINK_LEDS:
						m_timerLed.Preset(_TIMER_LED_SLOW_);
						if(m_state_led)
						{
							m_state_led = OFF;
							switchon_red_leds();	// accende tutti i led rossi e solo i rossi
						}
						else
						{
							m_state_led = ON;
							switchon_green_leds();	// accende tutti i led verdi e solo i verdi
						}
						break;
					case BLINK_LEDS_HW_ERROR:	// lampeggio continuo dei leds per segnalare errore bloccante
						m_timerLed.Preset(_TIMER_LED_SLOW_);
						if(m_state_led)
						{
							m_state_led = OFF;
							switchon_leds();		// accende tutti i led 
						}
						else
						{
							m_state_led = ON;
							switchoff_leds();		// spegne tutti i led
						}
						break;
					case DISPLAY_RE_INIT_1:	// periodicamente reinizializza il driver luci
						m_timerLed.Preset(_TIMER_LED_MEDIUM_);
						LedDrivers_sendData(NORMAL_OP);	// inizializzazione				
						g_stateMachine.ledState = DISPLAY_RE_INIT_2;
						break;
					case DISPLAY_RE_INIT_2:	// periodicamente reinizializza il driver luci
						m_timerLed.Preset(_TIMER_LED_FAST_);
						LedDrivers_sendData(DECODE_MODE);
						LedDrivers_sendData(INTENSITY);
						LedDrivers_sendData(SCANLIMIT);
						LedDrivers_sendData(NORMAL_OP);
						switchoff_leds();	// spegni tutte le luci
						g_stateMachine.ledState = DISPLAY_IDLE;
						g_stateMachine.driverState = CONTROLL_DRIVER;
						break;
					case DISPLAY_IDLE:
						m_timerLed.Preset(_TIMER_LED_FAST_);
						break;
					default:
						m_timerLed.Preset(_TIMER_LED_FAST_);
						break;
				}
	 		}
			break;
		case CONTROLL_DRIVER:	// se invece sono nella fase di normale controllo luci...
			for(m_line = 0; m_line < NUM_MAX_LINE; m_line++)
			{
				if(Via[m_line].timer_led.Match())		// timer relativo a questa via
				{
					if(Via[m_line].stato_led == OFF)	// qui il led è spento, via disabilitata
					{
						Via[m_line].timer_led.Preset(_t_BLINK_STOP);
						if(Via[m_line].stato_luci == SETTA_LUCE)	// via abilitata, luce verde accesa, poi va in IDLE
						{
							data = ((m_line + 1) << 8) | LED_VERDE;						
							reset_state_this_led(m_line, ON);	// setta in idle lo stato di tutti i led 
							Via[m_line].stato_luci = IDLE_LUCE;
						}
						else
						{
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
						}
						LedDrivers_sendData(data);
					}
					else
					{
						switch(Via[m_line].stato_luci)
						{
							case IDLE_LUCE:
							case BLINKA_LUCE:
							case SETTA_LUCE:
								Via[m_line].timer_led.Preset(_t_BLINK_START);
								data = ((m_line + 1) << 8) | LED_VERDE;
								break;
							case SPEGNI_LUCE:									// spegne la/le luci e va in idle
								Via[m_line].timer_led.Preset(_t_BLINK_START);
								data = ((m_line + 1) << 8) | TURN_OFF;
								reset_state_this_led(m_line, OFF);					// setta in idle lo stato di tutti i led
								break;
							case ERR_CELLA:										// errore di cella rotta, rosso fisso e verde blink con T = 800msec
								Via[m_line].timer_led.Preset(_t_BLINK_AIR_ERROR);
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
								Via[m_line].timer_led.Preset(_t_BLINK_AIR_ERROR);
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
								Via[m_line].timer_led.Preset(_t_BLINK_OTHER_ERR);
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
								Via[m_line].timer_led.Preset(_t_BLINK_START);
								break;
						}
						LedDrivers_sendData(data);	// valido per tutti gli stati
					}			
				}
			}
			break;
		case E_WAIT_DRIVER:
			break;
		default:
			StatusCmd.status = STATO_ERRORE;
			StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
			StatusCmd.error_monitor = 	ERR_CASE_LED_MNGR;
			break;
	}
}

/**
* Decoding of parameters recieved in last START MESSAGE from M3000
*/
byte SIFRA_Manager::Decode_StartMsg()
{
	byte i;
	word cmd = 0x0000;
	byte m_line;
	byte data;

	cmd = StartCmd.function & 0x00FF;			// maschera di bit
	switch(cmd)
	{
		case E_RESET_SISTEMA:
			if(m_calo_volume_stop > 0)
			{
				Via[bk.nextLine].capacita = Via[bk.nextLine].capacita - m_calo_volume_stop;
			}
			DefaultSystemReset();				// inizializzazione del sistema allo stato di avvio
			SifraResetBackup();
			//m_timer_control_restart.Stop();
			Setta_filtro_peso(E_FILT_LOW);
			structEnc.restart= 0x55;
			if(!Write_Param_Backup())
			{
				//StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
				//StatusCmd.flagsErrorLine = 0x01;
				StatusCmd.error_monitor = ERR_RAM_WRITE_BK;
				//break;
			}
			if(!Write_Param_Encoder())
			{
				//StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
				//StatusCmd.flagsErrorLine = 0x01;
				StatusCmd.error_monitor = ERR_RAM_WRITE_ENC;
				break;
			}
			changeSIFRAstatus(AttesaComando);
			break;
			
		case INIT_SISTEMA:// start con solo abort reset sistema e spegnimento luci
			SifraResetViaStatus();				// nuova sequenza, reset di tutte le vie
			SifraResetStatus();
			SifraResetStateMachine();
			SifraResetFillingData();
			ResetMotorStepCounter();			// resetta lo stato degli encoder, quindi azzera anche gli errori di pompa girata a mano
			SifraStopTimers();
			m_timerLed.Stop();
			g_stateMachine.driverState = CONTROLL_DRIVER;
			Setta_filtro_peso(E_FILT_LOW);
			#ifdef __DEBUG_ADC
				acquisition = True;
			#endif
			if(structEnc.restart == 0xAA)		// c'è un backup per mancanza rete
			{
				if(DownLoad_StatoStart())		// leggo i dati backappati	e li impiongo al sistema
				{
					StatusCmd.status = STATO_STOP;
					changeSIFRAstatus(RipresaMancanzaRete);
				}
			}
			else
			{
				StatusCmd.status = STATO_IDLE;
				changeSIFRAstatus(AttesaComando);
			}
			break;
			
		case NUOVA_SEQUENZA:	// start nuova sequenza
			SifraResetViaStatus();
			Setta_filtro_peso(E_FILT_LOW);
			StatusCmd.status = STATO_START;
			StatusCmd.tot_enabled_lines = bk.num_vie_da_eseguire = Find_Num_of_Enabled_Lines(StartCmd.support);// legge il numero di vie da eseguire												
			if(StatusCmd.tot_enabled_lines > 0)									// se ce'è almeno una via
			{
				for (i = 0; i < NUM_MAX_LINE; i++)
				{
					Via[i].peso_da_erogare = StartCmd.peso_linea[i];					// target comandato da tastiera
					Via[i].peso_spec = AdattaPesoSpecifico(StartCmd.peso_spec[i]);		// da intero compreso tra  990 e 1500 a decimale compreso tra 0,900 e 1,500
					Via[i].capacita = StartCmd.cap_linea[i];
				}
				g_stateMachine.driverState = CONTROLL_DRIVER;
				g_stateMachine.saccaState = ConfigurazioneSequenza;
				BackUpTimer.Preset(_TIME_x_BACKUP);
				structEnc.restart = 0xAA;
				if(!Write_Param_Encoder())
				{
					//StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
					//StatusCmd.flagsErrorLine = 0x01;
					StatusCmd.error_monitor = ERR_RAM_WRITE_ENC;
					//break;
				}
			}
			else
			{
				m_timerEndSeq.Preset(_WAIT_M300_READING);				// esco subito ma devo dare il tempo al padrone di leggere il mio stato start	
				g_stateMachine.saccaState = FineSequenza;
			}
			changeSIFRAstatus(StatoSacca);									// stato gestione sequenza riempimento
			break;
			
		case RESTART_SEQUENZA:	// START da pausa
			StatusCmd.status = STATO_START;
			changeSIFRAstatus(StatoSacca);									// stato gestione sequenza riempimento	
			StatusCmd.tot_enabled_lines = bk.num_vie_da_eseguire;
			StatusCmd.statusChan[_ADC1_] &= ERR_POMPA_MOSSA_A_MANO_NEG;
			if(StatusCmd.tot_enabled_lines > 0)							// se c'è un backup attivo e vie abilitate, altrimenti per ora fine sequenza
			{
				Setta_filtro_peso(E_FILT_LOW);
				for (i = 0; i < NUM_MAX_LINE; i++)
				{
					Via[i].peso_da_erogare = StartCmd.peso_linea[i];					// target comandato da tastiera
					Via[i].peso_spec = AdattaPesoSpecifico(StartCmd.peso_spec[i]);		// da intero da 990 a 1500 a decimale 0,900 to 1,500
					Via[i].peso_gia_erogato = bk.peso_erogato[i];
					Via[i].abilitazione = False;
				}
				g_stateMachine.driverState = CONTROLL_DRIVER;
				g_stateMachine.saccaState = RipresaSequenza;
				BackUpTimer.Preset(_TIME_x_BACKUP);
			}
			else	
			{
				StatusCmd.tot_enabled_lines = 0;									// decremento le vie da eseguire
 				m_timerEndSeq.Preset(_WAIT_M300_READING);				// esco subito ma devo dare il tempo al padrone di leggere il mio stato start	
				g_stateMachine.saccaState = FineSequenza;	
			}
			break;
			
		case TEST_PINZE:
			SifraResetViaStatus();
			StatusCmd.statusChan[_ADC1_] = 0;
			StatusCmd.status = STATO_START;
			StatusCmd.next_line = 0;
			Via[StatusCmd.next_line].abilitazione = True;
			changeSIFRAstatus(StatoTestPinze);	
			g_stateMachine.pinchState = E_PINCH_TEST_CONFIG;
			break;

		case MANAGER_LUCI_CALIBR:	// calibrazione SOLO GESTIONE LUCI
			g_stateMachine.driverState = CONTROLL_DRIVER;					// i led rispondono al tipo di comando
			StatusCmd.status = STATO_CALIBRAZIONE_UTENTE;
			if(StartCmd.support == 0x00)
			{
				for(i = 0; i < NUM_MAX_LINE; i++)	// scorrendo sulle possibili vie, qui va bene NUM_MAX_LINE
				{
					Via[i].stato_led = OFF;
					Via[i].fase_luci = 0;
					Via[i].stato_luci = SPEGNI_LUCE;
				}
			}
			else
			{
				for(i = 0; i < NUM_MAX_LINE; i++)	// scorrendo sulle possibili vie, qui va bene NUM_MAX_LINE
				{
					if(StartCmd.support & (0x01 << i)) 	// se questa via è abilitata
					{
						setta_led_pannello(i, ON);
					}
				}
			}
			break;
			
		case DEFAULT_CALIBR:		// calibr. incondizionata
			m_timerLed.Stop();
			g_stateMachine.driverState = E_WAIT_DRIVER;
			LedDrivers_clearFifo();
			switchoff_leds();
			StatusCmd.status = STATO_CALIBRAZIONE_FABBRICA;
			Setta_filtro_peso(E_FILT_LOW);
			break;

		case E_USR_CALIB_CMD:
			m_timerLed.Stop();
			g_stateMachine.driverState = E_WAIT_DRIVER;
			LedDrivers_clearFifo();
			switchoff_leds();
			StatusCmd.status = STATO_CALIBRAZIONE_UTENTE;
			Setta_filtro_peso(E_FILT_LOW);
			break;

		case RIEMPITUBI:		// MANUALE RIEMPITUBI 						// allarme aria disabilitato
			SifraResetViaStatus();
			//m_timer_control_restart.Stop();
			m_void_source = False;
			StatusCmd.error_stop = E_ERR_STOP_NONE;
			g_stateMachine.driverState = CONTROLL_DRIVER;					// i led rispondono al tipo di comando
			StatusCmd.status = STATO_START;
			StatusCmd.next_line= Find_Enable_line(StartCmd.support);			
			if (StatusCmd.next_line> -1)										// una via è abilitata
			{
				Setta_filtro_peso(E_FILT_LOW);
				m_peso_stop = m_peso_attuale;
				g_stateMachine.rtState = SetupRiempitubi;					// stato interno allo stato del riempitubi
			}
			else
			{
				m_timerEndSeq.Preset(_WAIT_M300_READING);			// altrimenti va subito in fine sequenza
				g_stateMachine.rtState = ChiusuraRiempitubi;
			}
			changeSIFRAstatus(StatoRiempitubi);
			break;

		case SVUOTATUBI:	// SVUOTATUBI - UNA VIA ALLA VOLTA
			SifraResetViaStatus();
			StatusCmd.next_line = Find_Enable_line(StartCmd.support);
			if(StatusCmd.next_line > -1)	// una via è abilitata
			{
				changeSIFRAstatus(StatoSvuotatubi);
				g_stateMachine.driverState = CONTROLL_DRIVER;				// i led rispondono al tipo di comando										
				g_stateMachine.stState = StartManuale;				
			}
			else
			{
				changeSIFRAstatus(AttesaComando);
			}
			break;

		case RISCIACQUO:	// pulizia tubi con acqua di via 15, allarme aria attivo per evitare di svuotare i tubi
		case RISCIACQUO_DA_SUPE:
			SifraResetViaStatus();
			Setta_filtro_peso(E_FILT_LOW);
			g_stateMachine.driverState = CONTROLL_DRIVER;					// i led rispondono al tipo di comando
			StatusCmd.status = STATO_START;
			changeSIFRAstatus(StatoRisciacquo);
			StatusCmd.next_line = Find_Enable_line(StartCmd.support);			
			if (StatusCmd.next_line > -1)										// risciacquo solo su via 14 o 15, la via di spinta, ma ci pensa la tastiera a controllare
			{
				g_stateMachine.rtState = SetupRiempitubi;					// primo stato della macchina di gestione del manuale
				bk.nextLine = StatusCmd.next_line;
			}
			else
			{
				m_timerEndSeq.Preset(_WAIT_M300_READING);			// altrimenti va subito in fine sequenza
				g_stateMachine.rtState = ChiusuraRiempitubi;
			}
			break;
			
		case MANUALE_CON_OBIETTIVO:	// MANUALE // allarme aria disabilitato // VALIDO ANCHE IN RIEMPITUBI
		case 0x2B:	// manuale + abort + start + riempitubi		
			StatusCmd.next_line = Find_Enable_line(StartCmd.support);
			if (StatusCmd.next_line == bk.nextLine)	// una via è abilitata
			{
				StatusCmd.status = STATO_START;
				g_stateMachine.driverState = CONTROLL_DRIVER;// i led rispondono al tipo di comando
				g_stateMachine.stState = StartManuale;		// primo stato della macchina di gestione del manuale
				BackUpTimer.Preset(_TIME_x_BACKUP);
				changeSIFRAstatus(StatoManuale);
			}
			else
			{
				StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
				StatusCmd.status = STATO_ERRORE;
				StatusCmd.error_monitor = ERR_RAM_WRITE_ENC;
			}
			break;

		case E_VERIFY_CALIB_VAL_CMD:
			for(m_line=0;m_line<_MAX_LOAD_CHAN_;m_line++)
			{
				EE_random_byte_read((ADDRESS_CHAN_ARE_MEM_CALIBRATE + m_line*BASESTRUCTADDR), &data);
				m_control_cal_values[m_line].typeOfOffsetCal = data;
				EE_random_byte_read((ADDRESS_CHAN_ARE_MEM_CALIBRATE + 1 + m_line*BASESTRUCTADDR), &data);
				m_control_cal_values[m_line].typeOfGainCal = data;
				m_control_cal_values[m_line].WeightFactoryGain = EE_read_float(ADDRESS_FACTORY_GAIN + m_line*BASESTRUCTADDR);
				m_control_cal_values[m_line].WeightFactoryOffset = EE_read_word(ADDRESS_FACTORY_OFFSET+ m_line*BASESTRUCTADDR);
				m_control_cal_values[m_line].Weightgain = EE_read_float(ADDRESS_GAIN + m_line*BASESTRUCTADDR);
				m_control_cal_values[m_line].Weightoffset = EE_read_word(ADDRESS_OFFSET + m_line*BASESTRUCTADDR);
				m_control_cal_values[m_line].AdcTo2Kg = EE_read_int(ADDRESS_2Kg_READ + m_line*BASESTRUCTADDR);
				m_control_cal_values[m_line].AdcTo2Kg_dx = EE_read_int(ADDRESS_2KgDx_READ + m_line*BASESTRUCTADDR);
			}
			asm("nop");
			break;
		
		case DEBUG_VEDI_ADC_VIE:	// TARA	// funzione di debug: inviando il comando TARA il peso inviato al M3000 non è in grammi ma in ADC (filtrato)
			// rispedendo lo stesso comando, si ritorna allo stato normale (invio dei pesi in grammi)
			if(m_SIFRAProtocol->m_show_adcvalue == True)
			{
				m_SIFRAProtocol->m_show_adcvalue = False;
			}
			else
			{
				m_SIFRAProtocol->m_show_adcvalue = True;
			}
			break;
		case DEBUG_LUCI:
			m_timerLed.Stop();
			g_stateMachine.driverState = E_WAIT_DRIVER;
			m_test_led_timer.Preset(_1_SEC_);
			StatusCmd.status = STATO_START;
			m_test_led = E_LED_START;
			changeSIFRAstatus(StatoTestLuci);
			break;
		case DEBUG_ELETTROVALVOLE:	// avvio test ciclico elettrovalvle
			m_timerServiceEv.Preset(_TIMER_READ_AIR);
			m_debug_EV = 0;
			m_testEVstate = E_EV_TEST_OPEN;
			changeSIFRAstatus(StatoServiceEV);
			break;
		case DEBUG_SERVICE:	// abort+start+manuale = SERVICE
			g_stateMachine.driverState = CONTROLL_DRIVER;	// i led rispondono al tipo di comando
			StatusCmd.next_line = Find_Enable_line(StartCmd.support);
			if (StatusCmd.next_line > -1)	// una via è abilitata
			{
				Via[StatusCmd.next_line].abilitazione = True;			
				setta_led_pannello(StatusCmd.next_line, ON);
				StatusCmd.m_FillingOnCourse = False;		// si attiva il controllo motori in caso di errore: i motori sono spenti
				StatusCmd.air_block_en = False;			// disattivazione blocco x aria
				StatusCmd.status = STATO_START;
				g_stateMachine.serviceState = setup_via;
				changeSIFRAstatus(StatoService);
			}
			else		// altrimento setto lo stato di errore
			{
				changeSIFRAstatus(AttesaComando);
			}
			break;
		case TEST_EMC:
			for(i = 0; i < NUM_MAX_LINE; i++)
			{	
				m_TimerAllarmeAria[i].Stop();		//disabilito i timer di controllo allarme aria
			}
			m_check_block_pump.Stop();
			g_stateMachine.driverState = CONTROLL_DRIVER;	// i led rispondono al tipo di comando
			StatusCmd.next_line = Find_Enable_line(StartCmd.support);
			if (StatusCmd.next_line > -1)	// una via è abilitata
			{
				//Via[StatusCmd.next_line].abilitazione = True;			
				setta_led_pannello(StatusCmd.next_line, ON);
				StatusCmd.m_FillingOnCourse = False;		// si attiva il controllo motori in caso di errore: i motori sono spenti
				StatusCmd.air_block_en = False;			// disattivazione blocco x aria
				StatusCmd.status = STATO_START;
				g_stateMachine.serviceState = setup_via;
				m_enc_value = cpld_counter1 & _MASK_16BIT_;
				m_enc_value_control = cpld_counter1 & _MASK_16BIT_;
				changeSIFRAstatus(StatoTestEMC);
			}
			else		// altrimenti setto lo stato di errore
			{
				changeSIFRAstatus(AttesaComando);
			}
			break;
		case DEBUG_VEL_POMPE:
			StatusCmd.next_line = Find_Enable_line(StartCmd.support);
			if (StatusCmd.next_line > -1)	// una via è abilitata
			{
				StatusCmd.status = STATO_START;
				m_SIFRAProtocol->m_show_velpompe = True;
				SIFRAOpenLineEV(StatusCmd.next_line);				
				g_stateMachine.serviceState = setup_via;
				m_timer_flowPump.Preset(_TIMER_LED_MEDIUM_);
				changeSIFRAstatus(StatoTestPompe);
			}
			break;
		case E_TEST_PKTS_DEBUG:
			StatusCmd.status = STATO_ERRORE;
			StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
			StatusCmd.error_monitor = ERR_CASE_TEST_DEBUG;
			StatusCmd.next_line = 1;
			break;
		case E_TEST_OPEN_MORE_EV:
			SifraResetViaStatus();
			StatusCmd.tot_enabled_lines = Find_Num_of_Enabled_Lines(StartCmd.support);
			if(StatusCmd.tot_enabled_lines > 0)
			{
				m_test_open_more_ev = E_SERVICE_EV_START;
				changeSIFRAstatus(StatoTestEvAperte);
			}
			break;
		case E_TEST_FLOW_PUMP:
			StatusCmd.next_line = Find_Enable_line(StartCmd.support);
			if (StatusCmd.next_line > -1)	// una via è abilitata
			{
				StatusCmd.status = STATO_START;
				SIFRAOpenLineEV(StatusCmd.next_line);				
				g_stateMachine.serviceState = setup_via;
				changeSIFRAstatus(StatoTestFlussoPompa);
			}
			break;
		case RESET_ADC_ON_BOARD:	// reset manuale della scheda, con recupero dello stato al riavvio
			reset_4_block = 0x55;
			SIFRAMsg_resetApplicationHandler();
			break;
		default:
			StatusCmd.status = STATO_ERRORE;
			StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
			StatusCmd.error_monitor = ERR_START_MESSAGE;
			break;
	}
	return 1;
}

/**
* Manager of the pinch test.
* If test doesn't pass, the module stops and rinsing must be done.
* Velocità pompa costante 25%, per 8000impulsi, mezzo giro pompa --> 1,8ml di volume teorici
*/
bool SIFRA_Manager::SIFRATestPinch()
{
	byte i;
	long delta1;
		
	switch (g_stateMachine.pinchState)
	{	
		case E_PINCH_TEST_IDLE:
			break;
		case E_PINCH_TEST_CONFIG:	
			for (i = 0;i < NUM_MAX_LINE; i++)					// Controllo la chiusura di tutte le valvole
			{
				SIFRACloseLineEV(i);
			}
			m_enc_value = cpld_counter1 & _MASK_16BIT_;		// copio il valore attuale del contatore
			g_stateMachine.pinchState = E_PINCH_TEST_ABIL_PUMP_CW;
			break;

		case E_PINCH_TEST_ABIL_PUMP_CW:
			if(m_dato_valido)									// solo quando c'è un dato valido di adc su cui laborare
			{
				m_dato_valido = False;
				m_peso1 = m_peso_attuale;							// salvo il peso attuale come peso inziale
				ResetMotorStepCounter();						// resetto stato contatori encoder
				setStatePump(0, ABILITAZIONE);					// Setta lo stato di partenza pompa, deve fare mezzo giro
				g_stateMachine.pinchState = E_PINCH_TEST_SET_SPEED_CW;
			}
			break;

		case E_PINCH_TEST_SET_SPEED_CW:
			setPwmMotCycle(0, VELOCITA_TEST_EV);			// setto la velocità desiderata
			m_check_block_pump.Preset(_1_SEC_);
			g_stateMachine.pinchState = E_PINCH_TEST_MOVE_CW;
			break;

		case E_PINCH_TEST_MOVE_CW:	// attendo qui che abbia compiuto mezzo giro, cioè 8000 impulsi d'encoder, cioè 
			if(MotorStatus.step_motor_done >= 8000)		// mezzo giro di pompa -> 1,8mL teorici
			{
				setStatePump(0, ARRESTO);						// Setta lo stato di arresto pompa, il pwm manager si occuperà del resto
				m_check_block_pump.Stop();
				m_timerPinchTest.Preset(TIME_PINCH_WEIGHT_STABLE);					// Attendo 3 secondi per stabilizzare la lettura	
				g_stateMachine.pinchState = E_PINCH_TEST_FIRST_CONTROL;
			}
			break;

		case E_PINCH_TEST_FIRST_CONTROL:	// qui attendo la stabilizzazione, poi leggo il peso attaule
			if (m_timerPinchTest.Match())
			{
				if(m_dato_valido)	// solo quando c'è un dato valido di adc su cui lavorare
				{
					m_enc_value = cpld_counter1 & _MASK_16BIT_;
					m_dato_valido = False;
					delta1 = m_peso1 - m_peso_attuale;				// Calcolo la differenza
					if (delta1 < SOGLIA_PARZ_TEST_PINZE)
					{
						g_stateMachine.pinchState = E_PINCH_TEST_ABIL_PUMP_CCW;		// procedo con la movimentazione in verso antiorario
					}
					else
					{
						g_stateMachine.pinchState = E_PINCH_TEST_ERR_MOV_CW;	// se il controllo è KO faccio 2° test
					}
				}
			}
			break;
	
		case E_PINCH_TEST_ERR_MOV_CW:	// qui devo fare un altro mezzo giro, perchè s'è letto qualcosa di anaomalo
			if(m_dato_valido)	// solo quando c'è un dato valido di adc su cui lavorare
			{
				m_dato_valido = False;	
				MotorStatus.step_motor_expected = MotorStatus.step_motor_done;
				m_giri_pinch_test = MotorStatus.step_motor_done;	// memorizzo il numero di passi dell'encoder fatti per eseguire il primo mezzo giro in avanti della pompa
				m_test_pinch_alarm = True;		// memorizzo il fatto che c'è stato un errore e quindi il test pinze farà due mezzi giri in avanti
				setStatePump(0, ABILITAZIONE);				// Setta lo stato di avanti, il pwm manager si occuperà del resto
				g_stateMachine.pinchState = E_PINCH_TEST_SET_SECOND_CW;
			}
			break;

		case E_PINCH_TEST_SET_SECOND_CW:	// imposto la vel desiderata
			setPwmMotCycle(0, VELOCITA_TEST_EV);
			m_check_block_pump.Preset(_1_SEC_);
			g_stateMachine.pinchState = E_PINCH_TEST_SECOND_MOVE_CW;
			break;

		case E_PINCH_TEST_SECOND_MOVE_CW:	// attendo che il contatore abbia contato gli impulsi desiderati oppure il tempo massimo
			if(MotorStatus.step_motor_done >= (8000 + MotorStatus.step_motor_expected))		// mezzo giro di pompa -> 1,8mL teorici
			{
				setStatePump(0, ARRESTO);			// Setta lo stato di arresto pompa, il pwm manager si occuperà del resto
				m_check_block_pump.Stop();
				m_timerPinchTest.Preset(TIME_PINCH_WEIGHT_STABLE);			// Attendo 6 secondi per stabilizzare la lettura	
				g_stateMachine.pinchState = E_PINCH_TEST_SECOND_CONTROL;
			}
			break;

		case E_PINCH_TEST_SECOND_CONTROL:
			if (m_timerPinchTest.Match())
			{
				if(m_dato_valido)	// solo quando c'è un dato valido di adc su cui laborare
				{
					m_dato_valido = False;
					delta1 = m_peso1 - m_peso_attuale;
					if (delta1 < SOGLIA_TEST_PINZE)
					{
						g_stateMachine.pinchState = E_PINCH_TEST_ABIL_PUMP_CCW;	// Variazione di peso all'interno della tolleranza proseguo con il test
					}
					else
					{	// Variazione di peso fuori tolleranza esco dal test e mostro allarme
						StatusCmd.flagsErrorLine |= 0x01;	// setto errore su entrambe le vie
						StatusCmd.statusChan[_ADC1_] = ERR_TESTPINZE_KO;
						g_stateMachine.pinchState = E_PINCH_TEST_IDLE;
						m_openEV_pich_fail = True;
						m_test_pinch_alarm = False;
						Via[StatusCmd.next_line].abilitazione = False;
						StatusCmd.next_line = -1;
						return False;
					}					
				}
			}
			break;
	
		case E_PINCH_TEST_ABIL_PUMP_CCW:	// devo ritornare indietro del numero di passi compiuti
			if(m_test_pinch_alarm)
			{
				m_test_pinch_alarm = False;
				MotorStatus.step_motor_expected = m_giri_pinch_test + CONST_TEST_PINZE * (MotorStatus.step_motor_done - m_giri_pinch_test);
			}
			else
			{
				MotorStatus.step_motor_expected = CONST_TEST_PINZE * (MotorStatus.step_motor_done); // ritorno di 2/3 rispetto a quanto avanzato, così non si svuota il tubo nella piovra
			}
			// controllo_errore_flusso = MotorStatus.step_motor_done - MotorStatus.step_motor_expected;  // calcolo i giri encoder di differenza tra l'andata ed il ritorno della pompa
			MotorStatus.step_motor_done = 0;
			setStatePump(0, INDIETRO);			// Setta lo stato di abilitazione pompa, il pwm manager si occuperà del resto
			m_timerPinchTest.Preset(TIME_PINCH_CONTROL);			// watchdog sul motore, se non si ferma con encoder
			g_stateMachine.pinchState = E_PINCH_TEST_SET_SPEED_CCW;
			break;

		case E_PINCH_TEST_SET_SPEED_CCW:
			setPwmMotCycle(0, VELOCITA_TEST_EV);			
			g_stateMachine.pinchState = E_PINCH_TEST_MOVE_CCW;
			break;

		case E_PINCH_TEST_MOVE_CCW:	// ho finito, ho esguito il ritorno per azzerare la pressione negativa, ora posso uscire
			if( (MotorStatus.step_motor_done >= MotorStatus.step_motor_expected) || (m_timerPinchTest.Match()))		// mezzo giro di pompa -> 1,8mL teorici
			{
				setStatePump(0, ARRESTO);			// Setta lo stato di arresto pompa, il pwm manager si occuperà del resto
				g_stateMachine.pinchState = E_PINCH_TEST_IDLE;
				return True;
			}
			break;
		default:
			StatusCmd.status = STATO_ERRORE;
			StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
			StatusCmd.error_monitor = 	ERR_CASE_PINCH_TEST_MNGR;
			break;
	}
	
	return False;
	
}

int SIFRA_Manager::SIFRAMsg_setStopAllHandler()
{
	m_SIFRAProtocol->SIFRA_set_Stop_All();
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

int SIFRA_Manager::SIFRAMsg_debugViaAskedHandler()
{
	m_SIFRAProtocol->sendSIFRAViaDebug();
	return 1;
}

bool SIFRA_Manager::SIFRAMsg_debugStartAskedHandler()
{
	m_SIFRAProtocol->sendSIFRAStartDebug();
	return True;
}

bool SIFRA_Manager::SIFRAMsg_debugStatusAskedHandler()
{
	m_SIFRAProtocol->sendSIFRAStatusDebug();
	return True;
}

bool SIFRA_Manager::SIFRAMsg_debugRamAskedHandler()
{
	m_SIFRAProtocol->sendSIFRARamDebug();
	return True;
}

int SIFRA_Manager::SIFRAMsg_startAcquisitionHandler()
{
	m_startLoadingSamples = True;
	return 1;
}

/**
Reset application and restar system automatically
@return always 1
*/
int SIFRA_Manager::SIFRAMsg_resetApplicationHandler()
{
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	
	restart_command = 0xA5;	// salvo in ram tamponata il comando di reset ricevuto da tastiera
	
	fpp = (void (*)()) __APPLICATION_START_ADDRESS__;	// inizio dell'applicone
    	(*fpp)();
		
	return 1;
}

int SIFRA_Manager::SIFRAMsg_hwErrorHandler()
{
	StatusCmd.SIFRAComHwErrors++;
	return 1;
}

int SIFRA_Manager::SIFRAMsg_protocolErrorHandler()
{
	StatusCmd.SIFRAComProtocolErrors++;
	return 1;
}

int SIFRA_Manager::SIFRAMsg_unknownErrorHandler()
{
	StatusCmd.SIFRAComUnknownErrors++;
	return 1;
}

/**
* In M3300 module this function enables CLAMP ELECTROMAGNET
*/
void SIFRA_Manager::SIFRAOpenLineEV(int m_line)
{
	OpenLineEV(m_line);

	if(m_openEV_pich_fail)
	{
		m_timer_openEV.Preset(_T_OPEN_EV_DELAY);
		m_openEV_pich_fail = False;
	}
	else
	{
		if(g_stateMachine.nonCalaState > no_allarme_non_cala)
		{
			m_timer_openEV.Preset(_T_OPEN_EV_DELAY);
			g_stateMachine.nonCalaState = no_allarme_non_cala;
		}
		else
		{
			m_timer_openEV.Preset(_T_OPEN_EV);
		}
	}
}

/**
* Chiude l'elettrovalvola sulla linea passata
*/
void SIFRA_Manager::SIFRACloseLineEV(int m_line)
{
	CloseLineEV(m_line);
}

void SIFRA_Manager::SifraResetSystemParams()
{
	SIFRA_resetStatusError();
	g_stateMachine.nonCalaState = no_allarme_non_cala;
	SifraResetStatus();
	SifraResetStartCmd();
	SifraResetFillingData();
	SifraResetServiceData();
	SifraResetViaStatus();
	SifraResetStateMachine();
	ResetMotorStepCounter();
	SifraStopTimers();
}

/**
* Resetto le strutture Via del sistema. Resetto tutto escluso il timerLed di ogni via.
*/
void SIFRA_Manager::SifraResetViaStatus()
{
	byte i;
	
	for(i = 0; i < NUM_MAX_LINE; i++)
	{
		Via[i].abilitazione = False;
		Via[i].da_eseguire = False;
		Via[i].eseguita = False;

		Via[i].stato_led = OFF;
		Via[i].fase_luci = 0;
		Via[i].stato_luci = IDLE_LUCE;
		
		Via[i].peso_da_erogare = 0;
		Via[i].peso_erogato = 0;
		Via[i].peso_gia_erogato = 0;
		Via[i].peso_spec = 0;
		Via[i].peso_iniziale = 0;
		Via[i].peso_finale = 0;
	}
}

/**
* Resetto lo stato del sistema e spengo i led della scheda luci.
*/
void SIFRA_Manager::SifraResetStatus()
{
	LedDrivers_clearFifo();
	switchoff_leds();						// spengo i led

	// resetto i dati di riempimento associati alla struttura StatusCmd
	StatusCmd.m_FillingOnCourse = False;
	StatusCmd.air_block_en = False;		// disattivazione blocco x aria
	StatusCmd.tot_enabled_lines = 0;
	StatusCmd.next_line = -1;

}

/**
* Resetto la struttura StartCmd del sistema
*/
void SIFRA_Manager::SifraResetStartCmd()
{
	byte i;

	StartCmd.function = 0;
	StartCmd.support = 0;

	for(i = 0; i <NUM_MAX_LINE;i++)
	{ 
		StartCmd.peso_linea[i] = 0;
		StartCmd.peso_spec[i] = 0;
		StartCmd.cap_linea[i] = 0;
	}

}

/**
* Resetto le variabili che gestiscono le varie macchine a stati presenti nel sistema
*/
void SIFRA_Manager::SifraResetStateMachine()
{
	byte i;
	
	for(i = 0; i < NUM_MAX_LINE;i++)
	{
		m_air_manager[i] = E_START_AIR;			//variabile usata per controllo allarmi aria
	}

	setStatePump(0, POMPA_DISABILITA);		//variabile usata per controllo stato pompa

	g_stateMachine.sampleState = init_stab;					//variabile usata nell'algoritmo del peso instabile
	g_stateMachine.calibState = StartAcq;					// variabile usata per gestire la macchina a stati in calibrazione
	g_stateMachine.saccaState = SaccaIdle;					// variabile usata per gestire la macchina a stati in sacca
	g_stateMachine.rtState = IdleRiempitubi;			// variabile usata per gestire la macchina a stati in svuotamento linee, riempimento linee e riempimento manuale
	g_stateMachine.stState = IdleManuale;
	g_stateMachine.pinchState = E_PINCH_TEST_IDLE;			// variabile usata per gestire test pinze e funziona di debug delle elettrovalvole
	g_stateMachine.ledState = DISPLAY_INIT;			// variabile usata per gestire i diversi lampeggii dei led
	g_stateMachine.serviceState = E_SERVICE_NONE;			// variabile usata per test di service, emc e pompe
	m_stato_check_blocco = verifica_contatore;	// variabile usata per controllo blocco lettura ADC
	m_testEVstate = E_EV_TEST_IDLE;
}

/**
* Resetto le variabili associate ad un riempimento automatico
*/
void SIFRA_Manager::SifraResetFillingData()
{
	m_enc_value = 0;				// variabile utilizzata in Test EMC
	m_restart = False;				// variabile usata in SaccaManager per stabilire se eseguire il controllo sull'errore prodotto
	m_RT_con_calib = False;		// variabile usata in RT_Manager per stabilire se occorre eseguire il calcolo del flusso peso
	m_controllo_errore_volume = no_errore_volume;		// variabile usata per distinguere eventuale errore di prodotto da errore di flusso
	m_controllo_volume_encoder = False;				// variabile usata per controllare se devo fare il controllo volume quando arrivo nello stato StatoControlloEncoder
	m_test_pinch_alarm = False;						// usata per identificare un doppio giro in avanti della pompa durante il test pinze
	m_giri_pinch_test = 0;							// usata in test pinze per salvare in caso di doppio mezzo giro in avanti della pompa i giri fatti nel primo mezzo giro
	m_peso_non_cala = 0;							// variabile usata in algoritmo non cala
	m_peso_non_cala_precedente = 0;				// variabile usata in algoritmo non cala
	m_volMaxST = 0;								// variabile usata in svuotamento linee
}

/**
* Resetto le variabili associate alle fasi di service
*/
void SIFRA_Manager::SifraResetServiceData()
{
	m_first_enc_control = False;

	m_timer_enc_measure.Stop();
	m_verify_enc_timer.Stop();
}

/**
* Resetto la struttura dati di backup
*/
void SIFRA_Manager::SifraResetBackup()
{
	byte i;

	bk.vie_da_eseguire = bk.vie_eseguite =  0x0000;
	bk.nextLine = -1;
	bk.num_vie_da_eseguire = 0;

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
	m_timerServiceEv.Stop();
	m_timerEndSeq.Stop();
	m_timerVerificaCalo.Stop();
	BackUpTimer.Stop();
	m_restore_stability.Stop();
}

/**
* It reads adc values in adc1 and adc2 when cells are not charged. With these values, offset will be calculated for these ADC, then saved in eeprom.
* It works in two way. In CALIBRAZIONE DI FABBRICA, it reads, calculates and backups if ADC values are in the range of 5% respect previous values saved
* In CALIBRAZIONE UTENTE calculated value is matched with factory value: in case of high difference (greater than 3 %), it sets ERR_LETTURA_ZERO.
* Valid for M3300 modules: two ADCs, two offsets.
*/
bool SIFRA_Manager::readZeroInBothLine()
{
	int diff1;
	int diff2;
	
	if(StatusCmd.status == STATO_CALIBRAZIONE_FABBRICA)
	{
		if((m_WeightSample[_ADC1_] > _DELTA_ADC_1_PERC_FS_) && (m_WeightSample[_ADC1_] > _DELTA_ADC_1_PERC_FS_)) // ha misurato qualcosa di diverso da zero, come soglia mettiamo l'1% del fondo scala della cella di carico
		{
			if(Chan[_ADC1_].AreCalibrate && Chan[_ADC2_].AreCalibrate)
			{
				if(((int)m_WeightSample[_ADC1_] > ((int)Chan[_ADC1_].WeightFactoryOffset - _DELTA_ADC_0Kg_DEF)) &&
						((int)m_WeightSample[_ADC1_] < ((int)Chan[_ADC1_].WeightFactoryOffset + _DELTA_ADC_0Kg_DEF)))
				{
					if(((int)m_WeightSample[_ADC2_] > ((int)Chan[_ADC2_].WeightFactoryOffset - _DELTA_ADC_0Kg_DEF)) &&
						((int)m_WeightSample[_ADC2_] < ((int)Chan[_ADC2_].WeightFactoryOffset + _DELTA_ADC_0Kg_DEF)))
					{
						weightChan->setOffset(_ADC1_, m_WeightSample[_ADC1_]);
						weightChan->setOffset(_ADC2_, m_WeightSample[_ADC2_]);
						// salvo i valori ADC di offset e offset factory per entrambi i canali
						Chan[_ADC1_].WeightFactoryOffset = Chan[_ADC1_].Weightoffset = (word)m_WeightSample[_ADC1_];			
						Chan[_ADC2_].WeightFactoryOffset = Chan[_ADC2_].Weightoffset = (word)m_WeightSample[_ADC2_];
						backup_factory_offset_value(_ADC1_, (word)m_WeightSample[_ADC1_]);
						backup_factory_offset_value(_ADC2_, (word)m_WeightSample[_ADC2_]);
						return True;
					}
					else
					{
						StatusCmd.error_monitor = 	ERR_CASE_FACTORY_CALIB_OFFSET_OUT_CELL_15;
					}
				}
				else
				{
					StatusCmd.error_monitor = ERR_CASE_FACTORY_CALIB_OFFSET_OUT_CELL_8;
				}
			}
			else
			{
				weightChan->setOffset(_ADC1_, m_WeightSample[_ADC1_]);
				weightChan->setOffset(_ADC2_, m_WeightSample[_ADC2_]);
				// salvo i valori ADC di offset e offset factory per entrambi i canali
				Chan[_ADC1_].WeightFactoryOffset = Chan[_ADC1_].Weightoffset = (word)m_WeightSample[_ADC1_];			
				Chan[_ADC2_].WeightFactoryOffset = Chan[_ADC2_].Weightoffset = (word)m_WeightSample[_ADC2_];
				backup_factory_offset_value(_ADC1_, (word)m_WeightSample[_ADC1_]);
				backup_factory_offset_value(_ADC2_, (word)m_WeightSample[_ADC2_]);
				return True;
			}
		}
		else
		{
			StatusCmd.error_monitor = ERR_CASE_FACTORY_CALIB_OFFSET_LOW;
		}
	}
	else
	{	
		diff1 = (int)m_WeightSample[_ADC1_] - (int)Chan[_ADC1_].Weightoffset;
		diff2 = (int)m_WeightSample[_ADC2_] - (int)Chan[_ADC2_].Weightoffset;
		if((diff1 > (-_DELTA_ADC_0Kg)) && (diff1 < _DELTA_ADC_0Kg))	// dentro la soglia
		{
			if((diff2 > (-_DELTA_ADC_0Kg)) && (diff2 < _DELTA_ADC_0Kg))	// dentro la soglia
			{
				weightChan->setOffset(_ADC1_, m_WeightSample[_ADC1_]);
				weightChan->setOffset(_ADC2_, m_WeightSample[_ADC2_]);
				// salvo i valori ADC di offset per entrambi i canali
				Chan[_ADC1_].Weightoffset = (word)m_WeightSample[_ADC1_];
				Chan[_ADC2_].Weightoffset = (word)m_WeightSample[_ADC2_];
				backup_new_offset_value(_ADC1_, Chan[_ADC1_].Weightoffset);
				backup_new_offset_value(_ADC2_, Chan[_ADC2_].Weightoffset);
				return True;
			}
			else
			{
				StatusCmd.error_monitor = ERR_CASE_CALIB_OFFSET_OUT_CELL_15;
			}
		}
		else
		{
			StatusCmd.error_monitor = ERR_CASE_CALIB_OFFSET_OUT_CELL_8;
		}
	}
	
	return False;
	
}

/**
* It saves the values corrisponding to 2kg sample weight put on line 8.
* If it is a CALIBRAZIONE DI FABBRICA, reads value on ADC1 and ADC2 and controls if they match with saved values (range of 5 %)
* If it is a CALIBRAZIONE UTENTE, reads value on ADC1 and ADC2 and controls if they match with saved values (range of 3 %)
*/
bool SIFRA_Manager::readLoadInLine8()
{

	int adc_gain_cell_8 = 0;
	int adc_gain_cell_15 = 0;
	//float this_gain_8;
	//float this_gain_15;
	float cal_adc_system;
	
#ifdef APPLY_DEBUG
	dword  d_adc_gain_cell_8 =0;
	dword  d_adc_gain_cell_15 = 0;

	if (m_WeightSample[_ADC1_] >= weightChan->getOffset(_ADC1_)){
			d_adc_gain_cell_8 = m_WeightSample[_ADC1_] - weightChan->getOffset(_ADC1_);	
			adc_gain_cell_8 = (int)d_adc_gain_cell_8 ;
	} else {
			/*d_adc_gain_cell_8 = weightChan->getOffset(_ADC1_) - m_WeightSample[_ADC1_] ;	
			adc_gain_cell_8 = (int)d_adc_gain_cell_8 ;	*/
			return False;
	}


	if (m_WeightSample[_ADC2_] >= weightChan->getOffset(_ADC2_)){
			d_adc_gain_cell_15 = m_WeightSample[_ADC2_] - weightChan->getOffset(_ADC2_);	
			adc_gain_cell_15= (int)d_adc_gain_cell_15 ;
	} else {
			/*d_adc_gain_cell_15 = weightChan->getOffset(_ADC2_) - m_WeightSample[_ADC2_] ;	
			adc_gain_cell_15 = (int)d_adc_gain_cell_15 ;	*/
			return False;
	}
	34rt

#else

	adc_gain_cell_8 = (int)m_WeightSample[_ADC1_] - (int)weightChan->getOffset(_ADC1_);
	adc_gain_cell_15 = (int)m_WeightSample[_ADC2_] - (int)weightChan->getOffset(_ADC2_);
	//this_gain_8 = (2000.0 /(float)((int)m_WeightSample[_ADC1_] - (int)weightChan->getOffset(_ADC1_)));
	//this_gain_15 = (2000.0 /(float)((int)m_WeightSample[_ADC2_] - (int)weightChan->getOffset(_ADC2_)));
#endif

	if(StatusCmd.status == STATO_CALIBRAZIONE_FABBRICA)
	{
		if(Chan[_ADC1_].AreCalibrate && Chan[_ADC2_].AreCalibrate)
		{
			if((adc_gain_cell_8 > ((int)Chan[_ADC1_].AdcTo2Kg - _DELTA_ADC_0Kg_DEF)) &&
					(adc_gain_cell_8 < ((int)Chan[_ADC1_].AdcTo2Kg + _DELTA_ADC_0Kg_DEF)))
			{
				if((adc_gain_cell_15 > ((int)Chan[_ADC2_].AdcTo2Kg - _DELTA_ADC_0Kg_DEF)) &&
					(adc_gain_cell_15 < ((int)Chan[_ADC2_].AdcTo2Kg + _DELTA_ADC_0Kg_DEF)))
				{
					// salvo i valori ADC di guadagno con 2 Kg su cella sinistra per entrambi i canali
					Chan[_ADC1_].AdcTo2Kg = adc_gain_cell_8;
					Chan[_ADC2_].AdcTo2Kg = adc_gain_cell_15;
					return True;
				}
				else
				{
					StatusCmd.error_monitor = ERR_CASE_FACTORY_CALIB_GAIN_SX_OUT_CELL_15;
				}
			}
			else
			{
				StatusCmd.error_monitor = ERR_CASE_FACTORY_CALIB_GAIN_SX_OUT_CELL_8;
			}
		}
		else
		{
			// salvo i valori ADC di guadagno con 2 Kg su cella sinistra per entrambi i canali
			Chan[_ADC1_].AdcTo2Kg = adc_gain_cell_8;
			Chan[_ADC2_].AdcTo2Kg = adc_gain_cell_15;
			return True;
		}
	}
	else
	{
		if((adc_gain_cell_8 > ((int)Chan[_ADC1_].AdcTo2Kg - _DELTA_ADC_0Kg)) &&
					(adc_gain_cell_8 < ((int)Chan[_ADC1_].AdcTo2Kg + _DELTA_ADC_0Kg)))
		{
			if((adc_gain_cell_15 > ((int)Chan[_ADC2_].AdcTo2Kg - _DELTA_ADC_0Kg)) &&
				(adc_gain_cell_15 < ((int)Chan[_ADC2_].AdcTo2Kg + _DELTA_ADC_0Kg)))
			{
				// salvo i valori ADC di guadagno con 2 Kg su cella sinistra per entrambi i canali
				Chan[_ADC1_].AdcTo2Kg = adc_gain_cell_8;
				Chan[_ADC2_].AdcTo2Kg = adc_gain_cell_15;
				return True;	
			}
			else
			{
				StatusCmd.error_monitor = ERR_CASE_CALIB_GAIN_SX_OUT_CELL_15;
			}
		}
		else
		{
			StatusCmd.error_monitor = ERR_CASE_CALIB_GAIN_SX_OUT_CELL_8;
		}
	}
	
	return False;
	
}

/**
* It saves the values corrisponding to 2kg sample weight put on line 8.
* If it is a CALIBRAZIONE DI FABBRICA, reads value on ADC1 and ADC2 and controls if they match with saved values (range of 5 %)
* If it is a CALIBRAZIONE UTENTE, reads value on ADC1 and ADC2 and controls if they match with saved values (range of 3 %)
*/
bool SIFRA_Manager::readLoadInLine15()
{
	int adc_gain_cell_8 = 0;
	int adc_gain_cell_15 = 0;
	//float this_gain_8;
	//float this_gain_15;
	float cal_adc_system;
	
#ifdef APPLY_DEBUG
	dword  d_adc_gain_cell_8 =0;
	dword  d_adc_gain_cell_15 = 0;

	if (m_WeightSample[_ADC1_] >= weightChan->getOffset(_ADC1_)){
			d_adc_gain_cell_8 = m_WeightSample[_ADC1_] - weightChan->getOffset(_ADC1_);	
			adc_gain_cell_8 = (int)d_adc_gain_cell_8 ;
	} else {
			/*d_adc_gain_cell_8 = weightChan->getOffset(_ADC1_) - m_WeightSample[_ADC1_] ;	
			adc_gain_cell_8 = (int)d_adc_gain_cell_8 ;		*/
			return False;
	}


	if (m_WeightSample[_ADC2_] >= weightChan->getOffset(_ADC2_)){
			d_adc_gain_cell_15 = m_WeightSample[_ADC2_] - weightChan->getOffset(_ADC2_);	
			adc_gain_cell_15= (int)d_adc_gain_cell_15 ;
	} else {
			/*d_adc_gain_cell_15 = weightChan->getOffset(_ADC2_) - m_WeightSample[_ADC2_] ;	
			adc_gain_cell_15 = (int)d_adc_gain_cell_15 ;*/
			return False;
	}
	#else

		adc_gain_cell_8 = (int)m_WeightSample[_ADC1_] - (int)weightChan->getOffset(_ADC1_);
		adc_gain_cell_15 = (int)m_WeightSample[_ADC2_] - (int)weightChan->getOffset(_ADC2_);
	#endif
	
	//this_gain_8 = (2000.0 /(float)((int)m_WeightSample[_ADC1_] - (int)weightChan->getOffset(_ADC1_)));
	//this_gain_15 = (2000.0 /(float)((int)m_WeightSample[_ADC2_] - (int)weightChan->getOffset(_ADC2_)));

	if(StatusCmd.status == STATO_CALIBRAZIONE_FABBRICA)
	{
		if(Chan[_ADC1_].AreCalibrate && Chan[_ADC2_].AreCalibrate)
		{
			if((adc_gain_cell_8 > ((int)Chan[_ADC1_].AdcTo2Kg_dx - _DELTA_ADC_0Kg_DEF)) &&
					(adc_gain_cell_8 < ((int)Chan[_ADC1_].AdcTo2Kg_dx + _DELTA_ADC_0Kg_DEF)))
			{
				if((adc_gain_cell_15 > ((int)Chan[_ADC2_].AdcTo2Kg_dx - _DELTA_ADC_0Kg_DEF)) &&
					(adc_gain_cell_15 < ((int)Chan[_ADC2_].AdcTo2Kg_dx + _DELTA_ADC_0Kg_DEF)))
				{
					// salvo i valori ADC di guadagno con 2 Kg su cella destra per entrambi i canali
					Chan[_ADC1_].AdcTo2Kg_dx = adc_gain_cell_8;
					Chan[_ADC2_].AdcTo2Kg_dx = adc_gain_cell_15;
					
					// calcolo un guadagno comune ad entrambi i canali. Tale valore incide sul calcolo di m_loadsystem2 che a sua volta incide sul valore di peso rilevato
					// questo calcolo è stato mantenuto dal firmware originale per non modificare il metodo di rilevazione del peso sulla rastrelliera
					if((adc_gain_cell_15 -Chan[_ADC2_].AdcTo2Kg) != 0)
					{
						cal_adc_system = (float)((Chan[_ADC1_].AdcTo2Kg - adc_gain_cell_8)/(float)(adc_gain_cell_15 -Chan[_ADC2_].AdcTo2Kg)); 
						Chan[_ADC2_].WeightFactoryGain = Chan[_ADC1_].WeightFactoryGain = cal_adc_system;
						Chan[_ADC2_].Weightgain = Chan[_ADC1_].Weightgain = cal_adc_system;
						weightChan->setGain(_ADC1_, cal_adc_system);
						weightChan->setGain(_ADC2_, cal_adc_system);
						
						backup_factory_gain_param(_ADC1_, Chan[_ADC1_].WeightFactoryGain, Chan[_ADC1_].AdcTo2Kg, Chan[_ADC1_].AdcTo2Kg_dx);
						backup_factory_gain_param(_ADC2_, Chan[_ADC2_].WeightFactoryGain, Chan[_ADC2_].AdcTo2Kg, Chan[_ADC2_].AdcTo2Kg_dx);
						return True;
					}
					else
					{
						StatusCmd.error_monitor = ERR_CASE_FACTORY_CALIB_DIVIDE_BY_ZERO;
					}
				}
				else
				{
					StatusCmd.error_monitor = ERR_CASE_FACTORY_CALIB_GAIN_DX_OUT_CELL_15;
				}
			}
			else
			{
				StatusCmd.error_monitor = ERR_CASE_FACTORY_CALIB_GAIN_DX_OUT_CELL_8;
			}
		}
		else
		{
			// salvo i valori ADC di guadagno con 2 Kg su cella destra per entrambi i canali
			Chan[_ADC1_].AdcTo2Kg_dx = adc_gain_cell_8;
			Chan[_ADC2_].AdcTo2Kg_dx = adc_gain_cell_15;
			
			// calcolo un guadagno comune ad entrambi i canali. Tale valore incide sul calcolo di m_loadsystem2 che a sua volta incide sul valore di peso rilevato
			// questo calcolo è stato mantenuto dal firmware originale per non modificare il metodo di rilevazione del peso sulla rastrelliera
			cal_adc_system = (float)((Chan[_ADC1_].AdcTo2Kg - adc_gain_cell_8)/(float)(adc_gain_cell_15 -Chan[_ADC2_].AdcTo2Kg)); 
			Chan[_ADC2_].WeightFactoryGain = Chan[_ADC1_].WeightFactoryGain = cal_adc_system;
			Chan[_ADC2_].Weightgain = Chan[_ADC1_].Weightgain = cal_adc_system;
			weightChan->setGain(_ADC1_, cal_adc_system);
			weightChan->setGain(_ADC2_, cal_adc_system);
			
			backup_factory_gain_param(_ADC1_, Chan[_ADC1_].WeightFactoryGain, Chan[_ADC1_].AdcTo2Kg, Chan[_ADC1_].AdcTo2Kg_dx);
			backup_factory_gain_param(_ADC2_, Chan[_ADC2_].WeightFactoryGain, Chan[_ADC2_].AdcTo2Kg, Chan[_ADC2_].AdcTo2Kg_dx);
			return True;
		}
	}
	else
	{
		if((adc_gain_cell_8 > ((int)Chan[_ADC1_].AdcTo2Kg_dx - _DELTA_ADC_0Kg)) &&
					(adc_gain_cell_8 < ((int)Chan[_ADC1_].AdcTo2Kg_dx + _DELTA_ADC_0Kg)))
		{
			if((adc_gain_cell_15 > ((int)Chan[_ADC2_].AdcTo2Kg_dx - _DELTA_ADC_0Kg)) &&
				(adc_gain_cell_15 < ((int)Chan[_ADC2_].AdcTo2Kg_dx + _DELTA_ADC_0Kg)))
			{
				// salvo i valori ADC di guadagno con 2 Kg su cella destra per entrambi i canali
				Chan[_ADC1_].AdcTo2Kg_dx = adc_gain_cell_8;
				Chan[_ADC2_].AdcTo2Kg_dx = adc_gain_cell_15;

				// calcolo un guadagno comune ad entrambi i canali. Tale valore incide sul calcolo di m_loadsystem2 che a sua volta incide sul valore di peso rilevato
				// questo calcolo è stato mantenuto dal firmware originale per non modificare il metodo di rilevazione del peso sulla rastrelliera
				cal_adc_system = (float)((Chan[_ADC1_].AdcTo2Kg - adc_gain_cell_8)/(float)(adc_gain_cell_15 -Chan[_ADC2_].AdcTo2Kg)); 
				Chan[_ADC2_].Weightgain = Chan[_ADC1_].Weightgain = cal_adc_system;
				weightChan->setGain(_ADC1_, cal_adc_system);
				weightChan->setGain(_ADC2_, cal_adc_system);
				
				backup_new_gain_value(_ADC1_, Chan[_ADC1_].Weightgain);
				backup_new_gain_value(_ADC2_, Chan[_ADC2_].Weightgain);
				return True;	
			}
			else
			{
				StatusCmd.error_monitor = ERR_CASE_CALIB_GAIN_DX_OUT_CELL_15;
			}
		}
		else
		{
			StatusCmd.error_monitor = ERR_CASE_CALIB_GAIN_DX_OUT_CELL_8;
		}
	}
	
	return False;

}

/**
* Sets board's hardware version, reading two bits of cpld
* Legge i due bit di configurazione HW per capire su che sistema è montata la scheda
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
			default:
				m_SIFRAProtocol->setLocalNODEID(UNKNOWN);
				return False;
				break;
		}
		return True;
	}

	return False;
}

/**
* Set a error occoursed in eeprom reading
*/
void SIFRA_Manager::setErrorEEpromReading()
{
	//StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
	//StatusCmd.flagsErrorLine = 0x01;
	StatusCmd.error_monitor = ERR_READ_EEPROM;
}

/**
 esegue tutte le impostazioni idonee per lo start pompa
 */
void SIFRA_Manager::Set_START_Erogazione(int i)
{
	setta_led_pannello(i, ON);
	Via[i].abilitazione = True;				// abilitazione via
}

/**
* Esegue il backup continuo dello stato in start salvando i dati nella ram tamponata, che può essere scritta senza limiti di massima
* 3600scritture/h x 6h/d x 6d/w x 45w/h x 10h = 
*/
void SIFRA_Manager::Backup_StatoStart()
{
	byte i;
	
	for(i = 0; i < NUM_MAX_LINE; i++)
	{
		if(Via[i].da_eseguire == True)
		{
			bk.vie_da_eseguire |= (0x0001 << i);			// salvo le vie previste da eseguire
			if(Via[i].eseguita == True)
			{
				bk.vie_eseguite |= (0x0001 << i);			// salvo le vie già eseguite
			}
		}
		bk.abilitazione[i] = Via[i].abilitazione;		// salvo l'abilitazione della via
		bk.peso_erogato[i] = Via[i].peso_erogato;			// peso erogato
		bk.capacita[i] = Via[i].capacita;
	}
	bk.peso_stop = m_peso_stop;
	
	if(!Write_Param_Backup())
	{
		//StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
		//StatusCmd.flagsErrorLine = 0x01;
		StatusCmd.error_monitor = ERR_RAM_WRITE_BK;
	}

}

/** 
* In caso di ripresa dopo mancanza rete, scarica il backup dello stato pre-stop che abbinato ai dati salvati dalla tastiera permettono di rirpendere.
*/
bool SIFRA_Manager::DownLoad_StatoStart()
{
	byte i;
	byte enable_line = 0;
	bool line_done = True;
	int line_next = -1;
	
	if(!Read_Param_Backup())
	{
		//StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
		//StatusCmd.flagsErrorLine = 0x01;
		StatusCmd.error_monitor = ERR_RAM_READ_BK;
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
		else
		{
			if((bk.abilitazione[i]) && (line_done))
			{
				line_done = False;
				line_next = i;
			}
		}
		Via[i].abilitazione = bk.abilitazione[i];
		Via[i].peso_erogato = bk.peso_erogato[i];
		Via[i].capacita = bk.capacita[i];
	}
	m_peso_stop = bk.peso_stop;

	//if((bk.num_vie_da_eseguire != enable_line) || (bk.nextLine != line_next))
	if(bk.nextLine != line_next)
	{
		StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
		StatusCmd.status = STATO_ERRORE;
		StatusCmd.error_monitor = ERR_RESTORE_DATA;
		return False;
	}

	StatusCmd.status = STATO_STOP;

	return True;
	
}

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
* Salva in tamp ram i valori user dei parametri flusso peso e restart.
* Se la scrittura non va, esce false, il dato attuale sarà usabile solo in questa sezione e al riavvio sucessivo saranno caricati i valori nominali.
*/
bool SIFRA_Manager::Write_Param_Encoder()
{
	bool writeOk;
	bool chkOk;

	writeOk = nvr_writeParam(&structEnc, sizeof(FP_STRUCT), OFFSET_VAL_ENC, &chkOk);
	if( writeOk == False )          						// se non ce la fa
	{
		writeOk = nvr_writeParam(&structEnc, sizeof(FP_STRUCT), OFFSET_VAL_ENC, &chkOk);
		if( writeOk == False )          					// se non ce la fa
		{
			return False;
		}
	}
	return True;
}

/**
legge dalla tamp ram i valori memorizzatti dei rapporti flusso peso per i due tipi di linea
verifica il flag in tamp ram: se settato allora legge e scarica i dati salvati tornando True.
Se trova il flag non settato oppure la lettura fallisce 2 volte (problemi di lettura o checksum incoerente) torna False
per fare caricare i dati di default
*/
bool SIFRA_Manager::Read_Param_Encoder()
{
	bool readOk;
	bool chkOk;

	readOk = nvr_readParam(&structEnc, sizeof(FP_STRUCT), OFFSET_VAL_ENC, &chkOk);
	if( readOk == False )               // se ho fallito in lettura, ci riprovo almeno un'altra volta
	{
		readOk = nvr_readParam(&structEnc, sizeof(FP_STRUCT), OFFSET_VAL_ENC, &chkOk);
	   	if( readOk == False )               // se non ci sono calibrazioni valide per questo canale
	   	{
			return False;
	   	}
	}
	return True;
}

/**
* Return estimated load on all the lines of M3300.
*/
float SIFRA_Manager::GetSystemLoad(dword* samples)
{
	float load1;
	float load2;
	float loadsystem1;
	float loadsystem;
	
// solo per sviluppo e debug
	if(m_SIFRAProtocol->m_show_adcvalue == True)	// per debug ADC: 
	{	// si inviano i valori degli adc filtrati delle due celle come pesi erogati sulle vie 13 e 14 
#ifdef _DEBUG_ADC_
		Via[5].peso_erogato = (word)samples[_ADC1_];
		Via[6].peso_erogato = (word)samples[_ADC2_];
#endif
#ifdef _debug_flussopeso_
		Via[0].peso_erogato = (word)m_peso_non_cala;
		Via[1].peso_erogato = 0;
		Via[2].peso_erogato = (word)m_peso_non_cala_precedente;
		Via[3].peso_erogato = 0;
		Via[4].peso_erogato = 0;
#endif
	}

	load1 = (float)((int)samples[_ADC1_] - (int)Chan[_ADC1_].Weightoffset);
	load2 = (float)((int)samples[_ADC2_] - (int)Chan[_ADC2_].Weightoffset);
	loadsystem1 = load1 + (load2 * Chan[_ADC1_].Weightgain);
	loadsystem = loadsystem1 * m_loadsystem2;
	
#ifdef _PESI_IN_GRAMMI_	
	// routine per evitare che gli arrotondamenti facciano muovere il valore dle pesp calcolato, a fronte della variaizone di uno solo dei due ADC e di
	// un solo punto di adc
	static float lastoldvalue = 0;
	float diffvalue = loadsystem - lastoldvalue;
	if((diffvalue < 0.09) && (diffvalue > -0.09))	// se la differena è inferiore ad un decimo
		loadsystem = lastoldvalue;				// allora non aggiorno il valore perchè le approssimazioni successive genererebbero un delta
	else
		lastoldvalue = loadsystem;			// altrimenti aggiorno
	
#endif

	return loadsystem;
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

byte SIFRA_Manager::getSIFRApreviousStatus()
{
	return StatusCmd.prevState;
}

byte SIFRA_Manager::getSIFRAphase()
{
	return StatusCmd.phase;
}

/**
* Riceve i dati per il calcolo del rapporto flusso peso, oltre alla linea utilizzata.
* Se il valore calcolato è accettabile, questo viene utilizzato per questa sessione e salvato in ram tamponata.
*/
float SIFRA_Manager::controllo_flussopeso(long step, word deltapeso, float reference)
{
	float diff;
	float calc = 0;

	calc = (float)step / (float)deltapeso;		// calcolo nuovo rapporto
	diff = abs(calc - reference);	// verifico la differenza 
	diff = diff / reference;
	if(diff < _SOGLIA_)		// verifico la differenza in percentuale
	{
		return calc;
	}
	
	return 0;
}

/**
* Durante il riempimento si verifica continuamente l'effettivo calo del peso per verificare ostruzioni della linea.
* La funzione errori torna un byte diverso da zero se c'è errore.
*/
byte SIFRA_Manager::controllo_non_cala(int line, int delta)
{
	byte error = 0;

	if(m_timerVerificaCalo.Match())
	{
		m_peso_non_cala = (word)m_peso_attuale;
		if(m_peso_non_cala_precedente > 0)
		{
			if(m_peso_non_cala_precedente >= m_peso_non_cala)
			{
				if(m_peso_non_cala_precedente > (m_peso_non_cala + delta))
				{
					g_stateMachine.nonCalaState = no_allarme_non_cala;
				}
				else
				{
					g_stateMachine.nonCalaState++;
				}
			}
			else
			{
				g_stateMachine.nonCalaState++;
			}
			switch (g_stateMachine.nonCalaState)
			{
				case no_allarme_non_cala:
				case alert_1_non_cala:
					break;
				case allarme_non_cala:
					m_timerVerificaCalo.Stop();
					error |= 0x01;
					StatusCmd.statusChan[_ADC1_] |= ERR_NON_CALA;
					return error;
					break;
				default:
					StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
					StatusCmd.status = STATO_ERRORE;
					error |= 0x01;
					StatusCmd.error_monitor = ERR_CASE_NON_CALA;
					return error;
					break;
						
			}
			if(getSpeedPump(0) < _PWM_CYCLE_LOW)
			{
				m_timerVerificaCalo.Preset(T_NON_CALA_ENC);
			}
			else
			{
				m_timerVerificaCalo.Preset(T_NON_CALA);
			}
		}
		else
		{
			if(getSpeedPump(0) < _PWM_CYCLE_LOW)
			{
				m_timerVerificaCalo.Preset(T_NON_CALA_ENC);
			}
			else
			{
				m_timerVerificaCalo.Preset(T_NON_CALA);
			}
		}
		m_peso_non_cala_precedente = m_peso_non_cala;
	}
	
	return error;
	
}

/**
* durante il riempimento si verifica continuamente la capacità residua di ogni componente
* per verificare l'eccessivo svuotamento dei flaconi
* la funzione errori torna un byte diverso da zero se c'è errore, con il bit a 1 che identifica la/le via/e in errore
*/
byte SIFRA_Manager::controllo_vuoto(int line)
{
	byte error = 0x00;
	
	if(Via[line].capacita < (_MARGINE_SU_VUOTO))	//capacità residua espressa in dml
	{
		error |=  0x01;	// setto un errore
		StatusCmd.statusChan[_ADC1_] |= ERR_RILEV_VUOTO;	// non c'è abbastanza liquido
		StatusCmd.error_monitor = ERR_CASE_VUOTO;
		m_void_source = True;
	}
	return error;
}

/**
* In M3300 system, this routine verifies if and which line is enabled
* Returns number of lines enabled (from 0 to 7 in M3300)
* Returns 0 if no lines enabled
*/
byte SIFRA_Manager::Find_Num_of_Enabled_Lines(byte command)
{
	byte i;
	byte m_enable = 0;							// azzero il numero delle vie da eseguire

	for(i = 0; i < NUM_MAX_LINE; i++)		// scorrendo sulle possibili vie, qui va bene NUM_MAX_LINE
	{
		
		if(command & (0x01 << i)) 			// se questa via è abilitata, leggendo dal comando ricevuto da tastiera
		{
			Via[i].da_eseguire = True;		// la via fa parte della sequenza
			m_enable++;					// incremento il numero delle vie da eseguire
		}		
	}
	if(m_enable > NUM_MAX_LINE) m_enable = NUM_MAX_LINE;	// non più di 8 vie abilitate
	
	return m_enable;
}

/**
* In M3300 system this routine verifies the first line that must be enabled.
* Returns number of the enabled line (from 0 to 7 in M3300).
* Returns -1 if no lines enabled.
*/
int SIFRA_Manager::Find_Enable_line(byte command)
{
	int i;
	
	for(i = 0; i < NUM_MAX_LINE; i++)	// scorrendo sulle possibili vie, qui va bene NUM_MAX_LINE
	{
		if(command & (0x01 << i)) // se questa via è abilitata
		{
			return i;	// ritorna il numero della via abilitata da 0 a (NUM_MAX_LINE - 1)
		}
	}
	
	return -1;	// se non trova linee abilitate
}

/**
* Verifica periodica dello stato delle celle e del peso massimo raggiunto.
* Se la cella è rotta (o anche solo disconnessa) il valore va ad uno degli estremi 0x0000 oppure 0xFFFF.
*/
bool SIFRA_Manager::Verifica_celle()
{
	bool result = False;

	if(m_TimerChkHw.Match())
	{
		// verifica della rottura delle celle o del distacco dei cavi cella
		if((m_WeightSample[_ADC1_] == 0x0000) || (m_WeightSample[_ADC1_] == 0xFFFF))
		{			
			StatusCmd.statusChan[_ADC1_] |= ERR_CELLA_ROTTA_SX;		// set errore cella rotta (a zero o a fondoscala)
			StatusCmd.flagsErrorLine = 0x01;					// questo è un errore bloccante
			Via[_VIA_8_].stato_luci = ERR_CELLA;				// accensione led e lampeggio adeguato
			m_TimerChkHw.Stop();
			result = True;										// errore massima priorità
			return result;
		}
		else
		{
			if((m_WeightSample[_ADC2_] == 0x0000) || (m_WeightSample[_ADC2_] == 0xFFFF))
			{
				StatusCmd.statusChan[_ADC1_] |= ERR_CELLA_ROTTA_DX;		// set errore cella rotta (a zero o a fondoscala)
				StatusCmd.flagsErrorLine = 0x01;					// questo è un errore bloccante
				Via[_VIA_15_].stato_luci = ERR_CELLA;						// accensione led e lampeggio adeguato
				m_TimerChkHw.Stop();
				result = True;											// errore massima priorità
				return result;
			}
			else
			{
				m_TimerChkHw.Preset(_TIMEOUT_CHKHW);
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
	bool result = False;

	if(m_TimerMaxWeight.Match())
	{
		// se la cella di sinistra è scollegata o non funzionante non eseguo il controllo, scatterà l'allarme di cella guasta
		if((m_WeightSample[_ADC1_] == 0x0000) || (m_WeightSample[_ADC1_] == 0xFFFF))
		{
			m_TimerMaxWeight.Preset(_3_SEC_);
			return False;
		}

		// se la cella di destra è scollegata o non funzionante non eseguo il controllo, scatterà l'allarme di cella guasta
		if((m_WeightSample[_ADC2_] == 0x0000) || (m_WeightSample[_ADC2_] == 0xFFFF))
		{
			m_TimerMaxWeight.Preset(_3_SEC_);
			return False;
		}
		
		if(m_peso_attuale > PESO_MASSIMO)			// ho caricato troppo la rastrelliera
		{
			StatusCmd.statusChan[_ADC1_] |= ERR_PESO_MASSIMO;
			StatusCmd.flagsErrorLine = 0x01;
			Via[_VIA_8_].stato_luci = Via[_VIA_15_].stato_luci = ERR_CELLA;		// lampeggiano i die led estermi delle vie 8 e 15
			m_TimerMaxWeight.Stop();
			result = True;
		}
		else
		{
			m_TimerMaxWeight.Preset(_3_SEC_);
		}
	}

	return result;
	
}

/**
* Verifica apertura sportello della pompa.
*/
bool SIFRA_Manager::Verifica_apertura_sportello()
{
	bool result = False;

	if(m_timerControlCover.Match())
	{
		m_timerControlCover.Preset(_1_SEC_);
		if(getCoverPumpState())	// se è aperto lo sportello, blocca tutto e genera errore
		{
			StatusCmd.flagsErrorLine = 0x01;
			StatusCmd.statusChan[_ADC1_] |= ERR_COVERAPERTA;
			m_timerControlCover.Stop();
			result = True;
		}
	}

	return result;

}

/**
* Aggiorna i contatori degli encoder motori.
* Se i motori sono in moto (se il timer m_check_block_pump sta contando) allora verifica anche eventuali blocchi.
*/
bool SIFRA_Manager::Verifica_encoder_motori()
{
	int vel_mot;
	bool state = True;
	int delta;
	
	if(m_check_block_pump.Match())
	{
		m_check_block_pump.Preset(_2_SEC_);
		if(StatusCmd.next_line >= 0)
		{
			if(Via[StatusCmd.next_line].abilitazione)
			{					
				vel_mot = getSpeedPump(0);
				m_old_step = m_this_step;
				m_this_step = (int)MotorStatus.step_motor_done;
				m_waiting_step = vel_mot * _SOGLIA_ENC_NON_GIRA;
				if(m_waiting_step >= 0)
				{	
					if (m_this_step > m_old_step)
					{
						delta = m_this_step - m_old_step;
					}
					else
					{
						delta = 0;
					}
					if((delta) < ((int)(m_waiting_step * _PERC_SOGLIA_ENCODER))) // se ho rilevato un numero di passi eseguiti, minore di una percentuale settabile dei passi attesi
					{
						StatusCmd.flagsErrorLine = 0x01;
						StatusCmd.statusChan[_ADC1_] |= ERR_POMPA_NON_GIRA;
						state = False;
					}
				}
				else
				{
					StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
					StatusCmd.status = STATO_ERRORE;
					StatusCmd.error_monitor = ERR_CASE_CALC_PUMP_NOT_MOVE;
				}
			}
		}
	}
	
	return state;
	
}

void SIFRA_Manager::Setta_filtro_peso(int val)
{
byte i;
	for(i = 0; i < _MAX_LOAD_CHAN_; i++)	// scorrendo sulle possibili vie, qui va bene NUM_MAX_LINE
		weightChan->set_filter_value(i, val);		// settaggio del filtro di media sul peso: 5 sample --> 60Hz/5 = circa 12Hz
}

/**
Leggo gli encoder e salvo il numero dei passi. Visto che il contatore non si azzera ma si ricicla, devo tener conto del giro, arrivato a 65535 va a 0
NOTA BENE: rid 32/1, ENC 500imp/giro motore --> 16000imp encoder / giro pompa
3.6mL / giro pompa 
3.6mL / 16000 impulsi encoder --> 4444impulsi/ml --> 444impulsi/decimo
1/8 di giro = 16000/8 = 2000impulsi --> 4,5decimi di ml
*/
void SIFRA_Manager::read_encoder_motors(int m_mot)		// aggiorna i contatori dei giri pompa
{
	dword this_step_motor = 0;
	word m_step_motor = 0;

	m_step_motor = cpld_counter1 & _MASK_16BIT_;		// copio il valore attuale del contatore
	if( m_step_motor < MotorStatus.previus_step_motor)	// valore corrente < precedente: il contatore ha fatto il giro
	{	
		if(MotorStatus.previus_step_motor > 65535)
			MotorStatus.previus_step_motor = MotorStatus.previus_step_motor & _MASK_16BIT_;
		this_step_motor = m_step_motor + (65535 - MotorStatus.previus_step_motor);	// devo sommare anche la parte del giro precednete
	}
	else
	{
		this_step_motor = m_step_motor - MotorStatus.previus_step_motor;
	}

	MotorStatus.step_motor_done += this_step_motor; // sommo i passi compiuti dal giro precedente
	// se è attiva la fase di movimento comandato, incremento il contatore dei passi voluti
	/*if((MotorStatus[m_mot].On == True) ||(MotorStatus[m_mot].LastReading == True))
	{
		MotorStatus[m_mot].LastReading = False;	// dopo aver settato il .On = False, bisogna fare un'ultima lettura, altrim si perdono gli ultimi impulsi
		MotorStatus[m_mot].step_motor_done += this_step_motor; // sommo i passi compiuti dal giro precedente
	}

	if(MotorStatus[m_mot].step_motor_really_done < MotorStatus[m_mot].step_motor_done)
		diff = MotorStatus[m_mot].step_motor_done - MotorStatus[m_mot].step_motor_really_done;
	else
		diff = MotorStatus[m_mot].step_motor_really_done - MotorStatus[m_mot].step_motor_done;

	if(diff > 4000)	// differenza superiore ad un quarto di giro
		MotorStatus[m_mot].count_error = True;	// differenza: si suppone che la pompa sia stata mossa manualmente 
	*/
	MotorStatus.previus_step_motor = m_step_motor;	// memorizzo il conteggio attuale per il giro successivo;
}

/**
si basa sull'uso di contatori a 14bit (16000 punti) perchè i due bit + significativi (15 e 16) sono usati per gestire il giro
*/
void SIFRA_Manager::encoder_updateCnt(void)
{
register word 		encCnt;
register word  	oldCnt;
register long   	deltaCnt;

	m_enc_value = cpld_counter1 & _MASK_16BIT_;
	encCnt = cpld_counter1 & _MASK_16BIT_;		// copio il valore attuale del contatore
	// magari ripeti + volte la lettura e confronta le due per escludere letture spurie

	oldCnt = (word)MotorStatus.previus_step_motor;		// e il precedente

	if( encCnt != oldCnt )			// se si muove ..
  	{
  		if( (encCnt & 0x8000) == (oldCnt & 0x8000) )  // bit msb non cambiato
  		{
  			deltaCnt = encCnt - oldCnt;
      			MotorStatus.step_motor_done += deltaCnt;
  		}
  		else
  		{
  			if( (encCnt & 0x8000) != 0 )		// possiamo avere caso 0x7f00 -> 0x8000 o 0x0000 -> 0xFF00
  			{
  				if( (encCnt & 0x4000) == 0 )	// sicuramente 0x7f00-> 0x8000
  				{							// 0x80
          				deltaCnt = encCnt - oldCnt;					// count up
              			MotorStatus.step_motor_done += deltaCnt;
  				}
  				else	// è andato all'indietro?	// sicuramente 0x00-> 0xFF
  				{							// 0xC0
          				deltaCnt = 0x10000;		// count down
          				deltaCnt -= encCnt;					
          				deltaCnt += oldCnt;
              			MotorStatus.step_motor_done -= deltaCnt;
  				}
  			}
  			else							// possiamo avere caso 0x8000 -> 0x7f00 o 0xFF00 -> 0x0000
  			{   				
  				if( (encCnt & 0x4000) == 0 )	// sicuramente 0xFF00-> 0x0000
  				{							// 0x00
	          			deltaCnt = 0x10000;
	          			deltaCnt -= oldCnt;		// count up
	          			deltaCnt += encCnt;
	              		MotorStatus.step_motor_done += deltaCnt;
  				}
  				else						// sicuramente 0x8000-> 0x7f00
  				{							// 0x40
	          			deltaCnt = oldCnt;		// count down
	          			deltaCnt -= encCnt;
	              		MotorStatus.step_motor_done -= deltaCnt;
  				}
  			}		
  		}
		MotorStatus.previus_step_motor = encCnt;				// aggiorna l'ultima lettura... in caso di variazioni
	}
}


/**
Reset of the encoder data struct of all the motors
*/
void SIFRA_Manager::ResetMotorStepCounter()
{
	MotorStatus.previus_step_motor = cpld_counter1 & _MASK_16BIT_;
	MotorStatus.step_motor_done = 0;
	MotorStatus.step_motor_expected = 0;
	m_old_step = 0;
	m_this_step = 0;
	m_waiting_step = 0;
}

void SIFRA_Manager::setStatePump(int i, int command)
{
	stato_pompa[ i ] = command;	
	if(command == ARRESTO)
	{
		setPwmMotCycle(0,0);		// setta la velocità in pwm a 0
		setBlockRelay(0, 0);		// ferma immediatamente il motore
	}
}

int SIFRA_Manager::getStatePump(int i)
{
	return stato_pompa[ i ];
}

/*
* Questa funzione è chiamata ogni volta che leggo lo stato dell'aria nei sensori
* Ad ogni accesso che avviene in una fase di riempimento calcolo la derivata dello stato encoder e da quello il flusso pompa stimato
*/
float SIFRA_Manager::calc_flusso_pompa()
{
	float flow = 0;
	long   deltaval = 0;

	m_enc_val = cpld_counter1 & _MASK_16BIT_;		// copio il valore attuale del contatore;		// copio il valore attuale del contatore

	m_enc_old_val = m_enc_prev_val;	// richiamo il valore dle giro precedente (100msec fa)

	if( m_enc_val != m_enc_old_val )			// se si muove ..
  	{
  		if( (m_enc_val & 0x8000) == (m_enc_old_val & 0x8000) )  // bit msb non cambiato
  		{
  			deltaval = m_enc_val - m_enc_old_val;
  		}
  		else
  		{
  			if( (m_enc_val & 0x8000) != 0 )		// possiamo avere caso 0x7f00 -> 0x8000 o 0x0000 -> 0xFF00
  			{
  				if( (m_enc_val & 0x4000) == 0 )	// sicuramente 0x7f00-> 0x8000
  				{							// 0x80
          				deltaval = m_enc_val - m_enc_old_val;					// count up
  				}
  				else	// è andato all'indietro?	// sicuramente 0x00-> 0xFF
  				{							// 0xC0
          				deltaval = 0x10000;		// count down
          				deltaval -= m_enc_val;					
          				deltaval += m_enc_old_val;
  				}
  			}
  			else							// possiamo avere caso 0x8000 -> 0x7f00 o 0xFF00 -> 0x0000
  			{   				
  				if( (m_enc_val & 0x4000) == 0 )	// sicuramente 0xFF00-> 0x0000
  				{							// 0x00
	          			deltaval = 0x10000;
	          			deltaval -= m_enc_old_val;		// count up
	          			deltaval += m_enc_val;
  				}
  				else						// sicuramente 0x8000-> 0x7f00
  				{							// 0x40
	          			deltaval = m_enc_old_val;		// count down
	          			deltaval -= m_enc_val;
  				}
  			}		
  		}
	}
	
	flow = ((float)deltaval) / structEnc.param_encoder;	// (impulsi /100msec) / (impulsi / deciml) = decimillilitri / 100msec
	m_enc_prev_val = m_enc_val;		// aggiorno il valore che mi servirà al prox giro
	
	return flow;	// ritorna il flusso attuale calcolato in decimi di millilitri/decimo di secondo
}

/**
* Calcola il rapporto passi_encoder/flusso e lo salva in ram tamponata
* solo se il delta peso (e quindi anche il delta encoder) sono superiori a un TOT, per evitare che i piccoli errori siano troppo influenti
*/
bool SIFRA_Manager::salva_rapporto_flussopeso(int m_line)
{
	float imp_flow;
	
	imp_flow =  controllo_flussopeso(MotorStatus.step_motor_done,Via[m_line].peso_erogato, _PAR_ENC_NOM_);	// verifica se il valore calcolato è accettabile o se ricarico quello nominale
	if(imp_flow > 0)	// valore coerente
	{
		structEnc.param_encoder = imp_flow;
		structEnc.tolleranza = m_tolleranza[_BASSA_TOLL];		// soglia dell'8%
		return True;
	}
	else
	{
		structEnc.tolleranza = m_tolleranza[_ALTA_TOLL];		// soglia del 15%
		return False;
	}
}

/**
* Inizializza la struttura parametri flusso peso con i valori nominali e di default
*/
void SIFRA_Manager::default_struttura_flussopeso()
{
	structEnc.param_encoder = _PAR_ENC_NOM_;			// carico il valore nominale
	structEnc.tolleranza = m_tolleranza[_ALTA_TOLL];		// tolleranza del 15% sulla verifica del flusso peso
	structEnc.restart = 0x55;
	SifraResetBackup();

	if(!Write_Param_Backup())
	{
		//StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
		//StatusCmd.flagsErrorLine = 0x01;
		StatusCmd.error_monitor = ERR_RAM_WRITE_BK;
		//break;
	}

	if(!Write_Param_Encoder())
	{
		//StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
		//StatusCmd.flagsErrorLine = 0x01;
		StatusCmd.error_monitor = ERR_RAM_WRITE_ENC;
	}
}

/**
* Tool di verifica di eventuali blocchi delle comunicazioni verso il driver pannello luci e verso gli adc, entrambi mappati su seriale sincrona e particolarmente colpiti dagli effetti delle ESD.
* L'algoritmo se riconosce un blocco del contatore che gira nella FSM degli adc o una o più celle a fondoscala, esegue una routine, che prevede prima il tenativo di ripristinare la comunicazione
* e se fallisce, allora impone il reset scheda, il tutto in modo che la tastiera si accorge solo di un allarme instabile. alla fine la scheda si dovrebbe ritrovare in stato errore isntabile, pompe ferme
* e pesi validi relativi all'ultimo backup
*/
void SIFRA_Manager::VerificaESD_Damage()
{
int line;
	switch(m_stato_check_blocco)
	{
		case verifica_contatore:	// verifico il contatore innestato nella macchina a stati della comunicazione con gli ADC
			if(CS5530_checkBlockComunication(10000) || CS5530_checkBlockCells(m_WeightSample)) // se il contatore supera la soglia, torna True
				m_stato_check_blocco = contatore_overflow;
			break;
		case contatore_overflow:
			for(line = 0; line < NUM_MAX_LINE; line++)
			{
					StatusCmd.flagsErrorLine |= (0x01 << line);	// setto un errore nella via m_chan solo se abilitata
					StatusCmd.statusChan[_ADC1_] |= ERR_PESOINSTABILE;	// set errore stabilità
			}
			CS5530_setWatchdogAdcTimer(8000);
			CS5530_setCounterWaitingData();
			load_stopSampling();
			//chiusra seriale verso pannello luci
			m_stato_check_blocco = verifica_watchdog_timer;
			break;
		case verifica_watchdog_timer:	// qui attende x secondi dal rilevamento blocco, prima di reinizializzare la comunicazione
			if(CS5530_getWatchdogAdcTimer())	// è scattato anche il watchdog fopo il blocco, è ora di resettare la comunic
			{
				CS5530_resetAdcComunication();		
				m_resetLedDrivers.Preset(10);	// allla prima esecuzione della led manager deve fare il reset
				StatusCmd.statusChan[_ADC1_] &= ERR_PESOINSTABILE_NEGATO;	// reset errore stabilità
				CS5530_setWatchdogAdcTimer(1500);		// un minimo ritardo per ripristinare la comunicazione con gli adcs
				m_stato_check_blocco =  verifica_contatore_dopo_reset_1;
			}
			break;
		case verifica_contatore_dopo_reset_1:
			if(CS5530_getWatchdogAdcTimer())	// è scattato anche il watchdog fopo il blocco, è ora di resettare la comunic
			{
				if(CS5530_checkBlockComunication(0))	// se si incrementa, quindi se funziona
				{
					CS5530_setWatchdogAdcTimer(10000);
					m_stato_check_blocco =  verifica_contatore_dopo_reset_2;
				}
				else
					m_stato_check_blocco = reset_scheda;	//	cambio stato reset_scheda
			}
			break;
		case verifica_contatore_dopo_reset_2:	
			if(CS5530_checkBlockCells(m_WeightSample)) // se il contatore supera la soglia, torna True
				m_stato_check_blocco = reset_scheda;	//	cambio stato reset_scheda
			else			// il contatore si incrementa ma non sappiamo ancora se va bene tutto
			{
				if(CS5530_getWatchdogAdcTimer())	// è scattato anche il watchdog fopo il blocco, è ora di resettare la comunic
					m_stato_check_blocco = verifica_contatore;	// sembra che tutto va bene, ritorno allo stato iniziale
				else
					if(CS5530_checkBlockComunication(10000)) // si incrementa ma il campionamento non è partito
						m_stato_check_blocco = contatore_overflow;
			}
			break;
		case reset_scheda:
			Backup_StatoStart();
			reset_4_block = 0xAA;	// setto il flag in eeprom, che al riavvio mi dice di fare un ripristino
			SIFRAMsg_resetApplicationHandler();	// impongo reset alla scheda
			break;
		default:
			break;
	}
}

/**
SET in red, in green or off the 'line' diode of the panel 
*/
void SIFRA_Manager::setta_led_pannello(int line, bool light)
{
	Via[line].stato_led = ON;
	if(light == ON)
		Via[line].stato_luci = SETTA_LUCE;
	else 
		Via[line].stato_luci = SPEGNI_LUCE;
}

void SIFRA_Manager::reset_state_this_led(int line, bool state)
{	
	Via[line].stato_led = state;
	Via[line].fase_luci = 0;
	Via[line].stato_luci = IDLE_LUCE;
}

/**
usata solo nella AirManager, serve per settare in maniera condizionata dallo stato attuale, le condizone e i flag di errore
se non c'è un allarme instabile sulla via e se è attivo il flag air_detect_en, allora accende la luce rossa lampegginate lenta
se la via è abilitata ed è attivo il flag air_block_en, allora setta l'errore macchina per il blocco motori
in ogni caso setta lo status chan così la tastiera vede la via in allarme e accende il flag rosso nella schermata service, anche nelle fasi di idle
*/
void SIFRA_Manager::set_air_alarm(int i)
{
	if(Via[i].abilitazione == True)
	{
		if(StatusCmd.air_block_en == True)
		{
			StatusCmd.flagsErrorLine |= 0x03;		// se errore aria sulla via abilitata, devo bloccare e lampeggiare il led rosso							
			StatusCmd.air_block_en = False;		// resetto il flag per evitare un loop errore/stop/errore/stop
		}
	}
	if((StatusCmd.statusChan[_ADC1_] & ERR_PESOINSTABILE) == 0)	// se non c'è un allarme instabile attivo, che ha priorità
	{
			Via[i].stato_luci = ERR_ARIA;
	}
}

/**
* Inizializzazione delle variabili di sistema e della classe SifraManager
*/
void SIFRA_Manager::Init_Variables()
{
	byte i;

	Via_Sample.clear();
	m_dato_valido = False;

	StatusCmd.status = STATO_IDLE;
	StatusCmd.error_monitor = 0;

	m_this_step = 0;
	m_old_step = 0;
	m_waiting_step = 0;
	m_vel = 0;
	m_enc_prev_val = 0;
	m_enc_val = 0;
	m_enc_old_val = 0;
	m_weight_real_time = 0;

	m_blink = False;
	
	m_void_source = False;
	m_calo_volume_stop = 0;
	
	for(i=0;i<NUM_MAX_LINE;i++)
	{
		m_count_aria[i] = 0;
		m_volume_aria[i] = 0;
		m_TimerAllarmeAria[i].Preset(_TIMER_INIT_READ_AIR);	// rilevamento aria con temporizzazione inizializzata a 5 secondi(tempo minimo di accensione della tastiera)
		Via[i].timer_led.Preset(_t_BLINK_STOP);			// inizializzo il timer dei led per ogni via
	}
	m_timer_test_fw_run.Preset(_TIME_TEST_FW);
	m_timerLed.Preset(_1_SEC_);		// timer necessario per far lampeggiare i led della scheda luci in INIT_DRIVER o se il fw caricato non corrisponde alla configurazione hw della CPU

	/*
	* Variabili di debug
	*/
	m_num_sample_mgr = 0;
	
}

void SIFRA_Manager::DefaultSystemReset()
{	
	Init_Variables();

	SifraResetSystemParams();
}

/**
* Flash board's led in case of hardware and firmware conflict
*/
void SIFRA_Manager::FlashLed_hw_error()
{
	if(m_timerLed.Match())
	{
		if(m_blink == True)
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
				LedDrivers_sendData(DECODE_MODE);
				LedDrivers_sendData(INTENSITY);
				LedDrivers_sendData(SCANLIMIT);
				LedDrivers_sendData(NORMAL_OP);
				m_timerLed.Preset(_TIMER_LED_FAST_);
				g_stateMachine.ledState = DISPLAY_READY;				
				break;
			case DISPLAY_READY:
				m_timerLed.Preset(_TIMER_LED_FAST_);
				switchoff_leds();								// spegni tutte le luci
				g_stateMachine.ledState = BLINK_LEDS_HW_ERROR;
				break;
			case BLINK_LEDS_HW_ERROR:	// lampeggio continuo dei leds per segnalare errore bloccante
				m_timerLed.Preset(_TIMER_LED_SLOW_);
				if(m_blink == True)
				{
					switchon_leds();		// accende tutti i led 
				}
				else
				{
					switchoff_leds();		// spegne tutti i led
				}
				break;
			default:
				break;
		}
		m_blink = ~m_blink;
	}
}

/**
* In base alla velocità pompa, alla frequenza di campionamento, calcola il delta peso stimato al prossimo campione
* ritornando un valore in centesimi di g
*/
word SIFRA_Manager::calc_next_theor_weight(int i)
{
	int t_velsu100;
	/*
	int t_freq;
	word t_stim;
	*/
	float weightExpected;
	float timeElapsed;
	word weightConv;

	timeElapsed = globalTimer.getTime() - m_timeControl;
	t_velsu100 = getSpeedPump(i);			// velocità istantanea pompa in PWM
	if(t_velsu100 > 0)
	{
		if(t_velsu100 > _PWM_CYCLE_MAX)
		{
			t_velsu100 = _PWM_CYCLE_MAX;				// a 65 per ora si ottine la velocità massima, oltre non va
		}
		weightExpected = (float)t_velsu100 * Speed_to_flow * timeElapsed;				// decimi di mL, per ora non considero il peso specifico associato alla via
		weightConv = (word)weightExpected;
		asm("nop");
		/*
		t_freq = weightChan->get_filter_value(i);			// numero campioni filtro secondario			
		t_stim = (word)(t_vel * (float)t_freq / _SAMPLE_FREQ);	// delta peso stimato fra questo ed il prossimo campione
		if(t_stim < 10)
		{
			t_stim = 10;								// delta minimo di un decimo di ml
		}
		else
		{
			asm("nop");
		}
		*/
	}
	else
	{
		weightConv = 0;
	}
	m_timeControl = globalTimer.getTime();

	// return t_stim;
	
	return weightConv;
	
}

/**
* It verifies if there are the conditions to check the volume. Verify is not to be done if:
* - continuative voided volume is less than 5ml
* - during run, there was a stop (for error type like air, instability or void)
*/
bool SIFRA_Manager::check_conditions(int line)
{
	bool ret = False;
	float controllo;

	controllo = ((float)(Via[line].peso_da_erogare))/Via[line].peso_spec;
	
	if(controllo > _VOL_MIN_FOR_ERR_PROD)						// target maggiore di 5ml
	{
		// togliere l'if con le due condizioni riportato di seguito se si vuole eseguire il controllo volume/prodotto anche in caso di stop per errori om desiderato dall'operatore durante l'esecuzione di una via
		if (m_restart == False)		// se la sequenza dopo una ripartenza doveva erogare  ancora più di 5 ml
		{
			ret = True;		// allora posso eseguire il contollo
		}
	}
	
	return ret;
}

/*
* Verifica di errore di volume, cioè non corrispondenza oltre una certa tolleranza tra il peso erogato e quello calcolato
* usando i passi encoder motore (volume stimato erogato * peso specifico).
* Non è un errore bloccante, ma deve essere comunicato alla tastiera.
*/
byte SIFRA_Manager::check_volume_error(int line)
{
	float peso_calcolato;
	float verifica_peso;
	float verifica_peso_abs;
	float correction_balance;
	byte controllo_volume = no_errore_volume;

	peso_calcolato = ((float)MotorStatus.step_motor_done /structEnc.param_encoder) * Via[line].peso_spec;		// grandezza "vera" considerata tolleranza del 5%

	if((line == _VIA_8_) || (line == _VIA_8_+1))
	{
		correction_balance = peso_calcolato * _SOGLIA_CORR_BALANCE;
		peso_calcolato = peso_calcolato + correction_balance;
	}

	if(peso_calcolato > 0)
	{
		verifica_peso = ((float)(Via[line].peso_erogato) - peso_calcolato)/peso_calcolato; // il peso calcolato è il mio riferimento attendibile
		verifica_peso_abs = abs(verifica_peso);
		if( (verifica_peso_abs > structEnc.tolleranza) && (verifica_peso_abs < SOGLIA_ERR_PRODOTTO))	// se il peso misurato è diverso da quello calcolato dai passi encoder entro il 30 %
		{
			controllo_volume = errore_prodotto;		// segnala errore prodotto
			StatusCmd.statusChan[_ADC1_] |= ERR_PRODOTTO;
			StatusCmd.statusChan[_ADC2_] |= (0x0100 << line); // setto l'errore flusso peso sul secondo byte della variabile
		}
		else		// se la differenza è maggiore del 30 %
		{
			if (abs(verifica_peso) > SOGLIA_ERR_PRODOTTO)		// per ora segnalo sempre errore prodotto
			{
				controllo_volume = errore_prodotto;		// segnala errore prodotto
				//controllo_volume = errore_flusso;		// segnala errore flusso dovuto a strozzatura della linea
				StatusCmd.statusChan[_ADC1_] |= ERR_PRODOTTO;
				//StatusCmd.statusChan[_ADC1_] |= ERR_FLUSSO;
				StatusCmd.statusChan[_ADC2_] |= (0x0100 << line); // setto l'errore flusso peso sul secondo byte della variabile, da modificare 0x0100 con il valore corretto che verrà riconosciuto dalla tastiera relativo all'errore di flusso
			}
		}
	}
	
	return controllo_volume;
}

float SIFRA_Manager::calc_param_weight(ChannelsBackupParam *m_chan)
{
	float param;

#ifdef _PESI_IN_GRAMMI
	// in grammi con decimali
	param = 2000.0 / (float)((float)m_chan[_ADC1_].AdcTo2Kg + ((float)(m_chan[_ADC2_].AdcTo2Kg)*m_chan[_ADC1_].Weightgain));
#else
	// in decimi di grammi
	if(m_chan[_ADC1_].AreCalibrate && m_chan[_ADC2_].AreCalibrate)
	{
		param = 20000 / (float)((float)m_chan[_ADC1_].AdcTo2Kg + ((float)(m_chan[_ADC2_].AdcTo2Kg)*m_chan[_ADC1_].Weightgain));
	}
	else
	{
		param = _LOAD_SYSTEM2_DEFAULT_;
	}
#endif

	return param;
}

/**
 * Funzione che aggiorna il valore della capacità corrente ad ogni chiamata. 
 * Dal delta peso, con il peso specifico, calcola il delta volume che poi arrotonda all'unità.
 * Cerca di arrotondare con precisione 0,5 ml.
 */
word SIFRA_Manager::aggiorna_capacita(word cap_ini, word peso_erogato, float peso_spec)
{
	word cap;
	word delta_cap;
	
	delta_cap = (word)((float)peso_erogato/peso_spec);
	if(delta_cap < cap_ini)
	{
		cap = cap_ini - delta_cap;
	}
	else
	{
		cap = 0;
	}
	
	return cap;			// ml
}

/*
* Setta la capacità iniziale all'inizio di una erogazione, controllando che la residua sia sufficiente per l'erogazione
*/
bool SIFRA_Manager::controllo_capacita_iniziale(word cap_passata)
{
	bool result;
	
	if(cap_passata < _MARGINE_SU_VUOTO)
	{
		StatusCmd.flagsErrorLine = 0x01;	// setto un errore
		StatusCmd.statusChan[_ADC1_] |= ERR_RILEV_VUOTO;	// non c'è abbastanza liquido
		m_void_source = True;
		result = False;
	}

	result = True;

	return result;
}

/**
* Scorre lo stato delle vie incluse in sequenza, per verificare quale è da eseguire e non è ancora stata eseguita.
* Alla prima trovata, esce e ritorna il numero di via da 0 a 7.
* @return int: -1 se nessuna via da eseguire
 */
int SIFRA_Manager::ViaDaEseguire()
{
	for(int i = 0; i < NUM_MAX_LINE; i++)
	{
		if((Via[i].da_eseguire == True) && (Via[i].eseguita == False))	// la via con numero più basso da eseguire 
		{
			return i;					// la prima che trovo, mi fa uscire dal ciclo, per proseguire
		}
	}
	return -1;
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
	if((spec > _PESO_SPEC_MIN_) && (spec < _PESO_SPEC_MAX_))
	{
		return spec;
	}
	else
	{
		spec = 1.0;
		return spec;
	}
}

void SIFRA_Manager::UpdateDebugStruct()
{
	StatusDebug.sifraPrevStatus = StatusCmd.prevState;
	StatusDebug.sifraStatus = StatusCmd.state;
	StatusDebug.sifraPhase = StatusCmd.phase;

	StatusDebug.peso_current_time_int = (int)m_peso_attuale;
	StatusDebug.peso_current_time_word = (word)m_peso_attuale;
	StatusDebug.sampleMngr_cycles = m_num_sample_mgr;

	StatusDebug.sacca_manager_state = g_stateMachine.saccaState;
	StatusDebug.peso_parziale = m_peso_parziale;

	StatusDebug.enc_value = m_enc_value;
	StatusDebug.enc_value_control= m_enc_value_control;
	StatusDebug.enc_movement = m_enc_movement;
	StatusDebug.enc_up_limit = m_enc_upper_limit;
	StatusDebug.enc_low_limit = m_enc_lower_limit;
	StatusDebug.weight = m_weight_real_time;
	StatusDebug.air_state[0] = cpld_pin_Air1;
	StatusDebug.air_state[1] = cpld_pin_Air2;
	StatusDebug.air_state[2] = cpld_pin_Air3;
	StatusDebug.air_state[3] = cpld_pin_Air4;
	StatusDebug.air_state[4] = cpld_pin_Air5;
	StatusDebug.air_state[5] = cpld_pin_Air6;
	StatusDebug.air_state[6] = cpld_pin_Air7;
	StatusDebug.air_state[7] = cpld_pin_Air8;
	StatusDebug.cover_pump = cpld_CoverPump;
	StatusDebug.error_control = StatusCmd.error_monitor;
	StatusDebug.init_weight = m_init_weight;
	asm("nop");
}

void SIFRA_Manager::Encoder_counter()
{
	word enc_value_temp;

	if(m_enc_value >= m_enc_value_control)
	{
		m_enc_movement = m_enc_value - m_enc_value_control;
	}
	else
	{
		enc_value_temp = MAX_VALUE_16_BIT - m_enc_value_control;
		m_enc_movement = m_enc_value + enc_value_temp;
	}
	
	m_enc_value_control = m_enc_value;
	
}

void SIFRA_Manager::EncoderPumpsControl()
{
	word range;
	word upper_limit;
	word lower_limit;

	range = (word)(_ENC_STEP_EXPECTED * _PERC_ENC_CONTROL);
	lower_limit = (word)(_ENC_STEP_EXPECTED - range);
	upper_limit = (word)(_ENC_STEP_EXPECTED + range);

	m_enc_lower_limit = lower_limit;
	m_enc_upper_limit = upper_limit;

	if(m_first_enc_control)
	{
		m_first_enc_control = False;
	}
	else
	{
		if((m_enc_movement < lower_limit) || (m_enc_movement > upper_limit))
		{
			StatusCmd.flagsErrorLine |= 0x01;	// setto la presenza di un errore
			StatusCmd.error_monitor = E_ERROR_ENCODER;	// set errore encoder
			m_verify_enc_timer.Stop();
		}
	}
}

void SIFRA_Manager::SIFRA_resetStatusErrorStop()
{
	StatusCmd.statusChan[_ADC1_] &= ERR_STOP;
	StatusCmd.statusChan[_ADC2_] &= 0xFF00;
}

void SIFRA_Manager::SIFRA_resetStatusError()
{
	StatusCmd.statusChan[_ADC1_] &= ERR_END_FORMULA;
	StatusCmd.statusChan[_ADC2_] &= 0x0000;
	StatusCmd.error_stop = E_ERR_STOP_NONE;
}

void SIFRA_Manager::Set_weight_threshold()
{
	m_weight_lower_limit = m_init_load - THRESHOLD_WEIGHT;
	m_weight_upper_limit = m_init_load + THRESHOLD_WEIGHT;
}

void SIFRA_Manager::LoadControl()
{
	if((m_peso_attuale < m_weight_lower_limit) || (m_peso_attuale > m_weight_upper_limit))
	{
		StatusCmd.flagsErrorLine |= (0x01);	// setto un errore nella via m_chan solo se abilitata
		StatusCmd.error_monitor = E_ERROR_WEIGHT;	// set errore pesi
		m_verify_enc_timer.Stop();
	}
}

void SIFRA_Manager::set_unstable_threshold(byte __level)
{
	switch(__level)
	{
		case E_LEVEL_UNSTABLE_LOW:
			m_weightUnstableLevel = THREESHOLD_FILLING_INCR_LEVEL_1;
			break;
		case E_LEVEL_UNSTABLE_MED_1:
			m_weightUnstableLevel = THREESHOLD_FILLING_INCR_LEVEL_2;
			break;
		case E_LEVEL_UNSTABLE_MED_2:
			m_weightUnstableLevel = THREESHOLD_FILLING_INCR_LEVEL_3;
			break;
		case E_LEVEL_UNSTABLE_HIGH:
			m_weightUnstableLevel = THREESHOLD_FILLING_INCR_LEVEL_4;
			break;
		default:
			StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
			StatusCmd.status = STATO_ERRORE;
			StatusCmd.error_monitor = ERR_CASE_SET_STABILITY_LEVEL;
			break;
	}
}

/**
* Identifica la fase del sistema in base alla variabile m_SIFRAstatus
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
		case StatoRisciacquo:
		case StatoTestPinze:
			StatusCmd.phase = E_PHASE_FILLING;
			break;
		case StatoSvuotatubi:
			StatusCmd.phase = E_PHASE_SVUOTATUBI;
			break;
		case StatoErrore:
			StatusCmd.phase = E_PHASE_ERROR;
			break;
		case StatoStop:
		case PausaErogazione:
		case StopAllStatus:
			StatusCmd.phase = E_PHASE_PAUSE;
			break;
		case TermineSequenza:
			StatusCmd.phase = E_PHASE_END_SEQUENZA;
			break;
		case StatoService:
		case StatoTestPompe:
		case StatoServiceEV:
		case StatoTestEMC:
		case StatoTestLuci:
		case StatoTestFlussoPompa:
		case StatoTestEvAperte:
			StatusCmd.phase = E_PHASE_SERVICE;
			break;
		case RipresaMancanzaRete:
			StatusCmd.phase = E_PHASE_RIPRESA_NO_ALIM;
			break;
		default:
			StatusCmd.phase = E_PHASE_NUM;
			break;
	}
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
				m_test_led = E_LED_6_GREEN;
				break;
			case E_LED_6_GREEN:		// accendo il led verde di via 6
				m_test_led_timer.Preset(_1_SEC_);
				LedDrivers_sendData(LED_VIA5 | TURN_OFF);
				LedDrivers_sendData(LED_VIA6 | LED_VERDE);
				m_test_led = E_LED_6_RED;
				break;
			case E_LED_6_RED:		// accendo il led rosso di via 6
				m_test_led_timer.Preset(_1_SEC_);
				LedDrivers_sendData(LED_VIA6 | TURN_OFF);
				LedDrivers_sendData(LED_VIA6 | LED_ROSSO);
				m_test_led = E_LED_7_GREEN;
				break;
			case E_LED_7_GREEN:		// accendo il led verde di via 7
				m_test_led_timer.Preset(_1_SEC_);
				LedDrivers_sendData(LED_VIA6 | TURN_OFF);
				LedDrivers_sendData(LED_VIA7 | LED_VERDE);
				m_test_led = E_LED_7_RED;
				break;
			case E_LED_7_RED:		// accendo il led rosso di via 7
				m_test_led_timer.Preset(_1_SEC_);
				LedDrivers_sendData(LED_VIA7 | TURN_OFF);
				LedDrivers_sendData(LED_VIA7 | LED_ROSSO);
				m_test_led = E_LED_8_GREEN;
				break;
			case E_LED_8_GREEN:		// accendo il led verde di via 8
				m_test_led_timer.Preset(_1_SEC_);
				LedDrivers_sendData(LED_VIA7 | TURN_OFF);
				LedDrivers_sendData(LED_VIA8 | LED_VERDE);
				m_test_led = E_LED_8_RED;
				break;
			case E_LED_8_RED:		// accendo il led rosso di via 8
				m_test_led_timer.Preset(_1_SEC_);
				LedDrivers_sendData(LED_VIA8 | TURN_OFF);
				LedDrivers_sendData(LED_VIA8 | LED_ROSSO);
				m_test_led = E_LED_GREEN;
				break;
			case E_LED_GREEN:		// accendo tutti i led verdi
				m_test_led_timer.Preset(_1_SEC_);
				LedDrivers_sendData(LED_VIA8 | TURN_OFF);
				LedDrivers_sendData(LED_VIA1 | LED_VERDE);
				LedDrivers_sendData(LED_VIA2 | LED_VERDE);
				LedDrivers_sendData(LED_VIA3 | LED_VERDE);
				LedDrivers_sendData(LED_VIA4 | LED_VERDE);
				LedDrivers_sendData(LED_VIA5 | LED_VERDE);
				LedDrivers_sendData(LED_VIA6 | LED_VERDE);
				LedDrivers_sendData(LED_VIA7 | LED_VERDE);
				LedDrivers_sendData(LED_VIA8 | LED_VERDE);
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

/**
* Metodo utilizzato per resettare i parametri di calibrazione memorizzati in EEPROM
*/
bool SIFRA_Manager::SIFRACalibResetData()
{
	bool result = True;
	byte type_of_cal;

	type_of_cal = CHAN_CALIB_RESET_VALUE;

	if(!reset_backup_factory_values(_ADC1_, type_of_cal))
	{
		result = False;
	}
	if(!reset_backup_factory_values(_ADC2_, type_of_cal))
	{
		result = False;
	}

	return result;
	
}

