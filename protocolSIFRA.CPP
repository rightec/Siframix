/**
@file		protocolSIFRA.cpp
@brief		Class to manage siframix protocol.

			This class manage the communication protocol through this module and the master module M3000 :
			- receives and decodes messages from the UART 
			- manages the answers to the messages
			- .....
			
@author		Fregni Francesco
@date		18/01/2011
@version	01.00
*/

#include "protocolSIFRA.h"

void (*fp)(void);
STRUCTSTARTCMD StartCmd;
STRUCTSTATUSCMD StatusCmd;
structStatusDebug_t StatusDebug;
structWeightCmd_t WeightCmd;
structRestartFormulaCmd_t g_restartFormulaCmd;
structScalesPhaseCmd_t g_scalesPhaseCmd;
LINES_Status Via[ NUM_MAX_LINE];
structMachineState_t g_stateMachine;

BK_struct bk;
FP_STRUCT structEnc;

extern const char sAppName[];
extern const char sAppRevision[];
extern const char sAppDate[];
extern const char sHwVersion[];

extern const int sAppNameLength;
extern const int sAppRevisionLength;
extern const int sAppDateLength;
extern const int sHwVersionLength;

extern ChannelsBackupParam Chan[];

//---------------------------------------------------------------------------------------
//------------------------------ pcProtocol ---------------------------------------------
//---------------------------------------------------------------------------------------
SIFRAProtocol::SIFRAProtocol(UARTDevice device, int dim) :
	stdUartProtocolAbstraction(device)
{
	UartDeviceAbstraction::SetBaudRate(BR19200, True);
	stdUartProtocolAbstraction::setReceiverState(stdUart_waitDeviceID);
	stdUartProtocolAbstraction::setLastDataReceivedBuffer(dim);
	rstBytesReceived();
	m_show_adcvalue = False;
	m_show_velpompe = False;
	m_lastRemoteNodeID = 0;
	m_localNodeID = 0;	
	m_lastHwError = SIFRAUart_Hw_NoError;

	m_numStopPkts = 0;
	m_numStartPkts = 0;
	
	stdUartProtocolAbstraction::SetDataAnalyseCallBack(&SIFRADataAnalyse);

	QueueUart::TurnOn();
}

SIFRAProtocol::~SIFRAProtocol()
{

}

bool SIFRAProtocol::SIFRADataAnalyse(unsigned short dataS, stdUartProtocolAbstraction *protocol)
{
	unsigned char data;
	int error;
	bool hwErrorRecognised;
	static int opCode = 0;
	static int lastRemoteID = 0;
	int numbytetorecieve = 0;
	
	error = (int)stdUart_NoError;
	data = (unsigned char) dataS;
	
	if (dataS & 0xff00)
	{
		hwErrorRecognised = False;
		((SIFRAProtocol*)protocol)->rstLastHwError();	
		
		if((dataS & ReceptionError) == ReceptionError)
		{
			hwErrorRecognised = True;
			protocol->setError((int)stdUart_Hw_ReceptionError, True);
		}
		if((dataS & ParityError) == ParityError)
		{
			hwErrorRecognised = True;
			protocol->setError((int)stdUart_Hw_ParityError, True);
		}
		if((dataS & FrameError) == FrameError)
		{
			hwErrorRecognised = True;
			protocol->setError((int)stdUart_Hw_FrameError, True);
		}
		if((dataS & OverrunError) == OverrunError)
		{
			hwErrorRecognised = True;
			protocol->setError((int)stdUart_Hw_OverrunError, True);
		}
		if((dataS & NoDevice) == NoDevice)
		{
			hwErrorRecognised = True;
			protocol->setError((int)stdUart_Hw_NoDevice, True);
		}
		
		if (!hwErrorRecognised)
		{
			protocol->setError((int)stdUart_Hw_ErrorUnknown, True);
		}
		
		protocol->setReceiverState(stdUart_waitDeviceID);
		protocol->stopRxTimeoutTimer();
		return False;
	}	

	switch(protocol->getReceiverState())
	{
// tutti i moduli acquisiscono il messaggio e solo alla fine, possono decidere se è destinato a loro e quindi da analizzare oppure se è da scartare			
		case stdUart_waitDeviceID:	// parto da qui
			if((data == M3300) || (data == STOP_ALL))		// IDENTIFICATIVO MODULO
			{
				protocol->stopTxTimeoutTimer();
				protocol->setTimeoutRx(500);
				protocol->startRxTimeoutTimer();
				protocol->setReceiverState(stdUart_waitOpCode1);
				lastRemoteID = data;
			}
			else
			{
				PinEnRS485_DE = 0;
				protocol->rstBytesReceived();
			}
			break;

		case stdUart_waitOpCode1:	// tutti i casi di opcode previsti dal prot
				// preleva il primo OPCODE e determina se ci sono ulteriori byte da leggere
				if (((SIFRAProtocol*)protocol)->checkOpCode(data, lastRemoteID, numbytetorecieve))	
				{
					opCode = (int)data;	// qui salvo il tipo di opcode
					
					protocol->setPacketlength((int)numbytetorecieve);
					protocol->rstBytesReceived();
					if(numbytetorecieve != 0)
					{
						protocol->setReceiverState(stdUart_receivingData);	
					}else
					{
						protocol->setReceiverState(stdUart_waitChecksum);	
					}		
				}
				else
				{
					protocol->setError(stdUart_UnknowOpcode, True);
					protocol->stopRxTimeoutTimer();			
					protocol->setReceiverState(stdUart_waitDeviceID);
					PinEnRS485_DE = 0;
					protocol->rstBytesReceived();
					return False;
				}
			break;

		case stdUart_receivingData:
			// m_numBytesRecieved è stato incrementato all'uscita del giro precedenete, qui vale almeno 1
			if (!protocol->addDataToReceiveBuffer(&data, protocol->getNumBytesReceived() -1 , 1))
			{
				protocol->setReceiverState(stdUart_waitDeviceID);
				protocol->stopRxTimeoutTimer();
				PinEnRS485_DE = 0;
				protocol->rstBytesReceived();
				return False;
			}
			if (protocol->getNumBytesReceived() >= protocol->getPacketLength())
			{
				protocol->setReceiverState(stdUart_waitChecksum);	
			}		
			break;
			
		case stdUart_waitChecksum:
			
			protocol->stopRxTimeoutTimer();
			((SIFRAProtocol*)protocol)->setLastRemoteNodeID(lastRemoteID);//protocol->setLastRemoteNodeID(lastRemoteID);

			if((((SIFRAProtocol*)protocol)->getLastRemoteNodeID() == ((SIFRAProtocol*)protocol)->getLocalNodeID()) ||
				(((SIFRAProtocol*)protocol)->getLastRemoteNodeID() == STOP_ALL))
			{
				protocol->setNumDataReceived(protocol->getPacketLength());
				if(((SIFRAProtocol*)protocol)->checksum(data, (byte)opCode, (byte)lastRemoteID))	// devo sommare anche il primo char, lastRemoteID
				{
					protocol->setReceivedOpCode(opCode);
					protocol->newPacketReceived();
					StatusCmd.SIFRAComPacketsRx = protocol->getPacketsReceived();
				}
				else
				{
					protocol->setError(stdUart_ChecksumError, True);
					protocol->rstBytesReceived();
					protocol->setReceiverState(stdUart_waitDeviceID);
					PinEnRS485_DE = 0;
					return False;
				}
			}
			else		// non è messaggio per me, resetta buffer, ignora opcode e ritorna alo stato iniziale
			{
				PinEnRS485_DE = 0;	// libera la rete per la risposta del modulo giusto
				opCode = 0;
				protocol->setReceiverState(stdUart_waitDeviceID);
				protocol->rstBytesReceived();
				return False;
			}
			protocol->setReceiverState(stdUart_waitDeviceID);
			return True;
			break;

		case stdUart_waitStx:
		case stdUart_waitNumPacketTransmit:	
		case stdUart_waitOpCode2:					
		case stdUart_waitDataLength:
		case stdUart_waitEtx:
		default:
			protocol->setError(stdUart_stateProtocolUnknown, True);
			break;
	}
	
	protocol->newByteReceived();
	
	return False;
	
}


void SIFRAProtocol::Manager()
{
	int error;
	int opCode;

	stdUartProtocolAbstraction::Manager();			// analisi dei dati ricevuti e verifica dei flag di errore di overflow

	//---- Remapping stdUartProtocolAbstraction error ----
	error = stdUartProtocolAbstraction::popError();		// se presente, preleva l'errore nella fifo degli errori
	while (error != stdUart_NoError)					// verifica TUTTI gli eventuali errori presenti nella fifo degli errori
	{
		switch(error)
		{
			// errori hardware
			case stdUart_Hw_ReceptionError:
				m_lastHwError |=  SIFRAUart_Hw_ReceptionError;
				sendMsg(SIFRAMsg_HwError);
				break;
			case stdUart_Hw_ParityError:
				m_lastHwError |=  SIFRAUart_Hw_ParityError;
				sendMsg(SIFRAMsg_HwError);
				break;
			case stdUart_Hw_FrameError:
				m_lastHwError |=  SIFRAUart_Hw_FrameError;
				sendMsg(SIFRAMsg_HwError);
				break;
			case stdUart_Hw_OverrunError:
				m_lastHwError |=  SIFRAUart_Hw_OverrunError;
				sendMsg(SIFRAMsg_HwError);
				break;
			case stdUart_Hw_NoDevice:
				m_lastHwError |=  SIFRAUart_Hw_NoDevice;
				sendMsg(SIFRAMsg_HwError);
				break;
			case stdUart_Hw_ErrorUnknown:
				m_lastHwError |= SIFRAUart_Hw_ErrorUnknown;
				sendMsg(SIFRAMsg_HwError);
				break;
			// errori di protocollo
			case stdUart_UnknowOpcode:
				sendMsg(SIFRAMsg_opcodeUnknown);
				break;
			case stdUart_stateProtocolUnknown:
				sendMsg(SIFRAMsg_stateProtocolUnknown);
				break;
			case stdUart_TimeOutErrorRx:
				sendMsg(SIFRAMsg_timoutRx);
				break;
			case stdUart_TimeOutErrorTx:
				sendMsg(SIFRAMsg_timoutTx);
				break;
			case stdUart_ChecksumError:
				sendMsg(SIFRAMsg_checkSumError);
				break;
			case stdUart_TransmitBufferOverflowError:
				sendMsg(SIFRAMsg_transmitBufferOverflowError);
				break;
			case stdUart_ReceptionBufferOverflowError:
				sendMsg(SIFRAMsg_receptionBufferOverflowError);
				break;
			case stdUart_DataReceivedBufferOverflow:
				sendMsg(SIFRAMsg_dataReceivedBufferOverflow);
				break;
			// errori sconosciuti
			default:
				sendMsg(SIFRAMsg_unknownError);	
				break;

		}
		error = stdUartProtocolAbstraction::popError();
	}
	
	//------- Checking for packets receiving ----------------
	/**
	* Identifico solamente i comandi previsti per il modulo M3300, quelli per gli altri moduli vengono filtrati dalla DataAnalyse richiamata nella Manager della classe protocolStdUart
	*/
	if(stdUartProtocolAbstraction::popPacketReceived(opCode))	// se c'è un comando in coda, lo rispedisce per conferma
	{
		switch (opCode)
		{
			case SIFRA_SET_STOP_ALL:
				SIFRA_set_Stop_All();		// handle comando stop generale
				break;
			case SIFRA_M3000_STOP:
				SIFRA_set_Stop_All();		// handle comando stop generale
				last_opCode = opCode;
				sendAcknowledge();
				break;
			case SIFRA_GET_STATUS_M3300:
				SIFRA_getStatusCmdDecode();		// status
				break;
			case SIFRA_SET_START_M3300:
				SIFRA_setStart();
				SIFRA_decodeStartCmd(stdUartProtocolAbstraction::m_lastDataReceived);
				break;
			case SIFRA_GET_DEBUG_VIA_M3300:
				SIFRA_getDebugViaCmdDecode();
				break;
			case SIFRA_GET_DEBUG_START_M3300:
				SIFRA_getDebugStartCmdDecode();
				break;
			case SIFRA_GET_DEBUG_STATUS_M3300:
				SIFRA_getDebugStatusCmdDecode();
				break;
			case SIFRA_GET_DEBUG_BACKUP_RAM_M3300:
				SIFRA_getDebugRamCmdDecode();
				break;
			case SIFRA_SET_WEIGHT_SENSIBILITY_M3300:
				SIFRA_setWeightSensibility();
				last_opCode = opCode;			// per la risposta
				SIFRA_decodeWeightSensibilityCmd(stdUartProtocolAbstraction::m_lastDataReceived);
				break;
			case SIFRA_RESTART_FORMULA_M3300:
				SIFRA_restartFormula();
				last_opCode = opCode;			// per la risposta
				SIFRA_decodeRestartFormulaCmd(stdUartProtocolAbstraction::m_lastDataReceived);
				break;
			case SIFRA_ERROR_RESTART_CONTROL:
				SIFRA_restartWeightCoverControl();
				last_opCode = opCode;
				break;
			case SIFRA_GET_INFO_M3300:
				SIFRA_getInfoCmdDecode();	
				break;
			case SIFRA_GET_SCALES_STATE_M3300:
				SIFRA_setScalesPhase();
				last_opCode = opCode;
				SIFRA_decodeScalesPhaseCmd(stdUartProtocolAbstraction::m_lastDataReceived);
				break;
			case SIFRA_GET_SCALES_NOT_CALIB_M3300:
				SIFRA_setScalesNotCalib();
				last_opCode = opCode;
				break;
			case SIFRA_SETZERO_VERIFICATION:
				SIFRA_setZeroVerification();
				last_opCode = opCode;			// per la risposta
				break;
			case SIFRA_RESET_CALIB_PARAMS:
				last_opCode = opCode;			// per la risposta
				SIFRA_setResetCalib();
				break;
			case SIFRA_SETZERO_CELL8_CELL15:
				SIFRA_setZeroCell8_Cell15();
				last_opCode = opCode;	// per la risposta
				break;
			case SIFRA_SET_CAL_CELL8:
				SIFRA_setGainCell8();
				last_opCode = opCode;	// per la risposta
				break;
			case SIFRA_SET_CAL_CELL15:
				SIFRA_setGainCell15();
				last_opCode = opCode;	// per la risposta
				break;
			case SIFRA_JUMP_TO_LOADER:
				SIFRA_setJumpToLoader();
				last_opCode = opCode;	// per la risposta
				break;
			default:
				StatusCmd.status = STATO_ERRORE;
				StatusCmd.statusChan[_ADC1_] = ERR_GENERIC;
				StatusCmd.error_monitor = ERR_OPCODE_UNKNOWN;
				break;
		}
	}
}

int SIFRAProtocol::SIFRA_decodeStartCmd( byte *cmd_byte_buffer)
{
short int i = 0;
short int num_line;

	StartCmd.function = ( 0x00 | cmd_byte_buffer[ i++ ]) << 8;	// MSB peso via 1
	StartCmd.function |= cmd_byte_buffer[ i++ ];		// POS_FUNC_IN_CMD = 1
	i++;		// byte nullo
	StartCmd.support = cmd_byte_buffer[ i++ ];		// POS_SUPP_IN_CMD = 2
	for( num_line = 0; num_line < NUM_MAX_LINE; num_line++)
	{
		#ifdef _RICEVI_PESI_SPECIFICI
			StartCmd.peso_spec[ num_line ] = ( 0x00 | cmd_byte_buffer[ i++ ]) << 8;	// MSB peso via 1
			StartCmd.peso_spec[ num_line ] |= cmd_byte_buffer[ i++ ];	// LSB peso via 1
		#else
			StartCmd.peso_spec[ num_line ] = 1.000;
		#endif
	}
	for( num_line = 0; num_line < NUM_MAX_LINE; num_line++)
	{
		StartCmd.peso_linea[ num_line ] = ( 0x00 | cmd_byte_buffer[ i++ ]) << 8;	// MSB peso via 1
		StartCmd.peso_linea[ num_line ] |= cmd_byte_buffer[ i++ ];	// LSB peso via 1
	}
	for( num_line = 0; num_line < NUM_MAX_LINE; num_line++)
	{
		StartCmd.cap_linea[ num_line ] = ( 0x00 | cmd_byte_buffer[ i++ ]) << 8;	// MSB tara via 1
		StartCmd.cap_linea[ num_line ] |= cmd_byte_buffer[ i++ ];	// LSB tara via 1
	}
	return 1;
}

int SIFRAProtocol::SIFRA_decodeWeightSensibilityCmd( byte *cmd_byte_buffer)
{
	byte i = 0;

	WeightCmd.sensibility = cmd_byte_buffer[i];
	return 1;
}

int SIFRAProtocol::SIFRA_decodeRestartFormulaCmd( byte *cmd_byte_buffer)
{
	byte i = 0;

	g_restartFormulaCmd.type_of_formula = cmd_byte_buffer[i];
	return 1;
}

int SIFRAProtocol::SIFRA_decodeScalesPhaseCmd( byte *cmd_byte_buffer)
{
	byte i = 0;

	g_scalesPhaseCmd.init_end = cmd_byte_buffer[i];
	i++;
	g_scalesPhaseCmd.type_of_phase = cmd_byte_buffer[i];
	
	return 1;
}

/**
* Sends answer to start command, with request parameters
*/
int SIFRAProtocol::sendSIFRAStart()
{
	byte packet[_PKT_START_LENGTH];
	byte i = 0;
	byte num_via = 0;

	packet[i++] = M3300;
	packet[i++] = SIFRA_SET_START_M3300;	// uguale anche per M3200
	
	packet[i++] = (StartCmd.function & 0xFF00) >> 8;	// funzione impostata
	packet[i++] = (StartCmd.function & 0x00FF);
	packet[i++] = 0x00;					// byte nullo previsto dal protocollo
	packet[i++] = StartCmd.support;	// 01

	for( num_via = 0; num_via < NUM_MAX_LINE; num_via++)
	{
		packet[i++] = (StartCmd.peso_spec[ num_via] & 0xFF00) >> 8;
		packet[i++] = (StartCmd.peso_spec[ num_via] & 0x00FF);
	}
	
	for( num_via = 0; num_via < NUM_MAX_LINE; num_via++)
	{
		packet[i++] = (StartCmd.peso_linea[ num_via] & 0xFF00) >> 8;
		packet[i++] = (StartCmd.peso_linea[ num_via] & 0x00FF);
	}

	for( num_via = 0; num_via < NUM_MAX_LINE; num_via++)
	{
		packet[i++] = (StartCmd.cap_linea[ num_via] & 0xFF00) >> 8;
		packet[i++] = (StartCmd.cap_linea[ num_via] & 0x00FF);
	}
	
	packet[i] = stdUartProtocolAbstraction::calculateChecksum(&packet[0], i);
	i++;
	PinEnRS485_DE = 1;
	return stdUartProtocolAbstraction::SendCommand(packet, i, 0);
}

/**
* Sends status command
*/
int SIFRAProtocol::sendSIFRAStatus()
{	
	byte packet[_PKT_STATUS_LENGTH];
	byte i = 0;
	byte num_via = 0;
	
	packet[i++] = M3300;
	packet[i++] = SIFRA_GET_STATUS_M3300; 
	
	packet[i++] = StatusCmd.tot_enabled_lines;
	packet[i++] = StatusCmd.status;
	for(num_via = 0; num_via < (_MAX_LOAD_CHAN_ ); num_via++)				// errore su tutte le vie
	{
		packet[i++] = (StatusCmd.statusChan[num_via] & 0xFF00) >> 8;	
		packet[i++] = (StatusCmd.statusChan[num_via] & 0x00FF);
	}
	packet[i++] = (StatusCmd.next_line & 0xFF00) >> 8;
	packet[i++] = (StatusCmd.next_line & 0x00FF);
	for(num_via = 0; num_via < NUM_MAX_LINE; num_via++)	// peso erogato di tutte le vie
	{
		packet[i++] = ((Via[num_via].peso_erogato + 200) & 0xFF00) >> 8;
		packet[i++] = ((Via[num_via].peso_erogato + 200) & 0x00FF);
	}
	packet[i++] = (byte)((total_weight & 0xFF000000) >> 24);	// peso complessivo sulla cella
	packet[i++] = (byte)((total_weight & 0x00FF0000) >> 16);	// peso complessivo sulla cella
	packet[i++] = (byte)((total_weight & 0x0000FF00) >> 8);	// peso complessivo sulla cella
	packet[i++] = (byte)((total_weight & 0x000000FF));
	for(num_via = 0; num_via < (NUM_MAX_LINE); num_via++)
	{
		packet[i++] = (Via[ num_via ].capacita & 0xFF00) >> 8;
		packet[i++] = (Via[ num_via ].capacita & 0x00FF);
	}
	packet[i++] = StatusCmd.error_monitor;
	packet[i++] = (byte)cpld_CoverPump;
	packet[i++] = (byte)(((StatusCmd.SIFRAComPacketsRx) & 0xFF00) >> 8);
	packet[i++] = (byte)((StatusCmd.SIFRAComPacketsRx) & 0x00FF);
	packet[i++] = (byte)(((StatusCmd.SIFRAComHwErrors) & 0xFF00) >> 8);
	packet[i++] = (byte)((StatusCmd.SIFRAComHwErrors) & 0x00FF);
	packet[i++] = (byte)(((StatusCmd.SIFRAComProtocolErrors) & 0xFF00) >> 8);
	packet[i++] = (byte)((StatusCmd.SIFRAComProtocolErrors) & 0x00FF);
	packet[i++] = (byte)(((StatusCmd.SIFRAComUnknownErrors) & 0xFF00) >> 8);
	packet[i++] = (byte)((StatusCmd.SIFRAComUnknownErrors) & 0x00FF);
	packet[i++] = StatusCmd.error_stop;
	packet[i++] = (byte)(((StatusCmd.variazione_peso) & 0xFF00) >> 8);
	packet[i++] = (byte)((StatusCmd.variazione_peso) & 0x00FF);
	packet[i++] = (byte)(Chan[_ADC1_].AreCalibrate);
	packet[i++] = (byte)(Chan[_ADC2_].AreCalibrate);
	packet[i++] = StatusCmd.test_pinch_done;
	packet[i++] = StatusCmd.statusAirSensors;
	packet[i++] = StatusCmd.state;
	packet[i++] = StatusCmd.prevState;
	packet[i++] = StatusCmd.phase;
	packet[i] = stdUartProtocolAbstraction::calculateChecksum(&packet[0], i);
	i++;
	PinEnRS485_DE = 1;
	return stdUartProtocolAbstraction::SendCommand(packet, i, 0);	 
}

/**
Sends info about firmware and harware version
*/
int SIFRAProtocol::sendSIFRAInfo()
{
	byte packet[_PKT_INFO_LENGTH];
	byte i = 0;
	byte k;
	
	packet[i++] = M3300;
	packet[i++] = SIFRA_GET_INFO_M3300;

	packet[i++] = 0x00;	// byte nullo
	for (k = 0; k < sAppRevisionLength; k++)
	{
		packet[i++] = sAppRevision[k];
	}
	packet[i++] = 0x00;	// byte nullo
	for (k = 0; k < sAppDateLength; k++)
	{
		packet[i++] = sAppDate[k];
	}
	packet[i++] = 0x00;	// byte nullo
	for (k = 0; k < sHwVersionLength; k++)
	{
		packet[i++] = sHwVersion[k];
	}
	packet[i++] = 0x00;	// byte nullo	
	packet[i] = stdUartProtocolAbstraction::calculateChecksum(&packet[0], i);
	i++;
	PinEnRS485_DE = 1;
	return stdUartProtocolAbstraction::SendCommand(packet, i, 0);
}

/**
* Sends debug command with data of Via structure
*/
int SIFRAProtocol::sendSIFRAViaDebug()
{	
	byte packet[_PKT_DBG_VIA_LENGTH];
	byte i = 0;
	int num_via;
	word peso_spec;



	num_via = StatusCmd.next_line;
	if(num_via < 0 || num_via > NUM_MAX_LINE)
	{
		num_via = 0;
	}

	peso_spec = (word)(Via[num_via].peso_spec * 1000.0);
	
	packet[i++] =  M3300;		// NODEID perifierica
	packet[i++] = SIFRA_GET_DEBUG_VIA_M3300;

	packet[i++] = (byte)(((StatusCmd.next_line) & 0xFF00) >> 8);
	packet[i++] = (byte)((StatusCmd.next_line) & 0x00FF);
	packet[i++] = (byte)(Via[num_via].abilitazione);
	packet[i++] = (byte)(Via[num_via].da_eseguire);
	packet[i++] = (byte)(Via[num_via].eseguita);
	packet[i++] = (byte)(Via[num_via].stato_led);
	packet[i++] = (byte)(Via[num_via].stato_luci);
	packet[i++] = (byte)(Via[num_via].fase_luci);

	packet[i++] = (byte)(((Via[num_via].peso_da_erogare) & 0xFF00) >> 8);
	packet[i++] = (byte)((Via[num_via].peso_da_erogare) & 0x00FF);

	packet[i++] = (byte)(((Via[num_via].capacita) & 0xFF00) >> 8);
	packet[i++] = (byte)((Via[num_via].capacita) & 0x00FF);

	packet[i++] = (byte)(((Via[num_via].peso_erogato) & 0xFF00) >> 8);
	packet[i++] = (byte)((Via[num_via].peso_erogato) & 0x00FF);

	packet[i++] = (byte)(((Via[num_via].peso_gia_erogato) & 0xFF00) >> 8);
	packet[i++] = (byte)((Via[num_via].peso_gia_erogato) & 0x00FF);

	packet[i++] = (byte)(((Via[num_via].peso_iniziale) & 0xFF00) >> 8);
	packet[i++] = (byte)((Via[num_via].peso_iniziale) & 0x00FF);

	packet[i++] = (byte)(((Via[num_via].peso_finale) & 0xFF00) >> 8);
	packet[i++] = (byte)((Via[num_via].peso_finale) & 0x00FF);

	packet[i++] = (byte)(((peso_spec) & 0xFF00) >> 8);
	packet[i++] = (byte)((peso_spec) & 0x00FF);

	packet[i++] = g_stateMachine.sampleState;
	packet[i++] = g_stateMachine.calibState;
	packet[i++] = g_stateMachine.saccaState;
	packet[i++] = g_stateMachine.rtState;
	packet[i++] = g_stateMachine.stState;
	packet[i++] = g_stateMachine.pinchState;
	packet[i++] = g_stateMachine.ledState;
	packet[i++] = g_stateMachine.driverState;
	packet[i++] = g_stateMachine.serviceState;
	packet[i++] = g_stateMachine.nonCalaState;
	
	packet[i] = stdUartProtocolAbstraction::calculateChecksum(&packet[0], i);
	i++;
	PinEnRS485_DE = 1;
	
	return stdUartProtocolAbstraction::SendCommand(packet, i, 0);
}

/**
* Sends debug command with data of StartCmd structure
*/
bool SIFRAProtocol::sendSIFRAStartDebug()
{	
	byte packet[_PKT_DBG_START_LENGTH];
	byte i = 0;
	byte num_via;
	
	packet[i++] =  M3300;		// NODEID perifierica
	packet[i++] = SIFRA_GET_DEBUG_START_M3300;
	
	// valore encoder per ogni via, essendo una word lo splitto in 2 byte
	packet[i++] = (byte)(((StartCmd.function) & 0xFF00) >> 8);
	packet[i++] = (byte)((StartCmd.function) & 0x00FF);

	packet[i++] = (byte)(StartCmd.support);

	for(num_via = 0; num_via < (NUM_MAX_LINE ); num_via++)	
	{
		packet[i++] = (byte)(((StartCmd.peso_spec[num_via]) & 0xFF00) >> 8);
		packet[i++] = (byte)((StartCmd.peso_spec[num_via]) & 0x00FF);
	}

	for(num_via = 0; num_via < (NUM_MAX_LINE ); num_via++)	
	{
		packet[i++] = (byte)(((StartCmd.peso_linea[num_via]) & 0xFF00) >> 8);
		packet[i++] = (byte)((StartCmd.peso_linea[num_via]) & 0x00FF);
	}

	for(num_via = 0; num_via < (NUM_MAX_LINE ); num_via++)	
	{
		packet[i++] = (byte)(((StartCmd.cap_linea[num_via]) & 0xFF00) >> 8);
		packet[i++] = (byte)((StartCmd.cap_linea[num_via]) & 0x00FF);
	}
	
	packet[i] = stdUartProtocolAbstraction::calculateChecksum(&packet[0], i);
	i++;
	PinEnRS485_DE = 1;
	return stdUartProtocolAbstraction::SendCommand(packet, i, 0);	 
}

/**
* Sends debug command with status data
*/
bool SIFRAProtocol::sendSIFRAStatusDebug()
{	
	byte packet[_PKT_DBG_STATUS_LENGTH];
	byte i = 0;
	byte num_via;
	
	packet[i++] =  M3300;		// NODEID perifierica
	packet[i++] = SIFRA_GET_DEBUG_STATUS_M3300;

	packet[i++] = (byte)StatusCmd.air_block_en;
	packet[i++] = (byte)StatusCmd.m_FillingOnCourse;
	packet[i++] = StatusCmd.status;
	packet[i++] = StatusCmd.flagsErrorLine;
	packet[i++] = StatusCmd.tot_enabled_lines;

	packet[i++] = (byte)(((StatusCmd.next_line) & 0xFF00) >> 8);
	packet[i++] = (byte)((StatusCmd.next_line) & 0x00FF);

	for(num_via = 0; num_via < (_MAX_LOAD_CHAN_); num_via++)	
	{
		packet[i++] = (byte)(((StatusCmd.statusChan[num_via]) & 0xFF00) >> 8);
		packet[i++] = (byte)((StatusCmd.statusChan[num_via]) & 0x00FF);
	}

	packet[i++] = (byte)(((StatusCmd.SIFRAComUnknownErrors) & 0xFF00) >> 8);
	packet[i++] = (byte)((StatusCmd.SIFRAComUnknownErrors) & 0x00FF);

	packet[i++] = StatusCmd.error_monitor;

	packet[i++] = StatusDebug.sifraStatus;
	packet[i++] = StatusDebug.sifraPrevStatus;
	packet[i++] = StatusDebug.sifraPhase;

	packet[i++] = (byte)(((StatusDebug.peso_current_time_int) & 0xFF00) >> 8);
	packet[i++] = (byte)((StatusDebug.peso_current_time_int) & 0x00FF);

	packet[i++] = (byte)(((StatusDebug.peso_current_time_word) & 0xFF00) >> 8);
	packet[i++] = (byte)((StatusDebug.peso_current_time_word) & 0x00FF);

	packet[i++] = StatusDebug.sacca_manager_state;

	packet[i++] = (byte)(((StatusDebug.peso_parziale) & 0xFF00) >> 8);
	packet[i++] = (byte)((StatusDebug.peso_parziale) & 0x00FF);

	packet[i++] = (byte)(((StatusDebug.sampleMngr_cycles) & 0xFF00) >> 8);
	packet[i++] = (byte)((StatusDebug.sampleMngr_cycles) & 0x00FF);
	
	packet[i++] = (byte)(((StatusDebug.weight) & 0xFF00) >> 8);
	packet[i++] = (byte)((StatusDebug.weight) & 0x00FF);

	packet[i++] = (byte)(((StatusDebug.enc_value) & 0xFF00) >> 8);
	packet[i++] = (byte)((StatusDebug.enc_value) & 0x00FF);

	packet[i++] = (byte)(((StatusDebug.enc_value_control) & 0xFF00) >> 8);
	packet[i++] = (byte)((StatusDebug.enc_value_control) & 0x00FF);

	packet[i++] = (byte)(((StatusDebug.enc_movement) & 0xFF00) >> 8);
	packet[i++] = (byte)((StatusDebug.enc_movement) & 0x00FF);

	packet[i++] = (byte)(((StatusDebug.enc_up_limit) & 0xFF00) >> 8);
	packet[i++] = (byte)((StatusDebug.enc_up_limit) & 0x00FF);

	packet[i++] = (byte)(((StatusDebug.enc_low_limit) & 0xFF00) >> 8);
	packet[i++] = (byte)((StatusDebug.enc_low_limit) & 0x00FF);
	
	for(num_via = 0; num_via < (NUM_MAX_LINE ); num_via++)	
	{
		packet[i++] = StatusDebug.air_state[num_via];
	}

	packet[i++] = StatusDebug.cover_pump;
	packet[i++] = StatusDebug.error_control;

	packet[i++] = (byte)(((StatusDebug.init_weight) & 0xFF00) >> 8);
	packet[i++] = (byte)((StatusDebug.init_weight) & 0x00FF);
	
	packet[i] = stdUartProtocolAbstraction::calculateChecksum(&packet[0], i);
	i++;
	PinEnRS485_DE = 1;
	return stdUartProtocolAbstraction::SendCommand(packet, i, 0);	 
}

/**
* Sends debug command with data of bk and fp structure (data in RAM tamponata)
*/
bool SIFRAProtocol::sendSIFRARamDebug()
{	
	byte packet[_PKT_DBG_BACKUP_RAM_LENGTH];
	byte i = 0;
	byte num_via;

	word paramEncoder;
	word paramTolleranza;

	paramEncoder = (word)(structEnc.param_encoder * 10.0);
	paramTolleranza = (word)(structEnc.tolleranza * 100.0);
	
	packet[i++] =  M3300;		// NODEID perifierica
	packet[i++] = SIFRA_GET_DEBUG_BACKUP_RAM_M3300;
	
	packet[i++] = (byte)(((bk.nextLine) & 0xFF00) >> 8);
	packet[i++] = (byte)((bk.nextLine) & 0x00FF);

	packet[i++] = (byte)(bk.num_vie_da_eseguire);
	packet[i++] = (byte)(bk.vie_da_eseguire);
	packet[i++] = (byte)(bk.vie_eseguite);

	packet[i++] = (byte)(((bk.peso_stop) & 0xFF000000) >> 24);
	packet[i++] = (byte)(((bk.peso_stop) & 0x00FF0000)>> 16);
	packet[i++] = (byte)(((bk.peso_stop) & 0x0000FF00) >> 8);
	packet[i++] = (byte)((bk.peso_stop) & 0x000000FF);

	for(num_via = 0; num_via < (NUM_MAX_LINE ); num_via++)	
	{
		packet[i++] = bk.abilitazione[num_via];
	}

	for(num_via = 0; num_via < (NUM_MAX_LINE ); num_via++)	
	{
		packet[i++] = (byte)(((bk.peso_erogato[num_via]) & 0xFF00) >> 8);
		packet[i++] = (byte)((bk.peso_erogato[num_via]) & 0x00FF);
	}

	for(num_via = 0; num_via < (NUM_MAX_LINE ); num_via++)	
	{
		packet[i++] = (byte)(((bk.capacita[num_via]) & 0xFF00) >> 8);
		packet[i++] = (byte)((bk.capacita[num_via]) & 0x00FF);
	}

	packet[i++] = (byte)(((paramEncoder) & 0xFF00) >> 8);
	packet[i++] = (byte)((paramEncoder) & 0x00FF);

	packet[i++] = (byte)(((paramTolleranza) & 0xFF00) >> 8);
	packet[i++] = (byte)((paramTolleranza) & 0x00FF);

	packet[i++] = (byte)(structEnc.restart);
	
	packet[i] = stdUartProtocolAbstraction::calculateChecksum(&packet[0], i);
	i++;
	PinEnRS485_DE = 1;
	return stdUartProtocolAbstraction::SendCommand(packet, i, 0);	 
}

/**
Generic Acknowledge to everything cmd.

The packet is builded as follow:
- Hardware ID - byte
- CMD (opcode) received - byte
- checksum - byte
-null byte

@return the number of byte transmitted. if 0 no bytes were transmitted.
*/
int SIFRAProtocol::sendAcknowledge()	//int SIFRAProtocol::sendAcknowledge(byte ack)
{
	byte packet[_PKT_ACK_LENGTH];
	int i = 0;
	
	packet[i++] = getLocalNodeID();
	packet[i++] = last_opCode;		
	packet[i] = stdUartProtocolAbstraction::calculateChecksum(&packet[0], i);
	i++;
	PinEnRS485_DE = 1;
	return stdUartProtocolAbstraction::SendCommand(packet, i, 0);
}

/**
* Calculates phisical load in M3300.
* Return load with cents of ml for the calculation, while the load for M3000 is with tenths of ml.
*/
long SIFRAProtocol::sendSIFRALoadSamples( float loadsystem)
{
	
#ifdef _PESI_IN_GRAMMI

	float load = loadsystem * _CONV_IN_DECIMI;
	// calcolo il valore da passare al sistema M3300, dato intero (32bit)
	long calc_sample = (long)(load); // prendo decimi e centesimi	
	if((loadsystem*100 - calc_sample * 10) > 5) calc_sample++;	// arrotondamento all'intero + vicino

	#ifdef _SMOOTHING_FILT_
		// routine per evitare che gli arrotondamenti facciano muovere il valore dle pesp calcolato, a fronte della variaizone di uno solo dei due ADC e di
		// un solo punto di adc
		static float lastoldvalue = 0;
		float diffvalue = loadsystem - lastoldvalue;
		if((diffvalue < 0.09) && (diffvalue > -0.09))	// se la differena è inferiore ad un decimo
			loadsystem = lastoldvalue;				// allora non aggiorno il valore perchè le approssimazioni successive genererebbero un delta
		else
			lastoldvalue = loadsystem;			// altrimenti aggiorno
	#endif
	// ora invece lavoro sul dato da passare a tastiera, in 16bit
	if((load + _OFFSET_SU_ZERO) > 0x0000FFFE)
		total_weight = 0xFFFE;
	else
		total_weight = (long)(load + _OFFSET_SU_ZERO);
	
#else		// peso espresso in decimi di grammo
	// calcolo il valore da passare al sistema M3300, dato intero (32bit)
	long calc_sample = (long)(loadsystem);
	if((loadsystem*10 - calc_sample * 10) > 5)
		calc_sample++;	// arrotondamento all'intero + vicino
	// ora invece lavoro sul dato da passare a tastiera, in 16bit
	if((calc_sample + _OFFSET_SU_ZERO) > 0x0000FFFE)
		total_weight = 0xFFFE;
	else
		total_weight = calc_sample + _OFFSET_SU_ZERO;
	
#endif

	return calc_sample;
}

// mette in spedizione (cioè nella fifo delle cose da re-inviare) il comando ricevuto
int SIFRAProtocol::SIFRA_getInfoCmdDecode()
{
	sendMsg(SIFRAMsg_infoAsked);
	return 1;
}

// mette in spedizione (cioè nella fifo delle cose da re-inviare) il comando ricevuto
int SIFRAProtocol::SIFRA_getDebugViaCmdDecode()
{
	sendMsg(SIFRAMsg_debugViaAsked);
	return 1;
}

// mette in spedizione (cioè nella fifo delle cose da re-inviare) il comando ricevuto
int SIFRAProtocol::SIFRA_getDebugStartCmdDecode()
{
	sendMsg(SIFRAMsg_debugStartAsked);
	return 1;
}

// mette in spedizione (cioè nella fifo delle cose da re-inviare) il comando ricevuto
int SIFRAProtocol::SIFRA_getDebugStatusCmdDecode()
{
	sendMsg(SIFRAMsg_debugStatusAsked);
	return 1;
}

// mette in spedizione (cioè nella fifo delle cose da re-inviare) il comando ricevuto
int SIFRAProtocol::SIFRA_getDebugRamCmdDecode()
{
	sendMsg(SIFRAMsg_debugRamAsked);
	return 1;
}

/**
* Sends to protocol the set weight sensibility command
*/
int SIFRAProtocol::SIFRA_setWeightSensibility()
{
	sendMsg(SIFRAMsg_SetWeightSensibility);
	return 1;
}

int SIFRAProtocol::SIFRA_restartWeightCoverControl()
{
	sendMsg(SIFRAMsg_restartWeightCoverControl);
	return 1;
}

/**
* Sends to protocol the set weight sensibility command
*/
int SIFRAProtocol::SIFRA_restartFormula()
{
	sendMsg(SIFRAMsg_restartFormula);
	return 1;
}

/**
* Sends to protocol the set weight sensibility command
*/
int SIFRAProtocol::SIFRA_setScalesPhase()
{
	sendMsg(SIFRAMsg_scalesPhase);
	return 1;
}

/**
* Sends to protocol the set of scales not calibrated
*/
int SIFRAProtocol::SIFRA_setScalesNotCalib()
{
	sendMsg(SIFRAMsg_scalesNotCalib);
	return 1;
}

/**
Sends to protocol the overall stop command
*/
int SIFRAProtocol::SIFRA_set_Stop_All()
{
	sendMsg(SIFRAMsg_StopAll);
	m_numStopPkts++;
	return 1;
}
/**
Sends to protocol the status ask command
*/
int SIFRAProtocol::SIFRA_getStatusCmdDecode()
{
	sendMsg(SIFRAMsg_statusAsked);
	return 1;
}
/**
Sends to protocol the start command
*/
int SIFRAProtocol::SIFRA_setStart()
{
	sendMsg(SIFRAMsg_setStart);
	m_numStartPkts++;
	return 1;
}
/**
Sends to protocol the cell8 and cell15 zero calibration command
*/
int SIFRAProtocol::SIFRA_setZeroVerification()
{
	sendMsg(SIFRAMsg_setZeroVerification);
	return 1;
}
/**
* Sends to protocol the command used to reset calibration data saved in EEPROM
*/
int SIFRAProtocol::SIFRA_setResetCalib()
{
	sendMsg(SIFRAMsg_resetCalib);
	return 1;
}
/**
Sends to protocol the cell6 and cellH2O zero calibration command
*/
int SIFRAProtocol::SIFRA_setZeroCell8_Cell15()
{
	sendMsg(SIFRAMsg_setZeroCalib);
	return 1;
}
/**
Sends to protocol the cell6 (m3200) or cell8(M3300) gain calibration command
*/
int SIFRAProtocol::SIFRA_setGainCell8()
{
	sendMsg(SIFRAMsg_setGain8);
	return 1;
}
/**
Sends to protocol the cellH2O (m3200) or cell15(M3300) gain calibration command
*/
int SIFRAProtocol::SIFRA_setGainCell15()
{
	sendMsg(SIFRAMsg_setGain15);
	return 1;
}
/**
Sends to protocol the jump-to-loader command
*/
int SIFRAProtocol::SIFRA_setJumpToLoader()
{
	sendMsg(SIFRAMsg_jumpToLoader);
	return 1;
}

/**
Manager of the command board jump to loader.

The command is received and decoded from the class AcqHrProtocol which sends a message to the board manager.
@return always 1
*/
int SIFRAProtocol::jumpToLoaderHandler()
{
	DecTimer wait;

	wait.Preset(500);
	while(!wait.Match());

	while (GetNumBytesInTransmissionBuffer() > 0)
	{
		wait.Preset(100);
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
	
	fp = (void (*)()) LOADER_FLASH_START_ADDRESS;	// SALTA IN LOADER
    	(*fp)();
		
	return 1;
}

/**
Reset application and restar system automatically
@return always 1
*/
int SIFRAProtocol::resetApplicationHandler()
{
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	asm("nop");
	
	restart_command = 0xA5;	// salvo in ram tamponata il comando di reset ricevuto da tastiera
	
	fp = (void (*)()) __APPLICATION_START_ADDRESS__;	// inizio dell'applicone
    	(*fp)();
		
	return 1;
}

void SIFRAProtocol::setLocalNodeID(byte node)
{
	m_localNodeID = node;
}

byte SIFRAProtocol::getLocalNodeID()
{
	return m_localNodeID;
}

void SIFRAProtocol::setLastRemoteNodeID(byte node)
{ 
	m_lastRemoteNodeID = node;
}

byte SIFRAProtocol::getLastRemoteNodeID()
{ 
	return m_lastRemoteNodeID;
}

/**
checks opcode just received in base of different hardware version. Then indicates how much bytes
have to be recevied and analysed
*/
bool SIFRAProtocol::checkOpCode (byte data, int NodeID, int &numDataToReceive)
{
	// se valido, indico il num di byte da ricevere. 
	// caso di pacchetto di comandi vari (caso di start)
	// casi di comandi singoli seguiti subito dal ckecksum
	numDataToReceive = 0;
	if(data == SIFRA_SET_STOP_ALL)
	{
		numDataToReceive = 0;	// è il comando di stop a tutto
		return True;
	}
	switch( NodeID)
	{	
		case M3300:
			switch(data)
			{
				case SIFRA_SET_START_M3300:	// messaggio di START con gli eventuali parametri
					numDataToReceive = _PKT_START_DATA_M3300;
					return True;
					break;
				case SIFRA_SET_WEIGHT_SENSIBILITY_M3300:
				case SIFRA_RESTART_FORMULA_M3300:
					numDataToReceive = _PKT_WEIGHT_DATA_M3300;
					return True;
					break;
				case SIFRA_GET_SCALES_STATE_M3300:
					numDataToReceive = _PKT_SCALES_DATA_M3300;
					return True;
					break;
				case SIFRA_GET_STATUS_M3300:					// messaggio richiesta stato
				case SIFRA_GET_DEBUG_VIA_M3300:				// messaggio richiesta debug struttura via
				case SIFRA_GET_DEBUG_START_M3300:			// messaggio richiesta debug struttura start
				case SIFRA_GET_DEBUG_STATUS_M3300:			// messaggio richiesta debug struttura status
				case SIFRA_GET_DEBUG_BACKUP_RAM_M3300:		// messaggio richiesta debug struttura ram tamponata
				case SIFRA_ERROR_RESTART_CONTROL:			// messaggio usato per riprendere controllo su peso massimo
				case SIFRA_SETZERO_VERIFICATION:				// comando gestire calibrazione di zero ad inizio di verifica bilance
				case SIFRA_RESET_CALIB_PARAMS:				// comando usato per resettare i parametri di calibrazione
				case SIFRA_SETZERO_CELL8_CELL15:				// calib. zero cella 8-15
				case SIFRA_SET_CAL_CELL8:					// guadagno cella 8
				case SIFRA_SET_CAL_CELL15:					// guadagno cella 15
				case SIFRA_GET_INFO_M3300:					// richiesta versione HW e FW
				case SIFRA_GET_SCALES_NOT_CALIB_M3300:		// comando usato per impostare la non calibrazione delle celle di carico
				case SIFRA_JUMP_TO_LOADER:					// salto in loader per riprogrammazione
				case SIFRA_M3000_STOP:
					numDataToReceive = 0;
					return True;
					break;
				default:
					break;
			}
			break;

		case M3100:
		case M3800:
		default:
			break;
	}
	
	numDataToReceive = 0;
	
	return False;
	
}

bool SIFRAProtocol::checksum(byte receivedChk, byte opCode, byte nodeID)
{	// modificato per adattare al protocollo siframix esistente
	int i;
	byte myChk = 0;

	myChk += nodeID;
	myChk += opCode;
	
	for ( i = 0; i < stdUartProtocolAbstraction::getNumDataReceived(); i++)
	{
		myChk += stdUartProtocolAbstraction::bufferLastDataReceived()[i];
	}
	myChk = ~myChk;	// negazione bit a bit
	if (myChk == receivedChk)
	{
		return True;
	}
	return false;
}

/**
Sends message just receveid from communication state machine to the controll state machine 
*/
int SIFRAProtocol::sendMsg(int i)
{
	if(!m_message.full())
	{
		m_message.push_back(i);
		return 0;
	}	
	return -1;
}

/**
Gets message from received message fifo
*/
int SIFRAProtocol::getMsg()
{
	int msg = SIFRAMsg_noMsg;
	if(!m_message.empty())
	{
		m_message.pop_front(msg);
	}
	return msg;
}
