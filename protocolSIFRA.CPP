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

STRUCTSTARTCMD StartCmd;
max_weight_error_rst_t g_rstMaxWeight;
restart_control_cmd_t g_restartControlCmd;
structScalesPhaseCmd_t g_scalesPhaseCmd;
structScalesNotCalibCmd_t g_scalesNotCalibCmd;
STRUCTSTATUSCMD StatusCmd;
STRUCTDEBUGCMD DebugCmd;
LINES_Status Via[ NUM_MAX_LINE];
machineStatus_t g_stateMachine;
BK_struct bk;
FP_STRUCT g_encStruct[NUM_MAX_LINE];
Restart_t g_restart;

word g_adc_real_value[ _MAX_LOAD_CHAN_ ];

extern const char sAppName[];
extern const char sAppRevision[];
extern const char sAppDate[];
extern const char sHwVersion[];

extern const int sAppNameLength;
extern const int sAppRevisionLength;
extern const int sAppDateLength;
extern const int sHwVersionLength;
extern ENCODER_Motor_Status	 MotorStatus[];
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
	setNumOfEnabledLines(0);	// azzeri numero delle vie abilitate
	m_show_adcvalue = False;
	m_lastRemoteNodeID = 0;
	m_localNodeID = 0;	
	m_lastHwError = SIFRAUart_Hw_NoError;
	
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
		// tutti i moduli acquisiscono il messaggio e solo alla fine possono decidere se è destinato a loro e quindi da analizzare oppure se è da scartare			
		case stdUart_waitDeviceID:	// parto da qui
			if((data == M3100) || (data == STOP_ALL))		// IDENTIFICATIVO MODULO
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
					protocol->setError(stdUart_UnknownOpcode, True);
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
			((SIFRAProtocol*)protocol)->setLastRemoteNodeID(lastRemoteID);

			if((((SIFRAProtocol*)protocol)->getLastRemoteNodeID() == ((SIFRAProtocol*)protocol)->getLocalNodeID()) ||
				(((SIFRAProtocol*)protocol)->getLastRemoteNodeID() == STOP_ALL))
			{
				protocol->setNumDataReceived(protocol->getPacketLength());
				if(((SIFRAProtocol*)protocol)->checksum(data, (byte)opCode, (byte)lastRemoteID))		// devo sommare anche il primo char, lastRemoteID
				{
					protocol->setReceivedOpCode(opCode);
					protocol->newPacketReceived();
					StatusCmd.commPacketsRx = protocol->getPacketsReceived();
				}
				else
				{
					protocol->setError(stdUart_ChecksumError, True);
					protocol->setReceiverState(stdUart_waitDeviceID);
					PinEnRS485_DE = 0;
					protocol->rstBytesReceived();
					return False;
				}
			}
			else		// non è messaggio per me, resetta buffer, ignora opcode e ritorna allo stato iniziale
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
			protocol->setError(stdUart_protocolStateUnknown, True);
			break;
	}
	
	protocol->newByteReceived();
	
	return False;
}


void SIFRAProtocol::Manager()
{
	int error, opCode;

	stdUartProtocolAbstraction::Manager();	// analisi dei dati ricevuti e verifica dei flag di errore di overflow

	//---- Remapping stdUartProtocolAbstraction error ----
	error = stdUartProtocolAbstraction::popError();			// se presente, preleva l'errore nella fifo degli errori
	while (error != stdUart_NoError)		// verifica TUTTI gli eventuali errori presenti nella fifo degli errori
	{
		switch(error)
		{
			//  errori hardware
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
			//  errori di protocollo
			case stdUart_UnknownOpcode:
				sendMsg(SIFRAMsg_opcodeUnknown);
				break;
			case stdUart_protocolStateUnknown:
				sendMsg(SIFRAMsg_protocolStateUnknown);
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
			//  errori sconosciuti
			default:
				sendMsg(SIFRAMsg_unknownError);
				break;

		}
		error = stdUartProtocolAbstraction::popError();
	}
	
	//------- Checking for packets receiving ----------------
	if(stdUartProtocolAbstraction::popPacketReceived(opCode))	// se c'è un comando in coda, lo rispedicse per conferma
	{
		switch (opCode)
		{
			case SIFRA_SET_STOP_ALL:			// 0x00
				SIFRA_set_Stop_All();			// handle comando stop generale			
				break;
			case SIFRA_M3000_STOP:
				SIFRA_set_Stop_All();
				last_opCode = opCode;
				sendAcknowledge();
				break;
			case SIFRA_GET_STATUS_M3100:
				SIFRA_getStatusCmdDecode();	// status
				break;
			case SIFRA_NEW_LINES_M3100:
				SIFRA_setLineToIdentify();
				last_opCode = opCode;
				break;
			case SIFRA_RESET_MAX_WEIGHT:
				SIFRA_decodeResetMaxWeight(stdUartProtocolAbstraction::m_lastDataReceived);
				SIFRA_resetMaxWeightError();
				last_opCode = opCode;
				break;
			case SIFRA_RESTART_CONTROL:
				SIFRA_decodeRestartControlCmd(stdUartProtocolAbstraction::m_lastDataReceived);
				SIFRA_restartControl();
				last_opCode = opCode;
				break;
			case SIFRA_GET_SCALES_STATE_M3100:
				SIFRA_setScalesPhase();
				last_opCode = opCode;
				SIFRA_decodeScalesPhaseCmd(stdUartProtocolAbstraction::m_lastDataReceived);
				break;
			case SIFRA_GET_CELLS_NOT_CALIB:
				SIFRA_setScalesNotCalib();
				last_opCode = opCode;
				SIFRA_decodeScalesNotCalibCmd(stdUartProtocolAbstraction::m_lastDataReceived);
				break;
			case SIFRA_SET_START_M3100:
				SIFRA_setStart();
				SIFRA_decodeStartCmd( stdUartProtocolAbstraction::m_lastDataReceived);
				break;
			case SIFRA_GET_INFO_M3100:
				SIFRA_getInfoCmdDecode();		// ora non usato: versioni hw e fw	
				break;
			case SIFRA_GET_DEBUG_M3100:
				SIFRA_getDebugCmdDecode();	
				break;
			case SIFRA_GET_DEBUG_VIE_1_3_M3100:
				SIFRA_getDebugVie_1_3_CmdDecode();	
				break;
			case SIFRA_GET_DEBUG_VIE_4_5_M3100:
				SIFRA_getDebugVie_4_5_CmdDecode();	
				break;
			case SIFRA_GET_DEBUG_START_M3100:
				SIFRA_getDebugStartCmdDecode();	
				break;
			case SIFRA_GET_DEBUG_RAM_M3100:
				SIFRA_getDebugRamCmdDecode();
				break;
            		case SIFRA_ZERO_CELLS_M3100:
		 		SIFRA_zeroLoadCmdDecode();
				last_opCode = opCode;	// per la risposta
				break;
			case SIFRA_SETZERO_CELL1:
				SIFRA_setZeroCell1();
				last_opCode = opCode;	// per la risposta
				break;
			case SIFRA_SETZERO_CELL2:
				SIFRA_setZeroCell2();
				last_opCode = opCode;	// per la risposta
				break;
			case SIFRA_SETZERO_CELL3:
				SIFRA_setZeroCell3();
				last_opCode = opCode;	// per la risposta
				break;
			case SIFRA_SETZERO_CELL4:
				SIFRA_setZeroCell4();
				last_opCode = opCode;	// per la risposta
				break;
			case SIFRA_SETZERO_CELL5:
				SIFRA_setZeroCell5();
				last_opCode = opCode;	// per la risposta
				break;
			case SIFRA_SET_CAL_CELL1:
				SIFRA_setGainCell1();
				last_opCode = opCode;	// per la risposta
				break;
			case SIFRA_SET_CAL_CELL2:
				SIFRA_setGainCell2();
				last_opCode = opCode;	// per la risposta
				break;
			case SIFRA_SET_CAL_CELL3:
				SIFRA_setGainCell3();
				last_opCode = opCode;	// per la risposta
				break;
			case SIFRA_SET_CAL_CELL4:
				SIFRA_setGainCell4();
				last_opCode = opCode;	// per la risposta
				break;
			case SIFRA_SET_CAL_CELL5:
				SIFRA_setGainCell5();
				last_opCode = opCode;	// per la risposta
				break;
			case SIFRA_RESET_CALIB_PARAMS:
				last_opCode = opCode;	// per la risposta
				SIFRA_setResetCalib();
				break;
			case SIFRA_JUMP_TO_LOADER:
				SIFRA_setJumpToLoader();
				last_opCode = opCode;	// per la risposta
				break;
			case E_SIFRA_LOOP_CMD:
				SIFRA_setLoopApplication();
				last_opCode = opCode;	// per la risposta
				break;
			default:
				StatusCmd.status = STATO_ERRORE;
				StatusCmd.errors = ERR_SYSTEM;
				StatusCmd.log_error = ERR_CASE_OPCODE_UNKNOWN;
				break;
		}
	}
}

int SIFRAProtocol::SIFRA_decodeStartCmd( byte *cmd_byte_buffer)
{
	byte i = 0;
	byte num_line;

	StartCmd.function = ( 0x00 | cmd_byte_buffer[ i++ ]) << 8;	// MSB peso via 1
	StartCmd.function |= cmd_byte_buffer[ i++ ];		// POS_FUNC_IN_CMD = 1
	i++;		// byte nullo
	StartCmd.support = cmd_byte_buffer[ i++ ];		// POS_SUPP_IN_CMD = 2
	for( num_line = 0; num_line < NUM_MAX_LINE; num_line++)
	{
		StartCmd.peso_linea[ num_line ] = ( 0x00 | cmd_byte_buffer[ i++ ]) << 8;	// MSB peso via 1
		StartCmd.peso_linea[ num_line ] |= cmd_byte_buffer[ i++ ];	// LSB peso via 1
	}
	for( num_line = 0; num_line < NUM_MAX_LINE; num_line++)
	{
		StartCmd.tara_linea[ num_line ] = ( 0x00 | cmd_byte_buffer[ i++ ]) << 8;	// MSB tara via 1
		StartCmd.tara_linea[ num_line ] |= cmd_byte_buffer[ i++ ];	// LSB tara via 1
	}
	for( num_line = 0; num_line < NUM_MAX_LINE; num_line++)
	{
		StartCmd.peso_spec[ num_line ] = ( 0x00 | cmd_byte_buffer[ i++ ]) << 8;	// MSB peso specifico via 1
		StartCmd.peso_spec[ num_line ] |= cmd_byte_buffer[ i++ ];	// LSB peso specifico via 1
	}
	return 1;
}

int SIFRAProtocol::SIFRA_decodeResetMaxWeight( byte *cmd_byte_buffer)
{
	byte i = 0;

	g_rstMaxWeight.lines = cmd_byte_buffer[i];

	return 1;
}

int SIFRAProtocol::SIFRA_decodeRestartControlCmd( byte *cmd_byte_buffer)
{
	byte i = 0;

	g_restartControlCmd.type_of_formula = cmd_byte_buffer[i];

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

int SIFRAProtocol::SIFRA_decodeScalesNotCalibCmd( byte *cmd_byte_buffer)
{
	byte i = 0;

	g_scalesNotCalibCmd.scales_not_calib = cmd_byte_buffer[i];
	
	return 1;
}

/**
Sends answer to start command, with request parameters
*/
int SIFRAProtocol::sendSIFRAStart()
{
	byte packet[_PKT_START_LENGTH];
	byte i = 0;
	byte num_via = 0;
	
	packet[i++] = M3100;		// NODEID perifierica
	packet[i++] = SIFRA_SET_START_M3100;
	
	packet[i++] = (StartCmd.function & 0xFF00) >> 8;	// funzione impostata
	packet[i++] = (StartCmd.function & 0x00FF);
	packet[i++] = 0x00;	// byte nullo previsto dal protocollo
	packet[i++] = StartCmd.support;	// 01
	
	for( num_via = 0; num_via < NUM_MAX_LINE; num_via++)
	{
		packet[i++] = (StartCmd.peso_linea[ num_via] & 0xFF00) >> 8;
		packet[i++] = (StartCmd.peso_linea[ num_via] & 0x00FF);
	}

	for( num_via = 0; num_via < NUM_MAX_LINE; num_via++)
	{
		packet[i++] = (StartCmd.tara_linea[ num_via] & 0xFF00) >> 8;
		packet[i++] = (StartCmd.tara_linea[ num_via] & 0x00FF);
	}
	
	for( num_via = 0; num_via < NUM_MAX_LINE; num_via++)
	{
		packet[i++] = (StartCmd.peso_spec[ num_via] & 0xFF00) >> 8;
		packet[i++] = (StartCmd.peso_spec[ num_via] & 0x00FF);
	}
	
	packet[i] = stdUartProtocolAbstraction::calculateChecksum(&packet[0], i );
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
	
	packet[i++] = M3100;		// NODEID perifierica
	packet[i++] = SIFRA_GET_STATUS_M3100;

	packet[i++] = g_restart.tipo_linea;
	packet[i++] = StatusCmd.status;	// STATO		
	// indicazione tipo di errore sulle vie: 5 per M3100
	for( num_via = 0; num_via < _MAX_LOAD_CHAN_; num_via++)	// errore su tutte le vie
	{
		packet[i++] = (byte)((StatusCmd.statusChan[num_via] & 0xFF00) >> 8);	
		packet[i++] = (byte)(StatusCmd.statusChan[num_via] & 0x00FF);		
	}
	
	// indicazione peso erogato per ogni via: 5 per M3100
	if(m_show_adcvalue == True)	// debug adc celle
	{
		// nei campi peso erogato vengono mostrati i valori di ADC letti sulle 5 vie
		for( num_via = 0; num_via < NUM_MAX_LINE; num_via++)	// peso erogato di tutte le vie
		{	
			packet[i++] = (byte)((g_adc_real_value[num_via] & 0xFF00) >> 8);
			packet[i++] = (byte)((g_adc_real_value[num_via] & 0x00FF));
		}
	}
	else 
	{	
		for( num_via = 0; num_via < NUM_MAX_LINE; num_via++)	// peso erogato di tutte le vie
		{	// invece che 20g sommo 1kg
			packet[i++] = (byte)(((Via[num_via].peso_erogato + 200) & 0xFF00) >> 8);
			packet[i++] = (byte)(((Via[num_via].peso_erogato + 200) & 0x00FF));
		}
	}
	// indicazione del peso attuale: uno per ogni cella nell'M3100
	for( num_via = 0; num_via < (NUM_MAX_LINE); num_via++)	// peso erogato di tutte le vie
	{
		packet[i++] = (byte)((loadchan[num_via] & 0xFF000000) >> 24);	// peso complessivo sulla cella
		packet[i++] = (byte)((loadchan[num_via] & 0x00FF0000) >> 16);	// peso complessivo sulla cella
		packet[i++] = (byte)((loadchan[num_via] & 0x0000FF00) >> 8);	// peso complessivo sulla cella
		packet[i++] = (byte)((loadchan[num_via] & 0x000000FF));
	}
	packet[i++] = (byte)StatusCmd.cover_open;
	packet[i++] = (byte)((StatusCmd.errors & 0xFF00) >> 8);
	packet[i++] = (byte)(StatusCmd.errors & 0x00FF);
	packet[i++] = StatusCmd.log_error;
	packet[i++] = (byte)(((StatusCmd.commPacketsRx) & 0xFF00) >> 8);
	packet[i++] = (byte)((StatusCmd.commPacketsRx) & 0x00FF);
	packet[i++] = (byte)(((StatusCmd.commErrorHw) & 0xFF00) >> 8);
	packet[i++] = (byte)((StatusCmd.commErrorHw) & 0x00FF);
	packet[i++] = (byte)(((StatusCmd.commErrorProtocol) & 0xFF00) >> 8);
	packet[i++] = (byte)((StatusCmd.commErrorProtocol) & 0x00FF);
	packet[i++] = (byte)(((StatusCmd.commErrorUnknown) & 0xFF00) >> 8);
	packet[i++] = (byte)((StatusCmd.commErrorUnknown) & 0x00FF);
	for( num_via = 0; num_via < NUM_MAX_LINE; num_via++)
	{
		packet[i++] = StatusCmd.error_stop[num_via];
	}
	for( num_via = 0; num_via < NUM_MAX_LINE; num_via++)
	{
		packet[i++] = (byte)((Via[num_via].variazione_peso_stop & 0xFF00) >> 8);
		packet[i++] = (byte)(Via[num_via].variazione_peso_stop & 0x00FF);
	}
	for( num_via = 0; num_via < NUM_MAX_LINE; num_via++)
	{
		packet[i++] = (byte)Chan[num_via].AreCalibrate;
	}
	packet[i++] = StatusCmd.state;
	packet[i++] = StatusCmd.prevState;
	packet[i++] = StatusCmd.phase;
	packet[i] = stdUartProtocolAbstraction::calculateChecksum(&packet[0], i );
	i++;
	PinEnRS485_DE = 1;	// abilito il tranceiver sulla rete 422
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
	
	packet[i++] =  M3100;		// NODEID perifierica

	packet[i++] = SIFRA_GET_INFO_M3100;

	packet[i++] = 0x00;	// byte nullo
	for (k = 0; k < sAppRevisionLength; k++)
	{
		packet[i++] = sAppRevision[k];		// 8 bytes
	}
	packet[i++] = 0x00;	// byte nullo
	for (k = 0; k < sAppDateLength; k++)
	{
		packet[i++] = sAppDate[k];		// 8 bytes
	}
	packet[i++] = 0x00;	// byte nullo
	for (k = 0; k < sHwVersionLength; k++)
	{
		packet[i++] = sHwVersion[k];		// 8 bytes
	}
	packet[i++] = 0x00;	// byte nullo	
	packet[i] = stdUartProtocolAbstraction::calculateChecksum(&packet[0], i );
	i++;
	PinEnRS485_DE = 1;
	return stdUartProtocolAbstraction::SendCommand(packet, i, 0);
}

/**
* Sends status command
*/
int SIFRAProtocol::sendSIFRADebug()
{	
	byte packet[_PKT_DEBUG_LENGTH];		//pacchetto di _PKT_DEBUG_LENGTH bytes
	byte i = 0;
	byte num_via;
	
	packet[i++] =  M3100;		// NODEID perifierica
	packet[i++] = SIFRA_GET_DEBUG_M3100;
	
	for(num_via = 0; num_via < (NUM_MAX_LINE ); num_via++)	
	{
		packet[i++] = (byte)(((DebugCmd.peso_init[num_via]) & 0xFF00) >> 8);
		packet[i++] = (byte)((DebugCmd.peso_init[num_via]) & 0x00FF);
		packet[i++] = (byte)(((DebugCmd.peso_real_time[num_via]) & 0xFF00) >> 8);
		packet[i++] = (byte)((DebugCmd.peso_real_time[num_via]) & 0x00FF);
		packet[i++] = (byte)(((DebugCmd.enc_value[num_via]) & 0xFF00) >> 8);
		packet[i++] = (byte)((DebugCmd.enc_value[num_via]) & 0x00FF);
		packet[i++] = (byte)(((DebugCmd.enc_value_control[num_via]) & 0xFF00) >> 8);
		packet[i++] = (byte)((DebugCmd.enc_value_control[num_via]) & 0x00FF);
		packet[i++] = (byte)(((DebugCmd.enc_movement[num_via]) & 0xFF00) >> 8);
		packet[i++] = (byte)((DebugCmd.enc_movement[num_via]) & 0x00FF);
		packet[i++] = (byte)(((DebugCmd.enc_upper_limit[num_via]) & 0xFF00) >> 8);
		packet[i++] = (byte)((DebugCmd.enc_upper_limit[num_via]) & 0x00FF);
		packet[i++] = (byte)(((DebugCmd.enc_lower_limit[num_via]) & 0xFF00) >> 8);
		packet[i++] = (byte)((DebugCmd.enc_lower_limit[num_via]) & 0x00FF);
		packet[i++] = DebugCmd.air_state[num_via];
	}
	
	packet[i] = stdUartProtocolAbstraction::calculateChecksum(&packet[0], i );
	i++;
	PinEnRS485_DE = 1;
	return stdUartProtocolAbstraction::SendCommand(packet, i, 0);	 
}

/**
* Sends debug vie 1-3 command
*/
int SIFRAProtocol::sendSIFRADebugVie_1_3()
{	
	byte packet[_PKT_DEBUG_VIE_1_3_LENGTH];
	byte i = 0;
	byte num_via;
	word peso_spec;
	word flusso;
	
	packet[i++] =  M3100;		// NODEID perifierica
	packet[i++] = SIFRA_GET_DEBUG_VIE_1_3_M3100;
	
	for(num_via = 0; num_via < 3; num_via++)
	{
		peso_spec = (word)(Via[num_via].peso_spec*1000.0);
		flusso = (word)(Via[num_via].flusso_pompa*10.0);
		packet[i++] = (byte)(Via[num_via].abilitazione);
		packet[i++] = (byte)(Via[num_via].da_eseguire);
		packet[i++] = (byte)(Via[num_via].eseguita);
		packet[i++] = (byte)(Via[num_via].restart_bag);
		packet[i++] = (byte)(Via[num_via].stato_led);
		packet[i++] = Via[num_via].stato_luci;
		packet[i++] = Via[num_via].fase_luci;
		packet[i++] = (byte)(((Via[num_via].vel) & 0xFF00) >> 8);
		packet[i++] = (byte)((Via[num_via].vel) & 0x00FF);
		packet[i++] = (byte)(((Via[num_via].peso_da_erogare) & 0xFF00) >> 8);
		packet[i++] = (byte)((Via[num_via].peso_da_erogare) & 0x00FF);
		packet[i++] = (byte)(((Via[num_via].peso_erogato) & 0xFF00) >> 8);
		packet[i++] = (byte)((Via[num_via].peso_erogato) & 0x00FF);
		packet[i++] = (byte)(((Via[num_via].peso_gia_erogato) & 0xFF00) >> 8);
		packet[i++] = (byte)((Via[num_via].peso_gia_erogato) & 0x00FF);
		/*
		packet[i++] = (byte)(((Via[num_via].peso_stop) & 0xFF00) >> 8);
		packet[i++] = (byte)((Via[num_via].peso_stop) & 0x00FF);
		*/
		packet[i++] = (byte)(((Via[num_via].variazione_peso_stop) & 0xFF00) >> 8);
		packet[i++] = (byte)((Via[num_via].variazione_peso_stop) & 0x00FF);
		packet[i++] = (byte)(((Via[num_via].tara) & 0xFF00) >> 8);
		packet[i++] = (byte)((Via[num_via].tara) & 0x00FF);
		packet[i++] = (byte)(((peso_spec) & 0xFF00) >> 8);
		packet[i++] = (byte)((peso_spec) & 0x00FF);
		packet[i++] = (byte)(((flusso) & 0xFF00) >> 8);
		packet[i++] = (byte)((flusso) & 0x00FF);
	}
	
	packet[i] = stdUartProtocolAbstraction::calculateChecksum(&packet[0], i );
	i++;
	PinEnRS485_DE = 1;
	return stdUartProtocolAbstraction::SendCommand(packet, i, 0);	 
}

/**
* Sends debug vie 4-5 command
*/
int SIFRAProtocol::sendSIFRADebugVie_4_5()
{	
	byte packet[_PKT_DEBUG_VIE_4_5_LENGTH];
	byte i = 0;
	byte num_via;
	word peso_spec;
	word flusso;
	
	packet[i++] =  M3100;		// NODEID perifierica
	packet[i++] = SIFRA_GET_DEBUG_VIE_4_5_M3100;
	
	for(num_via = 3; num_via < NUM_MAX_LINE; num_via++)
	{
		peso_spec = (word)(Via[num_via].peso_spec*1000.0);
		flusso = (word)(Via[num_via].flusso_pompa*10.0);
		packet[i++] = (byte)(Via[num_via].abilitazione);
		packet[i++] = (byte)(Via[num_via].da_eseguire);
		packet[i++] = (byte)(Via[num_via].eseguita);
		packet[i++] = (byte)(Via[num_via].restart_bag);
		packet[i++] = (byte)(Via[num_via].stato_led);
		packet[i++] = Via[num_via].stato_luci;
		packet[i++] = Via[num_via].fase_luci;
		packet[i++] = (byte)(((Via[num_via].vel) & 0xFF00) >> 8);
		packet[i++] = (byte)((Via[num_via].vel) & 0x00FF);
		packet[i++] = (byte)(((Via[num_via].peso_da_erogare) & 0xFF00) >> 8);
		packet[i++] = (byte)((Via[num_via].peso_da_erogare) & 0x00FF);
		packet[i++] = (byte)(((Via[num_via].peso_erogato) & 0xFF00) >> 8);
		packet[i++] = (byte)((Via[num_via].peso_erogato) & 0x00FF);
		packet[i++] = (byte)(((Via[num_via].peso_gia_erogato) & 0xFF00) >> 8);
		packet[i++] = (byte)((Via[num_via].peso_gia_erogato) & 0x00FF);
		/*
		packet[i++] = (byte)(((Via[num_via].peso_stop) & 0xFF00) >> 8);
		packet[i++] = (byte)((Via[num_via].peso_stop) & 0x00FF);
		*/
		packet[i++] = (byte)(((Via[num_via].variazione_peso_stop) & 0xFF00) >> 8);
		packet[i++] = (byte)((Via[num_via].variazione_peso_stop) & 0x00FF);
		packet[i++] = (byte)(((Via[num_via].tara) & 0xFF00) >> 8);
		packet[i++] = (byte)((Via[num_via].tara) & 0x00FF);
		packet[i++] = (byte)(((peso_spec) & 0xFF00) >> 8);
		packet[i++] = (byte)((peso_spec) & 0x00FF);
		packet[i++] = (byte)(((flusso) & 0xFF00) >> 8);
		packet[i++] = (byte)((flusso) & 0x00FF);
	}

	packet[i++] = (byte)StatusCmd.led_aria_en;
	packet[i++] = (byte)StatusCmd.air_block_en[_VIA_1_];
	packet[i++] = (byte)StatusCmd.air_block_en[_VIA_2_];
	packet[i++] = (byte)StatusCmd.air_block_en[_VIA_3_];
	packet[i++] = (byte)StatusCmd.air_block_en[_VIA_4_];
	packet[i++] = (byte)StatusCmd.air_block_en[_VIA_5_];
	packet[i++] = (byte)StatusCmd.fillingOnCourse;
	packet[i++] = StatusCmd.state;
	packet[i++] = StatusCmd.prevState;
	packet[i++] = StatusCmd.phase;
	packet[i++] = StatusCmd.status;
	packet[i++] = StatusCmd.flagsErrorLine;
	packet[i++] = (byte)(((StatusCmd.statusChan[_VIA_1_]) & 0xFF00) >> 8);
	packet[i++] = (byte)((StatusCmd.statusChan[_VIA_1_]) & 0x00FF);
	packet[i++] = (byte)(((StatusCmd.statusChan[_VIA_2_]) & 0xFF00) >> 8);
	packet[i++] = (byte)((StatusCmd.statusChan[_VIA_2_]) & 0x00FF);
	packet[i++] = (byte)(((StatusCmd.statusChan[_VIA_3_]) & 0xFF00) >> 8);
	packet[i++] = (byte)((StatusCmd.statusChan[_VIA_3_]) & 0x00FF);
	packet[i++] = (byte)(((StatusCmd.statusChan[_VIA_4_]) & 0xFF00) >> 8);
	packet[i++] = (byte)((StatusCmd.statusChan[_VIA_4_]) & 0x00FF);
	packet[i++] = (byte)(((StatusCmd.statusChan[_VIA_5_]) & 0xFF00) >> 8);
	packet[i++] = (byte)((StatusCmd.statusChan[_VIA_5_]) & 0x00FF);
	packet[i++] = StatusCmd.errors;
	packet[i++] = StatusCmd.log_error;
	packet[i++] = (byte)(((StatusCmd.commErrorHw) & 0xFF00) >> 8);
	packet[i++] = (byte)((StatusCmd.commErrorHw) & 0x00FF);
	packet[i++] = (byte)(((StatusCmd.commErrorProtocol) & 0xFF00) >> 8);
	packet[i++] = (byte)((StatusCmd.commErrorProtocol) & 0x00FF);
	packet[i++] = (byte)StatusCmd.cover_open;
	
	packet[i] = stdUartProtocolAbstraction::calculateChecksum(&packet[0], i );
	i++;
	PinEnRS485_DE = 1;
	return stdUartProtocolAbstraction::SendCommand(packet, i, 0);	 
}

/**
* Sends debug start command
*/
int SIFRAProtocol::sendSIFRADebugStart()
{	
	byte packet[_PKT_DEBUG_START_LENGTH];
	byte i = 0;
	byte num_via;
	
	packet[i++] =  M3100;		// NODEID perifierica
	packet[i++] = SIFRA_GET_DEBUG_START_M3100;

	packet[i++] = (byte)(((StartCmd.function) & 0xFF00) >> 8);
	packet[i++] = (byte)((StartCmd.function) & 0x00FF);
	packet[i++] = (byte)(StartCmd.support);
	for(num_via = 0; num_via < NUM_MAX_LINE; num_via++)
	{
		packet[i++] = (byte)(((StartCmd.peso_linea[num_via]) & 0xFF00) >> 8);
		packet[i++] = (byte)((StartCmd.peso_linea[num_via]) & 0x00FF);
		packet[i++] = (byte)(((StartCmd.tara_linea[num_via]) & 0xFF00) >> 8);
		packet[i++] = (byte)((StartCmd.tara_linea[num_via]) & 0x00FF);
		packet[i++] = (byte)(((StartCmd.peso_spec[num_via]) & 0xFF00) >> 8);
		packet[i++] = (byte)((StartCmd.peso_spec[num_via]) & 0x00FF);
	}

	packet[i++] = g_stateMachine.calibState;
	packet[i++] = g_stateMachine.rtState;
	packet[i++] = g_stateMachine.saccaState;
	packet[i++] = g_stateMachine.manualState;
	packet[i++] = g_stateMachine.driverState;
	packet[i++] = g_stateMachine.ledState;
	packet[i++] = g_stateMachine.serviceState;

	packet[i++] = g_restart.tipo_linea;
	packet[i++] = g_restart.restart_bk;
	
	packet[i] = stdUartProtocolAbstraction::calculateChecksum(&packet[0], i );
	i++;
	PinEnRS485_DE = 1;
	return stdUartProtocolAbstraction::SendCommand(packet, i, 0);	 
}

/**
* Sends debug ram command
*/
int SIFRAProtocol::sendSIFRADebugRam()
{	
	byte packet[_PKT_DEBUG_RAM_LENGTH];
	byte i = 0;
	byte num_via;
	word enc;
	word toll;
	
	packet[i++] =  M3100;		// NODEID perifierica
	packet[i++] = SIFRA_GET_DEBUG_RAM_M3100;

	packet[i++] = bk.start_cmd;
	packet[i++] = bk.vie_da_eseguire;
	packet[i++] = bk.vie_eseguite;
	for(num_via = 0; num_via < NUM_MAX_LINE; num_via++)
	{
		packet[i++] = (byte)(bk.abilitazione[num_via]);
		packet[i++] = (byte)(((bk.peso_erogato[num_via]) & 0xFF00) >> 8);
		packet[i++] = (byte)((bk.peso_erogato[num_via]) & 0x00FF);
		packet[i++] = (byte)(((bk.peso_stop[num_via]) & 0xFF000000) >> 24);
		packet[i++] = (byte)(((bk.peso_stop[num_via]) & 0x00FF0000) >> 16);
		packet[i++] = (byte)(((bk.peso_stop[num_via]) & 0x0000FF00) >> 8);
		packet[i++] = (byte)((bk.peso_stop[num_via]) & 0x000000FF);
	}

	for(num_via = 0; num_via < NUM_MAX_LINE; num_via++)
	{
		enc = (word)(g_encStruct[num_via].param_encoder[0]*10.0);
		toll = (word)(g_encStruct[num_via].tolleranza[0]*100.0);
		packet[i++] = (byte)(((enc) & 0xFF00) >> 8);
		packet[i++] = (byte)((enc) & 0x00FF);
		packet[i++] = (byte)(((toll) & 0xFF00) >> 8);
		packet[i++] = (byte)((toll) & 0x00FF);
		enc = (word)(g_encStruct[num_via].param_encoder[1]*10.0);
		toll = (word)(g_encStruct[num_via].tolleranza[1]*100.0);
		packet[i++] = (byte)(((enc) & 0xFF00) >> 8);
		packet[i++] = (byte)((enc) & 0x00FF);
		packet[i++] = (byte)(((toll) & 0xFF00) >> 8);
		packet[i++] = (byte)((toll) & 0x00FF);
	}
	
	packet[i] = stdUartProtocolAbstraction::calculateChecksum(&packet[0], i );
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
	byte i = 0;
	
	packet[i++] = getLocalNodeID();
	packet[i++] = last_opCode;		
	packet[i] = stdUartProtocolAbstraction::calculateChecksum(&packet[0], i );
	i++;
	PinEnRS485_DE = 1;
	return stdUartProtocolAbstraction::SendCommand(packet, i, 0);
}

/**
* Calculates physical load of every line in M3100
* NON MODIFICARE PER ALCUN MOTIVO I CAST ED IN GENERALE TALE METODO, RESTITUISCE CORRETTAMENTE PESI SIA POSITIVI CHE NEGATIVI
*/
long SIFRAProtocol::sendSIFRALoadSamples(dword samples, int chan)
{
	double calc_sample = 0;
	long result;
	
	calc_sample = weightChan->getGain(chan)* ((int)samples - (int)weightChan->getOffset(chan));
	/*
	if(m_show_adcvalue == True)	// funzione di debug: si invia il valore dell'adc (filtrato) non convertito in grammi,
	{
		loadchan[ chan ] = samples;	//(int)(_CONV_X_PROTOCOLLO * samples + _OFFSET_SU_ZERO); // adc espresso in modulo 2alla16
	}
	else		// via normale: si invia il dato di peso convertito in grammi con i dati di calibrazione memorizzati
	{
		loadchan[ chan ] = (int)(_CONV_X_PROTOCOLLO * calc_sample + _OFFSET_SU_ZERO);
	}
	*/
	loadchan[chan] = (int)((_CONV_X_PROTOCOLLO * calc_sample) + _OFFSET_SU_ZERO);
	result = (long)(_CONV_IN_DECIMI * calc_sample);
	
	return result;	// il peso sarà espresso in decimi di grammo con risuluzione +/-0.1g
	
}

/* ESESMPIO
1) calc_sample = 80,35567 -> x 5,0 -> 401,77835 -> + 200 -> 601,77835 -> (int)() -> 601 spedito -- RICEVUTO 601 - 200 = 401 -> /5.0 -> 80.2
m_peso_attuale = 
2) calc_sample = 80.2 -> x 5.0 -> 401.0 -> + 200 -> 601.0 -> (int)() -> 601 spedito -- RICEVUTO 601 - 200 = 401 -> /5.0 -> 80.2
3) calc_sample = 80.4 -> x 5.0 -> 402.0 -> + 200 -> 602.0 -> (int)() -> 602 spedito -- RICEVUTO 602 - 200 = 402 -> /5.0 -> 80.4
*/

/**
* Calculates phisical load in M3300
*/
int SIFRAProtocol::sendSIFRALoadSamples(dword *loadsystem)
{
	byte i;
	
	for( i = 0; i < _MAX_LOAD_CHAN_; i++)
	{
		g_adc_real_value[i] = (word)loadsystem[i];
	}
	return 0;
}

// mette in spedizione (cioè nella fifo delle cose da re-inviare) il comando ricevuto
int SIFRAProtocol::SIFRA_getInfoCmdDecode()
{
	sendMsg(SIFRAMsg_infoAsked);
	return 1;
}

// mette in spedizione (cioè nella fifo delle cose da re-inviare) il comando ricevuto
int SIFRAProtocol::SIFRA_getDebugCmdDecode()
{
	sendMsg(SIFRAMsg_debugAsked);
	return 1;
}

// mette in spedizione (cioè nella fifo delle cose da re-inviare) il comando ricevuto
int SIFRAProtocol::SIFRA_getDebugVie_1_3_CmdDecode()
{
	sendMsg(SIFRAMsg_debugVie_1_3_Asked);
	return 1;
}

// mette in spedizione (cioè nella fifo delle cose da re-inviare) il comando ricevuto
int SIFRAProtocol::SIFRA_getDebugVie_4_5_CmdDecode()
{
	sendMsg(SIFRAMsg_debugVie_4_5_Asked);
	return 1;
}

// mette in spedizione (cioè nella fifo delle cose da re-inviare) il comando ricevuto
int SIFRAProtocol::SIFRA_getDebugStartCmdDecode()
{
	sendMsg(SIFRAMsg_debugStartAsked);
	return 1;
}

// mette in spedizione (cioè nella fifo delle cose da re-inviare) il comando ricevuto
int SIFRAProtocol::SIFRA_getDebugRamCmdDecode()
{
	sendMsg(SIFRAMsg_debugRamAsked);
	return 1;
}

/**
Sends to protocol the overall stop command
*/
int SIFRAProtocol::SIFRA_set_Stop_All()
{
	sendMsg(SIFRAMsg_StopAll);
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
Sends to protocol the status ask command
*/
int SIFRAProtocol::SIFRA_setLineToIdentify()
{
	sendMsg(SIFRAMsg_newLines);
	return 1;
}

int SIFRAProtocol::SIFRA_resetMaxWeightError()
{
	sendMsg(SIFRAMsg_maxWeightReset);
	return 1;
}

int SIFRAProtocol::SIFRA_restartControl()
{
	sendMsg(SIFRAMsg_restartControl);
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
Sends to protocol the start command
*/
int SIFRAProtocol::SIFRA_setStart()
{
	sendMsg(SIFRAMsg_setStart);
	return 1;
}
/**
Sends to protocol the cell1 zero calibration command
*/
int SIFRAProtocol::SIFRA_setZeroCell1()
{
	sendMsg(SIFRAMsg_setOffset1);
	return 1;
}
/**
Sends to protocol the cell2 zero calibration command
*/
int SIFRAProtocol::SIFRA_setZeroCell2()
{
	sendMsg(SIFRAMsg_setOffset2);
	return 1;
}
/**
Sends to protocol the cell3 zero calibration command
*/
int SIFRAProtocol::SIFRA_setZeroCell3()
{
	sendMsg(SIFRAMsg_setOffset3);
	return 1;
}
/**
Sends to protocol the cell4 zero calibration command
*/
int SIFRAProtocol::SIFRA_setZeroCell4()
{
	sendMsg(SIFRAMsg_setOffset4);
	return 1;
}
/**
Sends to protocol the cell5 zero calibration command
*/
int SIFRAProtocol::SIFRA_setZeroCell5()
{
	sendMsg(SIFRAMsg_setOffset5);
	return 1;
}
/**
Sends to protocol the cell1 gain calibration command
*/
int SIFRAProtocol::SIFRA_setGainCell1()
{
	sendMsg(SIFRAMsg_setGain1);
	return 1;
}
/**
Sends to protocol the cell2 gain calibration command
*/
int SIFRAProtocol::SIFRA_setGainCell2()
{
	sendMsg(SIFRAMsg_setGain2);
	return 1;
}
/**
Sends to protocol the cell3 gain calibration command
*/
int SIFRAProtocol::SIFRA_setGainCell3()
{
	sendMsg(SIFRAMsg_setGain3);
	return 1;
}
/**
Sends to protocol the cell4 gain calibration command
*/
int SIFRAProtocol::SIFRA_setGainCell4()
{
	sendMsg(SIFRAMsg_setGain4);
	return 1;
}
/**
Sends to protocol the cell5 gain calibration command
*/
int SIFRAProtocol::SIFRA_setGainCell5()
{
	sendMsg(SIFRAMsg_setGain5);
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
Sends to protocol the jump-to-loader command
*/
int SIFRAProtocol::SIFRA_setJumpToLoader()
{
	sendMsg(SIFRAMsg_jumpToLoader);
	return 1;
}

/**
Sends to protocol the loop command
*/
int SIFRAProtocol::SIFRA_setLoopApplication()
{
	sendMsg(E_SIFRAMsg_loopApplication);
	return 1;
}
		
int SIFRAProtocol::SIFRA_zeroLoadCmdDecode()
{
	sendMsg(SIFRAMsg_setZeroCalib);
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
* Checks opcode just received for module M3100. Then indicates how many bytes have to be recevied and analysed.
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
		case M3100:
			switch(data)
			{
				case SIFRA_SET_START_M3100:	// messaggio di START con gli eventuali parametri
					numDataToReceive = _PKT_START_DATA_M3100;
					return True;
					break;
				case SIFRA_RESET_MAX_WEIGHT:
				case SIFRA_RESTART_CONTROL:
				case SIFRA_GET_CELLS_NOT_CALIB:
					numDataToReceive = _PKT_RST_MAX_WEIGHT_M3100;
					return True;
					break;
				case SIFRA_GET_SCALES_STATE_M3100:
					numDataToReceive = _PKT_SCALES_PHASE_LEN_M3100;
					return True;
					break;
				case SIFRA_GET_STATUS_M3100:	// messaggio richiesta stato
				case SIFRA_SETZERO_CELL1:	// calib. zero cella 1
				case SIFRA_SETZERO_CELL2:	// calib. zera cella 2
				case SIFRA_SETZERO_CELL3:	// calib. zera cella 3
				case SIFRA_SETZERO_CELL4:	// calib. zera cella 4
				case SIFRA_SETZERO_CELL5:	// calib. zero cella 5
				case SIFRA_SET_CAL_CELL1:	// guadagno cella 1
				case SIFRA_SET_CAL_CELL2:	// guadagno cella 2
				case SIFRA_SET_CAL_CELL3:	// guadagno cella 3
				case SIFRA_SET_CAL_CELL4:	// guadagno cella 4
				case SIFRA_SET_CAL_CELL5:	// guadagno cella 5
				case SIFRA_RESET_CALIB_PARAMS:		// reset parametri di calibrazione
				case SIFRA_ZERO_CELLS_M3100:	// zero generale per tutte le celle di carico
				case SIFRA_GET_DEBUG_M3100:	// debug M3100
				case SIFRA_GET_DEBUG_VIE_1_3_M3100:
				case SIFRA_GET_DEBUG_VIE_4_5_M3100:
				case SIFRA_GET_DEBUG_START_M3100:
				case SIFRA_GET_DEBUG_RAM_M3100:
				case SIFRA_GET_INFO_M3100:	// richiesta versione HW e FW
				case SIFRA_NEW_LINES_M3100:	// indicazione linee nuove ricevuta dalla tastiera
				case SIFRA_JUMP_TO_LOADER:	// salto in loader per riprogrammazione
				case SIFRA_M3000_STOP:
				case E_SIFRA_LOOP_CMD:
					numDataToReceive = 0;	// + il chk
					return True;
					break;
				default:
					return False;
					break;
			}
			break;
			
		case M3300:
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
	return False;
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
