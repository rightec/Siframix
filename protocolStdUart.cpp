/**
@file		protocolStdUart.cpp
@brief		Class to manage a general uart protocol
					
@author		Nicola
@date		18/01/2011
@version	01.1
*/

#include "protocolStdUart.h"


//---------------------------------------------------------------------------------------
//----------------------- stdUartProtocolAbstraction ------------------------------------
//---------------------------------------------------------------------------------------

/**
Class constructor.

@param device uart device
@see UARTDevice
*/
stdUartProtocolAbstraction::stdUartProtocolAbstraction(UARTDevice device) :
	QueueUart(device)
{
	m_timeoutTx = 100;
	m_timeoutRx = 100;
	m_TxTimeoutTimer.Stop();
	m_RxTimeoutTimer.Stop();
	m_lastError = stdUart_NoError;
	m_error.clear();
	m_numBytesRecieved = 0;
	m_packetLength = 0;
	m_stdUartProtocolAbstractionDataAnalyse = NULL;
	m_receiverState = stdUart_waitDeviceID;	//m_receiverState = stdUart_waitStx;
	m_lastReceivedOpCode = 0;
	m_numPacketsReceived = 0;
	m_numPacketsTransmitted = 0;
	//m_lastRemoteNodeID = 0;
	//m_localNodeID = 0;
	m_lastDataReceived = NULL;
	m_bufferLastDataReceivedDimension = 0;
	m_receivedOpCode.clear();
	
}

/**
Class destructor
*/
stdUartProtocolAbstraction::~stdUartProtocolAbstraction()
{
	//if (m_lastDataReceived != NULL)
	//{
	//	delete m_lastDataReceived;
	//}
}

/**
		Set the call back to decode the packet. The packet is decoded within the main loop. 
		@pcallback callback function
		*/
void stdUartProtocolAbstraction::SetDataAnalyseCallBack(bool (*pcallback)(unsigned short dataS, stdUartProtocolAbstraction *protocol))
{
	m_stdUartProtocolAbstractionDataAnalyse = pcallback;
}

/**
Sets the call back to decode the packet within the ISR. Not used!! the packet is decoded within the main loop 
@pcallback callback function
*/
void stdUartProtocolAbstraction::SetReceiveCallBack(void (* pcallback)(UartDeviceAbstraction *pdevice, unsigned data))
{
	// + Imposta la funzione di callback da invocare ogni qualvolta il buffer di ricezione è pieno

	m_pReceiveCallback = pcallback;
}


void stdUartProtocolAbstraction::stdUartProtocolAbstractionReceiveCallback(UartDeviceAbstraction *pdevice, unsigned data)
{
	// + Gestisce gli stati del ricevitore ZigBee
	
	//stdUartProtocolAbstraction *pUart = (stdUartProtocolAbstraction *) pdevice;
	
}

/**
Restores the initial condition.
*/
void stdUartProtocolAbstraction::restoreIntialCondition()
{
	m_receiverState = stdUart_waitDeviceID;	//m_receiverState = stdUart_waitStx;	
	m_TxTimeoutTimer.Stop();
	m_RxTimeoutTimer.Stop();
}

/**
It has to be called in the sons class. 

It manages the timer and rise thier overruns errors and the protocol's decode function. 
*/
void stdUartProtocolAbstraction::Manager()
{
	unsigned short dataBuff;

	if (m_stdUartProtocolAbstractionDataAnalyse == NULL)
	{
		return;
	}
	// + Analizzo i dati ricevuti
	// finchè dati nella coda degli short, li estrai e li manda alla SIFRADataAnalyse 
	while (QueueUart::ReceiveData(dataBuff))
	{
		if (this->m_stdUartProtocolAbstractionDataAnalyse(dataBuff, this))
			break;
	}

	//-- verifichiamo se ci sono stati errori di time out sulla ricezione dati
	/*
	if(checkTimeOutElapsed())
		goBuzzer(ALARM_LIKE);
	*/

	checkTimeOutElapsed();
	
	if (QueueUart::TransmitBufferOverflow())
	{
		setError(stdUart_TransmitBufferOverflowError, True);
		QueueUart::rstTransmitBufferOverflow();
		stdUartProtocolAbstraction::restoreIntialCondition();
	}
	if(QueueUart::ReceptionBufferOverflow())
	{
		setError(stdUart_ReceptionBufferOverflowError, True);
		QueueUart::rstReceptionBufferOverflow();
		stdUartProtocolAbstraction::restoreIntialCondition();
	}

}

bool stdUartProtocolAbstraction::checkTimeOutElapsed()
{
	bool error = False;
	// + ricezioni dati in modalità Listening
	if (timeoutRxElapsed())
	{
		setError(stdUart_TimeOutErrorRx, True);
		stdUartProtocolAbstraction::restoreIntialCondition();
	
		error = True;
	// + Invio comando al modulo e attesa dellla risposta
	}if (timeoutTxElapsed())
	{
		setError(stdUart_TimeOutErrorTx, True);
		stdUartProtocolAbstraction::restoreIntialCondition();
		//ClearTransmissionBuffer();
		error = True;
	}

	return error;
	
}

/**
Sets le last error occured.
@param error king of error
@param pushToQueue if true push the current error to the fifo's errors, if false just set the last error occurred. 
*/
void stdUartProtocolAbstraction::setError(int error, bool pushToQueue)
{
	m_lastError = error;
	
	if (pushToQueue)
	{
		if (!m_error.full())
		{
			m_error.push_back(m_lastError);
		}
		else
		{
			m_error.pop_front(error);
			m_error.push_back(m_lastError);
		}
	}
}

/**
Resets the last error and its fifo.
*/
void stdUartProtocolAbstraction::rstError()
{
	m_lastError = stdUart_NoError;
	m_error.clear();
}

/**
@return the last error occurred.
*/
int stdUartProtocolAbstraction::getLastError()
{
	return m_lastError;
}

/**
Pop the error from the fifo's errors.

@return the error, if the fifo is empty returns stdUart_NoError
*/
int stdUartProtocolAbstraction::popError()
{
	int error = stdUart_NoError;
	if(!m_error.empty())
	{
		m_error.pop_front(error);
	}
	return error;
}


/**
@return always true
*/
bool stdUartProtocolAbstraction::IsTransmissionAvailable()
{
	return True;
}

/**
@return always true
*/
bool stdUartProtocolAbstraction::IsReceptionAvailable()
{
	return True;
}

/**
Sends a command
@param pdata pointer to the command data
@param len command length
@param timeout time to wait before raise timeout TX error. If 0 the timer does NOT start.
@return the number of bytes transmitted. If 0 no bytes were transmitted.
*/
int stdUartProtocolAbstraction::SendCommand(unsigned char *pdata, int len, int timeOut)
{
	if (QueueUart::GetFreeBytesInTransmissionBuffer() < len)
		return 0;

	QueueUart::TransmitLock();
	QueueUart::TransmitData(pdata, len);
	if (timeOut > 0)
	{
		stdUartProtocolAbstraction::setTimeoutTx(timeOut);
		stdUartProtocolAbstraction::startTxTimeoutTimer();
	}
	QueueUart::TransmitUnlock();
	QueueUart::StartTransmission();
	return len;
}

void stdUartProtocolAbstraction::setLastDataReceivedBuffer(int i)
{
	if (m_lastDataReceived != NULL)
	{
		delete m_lastDataReceived;
	}
	m_bufferLastDataReceivedDimension = i;
	m_lastDataReceived = new byte[i];
}

/**
Adds bytes to the reception buffer.
@param val pointer to the byte array to add
@param start_index index of insertion
@param dim number of bytes to add
@return true if the data can be added to the reception buffer, false otherwise. 
If return flalse the error stdUart_DataReceivedBufferOverflow is pushed to the error fifo
@see setError
*/
bool stdUartProtocolAbstraction::addDataToReceiveBuffer(byte *val, int startIndex, int dim)
{
	int i;
	if ((m_lastDataReceived == NULL) || ((startIndex + dim ) >= m_bufferLastDataReceivedDimension) || (startIndex < 0))
	{
		setError(stdUart_DataReceivedBufferOverflow, True);
		return False;
	}
	for (i = startIndex; i < startIndex + dim; i++)
	{
		m_lastDataReceived[i] = val[i - startIndex];
	}
	return True; 
}

/**
Sets the received opcode. The opcode are pushed in a fifo.
@param code received opcode
*/
void stdUartProtocolAbstraction::setReceivedOpCode(int opCode)
{ 
	m_lastReceivedOpCode = opCode;
	if(!m_receivedOpCode.full())
	{
		m_receivedOpCode.push_back(m_lastReceivedOpCode);	// inserisce in coda il nuovo
	}else
	{	// se la fifo è piena
		m_receivedOpCode.pop_front(opCode);	// prelva il più vecchio
		m_receivedOpCode.push_back(m_lastReceivedOpCode);	// insericse il nuovo
	}
}

/**
Pop the received opcode
@param opCode received opcode (valid only if the function returns true)
@return true if the fifo is not empty, false otherwise. 
*/
bool stdUartProtocolAbstraction::popPacketReceived(int &opCode)
{ 
	if(!m_receivedOpCode.empty())
	{
		m_receivedOpCode.pop_front(opCode);
		return True;
	}
	return False;
}

/**
@return the number of opcode fifoed
*/
int stdUartProtocolAbstraction::getNewPacketReceived()
{
	return m_receivedOpCode.numItem();
}

/**
Resets the opcode fifo
*/
void stdUartProtocolAbstraction::rstNewPacketReceived()
{
	return m_receivedOpCode.clear();
}

/**
Calculates the checksum as sum of bytes.
@param data pointer to the data to use to calculate the checksum
@param len length of data
@return the checksum value
*/
byte stdUartProtocolAbstraction::calculateChecksum(byte *data, int len)
{
	byte checksum = 0;
	int i;
	for(i = 0; i < len; i++)
	{
		checksum += data[i];
	}
	return ~checksum;
}

