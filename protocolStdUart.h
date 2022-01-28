/**
@file		protocolStdUart.h
@brief		Class to manage a general uart protocol
					
@author		Nicola
@date		18/01/2011
@version	01.1
*/

#ifndef __STD_UART_PROTOCOL_H__
#define __STD_UART_PROTOCOL_H__

#include "global.h"
#include "swtim.h" 
#include "queueuart.h"
#include "sound.h"

/**
Receiver states
*/
enum stdUartRecieverStates
{
	stdUart_waitStx = 0,
	stdUart_waitDeviceID,
	stdUart_waitNumPacketTransmit,
	stdUart_waitOpCode1,
	stdUart_waitOpCode2,
	stdUart_waitDataLength,
	stdUart_receivingData,
	stdUart_waitChecksum,
	stdUart_waitEtx
};

/**
Standard errors. 
*/
enum
{
	stdUart_NoError = 0,

	// hardware errors
	stdUart_Hw_ReceptionError,				/**< Reception error. Generate from QueueUart*/
	stdUart_Hw_ParityError,					/**< Parity error. Generate from QueueUart*/
	stdUart_Hw_FrameError,					/**< Frame error. Generate from QueueUart*/
	stdUart_Hw_OverrunError,				/**< Overrun error. Generate from QueueUart*/
	stdUart_Hw_NoDevice,					/**< No device error. Generate from QueueUart*/
	stdUart_Hw_ErrorUnknown,				/**< Unknown error. Generate from QueueUart*/

	// protocol errors
	stdUart_UnknownOpcode,					/**< Unknown opcode*/
	stdUart_protocolStateUnknown,			/**< Protocol state unknown*/
	stdUart_TimeOutErrorRx,					/**< Timeout RX*/
	stdUart_TimeOutErrorTx,					/**< Timeout TX*/
	stdUart_ChecksumError,					/**< checksum error*/ 
	stdUart_TransmitBufferOverflowError,		/**< transmit buffer overflow*/ 
	stdUart_ReceptionBufferOverflowError,		/**< reception buffer overflow*/ 
	stdUart_DataReceivedBufferOverflow		/**< data received buffer overflow*/ 
};

/**
Class to manage a general uart protocol.
*/
class stdUartProtocolAbstraction: protected QueueUart
{
	public:
		
		stdUartProtocolAbstraction(UARTDevice device = UART2);
		~stdUartProtocolAbstraction();
		
		virtual void Manager();
		virtual void setError(int error, bool pushToQueue = False);
		virtual void rstError();
		virtual int getLastError();
		virtual int popError();

		/**
		@return the number of bytes in the transmission buffer.
		*/
		int GetNumBytesInTransmissionBuffer(){return QueueUart::GetNumBytesInTransmissionBuffer();};

		/**
		Sets the TX timeout .
		The TX timer starts when a message is sent and it has to be stop when stx is received.
		@param t time in msec
		@see SendCommand
		*/
		void setTimeoutTx(int t){ m_timeoutTx = t;};
		/**
		Starts the TX timeout timer
		*/
		inline void startTxTimeoutTimer(){ m_TxTimeoutTimer.Preset(m_timeoutTx);};
		/**
		@return true if the TX timeout is elapsed, false otherwise
		*/
		bool timeoutTxElapsed(){return  m_TxTimeoutTimer.Match();};
		/**
		Stops the timeout TX timer
		*/
		inline void stopTxTimeoutTimer(){ m_TxTimeoutTimer.Stop();};
		/**
		@return the timeout TX timer status
		@see DecTimer
		*/
		inline bool getStatusTxTimeoutTimer(){return m_TxTimeoutTimer.getStatus();};

		/**
		Sets the RX timeout .
		The RX timer has to be started when the stx is received and it has to be stopped when the last byte of the packet is received .
		@param t time in msec
		*/
		void setTimeoutRx(int t){m_timeoutRx = t;};
		/**
		Starts the RX timeout timer
		*/
		inline void startRxTimeoutTimer(){m_RxTimeoutTimer.Preset(m_timeoutRx);};
		/**
		@return true if the RX timeout is elapsed, false otherwise
		*/
		bool timeoutRxElapsed(){return m_RxTimeoutTimer.Match();};
		/**
		Stops the timeout RX timer
		*/
		inline void stopRxTimeoutTimer(){m_RxTimeoutTimer.Stop();};
		/**
		@return the timeout RX timer status
		@see DecTimer
		*/
		inline bool getStatusRxTimeoutTimer(){return m_RxTimeoutTimer.getStatus();};

		/**
		@return the received state
		@see stdUartRecieverStates
		*/
		int getReceiverState(){ return m_receiverState;};
		/**
		Sets the receiver state.
		@param state receiver state
		*/
		void setReceiverState(int state){ m_receiverState = state;};

		/**
		New byte received. Increments the byte receiver counter.
		@see m_numBytesRecieved
		*/
		void newByteReceived(){ m_numBytesRecieved++;};
		/**
		@return the number of bytes received.
		*/
		int getNumBytesReceived(){ return m_numBytesRecieved;};
		/**
		Reset the number of bytes received
		*/
		void rstBytesReceived(){ m_numBytesRecieved = 0;};
		/**
		Set the last packet received length
		@param i packet length
		*/
		void setPacketlength(int i){ m_packetLength = i;};
		/**
		@return the last packet receivd length
		*/
		int getPacketLength(){ return m_packetLength;};

		/**
		Increments the counter of packet received
		@see m_numPacketsReceived
		*/
		void newPacketReceived(){ m_numPacketsReceived++;};
		/**
		Adds i to the packets received counter
		@param i number added to the counter
		*/
		void addPacketsReceived(byte i = 1){ m_numPacketsReceived += i;};
		/**
		Resets the packets received counter.
		*/
		void rstPacketsReceived(){m_numPacketsReceived = 0;};
		/**
		@return the number of packets received
		*/
		byte getPacketsReceived(){ return m_numPacketsReceived;};
		/**
		Sets the packets received counter to i
		@param i number of packets received.
		*/
		void setPacketsReceived(byte i){ m_numPacketsReceived = i;};

		/**
		Increment the packet transmit counter
		*/
		void newPacketTransmitted(){ m_numPacketsTransmitted++;};
		/**
		Adds i to the packets transmitted counter
		@param i number add to the counter
		*/
		void addPacketsTransmitted(byte i = 1){ m_numPacketsTransmitted += i;};
		/**
		Resets the packets transmitted counter.
		*/
		void rstPacketsTransmitted(){m_numPacketsTransmitted = 0;};
		/**
		@return the number of packets transmitted
		*/
		byte getPacketsTransmitted(){ return m_numPacketsTransmitted;};
		/**
		Sets the packets transmitted counter to i
		@param i number of packets transmitted.
		*/
		void setPacketsTransmitted(byte i){ m_numPacketsTransmitted = i;};	

		/**
		@return the last received opcode
		*/
		int getLastReceivedOpCode(){ return m_lastReceivedOpCode;};
		void setReceivedOpCode(int opCode);
		bool popPacketReceived(int &opCode);
		int getNewPacketReceived();
		void rstNewPacketReceived();
		/*	
		void setLocalNodeID(byte node){m_localNodeID = node;};
		byte getLocalNodeID(){return m_localNodeID;};

		void setLastRemoteNodeID(byte node){ m_lastRemoteNodeID = node;};
		byte getLastRemoteNodeID(){ return m_lastRemoteNodeID;};
		*/
		bool addDataToReceiveBuffer(byte *val, int startIndex, int dim);
		/**
		@return the pointer to the received data buffer
		@see m_lastDataReceived
		*/
		byte* bufferLastDataReceived(){return m_lastDataReceived;};
		/**
		@return the number of data received. Bytes whitin the received data buffer
		*/
		int getNumDataReceived(){ return m_numDataReceived;};
		/**
		Sets the number of bytes received. Number of data within the reception data buffer. 
		@param i number of bytes
		@see bufferLastDataReceived
		*/
		void setNumDataReceived(int i){ if (i > m_bufferLastDataReceivedDimension) i = m_bufferLastDataReceivedDimension; m_numDataReceived = i;};

		
		byte calculateChecksum(byte *data, int len);

		/**
		Sets th number of packet lost
		@param i numner of packets lost
		*/
		void setNumPacketsLost(int i){ m_numPacketsLost = i;};
		/**
		@return the number of packets lost
		*/
		int getNumPacketsLost(){ return m_numPacketsLost;};

		/**
		This function has ti be overloaded from the son class.
		It's used to send messages to the board's manager
		*/
		virtual int sendMsg(int i){ return -1;};
		/**
		This function has ti be overloaded from the son class.
		It's used to get messages from the board's manager
		*/
		virtual int getMsg(){ return -1;};
		
		byte *m_lastDataReceived;
		
	protected:	
		virtual void restoreIntialCondition();
		/**
		Turns On the uart
		*/
		virtual void TurnOn(){ QueueUart::TurnOn();};
		/**
		Turns Off the uart
		*/
		virtual void TurnOff(){ QueueUart::TurnOff();};
		
		virtual void SetReceiveCallBack(void (* pcallback)(UartDeviceAbstraction *pdevice, unsigned data));

		/**
		Clears the tranmission buffer
		*/
		virtual void ClearTransmissionBuffer(){QueueUart::ClearTransmissionBuffer();};
		
		virtual void SetDataAnalyseCallBack(bool (*pcallback)(unsigned short dataS, stdUartProtocolAbstraction *protocol));
		
		virtual bool IsTransmissionAvailable();
		virtual bool IsReceptionAvailable();

		int SendCommand(unsigned char *pdata, int len, int timeOut = 200);
		/**
		Allocates the data received buffer

		@param i buffer length
		*/
		void setLastDataReceivedBuffer(int i);
		
	private:
		/**
		timeout TX in msec
		*/
		int m_timeoutTx;
		/**
		timeout RX in msec
		*/
		int m_timeoutRx;

		/**
		TX timeout timer
		*/
		DecTimer m_TxTimeoutTimer;
		/**
		RX timeout timer
		*/
		DecTimer m_RxTimeoutTimer;

		/**
		laset error occerred
		*/
		int m_lastError;

		/**
		error's fifo
		*/
		CSmallRingBuf <int, 10> m_error;

		/**
		number of bytes received
		*/
		int m_numBytesRecieved;

		/**
		packet length
		*/
		int m_packetLength;

		/**
		receiver state
		*/
		int m_receiverState;

		/**
		packets received counter
		*/
		byte m_numPacketsReceived;

		/**
		packets transmitted counter
		*/
		byte m_numPacketsTransmitted;

		/**
		last received opcode
		*/
		int m_lastReceivedOpCode;
		/**
		Received opcode buffer
		*/
		CSmallRingBuf <int , 5> m_receivedOpCode;
			
		int m_bufferLastDataReceivedDimension;

		/**
		Number of data received
		*/
		int m_numDataReceived;

		/**
		Number of packets lost
		*/
		int m_numPacketsLost;

		/**
		@return true if any timeout error occurred
		*/
		bool checkTimeOutElapsed();		

		/**
		Pointer to the function tha decode the packet. This function is defined in the sons classes according to the protocol.
		*/
		bool (*m_stdUartProtocolAbstractionDataAnalyse)(unsigned short dataS, stdUartProtocolAbstraction *protocol);
		
		void (*m_pReceiveCallback)(UartDeviceAbstraction *pdevice, unsigned data);
		static void stdUartProtocolAbstractionReceiveCallback(UartDeviceAbstraction *pdevice, unsigned data);
		static void stdUartProtocolAbstractionTransmitCallback(UartDeviceAbstraction *pdevice, bool last);
};




#endif




































