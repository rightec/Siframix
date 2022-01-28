/**
@file		QueueUart.cpp
@brief		Classes to manage the microntroller UART.

@author		Nicola
@date		18/01/2011
@version	01.1
*/


#include "QueueUart.h"

//
// Definizione della classe RingBuffer
//

/**
Class constructor
@param size size of the buffer in byte
*/
RingBuffer::RingBuffer(unsigned size)
{
	// + Crea il buffer circolare e lo inizializza
	
	m_BufferSize = size;
	m_pRingBuffer = new unsigned char[m_BufferSize];
	Clear(True);
}

/**
Class desctructor
*/
RingBuffer::~RingBuffer()
{
	// + Distrugge il buffer circolare
	
	if(m_pRingBuffer)
	{
		delete [] m_pRingBuffer;
	}
}

/**
Deletes the data within the buffer
@param reset if true all the elements into the buffer are zeroed, if false only the pointers are resets.
*/
void RingBuffer::Clear(bool reset)
{
	// + Azzera il contenuto del buffer circolare

	if(m_pRingBuffer)
	{
		if(reset)
		{
			for(unsigned counter = 0; counter < m_BufferSize; counter++)
			{
				*(m_pRingBuffer + counter) = '\0';
			}
		}
		m_pCurrent = m_pRingBuffer;
	}
}

// Inserimento dati

/**
Inserts a char in the buffer
@param data data to insert.
*/
void RingBuffer::Insert(unsigned char data)
{
	// + Inserisce un dato in fondo al buffer circolare
	
	if(m_pRingBuffer)
	{
		*m_pCurrent = data;
		m_pCurrent++;
		if(m_pCurrent == (m_pRingBuffer + m_BufferSize))
		{
			m_pCurrent = m_pRingBuffer;
		}
	}
}

/**
Checks if the last depth elements into the RingBuffer are equal to the ones in pdata.
@return true if the data are equal, false otherwise.
*/
bool RingBuffer::Equals(const unsigned char *pdata, unsigned depth)
{
	// + Confronta il dato puntato da pdata con gli ultimi depth elementi del buffer circolare

	bool IsEqual = False;

	if(m_pRingBuffer)
	{
		unsigned char *pCurrent = m_pCurrent;

		for(; depth; depth--)
		{
			if(pCurrent == m_pRingBuffer)
			{
				pCurrent = m_pRingBuffer + m_BufferSize;
			}
			pCurrent--;
			if(*(pdata + depth - 1) != *pCurrent)
			{
				break;
			}
		}
		if(!depth)
		{
			IsEqual = True;
		}
	}
	return IsEqual;
}

//
// Definizione della classe UartDeviceAbstraction
//

// Inizializzazione membri statici

UartDeviceAbstraction *UartDeviceAbstraction::m_pUart0DeviceAbstraction = 0;
UartDeviceAbstraction *UartDeviceAbstraction::m_pUart1DeviceAbstraction = 0;
UartDeviceAbstraction *UartDeviceAbstraction::m_pUart2DeviceAbstraction = 0;

/**
Class constructor
@param device UART  device
*/
UartDeviceAbstraction::UartDeviceAbstraction(UARTDevice device)
{
	// + Seleziona l'insieme di registri corretto in relazione alla UART da usare
	// + Abilita la funzionalità UART sui piedini di I/O corrispondenti
	// + Abilita l'interrupt di trasmissione con livello di priorità 7
	
	m_Device = device;
	m_DeviceOwner = False;
	m_LastError = NoErrorUART;
	m_pTransmitCallback = 0;
	m_pReceiveCallback = 0;

	switch(m_Device) {
		case UART0:
		{
			if(!m_pUart0DeviceAbstraction)
			{
				m_DeviceOwner = True;
				m_pUart0DeviceAbstraction = this;
			}
			SelectUart0();
			break;
		}
		case UART1:
		{
			if(!m_pUart1DeviceAbstraction)
			{
				m_DeviceOwner = True;
				m_pUart1DeviceAbstraction = this;
			}
			SelectUart1();
			break;
		}
		case UART2:
		{
			if(!m_pUart2DeviceAbstraction)
			{
				m_DeviceOwner = True;
				m_pUart2DeviceAbstraction = this;
			}
			SelectUart2();
			break;
		}
	}

	// + Inizilizzazioni di base
	// + Accende la periferica UART ed abilita le funzioni di ricezione e trasmissione
	TurnOff();
	Init();
	SetParity(ParityNone, True);
	SetFrameBits(Frame8Bits, True);
	SetStopBits(Stop1Bits, True);
	SetBaudRate(BR38400, True);	

	//TurnOn();
}

/**
Class desctructor
*/
UartDeviceAbstraction::~UartDeviceAbstraction()
{
	TurnOff();
	if(m_DeviceOwner)
	{
		switch(m_Device) {
			case UART0:
				m_pUart0DeviceAbstraction = 0;
				break;
			case UART1:
				m_pUart1DeviceAbstraction = 0;
				break;
			case UART2:
				m_pUart2DeviceAbstraction = 0;			
				break;
		}
	}
}

// Funzioni di accesso al livello hardware

/**
Selects the UART0. Register intialization.
*/
void UartDeviceAbstraction::SelectUart0()
{
	m_pUAnCTL0 = &UA0CTL0;
	m_pUAnCTL1 = &UA0CTL1;
	m_pUAnCTL2 = &UA0CTL2;
	m_pUAnOPT0 = &UA0OPT0;
	m_pUAnSTR = &UA0STR;
	m_pUAnRX = &UA0RX;		
	m_pUAnTX = &UA0TX;
	m_pUAnTIC = &UA0TIC;
	m_pUAnRIC = &UA0RIC;
	
	if(m_DeviceOwner)
	{
		_PMC30 = 1;
		_PFC30 = 0;
		_PMC31 = 1;
		_PFC31 = 0;
			
		irq_setVect(IRQ_VEC_INTUA0T_INTCB4T, *IrqHandUA0T);
		*m_pUAnTIC = 0x07;
		irq_setVect(IRQ_VEC_INTUA0R_INTCB4R, *IrqHandUA0R);
		*m_pUAnRIC = 0x07;
	}
}

/**
Selects the UART1. Register intialization.
*/
void UartDeviceAbstraction::SelectUart1()
{
	m_pUAnCTL0 = &UA1CTL0;
	m_pUAnCTL1 = &UA1CTL1;
	m_pUAnCTL2 = &UA1CTL2;
	m_pUAnOPT0 = &UA1OPT0;
	m_pUAnSTR = &UA1STR;
	m_pUAnRX = &UA1RX;		
	m_pUAnTX = &UA1TX;
	m_pUAnTIC = &UA1TIC;
	m_pUAnRIC = &UA1RIC;
	
	if(m_DeviceOwner)
	{
		_PMC90 = 1;
		_PFCE90 = 1;			
		_PFC90 = 0;
		_PMC91 = 1;
		_PFCE91 = 1;			
		_PFC91 = 0;
		
		irq_setVect(IRQ_VEC_INTUA1T, *IrqHandUA1T);
		*m_pUAnTIC = 0x07;
		irq_setVect(IRQ_VEC_INTUA1R_INTIIC2, *IrqHandUA1R);
		*m_pUAnRIC = 0x00;
	}
}

/**
Selects the UART2. Register intialization.
*/
void UartDeviceAbstraction::SelectUart2()
{
	m_pUAnCTL0 = &UA2CTL0;
	m_pUAnCTL1 = &UA2CTL1;
	m_pUAnCTL2 = &UA2CTL2;
	m_pUAnOPT0 = &UA2OPT0;
	m_pUAnSTR = &UA2STR;
	m_pUAnRX = &UA2RX;		
	m_pUAnTX = &UA2TX;
	m_pUAnTIC = &UA2TIC;
	m_pUAnRIC = &UA2RIC;

	if(m_DeviceOwner)
	{
		_PMC38 = 1;
		_PFC38 = 0;
		_PMC39 = 1;
		_PFC39 = 0;			

		irq_setVect(IRQ_VEC_INTUA2T, *IrqHandUA2T);
		*m_pUAnTIC = 0x07;
		irq_setVect(IRQ_VEC_INTUA2R_INTIIC0, *IrqHandUA2R);
		*m_pUAnRIC = 0x00;
	}
}

/**
Sets the UART defaults values.
*/
void UartDeviceAbstraction::Init()
{
	// + Imposta i parametri di startup della UART

	if(m_DeviceOwner)
	{
		*m_pUAnSTR = 0x00;
		while(!IsTransmissionBufferEmpty());

		*m_pUAnCTL0 = 0x10;
		*m_pUAnOPT0 = 0x14;
	}
}

/**
Enables the UART device
*/
void UartDeviceAbstraction::TurnOn()
{
	// + Accende il modulo UART e abilita trasmissione e ricezione

	if(m_DeviceOwner)
	{
		*m_pUAnCTL0 |= 0x80;
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		*m_pUAnCTL0 |= 0x40;
		asm("nop");
		asm("nop");
		*m_pUAnCTL0 |= 0x20;
		asm("nop");
		asm("nop");
	}
}

/**
Disables UART device.
*/
void UartDeviceAbstraction::TurnOff()
{
	// + Spegne il modulo UART e disabilita trasmissione e ricezione

	if(m_DeviceOwner)
	{
		*m_pUAnCTL0 &= 0xbf;
		asm("nop");
		asm("nop");
		*m_pUAnCTL0 &= 0xdf;
		asm("nop");
		asm("nop");
		asm("nop");
		asm("nop");
		*m_pUAnCTL0 &= 0x7f;
		asm("nop");
		asm("nop");
	}
}

/**
Check if the UART transmission buffer is empty (uC register)
@return true if the uC transmission register is empty
*/
bool UartDeviceAbstraction::IsTransmissionBufferEmpty()
{
	// + Controlla che il buffer di trasmissione sia vuoto
	if(m_DeviceOwner)
	{
		return !((*m_pUAnSTR) & 0x80);
	}
	return True;
}

/**
Sends a byte to the transmission buffer (uC register)
@param data value to send
*/
void UartDeviceAbstraction::SetTransmissionBuffer(unsigned char data)
{
	// + Invia un byte al buffer di trasmissione

	if(m_DeviceOwner)
	{
		*m_pUAnTX = data;
	}
}

/**
@return if any UART hardware error occurred.
*/
unsigned UartDeviceAbstraction::GetReceptionError()
{
	// + Determina l'esito della ricezione seriale

	unsigned error = NoDevice;

	if(m_DeviceOwner)
	{
		error = NoErrorUART;
		if((*m_pUAnSTR) & 0x04)
		{
			error |= ParityError;
		}
		if((*m_pUAnSTR) & 0x02)
		{
			error |= FrameError;
		}
		if((*m_pUAnSTR) & 0x01)
		{
			error |= OverrunError;
		}
		if (error != NoErrorUART)
			//unsigned data = GetReceptionBuffer();
			GetReceptionBuffer();
		(*m_pUAnSTR) &= 0xf8;
		//(*m_pUAnSTR) &= data;
	}
	return error;
}

/**
@return the data into the reception buffer (uC register)
*/
unsigned char UartDeviceAbstraction::GetReceptionBuffer()
{
	if(m_DeviceOwner)
		return *m_pUAnRX;
	return 0;
}

/**
ISR UART0 Transmission
*/
void UartDeviceAbstraction::IrqHandUA0T(int vector)
{
	// + Se registrata chiama la callback di trasmissione

	if(m_pUart0DeviceAbstraction)
	{
		if(m_pUart0DeviceAbstraction->m_pTransmitCallback)
		{
			m_pUart0DeviceAbstraction->m_pTransmitCallback(m_pUart0DeviceAbstraction);
		}
	}
}

/**
ISR UART1 Transmission
*/
void UartDeviceAbstraction::IrqHandUA1T(int vector)
{
	// + Se registrata chiama la callback di trasmissione

	if(m_pUart1DeviceAbstraction)
	{
		if(m_pUart1DeviceAbstraction->m_pTransmitCallback)
		{
			m_pUart1DeviceAbstraction->m_pTransmitCallback(m_pUart1DeviceAbstraction);
		}

	}
}
/**
ISR UART2 Transmission
*/
void UartDeviceAbstraction::IrqHandUA2T(int vector)
{
	// + Se registrata chiama la callback di trasmissione

	if(m_pUart2DeviceAbstraction)
	{
		if(m_pUart2DeviceAbstraction->m_pTransmitCallback)
		{
			m_pUart2DeviceAbstraction->m_pTransmitCallback(m_pUart2DeviceAbstraction);
		}
	}
}

/**
ISR UART0 Reception
*/
void UartDeviceAbstraction::IrqHandUA0R(int vector)
{
	// + Se registrata chiama la callback di ricezione
	unsigned data;
	if(m_pUart0DeviceAbstraction)
	{
		if(m_pUart0DeviceAbstraction->m_pReceiveCallback)
		{
			data = m_pUart0DeviceAbstraction->ReceiveData();
			
			m_pUart0DeviceAbstraction->m_pReceiveCallback(m_pUart0DeviceAbstraction, data);
		}else
		{
			//data = m_pUart0DeviceAbstraction->ReceiveData();
		}
	}else
	{
		//data = m_pUart0DeviceAbstraction->ReceiveData();
	}
}

/**
ISR UART1 Reception
*/
void UartDeviceAbstraction::IrqHandUA1R(int vector)
{
	// + Se registrata chiama la callback di ricezione
	//PinLED1_LIFE_M = 1; 
	unsigned data;

	if(m_pUart1DeviceAbstraction)
	{
		if(m_pUart1DeviceAbstraction->m_pReceiveCallback)
		{
			data = m_pUart1DeviceAbstraction->ReceiveData();

			m_pUart1DeviceAbstraction->m_pReceiveCallback(m_pUart1DeviceAbstraction, data);
		}else
		{
			//data = m_pUart1DeviceAbstraction->ReceiveData();
		}
	}else
	{
		//data = m_pUart1DeviceAbstraction->ReceiveData();
	}
	//PinLED1_LIFE_M = 0; 	
}

/**
ISR UART2 Reception
*/
void UartDeviceAbstraction::IrqHandUA2R(int vector)
{
	// + Se registrata chiama la callback di ricezione
	unsigned data;
	if(m_pUart2DeviceAbstraction)
	{
		if(m_pUart2DeviceAbstraction->m_pReceiveCallback)
		{
			data = m_pUart2DeviceAbstraction->ReceiveData();

			m_pUart2DeviceAbstraction->m_pReceiveCallback(m_pUart2DeviceAbstraction, data);
		}else
		{
			//data = m_pUart2DeviceAbstraction->ReceiveData();
		}
	}else
	{
		//data = m_pUart2DeviceAbstraction->ReceiveData();
	}
}


/**
Set the Parity of the UART protocol. 
@param parity king of parity
@param forcemodify if true the property value is overwritten also if equal to the previous one.
*/
void UartDeviceAbstraction::SetParity(UARTParity parity, bool forcemodify)
{
	if(m_DeviceOwner)
	{
		if(parity != m_Parity || forcemodify) {
			m_Parity = parity;
			*m_pUAnCTL0 &= 0xf3;
			switch(m_Parity)
			{
				case ParityNone:
					*m_pUAnCTL0 |= 0x00;
					break;
				case ParityZero:
					*m_pUAnCTL0 |= 0x04;
					break;
				case ParityOdd:
					*m_pUAnCTL0 |= 0x08;
					break;
				case ParityEven:
					*m_pUAnCTL0 |= 0x0c;
					break;
			}
		}
	}
}

/**
Set the number of bits per frame. 
@param framebits number of bit per frame
@param forcemodify if true the property value is overwritten also if equal to the previous one.
*/
void UartDeviceAbstraction::SetFrameBits(UARTFrameBits framebits, bool forcemodify)
{
	if(m_DeviceOwner)
	{
		if(framebits != m_FrameBits || forcemodify) {
			m_FrameBits = framebits;
			*m_pUAnCTL0 &= 0xfd;
			switch(m_FrameBits)
			{
				case Frame7Bits:
					*m_pUAnCTL0 |= 0x00;
					break;
				case Frame8Bits:
					*m_pUAnCTL0 |= 0x02;
					break;
			}
		}
	}
}

/**
Set the number of stop bits. 
@param stopbits number of stop bits
@param forcemodify if true the property value is overwritten also if equal to the previous one.
*/
void UartDeviceAbstraction::SetStopBits(UARTStopBits stopbits, bool forcemodify)
{
	if(m_DeviceOwner)
	{
		if(stopbits != m_StopBits || forcemodify) {
			m_StopBits = stopbits;
			*m_pUAnCTL0 &= 0xfe;	
			switch(m_StopBits)
			{
				case Stop1Bits:
					*m_pUAnCTL0 |= 0x00;
					break;
				case Stop2Bits:
					*m_pUAnCTL0 |= 0x01;
					break;
			}
		}
	}
}

/**
Set the buad rate. 
@param baudrate
@param forcemodify if true the property value is overwritten also if equal to the previous one.
*/
void UartDeviceAbstraction::SetBaudRate(UARTBaudRate baudrate, bool forcemodify)
{
	if(m_DeviceOwner)
	{
		if(baudrate != m_BaudRate || forcemodify) {

			m_BaudRate = baudrate;
			switch(m_BaudRate)
			{
				case BR1200:
					*m_pUAnCTL1 = 0x06;
					*m_pUAnCTL2 = 0x82;
					break;
				case BR2400:
					*m_pUAnCTL1 = 0x05;
					*m_pUAnCTL2 = 0x82;
					break;
				case BR4800:
					*m_pUAnCTL1 = 0x04;
					*m_pUAnCTL2 = 0x82;
					break;
				case BR9600:
					*m_pUAnCTL1 = 0x03;
					*m_pUAnCTL2 = 0x82;
					break;
				case BR19200:
					*m_pUAnCTL1 = 0x02;
					*m_pUAnCTL2 = 0x82;
					break;
				case BR38400:
					*m_pUAnCTL1 = 0x01;
					*m_pUAnCTL2 = 0x82;
					break;
				case BR76800:
					*m_pUAnCTL1 = 0x00;
					*m_pUAnCTL2 = 0x82;
					break;
				case BR115200:
					*m_pUAnCTL1 = 0x00;
					*m_pUAnCTL2 = 0x57; // error (%) = -0.2234%
					break;
				case BR230400:
					*m_pUAnCTL1 = 0x00;
					*m_pUAnCTL2 = 0x2B; // error (%) = +0.937% 
					break;
			}
		}
	}
}

// Gestione della trasmissione

/**
Lock the transmission disabling the interrupts
*/
void UartDeviceAbstraction::TransmitLock()
{
	// + Disabilita l'interruzione sulla trasmissione Uart

	if(m_DeviceOwner)
	{
		asm ( "di" );
		//*m_pUAnTIC |= 0x40;
		//asm	( "ei" );
	}
}

/**
UnLock the transmission enabling the interrupts
*/
void UartDeviceAbstraction::TransmitUnlock()
{
	// + Abilita l'interruzione sulla trasmissione Uart

	if(m_DeviceOwner)
	{
		//asm ( "di" );
		//*m_pUAnTIC &= ~0x40;
		asm ( "ei" );
	}
}

/**
Starts a transmission
*/
void UartDeviceAbstraction::StartTransmission()
{
	// + Avvia la trasmissione dei dati presenti nel buffer
	if(m_DeviceOwner)
	{
		asm ( "di" );
		*m_pUAnTIC |= 0x80;
		asm ( "ei" );	
	}
}

/**
Starts a transmission writing data in the transmission buffer
*/
void UartDeviceAbstraction::StartTransmission(unsigned char data)
{
	SetTransmissionBuffer(data);
}

/**
Transmits a byte. This is a blocking function, it waits that the previous transmission ends.
@ data byte to transmit
*/
void UartDeviceAbstraction::TransmitData(unsigned char data)
{
	// + Scrive un carattere nel buffer di trasmissione

	if(m_DeviceOwner)
	{
		while(!IsTransmissionBufferEmpty());
		SetTransmissionBuffer(data);
	}
}

/**
Clears the uart hardware error.
*/
void UartDeviceAbstraction::ClearError()
{
	m_LastError = NoErrorUART;
}

/**
sets the transmission call back
*/
void UartDeviceAbstraction::SetTransmitCallBack(void (* pcallback)(UartDeviceAbstraction *pdevice))
{
	// + Imposta la funzione di callback da invocare ogni qualvolta il buffer di trasmissione è vuoto

	m_pTransmitCallback = pcallback;
}

// Gestione della ricezione

/**
Locks the reception disabling the interrupts
*/
void UartDeviceAbstraction::ReceiveLock()
{
	// + Disabilita l'interruzione sulla ricezione Uart

	if(m_DeviceOwner)
	{
		asm ( "di" );
		//*m_pUAnRIC |= 0x40;
		//asm ( "ei" );
	}
}

/**
UnLocks the reception enabling the interrupts
*/
void UartDeviceAbstraction::ReceiveUnlock()
{
	// + Abilita l'interruzione sulla ricezione Uart

	if(m_DeviceOwner)
	{
		//asm ( "di" );
		//*m_pUAnRIC &= ~0x40;
		asm	( "ei" );		
	}
}

/**
Reads a byte from the recption buffer (uC register)
@return the read byte if no error occurred, ReceptionError otherwise
*/
unsigned UartDeviceAbstraction::ReceiveData()
{
	// + Legge un carattere dal buffer di ricezione.
	//   Ritorna ReceptionError in caso di errore.
	
	unsigned data = ReceptionError;

	if(m_DeviceOwner)
	{
		data = GetReceptionError();
		if(data == NoErrorUART)
		{
			data |= (unsigned) GetReceptionBuffer();
		}
	}
	return data;
}

/**
sets the receivce call back
*/
void UartDeviceAbstraction::SetReceiveCallBack(void (* pcallback)(UartDeviceAbstraction *pdevice, unsigned data))
{
	// + Imposta la funzione di callback da invocare ogni qualvolta il buffer di ricezione è pieno

	m_pReceiveCallback = pcallback;
}

//
// Definizione della classe QueueUart
//

// Costruttori e Distruttori
/**
Class constructor
@param device UART  device
*/
QueueUart::QueueUart(UARTDevice device) :
	UartDeviceAbstraction(device)
{
	// + Imposta i gestori delle code di trasmissione e ricezione

	m_TransmitLockDepth = 0;
	m_ReceiveLockDepth = 0;
	m_RunningTransmission = False;
	rstTransmitBufferOverflow();
	rstReceptionBufferOverflow();
	UartDeviceAbstraction::SetTransmitCallBack(&TransmitCallback);
	UartDeviceAbstraction::SetReceiveCallBack(&ReceiveCallback);
}

/**
Class desctructor
*/
QueueUart::~QueueUart()
{
}

/**
Start a transmission
*/
void QueueUart::StartTransmission()
{
	unsigned char data;
	if (!m_TransmissionBuffer.empty() && !m_RunningTransmission)
	{	
		asm("di");
		m_TransmissionBuffer.pop_front(data);
		asm("ei");
		UartDeviceAbstraction::StartTransmission(data);
	}
}
// Gestione della coda di trasmissione

/**
Locks the transmission
*/
void QueueUart::TransmitLock()
{
	// + Blocca l'accesso alla coda di trasmissione gestendo l'annidamento

	if(!m_TransmitLockDepth)
	{
		UartDeviceAbstraction::TransmitLock();
	}
	if(m_TransmitLockDepth + 1)
	{
		m_TransmitLockDepth++;
	}

	m_TransmissionBuffer.clear();
}

/**
Unlocks the transmission
*/
void QueueUart::TransmitUnlock()
{
	// + Ripristina l'accesso alla coda di trasmissione gestendo l'annidamento

	if(m_TransmitLockDepth)
	{
		m_TransmitLockDepth--;
	}
	if(!m_TransmitLockDepth)
	{
		UartDeviceAbstraction::TransmitUnlock();
	}
}

/**
Transmits a data.

- Locks the transmission disabling the interrupts.
- if the fifo is empty the data is immediately transmitted
- if the fifo is not empry the dato is pushed into the fifo.
- if the fifo is full sets the transmitt buffer overflow error
*/
void QueueUart::TransmitData(unsigned char data)
{
	// + Esegue il lock della coda di trasmissione, disabilitando le interruzioni
	// + Se la coda è vuota, trasmette il dato immediatamente
	// + Se la coda non è vuota, accoda il dato da trasmettere
	
	if(!m_TransmissionBuffer.full())
	{
		asm("di");
		m_TransmissionBuffer.push_back(data);
		asm("ei");
	}else
	{
		m_TransmitBufferOverflow = True;
	}
}

/**
Transmits count data from data. 
*/
void QueueUart::TransmitData(const unsigned char *pdata, int count)
{
	// + Trasmette una sequenza di dati di lunghezza count
	while(count--)
	{
		TransmitData(*pdata);
		pdata++;
	}
}

/**
Transmits the pdata string. 
*/
void QueueUart::TransmitData(const unsigned char *pdata)
{
	// + Trasmette una stringa terminata con zero

	TransmitData(pdata, strlen((const char*) pdata));
}

/**
@return the transmission fifo size (It's not how many elements are present into the fifo).
@see GetNumBytesInTransmissionBuffer()
*/
int QueueUart::GetTransmissionBufferSize()
{
	// + Ritorna la dimensione del buffer di trasmissione
	return BUFFER_SIZE;
}

/**
@return how many elements are present into the transmission fifo 
*/
int QueueUart::GetNumBytesInTransmissionBuffer()
{
	// + Ritorna i caratteri presenti nella coda di trasmissione ancora da inviare

	asm("di");
	int itemsToWrite = m_TransmissionBuffer.numItem();
	asm("ei");

	return itemsToWrite;
}

/**
@return the free space into the transmission fifo
*/
int QueueUart::GetFreeBytesInTransmissionBuffer()
{
	
	return GetTransmissionBufferSize()- GetNumBytesInTransmissionBuffer();
}

/**
Transmitt call back
*/
void QueueUart::TransmitCallback(UartDeviceAbstraction *pdevice)
{
	// + Se la coda di trasmissione non è vuota, invia il prossimo dato.

	QueueUart *pUart = (QueueUart *) pdevice;
	
	if(!pUart->m_TransmissionBuffer.empty())
	{
		unsigned char data;
		//asm("di");
		pUart->m_TransmissionBuffer.pop_front(data);
		//asm("ei");
		pUart->UartDeviceAbstraction::TransmitData(data);		
	}
	else
	{
		//PinEnRS485_DE = 0;	// libera la rete per la risposta del modulo giusto
		pUart->m_RunningTransmission = False;
	}
	if(pUart->m_pTransmitCallback)
	{
		pUart->m_pTransmitCallback(pdevice, !pUart->m_RunningTransmission);
	}	
}

/**
sets the transmit call back
*/
void QueueUart::SetTransmitCallBack(void (* pcallback)(UartDeviceAbstraction *pdevice, bool last))
{
	// + Imposta la funzione di callback da invocare ogni qualvolta il buffer di trasmissione è vuoto

	m_pTransmitCallback = pcallback;
}

// Gestione della coda di ricezione

/**
Locks the reception 
*/
void QueueUart::ReceiveLock()
{
	// + Blocca l'accesso alla coda di ricezione gestendo l'annidamento
	
	if(!m_ReceiveLockDepth)
	{
		UartDeviceAbstraction::ReceiveLock();
	}
	if(m_ReceiveLockDepth + 1)
	{
		m_ReceiveLockDepth++;
	}
}

/**
Unlock the receptions
*/
void QueueUart::ReceiveUnlock()
{
	// + Ripristina l'accesso alla coda di ricezione gestendo l'annidamento

	if(m_ReceiveLockDepth)
	{
		m_ReceiveLockDepth--;
	}
	if(!m_ReceiveLockDepth)
	{
		UartDeviceAbstraction::ReceiveUnlock();
	}
}

/**
Creal the reception fifo.
*/
void QueueUart::ClearReceptionBuffer()
{
	// + Svuota la coda di ricezione
	asm("di");
	m_ReceptionBuffer.clear();
	asm("ei");
}

/**
Clear the transmission fifo
*/
void QueueUart::ClearTransmissionBuffer()
{
	// + Svuota la coda di ricezione
	asm("di");
	m_TransmissionBuffer.clear();
	m_RunningTransmission = False;
	asm("ei");
}

/**
if the reception fifo is not empty pops a data from the fifo.
@param data data read.
@return true if the fifo is not empty, false otherwise
*/
bool QueueUart::ReceiveData(unsigned short &data)
{
	// + Se la coda di ricezione non è vuota, estrae un elemento e lo mette in data
	// + Se la coda di ricezione è vuota, ritorna false
	
	bool dataIsReceived = False;

	if(!m_ReceptionBuffer.empty())
	{
		asm("di");
		m_ReceptionBuffer.pop_front(data);
		asm("ei");
		dataIsReceived = True;
	}
	return dataIsReceived;
}

/**
Tries to read count elements from the reception fifo. 
@param pdata array of the read unsigned short
@param count number of unsigned short to read
@return true if count elements were read false otherwise. Anyway count contains the number of unsigned short read.
*/
bool QueueUart::ReceiveData(unsigned short *pdata, int &count)
{
	// + Tenta di estrarre count elementi dalla coda di ricezione
	// + Se non esistono count elementi nella coda di ricezione, ritorna false
	// + In ogni caso, count contiene in uscita il numero di elementi estratti dalla coda di ricezione

	bool dataIsReceived = False;

	int counter = count;
	
	while(counter)
	{
		if(!ReceiveData(*pdata))
		{
			break;
		}
		counter--;
		pdata++;
	}
	if(!counter)
	{
		dataIsReceived = True;
	}
	count = count - counter;

	return dataIsReceived;	
}

/**
if the reception fifo is not empty pops a data from the fifo.
@param data data read.
@return true if the fifo is not empty, false otherwise
*/
bool QueueUart::ReceiveData(unsigned char &data)
{
	// + Se la coda di ricezione non è vuota, estrae un elemento e lo mette in data
	// + Se la coda di ricezione è vuota, ritorna false
	
	bool dataIsReceived = False;
	unsigned short dataShort;

	if(!m_ReceptionBuffer.empty())
	{
		asm("di");
		m_ReceptionBuffer.pop_front(dataShort);
		asm("ei");
		data = (unsigned char) dataShort;
		dataIsReceived = True;
	}
	return dataIsReceived;
}

/**
Tries to read count elements from the reception fifo. 
@param pdata array of the read bytes
@param count number of unsigned bytes
@return true if count elements were read false otherwise. Anyway count contains the number of bytes read.
*/
bool QueueUart::ReceiveData(unsigned char *pdata, int &count)
{
	// + Tenta di estrarre count elementi dalla coda di ricezione
	// + Se non esistono count elementi nella coda di ricezione, ritorna false
	// + In ogni caso, count contiene in uscita il numero di elementi estratti dalla coda di ricezione

	bool dataIsReceived = False;

	//ReceiveLock();
	int counter = count;
	
	while(counter)
	{
		if(!ReceiveData(*pdata))
		{
			break;
		}
		counter--;
		pdata++;
	}
	if(!counter)
	{
		dataIsReceived = True;
	}
	count = count - counter;
	//ReceiveUnlock();
	return dataIsReceived;	
}

/**
Sets the receive call back
*/
void QueueUart::SetReceiveCallBack(void (* pcallback)(UartDeviceAbstraction *pdevice, unsigned data))
{
	// + Imposta la funzione di callback da invocare ogni qualvolta il buffer di ricezione è pieno

	m_pReceiveCallback = pcallback;
}

/**
@return the reception fifo size (It's not how many elements are present into the fifo).
@see GetNumBytesInReceptionBuffer()
*/
int QueueUart::GetReceptionBufferSize()
{
	// + Ritorna la dimensione del byte di ricezione
	return BUFFER_SIZE;	
}

/**
@return thow many elements are present into the fifo).
@see GetNumBytesInReceptionBuffer()
*/
int QueueUart::GetNumBytesInReceptionBuffer()
{
	// + Ritorna i caratteri presenti nella coda di ricezione

	asm("di");
	int itemsToRead = m_ReceptionBuffer.numItem();
	asm("ei");

	return itemsToRead;	
}

/**
@return the number of free bytes into the reception fifo
*/
int QueueUart::GetFreeBytesInReceptionBuffer()
{
	
	return GetReceptionBufferSize()- GetNumBytesInReceptionBuffer();
}

/**
Clears the transmission and reception fifo
*/
void QueueUart::resetFifo()
{
	ClearReceptionBuffer();
	ClearTransmissionBuffer();
}

/**
Receive call back
*/
void QueueUart::ReceiveCallback(UartDeviceAbstraction *pdevice, unsigned data)
{
	// + Se la coda di ricezione non è piena e non si sono verificati errori, memorizza il dato ricevuto

	QueueUart *pUart = (QueueUart *) pdevice;

	if(!pUart->m_ReceptionBuffer.full())
	{
		pUart->m_ReceptionBuffer.push_back((unsigned short) (data & 0xffff));
	}
	else
	{
		pUart->m_ReceptionBufferOverflow = True;
	}
		
	/*if(!pUart->m_ReceptionBuffer.full() && !(data & 0xff00))
	{
		pUart->m_ReceptionBuffer.push_back((unsigned char) (data & 0xff));
	}*/
	if(pUart->m_pReceiveCallback)
	{
		pUart->m_pReceiveCallback(pdevice, data);
	}
}
















































