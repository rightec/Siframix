/**
@file		Adc.cpp
@brief		Functions to handle internal and extern ADC.

			The internal adc samples the board current consumption and voltage. The external adc samples the impedance values.
			Here are dcumented only the functions not present in the file Adc.h 
@see Adc.h
@author		Francesco Fregni
@date		01/03/2011
@version	01.00
*/

#include "adc.h"

/**
Internal ADC Interrupt Service Routine (ISR).

@param vector interrupt vector number.
*/
static void IrqHandADC(int vector);

//------------- External ADC --------------------------------------
static void IrqHandSIO1_R(int vector);
/**
SPI2 data received Interrupt Service Routine (ISR).

@param vector interrupt vector number.
*/
static void IrqHandSIO2_R(int vector);

/**
TMQ0 Interrupt Service Routine (ISR).

Hardware timer uses as time base for impedance sampling.
@param vector interrupt vector number.
*/
static void IrqHandTMQ0(int vector);

/**
INTP6 Interrupt Service Routine (ISR).

This ISR received the signal of impedance measure overcurrent.
@param vector interrupt vector number.
*/
//static void IrqHandINTP6(int vector);

/**
Power voltage samples' buffer. 
*/
CSmallRingBuf <short, 20> mV_Power_ADC;

//CSmallRingBuf <unsigned int, DIM_BYTE_FIFOADC> ADC_serial_fifo;
CSmallRingBuf <int, DIM_BYTE_FIFOADC> ADC1_serial_fifo;//CSmallRingBuf <unsigned int, DIM_BYTE_FIFOADC> ADC1_serial_fifo;
CSmallRingBuf <int, DIM_BYTE_FIFOADC> ADC2_serial_fifo;//CSmallRingBuf <unsigned int, DIM_BYTE_FIFOADC> ADC2_serial_fifo;
CSmallRingBuf <int, DIM_BYTE_FIFOADC> ADC3_serial_fifo;
CSmallRingBuf <int, DIM_BYTE_FIFO> CS5530_tx_fifo;
CSmallRingBuf <int, DIM_BYTE_FIFO> CS5530_rx_fifo;
CSmallRingBuf <int, DIM_BYTE_FIFOADC> ADC4_serial_fifo;
CSmallRingBuf <int, DIM_BYTE_FIFOADC> ADC5_serial_fifo;
CSmallRingBuf <int, DIM_BYTE_FIFO> CS5530_tx_fifoSpi2;
CSmallRingBuf <int, DIM_BYTE_FIFO> CS5530_rx_fifoSpi2;

#ifdef __DEBUG_ADC
int adc1_data[1800];
int adc2_data[1800];
int adcfilt_data[1800];
bool acquisition = False;
//int contadc1 = 0;
int contadc2 = 0;
#endif
int contadc1 = 0;
//-------------------------------------------------------
//Variabili Globali
//-------------------------------------------------------
//int debug_tx_data;
//int debug_rx_data;
//dword med_val;
/**
Struct of channel's parameter, one for every fisical weight channel
*/
ChannelsBackupParam Chan[_MAX_LOAD_CHAN_];
/**
Sample time of power voltage and current consumption in ms.
*/
int powerSamplePeriod;
unsigned long pwrVoltageAverage;
byte numPwrSamples;
//word value_chan1;
/**
External adc state. Used in the ISR IrqHandSIO2_R. If the acquisition is stopped it has to be resetted. 
*/
volatile word state; 

/**
External adc value. Used in the ISR IrqHandSIO2_R. If the acquisition is stopped it has to be resetted. 
*/
volatile word state_adc1;
volatile word state_adc2;
volatile word state_adc3;
volatile word state_adc4;
volatile word state_adc5;
volatile word chanToSample;
volatile word vol_tick;
//-------------------------------------------------------
// Costanti statiche utili solo a questo modulo
//-------------------------------------------------------
enum CS5530_TxStates
{
	CS5530_TxState_idle = 0,
	CS5530_TxState_busy,	
};

enum CS5530_adcStates
{
	CS5530_initializing = 0,
	CS5530_commandMode,
	CS5530_pre_continuousConversionMode,
	CS5530_singleConversionMode,	
	CS5530_continuousConversionMode,
	CS5530_writeGainRegister
};
enum CS5530_initializationSequenceState
{
	CS5530_writingSync1AndSync0 = 0,
	CS5530_waitTowriteSync1andSync0,	
	CS5530_setRstBit = 2,
	CS5530_waitTosetRstBit,
	CS5530_wait8clk,
	CS5530_waitWait8Clk = 5,
	CS5530_rstRstBit,
	CS5530_waitToRstRstBit,
	CS5530_readConfRegFirstTime = 8,
	CS5530_waitToReadConfRegFirstTime,
	CS5530_writeConfRef,
	CS5530_waitToWriteConfRef = 11,
	CS5530_readConfRegSecondTime,
	CS5530_waitToReadConfSecondTime = 13,
	CS5530_waitToWriteGainRef,
	CS5530_wait_minimun_delay,
};

enum CS5530_chipselects_controll{ ADC_A = 1,
			ADC_B,
			ADC_C,
			ALL_ADC,
		};

byte m_confReg_PowerSave;
byte m_confReg_PowerDown;
byte m_confReg_RstSys;
byte m_confReg_RstValid;
byte m_confReg_InputShort;
byte m_confReg_VoltageRef;
byte m_confReg_LatchBit;
byte m_confReg_WordRate;
byte m_confReg_UniBiPolar;
byte m_confReg_OpenCircuitDetect;
byte m_confReg_filterRate;
dword m_confReg;
byte m_confRegStatus;
byte m_TxState;
byte m_CommandSent;
byte m_numByteReceived;
byte m_numByteToReceived;
byte m_adcStates;
bool m_endContinuousConversion;
byte m_initializationSequenceState;
byte m_txBufferEmpty;
byte m_spiOrPinMode;
byte m_startContinuousConversion;
dword m_lastConfRegWrote; 
byte m_validDataPresentInADCBuffer;
DecTimer timerSpi;
DecTimer watchdog_adc_timer;
byte m_num_max_adc_for_channel;
byte numByteSent;
byte adc_to_sample;

// ***** seconda nseriale validi per M3100
byte m_confRegSpi2_PowerSave;
byte m_confRegSpi2_PowerDown;
byte m_confRegSpi2_RstSys;
byte m_confRegSpi2_RstValid;
byte m_confRegSpi2_InputShort;
byte m_confRegSpi2_VoltageRef;
byte m_confRegSpi2_LatchBit;
byte m_confRegSpi2_WordRate;
byte m_confRegSpi2_UniBiPolar;
byte m_confRegSpi2_OpenCircuitDetect;
//byte m_sampleFrequencySpi2;
byte m_confRegSpi2_filterRate;
dword m_confRegSpi2;
byte m_confRegStatusSpi2;
byte m_TxStateSpi2;
byte m_CommandSentSpi2;
byte m_numByteReceivedSpi2;
byte m_numByteToReceivedSpi2;
byte m_adcStatesSpi2;
bool m_endContinuousConversionSpi2;
byte m_initializationSequenceStateSpi2;
byte m_txBufferEmptySpi2;
byte m_spiOrPinModeSpi2;
byte m_startContinuousConversionSpi2;
dword m_lastConfRegSpi2Wrote; 
byte m_validDataPresentInADCBufferSpi2;
DecTimer timerSpi2;
byte m_num_max_adc_for_channelSpi2;
byte adc_to_sampleSpi2;

//-------------------------------------------------------
// Variabili statiche utili solo a questo modulo
//-------------------------------------------------------

DecTimer wait_reset_ADC;
//-------------------------------------------------------
// Variabili extern
//-------------------------------------------------------

//------------- Internal ADC --------------------------------------
/**
Internal adc initialization.
*/
void adc_uC_init()
{
	irq_setVect(IRQ_VEC_INTAD, IrqHandADC);
	
	_PM70 = 1;
	_PM71 = 1;
	_PM72 = 1;

	ADA0S = 0x00;
	ADA0M2 = 0x00;
	ADA0M1 = 0x04;
	ADA0M0 = 0x20; 
	ADIC = 0x07;
	
	powerSamplePeriod = 1000;
	pwrVoltageAverage = 0;
	numPwrSamples = 0;
}

/**
Current cunsumption and voltage time scheduler.

This function enable the intern adc to sample the current consumption and voltage. This function is called within the uC time base interrupt.
*/
void samplePower()
{
	static int milliSec = 0;
	milliSec++;
	if (milliSec >= powerSamplePeriod)
	{
		milliSec = 0;
		_ADA0CE = 1;
	}
}

void IrqHandADC(int vector)
{
	unsigned short adc0, adc1;
	adc0 = (ADA0CR0 >>6);
	adc1 = (ADA0CR1 >>6);

	pwrVoltageAverage += adc0;
	numPwrSamples++;
	if(numPwrSamples >= 16)
	{
		pwrVoltageAverage = pwrVoltageAverage / numPwrSamples;  		
		powerVoltageChan->pushAdcDataToChan(0, adc0);
		pwrVoltageAverage = 0;
		numPwrSamples = 0;
		_ADA0CE = 0;
	}		
}

//------------- External Adc --------------------------------------
/**
External serial adc initialization.
*/
void adc_serial_init()
{
	state = 0;

	ioPinCSadc1 = 0;
	ioPinCSadc2 = 0;
	ioPinCSadc3 = 0;
	ioPinCSadc4 = 0;
	ioPinCSadc5 = 0;
	
	CS_ADC1_HIGH();
	CS_ADC2_HIGH();	
	CS_ADC3_HIGH();
	CS_ADC4_HIGH();	
	CS_ADC5_HIGH();

	//------ SPI1 -ADC1-ADC2 -ADC3---------
	irq_setVect(IRQ_VEC_INTCB1R, IrqHandSIO1_R);

	CB1CTL0 = 0x00;			// al momento niente in tutto
	CB1CTL1 = 0x1B;	//0x03;			// SCK = fsys/16 = 1,25 MHz 
	CB1CTL2 = 0x00;			// 8 bits data tramsfert	
	CB1TIC = 0x46;					// imposta irq off
	CB1RIC = 0x02;					// imposta irq on
							// Configurazione CSI1
	setSPI1mode();
	
	//------ SPI2 -ADC4 -ADC5 -----------------
	irq_setVect(IRQ_VEC_INTCB2R, IrqHandSIO2_R);

	CB2CTL0 = 0x00;			// SCK = fsys/16 = 1,25 MHz 
	CB2CTL1 = 0x1B;			// 8 bits data tramsfert
	CB2CTL2 = 0x00;			// non ora
	CB2TIC = 0x47;					// imposta irq off
	CB2RIC = 0x02;					// imposta irq on
							// Configurazione CSI2
	setSPI2mode();
	
	//---------- TMQ0 --------				
	irq_setVect(IRQ_VEC_INTTQ0CC0, IrqHandTMQ0);

	//-- Counter frequency  --> fc=fxx/8
	_TQ0CKS0 = 1;
	_TQ0CKS1 = 1;
	_TQ0CKS2 = 0;

	//-- Timer Mode --> Interval timer mode
	_TQ0MD0 = 0;
	_TQ0MD1 = 0;
	_TQ0MD1 = 0;

	TQ0CCR0 = 2499; // Frequenza di interrupt --> fi =  fc / (TP0CCR0 + 1) --> fi = 1 KHz
	
	TQ0CCIC0 = 0x03;
	load_stopSampling();
	// debug: initizlizzazione del filtro media mobile su chan1
	//int_average_filter();	
}


/**
Starts the data sampling from the external adc: enable hw timer as sampling base time and the spi interface.	
*/
void load_startSampling()
{
	CB1CTL0 = 0xE1;	// SPI1 RX/TX abilitati, MSB first, single transfer mode, coom start trigger valid			
	CB2CTL0 = 0xE1;	// SPI2
}

void load_stopSampling()
{
	word valspi1;//, valspi2;

	_CB1SCE = 0; 	// stop SPI bit 0 di CB1CTL0
	valspi1 = CB1RX;
	CB1CTL0 = 0x00;

	CS_ADC1_HIGH();
	CS_ADC2_HIGH();
	CS_ADC3_HIGH();

	_CB2SCE = 0;
	valspi1 = CB2RX;
	CB2CTL0 = 0x00;
	CS_ADC4_HIGH();
	CS_ADC5_HIGH();
}

/*
static void IrqHandINTP6(int vector)
{
}
*/
int CS5530_Spi1Initialization()
{
	//CS5530_rstInternalState();
	m_TxState = CS5530_TxState_idle;
	m_CommandSent = (byte)CS5530_null;	
	m_numByteReceived = 0;
	m_numByteToReceived = 0;	
	m_adcStates = CS5530_initializing;	
	m_endContinuousConversion = False;	
	m_initializationSequenceState = CS5530_wait_minimun_delay;
	m_txBufferEmpty = 1;
	m_spiOrPinMode = 1;
	m_startContinuousConversion = 0;	
	m_confReg = 0;
	m_confRegStatus = CS5530_confReg_notUpdated;	
	m_confReg_PowerSave = 0;
	m_confReg_PowerDown = 0;
	m_confReg_RstSys = 0;
	m_confReg_RstValid = 0;
	m_confReg_InputShort = 0;
	m_confReg_VoltageRef = 0;
	m_confReg_LatchBit = 0;
	m_confReg_WordRate = 0;
	m_confReg_UniBiPolar = 0;
	m_confReg_OpenCircuitDetect = 0;
	//m_sampleFrequency = 0;
	m_confReg_filterRate = 0;
	m_validDataPresentInADCBuffer = 0;
	m_num_max_adc_for_channel = 3;
	adc_to_sample = ALL_ADC;	
	numByteSent = 0;			
	timerSpi.Stop();
	CS5530_rx_fifo.clear();
	CS5530_tx_fifo.clear();	
	ADC1_serial_fifo.clear();
	ADC2_serial_fifo.clear();
	ADC3_serial_fifo.clear();
	wait_reset_ADC.Preset(25);	// tempo di attesa consigliato per la procedura di reset dell'ADC
	return 0;
}

int CS5530_Spi2Initialization()
{
	m_TxStateSpi2 = CS5530_TxState_idle;
	m_CommandSentSpi2 = (byte)CS5530_null;
	m_numByteReceivedSpi2 = 0;
	m_numByteToReceivedSpi2 = 0;
	m_adcStatesSpi2 = CS5530_initializing;
	m_endContinuousConversionSpi2 = False;
	m_initializationSequenceStateSpi2 = CS5530_wait_minimun_delay;
	m_txBufferEmptySpi2 = 1;
	m_spiOrPinModeSpi2 = 1;
	m_startContinuousConversionSpi2 = 0;		
	m_confRegSpi2 = 0;
	m_confRegSpi2_PowerSave = 0;
	m_confRegSpi2_PowerDown = 0;
	m_confRegSpi2_RstSys = 0;
	m_confRegSpi2_RstValid = 0;
	m_confRegSpi2_InputShort = 0;
	m_confRegSpi2_VoltageRef = 0;
	m_confRegSpi2_LatchBit = 0;
	m_confRegSpi2_WordRate = 0;
	m_confRegSpi2_UniBiPolar = 0;
	m_confRegSpi2_OpenCircuitDetect = 0;
	//m_sampleFrequencySpi2 = 0;
	m_confRegSpi2_filterRate = 0;
	m_validDataPresentInADCBufferSpi2 = 0; 
	m_num_max_adc_for_channelSpi2 = 2;	// numero fisso: se si usa questo canale sono 2 adc collegati
	adc_to_sampleSpi2 = ALL_ADC;
	timerSpi2.Stop();
	CS5530_rx_fifoSpi2.clear();
	CS5530_tx_fifoSpi2.clear();	
	ADC4_serial_fifo.clear();
	ADC5_serial_fifo.clear();

	return 0;
}
/*
void CS5530_rstInternalState()
{
}
*/

/**
Manage the setting of the 3 serial ADC's, the comunication on the SIO1 and the acquisition.
*/
void CS5530_EntrySpi1()
{
	int data;
	byte param[4];
	static dword adcVal = 0;
	static dword regVal = 0;

	// se la fifo non � vuota e il numero di dati ricevuti � inferiore al numero dei dati da ricevere
	while(!CS5530_rx_fifo.empty())	// se ci sono dati nel buffer di ricezione
	{
		asm("di");	
		CS5530_rx_fifo.pop_front(data);	// preleva dalla Fifo e mette nella struttura
		//debug_rx_data = data;	// per vederla in debug
		asm("ei");
		if(m_numByteReceived < m_numByteToReceived)	// se � rispettao il sincronismo entra, altrimenti ha svuotato il buffer e si spera che si risincronizzi
		{
			switch (m_CommandSent)
			{										
				case CS5530_rd_config_register:
					if (m_numByteReceived > 0)
					{
						m_confReg += (dword)(data << ((m_numByteToReceived - m_numByteReceived - 1)*8));
					}
					if(m_numByteReceived == (m_numByteToReceived -1))
					{
						m_confReg_PowerSave = (byte)((m_confReg >> 31) & 0x01);
						m_confReg_PowerDown = (byte)((m_confReg >> 30) & 0x01);
						m_confReg_RstSys = (byte)((m_confReg >> 29) & 0x01);
						m_confReg_RstValid = (byte)((m_confReg >> 28) & 0x01);
						m_confReg_InputShort = (byte)((m_confReg >> 27) & 0x01);
						m_confReg_VoltageRef = (byte)((m_confReg >> 25) & 0x01);
						m_confReg_LatchBit = (byte)((m_confReg >> 23) & 0x03);
						m_confReg_filterRate = (byte)((m_confReg >> 19) & 0x01);
						m_confReg_WordRate = (byte)((m_confReg >> 11) & 0x0F);
						m_confReg_UniBiPolar = (byte)((m_confReg >> 10) & 0x01);
						m_confReg_OpenCircuitDetect = (byte)((m_confReg >> 9) & 0x01);					
						m_confRegStatus = CS5530_confReg_Updated;
					}
					break;
					
				case CS5530_read_adc_data:		// lettura del dato di adc, 4 cicli per comporre il dato di 32bit
					if (m_numByteReceived == 0)
					{
						adcVal = 0;
					}else
					{
						adcVal += (((dword)data) << ((m_numByteToReceived - m_numByteReceived - 1)*8));			
					}			
					break;

				case CS5530_rd_gain_register:		// lettura del registro del gain dell'adc, 4 cicli per comporre il dato di 32bits
					if (m_numByteReceived == 0)
					{
						regVal = 0;
					}else
					{
						regVal += (((dword)data) << ((m_numByteToReceived - m_numByteReceived - 1)*8));			
					}			
					break;
					
				case CS5530_wr_gain_register:			
				case CS5530_wr_offset_register:			
				case CS5530_rd_offset_register:
				case CS5530_wr_config_register:
				case CS5530_perform_single_conversion:
				case CS5530_perform_continuous_conversion:		
				case CS5530_perform_sys_offset_cal_register:
				case CS5530_perform_sys_gain_cal_register:	
				case CS5530_end_continuous_conversion:
				case CS5530_sendInitSyncSequence:	
				case CS5530_sync1:
				case CS5530_sync0:
				case CS5530_null:	
					break;
				
			}
			m_numByteReceived++;
			// se i dati ricevuti sono maggiori o uguali (questo � il caso interessante) di quelli da ricevere
			if (m_numByteReceived >= m_numByteToReceived)
			{
				txBufferEmpty();
				timerSpi.Stop();
				m_numByteReceived = 0;
				if(m_adcStates != CS5530_continuousConversionMode && m_adcStates != CS5530_singleConversionMode)
				{
					//CS_ADC2_HIGH();
					//CS_ADC1_HIGH();
				}
				switch(m_adcStates)
				{					
					case CS5530_singleConversionMode:
						if(m_CommandSent == CS5530_read_adc_data)
						{
							m_adcStates = CS5530_commandMode;
						}
						m_CommandSent = (byte)CS5530_null;
						m_TxState = CS5530_TxState_idle;
						break;
					case CS5530_pre_continuousConversionMode:
						CS_ADC2_HIGH();
						CS_ADC3_HIGH();
						m_adcStates = CS5530_continuousConversionMode;
						m_TxState = CS5530_TxState_idle;
						break;
					case CS5530_continuousConversionMode:
						if (m_CommandSent == CS5530_read_adc_data)
						{	
							if((adcVal & _MASK_IF_OVERFLOW))	// se c'� over range 
								adcVal |= 0xFFFFFFFF;	// cos� avr� un dato a FFFF e posso trattarlo da non valido
							
							adcVal = (adcVal >> 8) & _MASK_24BIT_VALID;	// tolgo gli 8 bit LSB che descrivono l'eventuale overrun e i 6 meno significativi del dato
							switch( adc_to_sample )	// qui la variabile � gi� incrementata di 
							{
								case ADC_A:
									ADC1_serial_fifo.push_back(adcVal);
									break;
								case ADC_B:
									ADC2_serial_fifo.push_back(adcVal);
									break;
								case ADC_C:
									ADC3_serial_fifo.push_back(adcVal);
									break;
								case ALL_ADC:
									break;
							}
							if( adc_to_sample < m_num_max_adc_for_channel)
								adc_to_sample++;
							else	
							{
								adc_to_sample = 1;
								m_validDataPresentInADCBuffer = 1; // dati valide in tutte le fifo
							}
						}
						if(m_CommandSent == CS5530_end_continuous_conversion)
						{
							m_adcStates = CS5530_commandMode;
						}
						m_CommandSent = (byte)CS5530_null;
						m_TxState = CS5530_TxState_idle;
						if(m_endContinuousConversion)
						{
							CS5530_sendCmd(CS5530_end_continuous_conversion, param);
							adc_to_sample = ALL_ADC;	// messaggio per tutti gli ADC, devo abbassare tutti i CS
							m_endContinuousConversion = False;
						}
						break;
					case CS5530_initializing:
						switch (m_initializationSequenceState)
						{							
							case CS5530_waitTowriteSync1andSync0:
								numByteSent = 0;
								m_initializationSequenceState = CS5530_setRstBit;
								break;
								
							case CS5530_waitTosetRstBit:
								m_initializationSequenceState = CS5530_wait8clk;
								break;
								
							case CS5530_waitWait8Clk:
								m_initializationSequenceState = CS5530_rstRstBit;
								break;
															
							case CS5530_waitToRstRstBit:
								m_initializationSequenceState = CS5530_readConfRegFirstTime;
								break;
								
							case CS5530_waitToReadConfRegFirstTime:
		//	****** non torna il m_confReg = forse gli scrivo una word invece che un byte
								if((m_confReg_RstValid == (byte)CS5530_rstCycle) && (m_confReg == (((dword)m_confReg_RstValid) << 28)))
								{
									m_initializationSequenceState = CS5530_writeConfRef;
								}else
								{
									m_initializationSequenceState = CS5530_writingSync1AndSync0;
								}
								break;							
								
							case CS5530_waitToWriteConfRef:
								m_initializationSequenceState = CS5530_readConfRegSecondTime;
								break;
							case CS5530_waitToWriteGainRef:

								break;

							case CS5530_setRstBit:	
							case CS5530_wait8clk:
							case CS5530_rstRstBit:
							case CS5530_readConfRegFirstTime:
							case CS5530_writeConfRef:
							case CS5530_readConfRegSecondTime:
							case CS5530_writingSync1AndSync0:
								break;
								
							case CS5530_waitToReadConfSecondTime:
								if (m_confReg != m_lastConfRegWrote)
								{
									m_initializationSequenceState = CS5530_writeConfRef;
								}else
								{
									//m_initializationSequenceState = CS5530_writingSync1AndSync0;
									//m_adcStates = CS5530_commandMode;
									m_initializationSequenceState = CS5530_writeConfRef;
									m_adcStates = CS5530_writeGainRegister;	// vado nella sequenza automatica di scrittura giadagno adc
								}
								break;
								
						}
						m_CommandSent = (byte)CS5530_null;
						m_TxState = CS5530_TxState_idle;
						break;
						
					case CS5530_writeGainRegister:		// qui si gestisce la scrittura del registro di GAIN
						switch (m_initializationSequenceState)
						{
							case CS5530_writeConfRef:
								break;
							case CS5530_waitToWriteGainRef:
								m_initializationSequenceState = CS5530_writingSync1AndSync0;
								m_adcStates = CS5530_commandMode;
								break;
						}	
					case CS5530_commandMode:		// non c'� nulla da ricevere per cui non fa niente
					default:
						m_CommandSent = (byte)CS5530_null;
						m_TxState = CS5530_TxState_idle;
						break;
				}
			}
		}
	}
	
	//-----------------------------------------------------
	//qui ci entra ora solo nella fase iniziale di configurazione
	if( (m_adcStates != CS5530_continuousConversionMode) & (m_adcStates != CS5530_pre_continuousConversionMode)) // se siamo in conitnuous si fa tutto nell'interrupt
	{
		if (timerSpi.Match())
		{
			if(!CS5530_tx_fifo.empty() )	//byte_fifo_empty(&CS5530_tx_fifo))
			{
				timerSpi.Preset(_TIME_);			
				sio_disable_tx_interrupt_ch1();	//CB1TIC = 0x47;	// imposta irq off di_SPI();
				CS5530_tx_fifo.pop_front(data);	//byte_fifo_pop_front(&CS5530_tx_fifo, &data);
				sio_enable_tx_interrupt_ch1(0x07);	//asm("ei");//CB1TIC = 0x07;	// imposta irq on con priorit� 7 
				CB1TX = data; // Load Tx buffer
			}
		}
	}
	//-----------------------------------------------------
	//-----------------------------------------------------
	switch(m_adcStates)
	{
		case CS5530_commandMode:
			if (m_startContinuousConversion)
			{
				m_startContinuousConversion = 0;
				CS5530_sendCmd(CS5530_perform_continuous_conversion, NULL);
			}
			else
				m_startContinuousConversion = 1;
			break;
		
		case CS5530_singleConversionMode:
		case CS5530_continuousConversionMode:
		/* entro la prima volta, devo settare il pin_mode e abilitare solo un ADC per volta, cminciando dal pirmo ADC

		*/
			if (txBufferIsEmpty() && m_TxState == CS5530_TxState_idle)
			{
				if(m_spiOrPinMode == 1)	// la porta � ancora in modalit� SPI
				{					
					switch( adc_to_sample )
					{
						case ADC_A:
							CS_ADC1_LOW();
							CS_ADC2_HIGH();
							CS_ADC3_HIGH();
							
							m_spiOrPinMode = 0;
							setPINSPI1mode();	// Configure P97 as an input
							break;
						case ADC_B:
							CS_ADC1_HIGH();
							CS_ADC2_LOW();							
							CS_ADC3_HIGH();
							CS5530_sendCmd(CS5530_read_adc_data, NULL);
							break;
						case ADC_C:							
							CS_ADC1_HIGH();
							CS_ADC2_HIGH();
							CS_ADC3_LOW();
							CS5530_sendCmd(CS5530_read_adc_data, NULL);
							break;
						case ALL_ADC:
							break;
					}
				}
				else
				{				
					if( _P97 == 0 )	// quando l'ADC mi dice che � pronto con un dato
					{
						contadc1 = 1;
						m_spiOrPinMode = 1;	// riotnro in modalit� SPI per convertire e leggere: 40cicli di clock
						setSPI1mode();	// Re-Configure P97 as SDI of SPI1						
						CS5530_sendCmd(CS5530_read_adc_data, NULL);
					}					
					else
					{
						contadc1++;
					}
				}
			}
			break;
		case CS5530_initializing:
			switch (m_initializationSequenceState)
			{
				case CS5530_wait_minimun_delay:
					if(wait_reset_ADC.Match())
					{
						m_initializationSequenceState = CS5530_writingSync1AndSync0;
					}
					break;
				case CS5530_writingSync1AndSync0:
					CS5530_sendCmd(CS5530_sendInitSyncSequence, NULL);
					m_initializationSequenceState = CS5530_waitTowriteSync1andSync0;
					break;
					
				case CS5530_waitTowriteSync1andSync0:
					break;
						
				case CS5530_setRstBit:
					CS5530_setRstSys(CS5530_rstCycle);
					CS5530_sendCmd(CS5530_wr_config_register, NULL);
					m_initializationSequenceState = CS5530_waitTosetRstBit;
					break;
					
				case CS5530_waitTosetRstBit:
					break;
					
				case CS5530_wait8clk:
					CS5530_sendCmd(CS5530_null, NULL);
					m_initializationSequenceState = CS5530_waitWait8Clk;
					break;
					
				case CS5530_waitWait8Clk:
					break;
					
				case CS5530_rstRstBit:
					CS5530_setRstSys(CS5530_normalOperation);
					CS5530_sendCmd(CS5530_wr_config_register, NULL);
					m_initializationSequenceState = CS5530_waitToRstRstBit;
					break;
					
				case CS5530_waitToRstRstBit:
					break;
					
				case CS5530_readConfRegFirstTime:
					CS5530_sendCmd(CS5530_rd_config_register, NULL);
					m_initializationSequenceState = CS5530_waitToReadConfRegFirstTime;
					break;
					
				case CS5530_waitToReadConfRegFirstTime:				
					break;
					
				case CS5530_writeConfRef:
					CS5530_setPowerSafeMode(CS5530_standbyMode);
					CS5530_setPowerDownMode(CS5530_normalMode);
					CS5530_setRstSys(CS5530_normalOperation);
					CS5530_setInputShort(CS5530_normalInput);
					CS5530_setVoltageRef(CS5530_higherThan_2_5V);
					CS5530_setLatchBits(CS5530_latchBits_LL);
					CS5530_setSampleFrequecy(CS5530_sf_30_Hz);
					CS5530_setAdcCoding(CS5530_unipolar);
					CS5530_setOpenCircuitDetector(CS5530_openCircuitDetector_NOT_ACTIVE);
					
					CS5530_sendCmd(CS5530_wr_config_register, NULL);
					m_lastConfRegWrote = m_confReg;
					m_initializationSequenceState = CS5530_waitToWriteConfRef;
					break;
					
				case CS5530_waitToWriteConfRef:
					break;
					
				case CS5530_readConfRegSecondTime:
					CS5530_sendCmd(CS5530_rd_config_register, NULL);
					m_initializationSequenceState = CS5530_waitToReadConfSecondTime;
					break;
					
				case CS5530_waitToReadConfSecondTime:
					break;
					
			}
			break;
		case CS5530_writeGainRegister:		// qui si gestisce la scrittura del registro di GAIN
			switch (m_initializationSequenceState)
			{
				case CS5530_writeConfRef:
					// scrivo il registro di gain in modo che il valore del gain sia 2.000
					CS5530_setIntGainADConv(CS5530_doublegain, param);		//CS5530_setIntGainADConv(CS5530_gain, param);					
					CS5530_sendCmd(CS5530_wr_gain_register, param);
					m_initializationSequenceState = CS5530_waitToWriteGainRef;
					break;
				case CS5530_waitToWriteGainRef:
					break;
			}				
			break;
	}
}

int CS5530_sendCmd(CS5530_command cmd, byte *param)
{
	int i;
	
	if (m_TxState == CS5530_TxState_busy )
	{
		return -1;
	}
	
	if(m_adcStates != CS5530_initializing)
	{
		if ((m_adcStates == CS5530_singleConversionMode || m_adcStates == CS5530_continuousConversionMode))
		{
			if ((cmd != CS5530_end_continuous_conversion && cmd != CS5530_read_adc_data))
			{
				return -1;
			}
		}
	}
	
	m_TxState = CS5530_TxState_busy;
	m_CommandSent = (byte)cmd;
	m_numByteReceived = 0;
	// in fase di configurazione, i 3 adc sono in ascolto assieme, quindi i 3 CS bassi contemporaneamente
	// in continuous conversione mode, bisogna abbassare in sequenza uno solo CS alla volta, 
	switch( adc_to_sample )
	{
		case ADC_A:		// gestione CS dentro la ENtry
		case ADC_B:		// gestione CS dentro la ENtry
		case ADC_C:		// gestione CS dentro la ENtry
			break;
		case ALL_ADC:	// gestione qui
			CS_ADC1_LOW();
			CS_ADC2_LOW();
			CS_ADC3_LOW();
			break;
	}	
		
	switch (cmd)
	{			
		case CS5530_rd_config_register:
			m_confReg = 0;
			m_confRegStatus = CS5530_confReg_notUpdated;
			sio_disable_tx_interrupt_ch1();	//di_SPI();
			CS5530_tx_fifo.push_back(m_CommandSent);	//byte_fifo_push_back(&CS5530_tx_fifo, m_CommandSent);
			for(i = 0; i < 4; i++)
			{
				CS5530_tx_fifo.push_back((byte)CS5530_null);	//byte_fifo_push_back(&CS5530_tx_fifo, (byte)CS5530_null);
			}
			sio_enable_tx_interrupt_ch1(0x06);	//di_SPI();	//di_SPI();
			m_numByteToReceived = 5;
			break;
		case CS5530_rd_gain_register:
		case CS5530_rd_offset_register:
			sio_disable_tx_interrupt_ch1();	//di_SPI();
			CS5530_tx_fifo.push_back(m_CommandSent);	//byte_fifo_push_back(&CS5530_tx_fifo, m_CommandSent);
			for(i = 0; i < 4; i++)
			{
				CS5530_tx_fifo.push_back((byte)CS5530_null);	//byte_fifo_push_back(&CS5530_tx_fifo, (byte)CS5530_null);
			}
			sio_enable_tx_interrupt_ch1(0x06);	//di_SPI();	//di_SPI();
			m_numByteToReceived = 5;
			break;	
			
		case CS5530_wr_config_register:
			m_confReg = 0;
			m_confReg += ((dword)m_confReg_PowerSave << 31);
			m_confReg += ((dword)m_confReg_PowerDown << 30);
			m_confReg += ((dword)m_confReg_RstSys << 29);
			//m_confReg += ((dword)m_confReg_RstValid);
			m_confReg += ((dword)m_confReg_InputShort << 27);
			m_confReg += ((dword)m_confReg_VoltageRef << 25);
			m_confReg += ((dword)m_confReg_LatchBit << 23);
			m_confReg += ((dword)m_confReg_filterRate << 19);
			m_confReg += ((dword)m_confReg_WordRate << 11);
			m_confReg += ((dword)m_confReg_UniBiPolar << 10);
			m_confReg += ((dword)m_confReg_OpenCircuitDetect << 9);
			sio_disable_tx_interrupt_ch1();	//asm("di");
			CS5530_tx_fifo.push_back(m_CommandSent);	//byte_fifo_push_back(&CS5530_tx_fifo, m_CommandSent);
			CS5530_tx_fifo.push_back(((byte*)(&m_confReg))[3]);	//byte_fifo_push_back(&CS5530_tx_fifo, ((byte*)(&m_confReg))[3]);
			CS5530_tx_fifo.push_back(((byte*)(&m_confReg))[2]);	//byte_fifo_push_back(&CS5530_tx_fifo, ((byte*)(&m_confReg))[2]);
			CS5530_tx_fifo.push_back(((byte*)(&m_confReg))[1]);	//byte_fifo_push_back(&CS5530_tx_fifo, ((byte*)(&m_confReg))[1]);
			CS5530_tx_fifo.push_back(((byte*)(&m_confReg))[0]);	//byte_fifo_push_back(&CS5530_tx_fifo, ((byte*)(&m_confReg))[0]);
			sio_enable_tx_interrupt_ch1(0x06);	//di_SPI();	//di_SPI();
			m_confRegStatus = CS5530_confReg_Updated;
			m_numByteToReceived = 5;
			break;
			
		case CS5530_perform_single_conversion:
			sio_disable_tx_interrupt_ch1();	
			CS5530_tx_fifo.push_back(m_CommandSent);	//
			sio_enable_tx_interrupt_ch1(0x06);	//
			m_numByteToReceived = 1;
			m_adcStates = CS5530_singleConversionMode;
			break;
		case CS5530_perform_continuous_conversion:
			sio_disable_tx_interrupt_ch1();	//
			CS5530_tx_fifo.push_back(m_CommandSent);
			sio_enable_tx_interrupt_ch1(0x06);	
			m_numByteToReceived = 1;
			adc_to_sample = ADC_A;
			m_adcStates = CS5530_pre_continuousConversionMode;
			break;
			
		case CS5530_perform_sys_offset_cal_register:
		case CS5530_perform_sys_gain_cal_register:
			sio_disable_tx_interrupt_ch1();	//di_SPI();
			CS5530_tx_fifo.push_back(m_CommandSent);	//byte_fifo_push_back(&CS5530_tx_fifo, m_CommandSent);
			sio_enable_tx_interrupt_ch1(0x06);	//di_SPI();	//di_SPI();
			m_numByteToReceived = 1;
			break;
			
		case CS5530_sync1:
		case CS5530_sync0:
			sio_disable_tx_interrupt_ch1();	//di_SPI();
			CS5530_tx_fifo.push_back(m_CommandSent);	//byte_fifo_push_back(&CS5530_tx_fifo, m_CommandSent);
			sio_enable_tx_interrupt_ch1(0x06);	//di_SPI();	//di_SPI();
			m_numByteToReceived  = 1;
			break;

		case CS5530_end_continuous_conversion:
			sio_disable_tx_interrupt_ch1();	//di_SPI();
			for(i = 0; i < 5; i++)
			{
				CS5530_tx_fifo.push_back((byte)CS5530_sync1);	//byte_fifo_push_back(&CS5530_tx_fifo, (byte)CS5530_sync1);
			}
			sio_enable_tx_interrupt_ch1(0x06);	//di_SPI();	//di_SPI();
			m_numByteToReceived  = 5;
			break;
			
		case CS5530_read_adc_data:
			sio_disable_tx_interrupt_ch1();
			for(i = 0; i < 5; i++)
			{
				CS5530_tx_fifo.push_back((byte)CS5530_null);
			}
			CS5530_tx_fifo.push_back((byte)CS5530_null);
			sio_enable_tx_interrupt_ch1(0x06);
			m_numByteToReceived  = 5;
			break;
			
		case CS5530_sendInitSyncSequence:
			sio_disable_tx_interrupt_ch1();	//di_SPI();
			for(i = 0; i < 20; i++)
			{
				CS5530_tx_fifo.push_back((byte)CS5530_sync1);	//byte_fifo_push_back(&CS5530_tx_fifo, (byte)CS5530_sync1);
			}
			CS5530_tx_fifo.push_back((byte)CS5530_sync0);	//byte_fifo_push_back(&CS5530_tx_fifo, (byte)CS5530_sync0);
			sio_enable_tx_interrupt_ch1(0x06);	//di_SPI();
			m_numByteToReceived  = 21;
			break;
			
		case CS5530_null:	
			sio_disable_tx_interrupt_ch1();	//di_SPI();
			CS5530_tx_fifo.push_back((byte)CS5530_null);	//byte_fifo_push_back(&CS5530_tx_fifo, (byte)CS5530_null);
			sio_enable_tx_interrupt_ch1(0x06);	//di_SPI();
			m_numByteToReceived  = 1;
			break;

		case CS5530_wr_gain_register:
		case CS5530_wr_offset_register:
			sio_disable_tx_interrupt_ch1();
			CS5530_tx_fifo.push_back(m_CommandSent);	
			CS5530_tx_fifo.push_back(param[3]);	
			CS5530_tx_fifo.push_back(param[2]);	
			CS5530_tx_fifo.push_back(param[1]);	
			CS5530_tx_fifo.push_back(param[0]);
			sio_enable_tx_interrupt_ch1(0x06);
			m_numByteToReceived = 5;
			break;
			
		default:
			m_TxState = CS5530_TxState_idle;
			break;
	}

	timerSpi.Preset(_TIME_);
	SPI_StartTx();
	
	return 0;
}

/**
Manage the setting of the 2 serial ADC's, the comunication on the SIO2 and the acquisition.
*/
void CS5530_EntrySpi2()
{
	int data;
	byte param[4];
	static dword adcVal = 0;

	// se la fifo non � vuota e il numero di dati ricevuti � inferiore al numero dei dati da ricevere
	while(!CS5530_rx_fifoSpi2.empty() && (m_numByteReceivedSpi2 < m_numByteToReceivedSpi2))
	{
		sio_disable_tx_interrupt_ch2();
		CS5530_rx_fifoSpi2.pop_front(data);	// preleva dalla Fifo e mette nella struttura
		//debug_rx_data = data;	// per vederla in debug
		sio_enable_tx_interrupt_ch2(0x07);	//asm("ei");
		
		switch (m_CommandSentSpi2)
		{										
			case CS5530_rd_config_register:
				if (m_numByteReceivedSpi2 > 0)
				{
					m_confRegSpi2 += (dword)(data << ((m_numByteToReceivedSpi2 - m_numByteReceivedSpi2 - 1)*8));
				}
				if(m_numByteReceivedSpi2 == (m_numByteToReceivedSpi2 -1))
				{
					m_confRegSpi2_PowerSave = (byte)((m_confRegSpi2 >> 31) & 0x01);
					m_confRegSpi2_PowerDown = (byte)((m_confRegSpi2 >> 30) & 0x01);
					m_confRegSpi2_RstSys = (byte)((m_confRegSpi2 >> 29) & 0x01);
					m_confRegSpi2_RstValid = (byte)((m_confRegSpi2 >> 28) & 0x01);
					m_confRegSpi2_InputShort = (byte)((m_confRegSpi2 >> 27) & 0x01);
					m_confRegSpi2_VoltageRef = (byte)((m_confRegSpi2 >> 25) & 0x01);
					m_confRegSpi2_LatchBit = (byte)((m_confRegSpi2 >> 23) & 0x03);
					m_confRegSpi2_filterRate = (byte)((m_confRegSpi2 >> 19) & 0x01);
					m_confRegSpi2_WordRate = (byte)((m_confRegSpi2 >> 11) & 0x0F);
					m_confRegSpi2_UniBiPolar = (byte)((m_confRegSpi2 >> 10) & 0x01);
					m_confRegSpi2_OpenCircuitDetect = (byte)((m_confRegSpi2 >> 9) & 0x01);
					
					m_confRegStatusSpi2 = CS5530_confReg_Updated;
				}
				break;
				
			case CS5530_read_adc_data:
				if (m_numByteReceivedSpi2 == 0)
				{
					adcVal = 0;
				}else
				{
					adcVal += (((dword)data) << ((m_numByteToReceivedSpi2 - m_numByteReceivedSpi2 - 1)*8));			
				}			
				break;
				
			case CS5530_wr_gain_register:			
			case CS5530_wr_offset_register:
			case CS5530_rd_gain_register:
			case CS5530_rd_offset_register:
			case CS5530_wr_config_register:
			case CS5530_perform_single_conversion:
			case CS5530_perform_continuous_conversion:		
			case CS5530_perform_sys_offset_cal_register:
			case CS5530_perform_sys_gain_cal_register:	
			case CS5530_end_continuous_conversion:
			case CS5530_sendInitSyncSequence:	
			case CS5530_sync1:
			case CS5530_sync0:
			case CS5530_null:	
				break;
			
		}
		m_numByteReceivedSpi2++;
		// se i dati ricevuti sono maggiori o uguali (questo � il caso interessante) di quelli da ricevere
		if (m_numByteReceivedSpi2 >= m_numByteToReceivedSpi2)
		{
			txBufferEmptySpi2();
			timerSpi2.Stop();
			m_numByteReceivedSpi2 = 0;
			if(m_adcStatesSpi2 != CS5530_continuousConversionMode && m_adcStatesSpi2 != CS5530_singleConversionMode)
			{
				//CS_ADC2_HIGH();
				//CS_ADC1_HIGH();
			}
			switch(m_adcStatesSpi2)
			{				
				case CS5530_singleConversionMode:
					if(m_CommandSentSpi2 == CS5530_read_adc_data)
					{
						m_adcStatesSpi2 = CS5530_commandMode;
					}
					m_CommandSentSpi2 = (byte)CS5530_null;
					m_TxStateSpi2 = CS5530_TxState_idle;
					break;
				case CS5530_pre_continuousConversionMode:
					CS_ADC4_HIGH();
					CS_ADC5_HIGH();
					m_adcStatesSpi2 = CS5530_continuousConversionMode;
					m_TxStateSpi2 = CS5530_TxState_idle;
					break;
				case CS5530_continuousConversionMode:
					if (m_CommandSentSpi2 == CS5530_read_adc_data)
					{	
						if((adcVal & 0x00000004))	// se c'� over range
							adcVal |= 0xFFFFFFFF;	// cos� avr� un dato a FFFF e posso trattarlo da non valido
						adcVal = (adcVal >> 8) & 0x00FFFFFF;	// tolgo gli 8 bit LSB che descrivono l'eventuale overrun
						switch( adc_to_sampleSpi2 )	// qui la variabile � gi� incrementata di 
						{
							case ADC_A:
								ADC4_serial_fifo.push_back(adcVal);
								break;
							case ADC_B:
								ADC5_serial_fifo.push_back(adcVal);
								break;
							case ADC_C:
								break;
							case ALL_ADC:
								break;
						}
						if( adc_to_sampleSpi2 < m_num_max_adc_for_channelSpi2)
							adc_to_sampleSpi2++;
						else
						{
							adc_to_sampleSpi2 = ADC_A;
							m_validDataPresentInADCBufferSpi2 = 1;
						}
					}
					if(m_CommandSentSpi2 == CS5530_end_continuous_conversion)
					{
						m_adcStatesSpi2 = CS5530_commandMode;
					}
					m_CommandSentSpi2 = (byte)CS5530_null;
					m_TxStateSpi2 = CS5530_TxState_idle;
					if(m_endContinuousConversionSpi2)
					{
						CS5530_sendCmdSpi2(CS5530_end_continuous_conversion, param);
						adc_to_sampleSpi2 = ALL_ADC;	// messaggio per tutti gli ADC, devo abbassare tutti i CS
						m_endContinuousConversionSpi2 = False;
					}
					break;
				case CS5530_initializing:
					switch (m_initializationSequenceStateSpi2)
					{							
						case CS5530_waitTowriteSync1andSync0:
							//numByteSent = 0;
							m_initializationSequenceStateSpi2 = CS5530_setRstBit;
							break;
							
						case CS5530_waitTosetRstBit:
							m_initializationSequenceStateSpi2 = CS5530_wait8clk;
							break;
							
						case CS5530_waitWait8Clk:
							m_initializationSequenceStateSpi2 = CS5530_rstRstBit;
							break;
														
						case CS5530_waitToRstRstBit:
							m_initializationSequenceStateSpi2 = CS5530_readConfRegFirstTime;
							break;
							
						case CS5530_waitToReadConfRegFirstTime:
	//	****** non torna il m_confReg = forse gli scrivo una word invece che un byte
							if((m_confRegSpi2_RstValid == (byte)CS5530_rstCycle) && (m_confRegSpi2 == (((dword)m_confRegSpi2_RstValid) << 28)))
							{
								m_initializationSequenceStateSpi2 = CS5530_writeConfRef;
							}else
							{
								m_initializationSequenceStateSpi2 = CS5530_writingSync1AndSync0;
							}
							break;							
							
						case CS5530_waitToWriteConfRef:
							m_initializationSequenceStateSpi2 = CS5530_readConfRegSecondTime;
							break;

						case CS5530_setRstBit:	
						case CS5530_wait8clk:
						case CS5530_rstRstBit:
						case CS5530_readConfRegFirstTime:
						case CS5530_writeConfRef:
						case CS5530_readConfRegSecondTime:
						case CS5530_writingSync1AndSync0:
							break;
							
						case CS5530_waitToReadConfSecondTime:
							if (m_confRegSpi2 != m_lastConfRegSpi2Wrote)
							{
								m_initializationSequenceStateSpi2 = CS5530_writeConfRef;
							}else
							{
								//m_initializationSequenceStateSpi2 = CS5530_writingSync1AndSync0;
								//m_adcStatesSpi2 = CS5530_commandMode;
								m_initializationSequenceStateSpi2 = CS5530_writeConfRef;
								m_adcStatesSpi2 = CS5530_writeGainRegister;	// vado nella sequenza automatica di scrittura giadagno adc
							}
							break;
							
					}
					m_CommandSentSpi2 = (byte)CS5530_null;
					m_TxStateSpi2 = CS5530_TxState_idle;

					break;
				
				case CS5530_writeGainRegister:		// qui si gestisce la scrittura del registro di GAIN
					switch (m_initializationSequenceStateSpi2)
					{
						case CS5530_writeConfRef:
							break;
						case CS5530_waitToWriteGainRef:
							m_initializationSequenceStateSpi2 = CS5530_writingSync1AndSync0;
							m_adcStatesSpi2 = CS5530_commandMode;
							break;
					}
					m_CommandSentSpi2 = (byte)CS5530_null;
					m_TxStateSpi2 = CS5530_TxState_idle;
					break;
					
				case CS5530_commandMode:
				default:
					m_CommandSentSpi2 = (byte)CS5530_null;
					m_TxStateSpi2 = CS5530_TxState_idle;
					break;
			}
		}
	}
	
	//-----------------------------------------------------
	//-----------------------------------------------------
	if( (m_adcStatesSpi2 != CS5530_continuousConversionMode) & (m_adcStatesSpi2 != CS5530_pre_continuousConversionMode)) // se siamo in conitnuous si fa tutto nell'interrupt
	{
		if (timerSpi2.Match())
		{
			if(!CS5530_tx_fifoSpi2.empty() )	
			{
				timerSpi2.Preset(_TIME_);
				sio_disable_tx_interrupt_ch2();	//CB2TIC = 0x47;
				CS5530_tx_fifoSpi2.pop_front(data);
				sio_enable_tx_interrupt_ch2(0x07);	//CB2TIC = 0x07;	
				CB2TX = data; // Load Tx buffer
			}
		}
	}
	//-----------------------------------------------------
	//-----------------------------------------------------
	switch(m_adcStatesSpi2)
	{
		case CS5530_commandMode:
			if (m_startContinuousConversionSpi2)
			{
				m_startContinuousConversionSpi2 = 0;
				CS5530_sendCmdSpi2(CS5530_perform_continuous_conversion, NULL);
				//CS5530_rx_fifo.clear();
			}
			else
				m_startContinuousConversionSpi2 = 1;
			break;
		
		case CS5530_singleConversionMode:
		case CS5530_continuousConversionMode:
		// entro la prima volta, devo settare il pin_mode e abilitare solo un ADC per volta, cminciando dal pirmo ADC

			if (txBufferIsEmptySpi2() && m_TxStateSpi2 == CS5530_TxState_idle)
			{
				if(m_spiOrPinModeSpi2 == 1)	// la porta � ancora in modalit� SPI
				{					
					switch( adc_to_sampleSpi2 )
					{
						case ADC_A:
							CS_ADC4_LOW();
							CS_ADC5_HIGH();
							//CS_ADC6_HIGH();
							m_spiOrPinModeSpi2 = 0;
							setPINSPI2mode();	// Configure P97 as an input
							break;
						case ADC_B:
							CS_ADC5_LOW();
							CS_ADC4_HIGH();
							//CS_ADC6_HIGH();
							CS5530_sendCmdSpi2(CS5530_read_adc_data, NULL);
							break;
						case ADC_C:
							break;
						case ALL_ADC:
							break;
					}
				}
				else
				{				
					if( PinSPI2ready == 0 )	// quando l'ADC mi dice che � pronto con un dato
					{
						m_spiOrPinModeSpi2 = 1;	// riotnro in modalit� SPI per convertire e leggere: 40cicli di clock
						setSPI2mode();	// Re-Configure P97 as SDI of SPI1						
						CS5530_sendCmdSpi2(CS5530_read_adc_data, NULL);
					}
				}
			}
			break;
		case CS5530_initializing:
			switch (m_initializationSequenceStateSpi2)
			{
				case CS5530_wait_minimun_delay:
					if(wait_reset_ADC.Match())
						m_initializationSequenceStateSpi2 = CS5530_writingSync1AndSync0;
				case CS5530_writingSync1AndSync0:
					CS5530_sendCmdSpi2(CS5530_sendInitSyncSequence, NULL);
					m_initializationSequenceStateSpi2 = CS5530_waitTowriteSync1andSync0;
					break;
					
				case CS5530_waitTowriteSync1andSync0:
					break;
						
				case CS5530_setRstBit:
					CS5530_setRstSysSpi2(CS5530_rstCycle);
					CS5530_sendCmdSpi2(CS5530_wr_config_register, NULL);
					m_initializationSequenceStateSpi2 = CS5530_waitTosetRstBit;
					break;
					
				case CS5530_waitTosetRstBit:
					break;
					
				case CS5530_wait8clk:
					CS5530_sendCmdSpi2(CS5530_null, NULL);
					m_initializationSequenceStateSpi2 = CS5530_waitWait8Clk;
					break;
					
				case CS5530_waitWait8Clk:
					break;
					
				case CS5530_rstRstBit:
					CS5530_setRstSysSpi2(CS5530_normalOperation);
					CS5530_sendCmdSpi2(CS5530_wr_config_register, NULL);
					m_initializationSequenceStateSpi2 = CS5530_waitToRstRstBit;
					break;
					
				case CS5530_waitToRstRstBit:
					break;
					
				case CS5530_readConfRegFirstTime:
					CS5530_sendCmdSpi2(CS5530_rd_config_register, NULL);
					m_initializationSequenceStateSpi2 = CS5530_waitToReadConfRegFirstTime;
					break;
					
				case CS5530_waitToReadConfRegFirstTime:				
					break;
					
				case CS5530_writeConfRef:
					CS5530_setPowerSafeModeSpi2(CS5530_standbyMode);
					CS5530_setPowerDownModeSpi2(CS5530_normalMode);
					CS5530_setRstSysSpi2(CS5530_normalOperation);
					CS5530_setInputShortSpi2(CS5530_normalInput);
					CS5530_setVoltageRefSpi2(CS5530_higherThan_2_5V);
					CS5530_setLatchBitsSpi2(CS5530_latchBits_LL);
					CS5530_setSampleFrequecySpi2(CS5530_sf_30_Hz);
					CS5530_setAdcCodingSpi2(CS5530_unipolar);
					CS5530_setOpenCircuitDetectorSpi2(CS5530_openCircuitDetector_NOT_ACTIVE);
					
					CS5530_sendCmdSpi2(CS5530_wr_config_register, NULL);
					m_lastConfRegSpi2Wrote = m_confRegSpi2;
					m_initializationSequenceStateSpi2 = CS5530_waitToWriteConfRef;
					break;
					
				case CS5530_waitToWriteConfRef:
					break;
					
				case CS5530_readConfRegSecondTime:
					CS5530_sendCmdSpi2(CS5530_rd_config_register, NULL);
					m_initializationSequenceStateSpi2 = CS5530_waitToReadConfSecondTime;
					break;
					
				case CS5530_waitToReadConfSecondTime:
					break;
					
			}
			break;
			
		case CS5530_writeGainRegister:		// qui si gestisce la scrittura del registro di GAIN
			switch (m_initializationSequenceStateSpi2)
			{
				case CS5530_writeConfRef:
					// scrivo il registro di gain in modo che il valore del gain sia 2.000
					CS5530_setIntGainADConv(CS5530_doublegain, param);   //CS5530_setIntGainADConv(CS5530_gain, param);					
					CS5530_sendCmdSpi2(CS5530_wr_gain_register, param);
					m_initializationSequenceStateSpi2 = CS5530_waitToWriteGainRef;
					break;
				case CS5530_waitToWriteGainRef:
					break;
			}				
			break;
	}
}

int CS5530_sendCmdSpi2(CS5530_command cmd, byte *param)
{
	int i;
	
	if (m_TxStateSpi2 == CS5530_TxState_busy )
	{
		return -1;
	}
	
	if(m_adcStatesSpi2 != CS5530_initializing)
	{
		if ((m_adcStatesSpi2 == CS5530_singleConversionMode || m_adcStatesSpi2 == CS5530_continuousConversionMode))
		{
			if ((cmd != CS5530_end_continuous_conversion && cmd != CS5530_read_adc_data))
			{
				return -1;
			}
		}
	}
	
	m_TxStateSpi2 = CS5530_TxState_busy;
	m_CommandSentSpi2 = (byte)cmd;
	m_numByteReceivedSpi2 = 0;
	// in fase di configurazione, i 3 adc sono in ascolto assieme, quindi i 3 CS bassi contemporaneamente
	// in continuous conversione mode, bisogna abbassare in sequenza uno solo CS alla volta, 
	switch( adc_to_sampleSpi2 )
	{
		case ADC_A:		// gestione CS dentro la ENtry
		case ADC_B:		// gestione CS dentro la ENtry
		case ADC_C:		// gestione CS dentro la ENtry
			break;
		case ALL_ADC:	// gestione qui
			CS_ADC4_LOW();
			CS_ADC5_LOW();
			//CS_ADC6_LOW();
			break;
	}	
		
	switch (cmd)
	{			
		case CS5530_rd_config_register:
			m_confRegSpi2 = 0;
			m_confRegStatusSpi2 = CS5530_confReg_notUpdated;
			sio_disable_tx_interrupt_ch2();	//di_SPI();
			CS5530_tx_fifoSpi2.push_back(m_CommandSentSpi2);
			for(i = 0; i < 4; i++)
			{
				CS5530_tx_fifoSpi2.push_back((byte)CS5530_null);
			}
			sio_enable_tx_interrupt_ch2(0x07);	
			m_numByteToReceivedSpi2 = 5;
			break;
		case CS5530_rd_gain_register:
		case CS5530_rd_offset_register:
			sio_disable_tx_interrupt_ch2();	
			CS5530_tx_fifoSpi2.push_back(m_CommandSentSpi2);	
			for(i = 0; i < 4; i++)
			{
				CS5530_tx_fifoSpi2.push_back((byte)CS5530_null);	
			}
			sio_enable_tx_interrupt_ch2(0x07);	
			m_numByteToReceivedSpi2 = 5;
			break;	
			
		case CS5530_wr_config_register:
			m_confRegSpi2 = 0;
			m_confRegSpi2 += ((dword)m_confRegSpi2_PowerSave << 31);
			m_confRegSpi2 += ((dword)m_confRegSpi2_PowerDown << 30);
			m_confRegSpi2 += ((dword)m_confRegSpi2_RstSys << 29);
			//m_confReg += ((dword)m_confReg_RstValid);
			m_confRegSpi2 += ((dword)m_confRegSpi2_InputShort << 27);
			m_confRegSpi2 += ((dword)m_confRegSpi2_VoltageRef << 25);
			m_confRegSpi2 += ((dword)m_confRegSpi2_LatchBit << 23);
			m_confRegSpi2 += ((dword)m_confRegSpi2_filterRate << 19);
			m_confRegSpi2 += ((dword)m_confRegSpi2_WordRate << 11);
			m_confRegSpi2 += ((dword)m_confRegSpi2_UniBiPolar << 10);
			m_confRegSpi2 += ((dword)m_confRegSpi2_OpenCircuitDetect << 9);
			sio_disable_tx_interrupt_ch2();	//asm("di");
			CS5530_tx_fifoSpi2.push_back(m_CommandSentSpi2);	
			CS5530_tx_fifoSpi2.push_back(((byte*)(&m_confRegSpi2))[3]);	
			CS5530_tx_fifoSpi2.push_back(((byte*)(&m_confRegSpi2))[2]);	
			CS5530_tx_fifoSpi2.push_back(((byte*)(&m_confRegSpi2))[1]);	
			CS5530_tx_fifoSpi2.push_back(((byte*)(&m_confRegSpi2))[0]);	
			sio_enable_tx_interrupt_ch2(0x07);	
			m_confRegStatusSpi2 = CS5530_confReg_Updated;
			m_numByteToReceivedSpi2 = 5;
			break;
			
		case CS5530_perform_single_conversion:
			sio_disable_tx_interrupt_ch2();	
			CS5530_tx_fifoSpi2.push_back(m_CommandSentSpi2);	//
			sio_enable_tx_interrupt_ch2(0x07);	//
			m_numByteToReceivedSpi2 = 1;
			m_adcStatesSpi2 = CS5530_singleConversionMode;
			break;
		case CS5530_perform_continuous_conversion:
			sio_disable_tx_interrupt_ch2();	//
			CS5530_tx_fifoSpi2.push_back(m_CommandSentSpi2);
			sio_enable_tx_interrupt_ch2(0x07);	
			m_numByteToReceivedSpi2 = 1;
			adc_to_sampleSpi2 = ADC_A;
			m_adcStatesSpi2 = CS5530_pre_continuousConversionMode;
			break;
			
		case CS5530_perform_sys_offset_cal_register:
		case CS5530_perform_sys_gain_cal_register:
			sio_disable_tx_interrupt_ch2();	
			CS5530_tx_fifoSpi2.push_back(m_CommandSentSpi2);	
			sio_enable_tx_interrupt_ch2(0x07);	
			m_numByteToReceivedSpi2 = 1;
			break;
			
		case CS5530_sync1:
		case CS5530_sync0:
			sio_disable_tx_interrupt_ch2();
			CS5530_tx_fifoSpi2.push_back(m_CommandSentSpi2);	
			sio_enable_tx_interrupt_ch2(0x07);
			m_numByteToReceivedSpi2  = 1;
			break;

		case CS5530_end_continuous_conversion:
			sio_disable_tx_interrupt_ch2();	
			for(i = 0; i < 5; i++)
			{
				CS5530_tx_fifoSpi2.push_back((byte)CS5530_sync1);	
			}
			sio_enable_tx_interrupt_ch2(0x07);	
			m_numByteToReceivedSpi2  = 5;
			break;
			
		case CS5530_read_adc_data:
			sio_disable_tx_interrupt_ch2();
			for(i = 0; i < 5; i++)
			{
				CS5530_tx_fifoSpi2.push_back((byte)CS5530_null);
			}
			sio_enable_tx_interrupt_ch2(0x07);
			m_numByteToReceivedSpi2  = 5;
			break;
			
		case CS5530_sendInitSyncSequence:
			sio_disable_tx_interrupt_ch2();	
			for(i = 0; i < 20; i++)
			{
				CS5530_tx_fifoSpi2.push_back((byte)CS5530_sync1);	
			}
			CS5530_tx_fifoSpi2.push_back((byte)CS5530_sync0);	
			sio_enable_tx_interrupt_ch2(0x07);	
			m_numByteToReceivedSpi2  = 21;
			break;
			
		case CS5530_null:	
			sio_disable_tx_interrupt_ch2();	
			CS5530_tx_fifoSpi2.push_back((byte)CS5530_null);	
			sio_enable_tx_interrupt_ch2(0x07);	
			m_numByteToReceivedSpi2  = 1;
			break;

		case CS5530_wr_gain_register:
		case CS5530_wr_offset_register:
			sio_disable_tx_interrupt_ch2();
			CS5530_tx_fifoSpi2.push_back(m_CommandSentSpi2);	
			CS5530_tx_fifoSpi2.push_back(param[3]);	
			CS5530_tx_fifoSpi2.push_back(param[2]);	
			CS5530_tx_fifoSpi2.push_back(param[1]);	
			CS5530_tx_fifoSpi2.push_back(param[0]);
			sio_enable_tx_interrupt_ch2(0x07); 
			m_numByteToReceivedSpi2 = 5;
			break;
			
		default:
			m_TxStateSpi2 = CS5530_TxState_idle;
			break;
	}

	timerSpi2.Preset(_TIME_);
	SPI2_StartTx();
	
	return 0;
}

/**
if flag data valid is set, reset flag and gets a sample in adcs fifo if they aren't empty
Samples are reduced to 16bit, erasing 8 LSB and then sent to mobile average filter
*/
bool CS5530_validDataPresentInADCBufferSpi1()
{
int m_adcVal1 = 0,  m_adcVal2 = 0, m_adcVal3 = 0;

	if (m_validDataPresentInADCBuffer == 1)
	{
		CS5530_rstValidDataPresentInADCBuffer();
		if(!ADC1_serial_fifo.empty())
		{
			ADC1_serial_fifo.pop_front(m_adcVal1);  // prelevo il dato di 24bit
			#ifdef __DEBUG_ADC
			if(acquisition == True)
			{
				adc1_data[contadc1++] = ((m_adcVal1>>8) & 0x0000FFFF);
				if(contadc1 == 1800)
				{
					contadc1 = 0;
					contadc2 = 0;
					acquisition = False;
				}
			}
			#endif
			#ifdef _NORMALFILTER_
				m_adcVal1 = (word)((m_adcVal1>>8) & 0x0000FFFF);
			#endif
			weightChan->pushFiltAdcDataToChan(_ADC1_, m_adcVal1);
		}
		if(!ADC2_serial_fifo.empty())
		{
			ADC2_serial_fifo.pop_front(m_adcVal2);  // prelevo il dato di 24bit
			#ifdef _NORMALFILTER_
				m_adcVal2 = (word)((m_adcVal2>>8) & 0x0000FFFF);
			#endif
			weightChan->pushFiltAdcDataToChan(_ADC2_, m_adcVal2);
		}
		if(!ADC3_serial_fifo.empty())
		{
			ADC3_serial_fifo.pop_front(m_adcVal3);  // prelevo il dato di 24bit
			#ifdef _NORMALFILTER_
				m_adcVal3 = (word)((m_adcVal3>>8) & 0x0000FFFF);
			#endif
			weightChan->pushFiltAdcDataToChan(_ADC3_, m_adcVal3);
		}
		return True;
	}

	return False;
}

void CS5530_rstValidDataPresentInADCBuffer()
{
	m_validDataPresentInADCBuffer = 0;
}

void CS5530_setCounterWaitingData()
{
	contadc1 = 0;
}
int CS5530_getCounterWaitingData()
{
	return contadc1;
}
bool CS5530_checkBlockComunication(int match_count)
{
	if(CS5530_getCounterWaitingData() > match_count)	// comunicaz bloccata, adc da resettare
	{
		return True;
	}

	return False;
	
}

bool CS5530_checkBlockCells(dword *loadsystem)
{
	for(int i = 0; i < NUM_MAX_LINE;i++)
	{
		if((word)loadsystem[i] == MAX_VALUE_16_BIT)
		{
			return True;
		}
	}
	
	return False;
}

void CS5530_setWatchdogAdcTimer(dword match_time)
{
	watchdog_adc_timer.Preset(match_time);
}

bool CS5530_getWatchdogAdcTimer()
{
	if(watchdog_adc_timer.Match())
	{
		//watchdog_adc_timer.Stop();
		return True;
	}
	return False;
}

void CS5530_resetAdcComunication()
{
	asm("di");
	adc_serial_init();
	CS5530_Spi1Initialization();
	CS5530_Spi2Initialization();	
	asm	( "ei" );	// start a tutte le	interruzioni
	load_startSampling();
	contadc1 = 0;
}

void CS5530_setPowerSafeMode(CS5530_powerSaveMode mode)
{
	m_confReg_PowerSave = (byte)mode;
	m_confRegStatus = CS5530_confReg_notUpdated;
}

void CS5530_setPowerDownMode(CS5530_powerDownMode mode)
{
	m_confReg_PowerDown = (byte)mode;
	m_confRegStatus = CS5530_confReg_notUpdated;
}

void CS5530_setRstSys(CS5530_rstSys rst)
{
	m_confReg_RstSys = (byte)rst;
	m_confRegStatus = CS5530_confReg_notUpdated;
}

void CS5530_setInputShort(CS5530_inputShort input)
{
	m_confReg_InputShort = (byte) input;
	m_confRegStatus = CS5530_confReg_notUpdated;
}

void CS5530_setVoltageRef(CS5530_voltageRef voltage)
{
	m_confReg_VoltageRef = (byte)voltage;
	m_confRegStatus = CS5530_confReg_notUpdated;
}

void CS5530_setLatchBits(CS5530_outputLatchBits latch)
{
	m_confReg_LatchBit = (byte)latch;
	m_confRegStatus = CS5530_confReg_notUpdated;
}

void CS5530_setSampleFrequecy(CS5530_sampleFrequecies frq)
{
	//m_sampleFrequency  = (byte)frq;
	switch(frq)
	{
		case CS5530_sf_6_25_Hz:
			m_confReg_WordRate = 0x04;
			m_confReg_filterRate = 1;
			break;
		case CS5530_sf_7_5_Hz:
			m_confReg_WordRate = 0x04;
			m_confReg_filterRate = 0;
			break;
		case CS5530_sf_12_5_Hz:
			m_confReg_WordRate = 0x03;
			m_confReg_filterRate = 1;
			break;
		case CS5530_sf_15_Hz:
			m_confReg_WordRate = 0x03;
			m_confReg_filterRate = 0;
			break;
		case CS5530_sf_25_Hz:
			m_confReg_WordRate = 0x02;
			m_confReg_filterRate = 1;
			break;
		case CS5530_sf_30_Hz:
			m_confReg_WordRate = 0x02;
			m_confReg_filterRate = 0;
			break;
		case CS5530_sf_50_Hz:
			m_confReg_WordRate = 0x01;
			m_confReg_filterRate = 1;
			break;
		case CS5530_sf_60_Hz:
			m_confReg_WordRate = 0x01;
			m_confReg_filterRate = 0;
			break;
		case CS5530_sf_100_Hz:
			m_confReg_WordRate = 0x00;
			m_confReg_filterRate = 1;
			break;
		case CS5530_sf_120_Hz:
			m_confReg_WordRate = 0x00;
			m_confReg_filterRate = 0;
			break;
		case CS5530_sf_200_Hz:
			m_confReg_WordRate = 0x0C;
			m_confReg_filterRate = 1;
			break;
		case CS5530_sf_240_Hz:
			m_confReg_WordRate = 0x0C;
			m_confReg_filterRate = 0;
			break;
		case CS5530_sf_400_Hz:
			m_confReg_WordRate = 0x0B;
			m_confReg_filterRate = 1;
			break;
		case CS5530_sf_480_Hz:
			m_confReg_WordRate = 0x0B;
			m_confReg_filterRate = 0;
			break;
		case CS5530_sf_800_Hz:
			m_confReg_WordRate = 0x0A;
			m_confReg_filterRate = 1;
			break;
		case CS5530_sf_960_Hz:
			m_confReg_WordRate = 0x0A;
			m_confReg_filterRate = 0;
			break;
		case CS5530_sf_1600_Hz:
			m_confReg_WordRate = 0x09;
			m_confReg_filterRate = 1;
			break;
		case CS5530_sf_1920_Hz:
			m_confReg_WordRate = 0x09;
			m_confReg_filterRate = 0;
			break;
		case CS5530_sf_3200_Hz:
			m_confReg_WordRate = 0x08;
			m_confReg_filterRate = 1;
			break;
		case CS5530_sf_3840_Hz:
			m_confReg_WordRate = 0x08;
			m_confReg_filterRate = 0;
			break;
	}
	m_confRegStatus = CS5530_confReg_notUpdated;
}

void CS5530_setAdcCoding(CS5530_adcCoding coding)
{
	m_confReg_UniBiPolar = (byte) coding;
	m_confRegStatus = CS5530_confReg_notUpdated;
}

void CS5530_setOpenCircuitDetector(CS5530_openCircuitDetector ocd)
{
	m_confReg_OpenCircuitDetect = (byte)ocd;
	m_confRegStatus = CS5530_confReg_notUpdated;
}

void endContinuousConversion()
{
	m_endContinuousConversion = True;
}

void startContinuousConversion()
{
	m_startContinuousConversion = 1;
}

void CS5530_setPowerSafeModeSpi2(CS5530_powerSaveMode mode)
{
	m_confRegSpi2_PowerSave = (byte)mode;
	m_confRegStatusSpi2 = CS5530_confReg_notUpdated;
}

void CS5530_setPowerDownModeSpi2(CS5530_powerDownMode mode)
{
	m_confRegSpi2_PowerDown = (byte)mode;
	m_confRegStatusSpi2 = CS5530_confReg_notUpdated;
}

void CS5530_setRstSysSpi2(CS5530_rstSys rst)
{
	m_confRegSpi2_RstSys = (byte)rst;
	m_confRegStatusSpi2 = CS5530_confReg_notUpdated;
}

void CS5530_setInputShortSpi2(CS5530_inputShort input)
{
	m_confRegSpi2_InputShort = (byte) input;
	m_confRegStatusSpi2 = CS5530_confReg_notUpdated;
}

void CS5530_setVoltageRefSpi2(CS5530_voltageRef voltage)
{
	m_confRegSpi2_VoltageRef = (byte)voltage;
	m_confRegStatusSpi2 = CS5530_confReg_notUpdated;
}

void CS5530_setLatchBitsSpi2(CS5530_outputLatchBits latch)
{
	m_confRegSpi2_LatchBit = (byte)latch;
	m_confRegStatusSpi2 = CS5530_confReg_notUpdated;
}

void CS5530_setSampleFrequecySpi2(CS5530_sampleFrequecies frq)
{
	//m_sampleFrequency  = (byte)frq;
	switch(frq)
	{
		case CS5530_sf_6_25_Hz:
			m_confRegSpi2_WordRate = 0x04;
			m_confRegSpi2_filterRate = 1;
			break;
		case CS5530_sf_7_5_Hz:
			m_confRegSpi2_WordRate = 0x04;
			m_confRegSpi2_filterRate = 0;
			break;
		case CS5530_sf_12_5_Hz:
			m_confRegSpi2_WordRate = 0x03;
			m_confRegSpi2_filterRate = 1;
			break;
		case CS5530_sf_15_Hz:
			m_confRegSpi2_WordRate = 0x03;
			m_confRegSpi2_filterRate = 0;
			break;
		case CS5530_sf_25_Hz:
			m_confRegSpi2_WordRate = 0x02;
			m_confRegSpi2_filterRate = 1;
			break;
		case CS5530_sf_30_Hz:
			m_confRegSpi2_WordRate = 0x02;
			m_confRegSpi2_filterRate = 0;
			break;
		case CS5530_sf_50_Hz:
			m_confRegSpi2_WordRate = 0x01;
			m_confRegSpi2_filterRate = 1;
			break;
		case CS5530_sf_60_Hz:
			m_confRegSpi2_WordRate = 0x01;
			m_confRegSpi2_filterRate = 0;
			break;
		case CS5530_sf_100_Hz:
			m_confRegSpi2_WordRate = 0x00;
			m_confRegSpi2_filterRate = 1;
			break;
		case CS5530_sf_120_Hz:
			m_confRegSpi2_WordRate = 0x00;
			m_confRegSpi2_filterRate = 0;
			break;
		case CS5530_sf_200_Hz:
			m_confRegSpi2_WordRate = 0x0C;
			m_confRegSpi2_filterRate = 1;
			break;
		case CS5530_sf_240_Hz:
			m_confRegSpi2_WordRate = 0x0C;
			m_confRegSpi2_filterRate = 0;
			break;
		case CS5530_sf_400_Hz:
			m_confRegSpi2_WordRate = 0x0B;
			m_confRegSpi2_filterRate = 1;
			break;
		case CS5530_sf_480_Hz:
			m_confRegSpi2_WordRate = 0x0B;
			m_confRegSpi2_filterRate = 0;
			break;
		case CS5530_sf_800_Hz:
			m_confRegSpi2_WordRate = 0x0A;
			m_confRegSpi2_filterRate = 1;
			break;
		case CS5530_sf_960_Hz:
			m_confRegSpi2_WordRate = 0x0A;
			m_confRegSpi2_filterRate = 0;
			break;
		case CS5530_sf_1600_Hz:
			m_confRegSpi2_WordRate = 0x09;
			m_confRegSpi2_filterRate = 1;
			break;
		case CS5530_sf_1920_Hz:
			m_confRegSpi2_WordRate = 0x09;
			m_confRegSpi2_filterRate = 0;
			break;
		case CS5530_sf_3200_Hz:
			m_confRegSpi2_WordRate = 0x08;
			m_confRegSpi2_filterRate = 1;
			break;
		case CS5530_sf_3840_Hz:
			m_confRegSpi2_WordRate = 0x08;
			m_confRegSpi2_filterRate = 0;
			break;
	}
	m_confRegStatusSpi2 = CS5530_confReg_notUpdated;
}

void CS5530_setAdcCodingSpi2(CS5530_adcCoding coding)
{
	m_confRegSpi2_UniBiPolar = (byte) coding;
	m_confRegStatusSpi2 = CS5530_confReg_notUpdated;
}

void CS5530_setOpenCircuitDetectorSpi2(CS5530_openCircuitDetector ocd)
{
	m_confRegSpi2_OpenCircuitDetect = (byte)ocd;
	m_confRegStatusSpi2 = CS5530_confReg_notUpdated;
}
void endContinuousConversionSpi2()
{
	m_endContinuousConversionSpi2 = True;
}

void startContinuousConversionSpi2()
{
	m_startContinuousConversionSpi2 = 1;
}

/**
Costruisce ilregistro passato come puntatore, con dentro il valore del guadagno da imporre all'adc
*/
void CS5530_setIntGainADConv(int valgain, byte* valreg)
{
	valreg[0] = (valgain & 0x000000FF);
	valreg[1] = (valgain & 0x0000FF00) >> 8;
	valreg[2] = (valgain & 0x00FF0000) >> 16;
	valreg[3] = (valgain & 0xFF000000) >> 24;
}


/**
if flag data valid is set, reset flag and gets a sample in adcs fifo if they aren't empty
Samples are reduced to 16bit, erasing 8 LSB and then sent to mobile average filter
*/
bool CS5530_validDataPresentInADCBufferSpi2()
{
int m_adcVal1 = 0,  m_adcVal2 = 0;

	if (m_validDataPresentInADCBufferSpi2 == 1)
	{
		CS5530_rstValidDataPresentInADCBufferSpi2();
		if(!ADC4_serial_fifo.empty())
		{
			ADC4_serial_fifo.pop_front(m_adcVal1);  // prelevo il dato di 24bit
			#ifdef _NORMALFILTER_
				m_adcVal1 = (word)((m_adcVal1>>8) & 0x0000FFFF);
			#endif
			weightChan->pushFiltAdcDataToChan(_ADC4_, m_adcVal1);
		}
		if(!ADC5_serial_fifo.empty())
		{
			
			ADC5_serial_fifo.pop_front(m_adcVal2);
			/*
			#ifdef __DEBUG_ADC
			if(acquisition == True)
			{
				adc2_data[contadc2++] = m_adcVal2;
				if(contadc2 == 1800)
				{
					contadc2 = 0;
					acquisition = False;
				}
			}
			#endif
			*/
			#ifdef _NORMALFILTER_
				m_adcVal2 = (word)((m_adcVal2>>8) & 0x0000FFFF);
			#endif
			weightChan->pushFiltAdcDataToChan(_ADC5_, m_adcVal2);
		}
		return True;
	}

	return False;
}

void CS5530_rstValidDataPresentInADCBufferSpi2()
{
	m_validDataPresentInADCBufferSpi2 = 0;
}

void SPI_StartTx()
{
	int data;
	
	if (txBufferIsEmpty())
	{
		asm("di");
		CS5530_tx_fifo.pop_front(data);
		asm("ei");	//sio_enable_tx_interrupt_ch1(0x06);	//di_SPI();	//ei_SPI();
		CB1TX = data; // Load Tx buffer
		txBufferNotEmpty();
	}
}

void SPI2_StartTx()
{
	int data;
	
	if (txBufferIsEmptySpi2())
	{
		asm("di");
		CS5530_tx_fifoSpi2.pop_front(data);
		asm("ei");	//sio_enable_tx_interrupt_ch2(0x07);	//di_SPI();	//ei_SPI();
		CB2TX = data; // Load Tx buffer
		txBufferNotEmptySpi2();
	}
}

void txBufferEmpty()
{
	m_txBufferEmpty = 1;
	//SPICON &= ~BIT11;
}

void txBufferNotEmpty()
{
	sio_disable_tx_interrupt_ch1();	//di_SPI();
	m_txBufferEmpty = 0;
	sio_enable_tx_interrupt_ch1(0x06);	//di_SPI();
}

byte txBufferIsEmpty()
{
	return m_txBufferEmpty;
}

void txBufferEmptySpi2()
{
	m_txBufferEmptySpi2 = 1;
	//SPICON &= ~BIT11;
}

void txBufferNotEmptySpi2()
{
	sio_disable_tx_interrupt_ch2();	//di_SPI();
	m_txBufferEmptySpi2 = 0;
	sio_enable_tx_interrupt_ch2(0x07);	//di_SPI();
}

byte txBufferIsEmptySpi2()
{
	return m_txBufferEmptySpi2;
}

void setSPI1mode()
{
	_PM97 = 1;
	_PMC97 = 1;
	_PFC97 = 1;
	_PFCE97 = 0;
	
	_PMC98 = 1;
	_PFC98 = 1;
	
	_PMC99 = 1;
	_PFC99 = 1;
	
	CB1CTL0 = 0xE3;	// RX/TX abilitati, MSB first, single transfer mode, coom start trigger valid
}

void setPINSPI1mode()
{
	_PM97 = 1;
	_PMC97 = 0;

	CB1CTL0 = 0x00;	// RX/TX abilitati, MSB first, single transfer mode, coom start trigger valid
}

void setSPI2mode()
{

	_PM53 = 1;		// port mode SIB2
	_PMC53 = 1;		// port mode control register SIB2
	_PFC53 = 0;		// port function  control reg SIB2
	_PFCE53 = 0;		// port function control expansionregister SIB2
	
	_PMC54 = 1;		// port mode control register SOB2	
	_PFC54 = 0;
	_PFCE54 = 0;

	_PMC55 = 1;		// port mode control register SCKB2
	_PFC55 = 0;
	_PFCE55 = 0;

	CB2CTL0 = 0xE3;	// RX/TX abilitati, MSB first, single transfer mode, coom start trigger valid
}

void setPINSPI2mode()
{
	_PM53 = 1;
	_PMC53 = 0;
	
	CB2CTL0 = 0x00;	// RX/TX abilitati, MSB first, single transfer mode, coom start trigger valid
}

/**
Answer to the hadware interrut request of Spi1, when a new sample is available from adc's
*/
static void IrqHandSIO1_R(int vector)
{
static unsigned short cycle = 1;
byte val_sampled = 0;
	if( (m_adcStates != CS5530_continuousConversionMode) & (m_adcStates != CS5530_pre_continuousConversionMode)) // se siamo in conitnuous si fa tutto nell'interrupt
	{
		val_sampled = CB1RX;
		CS5530_rx_fifo.push_back(val_sampled);
	}
	else
	{
		val_sampled = (CB1RX & 0x00FF);
		CS5530_rx_fifo.push_back(val_sampled);
		if( cycle < m_numByteToReceived )
		{
			cycle++;
			CB1TX = 0;
		}
		else
			cycle = 1;
	}
}

/**
Answer to the hadware interrut request of Spi2, when a new sample is available from adc's
*/
static void IrqHandSIO2_R(int vector)
{
// nel caso del M3100 devo ciclare sui due ADC connessi, prima il 4 poi il 5
// nel caso del M32-3300 devo leggere il solo ADC5
// per ora faccio il caso dell'M3200
static unsigned short cycle = 1;
byte	val_sampled = 0;

	if( (m_adcStatesSpi2 != CS5530_continuousConversionMode) & (m_adcStatesSpi2 != CS5530_pre_continuousConversionMode)) // se siamo in conitnuous si fa tutto nell'interrupt
	{
		val_sampled = CB2RX;
		CS5530_rx_fifoSpi2.push_back(val_sampled);
	}
	else
	{
		val_sampled = (CB2RX & 0x00FF);
		CS5530_rx_fifoSpi2.push_back(val_sampled);
		if( cycle < m_numByteToReceivedSpi2 )
		{
			cycle++;
			CB2TX = 0;
		}
		else
			cycle = 1;
	}
}

static void IrqHandTMQ0(int vector)
{
/*
	if(vol_tick >= 20)
	{
		CS_ADC1_LOW();
		CB1TX = 0;
	}
	vol_tick++;
	*/
}

/**
* Restores from EEPROM the weight channels parameters, as gain & offset, of every channel in system.
* This function is called at every machine start. If factory calibration has never been done, it sets default values which aren't saved in eeprom.
* Return 0 if read in eeprom fails.
*/
byte get_adc_param()
{
	byte numchan;
	word i;
	byte *b;
	byte result;

	// legge dalla eeprom i dati di calibrazione
	for( numchan = 0; numchan < _MAX_LOAD_CHAN_; numchan++)
	{
		for( i = 0; i < sizeof(ChannelsBackupParam); i++)
		{
			b = (byte *)&Chan[numchan] + i;
			*b = 0x00;						// azzero il contenuto di Chan[numchan]
		}
	}

	result = get_factory_adc_param();

	return result;
	
}

/**
* Restores from EEPROM the weight channels parameters of every channel in system.
* This function is called at the system inizializzation.
*/
byte get_factory_adc_param()
{
	byte data;
	byte numchan;
	byte result = E_READ_IDLE;
	byte count_calib = 0;
	
	// legge dalla eeprom i dati di calibrazione
	for( numchan = 0; numchan < _MAX_LOAD_CHAN_; numchan++)
	{
		if(EE_random_byte_read((ADDRESS_CHAN_ARE_MEM_CALIB + numchan*BASESTRUCTADDR), &data))	// leggo il dato di calibrazione dell'offset
		{
			if (data == CHAN_CALIB_CHECK_FACTORY_VAL)	// calibrazione con memorizzazione fatta almeno una volta
			{
				if(EE_random_byte_read((ADDRESS_CHAN_ARE_MEM_CALIB +1 + numchan*BASESTRUCTADDR), &data)) // leggo il dato di calibrazione del guadagno
				{
					if (data == CHAN_CALIB_CHECK_FACTORY_VAL)	// calibrazione con memorizzazione fatta almeno una volta
					{
						Chan[numchan].WeightFactoryGain = EE_read_float(ADDRESS_FACTORY_GAIN + numchan*BASESTRUCTADDR);
						Chan[numchan].WeightFactoryOffset = EE_read_word(ADDRESS_FACTORY_OFFSET+ numchan*BASESTRUCTADDR);
						Chan[numchan].Weightgain = EE_read_float(ADDRESS_GAIN + numchan*BASESTRUCTADDR);
						Chan[numchan].Weightoffset = EE_read_word(ADDRESS_OFFSET + numchan*BASESTRUCTADDR);
						Chan[numchan].AdcOf2Kg = EE_read_word(ADDRESS_2Kg_READ + numchan*BASESTRUCTADDR);
						if(EE_random_byte_read((ADDRESS_CALIB + numchan*BASESTRUCTADDR), &data))
						{
							if(data > 0)
							{
								Chan[numchan].AreCalibrate = True;
							}
							else
							{
								Chan[numchan].AreCalibrate = False;
							}
						}
						count_calib++;
					}
					else		// calibrazione con memorizzazione mai eseguita
					{
						resetAdcValues(numchan);
						backup_calib_state(numchan, (byte)Chan[numchan].AreCalibrate);
						result = E_READ_NO_BK_FACTORY;
					}
				}
			}
			else		// calibrazione con memorizzazione mai eseguita
			{
				resetAdcValues(numchan);
				backup_calib_state(numchan, (byte)Chan[numchan].AreCalibrate);
				result = E_READ_NO_BK_FACTORY;
			}
		}
		else
		{
			result = E_READ_ERROR;
			return result;
		}
	}

	if(!controlAdcValuesRead())
	{
		count_calib = 0;
		result = E_READ_NO_BK_FACTORY;
	}

	if(count_calib == _MAX_LOAD_CHAN_)
	{
		result = E_READ_BK_FACTORY;
	}
	
	return result;
	
}

/**
* Saves in EEPROM calibration values after a calibration made by user. 
* Position in EEPROM is determinated using numchan parameter.
* Returns 0 if eeprom write operation returns error
*/
int backup_adc_param(byte numchan)
{
	byte type_of_calibration = CHAN_CALIB_CHECK_FACTORY_VAL;
	
	if(!EE_write(&Chan[numchan].Weightgain, sizeof(float), (ADDRESS_GAIN + numchan*BASESTRUCTADDR)))
	{
		return 0;
	}
	if(!EE_write(&Chan[numchan].Weightoffset, sizeof(short), (ADDRESS_OFFSET + numchan*BASESTRUCTADDR)))
	{
		return 0;
	}
	if(!EE_write(&Chan[numchan].AdcOf2Kg, (9*sizeof(char)), (ADDRESS_2Kg_READ + numchan*BASESTRUCTADDR)))
	{
		return 0;
	}
	if(!EE_write(&type_of_calibration, 1, (ADDRESS_CHAN_ARE_MEM_CALIB + numchan*BASESTRUCTADDR)))
	{
		return 0;
	}
	
	return 1;
	
}

/**
* Saves in EEPROM gain value after a calibration made by user. 
* Position in EEPROM is determinated using numchan parameter.
* Returns 0 if eeprom write operation returns error
*/
int backup_new_gain_value(byte numchan, float NewGain)
{
	if(!EE_write(&NewGain, sizeof(float), (ADDRESS_GAIN + numchan*BASESTRUCTADDR)))
	{
		return 0;
	}
	
	return 1;
	
}

/**
* Save in EEPROM zero values after new routine. 
* Position in EEPROM is determinated using numchan parameter.
* Returns 0 if eeprom write operation returns error
*/
int backup_new_offset_value(byte numchan, word NewOffset)
{
	if(!EE_write(&NewOffset, sizeof(short), (ADDRESS_OFFSET + numchan*BASESTRUCTADDR)))
	{
		return 0;
	}

	return 1;
	
}

/**
* Saves in EEPROM offset values after a factory calibration. 
* Position in EEPROM is determinated using numchan parameter.
* Returns 0 if eeprom write operation returns error
*/
int backup_factory_offset_value(byte numchan, word FactoryOffset)
{
	byte type_of_calibration = CHAN_CALIB_CHECK_FACTORY_VAL;	// offset calibration value wrote
	
	if(!EE_write(&FactoryOffset, sizeof(word), (ADDRESS_FACTORY_OFFSET + numchan*BASESTRUCTADDR)))
	{
		return 0;
	}
	// lo stesso valore � scritto anche come OFFSET USER
	if(!EE_write(&FactoryOffset, sizeof(word), (ADDRESS_OFFSET + numchan*BASESTRUCTADDR)))
	{
		return 0;
	}
	if(!EE_byte_write((ADDRESS_CHAN_ARE_MEM_CALIB + numchan*BASESTRUCTADDR), type_of_calibration))
	{
		return 0;	// ricorda che questo assume due valori a seconda che sia stata eseguita la sola calibrazione di fabbrica oppure anche quella user.
	}
	
	return 1;
	
}

/**
* Saves in EEPROM gain values after a factory calibration. 
* Address in EEPROM is determinated using numchan parameter.
* Returns 0 if eeprom write operation returns error
*/
int backup_factory_gain_param(byte numchan, float FactoryGain, word adc2Kg)
{
	byte type_of_calibration = CHAN_CALIB_CHECK_FACTORY_VAL;	// gain calibration value wrote
	
	if(!EE_write(&FactoryGain, sizeof(float), (ADDRESS_FACTORY_GAIN + numchan*BASESTRUCTADDR)))
	{
		return 0;
	}
	if(!EE_write(&FactoryGain, sizeof(float), (ADDRESS_GAIN + numchan*BASESTRUCTADDR))) // se factory, aggiorno anche il valore user
	{
		return 0;
	}
	if(!EE_write(&adc2Kg, sizeof(word), (ADDRESS_2Kg_READ + numchan*BASESTRUCTADDR)))
	{
		return 0;
	}
	if(!EE_byte_write((ADDRESS_CHAN_ARE_MEM_CALIB + 1 + numchan*BASESTRUCTADDR), type_of_calibration))
	{
		return 0;	// ricorda che questo assume due valori a seconda che sia stata eseguita la sola calibrazione di fabbrica oppure anche quella user.
	}
	
	return 1;
	
}

/**
* Save in EEPROM calib state of __numchan. 
* Position in EEPROM is determinated using __numchan parameter.
* Returns False if eeprom write operation returns error
*/
bool backup_calib_state(byte __numchan, byte __calib)
{
	if(!EE_write(&__calib, sizeof(byte), (ADDRESS_CALIB + __numchan*BASESTRUCTADDR)))
	{
		return False;
	}

	return True;
	
}

/**
* Resets calibration parameters saved.. 
* Address in EEPROM is determinated using numchan parameter.
* Returns False if eeprom write operation returns error.
*/
bool reset_backup_factory_values(byte numchan, byte areCalib)
{
	if(!EE_byte_write((ADDRESS_CHAN_ARE_MEM_CALIB + numchan*BASESTRUCTADDR), areCalib))
	{
		return False;	// ricorda che questo assume due valori a seconda che sia stata eseguita la sola calibrazione di fabbrica oppure anche quella user.
	}
	if(!EE_byte_write((ADDRESS_CHAN_ARE_MEM_CALIB + 1 + numchan*BASESTRUCTADDR), areCalib))
	{
		return False;
	}
	
	return True;
	
}


/**
* Saves in EEPROM calibration values after factory calibration. 
* Address in EEPROM is determinated using numchan parameter.
* Returns 0 if eeprom write operation returns error
*/
int backup_factory_adc_param(byte numchan)
{
	byte type_of_calibration = CHAN_CALIB_CHECK_FACTORY_VAL;
	
	if(!EE_write(&Chan[numchan].WeightFactoryGain, sizeof(float), (ADDRESS_FACTORY_GAIN + numchan*BASESTRUCTADDR)))
	{
		return 0;
	}
	if(!EE_write(&Chan[numchan].WeightFactoryOffset, sizeof(short), (ADDRESS_FACTORY_OFFSET + numchan*BASESTRUCTADDR)))
	{
		return 0;
	}
	if(!EE_write(&Chan[numchan].AdcOf2Kg, sizeof(short), (ADDRESS_2Kg_READ + numchan*BASESTRUCTADDR)))
	{
		return 0;
	}
	if(!EE_write(&type_of_calibration, 1, (ADDRESS_CHAN_ARE_MEM_CALIB + numchan*BASESTRUCTADDR)))
	{
		return 0;	// ricorda che questo assume due valori a seconda che sia stata eseguita la sola calibrazione di fabbrica oppure anche quella user.
	}
	
	return 1;
	
}

/**
* Saves in EEPROM last calibration date, overwriting previously set
*/
int backup_date_adc_param(char *s)
{
	if(!EE_write(s, sizeof(short), (ADDRESS_DATE_OF_CALIB)))
	{
		return 0;
	}
	
	return 1;
	
}

/**
* Controls offset and gain values read from EEPROM are in the ranges expected.
* Returns true if all values are as the ones expected, false otherwise.
*/
bool controlAdcValuesRead()
{
	byte numchan;
	bool result;

	bool value_return = True;

	for( numchan = 0; numchan < _MAX_LOAD_CHAN_; numchan++)
	{
		result = False;
		
		if((Chan[numchan].WeightFactoryGain <= WEIGHT_GAIN_LIMIT_HIGH) && (Chan[numchan].WeightFactoryGain > WEIGHT_GAIN_LIMIT_LOW))
		{
			if((Chan[numchan].Weightgain <= WEIGHT_GAIN_LIMIT_HIGH) && (Chan[numchan].Weightgain > WEIGHT_GAIN_LIMIT_LOW))
			{
				if((Chan[numchan].WeightFactoryOffset <= WEIGHT_OFFSET_LIMIT_HIGH) && (Chan[numchan].WeightFactoryOffset >= WEIGHT_OFFSET_LIMIT_LOW))
				{
					if((Chan[numchan].Weightoffset <= WEIGHT_OFFSET_LIMIT_HIGH) && (Chan[numchan].Weightoffset >= WEIGHT_OFFSET_LIMIT_LOW))
					{
						result = True;
					}
				}
			}
		}

		if(!result)
		{
			value_return = False;
			resetAdcValues(numchan);
			backup_calib_state(numchan, (byte)Chan[numchan].AreCalibrate);
		}
	}

	return value_return;
	
}


/**
* Resets values associated to weight channels.
*/
void resetAdcValues(byte __line)
{
	Chan[__line].WeightFactoryGain = WEIGHT_DEFAULT_GAIN;
	Chan[__line].WeightFactoryOffset = WEIGHT_DEFAULT_OFFSET;
	Chan[__line].Weightgain = WEIGHT_DEFAULT_GAIN;
	Chan[__line].Weightoffset = WEIGHT_DEFAULT_OFFSET;
	Chan[__line].AdcOf2Kg = 0;
	Chan[__line].AreCalibrate = False;
}

/*
word FIR_filter(int x)
{
word value = 0;

short int cn;
static byte index_buf_fir = 0;
float sum_dgt_filt = 0;
word value2 = 0;

	if(start_load_fir)	 // the buffer is full
	{
		med_buffer_adc3[0] = x;
		sum_dgt_filt = 0;
	  	for( cn = (DIM_DGT_FILT-1); cn >= 0; cn--)	// da DIM_DGT_FILT-1 a 0
		{
	       	 sum_dgt_filt = sum_dgt_filt + ( rampa[cn] * med_buffer_adc3[ cn]);
	       	 
	      		 if( cn < (DIM_DGT_FILT-1) )				//elimino dalla FIFO il dato + vecchio
	              	med_buffer_adc3[ cn + 1 ] = med_buffer_adc3[ cn ];
		}	
		value2 = (word)(sum_dgt_filt / somma_rampa); 
		value = (((int)( sum_dgt_filt / somma_rampa )) >> 2) & 0x0000FFFF;
	} 
	else 
	{ 	    	
		med_buffer_adc3[ DIM_DGT_FILT - index_buf_fir ] = x; 			// memorizzo nel buffer	    
		value = 0;
		index_buf_fir ++ ; 					// fill the buffer proceeding on the right
	    	if(index_buf_fir == DIM_DGT_FILT)		// sono arrivato al max 
	    	{
	           	start_load_fir = True;   
	           	index_buf_fir = 0;
	    	}   
	} 
	return value;
}

word average_filter(int x)
{
short int cn;
static byte index_buf_fir = 0;
static unsigned int sum_filt_buf = 0;
word value = 0;
unsigned int int_value = 0;
unsigned int currADCmin, currADCmax;
static bool start_load_fir = False;
	
	if(start_load_fir)	 // the buffer is full
	{
		currADCmin = MAX_ADC;
	    	currADCmax = 0;
		for (cn = 0; cn < DIM_AVE_FILT; cn++)
		{	    
			if( med_buffer_adc3[ cn ] > currADCmax )
				currADCmax = med_buffer_adc3[ cn ];
			if (med_buffer_adc3[ cn ] < currADCmin)
				currADCmin = med_buffer_adc3[ cn ];				
		}
		sum_filt_buf -= med_buffer_adc3[ DIM_AVE_FILT - 1]; // subtract the old value
	  	for( cn = (DIM_AVE_FILT-2); cn >= 0; cn--)	// da DIM_DGT_FILT-1 a 0
		{	       	 
	              med_buffer_adc3[ cn + 1 ] = med_buffer_adc3[ cn ];
		}
		med_buffer_adc3[0] = x;
		sum_filt_buf += x;
		int_value = (sum_filt_buf - currADCmax - currADCmin) / (DIM_AVE_FILT - 2);	// -2 perch� non considero minimo e massimo
		value = (int_value >> 2) & 0x0000FFFF;
	} 
	else 
	{ 	    	
		med_buffer_adc3[ DIM_AVE_FILT - index_buf_fir - 1 ] = x; 			// memorizzo nel buffer	 
		sum_filt_buf += x;
		value = 0;
		index_buf_fir ++ ; 					// fill the buffer proceeding on the right
	    	if(index_buf_fir == DIM_AVE_FILT)		// sono arrivato al max 
	    	{
	           	start_load_fir = True;   
	           	index_buf_fir = 0;
	    	}   
	} 
	return value;
}

word average_filter_adc1(int x)
{
short int cn;
static byte index_buf_fir = 0;
static unsigned int sum_filt_buf = 0;
word value = 0;
unsigned int int_value = 0;
unsigned int currADCmin, currADCmax;
static bool start_load_fir = False;
	
	if(start_load_fir)	 // the buffer is full
	{
		currADCmin = MAX_ADC;
	    	currADCmax = 0;
		for (cn = 0; cn < DIM_AVE_FILT; cn++)
		{	    
			if( med_buffer_adc1[ cn ] > currADCmax )
				currADCmax = med_buffer_adc1[ cn ];
			if (med_buffer_adc1[ cn ] < currADCmin)
				currADCmin = med_buffer_adc1[ cn ];				
		}
		sum_filt_buf -= med_buffer_adc1[ DIM_AVE_FILT - 1]; // subtract the old value
	  	for( cn = (DIM_AVE_FILT-2); cn >= 0; cn--)	// da DIM_DGT_FILT-1 a 0
		{	       	 
	              med_buffer_adc1[ cn + 1 ] = med_buffer_adc1[ cn ];
		}
		med_buffer_adc1[0] = x;
		sum_filt_buf += x;
		int_value = (sum_filt_buf - currADCmax - currADCmin) / (DIM_AVE_FILT - 2);	// -2 perch� non considero minimo e massimo
		value = (int_value >> 2) & 0x0000FFFF;
	} 
	else 
	{ 	    	
		med_buffer_adc1[ DIM_AVE_FILT - index_buf_fir - 1 ] = x; 			// memorizzo nel buffer	 
		sum_filt_buf += x;
		value = 0;
		index_buf_fir ++ ; 					// fill the buffer proceeding on the right
	    	if(index_buf_fir == DIM_AVE_FILT)		// sono arrivato al max 
	    	{
	           	start_load_fir = True;   
	           	index_buf_fir = 0;
	    	}   
	} 
	return value;
}
*/
