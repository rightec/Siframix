/**
@file		Adc.h
@brief		Functions to handle internal and extern ADC.

			The internal adc samples the board current consumption and voltage. The external adc samples the impedance values.
			
@author		Francesco Fregni
@date		18/01/2011
@version	01.00
*/

#ifndef _ADC_H
#define _ADC_H

#include "global.h"				
#include "swtim.h"	
#include "sioSG2.h"
#include "alloc.h"
#include "I2Ceeprom.h"
#include "channels.h"

//-- deifne per canali analogici generici ---
#define		CHAN_NAME_SIZE			6 // E' la stessa cosa di SIZEOF_CHAN_NAME definita in esame.h
#define		CHAN_LOC_SIZE			6

#define _INTERNAL_ADC_REF_mV_	3300
#define _TIME_	1
#define _MASK_24BIT_VALID 		0x00FFFFFF	// il dato letto a 32bit dall'ADC, solo 24bit sono di dato campionato
#define _MASK_IF_OVERFLOW		0x00000004	// verifica il bit di overflow
#define CS5530_gain			0x01200000	// guadagno imposto di 1.125
//#define CS5530_gain			0x01400000	// guadagno imposto di 1.250
//#define CS5530_gain			0x01000000	// 1 per uniformarlo al M3100
#define CS5530_doublegain	0x01700000	// guadagno imposto di 1.4375 corrispondente a 13500 punti per 1000
#define MASK_CONFIG_REG	0xFB887E00	// conf reg : XXXXX0XXX000X0000XXXXXX000000000

#define MAX_VALUE_16_BIT	65535

enum read_gain_offset_value_t
{
	E_READ_IDLE = 0,
	E_READ_ERROR,
	E_READ_NO_BK_FACTORY,
	E_READ_BK_FACTORY,
	E_READ_NUM
};

enum ChanCATEGORY
{
	ANALOG_uC,
	ANALOG_SERIAL_ADC,
	POWER, 
	DIGITAL, 
	VIRTUAL, 
	UNDEFINED,
};

enum ANALOGType
{
	UNCODED1, 
	FLOW_TYPE, 
	UEC_TYPE, 
	BREATH_TYPE, 
	PRES_TYPE, 
	EMG_TYPE, 
	PH_TYPE, 
	DETACHED, 
	SpO2, 
	HEART_BEAT,
	IMPEDANCE,
	EXT_TYPE,
};

enum Index_Udm 
{
	U_PASCAL, 
	U_MMHG, 
	U_CMH2O, 
	U_ML_SEC, 
	U_MILLIVOLT, 
	U_MICROVOLT, 
	U_PH, 
	U_Spo2, 
	U_Puls, 
	U_ML, 
	Upercent, 
	U_SEC, 
	U_MIN, 
	U_ORE, 
	NOUDM
};

enum CS5530_sampleFrequecies
{
	CS5530_sf_6_25_Hz = 0,
	CS5530_sf_7_5_Hz,
	CS5530_sf_12_5_Hz,
	CS5530_sf_15_Hz,
	CS5530_sf_25_Hz,
	CS5530_sf_30_Hz,
	CS5530_sf_50_Hz,
	CS5530_sf_60_Hz,
	CS5530_sf_100_Hz,
	CS5530_sf_120_Hz,
	CS5530_sf_200_Hz,
	CS5530_sf_240_Hz,
	CS5530_sf_400_Hz,
	CS5530_sf_480_Hz,
	CS5530_sf_800_Hz,
	CS5530_sf_960_Hz,
	CS5530_sf_1600_Hz,
	CS5530_sf_1920_Hz,
	CS5530_sf_3200_Hz,
	CS5530_sf_3840_Hz,
};

typedef CS5530_sampleFrequecies CS5530_sampleFrequecies;

enum CS5530_command
{
	CS5530_wr_offset_register = 0x01,
	CS5530_rd_offset_register = 0x09,	
	CS5530_wr_gain_register = 0x02,
	CS5530_rd_gain_register = 0x0A,
	CS5530_wr_config_register = 0x03,
	CS5530_rd_config_register = 0x0B,
	CS5530_perform_single_conversion = 0x80,
	CS5530_perform_continuous_conversion = 0xC0,
	CS5530_perform_sys_offset_cal_register = 0x85,
	CS5530_perform_sys_gain_cal_register = 0x86,
	CS5530_sync1 = 0xFF,
	CS5530_sync0 = 0xFE,
	CS5530_null = 0x00,
	CS5530_end_continuous_conversion = 0xFD,
	CS5530_read_adc_data = 0xFC,
	CS5530_sendInitSyncSequence = 0xFB
};
typedef CS5530_command CS5530_command;

enum CS5530_powerSaveMode
{
	CS5530_standbyMode = 0,
	CS5530_sleepMode = 1,	
};
typedef CS5530_powerSaveMode CS5530_powerSaveMode;

enum CS5530_powerDownMode
{
	CS5530_normalMode = 0,
	CS5530_powerDown = 1,	
};
typedef CS5530_powerDownMode CS5530_powerDownMode;

enum CS5530_rstSys
{
	CS5530_normalOperation = 0,
	CS5530_rstCycle = 1,	
};
typedef CS5530_rstSys CS5530_rstSys;

enum CS5530_inputShort
{
	CS5530_normalInput = 0,
	CS5530_shortedInput = 1,	
};
typedef CS5530_inputShort CS5530_inputShort;

enum CS5530_voltageRef
{
	CS5530_higherThan_2_5V = 0,
	CS5530_lowerOrEqualThan_2_5V = 1,	
};
typedef CS5530_voltageRef CS5530_voltageRef;

enum CS5530_outputLatchBits
{
	CS5530_latchBits_LL = 0,
	CS5530_latchBits_LH = 1,
	CS5530_latchBits_HL	= 2,
	CS5530_latchBits_HH = 3,
};
typedef CS5530_outputLatchBits CS5530_outputLatchBits;

enum CS5530_adcCoding
{
	CS5530_bipolar = 0,
	CS5530_unipolar = 1,	
};
typedef CS5530_adcCoding CS5530_adcCoding;

enum CS5530_openCircuitDetector
{
	CS5530_openCircuitDetector_NOT_ACTIVE = 0,
	CS5530_openCircuitDetector_ACTIVE = 1,	
};
typedef CS5530_openCircuitDetector CS5530_openCircuitDetector;

enum CS5530_configRegisterStatus
{
	CS5530_confReg_notUpdated = 0,
	CS5530_confReg_Updated,
};
typedef CS5530_configRegisterStatus CS5530_configRegisterStatus;

#define DIM_BYTE_FIFO	50
#define DIM_BYTE_FIFOADC	15	

#define CHAN_CALIB_CHECK_FACTORY_VAL		0xAA
#define CHAN_CALIB_RESET_VALUE			0x55

#define WEIGHT_DEFAULT_OFFSET		25000	
#define WEIGHT_DEFAULT_GAIN		1.03

#define WEIGHT_DEFAULT_2Kg_LOAD			21000
#define WEIGHT_DEFAULT_2Kg_NO_LOAD		100

#define WEIGHT_GAIN_LIMIT_LOW				0.0
#define WEIGHT_GAIN_LIMIT_HIGH			2.0

#define WEIGHT_OFFSET_LIMIT_LOW			1000
#define WEIGHT_OFFSET_LIMIT_HIGH			40000


typedef struct
{
	ANALOGType 	  	KindOfChan;
	ANALOGType 	  	PlugState;	// Serve per rilevare distacchi o inserzione di nuovi canali
	ChanCATEGORY	Category;
	unsigned char 	Calibration	: 1;
	short int	  		value;			// E' il valore corrente dell'ADC
	short int     		maxvalue;		// Serve a rilevare picchi in modulo dell'ADC.
	short int	  		CalValue[2][2];	// Nel PH corrispone a pH alto
	short int	  		OFFSET[2];
	float  		  	Gain[2];
	short int	  		PhysicalRef[2][2];	// col pH: PhysicalRef[0] = pHHi; PhysicalRef[1] = pHLow;
	float 		  	PhysicalValue;
	unsigned short  	freq; 		 	// La frequenza pu= essere trattata come multipla di 1Hz		
	unsigned short	ToNextSample;	// Conta quanti INTERRUPT da 1msec mancano al prossimo Sample
	char				nameOfChan[CHAN_NAME_SIZE];		// Dati da MMC: solo chiacchere e distintivo
	char				loc[CHAN_LOC_SIZE];
	byte				type;
	Index_Udm		Unity;
	int				range;
	bool				UARTacq;
	bool				isForUro;
// Buffer di filtraggio per rimediare alle bizze dell' ADC del micro nuovo
}AnalogChan;

#define DIM_STRING_DATE_CALIB 9	// data MM/DD/YY + finestringa

//this struct will serves for pressure channel on M330
typedef struct
{
	long			prsOUT_Average;
	byte			prsOUT_num_of_sample;
	short		prsOUTCalVal[2];
	float			prsOUTgain;
	short 		prsOUT_adc_val;
	short		prsOUT_val;
	bool              new_val;
	int			prsERROR;
	bool 		prsAreCalibrate;
} MeasureOfPressure;

typedef struct
{
	float			WeightFactoryGain;		// 4 byte
	word			WeightFactoryOffset;		// 2 byte
	float			Weightgain;				// 4 byte
	word			Weightoffset;			// 2 byte
	int			AdcTo2Kg;				// 4 byte, può essere occasionalmente negativa
	int			AdcTo2Kg_dx;			// 4 byte, può essere occasionalmente negativa
	byte			typeOfOffsetCal;			// 1 byte, usato per leggere il valore del byte che indica se i valori di offset associati alla cella corrispondono ad una calibrazione valida
	byte			typeOfGainCal;			// 1 byte, usato per leggere il valore del byte che indica se i valori di guadagno associati alla cella corrispondono ad una calibrazione valida
	bool 		AreCalibrate;				// 1 byte
} ChannelsBackupParam;

#define CS_ADC1_LOW() (PinCSadc1 = 0)
#define CS_ADC1_HIGH() (PinCSadc1 = 1)
#define CS_ADC2_LOW() (PinCSadc2 = 0)
#define CS_ADC2_HIGH() (PinCSadc2 = 1)
#define CS_ADC3_LOW() (PinCSadc3 = 0)
#define CS_ADC3_HIGH() (PinCSadc3 = 1)
#define CS_ADC4_LOW() (PinCSadc4 = 0)
#define CS_ADC4_HIGH() (PinCSadc4 = 1)
#define CS_ADC5_LOW() (PinCSadc5 = 0)
#define CS_ADC5_HIGH() (PinCSadc5 = 1)

//-----------------------------------------------------------------
//-------- Definizione Funzioni -----------------------------------
//-----------------------------------------------------------------

//------------- Internal ADC --------------------------------------
void adc_uC_init();
void samplePower();

//------------- External Adc --------------------------------------
byte get_factory_adc_param();
byte get_adc_param();
bool backup_new_offset_value(byte numchan, word NewOffset);
bool backup_new_gain_value(byte numchan, float NewGain);
bool backup_factory_offset_value(byte numchan, word FactoryOffset);
bool backup_factory_gain_param(byte numchan, float FactoryGain, int adc2Kg, int adc2Kgdx);
bool backup_calib_state(byte __numchan, byte __calib);
int backup_date_adc_param(char *s);
bool reset_backup_factory_values(byte numchan, byte areCalib);
void adc_serial_init();
void load_startSampling();
void load_stopSampling();

bool controlAdcValuesRead();
void resetAdcValues(byte __line);

int CS5530_Spi1Initialization();
int CS5530_Spi2Initialization();
void CS5530_EntrySpi1();
int CS5530_sendCmd(CS5530_command cmd, byte *param);
void CS5530_setPowerSafeMode(CS5530_powerSaveMode mode);
void CS5530_setPowerDownMode(CS5530_powerDownMode mode);
void CS5530_setRstSys(CS5530_rstSys rst);
void CS5530_setInputShort(CS5530_inputShort input);
void CS5530_setVoltageRef(CS5530_voltageRef voltage);
void CS5530_setLatchBits(CS5530_outputLatchBits latch);
void CS5530_setSampleFrequecy(CS5530_sampleFrequecies frq);
void CS5530_setAdcCoding(CS5530_adcCoding coding);
void CS5530_setOpenCircuitDetector(CS5530_openCircuitDetector ocd);
void endContinuousConversion();
void startContinuousConversion();
void txBufferEmpty();
void txBufferNotEmpty();
byte txBufferIsEmpty();
void CS5530_setCounterWaitingData();
int CS5530_getCounterWaitingData();
bool CS5530_checkBlockComunication(int match_count);
bool CS5530_checkBlockCells(dword *loadsystem);
void CS5530_setWatchdogAdcTimer(dword match_time);
bool CS5530_getWatchdogAdcTimer();
void CS5530_resetAdcComunication();
bool CS5530_validDataPresentInADCBuffer();
void CS5530_rstValidDataPresentInADCBuffer();
void SPI_StartTx();
void setSPI1mode();
void setPINSPI1mode();

void CS5530_EntrySpi2();
int CS5530_sendCmdSpi2(CS5530_command cmd, byte *param);
void CS5530_setPowerSafeModeSpi2(CS5530_powerSaveMode mode);
void CS5530_setPowerDownModeSpi2(CS5530_powerDownMode mode);
void CS5530_setRstSysSpi2(CS5530_rstSys rst);
void CS5530_setInputShortSpi2(CS5530_inputShort input);
void CS5530_setVoltageRefSpi2(CS5530_voltageRef voltage);
void CS5530_setLatchBitsSpi2(CS5530_outputLatchBits latch);
void CS5530_setSampleFrequencySpi2(CS5530_sampleFrequecies frq);
void CS5530_setAdcCodingSpi2(CS5530_adcCoding coding);
void CS5530_setOpenCircuitDetectorSpi2(CS5530_openCircuitDetector ocd);
void endContinuousConversionSpi2();
void startContinuousConversionSpi2();
void txBufferEmptySpi2();
void txBufferNotEmptySpi2();
byte txBufferIsEmptySpi2();
void CS5530_setIntGainADConv(int valgain, byte* valreg);
bool CS5530_validDataPresentInADCBufferSpi2();
void CS5530_rstValidDataPresentInADCBufferSpi2();
void SPI2_StartTx();

void setSPI2mode();
void setPINSPI2mode();

#endif

