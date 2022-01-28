/**
@file		Global.h
@brief		Global definitio and include. 
@author		Nicola Molinazzi
@date		18/01/2011
@version	01.00
*/

#ifndef _GLOBAL_H_
#define _GLOBAL_H_

/** GGG-4.3.1 define
*/
#define __GNUC__ 4
/** GGG-4.3.1 define
*/
#define __GNUG__ 4
/** GGG-4.3.1 define
*/
#define __GNUC_MINOR__ 3			
/** GGG-4.3.1 define
*/
#define __V851__
/** GGG-4.3.1 define
*/
#define __V850__
/** GGG-4.3.1 define
*/
#define __V851
/** GGG-4.3.1 define
*/
#define __V850
/** GGG-4.3.1 define
*/
#define ___cplusplus

/*		Defines per GCC-2.8.1  
#define __GNUC__ 2
#define __GNUG__ 2
#define __GNUC_MINOR__ 8			// ovvero versione 2.8.1

#define ___cplusplus
#define __cplusplus
#define __V851__
#define __V850__
#define __V851
#define __V850
#define __EXCEPTIONS
*/

#define __IEEE_LITTLE_ENDIAN

#include <stdio.h>					
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
//#include <math.h>
#include <fastmath.h>

/** 
	Loader start address.
*/
#define LOADER_FLASH_START_ADDRESS	0x00000008

/**
Application firmware start address.
*/
#define __APPLICATION_START_ADDRESS__	0x7000
/** 
	Firmware identification code. See file startup.s
*/
#define APPLICATION_MAGIC_NUMBER				0x8844CCEE	// memorizzarto all'indirizzo 0x600 - file startup.s
/** 
	Application firmware global checksum address. See file startup.s
*/
#define APPLICATION_GLOBAL_CHECKSUM_ADDRESS		0x0004
/** 
	Firmware identification code address. See file startup.s
*/
#define APPLICATION_MAGIC_NUMBER_ADDRESS		0x0600
/** 
	Application firmware number of segment address. See file startup.s
*/
#define APPLICATION_SEGMENT_NUMBER_ADDRESS  	0x0604
/** 
	Application firmware segment checksum address. See file startup.s
*/
#define APPLICATION_SEGMENT_CHECKSUM_ADDRESS	0x0614

/**
Application type
*/
#define APP_TYPE			APP_CPUSIFRA		

//#define __DEBUG__					// debug mode oppure no
//#define _DEBUG_CALIB_VALUES_
#define _DEBUG_ADC_				// per debuggurae i filtri sull'ADC
#define _NORMALFILTER_	1	// attivazione del filtraggio digitale (media mobile + media fissa + eliminazione del +/- 1 nel valore visualizzato
#define _AVERAGE_MOBILE_FILTER
#define _NEW_FILTERING
//#define _debug_flussopeso_
//#define _FIX_AVERAGE_FILTER
//#define __APP_IN_RAM__				// applicativo caricato in ram

#define True	true			
#define False	false

#define Enable	true			
#define Disable	false

#define HW_VERSION		001 //0.0.1

// tipo di scheda
#define _M3300_CPU

enum
{
	M3100 = 0x01,
	M3300 = 0x05,
	M3800 = 0x04,
	STOP_ALL = 0x03,
	UNKNOWN = 0XFF
};

#define ON		1
#define OFF		0
#define _RICEVI_PESI_SPECIFICI 

//	MACROS di importazione
#define LOBYTE(x)             ((byte) ((x) & 0xff))
#define HIBYTE(x)             ((byte) ((x) >> 8))
#define BYTES_TO_WORD(hi,lo)  (word) (((word)(hi) << 8) | (word)(lo))

#define _ALTA_TOLL	1
#define _BASSA_TOLL	0

/**
Power voltage measured for diagnostic
*/
enum BoardVoltageMeasures
{
    _24VDC_ = 0,
    _12VDC_,
    _5VDC_DIGITAL_,
    _5VDC_ANALOG_,
    _5VDC_BUBBLE_,
    _V_2_5_,
    _V_1_2_,
    _VCC_VINF_,
    _VCC_VV_,
    _VCC_PRS14_,
    _VCC_PRS58_,
    _NUM_VOLTAGE_MEASURE_,
};

//--------------------------------------------------------//
// Definizione dei tipi
//--------------------------------------------------------//

#ifdef _M3300_CPU
	#define _MAX_LOAD_CHAN_		2	// max 2celle nel m3300 con misura differenziale
	#define NUM_MAX_LINE			8
	#define _NUM_PUMPS				1
#endif

#define _NUM_MIN_DATA_in_FIFO			2
#define _LOAD_BUFFER_LENGTH_  			10
/**
Weight buffer length.
*/
#define _WEIGHT_BUFFER_LENGTH_ 		10
/**
Generic channel buffer length.
*/
#define _GENERIC_BUFFER_LENGTH_  		60
/**
Power voltage buffer length.
*/
#define _PWR_VOLTAGE_BUFFER_LENGTH_  	5
/**
Number of weight samples per packets 
@see SauPcProtocol::sendSauPcWeightSamples
*/
#define _NUM_WEIGHT_SAMPLES_PER_PACKET_	2
/**
Number of Genric samples per packets 
@see SauPcProtocol::sendSauPcEmgGenSamples
*/
#define _NUM_GEN_SAMPLES_PER_PACKET_	20

/**
kind of weight chan
*/
enum kindOfWeightChan{
	_ADC1_= 0,
	_ADC2_,
	_NUM_OF_WEIGHT_CHAN_
};

enum LANGUEGES
{
	ENGLISH = 0,
	ITALIAN,
	MAX_LANGUAGES
};

/**
word define
*/
typedef	unsigned short	word;
/**
byte define
*/
typedef unsigned char	byte;
/**
double word define
*/
typedef unsigned long	dword;

#ifdef __cplusplus
#else
typedef unsigned char	bool;
#endif

/**
It used to put in a const segment strings pointer array.
*/
typedef const char* textp;			

/**
It used to put in a const segment function pointer array.
*/
typedef void (*funp)();				
/**
It used to put in a const segment function pointer array.
*/
typedef void (*funpv)(void);	
/**
It used to put in a const segment function pointer array.
*/
typedef void (*funpi)(int);
/**
It used to put in a const segment function pointer array.
*/
typedef void (*funpb)(byte);

/**
Struct to manage bits variable.

16 bit are defined because the compilator reserves 2 bytes also if 8 bits variable was defined.
*/
struct BitVar{                       /* tipo per variabile gestita a bittini */
    unsigned B0  : 1;                /* nota: devo definire 16 bit perche` il */
    unsigned B1  : 1;                /* compilatore comunque riserva 2 bytes */
    unsigned B2  : 1;                /* anche se definisco 8 bit */
    unsigned B3  : 1;
    unsigned B4  : 1;
    unsigned B5  : 1;
    unsigned B6  : 1;
    unsigned B7  : 1;
    unsigned B8  : 1;
    unsigned B9  : 1;
    unsigned B10 : 1;
    unsigned B11 : 1;
    unsigned B12 : 1;
    unsigned B13 : 1;
    unsigned B14 : 1;
    unsigned B15 : 1;
};

typedef struct BitVar BitVar;

union IoVar{                        /* aggiunta per gestione a word e char */
    BitVar  bits;
    word    var;
    char    sing[2];
};

typedef union IoVar IoVar;


typedef char             BYTE;
typedef unsigned char   UBYTE;
typedef short            WORD;
typedef unsigned short  UWORD;
typedef long             LONG;
typedef unsigned long   ULONG;

extern UWORD time_msec;
extern UWORD time_sec;
//--------------------------------------------------------//
// Definizione delle classi
//--------------------------------------------------------//

/**
Implements a fifo.
@param T type of data to fifo.
@param capacity fifo length.
*/
template <class T, int capacity>
class  CSmallRingBuf{

public:
	/**
	Class constructor.
	*/
	CSmallRingBuf(){clear(); }

	/**
	Returns if the fifo is empty.
	@return true if the fifo is empty, false otherwise.
	*/
	inline bool empty() const {return count == 0; }
	/**
	Returns if the fifo is full.
	@return true if the fifo is full, false otherwise.
	*/
	inline bool full() const { return count == capacity; }
	/**
	@return the size of T. 
	*/
	inline int size() const { return sizeof(T); }
	/**
	@return the number of items fifoed. 
	*/
	inline int numItem() const {return count;}
	/**
	@return the memrization index.
	*/
	inline int getMemIndex() const{return mem;}
	/**
	@return the fifo capacity. 
	*/
	inline int getCapacity() const{return capacity;}

	/**
	Push a data in the fifo. NOTE: it not check if the fifo is full.
	@param x data to push
	@see full
	*/
	inline void push_back(const T& x) 
								{ 
									datacpy( (byte *) &data[mem], (byte *) &x, (byte *) &data[mem+1]);  
									mem++; 
									count++; 
									if( mem >= capacity ) mem = 0; 
								};
	/**
	Pop a data from the fifo. NOTE: it not check if the fifo is empty.
	@param x data popped
	@see empty
	*/							
	inline void pop_front(T& x) { 
									x = data[get++];
									count--; 
									if( get >= capacity ) 
										get = 0; 
								};
	/**
	Reads a data from the fifo.
	@param x read data.
	@param index reading index
	*/
	inline void read(T& x, int index) { 
										if( index >= capacity ) 
											index = capacity -1;
										if (index < 0)
											index = 0;
										x = data[index];								
									 };
	/**
	Reads the last data pushed.
	@param x data read.
	*/
	inline void readLastPushed(T& x) { 
										int i;
										i = mem;
										if (i == 0)
										{
											i = capacity;
										}
										i--;
										x = data[i];
									};
	/**
	Empties the fifo
	*/
	inline void clear() { mem = get = count = 0; };

protected:
	/**
	Copies data in the fifo.
	@param pd pointer to the start position address within the fifo.
	@param ps poiter to the data to copy.
	@param pe poniter to the end address to copy 
	*/
	inline void datacpy(byte *pd, byte *ps, byte *pe) { 
								for(; pd<pe; ) 		
									*pd++=*ps++; }

	T		data[capacity];
	/**
	Memorization index
	*/
	volatile int	mem; 	
	/**
	Reading index.
	*/
	volatile int	get;	
	/**
	Number of element.
	*/
	volatile int	count;
};


				
#include "v850sg2.h"						// selezione in base al microbo..
#include "iodef.h"							// ridefinizione degli I/O in base al target
#include "irqsg2.h"
//#include "error.h"

#endif

