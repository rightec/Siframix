/**
@file		Main.cpp
@brief		CPU SIFRA MODULE Main file			
@author		Francesco Fregni
@date		18/01/2011
@version	01.00
*/

#include "main.h"
/**
Application firmware name. Max length 32 bytes
*/
const char sAppName[]     = "CPU_M3300";
/**
Application firmware Revision. Max length 16 bytes
*/
const char sAppRevision[] = "00.04.54";
/**
Board's hardware revision.
*/
const char sHwVersion[] = "01.00.01";
/**
Application firmware's revision date.
*/
const char sAppDate[]     = "06/10/20";
//const char sAppDate[]     = "10/05/21";
/**
sAppName length
*/
const int sAppNameLength = (sizeof(sAppName)/sizeof(char) -1);
/**
sAppRevision length
*/
const int sAppRevisionLength = (sizeof(sAppRevision)/sizeof(char) -1);
/**
sAppName length
*/
const int sHwVersionLength = (sizeof(sHwVersion)/sizeof(char) -1);
/**
sAppDate length
*/
const int sAppDateLength = (sizeof(sAppDate)/sizeof(char) -1);
/**
@see GenericHwChanManager
*/
GenericHwChanManager <dword, _WEIGHT_BUFFER_LENGTH_> *weightChan;
/**
@see VoltageHwChanManager
*/
VoltageHwChanManager <word, _PWR_VOLTAGE_BUFFER_LENGTH_> *powerVoltageChan;
/**
@see SIFRAPcProtocol
*/
SIFRAProtocol *SifraProtocol;
/**
@see SIFRA_Manager
*/
SIFRA_Manager *CpuSifra;

UWORD time_msec;						// timer globalissimo !!
UWORD time_sec;				    		// timer globalissimo !!

//-------------------------------------------------------
// Variabili locali utili a moduli esterni
//-------------------------------------------------------
//-------------------------------------------------------
// Variabili Esterne
//-------------------------------------------------------
extern ChannelsBackupParam Chan[];

//-------------------------------------------------------
// Variabili Globabi
//-------------------------------------------------------

//-------------------------------------------------------
// Variabili  utili	solo a questo modulo
//-------------------------------------------------------

//-------------------------------------------------------
// Costanti	statiche utili solo	a questo modulo
//-------------------------------------------------------

//-------------------------------------------------------
// Funzioni	statiche utili solo	a questo modulo
//-------------------------------------------------------

void FlashLed(void);
void counter_manager();
static void hwsw_init(void);
static void pin_init(void);
static void pin_pld_init();
static void IrqHandTM0(int vector);
static void SetupTimers(void);
static bool HwChannels_init();
static void UartProcol_init();
byte Asciihex_to_num (char *pc);   // Converte carattere asci esadecimale in numero
void Num_to_Asciihex (word n, char *pc);

//-------------------------------------------------------

int	main()
{
	hwsw_init();
	SetupTimers();
	
	asm	( "ei" );					// start a tutte le interruzioni
	load_startSampling();
	
	while( 1 )
	{
		FlashLed();
		pwmManager();
		
		CS5530_EntrySpi1();		// lettura ADC 1 e 2
		CS5530_EntrySpi2();		// lettura ADC 4 e 5

		CS5530_validDataPresentInADCBuffer();		// estrazione nuovi campioni
		CS5530_validDataPresentInADCBufferSpi2();
		
		EVsManager();
		
		CpuSifra->Manager();	
	}
}

/**
Flash board's led
*/
void FlashLed(void)
{
static long	blink;

	blink++;

	if(	blink == 4000 )
	{
		PinLED0_LIFE_M  = 1;				// toggle solo P63 cosi' p64 rimane	libero per la comunicazione
		cpld_pin_led = 0;
	}							// e/o la segnalazione del vpp

	if(	blink == 8000 )
	{
		PinLED0_LIFE_M  = 0;
		cpld_pin_led = 1;
	}

	if( blink == 20000 )
	{
		PinLED0_LIFE_M  = 1;				// toggle solo P63 cosi' p64 rimane	libero per la comunicazione
		cpld_pin_led = 0;
	}							// e/o la segnalazione del vpp

	if( blink == 24000 )
	{
		PinLED0_LIFE_M = 0;				// toggle solo P63 cosi' p64 rimane	libero per la comunicazione
		cpld_pin_led = 1;
		blink = 0;
	}							// e/o la segnalazione del vpp
}

/**
Hardware and software initialization
*/
void hwsw_init(void)
{
	asm	( "di" );			// stop	a tutte	le interruzioni

	pin_init();				// init	pins. locali
	pin_pld_init();		// led pld e segnali di blocco relee
	mem_init();
	irq_init();
	i2c_init();             // init i2c virtuale	
	EE_init();
	adc_uC_init();
	LedDrivers_init();		// init interface to panel led drivers
	adc_serial_init();		
	UartProcol_init();		// new SIfraPorotocol
	
	CpuSifra = new SIFRA_Manager(SifraProtocol);

	if(!HwChannels_init())
	{
		while(1);
	}
	
	CpuSifra->setWeightChan(weightChan);	//CpuSifra->setLoadChan(LoadChan);	
	CpuSifra->setVoltageHwChanManager(powerVoltageChan);
	
	CS5530_Spi1Initialization();
	CS5530_Spi2Initialization();
	
}


/**
Harware channels initialization - intanzio le classi
*/
static bool HwChannels_init()
{
	byte i;
	
	//--- Weight Channels ---
	weightChan = new GenericHwChanManager <dword, _WEIGHT_BUFFER_LENGTH_> (_MAX_LOAD_CHAN_, _HW_CHAN_VV_TYPE_, _24_BITS_RESOLUTION_, 10);
				
	for (i = 0; i < _MAX_LOAD_CHAN_; i++)
	{
		weightChan->setEnabled(i, True);
		weightChan->setAverageFilter(i, True);
		weightChan->setGain(i, Chan[i].Weightgain);
		weightChan->setOffset(i, Chan[i].Weightoffset);
		weightChan->setChanType(i, _HW_CHAN_VV_TYPE_);
		weightChan->set_filter_value(i, 1);	// settaggio del filtro di media sul peso: 1*10 --> 66Hz/10 = circa 6,6Hz
	}
	//--- power Voltage channel ---
	powerVoltageChan = new VoltageHwChanManager <word, _PWR_VOLTAGE_BUFFER_LENGTH_> (1, _10_BITS_RESOLUTION_, 1);
	powerVoltageChan->setVoltageRefernce_mV(_INTERNAL_ADC_REF_mV_);
	powerVoltageChan->setIsZeroable(0, false);
	powerVoltageChan->setOffset(0, 0);
	powerVoltageChan->setGain(0, (powerVoltageChan->getVoltageRefence_mV()/powerVoltageChan->getAdcResolution()) * (5.0/3.0));
	powerVoltageChan->setEnabled(0, True);

	return True;

}

/**
Uart communication protocol initialization
*/
static void UartProcol_init()
{
	SifraProtocol = new SIFRAProtocol();			// inizializza il protocollo, stato iniziale della stduart = stdUart_waitDeviceId
}

/**
uC pin initialization.
*/
void pin_init(void)
{
	
	ioPinLED0_LIFE_M = 0;
	PinLED0_LIFE_M = 0;

	// Pin dalla CPLD ---
	//ioPinSPICPLDbusy = 1;
	//ioPinSPICPLDdataREADY = 1;
	PinEnRS485_DE = 0;
 	ioPinEnRS485_DE = 0;
	//-- Buzzer --
	ioPinBUZ = 0;
	PinBUZ = 0;
//-- pin x debug --
	 FDLM0	= 0;
	 ioFDLM0 = PDIR_OUTPUT;

	 FDLM1 = 0;
	 ioFDLM1 = PDIR_INPUT;

	 DRST = 0;
	 ioDRST = PDIR_INPUT;
	 _OCDM0 = 0;
	//-- Definizioni richieste per eeprom  --
	 PinI2C_SDA = 0;
	 ioPinI2C_SDA = PDIR_OUTPUT;
	 PinI2C_SCL = 0;
	 ioPinI2C_SCL = PDIR_OUTPUT;

	 PinWReeprom = 1;
	 ioPinWReeprom = PDIR_OUTPUT;
	 PinCSadc1 = 1;
	 ioPinCSadc1 = PDIR_OUTPUT;
	 
	 PinCMD_EV1 = 1;
	 ioPinCMD_EV1 = 0;	// pinout comando EV1
	 PinCMD_EV2 = 1;
	 ioPinCMD_EV2 = 0;	// pinout comando EV2
	 PinCMD_EV3 = 1;
	 ioPinCMD_EV3 = 0;	// pinout comando EV3
	 PinCMD_EV4 = 1;
	 ioPinCMD_EV4 = 0;	// pinout comando EV4
	 PinCMD_EV5 = 1;
	 ioPinCMD_EV5 = 0;	// pinout comando EV5
	 PinCMD_EV6 = 1;
	 ioPinCMD_EV6 = 0;	// pinout comando EV6
	 PinCMD_EV7 = 1;
	 ioPinCMD_EV7 = 0;	// pinout comando EV7
	 PinCMD_EV8 = 1;
	 ioPinCMD_EV8 = 0;	// pinout comando EV8

	 PinzaEXT = 0;		// pinza chiusa
	 ioPinzaEXT = 0;		// output
	 CMD_EXT = 0;
	 ioCMD_EXT = 0;
}

/**
Initialization of output pins of pld, which controll led_life_pld, enable/disable relays motor
*/
void pin_pld_init()
{
	cpld_pin_led = 1;
	cpld_pin_BLK1 = 0;	// relay motore 1 aperto
}

/**
* Timer initialization
*/
void SetupTimers(void)
{
						//////////////////////////////////////////////////////////
						//		Riepilogo risorse utilizzate
						//
						//	TM0	-> timer di sistema base dei tempi 1ms
						//	TMP1	-> bese dei tempi per campionamento base dei tempi xx
						//
						//////////////////////////////////////////////////////////

	//---------- TMM0 --------
	irq_setVect(IRQ_VEC_INTTM0EQ0, IrqHandTM0);

	//-- Counter frequency	--> fc=fxx/4
	//-- Bits in TM0CTL0 register
	_TM0CE = 0;
	TM0CTL0 = 0x02;

	TM0CMP0 = 4999; // Frequenza di interrupt --> fi =	fc / (TM0CCR0 + 1)

	TM0EQIC0 = 0x05; // Interrupt abilitato con priorità 5

	_TM0CE = 1; // TMP0 start
	
}


/**
ISR used as time base for the uC sofware time. Interrupt time 1 msec.
*/
void IrqHandTM0(int vector)
{
	//PinLED1_LIFE_M = !PinLED1_LIFE_M;
	time_msec++;
	globalTimer.clockTick(1, 0);	// avanti il contatore di riferimento per i	timer sw

}


// Interrupt handler timer P-0 base	tempi 100us

byte Asciihex_to_num (char *pc)           // Converte carattere asci esadecimale in numero
{
   if(*pc>0x2F && *pc<0x3A)               // Se carattere asci è un numero
   {
      return(*pc-0x30);                   // Toglie offset e lo restituisce
   }
   else
   {
      if(*pc>0x40 && *pc<0x47)            // Se è inoltre un carattere esadecimale
      {
         return(*pc-0x37);                // Toglie offset e lo restituisce
      }
      else
      {
         return(0x00);                    // Se nessuno dei casi , torna 0
      }
   }
}

void Num_to_Asciihex (word n, char *pc)
{
	if(n > 999)
	{
		pc[0]=0x30; pc[1]=0x30; pc[2]=0x30;
		return;
	}

	pc[0] = (n / 100) + 0x30;
	n 	  = (n % 100)       ;
	pc[1] = (n / 10)  + 0x30;
	pc[2] = (n % 10)  + 0x30;
}

/*
void counter_manager()
{
static long	count;

	count++;
	passi_motore = cpld_counter1;
	// 500 imp/giro su rotore motore
	// il motoriduttore ha riduzione 1:6, cioè 1 giro del rotore ogni 6 giri del motore
	// -> un giro rotore = 3000imp

	if( count == 1000 )
	{
		count = 0;
		if( cpld_counter1 >= 0x0BB8)	// leggo il contatore 0: se ha raggiunto i 10 conteggi, si resetta e suona il buzzer
		{
			cpld_counter1 = 0x0000;
			goBuzzer(KEY_LIKE);
		}
	}
}
*/
/**
RAM memory test function
@param ram_start ram start address
@param ram_end ram end address
@return always 0
*/
/*
int GrCpuMemTest(word *ram_start, word *ram_end)
{

	//////////////////////////////////////////////////////////////////////////
	// test ram-esterna
	//////////////////////////////////////////////////////////////////////////
	asm( "test_mem: 																					");
	asm( "                                                                                      		");
	asm( "			di							-- disable interrupts                            		");
	asm( "                                                                                      		");
//	asm( "---------------------------------------------------                                   		");
//	asm( "-- Inizio test ram esterna                                                          			");
//	asm( "--                                                                                    		");
	asm( "test_xram:                                                                            		");
	asm( "																								");
//	asm( "			movhi	hi(0x200000),r0,r6	-- r6 -> ram begin                             			");
//	asm( "			movhi	hi(0x220000),r0,r7	-- r7 -> ram end                             			");
//	asm( "			movea	lo(0x200000),r6,r6	-- r6 -> ram begin                             			");
//	asm( "			movea	lo(0x220000),r7,r7	-- r7 -> ram end                               			");
//	asm( "			movea	lo(0x27FFFF),r7,r7	-- r7 -> ram end                               			");

	asm( "			mov		r6,r8				-- r8 -> (copy-of) ram begin                           	");
	asm( "			mov		-1,r9				-- r9 -> filler				                          	");

//	asm( "---------------------------------------------------                                   		");
//	asm( "-- Primo step: fill della memoria con FFFF (bus a 16bit)                                      ");
	asm( "xram_fill:		                                                                            ");
	asm( "			st.h	r9,0[r6]			-- filla...     										");
	asm( "			add		2,r6				-- avanti il prossimo 				  					");
	asm( "			cmp		r7,r6				-- fine fillo ???				     					");
	asm( "			bl		xram_fill			-- se no salta 											");

//	asm( "---------------------------------------------------                                   		");
//	asm( "-- Secondo step: clear della memoria bit a bit                                    			");
	asm( "         	mov		1,r9                -- r9 ->                                                ");
	asm( "         	movhi	1,r0,r12			-- r12 -> fine test                                     ");
	asm( "          mov     r8,r6				-- r6 -> xram begin                                     ");
	asm( "          mov     r0,r10				-- r10 -> buffer                                        ");

	asm( "xram_clrbit:		                                                                            ");
	asm( "			ld.h	0[r6],r10			-- piglia...     										");
	asm( "			tst		r9,r10				-- test nuovamenteo col bit di test 					");
	asm( "			be		xram_clrbit_err		-- se non è alto segnala l'errore 						");

	asm( "										-- azzera il bit pure in memoria						");
	asm( "			xor		r9,r10				-- maschera col bit di test 							");
	asm( "			st.h	r10,0[r6]			-- riscrive...     										");

	asm( "			shl		1,r9				-- shifta il bit di test...     						");
	asm( "			tst		r12,r9				-- testa la fine del test ...     						");
	asm( "			bne		xram_clrbit			-- se non è finito salta  								");

	asm( "			add		2,r6				-- avanti il prossimo 				  					");
	asm( "         	mov		1,r9                -- r9 -> test bit                                       ");
	asm( "			cmp		r7,r6				-- fine test ???				     					");
	asm( "			bl		xram_clrbit			-- se no salta 											");

	asm( "xram_clrbit_ok:		                -- tutto ok: test passato                               ");
	asm( "			br		test_mem3			-- passa avanti											");

	asm( "xram_clrbit_err:		                                                                        ");
	asm( "         	mov		1,r10                                                                       ");
	asm( "         	jmp		[r31]		                                                                ");


//	asm( "---------------------------------------------------                                   		");
//	asm( "-- Terzo step: set della memoria bit a bit                                    			    ");
	asm( "test_mem3: 																					");
	asm( "         	mov		1,r9                -- r9 ->                                                ");
	asm( "         	movhi	1,r0,r12			-- r12 -> fine test                                     ");
	asm( "          mov     r8,r6				-- r6 -> xram begin                                     ");
	asm( "          mov     r0,r10				-- r10 -> buffer                                        ");

	asm( "xram_setbit:		                                                                            ");
	asm( "			ld.h	0[r6],r10			-- piglia...     										");
	asm( "			tst		r9,r10				-- test nuovamenteo col bit di test 					");
	asm( "			bne		xram_setbit_err		-- se è alto segnala l'errore 							");

	asm( "										-- setta il bit pure in memoria							");
	asm( "			or		r9,r10				-- maschera col bit di test 							");
	asm( "			st.h	r10,0[r6]			-- riscrive...     										");

	asm( "			shl		1,r9				-- shifta il bit di test...     						");
	asm( "			tst		r12,r9				-- testa la fine del test ...     						");
	asm( "			bne		xram_setbit			-- se non è finito salta  								");

	asm( "			add		2,r6				-- avanti il prossimo 				  					");
	asm( "         	mov		1,r9                -- r9 -> test bit                                       ");
	asm( "			cmp		r7,r6				-- fine test ???				     					");
	asm( "			bl		xram_setbit			-- se no salta 											");

	asm( "xram_setbit_ok:		                -- tutto ok: test passato                               ");
	asm( "			br		test_mem4			-- passa avanti											");

	asm( "xram_setbit_err:		                                                                        ");
	asm( "         	mov		2,r10                                                                       ");
	asm( "         	jmp		[r31]		                                                                ");


//	asm( "---------------------------------------------------                                   		");
//	asm( "-- Quarto step: clr della memoria bit a bit                                    			    ");
	asm( "test_mem4: 																					");
	asm( "         	movea	0x8000,r0,r9        -- r9 -> test bit                                       ");
	asm( "         	mov		0,r12				-- r12 -> fine test                                     ");
	asm( "          mov     r8,r6				-- r6 -> xram begin                                     ");
	asm( "          mov     r0,r10				-- r10 -> buffer                                        ");

	asm( "xram_mskbit:		                                                                            ");
	asm( "			ld.h	0[r6],r10			-- piglia...     										");
	asm( "			tst		r9,r10				-- test nuovamenteo col bit di test 					");
	asm( "			be		xram_mskbit_err		-- se non è alto segnala l'errore 						");

	asm( "										-- azzera il bit pure in memoria						");
	asm( "			xor		r9,r10				-- maschera col bit di test 							");
	asm( "			st.h	r10,0[r6]			-- riscrive...     										");

	asm( "			shr		1,r9				-- shifta il bit di test...     						");
	asm( "			tst		r12,r9				-- testa la fine del test ...     						");
	asm( "			bne		xram_mskbit			-- se non è finito salta  								");

	asm( "			add		2,r6				-- avanti il prossimo 				  					");
	asm( "         	mov		1,r9                -- r9 -> test bit                                       ");
	asm( "			cmp		r7,r6				-- fine test ???				     					");
	asm( "			bl		xram_mskbit			-- se no salta 											");

	asm( "xram_mskbit_ok:		                -- tutto ok: test passato                               ");
	asm( "			br		test_mem5			-- passa avanti											");

	asm( "xram_mskbit_err:		                                                                        ");
	asm( "         	mov		3,r10                                                                       ");
	asm( "         	jmp		[r31]		                                                                ");


//	asm( "---------------------------------------------------                                   		");
//	asm( "-- Quinto step: test address bus				                                  			    ");
	asm( "test_mem5: 																					");
	asm( "         	movea	0x1188,r0,r9        -- r9 -> test bit                                       ");
	asm( "         	movea	0x8811,r0,r12		-- r12 -> test adder                                    ");
	asm( "          mov     r8,r6				-- r6 -> xram begin                                     ");
	asm( "          mov   	r0,r10				-- r10 -> counter                                       ");

	asm( "xram_tstbus:		                                                                            ");
	asm( "			st.h	r9,0[r6]			-- filla...     										");

	asm( "			add		3,r10				-- avanti il prossimo count			  					");
	asm( "			add		r12,r9				-- avanti il prossimo count			  					");
	asm( "			add		2,r6				-- avanti il prossimo address		  					");
	asm( "			add		r10,r9				-- avanti il prossimo count			  					");
	asm( "			cmp		r7,r6				-- fine fillo ???				     					");
	asm( "			bl		xram_tstbus			-- se no salta 											");

	asm( "         	movea	0x1188,r0,r9        -- r9 -> test bit                                       ");
	asm( "         	movea	0x8811,r0,r12		-- r12 -> fine test                                     ");
	asm( "          mov     r8,r6				-- r6 -> xram begin                                     ");
	asm( "          mov   	r0,r10				-- r10 -> counter                                       ");
	asm( "          mov   	r0,r11				-- r11 -> buffer                                       	");

	asm( "xram_verbus:		                                                                            ");
	asm( "			andi	lo(-1),r9,r9		-- tosa un attimo r9 									");
	asm( "			ld.h	0[r6],r11			-- piglia il valore prec...								");
	asm( "			andi	lo(-1),r11,r11		-- tosa un attimo r11 									");
	asm( "			cmp		r9,r11				-- è uguale alla serie sviluppata prima 				");
	asm( "			bne		xram_tstbus_err		-- se no salta (siamo in buca)							");

	asm( "			add		3,r10				-- avanti il prossimo count			  					");
	asm( "			add		r12,r9				-- avanti il prossimo count			  					");
	asm( "			add		2,r6				-- avanti il prossimo address		  					");
	asm( "			add		r10,r9				-- avanti il prossimo count			  					");
	asm( "			cmp		r7,r6				-- fine fillo ???				     					");
	asm( "			bl		xram_verbus			-- se no salta 											");

	asm( "xram_tstbus_ok:		                -- tutto ok: test passato                               ");
	asm( "         	mov		r0,r10											                            ");
	asm( "         	jmp		[r31]		                                                                ");

	asm( "xram_tstbus_err:		                -- buca !!: qualcosa non va nel bus.                    ");
	asm( "         	mov		4,r10                                                                       ");
	asm( "         	jmp		[r31]		                                                                ");


	//////////////////////////////////////////////////////////////////////////
	// test ram-interna
	//////////////////////////////////////////////////////////////////////////

	return(0);
}
*/




