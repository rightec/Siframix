/**
@file		Channels.h
@brief		Classes to manage harwdare channels.

			Specifically here there are all the classes that manage the hw channels, so:
			- Impedance
			- Pressure
			- pH ISFET
			- Vol
			- Flw
			- EMG
			- Generic
			- Power Voltage 
			- Board's Current consumption

@author		Nicola Molinazzi
@date		18/01/2011
@version	01.10

@section history Revisions

From 01.00 to 01.10
- added general hardware channel
- modified the manage of the channel type propriety (m_chanType) with the HwChannel and within the HwChannelManager
- added the method inline bool popAdcData (T *adc, int num);

*/



#ifndef __CHANNELS_H__
#define __CHANNELS_H__

#include "global.h" 

#define DIM_FILT_BUF 		30					//30hz di campionamento, 30 campioni per buffer, il primo dato valido dopo 1sec poi uno ogni 33msec
#define DIM_FILT_BUF_OUT 	30
#define DIM_DGT_FILT 		9
#define DIM_FILT_BUF_STAT 	15					// 2 Hz
#define DIM_FILT_BUF_DIN 	2					// 15 Hz
#define DIM_FILT_MAX		30					// frequenza finale di 1Hz
#define DIM_FILT_MIN		1					// 30 Hz
#define DIM_FILT_STOP		DIM_FILT_BUF_STAT	// 2 Hz

#define _Fc_2Hz_		1	// media su 15 campioni
#define _Fc_15Hz_ 	2	// media su 2 campioni
#define _Fc_1Hz_		3	// media su 30 campioni
#define _Fc_30Hz_	4	// nessuna media, massima velocità di campionamento
#define _NO_FILT_	5	// esclusione filtro di media mobile e media fissa

/**
Acd bits resolution.
*/
typedef enum
{
	_8_BITS_RESOLUTION_ = 0, /**< 8 bits resolution - byte */
	_10_BITS_RESOLUTION_,	 /**< 8 bits resolution - word */
	_12_BITS_RESOLUTION_,	 /**< 8 bits resolution - word */
	_16_BITS_RESOLUTION_,    /**< 8 bits resolution - word */
	_24_BITS_RESOLUTION_,	 /**< 8 bits resolution - dword */
	_32_BITS_RESOLUTION_,	 /**< 8 bits resolution - dword */
		
}bitResolution;

/**
Analog channels type.
*/
typedef enum
{
	_HW_CHAN_NO_TYPE_ = 0,			/**< None */
	_HW_CHAN_VV_TYPE_,				/**< Voided volume */
	_HW_CHAN_POWER_VOLTAGE_TYPE_,	/**< power voltage */
	_HW_CHAN_GENERIC_TYPE_,			/**< Generic Hardware channel*/
	
}HwChannelType;

/**
Hardware channels errors.
*/
typedef enum
{
	_HW_CHAN_NO_ERROR_ = 0,			/**< None */
	_HW_CHAN_BUFFER_OVER_FLOW_,		/**< buffer overflow */
}HwChanKindOfError;

template <class T>
class AdcRingBuffer
{
	public:

	protected:

	private:

};
/**
Generic hardware channel implementation class.

It implements a general hardware channels with its main proprieties, as:
	- sample frequency
	- bit resolution
	- gain and offset
	- samples' buffer
	- ....
	
HwChannel is a template class were T is the of data sampled (byte, word or dword) 
and capacity the dimension of the samples's buffer. 

@param T type of data sampled (byte, word or dword). 
@param capacity dimension of the samples' buffer.
*/
template <class T, int capacity>
class HwChannel
{
	public:
		/**
		Class constructor. 
		@param bits adc bits resolution.
		*/
		HwChannel(bitResolution bits = _24_BITS_RESOLUTION_)
		{
		    m_fifoAdcData.clear();
			m_bitsResolution = bits;
			m_offset = 0;
			m_offsetToZero = 0;
			m_gain = 1.0;
			m_sampleFrq = 60;
			m_isZeroable = False;
			m_isEnabled = False;
			m_FiltEnabled = False;
			m_isTestenabled = False;
			m_adcVoltageReference_mV = 3300.0;
			m_hwChanError = _HW_CHAN_NO_ERROR_;
			m_lastAdcSampled = 0;
			m_errorsOccurence = 0;
			start_vol_av = False;
			med_buffer = NULL;
			switch (m_bitsResolution)
			{
				case _8_BITS_RESOLUTION_:
					m_adcResolution = 256.0;
					break;
				case _10_BITS_RESOLUTION_:
					m_adcResolution = 1024.0;
					break;
				case _12_BITS_RESOLUTION_:
					m_adcResolution = 4096.0;
					break;
				case _16_BITS_RESOLUTION_:	
					m_adcResolution = 65536.0;
					break;
				case _24_BITS_RESOLUTION_:	
					m_adcResolution = 16777216.0;
					break;
				case _32_BITS_RESOLUTION_:
					m_adcResolution = 4294967296.0;
					break;
			}
			m_chanType = _HW_CHAN_NO_TYPE_;
		};
		/**
		Class descructor.
		*/
		~HwChannel(){};

		/**
		@return the channel's sample frequency
		*/
		dword getSampleFrq(){ return m_sampleFrq;};
		
		/**
		Sets the channel sample frequency.
		@param frq channel's sample frequency.
		*/
		void setSampleFrq(dword frq){ m_sampleFrq = frq;};

		/**
		@return if the channel is zeroable
		*/
		bool getIsZeroable(){ return m_isZeroable;};
		/**
		Sets if the channel is zeroable accordint to z.
		@param z true set the channel as zeroable, false as NOT zeroable.
		@return the channel's sample frequency.
		*/
		void setIsZeroable(bool z){ m_isZeroable = z;};

		/**
		@return the channel's offset
		*/
		word getOffset(){ return m_offset;};//	long getOffset(){ return m_offset;};

		/**
		Sets the channel's offset.
		@param offset channel's offset
		*/
		void setOffset(word offset){ m_offset = offset;};		//void setOffset(long offset){ m_offset = offset;};

		/**
		@return the channel's gain
		*/
		float getGain(){ return m_gain;};
		/**
		Sets the channel's 2Kg calibration read value.
		@param gain channel's gain
		*/
		void set2KgValue(word value){ m_2Kgvalue = value;};	//void set2KgValue(dword value){ m_2Kgvalue = value;};
		/**
		@return the channel's 2Kg calibration read value
		*/
		word get2KgValue(){ return m_2Kgvalue;};	//dword get2KgValue(){ return m_2Kgvalue;};
		/**
		Sets the channel's gain.
		@param gain channel's gain
		*/
		void setGain(float gain){ m_gain = gain;};		
		/**
		@return channel's bits resolution
		*/
		bitResolution getBitResolution(){return m_bitsResolution;};

		/**
		@return the number of samples buffered.
		*/
		inline int getNumDataBuffered(){return m_fifoAdcData.numItem();};
		/**
		@return true if the samples buffer is empty, false otherwise.
		*/
		inline bool isSampleBufferEmpty(){return m_fifoAdcData.empty();};
		/**
		@return true if the samples buffer is full, false otherwise.
		*/
		bool isSampleBufferFull(){return m_fifoAdcData.full();};
		/**
		@param adc is the last data sampled - adc value.
		@return always 1.
		*/
		int getLastAdcDataSampled(T &adc){adc = m_lastAdcSampled; return 1;};		

		/**
		@param adc is the last data sampled - physical value.
		@return always 1.
		*/
		int getLastPhyDataSampled(float &data)
		{
			T tAdc;
			int i;
			i = getLastAdcDataSampled(tAdc);
			data = physicalValue(tAdc);
			return i;
		};

		/**
		Push a sample into the samples' buffer.
		@param adc value to push - adc value.
		@return 1 if the sample was correctly pushed, -1 otherwise. A sample can be not pushed for 2 reason or the channel 
				is disabled or the buffer is full (in this last case the error _HW_CHAN_BUFFER_OVER_FLOW_ is raised)
		@see isEnabled
		@see HwChanKindOfError
		*/
		inline int pushAdcData(T adc)
		{
			if (!m_fifoAdcData.full() && m_isEnabled)
			{
				m_lastAdcSampled = adc;
				m_fifoAdcData.push_back(adc);
				return 1;
			}
			if (m_isEnabled)
			{
				m_hwChanError = _HW_CHAN_BUFFER_OVER_FLOW_;
				m_errorsOccurence++;
			}
			return -1;
		};

		/**
		Push a sample into the samples' buffer, but first, if it is enabled mean filter function
		@ adc sample is sent to average filter algorithm. Filtered sample is sent to fifo only afther DIM_FILT_BUF sampling
		@param adc value to push - adc value.
		@return 1 if the sample was correctly pushed, -1 otherwise. A sample can be not pushed for 2 reason or the channel 
				is disabled or the buffer is full (in this last case the error _HW_CHAN_BUFFER_OVER_FLOW_ is raised)
		@see isEnabled
		@see HwChanKindOfError
		*/
		int pushFiltAdcData(T adc)
		{
			if (!m_fifoAdcData.full() && m_isEnabled)
			{
				if(isAvFilterEnabled() && (set_filtro > 0))
				{
					adc = AverageFilter(adc);	// protetta
					if( adc != 0 )
					{
						m_lastAdcSampled = adc;	//adc;
						m_fifoAdcData.push_back(adc);
					}
				}
				else {
					m_lastAdcSampled = adc;	//adc;
					m_fifoAdcData.push_back(adc);
				}
				return 1;
			}
			if (m_isEnabled)
			{
				m_hwChanError = _HW_CHAN_BUFFER_OVER_FLOW_;
				m_errorsOccurence++;
			}
			return -1;
		};
		/**
		Pop a sample from the samples' buffer.
		@param adc value popped - adc value.
		@return 1 if the sample was correctly popped, -1 if the buffer is empty.
		*/
		inline int popAdcData(T &adc)
		{
			if (!m_fifoAdcData.empty())
			{
				m_fifoAdcData.pop_front(adc);
				return 1;
			}
			return -1;
		}
		/**
		Clear the sample's buffer.
		*/
		void clearAdcFifo()
		{
			m_fifoAdcData.clear();
			m_hwChanError = _HW_CHAN_NO_ERROR_;
		};

		/**
		Returns the physical value of the channel according to gain, offset and offsetToZero.
		@param adc sample value - adc value
		@return physical value
		@see getGain
		@see setGain
		@see setOffset
		@see getOffset
		@see zero
		*/
		virtual float physicalValue(T adc){return (((long)adc - m_offset - m_offsetToZero)*m_gain);};
		
		/**
		channel zeroing.
		@return 1 if the channels is zeroable, -1 otherwise.
		@see setIsZeroable
		@see getIsZeroable
		*/
		int zero()
		{
			if (m_isZeroable)
			{
				m_offsetToZero = (long)m_lastAdcSampled - m_offset;
				return 1;
			}
			return -1;
		};

		/**
		Resets the zero value the channel.
		*/
		void rstZero(){m_offsetToZero = 0;};

		/**
		@return true if the channel is enable, false otherwise
		*/
		inline bool isEnabled(){ return m_isEnabled;};

		/**
		Enabled the channel according to e
		@param e true enable the channel, false to disable.
		*/
		void setEnabled(bool e){ m_isEnabled = e;};

		/**
		return true if the stability test is enable
		*/
		//inline bool isEnabledStabTest(){ return m_isTestenabled;};

		/**
		Enable the stability test according to e
		@param e true enable the channel, false to disable.
		*/
		//void setEnabledStabTest(bool e){ m_isTestenabled = e; };
		/**
		@return true if the moving average filter on channel adc value is enable, false otherwise
		*/
		inline bool isAvFilterEnabled(){ return m_FiltEnabled;};
		
		/**
		Enabled the moving average filter on adc value according to e	
		@param e true enable the filter, false to disable.
		*/
		bool setAverageFilter(bool e)
		{ 
		byte n;
			m_FiltEnabled = e;
			// filtro media mobile sul volume
			sum_filt_buf = 0;
			if (med_buffer == NULL )
				med_buffer = new dword [DIM_FILT_BUF];
			
			for(n = 0; n < DIM_FILT_BUF; n++)
				med_buffer[ n ] = 0; // memorizzo nel buffer
			index_buf_av = 0;
			previus_value = 0;
			//index_buf_av_out = 0;
		
			/*
			if (filt_buffer == NULL)
				filt_buffer = new float [DIM_DGT_FILT];
			for( n = 0; n < DIM_DGT_FILT; n++ )
				filt_buffer[ n ] = 0;
			somma_rampa = 0;
			for( n = 0; n < DIM_DGT_FILT; n++ )
				somma_rampa += rampa[ n ];
			*/
			
			return 1;
		};
		/**
		@return the type of channel
		@see HwChannelType
		*/
		HwChannelType getChannelType(){ return m_chanType;};

		/**
		@return adc resolution according to the bit resolution (i.e. if bit resolution is _8_BITS_RESOLUTION_ this funtion returns 256).
		@see bitResolution
		*/
		float getAdcResolution(){ return m_adcResolution;};

		/**
		@return the value of the adc voltage reference.
		*/
		float getVoltageRefence_mV(){return m_adcVoltageReference_mV;};
		/**
		Sets the value of the ad voltage reference
		@param mV value in mV of the voltage reference.
		*/
		void setVoltageRefernce_mV(float mV){ m_adcVoltageReference_mV = mV;};

		/**
		Returns the value in mV of the voltage sampled by the ADC.
		@param adc sampled value - adc value.
		@return mV sampled.
		*/
		float adcTOmV(T adc){return (float) (m_adcVoltageReference_mV / m_adcResolution)*(float) adc;};

		/**
		@return the error occured.
		@see HwChanKindOfError
		*/
		HwChanKindOfError kindOfError(){return m_hwChanError;};
		/**
		@return the number of error occurences.  
		*/
		dword getNumErrorOccurrence(){ return m_errorsOccurence;};	

		/**
		set the type of channel
		@type kind of channels.
		@see HwChannelType
		*/
		void setChannelType(HwChannelType type){ m_chanType = type;};
		/**
		set the second filtering level, 
		num of sample in the mean filter
		*/
		int set_filter_value(int m_value)
		{
			switch(m_value)
			{
				case 1:		//_Fc_2Hz_
					set_filtro = DIM_FILT_BUF_STAT;	// media su 15 campioni, 1Hz di frequenza finale, stato statico
					break;
				case 2: 		//_Fc_10Hz_
					set_filtro = DIM_FILT_BUF_DIN;	// media su 3 campioni, 5Hz di fr finale, stato dinamico
					break;
				case 3: 		// _Fc_1Hz_
					set_filtro = DIM_FILT_MAX;		// massimo filtraggio
					break;
				case 4:		//  _Fc_30Hz_
					set_filtro = DIM_FILT_MIN;		// nessun flitraggio oltre alla media mobile
					break;
				case 5:
					set_filtro = 0;					// di servizio, serve per disattivare i filtri
					break;
				default:
					set_filtro = DIM_FILT_BUF_STAT;	// media su 12 campioni, 5Hz di frequenza finale, stato statico
					break;
			}	
			return 1;
		};

		/**
		return the actual value of the second filtering leve
		*/
		int get_filter_value()
		{
			return set_filtro;		// quindi può valere DIM_FILT_BUF_STAT o DIM_FILT_BUF_DIN
		}
		
	protected:
		
		word AverageFilter(T& x)	// così dovrebbe uscire un dato filtrato sotto i 5Hz
		{
			unsigned short i;			
			dword currADCmin, currADCmax;
//			dword currADCmin2, currADCmax2;
			dword value_buff;
			
	#ifdef _AVERAGE_MOBILE_FILTER		
			if(start_vol_av)	 // the buffer is full
			{
				sum_filt_buf -= med_buffer[ index_buf_av]; 		// sottraggo dalla somma il valore del sample + vecchio
				med_buffer[ index_buf_av] = x;					// memorizzo il nuovo sample al posto del + vecchio
				currADCmin = currADCmax = med_buffer[0];
				for (i = 1; i < DIM_FILT_BUF; i++)				// cerco max e min di ogni finestra, compreso il nuovo
				{	    
					if( med_buffer[ i ] > currADCmax )
					{
						currADCmax = med_buffer[ i ];
					}
					if (med_buffer[ i ] < currADCmin)
					{
						currADCmin = med_buffer[ i ];
					}			
				}			 	
			 	sum_filt_buf += x;								// aggiungo alla somma il valore del nuovo sample 
			 	value_buff = sum_filt_buf - currADCmax - currADCmin;	// prima sottraggo min e max
			 	value_buff /= (DIM_FILT_BUF - 2);				// poi divido per il numero di campioni per ottenere la media di questa finestra (0-65535)	    		    	 
			    	index_buf_av++; 								// incremento contatone nel buffer		
			    	if (index_buf_av == DIM_FILT_BUF)				// se sono oltre il num max, lo rimetto a 0 (0<= index_buf_av < DIM_FILT_BUF)
			    		index_buf_av = 0;
			// CALCOLO MEDIA FISSA
				med_buffer_out += value_buff;					// somma valori finestre mobili, per calocalre media fissa
				index_buf_av_out++;							// corrisponde al numero di dati sommati
				if( index_buf_av_out >= set_filtro)				// se è uguale al valore richiesto di dati in media (freq di uscita)
				{
					old_value = value = (med_buffer_out / index_buf_av_out);	//old_value = value = ((value >> 8) & 0x0000FFFF);//  devo estrarre 16 bit
					med_buffer_out = 0;
					index_buf_av_out = 0;
				}
				else
					value = old_value;

			} 
			else 
			{ 	    	
				med_buffer[ index_buf_av] = x; 			// memorizzo nel buffer
				sum_filt_buf += x;		    
				old_value = value = x;
				index_buf_av++ ; 					// fill the buffer proceeding on the right
			    	if(index_buf_av == DIM_FILT_BUF)		// sono arrivato al max 
			    	{
			           	start_vol_av = True;   
			           	index_buf_av = 0;
					index_buf_av_out = 0;
					med_buffer_out = 0;
			    	}   
			} // fine media sul volume
	#endif
			
	#ifdef _FIX_AVERAGE_FILTER
		if( index_buf_av == DIM_FILT_BUF_OUT)
			{
				index_buf_av = 0;
				value_buff = 0;
				currADCmin = currADCmin2 = currADCmax = currADCmax2 = med_buffer[0];
				for (i = 1; i < DIM_FILT_BUF_OUT; i++)
				{	    
					if( med_buffer[ i ] > currADCmax )
					{
						currADCmax2 = currADCmax;
						currADCmax = med_buffer[ i ];
					}
					else if( med_buffer[ i ] > currADCmax2 )
							currADCmax2 = med_buffer[ i ];
					if (med_buffer[ i ] < currADCmin)
					{
						currADCmin2 = currADCmin;
						currADCmin = med_buffer[ i ];
					}
					else if (med_buffer[ i ] < currADCmin2)
							currADCmin2 = med_buffer[ i ];
					value_buff += med_buffer[i];
					med_buffer[i] = 0;
				}
				previus_value = (value_buff -currADCmax - currADCmin - currADCmax2 - currADCmin2) /( DIM_FILT_BUF_OUT-4);
				value = (previus_value & 0x0000FFFF);
				previus_value = value;
			}
			else
			{
				med_buffer[ index_buf_av] = x;			// memorizzo l'ultimo valore 
				index_buf_av++;
				value = previus_value;
				//value = 0;
			}
	#endif
			return value;
		}
		
		HwChannelType m_chanType;
		
	private:

		dword m_sampleFrq;
		word m_offset;	//long m_offset;
		long m_offsetToZero;
		float m_gain;
		word m_2Kgvalue;	//dword m_2Kgvalue;
		float m_adcResolution;
		float m_adcVoltageReference_mV;
		bool m_isZeroable;
		bool start_vol_av;
		float sum_dgt_filt;
		bitResolution m_bitsResolution;
		CSmallRingBuf <T, capacity> m_fifoAdcData;
		T m_lastAdcSampled;
		bool m_isEnabled;
		bool m_FiltEnabled;
		bool m_isTestenabled;
		HwChanKindOfError m_hwChanError;
		dword m_errorsOccurence;
		//unsigned short	  	value;
		dword value;
		dword old_value;
		dword 		*med_buffer;
		dword med_buffer_out;
		dword previus_value;
		unsigned short	index_buf_av; // indice nel buffer di filtraggio
		byte 	index_buf_av_out;
		dword        sum_filt_buf; 
		int set_filtro;
		//float	*filt_buffer;
		//int 		index_filt_dgt;
		//double *rampa;
		//int dim_rampa;
		//float somma_rampa;
};

/**
Generic hardware channel implementation class.

@param T type of data sampled (byte, word or dword). 
@param capacity dimension of the samples' buffer.
*/
template <class T, int capacity>
class GenericHwChannel: public HwChannel<T, capacity> 
{
	public:
		/**
		Class constructor. 
		@param type kind of channel @see HwChannelType
		@param bits adc bits resolution.
		*/
		GenericHwChannel(HwChannelType type = _HW_CHAN_GENERIC_TYPE_, bitResolution bits = _16_BITS_RESOLUTION_):
			HwChannel<T, capacity>(bits)
		{
			HwChannel<T, capacity>::m_chanType = type;
			HwChannel<T, capacity>::setSampleFrq(100);
			HwChannel<T, capacity>::setIsZeroable(False);
			HwChannel<T, capacity>::setOffset(0);
			HwChannel<T, capacity>::setGain(1);
			HwChannel<T, capacity>::setEnabled(False);
		};
			/**
			Class descructor.
			*/
		GenericHwChannel(){};

	protected:
		
	private:	
		
};


/**
Power voltage hardware channel implementation class.

@param T type of data sampled (byte, word or dword). 
@param capacity dimension of the samples' buffer.
*/
template <class T, int capacity>
class VoltageHwChannel: public HwChannel<T, capacity> 
{
	public:
		/**
		Class constructor. 
		@param bits adc bits resolution.
		*/
		VoltageHwChannel(bitResolution bits = _10_BITS_RESOLUTION_):
			HwChannel<T, capacity>(bits)
		{
			HwChannel<T, capacity>::m_chanType = _HW_CHAN_POWER_VOLTAGE_TYPE_;
			HwChannel<T, capacity>::setSampleFrq(1);
			HwChannel<T, capacity>::setIsZeroable(False);
			HwChannel<T, capacity>::setOffset(0);
			HwChannel<T, capacity>::setGain(1);
			HwChannel<T, capacity>::setEnabled(False);
		};
		/**
		Class destructor. 
		*/
		~VoltageHwChannel(){};

	protected:
		
	private:	
		
};


/**
Generic hardware channels manager class.

It implements a manager for generic hardware channels. 
This class does not allocate any  hardware channels so it can NOT be allocated any istances of it. 
It's used as base class for  more specific implementation.


HwChanManager is a template class were T is the of data sampled (byte, word or dword) 
and capacity the dimension of the samples's buffer. 

@param T type of data sampled (byte, word or dword). 
@param capacity dimension of the samples' buffer.
@see HwChannel
*/
template <class T, int capacity>
class HwChanManager
{
	public:
		/**
		Class constructor. 
		@param numChan number of hardware channels
		@param bits adc bits resolution.
		@param frq channels sample frequency.
		*/
		HwChanManager(int numChan, bitResolution bits = _16_BITS_RESOLUTION_, dword frq = 50)
		{
			m_hwChan = NULL;
			//m_hwChan = new HwChannel<T, capacity> *[numChan];
			m_numHwChan = numChan;
			m_numEnabledHwChan = 0;
			m_sampleFrq = frq;
			m_bitsResolution = bits;
			m_adcResolution = 1.0;
			m_voltageRefernce_mV = 3300;
			m_chanType = _HW_CHAN_NO_TYPE_;
		};
		/**
		Class destructor. 
		*/
		~HwChanManager(){};

		/**
		@return the channels' sample frequency 
		*/
		dword getSampleFrq(){ return m_sampleFrq;};

		/**
		@return the channels' bit resolution.
		@see bitResolution
		*/
		bitResolution getBitResolution(){ return m_bitsResolution;};

		/**
		@return adc resolution according to the bit resolution (i.e. if bit resolution is _8_BITS_RESOLUTION_ this funtion returns 256).
		@see bitResolution
		*/
		float getAdcResolution(){ return m_adcResolution;};

		/**
		@return the type of channels - pure virtual function.
		@see HwChannelType
		*/
		virtual HwChannelType getChannelType(){return _HW_CHAN_NO_TYPE_;};

		/**
		@return the value in mV of the adc voltage reference.
		*/
		float getVoltageRefence_mV(){ return m_voltageRefernce_mV;};

		/**
		@return the number of channels
		*/
		int getNumHwChan(){ return m_numHwChan;};

		/**
		Sets the i-th channel as zeroalble according to z
		@param i channel number
		@param true set the i-th channel as zeroable, false as NOT zeroable.
		*/
		void setIsZeroable(int i, bool z)
		{
			if (i < 0 || i >= m_numHwChan)
			{
				return;
			}
			m_hwChan[i]->setIsZeroable(z);
		};
		/**
		Returns if the i-th channel is zeroable
		@param i channel number
		@return true if the i-th channel is zeroable, false otherwise.
		*/
		bool getIsZeroable(int i)
		{
			if (i < 0 || i >= m_numHwChan)
			{
				return False;
			}
			return m_hwChan[i]->getIsZeroable();
		};
		/**
		Returns the i-th channel's offset
		@param i channel number
		@return i-th offset
		*/
		word getOffset(int i)	//long getOffset(int i)
		{
			if (i < 0 || i >= m_numHwChan)
			{
				return 0;
			}
			return m_hwChan[i]->getOffset();
		};
		/**
		Sets the offset of the i-th channel
		@param i channel number
		@param offset i-th channel offset
		*/
		void setOffset(int i, word offset)	//void setOffset(int i, long offset)
		{
			if (i < 0 || i >= m_numHwChan)
			{
				return;
			}
			m_hwChan[i]->setOffset(offset);
		};

		/**
		Returns the i-th channel's gain
		@param i channel number
		@return i-th gain
		*/
		float getGain(int i)
		{
			if (i < 0 || i >= m_numHwChan)
			{
				return 1.0;
			}
			return m_hwChan[i]->getGain();
		};

		/**
		Sets the gain of the i-th channel
		@param i channel number
		@param offset i-th channel offset
		*/
		void setGain(int i, float gain)
		{
			if (i < 0 || i >= m_numHwChan)
			{
				return;
			}
			m_hwChan[i]->setGain(gain);
		};
		/**
		Sets the 2Kg value (average value) of the i-th channel
		@param i channel number
		@param offset i-th channel offset
		*/
		void set2KgValue(int i, long value)
		{
			if (i < 0 || i >= m_numHwChan)
			{
				return;
			}
			m_hwChan[i]->set2KgValue(value);
		}

		/**
		returns the 2Kg value (average value) of the i-th channel
		@param i channel number
		@param offset i-th channel offset
		*/
		word get2KgValue(int i)
		{
			if (i < 0 || i >= m_numHwChan)
			{
				return 1.0;
			}
			return m_hwChan[i]->get2KgValue();
		}
		/**
		Returns the i-th channel's physical value
		@param i channel number
		@param adc sample value - adc value
		@return i-th channel physical value
		*/
		float physicalValue(int i, T adc)
		{
			if (i < 0 || i >= m_numHwChan)
			{
				return 0.0;
			}
			return m_hwChan[i]->physicalValue(adc);
		};
		/**
		Returns the value in mV of the voltage sampled by the ADC.
		@param i channel number
		@param adc sampled value - adc value.
		@return mV sampled.
		*/
		float adcTOmV(int i, T adc)
        {
            if (i < 0 || i >= m_numHwChan)
            {
                return 0.0;
            }
            return (float) m_hwChan[i]->adcTOmV(adc);
        };

		/**
		i-th channel zeroing.
		@return 1 if the channels is zeroable, -1 otherwise.
		@param i channel number
		@see setIsZeroable
		@see getIsZeroable
		*/
		int zero(int i)
		{
			if (i < 0 || i >= m_numHwChan)
			{
				return -1;
			}
			return m_hwChan[i]->zero();
		};
		/**
		Resets the zero value of the i-th channel
		*/
		void rstZero(int i)
		{
			if (i < 0 || i >= m_numHwChan)
			{
				return;
			}
			m_hwChan[i]->rstZero();
		};

		/**
		@return true if the i-th channel is enable, false otherwise
		@see setEnabled
		*/
		bool isEnabled(int i)
		{
			if (i < 0 || i >= m_numHwChan)
			{
				return False;
			}
			return m_hwChan[i]->isEnabled();
		};
		/**
		Sets the i-th channel enabled accordint to e value
		@param i channel number
		@param e if true se the i-th channel enabled, disabled otherwise.
		@see isEnabled
		*/
		void setEnabled(int i, bool e)
		{
			if (i < 0 || i >= m_numHwChan)
			{
				return;
			}
			m_hwChan[i]->setEnabled(e);
		};
		/**
		@return true if the i-th channel moving average filter is enable, false otherwise
		@see setEnabled
		*/
		bool isAvFilterEnabled(int i)
		{
			if (i < 0 || i >= m_numHwChan)
			{
				return False;
			}
			return m_hwChan[i]->isAvFilterEnabled();
		};
		/**
		Sets the i-th channel moving Average Filter enabled accordint to e value
		@param i channel number
		@param e if true se the i-th channel filter enabled, disabled otherwise.
		@see isEnabled
		*/
		void setAverageFilter(int i, bool e)
		{
			if (i < 0 || i >= m_numHwChan)
			{
				return;
			}
			m_hwChan[i]->setAverageFilter(e);
		}

		/**
		Sets the ADC voltage reference
		@param mV adc voltage reference value in mV 
		*/
		void setVoltageRefernce_mV(float mV)
		{
			int i;
			for (i = 0; i < m_numHwChan; i++)
			{
				m_hwChan[i]->setVoltageRefernce_mV(mV);
			}		
		};

		/**
		return true if the stability test is enable
		*/
		/*
		bool isEnabledStabTest(int i)
		{ 
			if (i < 0 || i >= m_numHwChan)
			{
				return false;
			}
			return m_hwChan[i]->isEnabledStabTest();
		}
		*/
		/**
		Enable the stability test according to e
		@param e true enable the channel, false to disable.
		*/
		/*
		void setEnabledStabTest(int i, bool e)
		{ 
			if (i < 0 || i >= m_numHwChan)
			{
				return;
			}
			m_hwChan[i]->setEnabledStabTest(e);
		}
		*/
		/**
		Push a sample into the i-th samples' buffer.
		@param i channel number
		@param adc value to push - adc value.
		@return 1 if the sample was correctly pushed, -1 otherwise. A sample can be not pushed for 2 reason or the channel 
				is disabled or the buffer is full (in this last case the error _HW_CHAN_BUFFER_OVER_FLOW_ is raised)
		@see isEnabled
		@see HwChanKindOfError
		*/
		inline int pushAdcDataToChan(int i, T adc)
		{
			if (i < 0 || i >= m_numHwChan)
			{
				return -1;
			}
			return m_hwChan[i]->pushAdcData(adc);
		};
		/**
		Push a sample into the i-th samples' buffer and if it is set, run average filter.
		@param i channel number
		@param adc value to push - adc value.
		@return 1 if the sample was correctly pushed, -1 otherwise. A sample can be not pushed for 2 reason or the channel 
				is disabled or the buffer is full (in this last case the error _HW_CHAN_BUFFER_OVER_FLOW_ is raised)
		@see isEnabled
		@see HwChanKindOfError
		*/
		inline int pushFiltAdcDataToChan(int i, T adc)
		{
			if (i < 0 || i >= m_numHwChan)
			{
				return -1;
			}
			return m_hwChan[i]->pushFiltAdcData(adc); // invia alla funzone che ( se attivo) esegue il filtro e poi archivia
		};

		/**
		Returns the last value sampled from channel i.
		@param i channel number
		@param adc is the last data sampled - adc value.
		@return 1 if i is comprised between 0 and m_numHwChan, -1 otherwise
		*/
		int getLastAdcDataSampledByChan(int i, T &adc)
		{
			if (i < 0 || i >= m_numHwChan)
			{
				return -1;
			}
			return m_hwChan[i]->getLastAdcDataSampled(adc);
		};
		/**
		Returns the last physical value sampled from channel i.
		@param i channel number
		@param adc is the last data sampled - physical value.
		@return 1 if i is comprised between 0 and numChan, -1 otherwise
		*/
		int getLastPhyDataSampledByChan(int i, float &data)
		{
			if (i < 0 || i >= m_numHwChan)
			{
				return -1;
			}
			return m_hwChan[i]->getLastPhyDataSampled(data);
		};

		/**
		Pop samples from the samples' buffers if at least one sample is present in each buffers.
		@param adc pointer to and array of data whose dimension are m_numHwChan.
		@return true if at least num samplea are present in each buffer, false otherwise.
		*/
		inline bool popAdcData (T *adc)
		{
			int i;
			bool readValues = False;
			bool bufferEmpty;

			for(i = 0; i < m_numHwChan; i++)
			{
				if (m_hwChan[i]->isEnabled())
				{
					asm("di");
					bufferEmpty = m_hwChan[i]->isSampleBufferEmpty();
					asm("ei");
					if (bufferEmpty)
					{
						readValues = False;
						break;
					}else
					{
						readValues = True;
					}
				}
			}
			if (readValues)
			{
				for(i = 0; i < m_numHwChan; i++)
				{
					if (m_hwChan[i]->isEnabled())
					{
						asm("di");
						m_hwChan[i]->popAdcData(adc[i]);
						asm("ei");
					}else
					{
						adc[i] = 0;
					}
				}
			}
			
			return readValues;
		};

		/**
		Pops samples from the samples' buffers if at least n samples are present in each buffers.

		The adc array contains first all the num samples of the first channel, then all the samples of second and so on.
		@param adc pointer to and array of data whose dimension are m_numHwChan.
		@param num number of samples that have to present in each buffer
		@return true if at least one sample is present in each buffer, false otherwise.
		*/
		inline bool popAdcData (T *adc, int num)
		{
			int i, k;
			bool readValues = False;
			int bufferData;

			for(i = 0; i < m_numHwChan; i++)
			{
				if (m_hwChan[i]->isEnabled())
				{
					asm("di");
					bufferData = m_hwChan[i]->getNumDataBuffered();
					asm("ei");
					if (bufferData < num)
					{
						readValues = False;
						break;
					}else
					{
						readValues = True;
					}
				}
			}
			if (readValues)
			{
				for(i = 0; i < m_numHwChan; i++)
				{
					if (m_hwChan[i]->isEnabled())
					{
						for(k = 0; k < num; i++)
						{
							asm("di");
							m_hwChan[i]->popAdcData(adc[i]);
							asm("ei");
						}
					}else
					{
						adc[i] = 0;
					}
				}
			}
			
			
			return readValues;
		};
		
		/**
		Clears all the samples's buffers.
		*/
		void clearFifo()
		{
			int i;
			asm("di");
			for (i = 0; i < m_numHwChan; i++)
			{
				m_hwChan[i]->clearAdcFifo();
			}
			asm("ei");
		};
		/**
		Returns the number of samples present in the i-th buffer
		@param i channel number
		@return the number of samples buffered from the i-th channel.
		*/
		int getNumDataFifoed(int i){return m_hwChan[i]->getNumDataBuffered();};
		/**
		Returns if the i-th bufffer is empty
		@param i channel number
		@return true is the i-th buffer is empty, false othewise
		*/
		bool isFifoEmpty(int i){return m_hwChan[i]->isSampleBufferEmpty();};
		/**
		Returns if the i-th bufffer is full
		@param i channel number
		@return true is the i-th buffer is full, false othewise
		*/
		bool isFifoFull(int i){return m_hwChan[i]->isSampleBufferFull();};

		/**
		Returns the error of the hardware channels
		@param errorType pointer to and array of data whose dimension are m_numHwChan. This array contains the error for each channel
		@return true if an error occured at least in one channel, false if no errors occurred in any channel.
		@see HwChanKindOfError
		*/
		bool HwChanManagerError(HwChanKindOfError *errorType)
		{
			int i;
			bool retVal = False;
			for (i = 0; i < m_numHwChan; i++)
			{
				errorType[i] = m_hwChan[i]->kindOfError();
				if (errorType[i] != _HW_CHAN_NO_ERROR_)
				{
					retVal = True;
				}
			}
			return retVal;
		};
		/**
		Returns the error of the hardware channels
		@return true if an error occured at least in one channel, false if no errors occurred in any channel.
		*/
		bool HwChanManagerError()
		{
			int i;
			HwChanKindOfError errorType;
			for (i = 0; i < m_numHwChan; i++)
			{
				errorType = m_hwChan[i]->kindOfError();
				if (errorType != _HW_CHAN_NO_ERROR_)
				{
					return True;
				}
			}
			return False;
		};

		/**
		Returns the number of errors' occurrences in the i-th channel
		@param i channel number
		@return number of errors' occurrences in the i-th channel
		*/
		dword getNumErrorOccurrence(int i)
		{
			if (i < 0 || i >= m_numHwChan)
			{
				return -1;
			}
			return m_hwChan[i]->getNumErrorOccurrence();
		};

		/**
		...
		*/
		int set_rampa( int i, float *m_rampa, int dimrampa)
		{
			if (i < 0 || i >= m_numHwChan)
			{
				return -1;
			}
			m_hwChan[i]->set_rampa(m_rampa, dimrampa);
			return 1;
		};

		int set_sommarampa( int i, float m_sommarampa)
		{
			if (i < 0 || i >= m_numHwChan)
			{
				return -1;
			}
			m_hwChan[i]->set_sommarampa(m_sommarampa);
			return 1;
		};

		int set_filter_value(int i, int m_value)
		{
			if(m_value > 6)
				m_value = 6;
			m_hwChan[i]->set_filter_value(m_value);
			return 1;
		};

		int get_filter_value(int i)
		{
			return m_hwChan[i]->get_filter_value();
		}
		
	protected:
		HwChannelType m_chanType;
		HwChannel<T, capacity> **m_hwChan;
		
		/**
		Attribute than holds the number of channels.
		*/
		word m_numHwChan;
		word m_numEnabledHwChan;
		float m_adcResolution;
		float m_voltageRefernce_mV;
		
	private:	
		dword m_sampleFrq;
		bitResolution m_bitsResolution;
		
};

/**
Generic hardware channels manager class.

@param T type of data sampled (byte, word or dword). 
@param capacity dimension of the samples' buffer.
@see HwChannel
*/
template <class T, int capacity>
class GenericHwChanManager: public HwChanManager<T, capacity> 
{
	public:
		/**
		Class constructor. 
		@param type kind of channel @see HwChannelType
		@param numChan number of hardware channels
		@param bits adc bits resolution.
		@param frq channels sample frequency.
		*/
		GenericHwChanManager(int numChan, HwChannelType type, bitResolution bits = _16_BITS_RESOLUTION_, dword frq = 100):
			HwChanManager<T, capacity>(numChan, bits, frq)
		{
			int i;
			if (numChan < 1)
				numChan = 1;
			HwChanManager<T, capacity>::m_hwChan = (HwChannel<T, capacity>**)new GenericHwChannel<T, capacity> *[numChan];
			for (i = 0; i < numChan; i++)
			{
				HwChanManager<T, capacity>::m_hwChan[i] = (HwChannel<T, capacity>*)new GenericHwChannel<T, capacity>(type, bits);
				HwChanManager<T, capacity>::m_hwChan[i]->setSampleFrq(frq);
				HwChanManager<T, capacity>::m_hwChan[i]->setVoltageRefernce_mV(HwChanManager<T, capacity>::m_voltageRefernce_mV);
				HwChanManager<T, capacity>::m_adcResolution = HwChanManager<T, capacity>::m_hwChan[i]->getAdcResolution();
			}
		};
		/**
		Class destructor. 
		*/	
		~GenericHwChanManager()
		{
			int i;
			if ( HwChanManager<T, capacity>::m_hwChan)
			{
				for (i = 0; i < HwChanManager<T, capacity>::m_numHwChan; i++)
				{
					delete HwChanManager<T, capacity>::m_hwChan[i];
				}
				delete HwChanManager<T, capacity>::m_hwChan;
				HwChanManager<T, capacity>::m_hwChan = NULL;
			}
		};

		/**
		sets the type of the i-th channels
		@param i channel number
		@param type kind of channel @see HwChannelType
		*/
		void setChanType(int i, HwChannelType type)
		{
			if(i >= 0 || i < HwChanManager<T, capacity>::m_numHwChan)
			{
				((GenericHwChannel<T, capacity>*)(HwChanManager<T, capacity>::m_hwChan[0]))->setChannelType(type);
			}
		}

		/**
		@return the type of channels. if i less or equal than 0 or i bigger than m_numHwChan, return HwChannelType::_HW_CHAN_NO_TYPE_
		@see HwChannelType
		*/
		HwChannelType getChannelType(int i)
		{
			if(i >= 0 || i < HwChanManager<T, capacity>::m_numHwChan)
			{
				return ((GenericHwChannel<T, capacity>*)(HwChanManager<T, capacity>::m_hwChan[0]))->getChannelType();
			}
			return _HW_CHAN_NO_TYPE_;
		}
	protected:

	private:
		
};

/**
Power Voltage hardware channels manager class.

@param T type of data sampled (byte, word or dword). 
@param capacity dimension of the samples' buffer.
@see HwChannel
*/
template <class T, int capacity>
class VoltageHwChanManager: public HwChanManager<T, capacity> 
{
	public:
		/**
		Class constructor. 
		@param numChan number of hardware channels
		@param bits adc bits resolution.
		@param frq channels sample frequency.
		*/
		VoltageHwChanManager(int numChan, bitResolution bits = _10_BITS_RESOLUTION_, dword frq = 1):
			HwChanManager<T, capacity>(numChan, bits, frq)
		{
			int i;
			if (numChan < 1)
				numChan = 1;
			HwChanManager<T, capacity>::m_hwChan = (HwChannel<T, capacity>**)new VoltageHwChannel<T, capacity> *[numChan];
			for (i = 0; i < numChan; i++)
			{
				HwChanManager<T, capacity>::m_hwChan[i] = (HwChannel<T, capacity>*)new VoltageHwChannel<T, capacity>(bits);
				HwChanManager<T, capacity>::m_hwChan[i]->setSampleFrq(frq);
				HwChanManager<T, capacity>::m_hwChan[i]->setVoltageRefernce_mV(HwChanManager<T, capacity>::m_voltageRefernce_mV);
				HwChanManager<T, capacity>::m_adcResolution = HwChanManager<T, capacity>::m_hwChan[i]->getAdcResolution();
			}
		};
		/**
		Class destructor. 
		*/	
		~VoltageHwChanManager()
		{
			int i;
			if ( HwChanManager<T, capacity>::m_hwChan)
			{
				for (i = 0; i < HwChanManager<T, capacity>::m_numHwChan; i++)
				{
					delete HwChanManager<T, capacity>::m_hwChan[i];
				}
				delete HwChanManager<T, capacity>::m_hwChan;
				HwChanManager<T, capacity>::m_hwChan = NULL;
			}
		};
		
		/**
		@return the type of channels.
		@see HwChannelType
		*/
		HwChannelType getChannelType(){return ((VoltageHwChannel<T, capacity>*)(HwChanManager<T, capacity>::m_hwChan[0]))->getChannelType();};
	protected:

	private:
		
};


extern GenericHwChanManager <dword, _WEIGHT_BUFFER_LENGTH_> *weightChan;
extern GenericHwChanManager <dword, _GENERIC_BUFFER_LENGTH_> *genericChan;
extern VoltageHwChanManager <word,_PWR_VOLTAGE_BUFFER_LENGTH_> *powerVoltageChan;

#endif
























































