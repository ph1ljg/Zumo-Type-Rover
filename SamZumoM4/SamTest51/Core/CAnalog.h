/*
* CAnalog.h
*
* Created: 19/05/2020 19:20:29
* Author: philg
*/


#ifndef __CANALOG_H__
#define __CANALOG_H__

#define ADC_RESOLUTION		12

// SAMD products have only one reference for ADC
// add internal voltages for ATSAMD51 SUPC VREF register
typedef enum _eAnalogReference
{
	AR_DEFAULT,
	AR_INTERNAL1V0,
	AR_INTERNAL1V1,
	AR_INTERNAL1V2,
	AR_INTERNAL1V25,
	AR_INTERNAL2V0,
	AR_INTERNAL2V2,
	AR_INTERNAL2V23,
	AR_INTERNAL2V4,
	AR_INTERNAL2V5,
	AR_INTERNAL1V65,
	AR_EXTERNAL
} eAnalogReference ;



class CAnalog
{
	//variables
	public:
	bool m_DacEnabled[2];
	int _readResolution		= 10;
	int _ADCResolution		= 10;
	int _writeResolution	= 12;
	int _dacResolution		= 12;


	protected:
	private:

	//functions
	public:
	CAnalog();
	~CAnalog();
	void Init();
	void InitDAC();
	void AnalogReference(eAnalogReference mode);
	float AnalogReadVolts(Adc *adc,EPortType port,uint32_t PortBitNo);
	uint32_t AnalogRead(Adc *adc,EPortType port,uint32_t pin);
	void DisableDAC(uint8_t channel);
	void AnalogWrite(uint32_t pin, uint32_t value);
	inline uint32_t mapResolution(uint32_t value, uint32_t from, uint32_t to);
	void AnalogReadCorrection (Adc *adc,int offset, uint16_t gain);
	protected:
	private:
	CAnalog( const CAnalog &c );
	CAnalog& operator=( const CAnalog &c );

}; //CAnalog

#endif //__CANALOG_H__
