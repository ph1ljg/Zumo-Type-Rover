/* 
* CNotchFilter.h
*
* Created: 19/03/2021 18:40:39
* Author: philg
*/


#ifndef __CNOTCHFILTER_H__
#define __CNOTCHFILTER_H__

typedef struct  
{
	int8_t _enable;
	float CenterFreqHz;
	float BandwidthHz;
	float Attenuation_dB;
}NotchParam_t;


template <class T>
class CNotchFilter
{
//variables
public:
	NotchParam_t NotchParams;
protected:
private:
    bool initialised;
    float b0, b1, b2, a1, a2, a0_inv;
    T ntchsig, ntchsig1, ntchsig2, signal2, signal1;
//functions
public:
	CNotchFilter();
//	~CNotchFilter();
	// set parameters
	void init(float sample_freq_hz, float center_freq_hz, float bandwidth_hz, float attenuation_dB);
	void init_with_A_and_Q(float sample_freq_hz, float center_freq_hz, float A, float Q);
	T apply(const T &sample);
	void reset();

	// calculate attenuation and quality from provided center frequency and bandwidth
	static void calculate_A_and_Q(float center_freq_hz, float bandwidth_hz, float attenuation_dB, float& A, float& Q);

protected:
private:
	CNotchFilter( const CNotchFilter &c );
	CNotchFilter& operator=( const CNotchFilter &c );

}; //CNotchFilter


typedef CNotchFilter<float> NotchFilterFloat;
typedef CNotchFilter<Vector3f> NotchFilterVector3f;




#endif //__CNOTCHFILTER_H__
