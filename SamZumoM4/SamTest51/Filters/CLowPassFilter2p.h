/* 
* CLowpassFilter2p.h
*
* Created: 19/03/2021 12:45:43
* Author: philg
*/


#ifndef __CLOWPASSFILTER2P_H__
#define __CLOWPASSFILTER2P_H__
#include "Includes.h"
/// @file   LowPassFilter2p.h
/// @brief  A class to implement a second order low pass filter
/// @authors: Leonard Hall <LeonardTHall@gmail.com>, template implmentation: Daniel Frenzel <dgdanielf@gmail.com>
template <class T>
class DigitalBiquadFilter 
{
	public:
	struct biquad_params 
	{
		float cutoff_freq;
		float sample_freq;
		float a1;
		float a2;
		float b0;
		float b1;
		float b2;
	};
	
	DigitalBiquadFilter();

	T apply(const T &sample, const struct biquad_params &params);
	void reset();
	void reset(const T &value, const struct biquad_params &params);
	static void compute_params(float sample_freq, float cutoff_freq, biquad_params &ret);
	
	private:
	T _delay_element_1;
	T _delay_element_2;
	bool initialised;
};

template <class T>
class CLowPassFilter2p
{
//variables
public:
protected:
	struct DigitalBiquadFilter<T>::biquad_params _params;
private:
    DigitalBiquadFilter<T> _filter;
//functions
public:
	CLowPassFilter2p();
//	~CLowPassFilter2p();
	CLowPassFilter2p(float sample_freq, float cutoff_freq);
	// change parameters
	void set_cutoff_frequency(float sample_freq, float cutoff_freq);
	// return the cutoff frequency
	float get_cutoff_freq(void) const;
	float get_sample_freq(void) const;
	T apply(const T &sample);
	void reset(void);
	void reset(const T &value);

protected:
private:
	CLowPassFilter2p( const CLowPassFilter2p &c );
	CLowPassFilter2p& operator=( const CLowPassFilter2p &c );

}; //CLowpassFilter2p

typedef CLowPassFilter2p<int>      LowPassFilter2pInt;
typedef CLowPassFilter2p<long>     LowPassFilter2pLong;
typedef CLowPassFilter2p<float>    LowPassFilter2pFloat;
typedef CLowPassFilter2p<Vector2f> LowPassFilter2pVector2f;
typedef CLowPassFilter2p<Vector3f> LowPassFilter2pVector3f;



#endif //__CLOWPASSFILTER2P_H__
