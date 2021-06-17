/* 
* CHarmonicNotchFilter.h
*
* Created: 20/03/2021 09:31:40
* Author: philg
*/


#ifndef __CHARMONICNOTCHFILTER_H__
#define __CHARMONICNOTCHFILTER_H__
#include "CNotchFilter.h"

#define HNF_MAX_HARMONICS 8
#define HNF_MAX_HMNC_BITSET 0xF
#define HNF_MAX_FILTERS 6 // must be even for double-notch filters
#define HNF_MAX_HARMONICS 8



typedef struct
{
	uint8_t _enable;
	float CenterFreqHz[4];
	float BandwidthHz;
	float Attenuation_dB;
	int8_t _harmonics;		// configured notch harmonics
	float _reference;
	int8_t _tracking_mode;	// notch dynamic tracking mode
	int16_t _options;	// notch options

}HarmonicParam_t;

//  A filter that manages a set of notch filters targeted at a fundamental center frequency
//   and multiples of that fundamental frequency
template <class T>
class CHarmonicNotchFilter
{
//variables
public:
	HarmonicParam_t Params;
protected:
private:
    // underlying bank of notch filters
    CNotchFilter<T>*  _filters;
    // sample frequency for each filter
    float _sample_freq_hz;
    // base double notch bandwidth for each filter
    float _notch_spread;
    // attenuation for each filter
    float _A;
    // quality factor of each filter
    float _Q;
    // a bitmask of the harmonics to use
    uint8_t _harmonics;
    // whether to use double-notches
    bool _double_notch;
    // number of allocated filters
    uint8_t _num_filters;
    // number of enabled filters
    uint8_t _num_enabled_filters;
    bool _initialised;

//functions
public:
	CHarmonicNotchFilter();
	~CHarmonicNotchFilter();
	// allocate a bank of notch filters for this harmonic notch filter
	void allocate_filters(uint8_t harmonics, bool double_notch);
	// initialize the underlying filters using the provided filter parameters
	void init(float sample_freq_hz, float center_freq_hz, float bandwidth_hz, float attenuation_dB);
	// update the underlying filters' center frequencies using center_freq_hz as the fundamental
	void update(float center_freq_hz);
	// update all of the underlying center frequencies individually
	void update(uint8_t num_centers,  float center_freq_hz[]);
	// apply a sample to each of the underlying filters in turn
	T apply(const T &sample);
	// reset each of the underlying filters
	void reset();

protected:
private:
	CHarmonicNotchFilter( const CHarmonicNotchFilter &c );
	CHarmonicNotchFilter& operator=( const CHarmonicNotchFilter &c );

}; //CHarmonicNotchFilter

typedef CHarmonicNotchFilter<Vector3f> HarmonicNotchFilterVector3f;

#endif //__CHARMONICNOTCHFILTER_H__
