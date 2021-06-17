/* 
* CLowPassFilter.h
*
* Created: 19/03/2021 14:52:09
* Author: philg
*/


#ifndef __CLOWPASSFILTER_H__
#define __CLOWPASSFILTER_H__
#include "Includes.h"
/// @file	LowPassFilter.h
/// @brief	A class to implement a low pass filter without losing precision even for int types
///         the downside being that it's a little slower as it internally uses a float
///         and it consumes an extra 4 bytes of memory to hold the constant gain

/*
  Note that this filter can be used in 2 ways:

   1) providing dt on every sample, and calling apply like this:

      // call once
      filter.set_cutoff_frequency(frequency_hz);

      // then on each sample
      output = filter.apply(sample, dt);

   2) providing a sample freq and cutoff_freq once at start

      // call once
      filter.set_cutoff_frequency(sample_freq, frequency_hz);

      // then on each sample
      output = filter.apply(sample);

  The second approach is more CPU efficient as it doesn't have to
  recalculate alpha each time, but it assumes that dt is constant
 */

// DigitalLPF implements the filter math
template <class T>
class DigitalLPF 
{
	public:
	DigitalLPF();
	// add a new raw value to the filter, retrieve the filtered result
	T apply(const T &sample, float cutoff_freq, float dt);
	T apply(const T &sample);

	void compute_alpha(float sample_freq, float cutoff_freq);
	
	// get latest filtered value from filter (equal to the value returned by latest call to apply method)
	const T &get() const;
	void reset(T value);

	private:
	T _output;
	float alpha = 1.0f;
};


template <class T>
class CLowPassFilter
{
//variables
public:
protected:
    float _cutoff_freq;
private:
    DigitalLPF<T> _filter;
//functions
public:
	CLowPassFilter();
	CLowPassFilter(float cutoff_freq);
	CLowPassFilter(float sample_freq, float cutoff_freq);

	// change parameters
	void set_cutoff_frequency(float cutoff_freq);
	void set_cutoff_frequency(float sample_freq, float cutoff_freq);

	// return the cutoff frequency
	float get_cutoff_freq(void) const;
	T apply(T sample, float dt);
	T apply(T sample);
	const T &get() const;
	void reset(T value);
	void reset(void) { reset(T()); }

protected:
private:
	CLowPassFilter( const CLowPassFilter &c );
	CLowPassFilter& operator=( const CLowPassFilter &c );

}; //CLowPassFilter

// typedefs for compatibility
typedef CLowPassFilter<int>      LowPassFilterInt;
typedef CLowPassFilter<long>     LowPassFilterLong;
typedef CLowPassFilter<float>    LowPassFilterFloat;
typedef CLowPassFilter<Vector2f> LowPassFilterVector2f;
typedef CLowPassFilter<Vector3f> LowPassFilterVector3f;



#endif //__CLOWPASSFILTER_H__
