/* 
* CLowPassFilter.cpp
*
* Created: 19/03/2021 14:52:09
* Author: philg
*/

#include "Includes.h"
#include "CLowPassFilter.h"

////////////////////////////////////////////////////////////////////////////////////////////
// DigitalLPF
////////////////////////////////////////////////////////////////////////////////////////////

template <class T>
DigitalLPF<T>::DigitalLPF() 
{
  // built in initialization
  _output = T();
}

// add a new raw value to the filter, retrieve the filtered result
template <class T>
T DigitalLPF<T>::apply(const T &sample, float cutoff_freq, float dt) 
{
   CMyMath Math;
    if (cutoff_freq <= 0.0f || dt <= 0.0f) 
	{
        _output = sample;
        return _output;
    }
    float rc = 1.0f/(M_2PI*cutoff_freq);
    alpha = Math.constrain_float(dt/(dt+rc), 0.0f, 1.0f);
    _output += (sample - _output) * alpha;
    return _output;
}

template <class T>
T DigitalLPF<T>::apply(const T &sample) 
{
    _output += (sample - _output) * alpha;
    return _output;
}

template <class T>
void DigitalLPF<T>::compute_alpha(float sample_freq, float cutoff_freq) 
{	
	CMyMath Math;
    if (sample_freq <= 0) 
        alpha = 1;
	else
        alpha = Math.calc_lowpass_alpha_dt(1.0/sample_freq, cutoff_freq);
}

// get latest filtered value from filter (equal to the value returned by latest call to apply method)
template <class T>
const T &DigitalLPF<T>::get() const 
{
    return _output;
}

template <class T>
void DigitalLPF<T>::reset(T value) 
{ 
    _output = value; 
}
    
////////////////////////////////////////////////////////////////////////////////////////////
// LowPassFilter
////////////////////////////////////////////////////////////////////////////////////////////

// constructors
template <class T>
CLowPassFilter<T>::CLowPassFilter() :
    _cutoff_freq(0.0f) {}

template <class T>
CLowPassFilter<T>::CLowPassFilter(float cutoff_freq) :
    _cutoff_freq(cutoff_freq) {}

template <class T>
CLowPassFilter<T>::CLowPassFilter(float sample_freq, float cutoff_freq)
{
    set_cutoff_frequency(sample_freq, cutoff_freq);
}

// change parameters
template <class T>
void CLowPassFilter<T>::set_cutoff_frequency(float cutoff_freq) 
{
    _cutoff_freq = cutoff_freq;
}

template <class T>
void CLowPassFilter<T>::set_cutoff_frequency(float sample_freq, float cutoff_freq) 
{
    _cutoff_freq = cutoff_freq;
    _filter.compute_alpha(sample_freq, cutoff_freq);
}

// return the cutoff frequency
template <class T>
float CLowPassFilter<T>::get_cutoff_freq(void) const 
{
    return _cutoff_freq;
}

template <class T>
T CLowPassFilter<T>::apply(T sample, float dt) 
{
    return _filter.apply(sample, _cutoff_freq, dt);
}

template <class T>
T CLowPassFilter<T>::apply(T sample) 
{
    return _filter.apply(sample);
}

template <class T>
const T &CLowPassFilter<T>::get() const 
{
    return _filter.get();
}

template <class T>
void CLowPassFilter<T>::reset(T value) 
{
    _filter.reset(value);
}

/* 
 * Make an instances
 * Otherwise we have to move the constructor implementations to the header file :P
 */
template class CLowPassFilter<int>;
template class CLowPassFilter<long>;
template class CLowPassFilter<float>;
template class CLowPassFilter<Vector2f>;
template class CLowPassFilter<Vector3f>;

