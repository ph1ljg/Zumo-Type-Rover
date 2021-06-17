/* 
* CNotchFilter.cpp
*
* Created: 19/03/2021 18:40:39
* Author: philg
*/


#include "math.h"
#include "Includes.h"
#include "CNotchFilter.h"


// default constructor
template <class T>
CNotchFilter<T>::CNotchFilter()
{
	NotchParams._enable = 1;
	NotchParams.CenterFreqHz = 80;
	NotchParams.BandwidthHz = 20;
	NotchParams.Attenuation_dB =15;

} //CNotchFilter

// default destructor
// template <class T>
// CNotchFilter<T>::~CNotchFilter()
// {
// 
// } //~CNotchFilter

//   calculate the attenuation and quality factors of the filter
template <class T>
void CNotchFilter<T>::calculate_A_and_Q(float center_freq_hz, float bandwidth_hz, float attenuation_dB, float& A, float& Q) 
{
    A = powf(10, -attenuation_dB / 40.0f);
    if (center_freq_hz > 0.5 * bandwidth_hz) 
	{
        const float octaves = log2f(center_freq_hz / (center_freq_hz - bandwidth_hz / 2.0f)) * 2.0f;
        Q = sqrtf(powf(2, octaves)) / (powf(2, octaves) - 1.0f);
    } 
	else 
	{
        Q = 0.0;
    }
}

//  initialise filter
template <class T>
void CNotchFilter<T>::init(float sample_freq_hz, float center_freq_hz, float bandwidth_hz, float attenuation_dB)
{
    // check center frequency is in the allowable range
    if ((center_freq_hz > 0.5 * bandwidth_hz) && (center_freq_hz < 0.5 * sample_freq_hz)) 
	{
        float A, Q;
        calculate_A_and_Q(center_freq_hz, bandwidth_hz, attenuation_dB, A, Q);
        init_with_A_and_Q(sample_freq_hz, center_freq_hz, A, Q);
    } 
	else 
	{
        initialised = false;
    }
}

template <class T>
void CNotchFilter<T>::init_with_A_and_Q(float sample_freq_hz, float center_freq_hz, float A, float Q)
{
    if ((center_freq_hz > 0.0) && (center_freq_hz < 0.5 * sample_freq_hz) && (Q > 0.0)) 
	{
        float omega = 2.0 * M_PI * center_freq_hz / sample_freq_hz;
        float alpha = sinf(omega) / (2 * Q);
        b0 =  1.0 + alpha*sq(A);
        b1 = -2.0 * cosf(omega);
        b2 =  1.0 - alpha*sq(A);
        a0_inv =  1.0/(1.0 + alpha);
        a1 = b1;
        a2 =  1.0 - alpha;
        initialised = true;
    } 
	else 
	{
        initialised = false;
    }
}

/*
  apply a new input sample, returning new output
 */
template <class T>
T CNotchFilter<T>::apply(const T &sample)
{
    if (!initialised) 
	{
        // if we have not been initialised when return the input
        // sample as output and update delayed samples
        ntchsig2 = ntchsig1;
        ntchsig1 = ntchsig;
        ntchsig = sample;
        signal2 = signal1;
        signal1 = sample;
        return sample;
    }
    ntchsig2 = ntchsig1;
    ntchsig1 = ntchsig;
    ntchsig = sample;
    T output = (ntchsig*b0 + ntchsig1*b1 + ntchsig2*b2 - signal1*a1 - signal2*a2) * a0_inv;
    signal2 = signal1;
    signal1 = output;
    return output;
}

template <class T>
void CNotchFilter<T>::reset()
{
    ntchsig2 = ntchsig1 = T();
    signal2 = signal1 = T();
}

template class CNotchFilter<float>;
template class CNotchFilter<Vector3f>;
