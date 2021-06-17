/* 
* CHarmonicNotchFilter.cpp
*
* Created: 20/03/2021 09:31:40
* Author: philg
*/


#include "Includes.h"
#include "CHarmonicNotchFilter.h"


// default constructor
 template <class T>
CHarmonicNotchFilter<T>::CHarmonicNotchFilter()
{
	Params._enable = 0;
	Params.CenterFreqHz[0] = 80;  //Harmonic Notch Filter base center frequency in Hz. This should be set at most half the backend gyro rate (which is typically 1Khz)
	Params.CenterFreqHz[1] = 80;
	Params.CenterFreqHz[2] = 80;
	Params.CenterFreqHz[3] = 80;
	Params.BandwidthHz = 40;	 //Harmonic Notch Filter bandwidth in Hz. This is typically set to half the base frequency. The ratio of base frequency to bandwidth determines the notch quality factor and is fixed across harmonics.
	Params.Attenuation_dB = 40; //Harmonic Notch Filter attenuation in dB. Values greater than 40dB will typically produce a hard notch rather than a modest attenuation	
	Params._harmonics =	3;		// 0:1st harmonic,1:2nd harmonic,2:3rd harmonic,3:4th harmonic,4:5th harmonic,5:6th harmonic,6:7th harmonic,7:8th harmonic
	Params._reference = 0;		// A reference value of zero disables dynamic updates on the Harmonic Notch Filter
	Params._tracking_mode = 1;	// 0:Disabled,1:Throttle,2:RPM Sensor,3:ESC Telemetry,4:Dynamic FFT
	Params._options = 0;		// 0:Double notch,1:Dynamic harmonic Double-notches can provide deeper attenuation across a wider bandwidth than single notches and are suitable for larger aircraft. Dynamic harmonics attaches a harmonic notch to each detected noise frequency instead of simply being multiples of the base frequency, in the case of FFT it will attach notches to each of three detected noise peaks
} //CHarmonicNotchFilter

//  destroy all of the associated notch filters
 template <class T>
CHarmonicNotchFilter<T>::~CHarmonicNotchFilter() 
{
    delete[] _filters;
    _num_filters = 0;
    _num_enabled_filters = 0;
}


//  initialise the associated filters with the provided shaping constraints
//  the constraints are used to determine attenuation (A) and quality (Q) factors for the filter
template <class T>
void CHarmonicNotchFilter<T>::init(float sample_freq_hz, float center_freq_hz, float bandwidth_hz, float attenuation_dB)
{
   CMyMath Math;
    // sanity check the input
    if (_filters == nullptr || is_zero(sample_freq_hz) || isnan(sample_freq_hz)) 
	{
        return;
    }

    _sample_freq_hz = sample_freq_hz;

    const float nyquist_limit = sample_freq_hz * 0.48f;
    const float bandwidth_limit = bandwidth_hz * 0.52f;
    // adjust the fundamental center frequency to be in the allowable range
    center_freq_hz = Math.constrain_float(center_freq_hz, bandwidth_limit, nyquist_limit);
    // Calculate spread required to achieve an equivalent single notch using two notches with Bandwidth/2
    _notch_spread = bandwidth_hz / (32 * center_freq_hz);

    if (_double_notch) 
	{
        // position the individual notches so that the attenuation is no worse than a single notch
        // calculate attenuation and quality from the shaping constraints
        CNotchFilter<T>::calculate_A_and_Q(center_freq_hz, bandwidth_hz * 0.5, attenuation_dB, _A, _Q);
    } 
	else 
	{
        // calculate attenuation and quality from the shaping constraints
        CNotchFilter<T>::calculate_A_and_Q(center_freq_hz, bandwidth_hz, attenuation_dB, _A, _Q);
    }

    _initialised = true;
    update(center_freq_hz);
}

//  allocate a collection of, at most HNF_MAX_FILTERS, notch filters to be managed by this harmonic notch filter
 template <class T>
void CHarmonicNotchFilter<T>::allocate_filters(uint8_t harmonics, bool double_notch)
{
    _double_notch = double_notch;

    for (uint8_t i = 0; i < HNF_MAX_HARMONICS && _num_filters < HNF_MAX_FILTERS; i++) 
	{
        if ((1U<<i) & harmonics) 
		{
            _num_filters++;
            if (_double_notch) 
	            _num_filters++;
        }
    }
    if (_num_filters > 0) 
	{
        _filters = new CNotchFilter<T>[_num_filters];
        if (_filters == nullptr) 
		{
            DebugDisplay.Printf( "Failed to allocate %u bytes for HarmonicNotchFilter", (unsigned int)(_num_filters * sizeof(CNotchFilter<T>)));
            _num_filters = 0;
        }

    }
    _harmonics = harmonics;
}


//  update the underlying filters' center frequency using the current attenuation and quality
//  this function is cheaper than init() because A & Q do not need to be recalculated
template <class T>
void CHarmonicNotchFilter<T>::update(float center_freq_hz)
{
    CMyMath Math;
	if (!_initialised) 
	{
        return;
    }

    // adjust the fundamental center frequency to be in the allowable range
    const float nyquist_limit = _sample_freq_hz * 0.48f;
    center_freq_hz = Math.constrain_float(center_freq_hz, 1.0f, nyquist_limit);

    _num_enabled_filters = 0;
    // update all of the filters using the new center frequency and existing A & Q
    for (uint8_t i = 0; i < HNF_MAX_HARMONICS && _num_enabled_filters < _num_filters; i++) 
	{
        if ((1U<<i) & _harmonics) 
		{
            const float notch_center = center_freq_hz * (i+1);
            if (!_double_notch) 
			{
                // only enable the filter if its center frequency is below the nyquist frequency
                if (notch_center < nyquist_limit) 
				{
                    _filters[_num_enabled_filters++].init_with_A_and_Q(_sample_freq_hz, notch_center, _A, _Q);
                }
            } 
			else 
			{
                float notch_center_double;
                // only enable the filter if its center frequency is below the nyquist frequency
                notch_center_double = notch_center * (1.0 - _notch_spread);
                if (notch_center_double < nyquist_limit) 
				{
                    _filters[_num_enabled_filters++].init_with_A_and_Q(_sample_freq_hz, notch_center_double, _A, _Q);
                }
                // only enable the filter if its center frequency is below the nyquist frequency
                notch_center_double = notch_center * (1.0 + _notch_spread);
                if (notch_center_double < nyquist_limit) 
				{
                    _filters[_num_enabled_filters++].init_with_A_and_Q(_sample_freq_hz, notch_center_double, _A, _Q);
                }
            }
        }
    }
}

//  update the underlying filters' center frequency using the current attenuation and quality
//  this function is cheaper than init() because A & Q do not need to be recalculated

template <class T>
void CHarmonicNotchFilter<T>::update(uint8_t num_centers,  float center_freq_hz[])
{
   CMyMath Math;
    if (!_initialised) 
	{
        return;
    }

    // adjust the frequencies to be in the allowable range
    const float nyquist_limit = _sample_freq_hz * 0.48f;

    _num_enabled_filters = 0;
    // update all of the filters using the new center frequencies and existing A & Q
    for (uint8_t i = 0; i < HNF_MAX_HARMONICS && i < num_centers && _num_enabled_filters < _num_filters; i++) 
	{
        const float notch_center = Math.constrain_float(center_freq_hz[i], 1.0f, nyquist_limit);
        if (!_double_notch) 
		{
            // only enable the filter if its center frequency is below the Nyquist frequency
            if (notch_center < nyquist_limit) 
			{
                _filters[_num_enabled_filters++].init_with_A_and_Q(_sample_freq_hz, notch_center, _A, _Q);
            }
        } 
		else 
		{
            float notch_center_double;
            // only enable the filter if its center frequency is below the Nyquist frequency
            notch_center_double = notch_center * (1.0 - _notch_spread);
            if (notch_center_double < nyquist_limit) 
			{
                _filters[_num_enabled_filters++].init_with_A_and_Q(_sample_freq_hz, notch_center_double, _A, _Q);
            }
            // only enable the filter if its center frequency is below the Nyquist frequency
            notch_center_double = notch_center * (1.0 + _notch_spread);
            if (notch_center_double < nyquist_limit) 
			{
                _filters[_num_enabled_filters++].init_with_A_and_Q(_sample_freq_hz, notch_center_double, _A, _Q);
            }
        }
    }
}


//  apply a sample to each of the underlying filters in turn and return the output
template <class T>
T CHarmonicNotchFilter<T>::apply(const T &sample)
{
    if (!_initialised) 
	{
        return sample;
    }

    T output = sample;
    for (uint8_t i = 0; i < _num_enabled_filters; i++) 
	{
        output = _filters[i].apply(output);
    }
    return output;
}

//  reset all of the underlying filters
template <class T>
void CHarmonicNotchFilter<T>::reset()
{
    if (!_initialised) 
	{
        return;
    }

    for (uint8_t i = 0; i < _num_filters; i++) 
	{
        _filters[i].reset();
    }
}


 
//  instantiate template classes
template class CHarmonicNotchFilter<Vector3f>;
