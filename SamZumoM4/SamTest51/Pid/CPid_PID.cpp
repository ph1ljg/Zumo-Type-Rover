/* 
* CPid2.cpp
*
* Created: 26/07/2020 13:49:08
* Author: philg
*/

#include "Includes.h"


// default constructor
CPid_PID::CPid_PID(float initial_p, float initial_i, float initial_d, float initial_ff, float initial_imax, float initial_filt_T_hz, float initial_filt_E_hz, float initial_filt_D_hz, float dt) :
m_dt(dt),
m_Integrator(0.0f),
m_Error(0.0f),
m_Derivative(0.0f)
{
	m_Kp = initial_p;
	m_Ki = initial_i;
	m_Kd = initial_d;
	m_Kff = initial_ff;
	m_KiMax = fabsf(initial_imax);
	Filt_T_hz(initial_filt_T_hz);
	Filt_E_hz(initial_filt_E_hz);
	Filt_D_hz(initial_filt_D_hz);

	// reset input filter to first value received
	ResetFilter();

}



CPid_PID::CPid_PID() :m_dt(0),m_Integrator(0.0f),m_Error(0.0f),m_Derivative(0.0f)
{
	m_Flags.ResetFilter = true;		// reset input filter to first value received

} //CPid2

// default destructor
CPid_PID::~CPid_PID()
{
} //~CPid2



float CPid_PID::GetError()
{
	return(m_Error);
}

void CPid_PID::GetValues(float &Error,float &Input, float &Output)
{
	Error = m_Error;
	Input = m_DebugValue;
	Output = m_Output;
}



// set_dt - set time step in seconds
void CPid_PID::set_dt(float dt)
{
	m_dt = dt;
}

// filt_T_hz - set target filter hz
void CPid_PID::Filt_T_hz(float hz)
{
	m_Filt_T_Hz = fabsf(hz);
}

// filt_E_hz - set error filter hz
void CPid_PID::Filt_E_hz(float hz)
{
	m_Filt_E_Hz = fabsf(hz);
}

// filt_D_hz - set derivative filter hz
void CPid_PID::Filt_D_hz(float hz)
{
	Filt_D_Hz = fabsf(hz);
}

//  update_all - set target and measured inputs to PID controller and calculate outputs  target and error are filtered
//  the derivative is then calculated and filtered  the integral is then updated based on the setting of the limit flag
float CPid_PID::update_all(float target, float measurement, bool limit)
{
//	uint32_t TempVal = Core.millis()-m_LastUpdateTime;
//	m_dt = (float)Time;
	
	if (!isfinite(target) || !isfinite(measurement))		// don't process inf or NaN
		return 0.0f;
	m_DebugValue = target;
	
	if (m_Flags.ResetFilter)								// reset input filter to value received 
	{
		m_Flags.ResetFilter = false;
		m_Target = target;
		m_Error = m_Target - measurement;
		m_Derivative = 0.0f;
	} 
	else 
	{
		float error_last = m_Error;
		m_Target += GetFilt_T_Alpha() * (target - m_Target);
		m_Error += GetFilt_E_Alpha() * ((m_Target - measurement) - m_Error);

		// calculate and filter derivative
		if (m_dt > 0.0f) 
		{
			float derivative = (m_Error - error_last) / m_dt;
			m_Derivative += GetFilt_D_Alpha() * (derivative - m_Derivative);
		}
	}

	// update I term
	UpdateI(limit);

	float P_out = (m_Error * m_Kp);
	float D_out = (m_Derivative * m_Kd);
	m_Output =  P_out + m_Integrator + D_out;
	return(m_Output) ;
}

//  Set error input to PID controller and calculate outputs  target is set to zero and error is set and filtered  the derivative then is calculated and filtered
//  the integral is then updated based on the setting of the limit flag  
float CPid_PID::update_error(float error, bool limit)
{
	uint32_t TempVal = Core.millis()-m_LastUpdateTime;
	m_dt = (float)TempVal/1000;

	if (!isfinite(error))		// don't process inf or NaN
		return 0.0f;

	m_Target = 0.0f;

	// reset input filter to value received
	if (m_Flags.ResetFilter) 
	{
		m_Flags.ResetFilter = false;
		m_Error = error;
		m_Derivative = 0.0f;
	} 
	else 
	{
		float error_last = m_Error;
		m_Error += GetFilt_E_Alpha() * (error - m_Error);

		// calculate and filter derivative
		if (m_dt > 0.0f) 
		{
			float derivative = (m_Error - error_last) / m_dt;
			m_Derivative += GetFilt_D_Alpha() * (derivative - m_Derivative);
		}
	}

	// update I term
	UpdateI(limit);

	float P_out = (m_Error * m_Kp);
	float D_out = (m_Derivative * m_Kd);
	m_Output = P_out + m_Integrator + D_out;
	m_LastUpdateTime = Core.millis();

	return(m_Output) ;
}

// update_i - update the integral  If the limit flag is set the integral is only allowed to shrink
void CPid_PID::UpdateI(bool limit)
{
	CMyMath Math;
	if (!is_zero(m_Ki) && is_positive(m_dt) && !is_zero(m_Error)) 
	{
		// Ensure that integrator can only be reduced if the output is saturated
		if (!limit || ((is_positive(m_Integrator) && is_negative(m_Error)) || (is_negative(m_Integrator) && is_positive(m_Error)))) 
		{
			m_Integrator += ((float)m_Error * m_Ki) * m_dt;
			m_Integrator = Math.constrain_float(m_Integrator, -m_KiMax, m_KiMax);
		}
	} 
	else 
	{
		m_Integrator = 0.0f;
	}
}


void CPid_PID::ResetAll()
{
	ResetI();
	m_Flags.ResetFilter = true;
}

void CPid_PID::SetValues( float Kp,float Ki,float Kd)
{
    m_Kp = Kp;
    m_Ki = Ki;
    m_Kd = Kd;
	ResetI();
	m_Flags.ResetFilter = true;
}

float CPid_PID::GetP() const
{
	return m_Error * m_Kp;
}

float CPid_PID::GetI() const
{
	return m_Integrator;
}

float CPid_PID::GetD() const
{
	return m_Kd * m_Derivative;
}

float CPid_PID::GetFF()
{
	return m_Target * m_Kff;
}


void CPid_PID::ResetI()
{
	m_Integrator = 0;
}



/// Overload the function call operator to permit easy initialisation
void CPid_PID::operator()(float p_val, float i_val, float d_val,float initial_ff, float imax_val, float input_filt_T_hz, float input_filt_E_hz, float input_filt_D_hz, float dt)
{
	m_Kp = p_val;
	m_Ki = i_val;
	m_Kd = d_val;
	m_Kff = initial_ff;
	m_KiMax = fabsf(imax_val);
	m_Filt_T_Hz = input_filt_T_hz;
	m_Filt_E_Hz = input_filt_E_hz;
	Filt_D_Hz = input_filt_D_hz;
	m_dt = dt;
}

// get_filt_T_alpha - get the target filter alpha
float CPid_PID::GetFilt_T_Alpha() const
{
	return GetFiltAlpha(m_Filt_T_Hz);
}

// get_filt_E_alpha - get the error filter alpha
float CPid_PID::GetFilt_E_Alpha() const
{
	return GetFiltAlpha(m_Filt_E_Hz);
}

// get_filt_D_alpha - get the derivative filter alpha
float CPid_PID::GetFilt_D_Alpha() const
{
	return GetFiltAlpha(Filt_D_Hz);
}

// get_filt_alpha - calculate a filter alpha
float CPid_PID::GetFiltAlpha(float filt_hz) const
{
	if (is_zero(filt_hz)) 
		return 1.0f;

	// calculate alpha
	float rc = 1 / (M_2PI * filt_hz);
	return m_dt / (m_dt + rc);
}

void CPid_PID::SetIntegrator(float target, float measurement, float i)
{
	SetIntegrator(target - measurement, i);
}

void CPid_PID::SetIntegrator(float error, float i)
{
	CMyMath Math;
	m_Integrator = Math.constrain_float(i - error * m_Kp, -m_KiMax, m_KiMax);
}

void CPid_PID::SetIntegrator(float i)
{
	CMyMath Math;
	m_Integrator = Math.constrain_float(i, -m_KiMax, m_KiMax);
}
