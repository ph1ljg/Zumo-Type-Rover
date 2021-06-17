/* 
* CMyMath.cpp
*
* Created: 19/05/2020 09:42:36
* Author: philg
*/


#include "CMyMath.h"
#include <stdio.h>
#include "math.h"
#include "CMyMath.h"
#include "Compiler.h"
// default constructor
CMyMath::CMyMath()
{
} //CMyMath

// default destructor
CMyMath::~CMyMath()
{
} //~CMyMath





long CMyMath::Map(long x, long in_min, long in_max, long out_min, long out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int CMyMath::Constrain(int amt, int low, int high)
{
	return ((amt)<(low)?(low):((amt)>(high)?(high):(amt)));
}


//=============================================================
// linear interpolation based on a variable in a range
//=============================================================
float CMyMath::LinearInterpolate(float low_output, float high_output,float var_value, float var_low, float var_high)
{
    if (var_value <= var_low) 
        return low_output;

    if (var_value >= var_high) 
        return high_output;

    float p = (var_value - var_low) / (var_high - var_low);
    return low_output + p * (high_output - low_output);
}



/*
template <class T> T constrain_value(const T amt, const T low, const T high)
{
	// the check for NaN as a float prevents propagation of floating point
	// errors through any function that uses constrain_float(). The normal
	// float semantics already handle -Inf and +Inf
	if (isnan(amt))
		return (low + high) * 0.5f;

	if (amt < low)
		return low;

	if (amt > high)
		return high;

	return amt;
}



template int constrain_value<int>(const int amt, const int low, const int high);
template short constrain_value<short>(const short amt, const short low, const short high);
template float constrain_value<float>(const float amt, const float low, const float high);
template double constrain_value<double>(const double amt, const double low, const double high);
*/






double CMyMath::Clip(double n, double minValue, double maxValue)
{
	return Min(Max(n, minValue), maxValue);
}


//  calculate a low pass filter alpha value
 float CMyMath::calc_lowpass_alpha_dt(float dt, float cutoff_freq)
{
    if (dt <= 0.0f || cutoff_freq <= 0.0f) 
        return 1.0;

    float rc = 1.0f/(M_2PI*cutoff_freq);
    return constrain_float(dt/(dt+rc), 0.0f, 1.0f);
}




template <class T> float wrap_360(const T angle, float unit_mod)
{
	const float ang_360 = 360.f * unit_mod;
	float res = fmod(static_cast<float>(angle), ang_360);
	if (res < 0) 
	{
		res += ang_360;
	}
	return res;
}


long wrap_360_cd(const long angle)
{
	long res = angle % 36000;
	if (res < 0) 
	{
		res += 36000;
	}
	return res;
}




template float wrap_360<int>(const int angle, float unit_mod);
template float wrap_360<short>(const short angle, float unit_mod);
template float wrap_360<float>(const float angle, float unit_mod);
template float wrap_360<double>(const double angle, float unit_mod);

template <typename T> T wrap_180(const T angle)
{
	auto res = wrap_360(angle);
	if (res > T(180)) 
	{
		res -= T(360);
	}
	return res;
}


template int wrap_180<int>(const int angle);
template short wrap_180<short>(const short angle);
template float wrap_180<float>(const float angle);




template <class T> float safe_asin(const T v)
{
	if (isnan(static_cast<float>(v))) 
	{
		return 0.0f;
	}
	if (v >= 1.0f) 
	{
		return static_cast<float>(M_PI_2);
	}
	if (v <= -1.0f) {
		return static_cast<float>(-M_PI_2);
	}
	return asinf(static_cast<float>(v));
}

template float safe_asin<int>(const int v);
template float safe_asin<short>(const short v);
template float safe_asin<float>(const float v);
template float safe_asin<double>(const double v);



template <typename T>	float wrap_PI(const T radian)
{
	auto res = wrap_2PI(radian);
	if (res > M_PI)
	{
		res -= M_2PI;
	}
	return res;
}



template float wrap_PI<int>(const int radian);
template float wrap_PI<short>(const short radian);
template float wrap_PI<float>(const float radian);
template float wrap_PI<double>(const double radian);







template <typename T>	T  wrap_180_cd(const T angle)
{
	CMyMath Math;
	auto res = Math.wrap_360_cd( angle);
	if (res > T(18000))
	{
		res -= T(36000);
	}
	return res;
}

template int wrap_180_cd<int>(const int angle);
template int32_t wrap_180_cd<int32_t>(const int32_t angle);
template short wrap_180_cd<short>(const short angle);
template float wrap_180_cd<float>(const float angle);


template <typename T> float wrap_2PI(const T radian)
{
	float res = fmodf(static_cast<float>(radian), M_2PI);
	if (res < 0)
	{
		res += M_2PI;
	}
	return res;
}

template float wrap_2PI<int>(const int radian);
template float wrap_2PI<short>(const short radian);
template float wrap_2PI<float>(const float radian);
template float wrap_2PI<double>(const double radian);




// Small-angle approximation: http://en.wikipedia.org/wiki/Small-angle_approximation
// Rotation matrix: http://en.wikipedia.org/wiki/Rotation_matrix
void CMyMath::VectorRotate(vectorf &RotateVector,float* delta)
{
	vectorf TmpVector = RotateVector;
	RotateVector.Axis.z -= delta[0]  * TmpVector.Axis.x + delta[1] * TmpVector.Axis.y;
	RotateVector.Axis.x += delta[0]  * TmpVector.Axis.z - delta[2]   * TmpVector.Axis.y;
	RotateVector.Axis.y += delta[1] * TmpVector.Axis.z + delta[2]   * TmpVector.Axis.x;
}



// Rotate Estimated vector(s) with small angle approximation,
void CMyMath::VectorRotateV32( Vector32U_t *v,int16_t* delta)
{
	int16_t X = v->V16.X;
	int16_t Y = v->V16.Y;
	int16_t Z = v->V16.Z;

	v->V32.Z -=  (delta[0] *   X)  + (delta[1] * Y);
	v->V32.X +=  (delta[0]  *  Z) - (delta[2]   * Y);
	v->V32.Y +=  (delta[1] *  Z) + (delta[2]   * X);
}


float CMyMath::InvSqrt (float x)
{
	union
	{
		int32_t i;
		float   f;
	} conv;
	conv.f = x;
	conv.i = 0x5f1ffff9 - (conv.i >> 1);
	return conv.f * (1.68191409f - 0.703952253f * x * conv.f * conv.f);
}


double safe_sqrt(double x) 
{
	if (x < 0) 
	{
		return(0); 
	}
	return (sqrt (x));
}

// float CMyMath::InvSqrt (float x)
// {
//   union
//   {
//     int32_t i;
//     float   f;
//   } conv;
//   conv.f = x;
//   conv.i = 0x5f3759df - (conv.i >> 1);
//   return 0.5f * conv.f * (3.0f - x * conv.f * conv.f);
// }



void CMyMath::VectorCrossI(const vector &a, const vector &b, vector &out)
{
	out.Axis.x = a.Axis.y * b.Axis.z - a.Axis.z * b.Axis.y;
	out.Axis.y = a.Axis.z * b.Axis.x - a.Axis.x * b.Axis.z;
	out.Axis.z = a.Axis.x * b.Axis.y - a.Axis.y * b.Axis.x;
}

void CMyMath::VectorCrossF(const vectorf &a, const vectorf &b, vectorf &out)
{
	out.Axis.x = a.Axis.y * b.Axis.z - a.Axis.z * b.Axis.y;
	out.Axis.y = a.Axis.z * b.Axis.x - a.Axis.x * b.Axis.z;
	out.Axis.z = a.Axis.x * b.Axis.y - a.Axis.y * b.Axis.x;
}



float CMyMath::VectorDotI(const vector &a, const vector &b)
{
	return a.Axis.x * b.Axis.x + a.Axis.y * b.Axis.y + a.Axis.z * b.Axis.z;
}


float CMyMath::VectorDotF(const vectorf &a, const vectorf &b)
{
	return a.Axis.x * b.Axis.x + a.Axis.y * b.Axis.y + a.Axis.z * b.Axis.z;
}


void CMyMath::VectorNormalizeI(vector &a)
{
	float mag = sqrt(VectorDotI(a, a));
	if(mag != 0)
	{
		a.Axis.x /= mag;
		a.Axis.y /= mag;
		a.Axis.z /= mag;
	}
}

void CMyMath::VectorNormalizeF(vectorf &a)
{
	float mag = sqrt(VectorDotF(a, a));
	if(mag != 0)
	{
		a.Axis.x /= mag;
		a.Axis.y /= mag;
		a.Axis.z /= mag;
	}
}




void CMyMath::vector_normalize(vectorf &a)
{
	float mag = sqrt(vector_dot(a, a));
	a.Axis.x /= mag;
	a.Axis.y /= mag;
	a.Axis.z /= mag;
}



int32_t CMyMath::Round(double number)
{
	return (number >= 0) ? (int32_t)(number + 0.5) : (int32_t)(number - 0.5);
}


float CMyMath::Roundf_2(float var)
{
	// 37.66666 * 100 =3766.66
	// 3766.66 + .5 =3767.16    for rounding off value
	// then type cast to int so value is 3767
	// then divided by 100 so the value converted into 37.67
	float value = (int)(var * 100 + .5);
	return (float)value / 100;
}

// int main()
// {
//     float var = 37.66666;
//     cout << round(var);
//     return 0;
// }



double CMyMath::RoundF(double Value)
{
	if (Value == 0.)
	return(Value);
	Value>=0. ? (floor(Value*100.)/100.0) : (ceil(Value*100.)/100.0);
	return(Value);
}



float CMyMath::Roundf2(float var)
{
	
	char str[40];					// use array of chars to store number as a string.
	
	sprintf(str, "%.2f", var);		// Print in string the value of var  with two decimal point
	sscanf(str, "%f", &var);		// scan string value in var
	return var;
}





int16_t CMyMath::_atan2(int32_t y, int32_t x)
{
	float z = (float)y / x;
	int16_t a;
	if ( abs(y) < abs(x) )
	{
		a = 573 * z / (1.0f + 0.28f * z * z);
		if (x<0)
		{
			if (y<0)
			a -= 1800;
			else
			a += 1800;
		}
	}
	else
	{
		a = 900 - 573 * z / (z * z + 0.28f);
		if (y<0)
		a -= 1800;
	}
	return a;
}


uint16_t CMyMath::KnotsToMotorMs(uint16_t Knots)
{
	uint16_t Ms = 125*Knots+1000;
	return(Ms);
}

int16_t CMyMath::MetersSecToKnots(int16_t MetersSec)
{
	return(MetersSec * 1.94384);
}


int16_t CMyMath::CentMetersSecToKnots(int16_t CmMetersSec)
{
	return(CmMetersSec * 0.0194384);
}
