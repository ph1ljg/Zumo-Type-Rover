/* 
* CPid_P.cpp
*
* Created: 14/08/2020 09:58:23
* Author: philg
*/


#include "Includes.h"

// default constructor
CPid_P::CPid_P(const float& Initialp)
{
	m_kp = Initialp;
} //CPid_P

// default destructor
CPid_P::~CPid_P()
{
} //~CPid_P


float CPid_P::Get_p(float error) const
{
	return (float)error * m_kp;
}

void CPid_P::Set_p(float Kp)
{
	  m_kp = Kp;
}
