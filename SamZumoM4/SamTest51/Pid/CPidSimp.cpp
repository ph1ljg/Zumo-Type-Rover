/* 
* CPid.cpp
*
* Created: 31/12/2016 17:02:36
* Author: phil
*/

#include "Includes.h"
#include "CPidSimp.h"

// default constructor
CPidSimp::CPidSimp(	)
{
	m_KPValue = 0;
	m_KIValue = 0;
	m_KDValue = 0;

	m_IErroSum = 0;

} //CPid

// default destructor
CPidSimp::~CPidSimp()
{
} //~CPid

//=================================================================================
void CPidSimp::Initialise(int16_t* Input, int16_t* Output,int16_t SetPointIn)
{
    m_pOutput = Output;
    m_pInput = Input;
    m_Setpoint = SetPointIn;
    SetOutputLimits(1000, 2000);				//default output limit corresponds to
}

void CPidSimp::SetGains(uint16_t KPValue,uint16_t KIValue,uint16_t KDValue)
{
	m_KPValue = KPValue;
	m_KIValue = KIValue;
	m_KDValue = KDValue;
	
} 
void CPidSimp::Compute()					
{
	CMyMath Math; 
	int16_t InputTemp ;
	int32_t PTerm =0;
	int32_t ITerm =0;
	int32_t DTerm =0;
	int32_t dInput;
	int32_t OutTemp;
	
	InputTemp = *m_pInput;					//Compute all the working error variables
	m_Error = (m_Setpoint- InputTemp );
		
// 	if((Error > -2) && (Error<2))
// 	{
// 		m_IErroSum = 0;
// 		m_LastInput = InputTemp;
// //			*pOutput = 0;
// //			return(true);
// 	}
	PTerm = (m_Error * m_KPValue)/10;
	m_IErroSum  += (m_Error  * m_KIValue)/100;
	m_IErroSum = Math.Constrain(m_IErroSum,1000,2000);
	ITerm = m_IErroSum;
		
	dInput = (InputTemp - m_LastInput);
	m_LastInput = InputTemp;
		
	DTerm = (dInput* m_KDValue);		
	//printf("%ld  %ld  %ld\n ",PTerm,ITerm,DTerm);	
		
	OutTemp = (PTerm+ITerm+DTerm);
	
		
	OutTemp = Math.Constrain(OutTemp,m_OutMin,m_OutMax);
	*m_pOutput = OutTemp;
}



void CPidSimp::SetOutputLimits(int16_t Min, int16_t Max)
{
   if(Min >= Max) 
	return;
   m_OutMin = Min;
   m_OutMax = Max;

 
}


void CPidSimp::Initialize()
{
   m_IErroSum =0;
   m_LastInput = *m_pInput;
}

