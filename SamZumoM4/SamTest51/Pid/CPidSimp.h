/* 
* CPid.h
*
* Created: 31/12/2016 17:02:36
* Author: phil
*/


#ifndef __CPID_H__
#define __CPID_H__
 //Constants used in some of the functions below
  #define PID_AUTOMATIC	1
  #define PID_MANUAL	0
  #define PID_DIRECT  0
  #define PID_REVERSE  1
  #define P_ON_M 0
  #define P_ON_E 1


class CPidSimp
{
//variables
public:


	int16_t *m_pInput;              // * Pointers to the Input, Output, and Setpoint variables
	int16_t *m_pOutput;             //   This creates a hard link between the variables and the PID
	int16_t m_Setpoint;           
	int16_t m_IErroSum, m_LastInput;
	int32_t m_Error ;
	int16_t m_OutMin, m_OutMax;
protected:
private:
	uint16_t m_KPValue;
	uint16_t m_KIValue;
	uint16_t m_KDValue;
//functions
public:
//	CPid();
	CPidSimp( );
	~CPidSimp();
	void Initialise(int16_t* Input, int16_t* Output,int16_t SetPointIn);
	void SetGains(uint16_t KPValue,uint16_t KIValue,uint16_t KDValue);
	void Compute();
	void SetOutputLimits(int16_t Min, int16_t Max);
	void Initialize();
protected:
private:
	CPidSimp( const CPidSimp &c );
	CPidSimp& operator=( const CPidSimp &c );

}; //CPid

#endif //__CPID_H__
