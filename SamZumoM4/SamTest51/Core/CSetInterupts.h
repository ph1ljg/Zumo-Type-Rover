/* 
* CSetInterupts.h
*
* Created: 21/07/2020 18:41:20
* Author: philg
*/


#ifndef __CSETINTERUPTS_H__
#define __CSETINTERUPTS_H__

#define CHANGE 2
#define FALLING 3
#define RISING 4

class CSetInterupts
{
//variables
public:
protected:
private:

//functions
public:
	CSetInterupts();
	~CSetInterupts();
	void Init();
	void SetInterrupt(EPortType port ,uint32_t pin,EExt_Interrupts ExtIntNumber, EExt_Interrupts InType, uint32_t mode);
	void SetEnable(EExt_Interrupts ExtIntNumber,bool Enable);
protected:
private:
	CSetInterupts( const CSetInterupts &c );
	CSetInterupts& operator=( const CSetInterupts &c );

}; //CSetInterupts
#endif //__CSETINTERUPTS_H__
