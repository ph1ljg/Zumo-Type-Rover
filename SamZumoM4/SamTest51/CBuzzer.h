/* 
* CBuzzer.h
*
* Created: 03/10/2020 17:37:20
* Author: philg
*/


#ifndef __CBUZZER_H__
#define __CBUZZER_H__


class CBuzzer
{
//variables
public:
protected:
private:
	bool m_LastState;
//functions
public:
	CBuzzer();
	~CBuzzer();
	void Init();
	void Tone();
	void ToggleOutput();
protected:
private:
	CBuzzer( const CBuzzer &c );
	CBuzzer& operator=( const CBuzzer &c );

}; //CBuzzer

#endif //__CBUZZER_H__
