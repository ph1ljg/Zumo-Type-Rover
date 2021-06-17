/* 
* CCirBuffer.h
*
* Created: 18/05/2020 14:41:59
* Author: philg
*/


#ifndef __CCIRBUFFER_H__
#define __CCIRBUFFER_H__
#include "stdio.h"

class CCirBuffer
{
//variables
public:
	uint16_t m_Head;
	uint16_t m_Tail;
	uint16_t m_Len, m_Size;
	unsigned char *m_Buffer;

protected:
private:

//functions
public:
	CCirBuffer(uint8_t *InBuffer,uint8_t Size,void (*EnIrq)(bool));
	CCirBuffer();
	~CCirBuffer();
	bool IsFull();
	void ClearBuffer();
	uint16_t Write(unsigned char ch);
	bool Read(unsigned char *Data);
	uint16_t Available();
protected:
private:
	CCirBuffer( const CCirBuffer &c );
	CCirBuffer& operator=( const CCirBuffer &c );

}; //CCirBuffer

#endif //__CCIRBUFFER_H__
