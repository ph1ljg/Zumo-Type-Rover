/* 
* CCirBuffer.cpp
*
* Created: 18/05/2020 14:41:59
* Author: philg
*/

#include "Includes.h"


static void (*EnableIrq)(bool);


// default constructor
CCirBuffer::CCirBuffer(uint8_t *InBuffer,uint8_t Size,void (*EnIrq)(bool))
{
	EnableIrq = EnIrq;
	m_Size = Size;
	m_Buffer = InBuffer;
} //CCirBuffer

// default destructor
CCirBuffer::~CCirBuffer()
{
} //~CCirBuffer


bool CCirBuffer::IsFull()
{
	if(m_Len >= m_Size)
		return(true);
	return(false);
}
void CCirBuffer::ClearBuffer()
{
	uint16_t i;
	EnableIrq(false);	m_Head = 0;
	m_Len = 0;
	m_Tail = 0;
	for(i=0;i<m_Size;i++)
		m_Buffer[i] =0;
	EnableIrq(true);	
}

uint16_t CCirBuffer::Write(unsigned char ch)
{
	if(m_Len < m_Size)	{		m_Buffer[m_Head] = ch;		m_Head++;		if(m_Head >= m_Size)			m_Head = 0;		EnableIrq(false);		m_Len++;		EnableIrq(true);		return(true);	}	else	return(false);}

bool CCirBuffer::Read(unsigned char *Data)
{
	if(m_Len > 0)
	{
		*Data = m_Buffer[m_Tail];
		m_Tail++;
		if(m_Tail >= m_Size)
			m_Tail = 0;
		EnableIrq(false);		m_Len--;
		EnableIrq(true);		
		return(true);
	}
	else
	return(false);
}

uint16_t CCirBuffer::Available()
{
	return(m_Len);
}
