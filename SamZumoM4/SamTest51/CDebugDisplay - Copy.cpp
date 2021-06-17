/* 
* CDebugDisplay.cpp
*
* Created: 11/06/2020 12:42:50
* Author: philg
*/
#include "stdarg.h"
#include "stdio.h"
#include "CSoftwareSerial.h"
#include "CDebugDisplay.h"

extern CSoftwareSerial SoftwareSerial;


// default constructor
CDebugDisplay::CDebugDisplay()
{
} //CDebugDisplay

// default destructor
CDebugDisplay::~CDebugDisplay()
{
} //~CDebugDisplay



void CDebugDisplay::Init()
{
	WriteChar(27);	// ESC command
//	delay_us(10);
	WriteBytes((uint8_t*)"[2J",3);
//	delay_us(10);
	WriteChar(27);	// ESC command
//	delay_us(10);
	WriteBytes((uint8_t*)"[H",2);
//	delay_us(10);
	SetCursorState(false);
//	delay_us(10);
}



void CDebugDisplay::Printf(unsigned char Row,unsigned char Col, unsigned char ClearLen,const char *fmt,  ... )
{
	uint8_t Len;
	char Buffer[DEBUG_MAX_LINE_LEN];
	va_list ap;

	va_start(ap, fmt);
	Len = vsprintf(Buffer, fmt, ap);
	va_end(ap);
	//	ATOMIC_SECTION_ENTER;
	ClearLine(Row,Col,ClearLen);
	for(uint8_t i=0;i<Len;i++)
	{
		if (Buffer[i] == '\n')
			WriteChar('\r');
		WriteChar(Buffer[i]);
	}
	//	ATOMIC_SECTION_LEAVE;
}

void CDebugDisplay::Printf(const char *fmt,  ... )
{
	uint8_t Len;
	char Buffer[DEBUG_MAX_LINE_LEN];
	va_list ap;

	va_start(ap, fmt);
	Len = vsprintf(Buffer, fmt, ap);
	va_end(ap);
	//	ATOMIC_SECTION_ENTER;
	for(uint8_t i=0;i<Len;i++)
	{
		if (Buffer[i] == '\n')
		WriteChar('\r');
		WriteChar(Buffer[i]);
	}
	//	ATOMIC_SECTION_LEAVE;
}





void CDebugDisplay::ClearLine(unsigned char Row,unsigned char Col,unsigned char Length)
{
	unsigned char i;
	if(Length >70)
	Length = 70;
	SetDisplayRowCol(Row,Col);
	
	for(i=0;i<Length;i++)
	WriteChar(' ');
	SetDisplayRowCol(Row,Col);
}


void CDebugDisplay::SetDisplayRowCol(unsigned char Row,unsigned char Col)
{

	uint8_t TmpVal;
	unsigned char Hundreds;
	WriteChar(27);
	WriteChar('[');
	TmpVal = (Row/10)+'0';
	WriteChar(TmpVal);
	TmpVal = (Row%10)+'0';
	WriteChar(TmpVal);
	WriteChar(';');
	Hundreds = (Col/100);
	WriteChar(Hundreds+'0');
	Hundreds *= 100;
	TmpVal = Col - Hundreds;
	TmpVal = (TmpVal/10)+'0';
	WriteChar(TmpVal);
	TmpVal = (Col%10)+'0';
	WriteChar(TmpVal);
	WriteChar('f');

}


void CDebugDisplay::SetCursorState(bool OnOff)
{
	WriteChar( 27);	// ESC command
//	delay_us(10);
	if(OnOff)
	WriteBytes((uint8_t*) "[?25h",5);

	else
	WriteBytes((uint8_t*)"[?25l",5);
	
}


void CDebugDisplay::WriteChar(uint8_t Char)
{
	SoftwareSerial.Write(&Char,1); 
}

void CDebugDisplay::WriteBytes(uint8_t *Bytes,uint8_t Size)
{
	SoftwareSerial.Write(Bytes,Size);

}
