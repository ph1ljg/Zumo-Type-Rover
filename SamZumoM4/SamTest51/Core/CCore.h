/* 
* CCore.h
*
* Created: 31/01/2018 16:44:36
* Author: phil
*/


#ifndef __CCORE_H__
#define __CCORE_H__

#include "stdio.h"
#include "samd51j19a.h"

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#define VARIANT_MCK (120000000ul)

typedef void (*voidFuncPtr)( void ) ;

#define clockCyclesPerMicrosecond() ( SystemCoreClock / 1000000L )
#define clockCyclesToMicroseconds(a) ( ((a) * 1000L) / (SystemCoreClock / 1000L) )
#define microsecondsToClockCycles(a) ( (a) * (SystemCoreClock / 1000000L) )


#ifndef UNUSED
#define UNUSED(x) (void)(x)
#endif

enum BitOrder 
{
	LSBFIRST = 0,
	MSBFIRST = 1
};



//#include "compiler.h"


extern volatile uint32_t __atomic;

void atomic_enter_criticalp(uint32_t volatile *atomic);
void atomic_leave_critical(uint32_t volatile *);



#define CRITICAL_SECTION_ENTER()                  \
{                                                 \
	atomic_enter_criticalp(&__atomic);}


#define CRITICAL_SECTION_LEAVE()				  \
{											      \
atomic_leave_critical(&__atomic);}                


#define SERCOM_CRITICAL_SECTION_ENTER() CRITICAL_SECTION_ENTER()
#define SERCOM_CRITICAL_SECTION_LEAVE() CRITICAL_SECTION_LEAVE()

#include <stdio.h>

//#define GCLK_SOURCE_FDPLL           8        // FDPLL output

#define CpuUs2cycles(us, f_cpu) 	(((uint64_t)(us) * (f_cpu) + (uint64_t)(7e6-1ul)) / (uint64_t)7e6)


class CCore
{
//variables
public:
	uint32_t m_CoreCpuSpeed; 
	 int ticks = -1;
protected:
private:

//functions
public:
	CCore();
	~CCore();
	void Init();
	void tickReset();
	uint32_t millis( void );
	uint32_t micros( void );
	void delay( uint32_t ms );
	bool SetClkto48mhPLL();
	int FreeRam ();
	void delayMicroseconds(unsigned int us);
protected:
private:
	CCore( const CCore &c );
	CCore& operator=( const CCore &c );

}; //CCore
void SysTick_DefaultHandler(void);
#endif //__CCORE_H__
