/* 
* CCore.cpp
*
* Created: 31/01/2018 16:44:36
* Author: phil
*/

#include "Includes.h"

extern "C"
{
//#include <asf.h>
//#include "gclk.h"
	
	};




volatile uint32_t __atomic;

volatile uint32_t Freq;

#include "CCore.h"
//#include "Reset.h" // for tickReset()
//#include <compiler.h>
/** Tick Counter united by ms */
volatile uint32_t _ulTickCount=0 ;



void atomic_enter_criticalp(uint32_t volatile *atomic)
{
	*atomic = __get_PRIMASK();
	__disable_irq();
	__DMB();
}

void atomic_leave_critical(uint32_t volatile *atomic)
{
	__DMB();
	__set_PRIMASK(*atomic);
}


extern CCore Core;

void SysTick_Handler(void)
{
	// Increment tick count each ms
	_ulTickCount++;
	Core.tickReset();
}

void SysTick_DefaultHandler(void)
{
	// Increment tick count each ms
	_ulTickCount++;
	Core.tickReset();
}

// default constructor
CCore::CCore()
{
} //CCore

// default destructor
CCore::~CCore()
{
} //~CCore





void CCore::Init()
{
	SystemCoreClock = 120000000L;
	SysTick_Config( SystemCoreClock / 1000 );
//	system_gclk_gen_get_hz(0);
}


void CCore::tickReset() 
{
	if (ticks == -1)
		return;
	ticks--;
	if (ticks == 0)
		NVIC_SystemReset() ;
}


uint32_t CCore::millis( void )
{
	// todo: ensure no interrupts
	return _ulTickCount ;
}


// Interrupt-compatible version of micros
// Theory: repeatedly take readings of SysTick counter, millis counter and SysTick interrupt pending flag.
// When it appears that millis counter and pending is stable and SysTick hasn't rolled over, use these
// values to calculate micros. If there is a pending SysTick, add one to the millis counter in the calculation.
uint32_t CCore::micros( void )
{
	uint32_t ticks, ticks2;
	uint32_t pend, pend2;
	uint32_t count, count2;

	ticks2  = SysTick->VAL;
	pend2   = !!(SCB->ICSR & SCB_ICSR_PENDSTSET_Msk)  ;
	count2  = _ulTickCount ;

	do
	{
		ticks=ticks2;
		pend=pend2;
		count=count2;
		ticks2  = SysTick->VAL;
		pend2   = !!(SCB->ICSR & SCB_ICSR_PENDSTSET_Msk)  ;
		count2  = _ulTickCount ;
	} while ((pend != pend2) || (count != count2) || (ticks < ticks2));

	return ((count+pend) * 1000) + (((SysTick->LOAD  - ticks)*(1048576/(VARIANT_MCK/1000000)))>>20) ;
	// this is an optimization to turn a runtime division into two compile-time divisions and
	// a runtime multiplication and shift, saving a few cycles
}

void CCore::delay( uint32_t ms )
{
	if (ms == 0)
	{
		return;
	}

	uint32_t start = micros();

	while (ms > 0)
	{
//		yield();
		while (ms > 0 && (micros() - start) >= 1000)
		{
			ms--;
			start += 1000;
		}
	}
}

//========================================================================================================================================
//  On SAMD51, use the (32bit) cycle count maintained by the DWT unit,  and count exact number of cycles elapsed, rather than guessing how
//  many cycles a loop takes, which is dangerous in the presence of cache.  The overhead of the call and internal code is "about" 20
//  cycles.  (at 120MHz, that's about 1/6 us)
//========================================================================================================================================
 void CCore::delayMicroseconds(unsigned int us)
{
  uint32_t start, elapsed;
  uint32_t count;

  if (us == 0)
    return;

  count = us * (VARIANT_MCK / 1000000) - 20;  // convert us to cycles.
  start = DWT->CYCCNT;  //CYCCNT is 32bits, takes 37s or so to wrap.
  while (1) 
  {
    elapsed = DWT->CYCCNT - start;
    if (elapsed >= count)
      return;
  }
}





extern "C" char *sbrk(int i);
int CCore::FreeRam () 
{
	char stack_dummy = 0;
	return &stack_dummy - sbrk(0);
}

// void CCore::delayMicroseconds( uint32_t usec )
// {
// 	uint32_t Value = CpuUs2cycles(usec,m_CoreCpuSpeed);
// 	DelayCycles(Value);
// } 



