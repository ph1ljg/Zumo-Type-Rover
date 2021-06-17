/*
 * Interrupts.cpp
 *
 * Created: 28/05/2020 09:54:54
 *  Author: philg
 */ 

#include "Includes.h"





void SERCOM5_0_Handler()		//DRE	indicates that the register is empty and ready for new data
{
	Uart.TxInterruptHandler();
}
void SERCOM5_1_Handler()		//TXC When the entire data frame including Stop bit(s) has been transmitted and no new data was written to DATA
{
	//Uart.IrqHandler();
}
void SERCOM5_2_Handler()
{
	Uart.RxInterruptHandler();  // RXC	Receive Complete
}
void SERCOM5_3_Handler()		// RXS generated immediately when a start is detected
{
	Uart.RxInterruptHandler();
}



void SERCOM3_0_Handler()
{
	SbusUart.TxInterruptHandler();
}
void SERCOM3_1_Handler()
{
	SbusUart.IrqHandler();
}
void SERCOM3_2_Handler()
{
	SbusUart.RxInterruptHandler();
}
void SERCOM3_3_Handler()
{
	SbusUart.RxInterruptHandler();
}


//===================================== Wheel encoders ===========================================================================================
void EIC_4_Handler()
{
	EIC->INTFLAG.reg |= 1;
	WheelEncoder.IncrementRightWheel();
}

void EIC_2_Handler()
{
	EIC->INTFLAG.reg |= 2;
	WheelEncoder.IncrementLeftWheel();
	
}



/*
 //  \brief Specifies a named Interrupt Service Routine (ISR) to call when an interrupt occurs.
 //        Replaces any previous function that was attached to the interrupt.
void AttachInterrupt(uint32_t pin, voidFuncPtr callback, uint32_t mode)
{
  static int enabled = 0;
  uint32_t config;
  uint32_t pos;

#if ARDUINO_SAMD_VARIANT_COMPLIANCE >= 10606
  EExt_Interrupts in = g_APinDescription[pin].ulExtInt;
#else
  EExt_Interrupts in = digitalPinToInterrupt(pin);
#endif
  if (in == NOT_AN_INTERRUPT) return;

  if (!enabled) {
    __initialize();
    enabled = 1;
  }

  if (in == EXTERNAL_INT_NMI) {
    EIC->NMIFLAG.bit.NMI = 1; // Clear flag
    switch (mode) {
      case LOW:
        EIC->NMICTRL.bit.NMISENSE = EIC_NMICTRL_NMISENSE_LOW;
        break;

      case HIGH:
        EIC->NMICTRL.bit.NMISENSE = EIC_NMICTRL_NMISENSE_HIGH;
        break;

      case CHANGE:
        EIC->NMICTRL.bit.NMISENSE = EIC_NMICTRL_NMISENSE_BOTH;
        break;

      case FALLING:
        EIC->NMICTRL.bit.NMISENSE = EIC_NMICTRL_NMISENSE_FALL;
        break;

      case RISING:
        EIC->NMICTRL.bit.NMISENSE = EIC_NMICTRL_NMISENSE_RISE;
        break;
    }

    // Assign callback to interrupt
    callbacksInt[EXTERNAL_INT_NMI] = callback;

  } else { // Not NMI, is external interrupt

    // Enable wakeup capability on pin in case being used during sleep
    EIC->WAKEUP.reg |= (1 << in);

    // Assign pin to EIC
    pinPeripheral(pin, PIO_EXTINT);

    // Look for right CONFIG register to be addressed
    if (in > EXTERNAL_INT_7) {
      config = 1;
    } else {
      config = 0;
    }

    // Configure the interrupt mode
    pos = (in - (8 * config)) << 2;
    switch (mode)
    {
      case LOW:
        EIC->CONFIG[config].reg |= EIC_CONFIG_SENSE0_LOW_Val << pos;
        break;

      case HIGH:
        EIC->CONFIG[config].reg |= EIC_CONFIG_SENSE0_HIGH_Val << pos;
        break;

      case CHANGE:
        EIC->CONFIG[config].reg |= EIC_CONFIG_SENSE0_BOTH_Val << pos;
        break;

      case FALLING:
        EIC->CONFIG[config].reg |= EIC_CONFIG_SENSE0_FALL_Val << pos;
        break;

      case RISING:
        EIC->CONFIG[config].reg |= EIC_CONFIG_SENSE0_RISE_Val << pos;
        break;
    }

    // Enable the interrupt
    EIC->INTENSET.reg = EIC_INTENSET_EXTINT(1 << in);

    // Assign callback to interrupt
    callbacksInt[in] = callback;
  }
}


//brief Turns off the given interrupt.

void detachInterrupt(uint32_t pin)
{
#if (ARDUINO_SAMD_VARIANT_COMPLIANCE >= 10606)
  EExt_Interrupts in = g_APinDescription[pin].ulExtInt;
#else
  EExt_Interrupts in = digitalPinToInterrupt(pin);
#endif 
  if (in == NOT_AN_INTERRUPT) return;

  if(in == EXTERNAL_INT_NMI) {
    EIC->NMICTRL.bit.NMISENSE = 0; // Turn off detection
  } else {
    EIC->INTENCLR.reg = EIC_INTENCLR_EXTINT(1 << in);
  
    // Disable wakeup capability on pin during sleep
    EIC->WAKEUP.reg &= ~(1 << in);
  }
}

void PIOA_Handler(void) {
	uint32_t isr = PIOA->PIO_ISR;
	uint32_t i;
	for (i=0; i<32; i++, isr>>=1) {
		if ((isr & 0x1) == 0)
		continue;
		if (callbacksPioA[i])
		callbacksPioA[i]();
	}
}

void PIOB_Handler(void) {
	uint32_t isr = PIOB->PIO_ISR;
	uint32_t i;
	for (i=0; i<32; i++, isr>>=1) {
		if ((isr & 0x1) == 0)
		continue;
		if (callbacksPioB[i])
		callbacksPioB[i]();
	}
}

void PIOC_Handler(void) {
	uint32_t isr = PIOC->PIO_ISR;
	uint32_t i;
	for (i=0; i<32; i++, isr>>=1) {
		if ((isr & 0x1) == 0)
		continue;
		if (callbacksPioC[i])
		callbacksPioC[i]();
	}
}

void PIOD_Handler(void) {
	uint32_t isr = PIOD->PIO_ISR;
	uint32_t i;
	for (i=0; i<32; i++, isr>>=1) {
		if ((isr & 0x1) == 0)
		continue;
		if (callbacksPioD[i])
		callbacksPioD[i]();
	}
}





*/