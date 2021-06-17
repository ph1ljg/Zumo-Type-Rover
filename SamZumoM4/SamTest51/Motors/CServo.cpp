/* 
* CPwm.cpp
*
* Created: 18/05/2020 19:19:07
* Author: philg
*/

#include "Includes.h"



volatile uint8_t ServoCount = 0;                                    // the total number of attached servos
 static servo_t servos[MAX_SERVOS];                         // static array of servo structures
 static volatile int8_t currentServoIndex[_Nbr_16timers];   // index for the servo being pulsed for each timer (or -1 if refresh interval)

 //============ static functions common to all instances ================

 void Servo_Handler(timer16_Sequence_t timer, Tc *pTc, uint8_t channel, uint8_t intFlag);
 
 #if defined (_useTimer1)
 void HANDLER_FOR_TIMER1(void) 
 {
	 Servo_Handler(_timer1, TC_FOR_TIMER1, CHANNEL_FOR_TIMER1, INTFLAG_BIT_FOR_TIMER_1);
 }
 #endif
 
 #if defined (_useTimer2)
 void HANDLER_FOR_TIMER2(void) 
 {
	 Servo_Handler(_timer2, TC_FOR_TIMER2, CHANNEL_FOR_TIMER2, INTFLAG_BIT_FOR_TIMER_2);
 }
 #endif

 void Servo_Handler(timer16_Sequence_t timer, Tc *tc, uint8_t channel, uint8_t intFlag)
 {
	 if (currentServoIndex[timer] < 0) 
	 {
		 tc->COUNT16.COUNT.reg = (uint16_t) 0;
		 while(tc->COUNT16.SYNCBUSY.bit.COUNT);
	} 
	else 
	{
		if (SERVO_INDEX(timer, currentServoIndex[timer]) < ServoCount && SERVO(timer, currentServoIndex[timer]).Pin.isActive == true) 
		{
			PinPeripheral.SetPin(SERVO(timer, currentServoIndex[timer]).port,SERVO(timer, currentServoIndex[timer]).Pin.nbr, LOW);   // pulse this channel low if activated
		}
    }

   // Select the next servo controlled by this timer
   currentServoIndex[timer]++;

   if (SERVO_INDEX(timer, currentServoIndex[timer]) < ServoCount && currentServoIndex[timer] < SERVOS_PER_TIMER) 
   {
	   if (SERVO(timer, currentServoIndex[timer]).Pin.isActive == true) 
	   {   // check if activated
		 PinPeripheral.SetPin(SERVO(timer, currentServoIndex[timer]).port,SERVO(timer, currentServoIndex[timer]).Pin.nbr, HIGH);   // it's an active channel so pulse it high
	   }

		// Note from data sheet: Prior to any read access, this register must be synchronized by user by writing the according TC
		// Command value to the Control B Set register (CTRLBSET.CMD=READSYNC)
		while (tc->COUNT16.SYNCBUSY.bit.CTRLB);
		tc->COUNT16.CTRLBSET.bit.CMD = TC_CTRLBSET_CMD_READSYNC_Val;
		while (tc->COUNT16.SYNCBUSY.bit.CTRLB);
		uint16_t tcCounterValue = tc->COUNT16.COUNT.reg;
		while(tc->COUNT16.SYNCBUSY.bit.COUNT);

		tc->COUNT16.CC[channel].reg = (uint16_t) (tcCounterValue + SERVO(timer, currentServoIndex[timer]).ticks);
		if(channel == 0) 
		{
			while(tc->COUNT16.SYNCBUSY.bit.CC0);
		}
		else if(channel == 1) 
		{
			while(tc->COUNT16.SYNCBUSY.bit.CC1);
		}
	}
	else 
	{
		// finished all channels so wait for the refresh period to expire before starting over Get the counter value
		uint16_t tcCounterValue = tc->COUNT16.COUNT.reg;
		while(tc->COUNT16.SYNCBUSY.bit.COUNT);

		if (tcCounterValue + 4UL < usToTicks(REFRESH_INTERVAL)) {   // allow a few ticks to ensure the next OCR1A not missed
			tc->COUNT16.CC[channel].reg = (uint16_t) usToTicks(REFRESH_INTERVAL);
		}
		else 
		{
			tc->COUNT16.CC[channel].reg = (uint16_t) (tcCounterValue + 4UL);   // at least REFRESH_INTERVAL has elapsed
		}
		if(channel == 0) 
		{
			while(tc->COUNT16.SYNCBUSY.bit.CC0);
		} 
		else if(channel == 1) 
		{
			while(tc->COUNT16.SYNCBUSY.bit.CC1);
		}

		currentServoIndex[timer] = -1;   // this will get incremented at the end of the refresh period to start again at the first channel
	}

	// Clear the interrupt
	tc->COUNT16.INTFLAG.reg = intFlag;
}

static inline void resetTC (Tc* TCx)
{
	// Disable TCx
	TCx->COUNT16.CTRLA.reg &= ~TC_CTRLA_ENABLE;
	while(TCx->COUNT16.SYNCBUSY.bit.ENABLE);

	// Reset TCx
	TCx->COUNT16.CTRLA.reg = TC_CTRLA_SWRST;
	while(TCx->COUNT16.SYNCBUSY.bit.SWRST);
	while (TCx->COUNT16.CTRLA.bit.SWRST);
}

static void InitTimer(Tc *tc, uint8_t channel, IRQn_Type irqn, uint8_t gcmForTimer, uint8_t intEnableBit)
{
	// Select GCLK0 as timer/counter input clock source
	int idx = gcmForTimer;           // see datasheet Table 14-9
	GCLK->PCHCTRL[idx].bit.GEN  = 0; // Select GCLK0 as periph clock source
	GCLK->PCHCTRL[idx].bit.CHEN = 1; // Enable peripheral
	while(!GCLK->PCHCTRL[idx].bit.CHEN);

	// Reset the timer
	// TODO this is not the right thing to do if more than one channel per timer is used by the Servo library
	resetTC(tc);

	// Set timer counter mode to 16 bits
	tc->COUNT16.CTRLA.reg |= TC_CTRLA_MODE_COUNT16;

	// Set timer counter mode as normal PWM
	tc->COUNT16.WAVE.bit.WAVEGEN = TCC_WAVE_WAVEGEN_NPWM_Val;

	// Set the prescaler factor to 64 or 128 depending on FCPU
	// (avoid overflowing 16-bit clock counter)
	#if(F_CPU > 200000000)
	tc->COUNT16.CTRLA.bit.PRESCALER = TCC_CTRLA_PRESCALER_DIV128_Val;
	#else
	// At 120-200 MHz GCLK this is 1875-3125 ticks per millisecond
	tc->COUNT16.CTRLA.bit.PRESCALER = TCC_CTRLA_PRESCALER_DIV64_Val;
	#endif

	// Count up
	tc->COUNT16.CTRLBCLR.bit.DIR = 1;
	while(tc->COUNT16.SYNCBUSY.bit.CTRLB);

	// First interrupt request after 1 ms
	tc->COUNT16.CC[channel].reg = (uint16_t) usToTicks(1000UL);
	if(channel == 0)
	{
		while(tc->COUNT16.SYNCBUSY.bit.CC0);
			}
	else if(channel == 1)
	{
		while(tc->COUNT16.SYNCBUSY.bit.CC1);
	}

	// Configure interrupt request
	// TODO this should be changed if more than one channel per timer is used by the Servo library
	NVIC_DisableIRQ(irqn);
	NVIC_ClearPendingIRQ(irqn);
	NVIC_SetPriority(irqn, 6);
	NVIC_EnableIRQ(irqn);

	// Enable the match channel interrupt request
	tc->COUNT16.INTENSET.reg = intEnableBit;

	// Enable the timer and start it
	tc->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
	while(tc->COUNT16.SYNCBUSY.bit.ENABLE);
}


static void initISR(timer16_Sequence_t timer)
{
	#if defined (_useTimer1)
		if (timer == _timer1)
		InitTimer(TC0, CHANNEL_FOR_TIMER1,  IRQn_FOR_TIMER1, GCM_FOR_TIMER_1, INTENSET_BIT_FOR_TIMER_1);
	#endif
	#if defined (_useTimer2)
		if (timer == _timer2)
		InitTimer(TC_FOR_TIMER2,  ID_TC_FOR_TIMER2, IRQn_FOR_TIMER2, GCM_FOR_TIMER_2, INTENSET_BIT_FOR_TIMER_2);
	#endif
}

static void finISR()
{
	#if defined (_useTimer1)
		// Disable the match channel interrupt request
		TC_FOR_TIMER1->COUNT16.INTENCLR.reg = INTENCLR_BIT_FOR_TIMER_1;
	#endif
		#if defined (_useTimer2)
	// Disable the match channel interrupt request
		TC_FOR_TIMER2->COUNT16.INTENCLR.reg = INTENCLR_BIT_FOR_TIMER_2;
	#endif
}

static bool isTimerActive(timer16_Sequence_t timer)
{
	// returns true if any servo is active on this timer
	for(uint8_t channel=0; channel < SERVOS_PER_TIMER; channel++) 
	{
		if(SERVO(timer,channel).Pin.isActive == true)
			return true;
	}
	return false;
}

// ================= end of static functions ====================================

// default constructor
CServo::CServo()
{
	if (ServoCount < MAX_SERVOS)
	{
		this->servoIndex = ServoCount++;                    // assign a servo index to this instance
		servos[this->servoIndex].ticks = usToTicks(DEFAULT_PULSE_WIDTH);   // store default values
	}
	else
	this->servoIndex = INVALID_SERVO;  // too many servos

} //CSerVo

// default destructor
CServo::~CServo()
{
} //~CSerVo


uint8_t CServo::Attach(EPortType port,int pin)
{
	return this->Attach(port,pin, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
}

uint8_t CServo::Attach(EPortType port,int pin, int min, int max)
{
	timer16_Sequence_t timer;

	if (this->servoIndex < MAX_SERVOS) 
	{
		PinPeripheral.SetPinMode(port,pin, OUTPUT);                                   // set servo pin to output
		servos[this->servoIndex].Pin.nbr = pin;
		servos[this->servoIndex].port = port;
																// todo min/max check: abs(min - MIN_PULSE_WIDTH) /4 < 128
//		this->min  = (MIN_PULSE_WIDTH - min)/4;					//resolution of min/max is 4 uS
//		this->max  = (MAX_PULSE_WIDTH - max)/4;
		
		this->min  =  min;					//resolution of min/max is 4 uS
		this->max  = max;


		timer = SERVO_INDEX_TO_TIMER(servoIndex);				// initialize the timer if it has not already been initialized
		if (isTimerActive(timer) == false) 
		{
			initISR(timer);
		}
		servos[this->servoIndex].Pin.isActive = true;			// this must be set after the check for isTimerActive
	}
	Write(1500);
	return this->servoIndex;
}

void CServo::Detach()
{
	timer16_Sequence_t timer;

	servos[this->servoIndex].Pin.isActive = false;
	timer = SERVO_INDEX_TO_TIMER(servoIndex);
	if(isTimerActive(timer) == false) 
	{
		finISR();
	}
}

void CServo::Write(int value)
{
	CMyMath Math;
	// treat values less than 544 as angles in degrees (valid values in microseconds are handled as microseconds)
	if (value < MIN_PULSE_WIDTH)
	{
		if (value < 0)
		value = 0;
		else if (value > 180)
		value = 180;

		value = Math.Map(value, 0, 180, SERVO_MIN(), SERVO_MAX());
	}
	WriteMicroseconds(value);
}

// Degree -4500 to 4500
void CServo::SetDegrees45(int Degree)
{
	CMyMath Math;
	uint16_t Pwm = Math.Map(Degree,-45,45,min,max);
	WriteMicroseconds(Pwm);
}


// Degree -4500 to 4500
void CServo::SetDegrees4500(int Degree)
{
	CMyMath Math;
	uint16_t Pwm = Math.Map(Degree,-4500,4500,min,max);
	WriteMicroseconds(Pwm);
}


// Degree -4500 to 4500
void CServo::SetDegrees90(int Degree)
{
	CMyMath Math;
	uint16_t Pwm = Math.Map(Degree,0,90,min,max);
	WriteMicroseconds(Pwm);
}



void CServo::WriteMicroseconds(int value)
{
	// calculate and store the values for the given channel
	CMyMath Math;
	uint8_t channel = this->servoIndex;
	if( (channel < MAX_SERVOS) )   // ensure channel is valid
	{
		Math.Constrain(value,SERVO_MIN(),SERVO_MAX());

		value = value - TRIM_DURATION;
		value = usToTicks(value);  // convert to ticks after compensating for interrupt overhead
		servos[channel].ticks = value;
	}
}

int CServo::Read() // return the value as degrees
{
	CMyMath Math;
	return Math.Map(ReadMicroseconds()+1, SERVO_MIN(), SERVO_MAX(), 0, 180);
}

int CServo::ReadMicroseconds()
{
	unsigned int pulsewidth;
	if (this->servoIndex != INVALID_SERVO)
	pulsewidth = ticksToUs(servos[this->servoIndex].ticks)  + TRIM_DURATION;
	else
	pulsewidth  = 0;

	return pulsewidth;
}

bool CServo::Attached()
{
	return servos[this->servoIndex].Pin.isActive;
}

