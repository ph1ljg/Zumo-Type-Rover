/* 
* CPwm.h
*
* Created: 18/05/2020 19:19:07
* Author: philg
*/


#ifndef __CSERVO_H__
#define __CSERVO_H__

// Different pre scalers depending on FCPU (avoid overflowing 16-bit counter)
#if(F_CPU > 200000000)
#define usToTicks(_us)    ((clockCyclesPerMicrosecond() * _us) / 128)
#define ticksToUs(_ticks) (((unsigned) _ticks * 128) / clockCyclesPerMicrosecond())
#else
#define usToTicks(_us)    ((clockCyclesPerMicrosecond() * _us) / 64)
#define ticksToUs(_ticks) (((unsigned) _ticks * 64) / clockCyclesPerMicrosecond())
#endif

#define TRIM_DURATION  5                                   // compensation ticks to trim adjust for digitalWrite delays

 // convenience macros
 #define SERVO_INDEX_TO_TIMER(_servo_nbr) ((timer16_Sequence_t)(_servo_nbr / SERVOS_PER_TIMER))   // returns the timer controlling this servo
 #define SERVO_INDEX_TO_CHANNEL(_servo_nbr) (_servo_nbr % SERVOS_PER_TIMER)                       // returns the index of the servo on this timer
 #define SERVO_INDEX(_timer,_channel)  ((_timer*SERVOS_PER_TIMER) + _channel)                     // macro to access servo index by timer and channel
 #define SERVO(_timer,_channel)  (servos[SERVO_INDEX(_timer,_channel)])                           // macro to access servo class by timer and channel

 #define SERVO_MIN() (MIN_PULSE_WIDTH - this->min * 4)   // minimum value in uS for this servo
 #define SERVO_MAX() (MAX_PULSE_WIDTH - this->max * 4)   // maximum value in uS for this servo

 // Referenced in SAMD21 code only, no harm in defining regardless
 //#define WAIT_TC16_REGS_SYNC(x) while(x->COUNT16.STATUS.bit.SYNCBUSY);







#define _useTimer1

#if defined (_useTimer1)
   #define TC_FOR_TIMER1             TC0
   #define CHANNEL_FOR_TIMER1        0
   #define INTENSET_BIT_FOR_TIMER_1  TC_INTENSET_MC0
   #define INTENCLR_BIT_FOR_TIMER_1  TC_INTENCLR_MC0
   #define INTFLAG_BIT_FOR_TIMER_1   TC_INTFLAG_MC0
   #define ID_TC_FOR_TIMER1          ID_TC0
   #define IRQn_FOR_TIMER1           TC0_IRQn
   #define HANDLER_FOR_TIMER1        TC0_Handler
   #define GCM_FOR_TIMER_1           9 // GCLK_TC0
#endif
#if defined (_useTimer2)
   #define TC_FOR_TIMER2             TC0
   #define CHANNEL_FOR_TIMER2        1
   #define INTENSET_BIT_FOR_TIMER_2  TC_INTENSET_MC1
   #define INTENCLR_BIT_FOR_TIMER_2  TC_INTENCLR_MC1
   #define INTFLAG_BIT_FOR_TIMER_2   TC_INTFLAG_MC1
   #define ID_TC_FOR_TIMER2          ID_TC0
   #define IRQn_FOR_TIMER2           TC0_IRQn
   #define HANDLER_FOR_TIMER2        TC0_Handler
   #define GCM_FOR_TIMER_2           9 // GCLK_TC0
#endif
   
typedef enum 
{
	#if defined (_useTimer1)
		_timer1,
	#endif
	#if defined (_useTimer2)
		_timer2,
	#endif
	_Nbr_16timers 
} timer16_Sequence_t;


// Different prescalers depending on FCPU (avoid overflowing 16-bit counter)
#if(F_CPU > 200000000)
#define usToTicks(_us)    ((clockCyclesPerMicrosecond() * _us) / 128)
#define ticksToUs(_ticks) (((unsigned) _ticks * 128) / clockCyclesPerMicrosecond())
#else
#define usToTicks(_us)    ((clockCyclesPerMicrosecond() * _us) / 64)
#define ticksToUs(_ticks) (((unsigned) _ticks * 64) / clockCyclesPerMicrosecond())
#endif
#else
#define usToTicks(_us)    ((clockCyclesPerMicrosecond() * _us) / 16)                 // converts microseconds to tick
#define ticksToUs(_ticks) (((unsigned) _ticks * 16) / clockCyclesPerMicrosecond())   // converts from ticks back to microseconds
#endif

#define TRIM_DURATION  5                                   // compensation ticks to trim adjust for digitalWrite delays

// convenience macros
#define SERVO_INDEX_TO_TIMER(_servo_nbr) ((timer16_Sequence_t)(_servo_nbr / SERVOS_PER_TIMER))   // returns the timer controlling this servo
#define SERVO_INDEX_TO_CHANNEL(_servo_nbr) (_servo_nbr % SERVOS_PER_TIMER)                       // returns the index of the servo on this timer
#define SERVO_INDEX(_timer,_channel)  ((_timer*SERVOS_PER_TIMER) + _channel)                     // macro to access servo index by timer and channel
#define SERVO(_timer,_channel)  (servos[SERVO_INDEX(_timer,_channel)])                           // macro to access servo class by timer and channel

#define SERVO_MIN() (MIN_PULSE_WIDTH - this->min * 4)   // minimum value in uS for this servo
#define SERVO_MAX() (MAX_PULSE_WIDTH - this->max * 4)   // maximum value in uS for this servo

// Referenced in SAMD21 code only, no harm in defining regardless
#define WAIT_TC16_REGS_SYNC(x) while(x->COUNT16.STATUS.bit.SYNCBUSY);

#define MIN_PULSE_WIDTH       544     // the shortest pulse sent to a servo
#define MAX_PULSE_WIDTH      2400     // the longest pulse sent to a servo
#define DEFAULT_PULSE_WIDTH  1500     // default pulse width when servo is attached
#define REFRESH_INTERVAL    20000     // minumim time to refresh servos in microseconds

#define SERVOS_PER_TIMER       12     // the maximum number of servos controlled by one timer
#define MAX_SERVOS   (_Nbr_16timers  * SERVOS_PER_TIMER)

#define INVALID_SERVO         255     // flag indicating an invalid servo index

#if !defined(ARDUINO_ARCH_STM32F4)

typedef struct  
{
	uint8_t nbr        :6 ;             // a pin number from 0 to 63
	uint8_t isActive   :1 ;             // true if this channel is enabled, pin not pulsed if false
} ServoPin_t   ;

typedef struct 
{
	EPortType port;
	ServoPin_t Pin;
	volatile unsigned int ticks;
} servo_t;




class CServo
{
//variables
public:
protected:
private:
   uint8_t servoIndex;               // index into the channel data for this servo
   int16_t min;                       // minimum is this value times 4 added to MIN_PULSE_WIDTH
   int16_t max;                       // maximum is this value times 4 added to MAX_PULSE_WIDTH


//functions
public:
	CServo();
	~CServo();
	uint8_t Attach(EPortType port,int pin);
	uint8_t Attach(EPortType port,int pin, int min, int max);
	void Detach();
	void Write(int value);
	void SetDegrees45(int Degree);
	void SetDegrees4500(int Degree);
	void SetDegrees90(int Degree);
	void WriteMicroseconds(int value);
	int Read(); // return the value as degrees
	int ReadMicroseconds();
	bool Attached();
protected:
private:
	CServo( const CServo &c );
	CServo& operator=( const CServo &c );

}; //CPwm
#endif //__CSERVO_H__
