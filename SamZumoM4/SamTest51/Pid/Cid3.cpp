/* 
* Cid3.cpp
*
* Created: 08/08/2020 09:56:19
* Author: philg
*/

#include "Includes.h"
#include "Cid3.h"

// default constructor
Cid3::Cid3()
{
} //Cid3

// default destructor
Cid3::~Cid3()
{
} //~Cid3



Cid3::Cid3(int *Input, int *Output, int *Setpoint, float Kc, float TauI, float TauD)
{

  ConstructorCommon(Input, Output, Setpoint, Kc, TauI, TauD);  
  UsingFeedForward = false;
  Reset();

  
}

//  implement Feed Forward Control. 
Cid3::Cid3(int *Input, int *Output, int *Setpoint, int *FFBias, float Kc, float TauI, float TauD)
{

  ConstructorCommon(Input, Output, Setpoint, Kc, TauI, TauD);  
  UsingFeedForward = true;										// Set the controller to use an external bias
  myBias = FFBias;                              
  Reset();
}

void Cid3::ConstructorCommon(int *Input, int *Output, int *Setpoint, float Kc, float TauI, float TauD)
{
  SetInputLimits(0, 0);		
  SetOutputLimits(0, 0);		

  tSample = 1000;			//default Controller Sample Time is 1 second

  SetTunings( Kc, TauI, TauD);

  nextCompTime = Core.millis();
  inAuto = false;
  myOutput = Output;
  myInput = Input;
  mySetpoint = Setpoint;

  Err = lastErr = prevErr = 0;
}

void Cid3::SetInputLimits(int INMin, int INMax)
{
	if(INMin >= INMax) return;		//after verifying that mins are smaller than maxes, set the values
		inMin = INMin;
	inSpan = INMax - INMin;
}


void Cid3::SetOutputLimits(int OUTMin, int OUTMax)
{
	if(OUTMin >= OUTMax)				//after verifying that mins are smaller than maxes, set the values 
		return;

	outMin = OUTMin;
	outSpan = OUTMax - OUTMin;
}


void Cid3::SetTunings(float Kc, float TauI, float TauD)
{
	
	if (Kc == 0.0 || TauI < 0.0 || TauD < 0.0)		// verify that the tunings make sense
		return;

	P_Param = Kc;				// store inputs
	I_Param = TauI;
	D_Param = TauD;

	
	float tSampleInSec = ((float)tSample / 1000.0);	// convert Reset Time into Reset Rate, and compensate for Calculation frequency
	float tempTauR;
	if (TauI == 0.0) 
		tempTauR = 0.0;
	else 
		tempTauR = (1.0 / TauI) * tSampleInSec;

	
	kc = Kc;
	taur = tempTauR;
	taud = TauD / tSampleInSec;

	cof_A = kc * (1 + taur + taud);
	cof_B = kc * (1 + 2 * taud);
	cof_C = kc * taud;
}

void Cid3::Reset()
{
	if(UsingFeedForward)
	  bias = (*myBias - outMin) / outSpan;
	else
	  bias = (*myOutput - outMin) / outSpan;

}

void Cid3::SetMode(int Mode)
{
	if (Mode!=0 && !inAuto)
	{	
		Cid3::Reset();			//Was in manual, and  just got set to auto. Reset the controller internals
	}
	inAuto = (Mode!=0);
}

//sets the frequency, in Milliseconds, with which the Cid3 calculation is performed	
void Cid3::SetSampleTime(int NewSampleTime)
{
	if (NewSampleTime > 0)
	{ 
		taur *= ((float)NewSampleTime)/((float) tSample);		//convert the time-based tunings to reflect this change
		taud *= ((float)NewSampleTime)/((float) tSample);
		tSample = (unsigned long)NewSampleTime;

		cof_A = kc * (1 + taur + taud);
		cof_B = kc * (1 + 2 * taud);
		cof_C = kc * taud;
	}
}

// tunings.  lock in the I and D, and then just vary P to get more  aggressive or conservative
// when tunings are entered the I term is converted to Reset Rate. this is merely to avoid 
// the div0 error when the  Integral action  is turned off.
//     
//  Derivative on Measurement is being used instead of Derivative on Error.  The
// performance is identical, with one notable exception.  DonE causes a kick in
// the controller output whenever there's a setpoint change. DonM does not.

void Cid3::Compute()
{
	justCalced=false;
	if (!inAuto) 
		return;						// if in manual just leave; 

	unsigned long now = Core.millis();	// millis() wraps around to 0 at some point this is not currently addressed by this algorithm.
									
	
	if (now>=nextCompTime)			// Perform Cid3 Computations if it's time					
	{
		Err = *mySetpoint - *myInput;
		if(UsingFeedForward)		//if  using an external bias (i.e. the user used the overloaded constructor,) then pull that in now
		{
			bias = *myBias - outMin;
		}
		int output = bias + (cof_A * Err - cof_B * lastErr + cof_C * prevErr);

		
		if (output < -outSpan)		// Make sure the computed output is within output constraints
			output = -outSpan;		
		else if (output > outSpan) 
			output = outSpan;
		
		prevErr = lastErr;
		lastErr = Err;

		*myOutput = output;			// Scale the output from percent span back out to a real world number

		nextCompTime += tSample;				// determine the next time the computation
		if(nextCompTime < now) nextCompTime = now + tSample;	// should be performed	

		justCalced=true;  //set the flag that will tell the outside world that the output was just computed

	}								

}


