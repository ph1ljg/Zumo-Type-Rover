/* 
* Cid3.h
*
* Created: 08/08/2020 09:56:19
* Author: philg
*/


#ifndef __CID3_H__
#define __CID3_H__

  #define AUTO	1
  #define MANUAL	0


class Cid3
{
//variables
public:
   //scaled, tweaked parameters we'll actually be using
   float kc;                    // * (P)roportional Tuning Parameter
   float taur;                  // * (I)ntegral Tuning Parameter
   float taud;                  // * (D)erivative Tuning Parameter

   float cof_A;
   float cof_B;
   float cof_C;

   //nice, pretty parameters we'll give back to the user if they ask what the tunings are
   float P_Param;
   float I_Param;
   float D_Param;


   int *myInput;				// * Pointers to the Input, Output, and Setpoint variables
   int *myOutput;				//   This creates a hard link between the variables and the
   int *mySetpoint;			//   PID, freeing the user from having to constantly tell us
   //   what these values are.  with pointers we'll just know.

   int *myBias;				// * Pointer to the External FeedForward bias, only used
   //   if the advanced constructor is used
   bool UsingFeedForward;		// * internal flag that tells us if we're using FeedForward or not

   unsigned long nextCompTime;    // * Helps us figure out when the PID Calculation needs to
   //   be performed next
   //   to determine when to compute next
   unsigned long tSample;       // * the frequency, in milliseconds, with which we want the
   //   the PID calculation to occur.
   bool inAuto;                  // * Flag letting us know if we are in Automatic or not

   //   the derivative required for the D term
   //float accError;              // * the (I)ntegral term is based on the sum of error over
   //   time.  this variable keeps track of that
   float bias;                  // * the base output from which the PID operates
   
   int Err;
   int lastErr;
   int prevErr;
   
   float inMin, inSpan;         // * input and output limits, and spans.  used convert
   float outMin, outSpan;       //   real world numbers into percent span, with which
   //   the PID algorithm is more comfortable.

   bool justCalced;			// * flag gets set for one cycle after the pid calculates

protected:
private:

//functions
public:
	Cid3();
	Cid3(int *Input, int *Output, int *Setpoint, float Kc, float TauI, float TauD);
	Cid3(int *Input, int *Output, int *Setpoint, int *FFBias, float Kc, float TauI, float TauD);
	void ConstructorCommon(int *Input, int *Output, int *Setpoint, float Kc, float TauI, float TauD);
	void SetInputLimits(int INMin, int INMax);
	void SetOutputLimits(int OUTMin, int OUTMax);
	void SetTunings(float Kc, float TauI, float TauD);
	void Reset();
	void SetMode(int Mode);
	void SetSampleTime(int NewSampleTime);
	void Compute();
	~Cid3();
protected:
private:
	Cid3( const Cid3 &c );
	Cid3& operator=( const Cid3 &c );

}; //Cid3

#endif //__CID3_H__
