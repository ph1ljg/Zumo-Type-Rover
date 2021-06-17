/* 
* CFilters.cpp
*
* Created: 31/12/2016 12:39:10
* Author: phil
*/
#include "stdio.h"
#include "cmymath.h"
#include "Vector2.h"
#include "CFilters.h"

// default constructor
CFilters::CFilters()
{
} //CFilters

// default destructor
CFilters::~CFilters()
{
} //~CFilters




//================================================ Mode Filter ==================================================================
// Mode filter which is basically picking the median value from the last x samples
// buffer should be odd size limited to 255 samples

int16_t CFilters::ModeFilterApplyI(int16_t NewSample,ModeFilterI_t *Modefilter)
{
	unsigned char i;
	
	if( Modefilter->SampleIndex < Modefilter->BuffSize )				// if the buffer isn't full simply increase the No items in the buffer
	{																	// This is the same as dropping the high sample
		Modefilter->SampleIndex++;
		Modefilter->DropHighSample = true;
	}
	if( Modefilter->DropHighSample )									// drop highest sample from the buffer to make room for our new sample
	{
		i = Modefilter->SampleIndex-1;									// start from top. Note: sample_index always points to the next open space so we start from sample_index-1
		while( Modefilter->Buffer[i-1] > NewSample && i > 0 )			// if the next element is higher than our new sample, push it up one position
		{
			Modefilter->Buffer[i] = Modefilter->Buffer[i-1];
			i--;
		}
		Modefilter->Buffer[i] = NewSample;								// add our new sample to the buffer
	}
	else																// drop lowest sample from the buffer to make room for our new sample
	{
		i = 0;															// start from the bottom
		while( Modefilter->Buffer[i+1] < NewSample && i < Modefilter->SampleIndex-1 )// if the element is lower than our new sample, push it down one position
		{
			Modefilter->Buffer[i] = Modefilter->Buffer[i+1];
			i++;
		}
		Modefilter->Buffer[i] = NewSample;								// add our new sample to the buffer
	}

	Modefilter->DropHighSample = !Modefilter->DropHighSample;			// next time drop from the other end of the sample buffer
	return  Modefilter->Buffer[( Modefilter->BuffSize / 2)];		// middle sample
}


//================================================ Mode Filter ==================================================================
// Mode filter which is basically picking the median value from the last x samples
// buffer should be odd size limited to 255 samples

float CFilters::ModeFilterApplyF(float NewSample,ModeFilterF_t *Modefilter)
{
	unsigned char i;
	
	if( Modefilter->SampleIndex < Modefilter->BuffSize )				// if the buffer isn't full simply increase the No items in the buffer
	{																	// This is the same as dropping the high sample
		Modefilter->SampleIndex++;
		Modefilter->DropHighSample = true;
	}
	if( Modefilter->DropHighSample )									// drop highest sample from the buffer to make room for our new sample
	{
		i = Modefilter->SampleIndex-1;									// start from top. Note: sample_index always points to the next open space so we start from sample_index-1
		while( Modefilter->Buffer[i-1] > NewSample && i > 0 )			// if the next element is higher than our new sample, push it up one position
		{
			Modefilter->Buffer[i] = Modefilter->Buffer[i-1];
			i--;
		}
		Modefilter->Buffer[i] = NewSample;								// add our new sample to the buffer
	}
	else																// drop lowest sample from the buffer to make room for our new sample
	{
		i = 0;															// start from the bottom
		while( Modefilter->Buffer[i+1] < NewSample && i < Modefilter->SampleIndex-1 )// if the element is lower than our new sample, push it down one position
		{
			Modefilter->Buffer[i] = Modefilter->Buffer[i+1];
			i++;
		}
		Modefilter->Buffer[i] = NewSample;								// add our new sample to the buffer
	}

	Modefilter->DropHighSample = !Modefilter->DropHighSample;			// next time drop from the other end of the sample buffer
	return  Modefilter->Buffer[( Modefilter->BuffSize / 2)];		// middle sample
}


// 	int sensVal;           // for raw sensor values 
// 	float filterVal;       // this determines smoothness  - .0001 is max  1 is off (no smoothing)
// 	float smoothedVal;     // this holds the last loop value just use a unique variable for every instance
int CFilters::Smooth(int data, float filterVal, float smoothedVal)
{

  if (filterVal > 1)		// check to make sure param's are within range
    filterVal = .99;
  else if (filterVal <= 0)
    filterVal = 0;

  smoothedVal = (data * (1 - filterVal)) + (smoothedVal  *  filterVal);
  return (int)smoothedVal;
}



float CFilters::Smooth2(float x)  
{
	static float v[2];

	v[0] = v[1];
	v[1] = (0.8 * x)	+ (0.2 * v[0]);
	return 	(v[0] + v[1]);
}


uint16_t CFilters::Median5Filter(uint16_t * p)
{
	VALUE_SORT(p[0],p[1]) ; VALUE_SORT(p[3],p[4]) ; VALUE_SORT(p[0],p[3]) ;
	VALUE_SORT(p[1],p[4]) ; VALUE_SORT(p[1],p[2]) ; VALUE_SORT(p[2],p[3]) ;
	VALUE_SORT(p[1],p[2]) ; return(p[2]) ;
}
//FILT <-- FILT + FF(NEW - FILT)


