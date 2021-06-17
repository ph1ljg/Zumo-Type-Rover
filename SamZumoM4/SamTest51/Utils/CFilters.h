/* 
* CFilters.h
*
* Created: 31/12/2016 12:39:11
* Author: phil
*/


#ifndef __CFILTERS_H__
#define __CFILTERS_H__

#include "cmymath.h"
#define VALUE_SORT(a,b) { if ((a)>(b)) VALUE_SWAP((a),(b)); }
#define VALUE_SWAP(a,b) { uint16_t temp=(a);(a)=(b);(b)=temp; }


typedef struct
{
	float *Buffer;
	unsigned char BuffSize;
	unsigned char SampleIndex;
	bool DropHighSample;
}ModeFilterF_t;

typedef struct
{
	int16_t *Buffer;
	unsigned char BuffSize;
	unsigned char SampleIndex;
	bool DropHighSample;
}ModeFilterI_t;


class CFilters
{
//variables
public:
protected:
private:

//functions
public:
	CFilters();
	~CFilters();
	int16_t ModeFilterApplyI(int16_t NewSample,ModeFilterI_t *Modefilter);
	float ModeFilterApplyF(float NewSample,ModeFilterF_t *Modefilter);
	int Smooth(int data, float filterVal, float smoothedVal);
	float Smooth2(float x);

	uint16_t Median5Filter(uint16_t * p);
protected:
private:
	CFilters( const CFilters &c );
	CFilters& operator=( const CFilters &c );

}; //CFilters



//MovingAverage<unsigned int, 3> average;
template <typename V, int N> class MovingAverage
{
public:
     // param def the default value to initialize the average.
    MovingAverage(V def = 0) : sum(0), p(0)
    {
        for (int i = 0; i < N; i++) 
		{
            samples[i] = def;
            sum += samples[i];
        }
    }

//   Add a new sample.
//   param new_sample the new sample to add to the moving average.
//   return the updated average.
    V add(V new_sample)
    {
        sum = sum - samples[p] + new_sample;
        samples[p++] = new_sample;
        if (p >= N)
            p = 0;
        return sum / N;
    }

private:
    V samples[N];
    V sum;
    V p;
};



typedef  MovingAverage<unsigned int, 3> CMovingAverage;






#endif //__CFILTERS_H__
