/*
* CQuatToDcm.h
*
* Created: 25/02/2021 17:43:00
* Author: philg
*/


#ifndef __CQUATTODCM_H__
#define __CQUATTODCM_H__


class CQuatToDcm
{
	//variables
public:
protected:
private:

	//functions
public:
	CQuatToDcm();
	~CQuatToDcm();
	 void Convert(float Quat[], Matrix3f& DCM);
protected:
private:
	CQuatToDcm(const CQuatToDcm& c);
	CQuatToDcm& operator=(const CQuatToDcm& c);

}; //CQuatToDcm

#endif //__CQUATTODCM_H__

