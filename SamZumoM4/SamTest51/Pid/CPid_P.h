/* 
* CPid_P.h
*
* Created: 14/08/2020 09:58:23
* Author: philg
*/


#ifndef __CPID_P_H__
#define __CPID_P_H__


class CPid_P
{
//variables
public:
protected:
private:
	 float m_kp;
//functions
public:
	CPid_P(const float& initial_p = 0.0f);
	~CPid_P();
	float Get_p(float error) const;
	void Set_p(float Kp) ;
	// Overload the function call operator to permit relatively easy initialization
	void operator() (const float p) { m_kp = p; }

protected:
private:
	CPid_P( const CPid_P &c );
	CPid_P& operator=( const CPid_P &c );

}; //CPid_P

#endif //__CPID_P_H__
