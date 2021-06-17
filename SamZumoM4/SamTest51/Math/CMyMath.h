/* 
* CMyMath.h
*
* Created: 19/05/2020 09:42:36
* Author: philg
*/


#ifndef __CMYMATH_H__
#define __CMYMATH_H__

#include "float.h"
#include <math.h>
#include "stdint.h"
#include <limits>
#include "type_traits"
#include "assert.h"
#include <type_traits>




template <typename T, size_t N> char(&_ARRAY_SIZE_HELPER(T(&_arr)[N]))[N];

template <typename T> char(&_ARRAY_SIZE_HELPER(T(&_arr)[0]))[0];

#define ARRAY_SIZE(_arr) sizeof(_ARRAY_SIZE_HELPER(_arr))




#define Max(a, b)           (((a) > (b)) ?  (a) : (b))
#define Min(a, b)           (((a) < (b)) ?  (a) : (b))
#define min(a, b)   Min(a, b)
#define max(a, b)   Max(a, b)

#define GRAVITY_MSS     9.80665f	// 96.10517 m/2/2
#define BSWAP_16(x)  ((((x) >> 8) & 0xff) | (((x) & 0xff) << 8))

enum VectorNames
{
	ROLL,
	PITCH,
	YAW,
};

//#define M_PI      (3.141592653589793f)

#define M_PIf  3.14159265358979323846f
#define PI 3.1415926535897932384626433832795
#define HALF_PI 1.5707963267948966192313216916398
#define TWO_PI 6.283185307179586476925286766559
//#define DEG_TO_RAD 0.017453292519943295769236907684886
//#define RAD_TO_DEG 57.295779513082320876798154814105
#define RAD    (M_PIf / 180.0f)
#define DEG_TO_RAD      0.01745329251994329
#define RAD_TO_DEG      (180.0f / M_PI)


#define M_GOLDEN  1.6180339f

#define M_2PI         (M_PI * 2)




#define DEGREES_TO_DECIDEGREES(angle) (angle * 10)
#define DECIDEGREES_TO_DEGREES(angle) (angle / 10)
#define DECIDEGREES_TO_RADIANS(angle) ((angle / 10.0f) * 0.0174532925f)
#define DEGREES_TO_RADIANS(angle) ((angle) * 0.0174532925f)
#define RADIANS_TO_DECIDEGREES(angle) (((angle) * 10.0f) / RAD)

#define atan2_approx(y,x)   atan2f(y,x)

#define MIN(a,b) \
__extension__ ({ __typeof__ (a) _a = (a); \
	__typeof__ (b) _b = (b); \
_a < _b ? _a : _b; })
#define MAX(a,b) \
__extension__ ({ __typeof__ (a) _a = (a); \
	__typeof__ (b) _b = (b); \
_a > _b ? _a : _b; })
#define ABS(x) \
__extension__ ({ __typeof__ (x) _x = (x); \
_x > 0 ? _x : -_x; })


//#define min(a,b) ((a)<(b)?(a):(b))
//#define max(a,b) ((a)>(b)?(a):(b))
#define radians(deg) ((deg)*DEG_TO_RAD)
#define degrees(rad) ((rad)*RAD_TO_DEG)
#define square(x) ((x)*(x))
//#define constrain(amt,low,high) ((amt<low) ?low :((amt>high) ? high :amt))






typedef struct
{
	int32_t X,Y,Z;
} Vector32_t;

typedef struct
{
	uint16_t XL;
	int16_t X;
	uint16_t YL;
	int16_t Y;
	uint16_t ZL;
	int16_t Z;
} Vector16_t;

// note: we use implicit first 16 MSB bits 32 -> 16 cast. ie V32.X>>16 = V16.X
typedef union
{
	int32_t A32[3];
	Vector32_t V32;
	int16_t A16[6];
	Vector16_t V16;
} Vector32U_t;





template <typename T> struct vectorT
{
	T x;
	T y;
	T z;
};

typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
}Axis_t;


typedef union
{
	Axis_t Axis;
	int16_t Values[3];
}vector;


typedef struct
{
	float x;
	float y;
	float z;
}Axisf_t;


typedef union
{
	Axisf_t Axis;
	float Values[3];
}vectorf;



typedef struct
{
	int16_t x;
	int16_t y;
	int16_t z;
}VectorI_t;



typedef struct
{
	float x,y,z;
} VectorF_t;

typedef struct
{
	long x, y, z;
} VectorL_t;

/* 
 * @brief: Check whether a float is zero
 */
template <class T> inline bool is_zero(const T fVal1) 
{
    static_assert(std::is_floating_point<T>::value || std::is_base_of<T,float>::value,
                  "Template parameter not of type float");
    return fabsf(static_cast<float>(fVal1)) < FLT_EPSILON ? true : false;
}


template <class FloatOne, class FloatTwo> bool is_equal(const FloatOne v_1, const FloatTwo v_2)
{
	return fabsf(v_1 - v_2) < std::numeric_limits<decltype(v_1 - v_2)>::epsilon();
}

template bool is_equal<int>(const int v_1, const int v_2);
template bool is_equal<short>(const short v_1, const short v_2);
template bool is_equal<float>(const float v_1, const float v_2);
template bool is_equal<double>(const double v_1, const double v_2);


/*
 * A variant of asin() that checks the input ranges and ensures a valid angle
 * as output. If nan is given as input then zero is returned.
 */
template <class T> float safe_asin(const T v);


//  Constrain an euler angle to be within the range: 0 to 360 degrees. The
//  second parameter changes the units. Default: 1 == degrees, 10 == deci,
//  100 == centi.
template <class T> float wrap_360(const T angle, float unit_mod = 1);
long wrap_360_cd(const long angle);
template <typename T> T wrap_180(const T angle);
template <typename T>	float wrap_PI(const T radian);
template <typename T>	T  wrap_180_cd(const T angle);
template <typename T> float wrap_2PI(const T radian);
double safe_sqrt(double x);
template <class T> T constrain_value(const T amt, const T low, const T high);

template<class T> float sq(const T val)
{
	return pow(static_cast<float>(val), 2);
}


 template<class T>  T safe_sqrt( T x )
{
	if( x <= 0 ) 
		return 0;
	return sqrt(x);
}


/*
 * Variadic template for calculating the square norm of a vector of any
 * dimension.
 */
template<class T, class... Params> float sq(const T first, const Params... parameters)
{
    return sq(first) + sq(parameters...);
}


// Variadic template for calculating the norm (pythagoras) of a vector of any dimension.
template<class T, class... Params>
float norm(const T first, const Params... parameters)
{
    return sqrt(static_cast<float>(sq(first, parameters...)));
}


template <typename T>
inline bool is_positive(const T fVal1) {
	static_assert(std::is_floating_point<T>::value || std::is_base_of<T,float>::value,
	"Template parameter not of type float");
	return (static_cast<float>(fVal1) >= FLT_EPSILON);
}

template <typename T>
inline bool is_negative(const T fVal1) {
	static_assert(std::is_floating_point<T>::value || std::is_base_of<T,float>::value,
	"Template parameter not of type float");
	return (static_cast<float>(fVal1) <= (-1.0 * FLT_EPSILON));
}



class CMyMath
{
//variables
public:
protected:
private:

//functions
public:
	CMyMath();
	~CMyMath();

	long  Map(long x, long in_min, long in_max, long out_min, long out_max);
	int   Constrain(int amt, int low, int high);
	float LinearInterpolate(float low_output, float high_output,float var_value, float var_low, float var_high);
	double Clip(double n, double minValue, double maxValue);
	float calc_lowpass_alpha_dt(float dt, float cutoff_freq);
	void  VectorRotate(vectorf &RotateVector,float* delta);
	void VectorRotateV32( Vector32U_t *v,int16_t* delta);
	void  VectorCrossI(const vector &a, const vector &b, vector &out);
	void  VectorCrossF(const vectorf &a, const vectorf &b, vectorf &out);
	float VectorDotI(const vector &a, const vector &b);
	float VectorDotF(const vectorf &a, const vectorf &b);
	void  VectorNormalizeI(vector &a);
	void  VectorNormalizeF(vectorf &a);
	float InvSqrt (float x);
	//	int32_t Round(double number);
	int32_t Round(double x);
	float Roundf_2(float var);
	double RoundF(double Value);
	float Roundf2(float var);

	int16_t _atan2(int32_t y, int32_t x);
	uint16_t KnotsToMotorMs(uint16_t Knots);
	int16_t MetersSecToKnots(int16_t MetersSec);
	int16_t CentMetersSecToKnots(int16_t CmMetersSec);

	//	template <typename Ta, typename Tb, typename To> void vector_cross(const vector<Ta> *a, const vector<Tb> *b, vector<To> *out);
	//	template <typename Ta, typename Tb> float vector_dot(const vector<Ta> *a, const vector<Tb> *b);
	void vector_normalize(vectorf &a);



//	template <class T> T ConstrainValue(const T amt, const T low, const T high);

	template<typename T>T ConstrainValue(T Value, T Min, T Max)
	{
		return (Value < Min)? Min : (Value > Max)? Max : Value;
	}

	inline float constrain_float(const float amt, const float low, const float high)
	{
		return ConstrainValue(amt, low, high);
	}

	inline int16_t constrain_int16(const int16_t amt, const int16_t low, const int16_t high)
	{
		return ConstrainValue(amt, low, high);
	}

	inline int32_t constrain_int32(const int32_t amt, const int32_t low, const int32_t high)
	{
		return ConstrainValue(amt, low, high);
	}

	// degrees -> radians
// 	inline  float radians(float deg)
// 	{
// 		return deg * DEG_TO_RAD;
// 	}

	// radians -> degrees
// 	inline constexpr float degrees(float rad)
// 	{
// 		return rad * RAD_TO_DEG;
// 	}



	void vector_cross(vectorf &a, const vectorf &b, vectorf &out)
	{
		out.Axis.x = (a.Axis.y * b.Axis.z) - (a.Axis.z * b.Axis.y);
		out.Axis.y = (a.Axis.z * b.Axis.x) - (a.Axis.x * b.Axis.z);
		out.Axis.z = (a.Axis.x * b.Axis.y) - (a.Axis.y * b.Axis.x);
	}

	float vector_dot(const vectorf &a, const vectorf &b)
	{
		return (a.Axis.x * b.Axis.x) + (a.Axis.y * b.Axis.y) + (a.Axis.z * b.Axis.z);
	}
	
	
	// Check whether a float is zero
	template <typename T>
	inline bool is_zero(const T fVal1)
	{
		return (fabsf(static_cast<float>(fVal1)) < FLT_EPSILON);
	}

	// Check whether a float is greater than zero
	template <typename T>
	inline bool is_positive(const T fVal1)
	{
		return (static_cast<float>(fVal1) >= FLT_EPSILON);
	}


	// Check whether a float is less than zero
	template <typename T>	inline bool is_negative(const T fVal1)
	{
		return (static_cast<float>(fVal1) <= (-1.0 * FLT_EPSILON));
	}
	
	long wrap_360_cd(const long angle)
	{
		long res = angle % 36000;
		if (res < 0) 
		{
			res += 36000;
		}
		return res;
	}

	
	
	float wrap_360_cd(const float angle)
	{
		float res = fmodf(angle, 36000.0f);
		if (res < 0) {
			res += 36000.0f;
		}
		return res;
	}
	
	int wrap_360_cd(const int angle)
	{
		int res = angle % 36000;
		if (res < 0) {
			res += 36000;
		}
		return res;
	}


protected:
private:
	CMyMath( const CMyMath &c );
	CMyMath& operator=( const CMyMath &c );

}; //CMyMath

/*

template <typename T>
struct vector2
{
    T x, y;

    // trivial ctor
    constexpr vector2<T>()
        : x(0)
        , y(0) {}

    // setting ctor
    constexpr vector2<T>(const T x0, const T y0)
        : x(x0)
        , y(y0) {}

    // function call operator
    void operator ()(const T x0, const T y0)
    {
        x= x0; y= y0;
    }

    // test for equality
    bool operator ==(const vector2<T> &v) const;

    // test for inequality
    bool operator !=(const vector2<T> &v) const;

    // negation
    vector2<T> operator -(void) const;

    // addition
    vector2<T> operator +(const vector2<T> &v) const;

    // subtraction
    vector2<T> operator -(const vector2<T> &v) const;

    // uniform scaling
    vector2<T> operator *(const T num) const;

    // uniform scaling
    vector2<T> operator  /(const T num) const;

    // addition
    vector2<T> &operator +=(const vector2<T> &v);

    // subtraction
    vector2<T> &operator -=(const vector2<T> &v);

    // uniform scaling
    vector2<T> &operator *=(const T num);

    // uniform scaling
    vector2<T> &operator /=(const T num);

    // dot product
    T operator *(const vector2<T> &v) const;

    // cross product
    T operator %(const vector2<T> &v) const;

    // computes the angle between this vector and another vector
    // returns 0 if the vectors are parallel, and M_PI if they are antiparallel
    float angle(const vector2<T> &v2) const;

    // check if any elements are NAN
    bool is_nan(void) const;

    // check if any elements are infinity
    bool is_inf(void) const;

    // check if all elements are zero
    bool is_zero(void) const { return (fabsf(x) < FLT_EPSILON) && (fabsf(y) < FLT_EPSILON); }

    // allow a vector2 to be used as an array, 0 indexed
    T & operator[](uint8_t i) {
        T *_v = &x;
#if MATH_CHECK_INDEXES
        assert(i >= 0 && i < 2);
#endif
        return _v[i];
    }

    const T & operator[](uint8_t i) const {
        const T *_v = &x;
#if MATH_CHECK_INDEXES
        assert(i >= 0 && i < 2);
#endif
        return _v[i];
    }
    
    // zero the vector
    void zero()
    {
        x = y = 0;
    }

    // gets the length of this vector squared
    T   length_squared() const
    {
        return (T)(*this * *this);
    }

    // gets the length of this vector
    float           length(void) const;

    // normalizes this vector
    void    normalize()
    {
        *this/=length();
    }

    // returns the normalized vector
    vector2<T>  normalized() const
    {
        return *this/length();
    }

    // reflects this vector about n
    void    reflect(const vector2<T> &n)
    {
        const vector2<T>        orig(*this);
        project(n);
        *this= *this*2 - orig;
    }

    // projects this vector onto v
    void    project(const vector2<T> &v)
    {
        *this= v * (*this * v)/(v*v);
    }

    // returns this vector projected onto v
    vector2<T>  projected(const vector2<T> &v)
    {
        return v * (*this * v)/(v*v);
    }

    // given a position p1 and a velocity v1 produce a vector
    // perpendicular to v1 maximising distance from p1
    static vector2<T> perpendicular(const vector2<T> &pos_delta, const vector2<T> &v1)
    {
        const vector2<T> perpendicular1 = vector2<T>(-v1[1], v1[0]);
        const vector2<T> perpendicular2 = vector2<T>(v1[1], -v1[0]);
        const T d1 = perpendicular1 * pos_delta;
        const T d2 = perpendicular2 * pos_delta;
        if (d1 > d2) {
            return perpendicular1;
        }
        return perpendicular2;
    }

//      * Returns the point closest to p on the line segment (v,w).
//      *
//      * Comments and implementation taken from
//      * http://stackoverflow.com/questions/849211/shortest-distance-between-a-point-and-a-line-segment
    static vector2<T> closest_point(const vector2<T> &p, const vector2<T> &v, const vector2<T> &w)
    {
        // length squared of line segment
        const float l2 = (v - w).length_squared();
        if (l2 < FLT_EPSILON) {
            // v == w case
            return v;
        }
        // Consider the line extending the segment, parameterized as v + t (w - v).
        // We find projection of point p onto the line.
        // It falls where t = [(p-v) . (w-v)] / |w-v|^2
        // We clamp t from [0,1] to handle points outside the segment vw.
        const float t = ((p - v) * (w - v)) / l2;
        if (t <= 0) 
		{
            return v;
        } 
		else if (t >= 1) 
		{
            return w;
        } 
		else {
            return v + (w - v)*t;
        }
    }

    // w defines a line segment from the origin
    // p is a point
    // returns the closest distance between the radial and the point
    static float closest_distance_between_radial_and_point(const vector2<T> &w,
                                                           const vector2<T> &p)
    {
        const vector2<T> closest = closest_point(p, vector2<T>(0,0), w);
        const vector2<T> delta = closest - p;
        return delta.length();
    }

    // find the intersection between two line segments
    // returns true if they intersect, false if they do not
    // the point of intersection is returned in the intersection argument
    static bool segment_intersection(const vector2<T>& seg1_start, const vector2<T>& seg1_end, const vector2<T>& seg2_start, const vector2<T>& seg2_end, vector2<T>& intersection);

    // find the intersection between a line segment and a circle
    // returns true if they intersect and intersection argument is updated with intersection closest to seg_start
    static bool circle_segment_intersection(const vector2<T>& seg_start, const vector2<T>& seg_end, const vector2<T>& circle_center, float radius, vector2<T>& intersection);

    static bool point_on_segment(const vector2<T>& point,
                                 const vector2<T>& seg_start,
                                 const vector2<T>& seg_end) {
        const float expected_run = seg_end.x-seg_start.x;
        const float intersection_run = point.x-seg_start.x;
        // check slopes are identical:
        if (fabsf(expected_run) < FLT_EPSILON) {
            if (fabsf(intersection_run) > FLT_EPSILON) {
                return false;
            }
        } else {
            const float expected_slope = (seg_end.y-seg_start.y)/expected_run;
            const float intersection_slope = (point.y-seg_start.y)/intersection_run;
            if (fabsf(expected_slope - intersection_slope) > FLT_EPSILON) {
                return false;
            }
        }
        // check for presence in bounding box
        if (seg_start.x < seg_end.x) {
            if (point.x < seg_start.x || point.x > seg_end.x) {
                return false;
            }
        } else {
            if (point.x < seg_end.x || point.x > seg_start.x) {
                return false;
            }
        }
        if (seg_start.y < seg_end.y) {
            if (point.y < seg_start.y || point.y > seg_end.y) {
                return false;
            }
        } else {
            if (point.y < seg_end.y || point.y > seg_start.y) {
                return false;
            }
        }
        return true;
    }


};




typedef vector2<int16_t>        Vector2i;
typedef vector2<uint16_t>       Vector2ui;
typedef vector2<int32_t>        Vector2l;
typedef vector2<uint32_t>       Vector2ul;
typedef vector2<float>          Vector2f;


*/



#endif //__CMYMATH_H__
