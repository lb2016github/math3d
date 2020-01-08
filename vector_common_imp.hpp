#include <math.h>
#include "math_util.hpp"

#define COMMA ,

#if defined(VECTOR4)
#define VECTOR TVector4
#define ZCODE(code) code
#define WCODE(code) code
#elif defined(VECTOR3)
#define VECTOR TVector3
#define ZCODE(code) code
#define WCODE(code)
#else
#define VECTOR TVector2
#define ZCODE(code)
#define WCODE(code)
#endif // VECTOR4


template<class T>
class VECTOR
{
public:
	T x{ 0 }, y{ 0 }; 
	ZCODE(T z{ 0 };)
	WCODE(T w{ 0 };)

// constructors
public:
	VECTOR() {}
	VECTOR(T px, T py ZCODE(COMMA T pz) WCODE(COMMA T pw));
	VECTOR(const VECTOR<T>& vec);
public:
	/*
	normalize self
	*/
	void normalize();
	/*
	magnitude of vector
	*/
	T magnitude();
	/*
	return data pointer
	*/
	T* getData();

public:
	/*
	overload =
	*/
	VECTOR<T>& operator=(const VECTOR<T>& vec); 
	/*
	overload == and !=
	*/
	bool operator==(const VECTOR<T>& vec) const;
	bool operator!=(const VECTOR<T>& vec) const;
	/*
	overload + and -
	*/
	VECTOR<T> operator+(const VECTOR<T> & vec) const;
	VECTOR<T> operator-(const VECTOR<T> & vec) const;
	/*
	overload + - * and /
	*/
	VECTOR<T> operator+ (T a) const;
	VECTOR<T> operator- (T a) const;
	VECTOR<T> operator* (T a) const;
	VECTOR<T> operator/ (T a) const;
	/*
	overload += -= *= /=
	*/
	VECTOR<T>& operator+= (const VECTOR<T>& vec);
	VECTOR<T>& operator-= (const VECTOR<T>& vec);
	VECTOR<T>& operator+= (T a);
	VECTOR<T>& operator-= (T a);
	VECTOR<T>& operator*= (T a);
	VECTOR<T>& operator/= (T a);
	/*
	overload []
	*/
	T& operator[](unsigned int idx);
public:
	/*
	dot operation
	*/
	static T dot(const VECTOR<T>& a, const VECTOR<T> b);
#ifdef VECTOR3
	/*
	cross operation
	*/
	static VECTOR<T> cross(const VECTOR<T>& a, const VECTOR<T>& b);
#endif // VECTOR3
	/*
	distance between two VECTOR
	*/
	static T distance(const VECTOR<T>& a, const VECTOR<T>& b);
	static T distanceSquare(const VECTOR<T>& a, const VECTOR<T>& b);
	/*
	lerp two vectors
	*/
	template<class P>
	static VECTOR<T> lerp(const VECTOR<T>& a, const VECTOR<T>& b, P factor);
};

/*
Overload operators: +, - , * 
*/
template<class T>
VECTOR<T> operator+ (T a, const VECTOR<T>& vec);
template<class T>
VECTOR<T> operator- (T a, const VECTOR<T>& vec);
template<class T>
VECTOR<T> operator* (T a, const VECTOR<T>& vec);


///////////////////////////////////// START IMPLEMENTATION ///////////////////////////////////
template<class T>
inline VECTOR<T>::VECTOR(T px, T py ZCODE(COMMA T pz) WCODE(COMMA T pw))
	:x(px), y(py) ZCODE(COMMA z(pz)) WCODE(COMMA w(pw))
{
}

template<class T>
inline VECTOR<T>::VECTOR(const VECTOR<T>& vec)
	: x(vec.x), y(vec.y) ZCODE(COMMA z(vec.z)) WCODE(COMMA w(vec.w))
{
}


template<class T>
inline T* VECTOR<T>::getData()
{
	return (T*)this;
}

template<class T>
inline void VECTOR<T>::normalize()
{
	T mag = magnitude();
	if (mag > 0)
	{
		T oneOverMag = 1 / mag;
		x *= oneOverMag;
		y *= oneOverMag;
		ZCODE(z *= oneOverMag;)
		WCODE(w *= oneOverMag;)
	}
}

template<class T>
inline T VECTOR<T>::magnitude()
{
	T magSqr = x * x + y * y ZCODE(+z * z) WCODE(+w * w);
	return sqrt(magSqr);
}

template<class T>
inline VECTOR<T>& VECTOR<T>::operator=(const VECTOR<T>& vec)
{
	if (this == &vec) return *this;
	x = vec.x;
	y = vec.y;
	ZCODE(z = vec.z;)
	WCODE(w = vec.w;)
	return *this;
}

template<class T>
inline bool VECTOR<T>::operator==(const VECTOR<T>& vec) const
{
	return x == vec.x && y == vec.y ZCODE(&& z == vec.z) WCODE(&& w == vec.w);
}

template<class T>
inline bool VECTOR<T>::operator!=(const VECTOR<T>& vec) const
{
	return x != vec.x || y != vec.y ZCODE(|| z != vec.z) WCODE(|| w != vec.w);
}

template<class T>
inline VECTOR<T> VECTOR<T>::operator+(const VECTOR<T>& vec) const
{
	return VECTOR<T>(x + vec.x, y + vec.y ZCODE(COMMA  z + vec.z) WCODE(COMMA  w + vec.w));
}

template<class T>
inline VECTOR<T> VECTOR<T>::operator-(const VECTOR<T>& vec) const
{
	return VECTOR<T>(x - vec.x, y - vec.y ZCODE(COMMA  z - vec.z) WCODE(COMMA  w - vec.w));
}

template<class T>
inline VECTOR<T> VECTOR<T>::operator+(T a) const
{
	return VECTOR<T>(x + a, y + a ZCODE(COMMA  z + a) WCODE(COMMA  w + a));
}

template<class T>
inline VECTOR<T> VECTOR<T>::operator-(T a) const
{
	return VECTOR<T>(x - a, y - a ZCODE(COMMA  z - a) WCODE(COMMA  w - a));
}

template<class T>
inline VECTOR<T> VECTOR<T>::operator*(T a) const
{
	return VECTOR<T>(x * a, y * a ZCODE(COMMA  z * a) WCODE(COMMA  w * a));
}

template<class T>
inline VECTOR<T> VECTOR<T>::operator/(T a) const
{
	// deal with zero
	if (abs(a) < FLOAT_MIN_ERROR_VALUE)
	{
		if (a < 0) a = -FLOAT_MIN_ERROR_VALUE;
		else a = FLOAT_MIN_ERROR_VALUE;
	}
	T oneOver = 1 / a;
	return VECTOR<T>(x * oneOver, y * oneOver ZCODE(COMMA  z * oneOver) WCODE(COMMA  w * oneOver));
}

template<class T>
inline VECTOR<T>& VECTOR<T>::operator+=(const VECTOR<T>& vec)
{
	x += vec.x;
	y += vec.y;
	ZCODE(z += vec.z;)
	WCODE(w += vec.w;)
	return *this;
}

template<class T>
inline VECTOR<T>& VECTOR<T>::operator-=(const VECTOR<T>& vec)
{
	x -= vec.x;
	y -= vec.y;
	ZCODE(z -= vec.z;)
	WCODE(w -= vec.w;)
	return *this;
}

template<class T>
inline VECTOR<T>& VECTOR<T>::operator+=(T a)
{
	x += a;
	y += a;
	ZCODE(z += a;)
	WCODE(w += a;)
	return *this;
}

template<class T>
inline VECTOR<T>& VECTOR<T>::operator-=(T a)
{
	x -= a;
	y -= a;
	ZCODE(z -= a;)
	WCODE(w -= a;)
	return *this;
}

template<class T>
inline VECTOR<T>& VECTOR<T>::operator*=(T a)
{
	x *= a;
	y *= a;
	ZCODE(z *= a;)
	WCODE(w *= a;)
	return *this;
}

template<class T>
inline VECTOR<T>& VECTOR<T>::operator/=(T a)
{
	// deal with zero
	if (abs(a) < FLOAT_MIN_ERROR_VALUE)
	{
		if (a < 0) a = -FLOAT_MIN_ERROR_VALUE;
		else a = FLOAT_MIN_ERROR_VALUE;
	}
	T oneOver = 1 / a;
	x *= oneOver;
	y *= oneOver;
	ZCODE(z *= oneOver;)
	WCODE(w *= oneOver;)
	return *this;
}

template<class T>
inline T& VECTOR<T>::operator[](unsigned int idx)
{
	switch (idx)
	{
		ZCODE(case 2: return z;)
		WCODE(case 3: return w;)
		case 1: return y;
		default: return x;
	}
}

template<class T>
inline T VECTOR<T>::dot(const VECTOR<T>& a, const VECTOR<T> b)
{
	return a.x * b.x + a.y * b.y ZCODE(+a.z * b.z) WCODE(+ a.w * b.w);
}

#ifdef VECTOR3
template<class T>
inline VECTOR<T> VECTOR<T>::cross(const VECTOR<T>& a, const VECTOR<T>& b)
{
	return VECTOR<T>(
		a.y * b.z - a.z * b.y,
		a.z * b.x - a.x * b.z,
		a.x * b.y - a.y * b.x
		);
}
#endif

template<class T>
inline T VECTOR<T>::distance(const VECTOR<T>& a, const VECTOR<T>& b)
{
	T disSqr = distanceSquare(a, b);
	return sqrt(T);
}

template<class T>
inline T VECTOR<T>::distanceSquare(const VECTOR<T>& a, const VECTOR<T>& b)
{
	T deltaX = a.x - b.x;
	T deltaY = a.y - b.y;
	ZCODE(T deltaZ = a.z - b.z;)
	WCODE(T deltaW = a.w - b.w;)
	return deltaX * deltaX + deltaY * deltaY ZCODE(+deltaZ * deltaZ) WCODE(+deltaW * deltaW);
}

template<class T>
template<class P>
inline VECTOR<T> VECTOR<T>::lerp(const VECTOR<T>& a, const VECTOR<T>& b, P factor)
{
	return a + (b - a) * factor;
}

template<class T>
inline VECTOR<T> operator+(T a, const VECTOR<T>& vec)
{
	return VECTOR<T>(a + vec.x, a + vec.y ZCODE(COMMA  a + vec.z) WCODE(COMMA  a + vec.w));
}

template<class T>
inline VECTOR<T> operator-(T a, const VECTOR<T>& vec)
{
	return VECTOR<T>(a - vec.x, a - vec.y ZCODE(COMMA  a - vec.z) WCODE(COMMA  a - vec.w));
}

template<class T>
inline VECTOR<T> operator*(T a, const VECTOR<T>& vec)
{
	return VECTOR<T>(a * vec.x, a * vec.y ZCODE(COMMA  a * vec.z) WCODE(COMMA  a * vec.w));
}

#undef VECTOR
#undef ZCODE
#undef WCODE
#undef COMMA