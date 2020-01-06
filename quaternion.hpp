#pragma once
#include "vector.hpp"
#include "math_util.hpp"
#include <assert.h>


template<class T>
class TQuaternion
{
public:
	T x{ 0 }, y{ 0 }, z{ 0 }, w{ 1 };
public:
	/*
	constrctors
	*/
	TQuaternion() {};
	TQuaternion(T w, T x, T y, T z);
	TQuaternion(const TQuaternion& q);
public:
	/*
	normalize self
	*/
	void normalize();
	/*
	get rotation angle and rotation axis
	*/
	T getRotateAngle() const;
	TVector3<T> getRotateAxis() const;
// STATIC METHODS
public:
	/*
	set quaternion identity
	*/
	static TQuaternion<T> identity();
	/*
	get rotation quaternions
	*/
	template<class P>
	static TQuaternion<T> getRotateAboutX(P delta);
	template<class P>
	static TQuaternion<T> getRotateAboutY(P delta);
	template<class P>
	static TQuaternion<T> getRotateAboutZ(P delta);
	template<class P, class Q>
	static TQuaternion<T> getRotateAboutAxis(const TVector3<P>& axis, Q delta);

	/*
	dot product
	*/
	template<class P>
	static T dot(const TQuaternion<T>& a, const TQuaternion<P>& b);
	/*
	slerp
	*/
	template<class P, class Q>
	static TQuaternion<T> slerp(const TQuaternion<T>& a, const TQuaternion<P>& b, Q t);
	/*
	conjugate
	*/
	static TQuaternion<T> conjugate(const TQuaternion<T>& q);
	/*
	pow
	*/
	template<class P>
	static TQuaternion<T> pow(const TQuaternion<T>& q, P exponent);
	/*
	normalize
	*/
	static TQuaternion<T> normalize(const TQuaternion<T>& q);

// OPERATOR METHODS
public:
	TQuaternion<T>& operator=(const TQuaternion& q);
	TQuaternion<T> operator*(const TQuaternion& a) const;
	TQuaternion<T>& operator*=(const TQuaternion<T>& a);
};

template<class T>
inline TQuaternion<T>::TQuaternion(T w, T x, T y, T z):
	x(x), y(y), z(z), w(w)
{
}

template<class T>
inline TQuaternion<T>::TQuaternion(const TQuaternion& q):
	x(q.x), y(q.y), z(q.z), w(q.w)
{
}

template<class T>
inline void TQuaternion<T>::normalize()
{
	T mag = sqrt(x * x + y * y + z * z + w * w);
	if (mag > 0)
	{
		T oneOverMag = 1 / mag;
		x *= oneOverMag;
		y *= oneOverMag;
		z *= oneOverMag;
		w *= oneOverMag;
	}
	else
	{
		assert(false);
		return TQuaternion<T>::identity();
	}
}

template<class T>
inline T TQuaternion<T>::getRotateAngle() const
{
	T halfTheta = safeCos(w);
	return halfTheta * 2;
}

template<class T>
inline TVector3<T> TQuaternion<T>::getRotateAxis() const
{
	T sinHalfThetaSqr2 = 1 - w * w;

	if (sinHalfThetaSqr2 <= 0)
	{
		// not normalized. firsts normalize
		TQuaternion<T> qua = TQuaternion<T>::normalize(*this);
		sinHalfThetaSqr2 = 1 - qua.w * qua.w;
		if (sinHalfThetaSqr2 <= 0) return TVector3<T>(1, 0, 0);
		T oneOverSinHalfTheta = 1 / sqrt(sinHalfThetaSqr2);
		return TVector3<T>(qua.x * oneOverSinHalfTheta, qua.y * oneOverSinHalfTheta, qua.z * oneOverSinHalfTheta);
	}
	T oneOverSinHalfTheta = 1 / sqrt(sinHalfThetaSqr2);
	return TVector3<T>(x * oneOverSinHalfTheta, y * oneOverSinHalfTheta, z * oneOverSinHalfTheta);
}

template<class T>
inline TQuaternion<T> TQuaternion<T>::identity()
{
	return TQuaternion<T>();
}

template<class T>
inline TQuaternion<T> TQuaternion<T>::conjugate(const TQuaternion<T>& q)
{
	return TQuaternion<T>(-q.w, -q.x, -q.y, -q.z);
}

template<class T>
inline TQuaternion<T> TQuaternion<T>::normalize(const TQuaternion<T>& q)
{
	T mag = sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
	if (mag > 0)
	{
		T oneOverMag = 1 / mag;
		return TQuaternion<T>(
			q.w * oneOverMag,
			q.x * oneOverMag,
			q.y * oneOverMag,
			q.z * oneOverMag
			)
	}
	else
	{
		assert(false);
		return TQuaternion<T>::identity();
	}
}

template<class T>
inline TQuaternion<T>& TQuaternion<T>::operator=(const TQuaternion& q)
{
	if (this == &q) return *this;
	x = q.x;
	y = q.y;
	z = q.z;
	w = q.w;
	return *this;
}

template<class T>
inline TQuaternion<T> TQuaternion<T>::operator*(const TQuaternion& q) const
{
	T w = w * q.w - x * q.x - y * q.y - z * q.z;	// w1w1 - x1x2 - y1y2- z1z2
	T x = w * q.x + x * q.w + y * q.z - z * q.y;	// w1x2 + x1w2 + y1z2 - z1y2
	T y = w * q.y + y * q.w + z * q.x - x * q.z;	// w1y2 + y1w2 + z1x2 - x1z2
	T z = w * q.z + z * q.w + x * q.y - y * q.x;	// w1z2 + z1w2 + x1y2 - y1x2
	return TQuaternion<T>(w, x, y, z);
}

template<class T>
inline TQuaternion<T>& TQuaternion<T>::operator*=(const TQuaternion<T>& a)
{
	w = w * q.w - x * q.x - y * q.y - z * q.z;	// w1w1 - x1x2 - y1y2- z1z2
	x = w * q.x + x * q.w + y * q.z - z * q.y;	// w1x2 + x1w2 + y1z2 - z1y2
	y = w * q.y + y * q.w + z * q.x - x * q.z;	// w1y2 + y1w2 + z1x2 - x1z2
	z = w * q.z + z * q.w + x * q.y - y * q.x;	// w1z2 + z1w2 + x1y2 - y1x2
	return *this;
}

template<class T>
template<class P>
inline TQuaternion<T> TQuaternion<T>::getRotateAboutX(P delta)
{
	T halftheta = delta * 0.5;
	return TQuaternion<T>(cos(halftheta), sin(halftheta), 0, 0);
}

template<class T>
template<class P>
inline TQuaternion<T> TQuaternion<T>::getRotateAboutY(P delta)
{
	T halftheta = delta * 0.5;
	return TQuaternion<T>(cos(halftheta), 0, sin(halftheta), 0);
}

template<class T>
template<class P>
inline TQuaternion<T> TQuaternion<T>::getRotateAboutZ(P delta)
{
	T halftheta = delta * 0.5;
	return TQuaternion<T>(cos(halftheta), 0, 0, sin(halftheta));
}

template<class T>
template<class P, class Q>
inline TQuaternion<T> TQuaternion<T>::getRotateAboutAxis(const TVector3<P>& axis, Q delta)
{
	T halftheta = delta * 0.5;
	T sinHalfTheta = sin(halftheta);
	return TQuaternion<T>(
		cos(halftheta),
		axis.x * sinHalfTheta,
		axis.y * sinHalfTheta,
		axis.z * sinHalfTheta
		);
}

template<class T>
template<class P>
inline T TQuaternion<T>::dot(const TQuaternion<T>& a, const TQuaternion<P>& b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
}

template<class T>
template<class P, class Q>
inline TQuaternion<T> TQuaternion<T>::slerp(const TQuaternion<T>& a, const TQuaternion<P>& b, Q t)
{
	if (t < 0) return a;
	if (t > 1) return b;
	T cosAB = TQuaternion::dot(a, b);

	T ax = a.x, ay = a.y, az = a.z, aw = a.w;
	if (cosAB < 0)
	{
		ax = -a.x;
		ay = -a.y;
		az = -a.z;
		aw = -a.w;
		cosAB = -cosAB;
	}

	assert(cosAB < 1.1f);

	// todo
}

template<class T>
template<class P>
inline TQuaternion<T> TQuaternion<T>::pow(const TQuaternion<T>& q, P exponent)
{
	// identity
	if (fabs(q.w) >= 0.9999f)
	{
		return q;
	}

	T halfTheta = acos(q.w);
	T newHalfTheta = halfTheta * exponent;
	T sinOld = sin(halfTheta);
	T cosNew = cos(newHalfTheta);
	T sinNew = sin(newHalfTheta);
	T sinNewOverOld = sinNew / sinOld;
	return TQuaternion<T>(
		cosNew,
		q.x * sinNewOverOld,
		q.y * sinNewOverOld,
		q.z * sinNewOverOld
		);
}

