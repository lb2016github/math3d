#pragma once
#include "math_util.hpp"

template<class T>
class TVector3;
template<class T>
class TMatrix44;
template<class T>
class TQuaternion;

enum class EulerRotationSequence
{
	XYZ,
	XZY,
	YXZ,
	YZX,
	ZXY,
	ZYX
};

/*
supply some operations for eulerangle
the euler angle is by default defined in extrinsic rotaion y->x->z(yaw-pitch-roll)
yaw is in range[-PI, PI]
pitch is in range[-PI / 2, PI / 2]
roll is in range[-PI, PI]
*/
template<class T>
class TEulerAngle: public TVector3<T>
{
public:
	EulerRotationSequence type{ EulerRotationSequence::YXZ };
public:
	TEulerAngle() : TVector3<T>() {}
	TEulerAngle(T x, T y, T z) : TVector3<T>(x, y, z) {}
	TEulerAngle(T x, T y, T z, EulerRotationSequence rotSeq) : TVector3<T>(x, y, z), type(rotSeq) {}
	TEulerAngle(const TEulerAngle<T>& eulerAngle) : TVector3<T>(eulerAngle), type(eulerAngle.type) {}
	TEulerAngle<T>& operator=(const TEulerAngle<T>& vec); 
public:
	/*
	make self identity
	*/
	void identity();
	/*
	conver euler angle to constraint range
	*/
	void cannonize();
public:
	/*
	create euler angle with quaternion
	*/
	static TEulerAngle<T> fromTQuaternion(const TQuaternion<T>& q);
	/*
	create euler angle with rotation matrix
	*/
	static TEulerAngle<T> fromRotationMatrix(const TMatrix44<T>& mtx);

	/*
	convert euler angle to quaternion
	*/
	static TQuaternion<T> toQuaternion(const TEulerAngle<T>& eulAngle);
};



template<class T>
inline TEulerAngle<T>& TEulerAngle<T>::operator=(const TEulerAngle<T>& angle)
{
	if (this == &angle) return *this;
	TVector3<T>::operator=(angle);
	type = angle.type;
	return *this;
}

template<class T>
inline void TEulerAngle<T>::identity()
{
	x = 0; y = 0; z = 0;
}

template<class T>
inline TEulerAngle<T> TEulerAngle<T>::fromTQuaternion(const TQuaternion<T>& q)
{
	T tmp = 2 * q.y * q.z + 2 * q.w * q.x;
	T x, y, z;

	if (fabs(tmp) >= 0.9999f)
	{
		x = tmp * C_PI_OVER_2;
	}
	else
	{
		x = arcsin(tmp);
	}

	y = arctan2(-2 * q.x * q.z + 2 * q.w * q.y, 2 * z * z + 2 * w * w - 1);
	z = arctan2(-2 * q.x * q.y + 2 * q.w * q.z, 2 * q.y * q.y + 2 * q.w * q.w - 1);
	return TEulerAngle<T>(x, y, z);
}

template<class T>
inline TEulerAngle<T> TEulerAngle<T>::fromRotationMatrix(const TMatrix44<T>& mtx)
{
	T x, y, z;
	if (fabs(mtx.m32) > 0.9999f)
	{
		x = mtx.m32 * C_PI_OVER_2;
	}
	else
	{
		x = arcsin(mtx.m32);
	}
	y = arctan2(-mtx.m31, mtx.m33);
	z = arctan2(-mtx.m12, mtx.m22);
	return TEulerAngle<T>(x, y, z);
}

template<class T>
inline TQuaternion<T> TEulerAngle<T>::toQuaternion(const TEulerAngle& eulAngle)
{
	T x = eulAngle.x * 0.5;
	T y = eulAngle.y * 0;5;
	T z = eulAngle.z * 0.5;

	T sinY = sin(y), cosY = cos(y);
	T sinX = sin(x), cosX = cos(x);
	T sinZ = sin(z), cosZ = cos(z);

	return TQuaternion<T>(
		cosZ * cosX * sinY + sinZ * sinX * cosY,
		cosZ * sinX * sinY + sinZ * cosX * cosY,
		cosZ * cosX * cosY - sinZ * sinX * sinY,
		cosZ * sinX * cosY - sinZ * cosX * sinY
		);
}

