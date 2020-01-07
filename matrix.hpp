#pragma once
#include "vector.hpp"
#include "euler_angle.hpp"
#include "quaternion.hpp"
#include <math.h>

/*
class of Matrix 4 * 4
*/
template<class T>
class TMatrix44
{
public:
	T m11{ 1 }, m12{ 0 }, m13{ 0 }, m14{ 0 };
	T m21{ 0 }, m22{ 1 }, m23{ 0 }, m24{ 0 };
	T m31{ 0 }, m32{ 0 }, m33{ 1 }, m34{ 0 };
	T m41{ 0 }, m42{ 0 }, m43{ 0 }, m44{ 1 };
public:
	TMatrix44() {};
	TMatrix44(const TMatrix44<T>& mtx);
	TMatrix44(
		T m11, T m12, T m13, T m14,
		T m21, T m22, T m23, T m24,
		T m31, T m32, T m33, T m34,
		T m41, T m42, T m43, T m44
	);

// operation to data
public:
	/*
	get data
	*/
	T* getData() { return (T*)this; }
	/*
	get row of index, which is in range[0, 3]
	*/
	TVector4<T> row(unsigned int index) const;
	/*
	get col of index, which is in range[0, 3]
	*/
	TVector4<T> col(unsigned int index) const;
	/*
	return value of certain 
	*/
	T& value(unsigned int row, unsigned int col);
	/*
	set value of matrix
	*/
	void setValue(unsigned int row, unsigned int col, T value);

// operation as a rotation matrix
public:
	/*
	get forward
	*/
	TVector3<T> getForward();
	/*
	get right
	*/
	TVector3<T> getRight();
	/*
	get up
	*/
	TVector3<T> getUp();


public:
	/*
	get scale
	*/
	TVector3<T> getScale();
	/*
	get scale
	*/
	void getScale(TVector3<T>& scale);
	/*
	get tranlation
	*/
	TVector3<T> getTranslation();
	/*
	get tranlation
	*/
	void getTranslation(TVector3<T>& trans);
	/*
	get rotation
	*/
	void getRotation(TEulerAngle<T, EulerType::YXZ>& eulerAngle);
	void getRotation(TQuaternion<T>& q);
	/*
	set translation of matrix
	*/
	void setTranslation(const TVector3<T>& trans);
	/*
	set rotation of matrix
	*/
	void setRotation(const TQuaternion<T>& q);
	/*
	set rotation of matrix
	*/
	template<EulerType Type>
	void setRotation(const TEulerAngle<T, Type>& eulAngle);
	/*
	set scale of matrix
	*/
	void setScale(const TVector3<T>& scale);
	void setScale(T scale);
	/*
	inverse self
	Notice: the method only support transform matrix, as the implementatio suppose the matrix compose
	of scale, rotation and transformation. If you want to inverse a common matrix, please use "uniInverse"
	*/
	void inverse();
	/*
	return inverse matrix
	*/
	TMatrix44<T> getInverseMatrix();
	/*
	get inverse of a universal matrix.
	Notice: for transform matrix, "inverse" is faster
	*/
	void uniInverse();
	/*
	get inverse of a universal matrix.
	Notice: for transform matrix, "getInverseMatrix" is faster
	*/
	TMatrix44<T> getUniInverseMatrix();
	/*
	make self identity
	*/
	void identity();
	/*
	make self transpose
	*/
	void transpose();
	/*
	return transpose matrix of self
	*/
	TMatrix44<T> getTransposeMatrix();
	/*
	decompse matrix
	*/
	void decompose(TVector3<T>& scale, TEulerAngle<T, EulerType::YXZ>& eulerAngle, TVector3<T>& trans);
	/*
	return determinant
	Notice: this is only valid for transform matrix, in which m41=m42=m43=0, m44=1
	*/
	T determinant();
	/*
	return determinant
	Notice: different to function "determinant", this function is valid for universal matrix
	*/
	T uniDeterminant();

public:
	/*
	compose matrix
	*/
	template<EulerType Type>
	static TMatrix44<T> makeMatrix(const TVector3<T>& scale, const TEulerAngle<T, Type>& eulerAngle, const TVector3<T>& trans);
	/*
	return rotation matrix
	*/
	static TMatrix44<T> makeRotationMatrix(const TQuaternion<T>& q);
	template<EulerType Type>
	static TMatrix44<T> makeRotationMatrix(const TEulerAngle<T, Type>& eulerAngle);
	/*
	return tranlation matrix
	*/
	static TMatrix44<T> makeTranlationMatrix(const TVector3<T>& trans);
	/*
	return scale matrix
	*/
	static TMatrix44<T> makeScaleMatrix(const TVector3<T>& scale);
	/*
	make perspective project matrix with given param
	@param fovy: fov in y direction
	@param aspect: w / h
	@param near: distance of near plane
	@param far: distance of far plane
	@return: matrix
	*/
	static TMatrix44<T> makePerspectiveProjectionMatrix(T fovy, T aspect, T n, T f);
	/*
	return identity matrix
	*/
	static TMatrix44<T> makeIdentityMatrix();

public:
	/*
	overload operator =
	*/
	TMatrix44<T>& operator=(const TMatrix44<T>& mtx);

	/*
	overload operators
	*/
	TMatrix44<T> operator+(T a);
	TMatrix44<T> operator+(const TMatrix44<T>& mtx);

	TMatrix44<T> operator-(T a);
	TMatrix44<T> operator-(const TMatrix44<T>& mtx);

	TMatrix44<T> operator*(T a);
	TMatrix44<T> operator*(const TMatrix44<T>& mtx);
	TVector4<T> operator*(const TVector4<T>& vec);

	TMatrix44<T> operator/(T a);

	TMatrix44<T>& operator+=(T a);
	TMatrix44<T>& operator+=(const TMatrix44<T>& mtx);

	TMatrix44<T>& operator-=(T a);
	TMatrix44<T>& operator-=(const TMatrix44<T>& mtx);
	
	TMatrix44<T>& operator*=(T a);
	TMatrix44<T>& operator*=(const TMatrix44<T>& mtx);

	TMatrix44<T>& operator/=(T a);
};

template<class T>
TMatrix44<T> operator+(T a, const TMatrix44<T>& mtx);
template<class T>
TMatrix44<T> operator-(T a, const TMatrix44<T>& mtx);
template<class T>
TMatrix44<T> operator*(T a, const TMatrix44<T>& mtx);


//////////////////////////////////////////////// START IMPLEMENTATION //////////////////////////////////
#define OP_NUM(_, a) \
m11 _ a, m12 _ a, m13 _ a, m14 _ a,\
m21 _ a, m22 _ a, m23 _ a, m24 _ a,\
m31 _ a, m32 _ a, m33 _ a, m34 _ a,\
m41 _ a, m42 _ a, m43 _ a, m44 _ a

#define OP_MTX(_, mtx) \
m11 _ mtx.m11, m12 _ mtx.m12, m13 _ mtx.m13, m14 _ mtx.m14,\
m21 _ mtx.m21, m22 _ mtx.m22, m23 _ mtx.m23, m24 _ mtx.m24,\
m31 _ mtx.m31, m32 _ mtx.m32, m33 _ mtx.m33, m34 _ mtx.m34,\
m41 _ mtx.m41, m42 _ mtx.m42, m43 _ mtx.m43, m44 _ mtx.m44

#define OP_SELF_MTX(_, mtx) \
m11 _ mtx.m11; m12 _ mtx.m12; m13 _ mtx.m13; m14 _ mtx.m14;\
m21 _ mtx.m21; m22 _ mtx.m22; m23 _ mtx.m23; m24 _ mtx.m24;\
m31 _ mtx.m31; m32 _ mtx.m32; m33 _ mtx.m33; m34 _ mtx.m34;\
m41 _ mtx.m41; m42 _ mtx.m42; m43 _ mtx.m43; m44 _ mtx.m44;

#define OP_SELF_NUM(_, a) \
m11 _ a; m12 _ a; m13 _ a; m14 _ a; \
m21 _ a; m22 _ a; m23 _ a; m24 _ a; \
m31 _ a; m32 _ a; m33 _ a; m34 _ a; \
m41 _ a; m42 _ a; m43 _ a; m44 _ a; 

#define NUM_OP_MTX(a, _, mtx) \
a _ mtx.m11, a _ mtx.m12, a _ mtx.m13, a _ mtx.m14, \
a _ mtx.m21, a _ mtx.m22, a _ mtx.m23, a _ mtx.m24, \
a _ mtx.m31, a _ mtx.m32, a _ mtx.m33, a _ mtx.m34, \
a _ mtx.m41, a _ mtx.m42, a _ mtx.m43, a _ mtx.m44

#define DETERMINANT3(m11, m12, m13, m21, m22, m23, m31, m32, m33) \
m11 * (m22 * m33 - m23 * m32) - m12 * (m21 * m33 - m23 * m31) + m13 * (m21 * m32 - m22 * m31)


template<class T>
inline TMatrix44<T>::TMatrix44(const TMatrix44<T>& mtx):
	m11(mtx.m11), m12(mtx.m12), m13(mtx.m13), m14(mtx.m14),
	m21(mtx.m21), m22(mtx.m22), m23(mtx.m23), m24(mtx.m24),
	m31(mtx.m31), m32(mtx.m32), m33(mtx.m33), m34(mtx.m34),
	m41(mtx.m41), m42(mtx.m42), m43(mtx.m43), m44(mtx.m44)
{
}


template<class T>
inline TMatrix44<T>::TMatrix44(T m11, T m12, T m13, T m14, T m21, T m22, T m23, T m24, T m31, T m32, T m33, T m34, T m41, T m42, T m43, T m44):
	m11(m11), m12(m12), m13(m13), m14(m14),
	m21(m21), m22(m22), m23(m23), m24(m24),
	m31(m31), m32(m32), m33(m33), m34(m34),
	m41(m41), m42(m42), m43(m43), m44(m44)
{
}

template<class T>
inline TVector4<T> TMatrix44<T>::row(unsigned int index) const
{
	switch (index)
	{
	case 0:
		return TVector4<T>(m11, m12, m13, m14);
	case 1:
		return TVector4<T>(m21, m22, m23, m24);
	case 2:
		return TVector4<T>(m31, m32, m33, m34);
	default:
		return TVector4<T>(m41, m42, m43, m44);
	}
}

template<class T>
inline TVector4<T> TMatrix44<T>::col(unsigned int index) const
{
	switch (index)
	{
	case 0:
		return TVector4<T>(m11, m21, m31, m41);
	case 1:
		return TVector4<T>(m12, m22, m32, m42);
	case 2:
		return TVector4<T>(m13, m23, m33, m43);
	default:
		return TVector4<T>(m14, m24, m34, m44);
	}
}

template<class T>
inline T& TMatrix44<T>::value(unsigned int row, unsigned int col)
{
	switch (row)
	{
	case 0:
		switch (col)
		{
		case 0: return m11;
		case 1: return m12;
		case 2: return m13;
		default: return m14;
		}
	case 1:
		switch (col)
		{
		case 0: return m21;
		case 1: return m22;
		case 2: return m23;
		default: return m24;
		}
	case 2:
		switch (col)
		{
		case 0: return m31;
		case 1: return m32;
		case 2: return m33;
		default: return m34;
		}
	default:
		switch (col)
		{
		case 0: return m41;
		case 1: return m42;
		case 2: return m43;
		default: return m44;
		}
	}
}

template<class T>
inline void TMatrix44<T>::setValue(unsigned int row, unsigned int col, T value)
{
	switch (row)
	{
	case 0:
		switch (col)
		{
		case 0: m11 = value; return;
		case 1: m12 = value; return;
		case 2: m13 = value; return;
		default: m14 = value; return;
		}
	case 1:
		switch (col)
		{
		case 0: m21 = value; return;
		case 1: m22 = value; return;
		case 2: m23 = value; return;
		default: m24 = value; return;
		}
	case 2:
		switch (col)
		{
		case 0: m31 = value; return;
		case 1: m32 = value; return;
		case 2: m33 = value; return;
		default: m34 = value; return;
		}
	default:
		switch (col)
		{
		case 0: m41 = value; return;
		case 1: m42 = value; return;
		case 2: m43 = value; return;
		default: m44 = value; return;
		}
	}
}

template<class T>
inline TVector3<T> TMatrix44<T>::getForward()
{
	auto c = col(2);
	TVector3<T> forward(c.x, c.y, c.z);
	forward.normalize();
	return forward;
}

template<class T>
inline TVector3<T> TMatrix44<T>::getRight()
{
	auto c = col(0);
	TVector3<T> right(c.x, c.y, c.z);
	right.normalize();
	return right;
}

template<class T>
inline TVector3<T> TMatrix44<T>::getUp()
{
	auto c = col(1);
	TVector3<T> up(c.x, c.y, c.z);
	up.normalize();
	return up;
}

template<class T>
inline TVector3<T> TMatrix44<T>::getScale()
{
	return TVector3<T>(
			sqrt(m11 * m11 + m21 * m21 + m31 * m31),
			sqrt(m12 * m12 + m22 * m22 + m32 * m32),
			sqrt(m13 * m13 + m23 * m23 + m33 * m33)
		);
}

template<class T>
inline void TMatrix44<T>::getScale(TVector3<T>& scale)
{
	scale.x = sqrt(m11 * m11 + m21 * m21 + m31 * m31);
	scale.y = sqrt(m12 * m12 + m22 * m22 + m32 * m32);
	scale.z = sqrt(m13 * m13 + m23 * m23 + m33 * m33);
}

template<class T>
inline TVector3<T> TMatrix44<T>::getTranslation()
{
	return TVector3<T>(m14, m24, m34);
}

template<class T>
inline void TMatrix44<T>::getTranslation(TVector3<T>& trans)
{
	trans.x = m14;
	trans.y = m24;
	trans.z = m34;
}

template<class T>
inline void TMatrix44<T>::getRotation(TEulerAngle<T, EulerType::YXZ>& eulerAngle)
{
	TVector3<T> scale = getScale();
	T new31 = m31 / scale.x;
	T new32 = m32 / scale.y;
	T new33 = m33 / scale.z;
	if (fabs(new32) > 0.9999f)
	{
		eulerAngle.x = new32 * C_PI_OVER_2;
	}
	else
	{
		eulerAngle.x = asin(new32);
	}
	eulerAngle.y = atan2(-new31, new33);
	eulerAngle.z = atan2(-m12, m22);

}

template<class T>
inline void TMatrix44<T>::getRotation(TQuaternion<T>& q)
{
	TVector3<T> scale = getScale();
	T new11 = m11 / scale.x, new22 = m22 / scale.y, new33 = m33 / scale.z;
	q.w = sqrt(new11 + new22 + new33 + 1) * 0.5;
	q.x = sqrt(new11 - new22 - new33 + 1) * 0.5;
	q.y = sqrt(-new11 + new22 - new33 + 1) * 0.5;
	q.z = sqrt(-new11 - new22 + new33 + 1) * 0.5;
}

template<class T>
inline void TMatrix44<T>::uniInverse()
{
	*this = getUniInverseMatrix();
}

template<class T>
inline TMatrix44<T> TMatrix44<T>::getUniInverseMatrix()
{
	// first calculate algebraic component
	T ac11 = DETERMINANT3(m22, m23, m24, m32, m33, m34, m42, m43, m44);
	T ac12 = -(DETERMINANT3(m21, m23, m24, m31, m33, m34, m41, m43, m44));
	T ac13 = DETERMINANT3(m21, m22, m24, m31, m32, m34, m41, m42, m44);
	T ac14 = -(DETERMINANT3(m21, m22, m23, m31, m32, m33, m41, m42, m43));

	T ac21 = -(DETERMINANT3(m12, m13, m14, m32, m33, m34, m42, m43, m44));
	T ac22 = DETERMINANT3(m11, m13, m14, m31, m33, m34, m41, m43, m44);
	T ac23 = -(DETERMINANT3(m11, m12, m14, m31, m32, m34, m41, m42, m44));
	T ac24 = DETERMINANT3(m11, m12, m13, m31, m32, m33, m41, m42, m43);

	T ac31 = DETERMINANT3(m12, m13, m14, m22, m23, m24, m42, m43, m44);
	T ac32 = -(DETERMINANT3(m11, m13, m14, m21, m23, m24, m41, m43, m44));
	T ac33 = DETERMINANT3(m11, m12, m14, m21, m22, m24, m41, m42, m44);
	T ac34 = -(DETERMINANT3(m11, m12, m13, m21, m22, m23, m41, m42, m43));

	T ac41 = -(DETERMINANT3(m12, m13, m14, m22, m23, m24, m32, m33, m34));
	T ac42 = DETERMINANT3(m11, m13, m14, m21, m23, m24, m31, m33, m34);
	T ac43 = -(DETERMINANT3(m11, m12, m14, m21, m22, m24, m31, m32, m34));
	T ac44 = DETERMINANT3(m11, m12, m13, m21, m22, m23, m31, m32, m33);


	T det = m11 * ac11 + m12 * ac12 + m13 * ac13 + m14 * ac14;
	// no inverse, just return identity
	if (det == 0)
	{
		return TMatrix44<T>();
	}

	T oneOverDet = 1 / det;

	return TMatrix44<T>(
		ac11 * oneOverDet, ac21 * oneOverDet, ac31 * oneOverDet, ac41 * oneOverDet,
		ac12 * oneOverDet, ac22 * oneOverDet, ac32 * oneOverDet, ac42 * oneOverDet,
		ac13 * oneOverDet, ac23 * oneOverDet, ac33 * oneOverDet, ac43 * oneOverDet,
		ac14 * oneOverDet, ac24 * oneOverDet, ac34 * oneOverDet, ac44 * oneOverDet
		);
}

template<class T>
inline void TMatrix44<T>::identity()
{
	m11 = 1, m12 = 0, m13 = 0, m14 = 0;
	m21 = 0, m22 = 1, m23 = 0, m24 = 0;
	m31 = 0, m32 = 0, m33 = 1, m34 = 0;
	m41 = 0, m42 = 0, m43 = 0, m44 = 1;
}

template<class T>
void TMatrix44<T>::transpose()
{
	std::swap<T>(m12, m21);
	std::swap<T>(m13, m31);
	std::swap<T>(m14, m41);
	std::swap<T>(m23, m32);
	std::swap<T>(m24, m42);
	std::swap<T>(m34, m43);
}

template<class T>
inline TMatrix44<T> TMatrix44<T>::getTransposeMatrix()
{
	 return TMatrix44<T>(
			m11, m21, m31, m41,
			m12, m22, m32, m42,
			m13, m23, m33, m43,
			m13, m24, m34, m44
		);
}

template<class T>
void TMatrix44<T>::decompose(TVector3<T>& scale, TEulerAngle<T, EulerType::YXZ>& eulerAngle, TVector3<T>& trans)
{
	// get scale
	scale = getScale();
	// get euler angle
	T new31 = m31 / scale.x;
	T new32 = m32 / scale.y;
	T new33 = m33 / scale.z;
	if (fabs(new32) > 0.9999f)
	{
		eulerAngle.x = new32 * C_PI_OVER_2;
	}
	else
	{
		eulerAngle.x = asin(new32);
	}
	eulerAngle.y = atan2(-new31, new33);
	eulerAngle.z = atan2(-m12, m22);
	// get translation
	trans.x = m14;
	trans.y = m24;
	trans.z = m34;
}

template<class T>
inline T TMatrix44<T>::determinant()
{
	// as the last row is [0, 0, 0, 1], det(mtx44) equals to det(mtx33)
	return DETERMINANT3(
		m11, m12, m13,
		m21, m22, m23,
		m31, m32, m33
	);
}

template<class T>
inline T TMatrix44<T>::uniDeterminant()
{
	T det11 = DETERMINANT3(m22, m23, m24, m32, m33, m34, m42, m43, m44);
	T det12 = DETERMINANT3(m21, m23, m24, m31, m33, m34, m41, m43, m44);
	T det13 = DETERMINANT3(m21, m22, m24, m31, m32, m34, m41, m42, m44);
	T det14 = DETERMINANT3(m21, m22, m23, m31, m32, m33, m41, m42, m43);
	return m11 * det11 - m12 * det12 + m13 * det13 - m14 * det14;
}

template<class T>
inline void TMatrix44<T>::setTranslation(const TVector3<T>& trans)
{
	m14 = trans.x;
	m24 = trans.y;
	m34 = trans.z;
	m44 = 1;
}

template<class T>
inline void TMatrix44<T>::setRotation(const TQuaternion<T>& q)
{
	TVector3<T> scale = getScale();
	m11 = (2 * q.x * q.x + 2 * q.w * q.w - 1) * scale.x,	m12 = (2 * q.x * q.y - 2 * q.w * q.z) * scale.y,		m13 = (2 * q.x * q.z - 2 * q.w * q.y) * scale.z;
	m21 = (2 * q.x * q.y + 2 * q.w * q.z) * scale.x,		m22 = (2 * q.y * q.y + 2 * q.w * q.w - 1) * scale.y,	m23 = (2 * q.y * q.z - 2 * q.w * q.x) * scale.z;
	m31 = (2 * q.x * q.z - 2 * q.w * q.y) * scale.x,		m32 = (2 * q.y * q.z + 2 * q.w * q.x) * scale.y,		m33 = (2 * q.z * q.z + 2 * q.w * q.w - 1) * scale.z;
}

template<class T>
template<EulerType Type>
inline void TMatrix44<T>::setRotation(const TEulerAngle<T, Type>& eulAngle)
{
	TVector3<T> scale = getScale();
	TMatrix44<T> mtx = makeRotationMatrix(eulAngle);
	m11 = mtx.m11 * scale.x, m12 = mtx.m12 * scale.y, m13 = mtx.m13 * scale.z;
	m21 = mtx.m21 * scale.x, m22 = mtx.m22 * scale.y, m23 = mtx.m23 * scale.z;
	m31 = mtx.m31 * scale.x, m32 = mtx.m32 * scale.y, m33 = mtx.m33 * scale.z;
}

template<class T>
void TMatrix44<T>::setScale(const TVector3<T>& scale)
{
	TVector3<T> oldScale = getScale();
	TVector3<T> factor(scale.x / oldScale.x, scale.y / oldScale.y, scale.z / oldScale.z);
	m11 *= scale.x, m12 *= scale.y, m13 *= scale.z;
	m21 *= scale.x, m22 *= scale.y, m23 *= scale.z;
	m31 *= scale.x, m32 *= scale.y, m33 *= scale.z;
}

template<class T>
inline void TMatrix44<T>::setScale(T scale)
{
	setScale({ scale, scale, scale });
}

template<class T>
inline void TMatrix44<T>::inverse()
{
	T det = determinant();
	assert(abs(det) > FLOAT_MIN_ERROR_VALUE);
	T oneOverDet = 1 / det;
	// M = T*R*S=>inv(M) = inv(TRS) = inv(RS)*inv(T)
	// first calculate inv(RS)
	T new11 = (m22 * m33 - m23 * m32) * oneOverDet;
	T new12 = (m13 * m32 - m12 * m33) * oneOverDet;
	T new13 = (m12 * m23 - m13 * m22) * oneOverDet;

	T new21 = (m23 * m31 - m21 * m33) * oneOverDet;
	T new22 = (m11 * m33 - m13 * m31) * oneOverDet;
	T new23 = (m13 * m21 - m11 * m23) * oneOverDet;

	T new31 = (m21 * m32 - m22 * m31) * oneOverDet;
	T new32 = (m12 * m31 - m11 * m32) * oneOverDet;
	T new33 = (m11 * m22 - m12 * m21) * oneOverDet;

	// then calculate inv(RS) * inv(T)
	T new14 = -(new11 * m14 + new12 * m24 + new13 * m34);
	T new24 = -(new21 * m14 + new22 * m24 + new23 * m34);
	T new34 = -(new31 * m14 + new32 * m24 + new33 * m34);

	m11 = new11, m12 = new12, m13 = new13, m14 = new14;
	m21 = new21, m22 = new22, m23 = new23, m24 = new24;
	m31 = new31, m32 = new32, m33 = new33, m34 = new34;
}

template<class T>
inline TMatrix44<T> TMatrix44<T>::getInverseMatrix()
{
	T det = determinant();
	assert(abs(det) > FLOAT_MIN_ERROR_VALUE);
	T oneOverDet = 1 / det;
	// M = T*R*S=>inv(M) = inv(TRS) = inv(RS)*inv(T)
	// first calculate inv(RS)
	T new11 = (m22 * m33 - m23 * m32) * oneOverDet;
	T new12 = (m13 * m32 - m12 * m33) * oneOverDet;
	T new13 = (m12 * m23 - m13 * m22) * oneOverDet;

	T new21 = (m23 * m31 - m21 * m33) * oneOverDet;
	T new22 = (m11 * m33 - m13 * m31) * oneOverDet;
	T new23 = (m13 * m21 - m11 * m23) * oneOverDet;

	T new31 = (m21 * m32 - m22 * m31) * oneOverDet;
	T new32 = (m12 * m31 - m11 * m32) * oneOverDet;
	T new33 = (m11 * m22 - m12 * m21) * oneOverDet;

	// then calculate inv(RS) * inv(T)
	T new14 = -(new11 * m14 + new12 * m24 + new13 * m34);
	T new24 = -(new21 * m14 + new22 * m24 + new23 * m34);
	T new34 = -(new31 * m14 + new32 * m24 + new33 * m34);
	return TMatrix44<T>(
		new11, new12, new13, new14,
		new21, new22, new23, new24,
		new31, new32, new33, new34,
		0,		0,		0,		1
		);
}


template<class T>
template<EulerType Type>
TMatrix44<T> TMatrix44<T>::makeMatrix(const TVector3<T>& scale, const TEulerAngle<T, Type>& eulerAngle, const TVector3<T>& trans)
{
	auto rotMtx = makeRotationMatrix(eulerAngle);
	return TMatrix44<T>(
			rotMtx.m11 * scale.x,	rotMtx.m12 * scale.y, rotMtx.m13 * scale.z, trans.x,
			rotMtx.m21 * scale.x,	rotMtx.m22 * scale.y, rotMtx.m23 * scale.z, trans.y,
			rotMtx.m31 * scale.x,	rotMtx.m32 * scale.y, rotMtx.m33 * scale.z, trans.z,
			0,	0,	0,	1
		);
}

template<class T>
inline TMatrix44<T> TMatrix44<T>::makeRotationMatrix(const TQuaternion<T>& q)
{
	return TMatrix44<T>(
		2 * q.x * q.x + 2 * q.w * q.w - 1,	2 * q.x * q.y - 2 * q.w * q.z,		2 * q.x * q.z - 2 * q.w * q.y,		0,
		2 * q.x * q.y + 2 * q.w * q.z,		2 * q.y * q.y + 2 * q.w * q.w - 1,	2 * q.y * q.z - 2 * q.w * q.x,		0,
		2 * q.x * q.z - 2 * q.w * q.y,		2 * q.y * q.z + 2 * q.w * q.x,		2 * q.z * q.z + 2 * q.w * q.w -1,	0,
		0,									0,									0,									1,
		);
}

template<class T>
template<EulerType Type>
inline TMatrix44<T> TMatrix44<T>::makeRotationMatrix(const TEulerAngle<T, Type>& eulerAngle)
{
	T cosY = cos(eulerAngle.y), cosX = cos(eulerAngle.x), cosZ = cos(eulerAngle.z);
	T sinY = sin(eulerAngle.y), sinX = sin(eulerAngle.x), sinZ = sin(eulerAngle.z);
	switch (eulerAngle.type)
	{
	case EulerType::YXZ:
#define c1 cosZ
#define s1 sinZ
#define c2 cosX
#define s2 sinX
#define c3 cosY
#define s3 sinY
		return TMatrix44<T>(
			c1 * c3 - s1 * s2 * s3,	-c2 * s1,	c1 * s3 + c3 * s1 * s2,	0,
			c3 * s1 + c1 * s2 * s3,	c1 * c2,	s1 * s3 - c1 * c3 * s2,	0,
			-c2 * s3,	s2,	c2 * c3,	0,
			0,	0,	0,	1
			);
#undef c1
#undef s1
#undef c2
#undef s2
#undef c3
#undef s3
	case EulerType::ZXY:
#define c1 cosY
#define s1 sinY
#define c2 cosX
#define s2 sinX
#define c3 cosY
#define s3 sinY
		return TMatrix44<T>(
			c1 * c3 + s1 * s2 * s3,	c3 * s1 * s2 - c1 * s3,		c2 * s1,	0,
			c2 * s3,	c2 * c3,	-s2,	0,
			c1 * s2 * s3 - c3 * s1,	c1 * c3 * s2 + s1 * s3,	c1 * c2,	0,
			0,	0,	0,	1
			);
#undef c1
#undef s1
#undef c2
#undef s2
#undef c3
#undef s3
	case EulerType::YZX:
#define c1 cosX
#define s1 sinX
#define c2 cosZ
#define s2 sinZ
#define c3 cosY
#define s3 sinY
		return TMatrix44<T>(
			c2 * c3,	-s2,	c2 * s3,	0,
			s1 * s3 + c1 * c3 * s2,	c1 * c2,	c1 * s2 * s3 - c3 * s1,	0,
			c3 * s1 * s2 - c1 * s3,	c2 * s1,	c1 * c3 + s1 * s2 * s3,	0,
			0,	0,	0,	1
			);
#undef c1
#undef s1
#undef c2
#undef s2
#undef c3
#undef s3
	case EulerType::XZY:
#define c1 cosY
#define s1 sinY
#define c2 cosZ
#define s2 sinZ
#define c3 cosX
#define s3 sinX
		return TMatrix44<T>(
			c1 * c2, s1 * s3 - c1 * c3 * s2, c3 * s1 + c1 * s2 * s3,	0,
			s2, c2 * c3, -c2 * s3,	0,
			-c2 * s1, c1 * s3 + c3 * s1 * s2, c1 * c3 - s1 * s2 * s3,	0,
			0,	0,	0,	1
			);
#undef c1
#undef s1
#undef c2
#undef s2
#undef c3
#undef s3
	case EulerType::XYZ:
#define c1 cosZ
#define s1 sinZ
#define c2 cosY
#define s2 sinY
#define c3 cosX
#define s3 sinX
		return TMatrix44<T>(
			c1 * c2,	c1 * s2 * s3 - c3 * s1,		s1 * s3 + c1 * c3 * s2,	0,
			c2 * s1,	c1 * c3 + s1 * s2 * s3,	c3 * s1 * s2 - c1 * s3,	0,
			-s2,	c2 * s3,	c2 * c3,	0,
			0,	0,	0,	1
			);
#undef c1
#undef s1
#undef c2
#undef s2
#undef c3
#undef s3
	case EulerType::ZYX:
#define c1 cosX
#define s1 sinX
#define c2 cosY
#define s2 sinY
#define c3 cosZ
#define s3 sinZ
		return TMatrix44<T>(
			c2 * c3,	-c2 * s3,	s2,	0,
			c1 * s3 + c3 * s1 * s2,	c1 * c3 - s1 * s2 * s3,	-c2 * s1,	0,
			s1 * s3 - c1 * c3 * s2,	c3 * s1 + c1 * s2 * s3,	c1 * c2,	0,
			0,	0,	0,	1
			);
#undef c1
#undef s1
#undef c2
#undef s2
#undef c3
#undef s3
	}
}

template<class T>
TMatrix44<T> TMatrix44<T>::makeTranlationMatrix(const TVector3<T>& trans)
{
	TMatrix44<T> mtx;
	mtx.m14 = trans.x;
	mtx.m24 = trans.y;
	mtx.m34 = trans.z;
	mtx.m44 = 1;
	return mtx;
}

template<class T>
TMatrix44<T> TMatrix44<T>::makeScaleMatrix(const TVector3<T>& scale)
{
	TMatrix44<T> mtx;
	mtx.m11 = scale.x;
	mtx.m22 = scale.y;
	mtx.m33 = scale.z;
	return mtx;
}

template<class T>
inline TMatrix44<T> TMatrix44<T>::makePerspectiveProjectionMatrix(T fovy, T aspect, T n, T f)
{
	assert(fovy != 0);
	T t = tan(fovy * 0.5);
	T nOneOverFN = -1 / (f - n);
	T oneOverT = 1 / t;
	return TMatrix44<T>(
		oneOverT / aspect,	0,	0,	0,
		0,	oneOverT,	0,	0,
		0,	0,	(f + n) * nOneOverFN,	2 * f * n * nOneOverFN,
		0,	0,	-1,	0
		);
}

template<class T>
inline TMatrix44<T> TMatrix44<T>::makeIdentityMatrix()
{
	return TMatrix44<T>(
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0,
		0, 0, 0, 1
		);
}

template<class T>
inline TMatrix44<T>& TMatrix44<T>::operator=(const TMatrix44<T>& mtx)
{
	if (this == &mtx) return *this;
	m11 = mtx.m11; m12 = mtx.m12; m13 = mtx.m13; m14 = mtx.m14;
	m21 = mtx.m21; m22 = mtx.m22; m23 = mtx.m23; m24 = mtx.m24;
	m31 = mtx.m31; m32 = mtx.m32; m33 = mtx.m33; m34 = mtx.m34;
	m41 = mtx.m41; m42 = mtx.m42; m43 = mtx.m43; m44 = mtx.m44;
	return *this;
}

template<class T>
inline TMatrix44<T> TMatrix44<T>::operator+(T a)
{
	return TMatrix44<T>(
		OP_NUM(+, a)
		);
}

template<class T>
inline TMatrix44<T> TMatrix44<T>::operator+(const TMatrix44<T>& mtx)
{
	return TMatrix44<T>(
		OP_MTX(+, mtx)
		);
}

template<class T>
inline TMatrix44<T> TMatrix44<T>::operator-(T a)
{
	return TMatrix44<T>(
		OP_NUM(-, a)
		);
}

template<class T>
inline TMatrix44<T> TMatrix44<T>::operator-(const TMatrix44<T>& mtx)
{
	return TMatrix44<T>(
		OP_MTX(-, mtx)
		);
}

template<class T>
inline TMatrix44<T> TMatrix44<T>::operator*(T a)
{
	return TMatrix44<T>(
		OP_NUM(*, a)
		);
}

template<class T>
inline TMatrix44<T> TMatrix44<T>::operator*(const TMatrix44<T>& mtx)
{
	return TMatrix44<T>(
		m11 * mtx.m11 + m12 * mtx.m21 + m13 * mtx.m31 + m14 * mtx.m41,	// m11
		m11 * mtx.m12 + m12 * mtx.m22 + m13 * mtx.m32 + m14 * mtx.m42,	// m12
		m11 * mtx.m13 + m12 * mtx.m23 + m13 * mtx.m33 + m14 * mtx.m43,	// m13
		m11 * mtx.m14 + m12 * mtx.m24 + m13 * mtx.m34 + m14 * mtx.m44,	// m14

		m21 * mtx.m11 + m22 * mtx.m21 + m23 * mtx.m31 + m24 * mtx.m41,	// m21
		m21 * mtx.m12 + m22 * mtx.m22 + m23 * mtx.m32 + m24 * mtx.m42,	// m22
		m21 * mtx.m13 + m22 * mtx.m23 + m23 * mtx.m33 + m24 * mtx.m43,	// m23
		m21 * mtx.m14 + m22 * mtx.m24 + m23 * mtx.m34 + m24 * mtx.m44,	// m24

		m31 * mtx.m11 + m32 * mtx.m21 + m33 * mtx.m31 + m34 * mtx.m41,	// m31
		m31 * mtx.m12 + m32 * mtx.m22 + m33 * mtx.m32 + m34 * mtx.m42,	// m32
		m31 * mtx.m13 + m32 * mtx.m23 + m33 * mtx.m33 + m34 * mtx.m43,	// m33
		m31 * mtx.m14 + m32 * mtx.m24 + m33 * mtx.m34 + m34 * mtx.m44,	// m34

		m41 * mtx.m11 + m42 * mtx.m21 + m43 * mtx.m31 + m44 * mtx.m41,	// m41
		m41 * mtx.m12 + m42 * mtx.m22 + m43 * mtx.m32 + m44 * mtx.m42,	// m42
		m41 * mtx.m13 + m42 * mtx.m23 + m43 * mtx.m33 + m44 * mtx.m43,	// m43
		m41 * mtx.m14 + m42 * mtx.m24 + m43 * mtx.m34 + m44 * mtx.m44	// m44
		);
}

template<class T>
inline TVector4<T> TMatrix44<T>::operator*(const TVector4<T>& vec)
{
	return TVector4<T>(
		m11 * vec.x + m12 * vec.y + m13 * vec.z + m14 * vec.w,
		m21 * vec.x + m22 * vec.y + m23 * vec.z + m24 * vec.w,
		m31 * vec.x + m32 * vec.y + m33 * vec.z + m34 * vec.w,
		m41 * vec.x + m42 * vec.y + m43 * vec.z + m44 * vec.w
		);
}

template<class T>
inline TMatrix44<T> TMatrix44<T>::operator/(T a)
{
	return TMatrix44<T>(
		OP_NUM(/ , a)
		);
}

template<class T>
inline TMatrix44<T>& TMatrix44<T>::operator+=(T a)
{
	OP_SELF_NUM(+=, a);
	return *this;
}

template<class T>
inline TMatrix44<T>& TMatrix44<T>::operator+=(const TMatrix44<T>& mtx)
{
	OP_SELF_MTX(+=, mtx);
	return *this;
}

template<class T>
inline TMatrix44<T>& TMatrix44<T>::operator-=(T a)
{
	OP_SELF_NUM(-=, a);
	return *this;
}

template<class T>
inline TMatrix44<T>& TMatrix44<T>::operator-=(const TMatrix44<T>& mtx)
{
	OP_SELF_MTX(-=, mtx);
	return *this;
}

template<class T>
inline TMatrix44<T>& TMatrix44<T>::operator*=(T a)
{
	OP_SELF_NUM(*=, a);
	return *this;
}

template<class T>
inline TMatrix44<T>& TMatrix44<T>::operator*=(const TMatrix44<T>& mtx)
{
	m11 = m11 * mtx.m11 + m12 * mtx.m21 + m13 * mtx.m31 + m14 * mtx.m41;	// m11
	m12 = m11 * mtx.m12 + m12 * mtx.m22 + m13 * mtx.m32 + m14 * mtx.m42;	// m12
	m13 = m11 * mtx.m13 + m12 * mtx.m23 + m13 * mtx.m33 + m14 * mtx.m43;	// m13
	m14 = m11 * mtx.m14 + m12 * mtx.m24 + m13 * mtx.m34 + m14 * mtx.m44;	// m14

	m21 = m21 * mtx.m11 + m22 * mtx.m21 + m23 * mtx.m31 + m24 * mtx.m41;	// m21
	m22 = m21 * mtx.m12 + m22 * mtx.m22 + m23 * mtx.m32 + m24 * mtx.m42;	// m22
	m23 = m21 * mtx.m13 + m22 * mtx.m23 + m23 * mtx.m33 + m24 * mtx.m43;	// m23
	m24 = m21 * mtx.m14 + m22 * mtx.m24 + m23 * mtx.m34 + m24 * mtx.m44;	// m24

	m31 = m31 * mtx.m11 + m32 * mtx.m21 + m33 * mtx.m31 + m34 * mtx.m41;	// m31
	m32 = m31 * mtx.m12 + m32 * mtx.m22 + m33 * mtx.m32 + m34 * mtx.m42;	// m32
	m33 = m31 * mtx.m13 + m32 * mtx.m23 + m33 * mtx.m33 + m34 * mtx.m43;	// m33
	m34 = m31 * mtx.m14 + m32 * mtx.m24 + m33 * mtx.m34 + m34 * mtx.m44;	// m34

	m41 = m41 * mtx.m11 + m42 * mtx.m21 + m43 * mtx.m31 + m44 * mtx.m41;	// m41
	m42 = m41 * mtx.m12 + m42 * mtx.m22 + m43 * mtx.m32 + m44 * mtx.m42;	// m42
	m43 = m41 * mtx.m13 + m42 * mtx.m23 + m43 * mtx.m33 + m44 * mtx.m43;	// m43
	m44 = m41 * mtx.m14 + m42 * mtx.m24 + m43 * mtx.m34 + m44 * mtx.m44;	// m44
	return *this;
}

template<class T>
inline TMatrix44<T>& TMatrix44<T>::operator/=(T a)
{
	OP_SELF_NUM(/=, a);
	return *this;
}

template<class T>
inline TMatrix44<T> operator+(T a, const TMatrix44<T>& mtx)
{
	return TMatrix44<T>(
		NUM_OP_MTX(a, +, mtx)
		);
}

template<class T>
inline TMatrix44<T> operator-(T a, const TMatrix44<T>& mtx)
{
	return TMatrix44<T>(
		NUM_OP_MTX(a, -, mtx)
		);
}

template<class T>
inline TMatrix44<T> operator*(T a, const TMatrix44<T>& mtx)
{
	return TMatrix44<T>(
		NUM_OP_MTX(a, *, mtx)
		);
}
