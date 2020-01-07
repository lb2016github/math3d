#include <time.h>
#include <iostream>
#include <Windows.h>
#include "vector.hpp"
#include <assert.h>
#include "matrix.hpp"
#include "quaternion.hpp"

typedef TVector2<float> Vector2f;
typedef TVector3<float> Vector3f;
typedef TVector4<float> Vector4f;
typedef TMatrix44<float> Matrix;
typedef TQuaternion<float> Quaternion;

template<class T>
void assertVector(const TVector3<T>& vec, T x, T y, T z)
{
	if (vec != TVector3<T>(x, y, z))
	{
		std::cout << "Vector3 assert failed" << std::endl;
	}
}
template<class T>
void assertValue(T a, T b)
{
	if (a != b)
	{
		std::cout << "Value assert failed" << a << b << std::endl;
	}
}

void testVector3()
{
	Vector3f vec(1, 1, 1);
	vec = (2.f * (3.f- (2.f + (vec * 4 / 2 + 1 - 0.5))));
	assertVector<float>(vec, -3, -3, -3);
	vec += 1;
	vec -= 1.5;
	vec *= 2;
	vec /= 7;
	assertVector<float>(vec, -1, -1, -1);
	vec = 1.f + vec;
	vec = 2.f - vec;
	vec = 2.f * vec;
	assertVector<float>(vec, 4, 4, 4);
	vec += {1, 1, 1};
	vec -= {1, 2, 3};
	assertVector<float>(vec, 4, 3, 2);
	std::cout << (vec == Vector3f(4, 3, 2)) << (vec != Vector3f(2, 3, 4)) << std::endl;
	assert(Vector3f::dot(vec, Vector3f(2, 3, 4)) == 25);
	assert(Vector3f::distanceSquare(vec, Vector3f(2, 3, 4)) == 8);
	std::cout << vec.magnitude() << std::endl;
}

void testVector2()
{
	Vector2f vec(0, 0);
	vec += Vector2f(3, 4);
	std::cout << vec.x << vec.y << std::endl;
}

void testVector4()
{
	Vector4f vec(1, 2, 3, 4);
	std::cout << vec.x << vec.y << vec.z << vec.w<<std::endl;
}

void testMatrix44()
{
	auto printMtx = [](const Matrix& mtx)
	{
		std::cout << std::endl;
		std::cout << mtx.m11 << " " << mtx.m12 << " " << mtx.m13 << " " << mtx.m14 << std::endl;
		std::cout << mtx.m21 << " " << mtx.m22 << " " << mtx.m23 << " " << mtx.m24 << std::endl;
		std::cout << mtx.m31 << " " << mtx.m32 << " " << mtx.m33 << " " << mtx.m34 << std::endl;
		std::cout << mtx.m41 << " " << mtx.m42 << " " << mtx.m43 << " " << mtx.m44 << std::endl;
		std::cout << std::endl;
	};
	auto printVect = [](const Vector3f& vec)
	{
		std::cout << vec.x << " " << vec.y << " " << vec.z << std::endl;
	};
	Matrix mtx;
	mtx += 5.f - (1.f + (2.f * (mtx * 4 / 2 + 1 -2)));
	printMtx(mtx);
	mtx += Matrix();
	mtx *= Matrix();
	printMtx(mtx);
	Vector4f vec(1, 2, 3, 1);
	vec = mtx * vec;

	Matrix invMtx = mtx.getUniInverseMatrix();
	Matrix rstMtx = mtx * invMtx;
	printMtx(rstMtx);
	printf("%f\n", rstMtx.uniDeterminant());


	
	TEulerAngle<float> angle(TO_RAD(30), TO_RAD(40), TO_RAD(50));
	Vector3f scale(1, 3, 4), trans(10, 20 ,30);
	printVect(angle);
	Matrix transform = Matrix::makeMatrix(scale, angle, trans);
	transform.decompose(scale, angle, trans);
	printf("transform\n");
	printMtx(transform);

	invMtx = transform.getInverseMatrix();
	rstMtx = transform * invMtx;
	printf("inverse transform\n");
	printMtx(invMtx);
	printf("%f\n", rstMtx.determinant());

	invMtx = transform.getUniInverseMatrix();
	rstMtx = transform * invMtx;
	printf("universal inverse transform\n");
	printMtx(invMtx);
	printf("%f\n", rstMtx.determinant());

}

union TestUnion
{
	TestUnion(): mtx() {}
	Vector3f vec;
	Matrix mtx;
	Quaternion qua;
};

int main(int argc, char** argv)
{
	//testVector3();
	//testVector2();
	//testVector4();
	testMatrix44();
	TestUnion tu;
	system("pause");
}