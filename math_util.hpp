#pragma once
#include <math.h>

const float C_PI = 3.14159265f;
const float C_2_PI = C_PI * 2;
const float C_PI_OVER_2 = C_PI / 2;
const float C_1_OVER_PI = 1 / C_PI;
const float C_1_OVER_2_PI = 1 / C_2_PI;

#define FLOAT_MIN_ERROR_VALUE 1E-6
#define TO_RAD(angle) angle * 0.017453292519943295f
#define TO_ANGLE(rad) rad * 57.29577951308232f
/*
wrap theta in range [-pi, pi]
*/
template<class T>
T wrapPi(T theta)
{
	theta += C_PI;
	theta -= floor(theta * C_1_OVER_2_PI) * C_2_PI;
	theta -= C_PI;
	return theta;
}

template<class T>
T safeCos(T x)
{
	if (x <= -1.f)
	{
		return C_PI;
	}
	if (x >= 1.f)
	{
		return 0;
	}
	return acos(x);
}

