#ifndef FIXMATH_H
#define FIXMATH_H

#include <stdint.h>
#include "arduino.h"

#define intFix int32_t
#define uintFix uint32_t
#define longFix int64_t
#define ulongFix uint64_t

#define FIX_BITS_BEFORE_DOT 2
#define FIX_SHIFTS (sizeof(intFix) * 8 - 1 - FIX_BITS_BEFORE_DOT)
#define FIX_MSB (1 << (sizeof(intFix) * 8 - 1))
#define FIX_UNITY (1 << FIX_SHIFTS)
#define FIX_MAX (FIX_MSB - 1)
#define FIX_MIN FIX_MSB

inline intFix float2Fix(float f) 
{
	return f * FIX_UNITY;
}

inline float fix2Float(longFix i)
{
	return (float)i / FIX_UNITY;
}

inline intFix fixMult(intFix a, intFix b) {
	return ((longFix)a * b) >> FIX_SHIFTS;
} 

inline intFix fixMult3(intFix a, intFix b, intFix c) {
	return fixMult(fixMult(a, b), c);
} 

inline intFix fixSat(longFix input) {
	if (input > FIX_MAX)
	{
		Serial.printf("OVERFLOW!\n\r");
		return FIX_MAX;
	}
	else if (input < FIX_MIN)
		return FIX_MIN;
	else
		return input;
}

inline intFix fixDiv(intFix a, intFix b) { // Implements saturation, quite slow
	return fixSat(((longFix)a << FIX_SHIFTS) / b);
}

inline uintFix fixSqrt(ulongFix sq) {
	uintFix res = 0;
	sq <<= FIX_SHIFTS % 2; // Needed for correct calculation
	// Can be optimized, because result can never equal max i. Also, consider early exit
	for (uintFix i = FIX_MSB; i != 0; i >>= 1) {
		ulongFix tmp = res | i;
		if (sq >= tmp * tmp)
			res = tmp;
	}
	return res << FIX_SHIFTS / 2;
}

inline intFix fixSin(intFix theta) {
	return float2Fix(sin(PI * fix2Float(theta)));
}


// This function requires input between -1 and 1
inline intFix fixSin2(intFix theta) {
	intFix tmp = (theta - fixMult(abs(theta), theta)) << 2; // Initial step, very imprecise
	return (tmp >> 1) + (tmp >> 2) + (fixMult(abs(tmp), tmp) >> 2); // Precision increase
}


inline intFix fixCos(intFix theta) {
	return float2Fix(cos(PI * fix2Float(theta)));
}

inline intFix fixAcos(intFix input) {
	return float2Fix(acos(fix2Float(input)) / (PI));
}

inline intFix fixAtan2(intFix B, intFix A) {
	float fA = fix2Float(A);
	float fB = fix2Float(B);
	return float2Fix(atan2(fB, fA) / (2 * PI));
}

#endif /* FIXMATH_H */
