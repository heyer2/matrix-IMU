#ifndef FIXMATH_H
#define FIXMATH_H

#include <stdint.h>
#include "arduino.h"

#define intFix int32_t
#define longFix int64_t

#define FIX_SHIFTS (sizeof(intFix) * 8 - 1)
#define FIX_UNITY ((1 << FIX_SHIFTS) - 1)

inline intFix float2Fix(float f) 
{
	return f * FIX_UNITY;
}

inline float fix2Float(intFix i)
{
	return (float)i / FIX_UNITY;
}

inline intFix fixMult(intFix a, intFix b) {
	return ((longFix)a * b) >> FIX_SHIFTS;
} 

inline intFix fixMult3(intFix a, intFix b, intFix c) {
	return fixMult(fixMult(a, b), c);
} 

inline intFix fixDiv(intFix a, intFix b) {
	return ((longFix)a << FIX_SHIFTS) / b;
}
s

// THIS IS WRONG, CAN BE COMPARISON OF NEGATIVE NUMBER
inline intFix fixSqrt(intFix sq) {
	sq <<= 1;
	intFix res = 0;
	// Can be optimized, because result can never equal max i. Also, consider early exit
	for (int i = FIX_SHIFTS / 2; i >= 0; i--) {
		intFix tmp = res + (1 << i);
		if (sq >= tmp * tmp)
			res = tmp;
	}
	return res << FIX_SHIFTS / 2;
}



inline intFix fixSin(intFix theta) {
	return float2Fix(sin(2 * PI * fix2Float(theta)));
}

inline intFix fixCos(intFix theta) {
	return float2Fix(cos(2 * PI * fix2Float(theta)));
}


#endif /* FIXMATH_H */
