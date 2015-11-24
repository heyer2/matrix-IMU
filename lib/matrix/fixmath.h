#ifndef FIXMATH_H
#define FIXMATH_H

#include <stdint.h>
#include "arduino.h"

#define intFix int32_t
#define uintFix uint32_t
#define longFix int64_t

#define FIX_SHIFTS (sizeof(intFix) * 8 - 2)
#define FIX_UNITY (1 << FIX_SHIFTS)

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

inline intFix fixDiv(intFix a, intFix b) { // Implements saturation, quite slow
	return ((longFix)a << FIX_SHIFTS) / b;
}

// THIS IS WRONG, CAN BE COMPARISON OF NEGATIVE NUMBER
inline uintFix fixSqrt(uintFix sq) {
	uintFix res = 0;
	// Can be optimized, because result can never equal max i. Also, consider early exit
	for (int i = sizeof(intFix) * 8 / 2 - 1; i >= 0; i--) {
		uintFix tmp = res | (1 << i);
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

inline intFix fixAcos(intFix input) {
	return float2Fix(acos(fix2Float(input)) / (2 * PI));
}

#endif /* FIXMATH_H */
