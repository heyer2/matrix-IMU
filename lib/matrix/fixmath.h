#ifndef FIXMATH_H
#define FIXMATH_H

#include <stdint.h>

#define intFix int32_t
#define longFix int64_t

#define FIX_SHIFTS (sizeof(intFix) * 8)
#define FIX_UNITY (1 << FIX_SHIFTS - 1)

inline intFix fixMult(intFix a, intFix b) {
	return ((longFix)a * b) >> FIX_SHIFTS;
} 

inline intFix divFix(intFix a, intFix b) {
	return ((longFix)a << FIX_SHIFTS) / b;
}

intFix sqrtFix(intFix a) {	
	intFix res = 0;
	// Can be optimized, because result can never equal max i. Also, consider early exit
	for (int i = sizeof(intFix) / 2; i >= 0; i--) {
		intFix tmp = res + (1 << i);
		if (a >= tmp * tmp)
			res += 1 << i;
	}
	return res << FIX_SHIFTS / 2;
}

inline intFix float2Fix(float f) 
{
	return f * FIX_UNITY;
}

inline intFix fix2Float(float i)
{
	return (float)i / FIX_UNITY;
}

#endif /* FIXMATH_H */
