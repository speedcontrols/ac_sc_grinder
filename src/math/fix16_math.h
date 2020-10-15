#ifndef __FIX16_MATH__
#define __FIX16_MATH__


#include <stdint.h>
#include "libfixmath/fix16.h"


fix16_t fix16_sinusize(fix16_t x);

//
// Prior to calclate a/b - reduce bits count to use 32-bits division
// We do hat for restricted case:
//
// 1. a & b > 0
// 2. a > b
//
// That's the only possible scenario for our needs.
//
#define NORMALIZE_TO_31_BIT(a, b) while (a & 0xFFFFFFFF80000000UL) { a >>=1; b >>= 1; }

#endif
