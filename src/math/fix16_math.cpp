
#include <stdint.h>
#include "libfixmath/fix16.h"


// Clamp x to be in (0.0 <= x < 1.0) range
// !!! 1.0 is NOT included
static inline fix16_t fix16_clamp_zero_one(fix16_t x)
{
    return fix16_max(fix16_min(x, F16(1) - 1), 0);
}


#include "fix16_sinusize_table.h"

// Convert linear requested "energy" to Sine-wave shift (used to
// calculate triac opening phase)
// - Input: [0.0..1.0), desired energy (equivalent to 0..100%)
// - Output: [0.0..1.0)
//
// NOTE: Don-t fogret to reverse range to get real triac opening phasephase
//
fix16_t fix16_sinusize(fix16_t x)
{
    fix16_t tmp = fix16_clamp_zero_one(x);

    // 16 bits - fractional part. Need 9 bit for lookup
    return (fix16_t)(sinusize_table[ tmp >> (16 - SINUSIZE_TABLE_SIZE_BITS) ]);
}
