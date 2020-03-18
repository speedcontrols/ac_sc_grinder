#include <stdint.h>
#include "libfixmath/fix16.h"

// 1. Calculate σ (discrete random variable)
// 2. Drop everything with deviation > 2σ and count mean for the rest.
//
// https://upload.wikimedia.org/wikipedia/commons/8/8c/Standard_deviation_diagram.svg
//
// For efficiency, don't use root square (work with σ^2 instead)
//
// !!! count should NOT be > 16
//
// src    - uint16 array
// count  - number of elements
// window - sigma multiplier (usually [1..2])
//

static fix16_t inv_div[17] = {
    fix16_one,
    fix16_one,
    F16(1.0/2),
    F16(1.0/3),
    F16(1.0/4),
    F16(1.0/5),
    F16(1.0/6),
    F16(1.0/7),
    F16(1.0/8),
    F16(1.0/9),
    F16(1.0/10),
    F16(1.0/11),
    F16(1.0/12),
    F16(1.0/13),
    F16(1.0/14),
    F16(1.0/15),
    F16(1.0/16)
};

uint32_t truncated_mean(uint16_t *src, uint8_t count, fix16_t window)
{
    uint8_t idx = 0;

    // Count mean & sigma in one pass
    // https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
    idx = count;
    uint32_t s = 0;
    uint32_t s2 = 0;
    while (idx)
    {
        uint16_t val = src[--idx];
        s += val;
        s2 += val * val;
    }

    int mean = ((s + (count >> 1)) * inv_div[count]) >> 16;

    int sigma_square = (s2 - (s * s / count)) / (count - 1);
    // quick & dirty multiply to win^2, when win is in fix16 format.
    // we suppose win is 1..2, and sigma^2 - 24 bits max
    int sigma_win_square = ((((window >> 8) * (window >> 8)) >> 12) * sigma_square) >> 4;

    // Drop big deviations and count mean for the rest
    idx = count;
    int s_mean_filtered = 0;
    int s_mean_filtered_cnt = 0;

    while (idx)
    {
        int val = src[--idx];

        if ((mean - val) * (mean - val) < sigma_win_square)
        {
            s_mean_filtered += val;
            s_mean_filtered_cnt++;
        }
    }

    // Protection from zero div. Should never happen
    if (!s_mean_filtered_cnt) return mean;

    return ((s_mean_filtered + (s_mean_filtered_cnt >> 1)) * inv_div[s_mean_filtered_cnt]) >> 16;
}
