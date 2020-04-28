#ifndef __STABILITY_FILTER_TEMPLATE__
#define __STABILITY_FILTER_TEMPLATE__


#include "fix16_math.h"
#include "median.h"


#define STABILITY_FILTER_LENGTH 3

// Sliding tracker to wait until input value become stable.
//
// 1. Apply median filter first (if enabled).
// 2. Test deviation of median filter output.
//
template <fix16_t PRECISION_IN_PERCENTS, uint8_t MEDIAN_LEN = 1, int32_t MAX_TICKS = -1>
class StabilityFilterTemplate {

public:
    StabilityFilterTemplate() {
        reset();
    }

    void reset()
    {
        head_idx = 0;
        data_count = 0;
        median_filter.reset();
        ticks_count = 0;
        median_count = 0;
    }

    void push(fix16_t val) {
        // Skip median filter if too short
        if (MEDIAN_LEN <= 1) {
            ticks_count++;
            data[head_idx++] = val;
            if (head_idx == STABILITY_FILTER_LENGTH) head_idx = 0;
            data_count++;
            return;
        }

        median_filter.add(val);
        ticks_count++;
        median_count++;

        if (median_count >= MEDIAN_LEN)
        {
            data[head_idx++] = median_filter.result();
            if (head_idx == STABILITY_FILTER_LENGTH) head_idx = 0;
            data_count++;
            median_count = 0;
            median_filter.reset();
        }
    }

    bool is_stable() {
        if (data_count < STABILITY_FILTER_LENGTH) return false;

        // Temporary hardcoded for fixed length (3)
        fix16_t a = data[0], b = data[1], c = data[2];

        fix16_t min = (a <= b && a <= c) ? a : ((b <= a && b <= c) ? b : c);
        fix16_t max = (a >= b && a >= c) ? a : ((b >= a && b >= c) ? b : c);

        fix16_t diff = max - min;

        fix16_t abs_max = max > 0 ? max : - max;
        fix16_t abs_diff = diff > 0 ? diff : - diff;

        if (fix16_mul(abs_max, edge_multiplier) < abs_diff) return false;

        return true;
    }

    bool is_exceeded() {
        return MAX_TICKS > 0 && ticks_count > MAX_TICKS;
    }

    bool is_stable_or_exceeded() {
      return is_exceeded() || is_stable();
    }

    fix16_t average() {
        return fix16_mul(data[0] + data[1] + data [2], F16(1.0 / STABILITY_FILTER_LENGTH));
    }


private:
    fix16_t data[STABILITY_FILTER_LENGTH];
    int head_idx;
    int data_count;
    int ticks_count;
    int median_count;

    fix16_t edge_multiplier = PRECISION_IN_PERCENTS / 100;

    MedianIteratorTemplate<fix16_t, MEDIAN_LEN >= 2 ? MEDIAN_LEN : 2> median_filter;
};

#endif
