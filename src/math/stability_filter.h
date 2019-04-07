#ifndef __STABILITY_FILTER_TEMPLATE__
#define __STABILITY_FILTER_TEMPLATE__


#include "./fix16_math.h"

#define STABILITY_FILTER_LENGTH 3

// Sliding tracker to wait until input value become stable.
//
template <fix16_t PRECISION_IN_PERCENTS>
class StabilityFilterTemplate {

public:
  StabilityFilterTemplate() {
    reset();
  }

  void reset()
  {
    head_idx = 0;
    data_count = 0;
  }

  void push(fix16_t val) {
    data[head_idx++] = val;
    if (head_idx == STABILITY_FILTER_LENGTH) head_idx = 0;
    data_count++;
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

  fix16_t average() {
    return (data[0] + data[1] + data [2]) / STABILITY_FILTER_LENGTH;
  }

private:
  fix16_t data[STABILITY_FILTER_LENGTH];
  int head_idx;
  int data_count;

  fix16_t edge_multiplier = PRECISION_IN_PERCENTS / 100;
};

#endif
