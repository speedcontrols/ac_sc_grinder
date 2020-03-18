#ifndef __TRUNCATED_MEAN__
#define __TRUNCATED_MEAN__

#include <stdint.h>
#include "libfixmath/fix16.h"


uint32_t truncated_mean(uint16_t *src, uint8_t count, fix16_t window);


#endif
