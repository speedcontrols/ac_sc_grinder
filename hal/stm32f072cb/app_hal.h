#ifndef __APP_HAL__
#define __APP_HAL__

#include <stdint.h>

// Oversampling ratio. Used to define buffer sizes
#define ADC_FETCH_PER_TICK 8

// How many channels are sampled "in parallel".
// Used to define global DMA buffer size.
#define ADC_CHANNELS_COUNT 4

// Frequency of measurements & state updates.
// Currently driven by ADC for simplicity.
#define APP_TICK_FREQUENCY 17857

namespace hal {

void setup();
void start();
void triac_ignition_on();
void triac_ignition_off();

} // namespace

#endif