#ifndef __TRIAC_DRIVER__
#define __TRIAC_DRIVER__


#include "stm32f1xx_hal.h"
#include "math/fix16_math.h"

#include "sensors.h"

// Don't open triac at the end of wave. Helps to avoid issues if measured zero
// cross point drifted a bit.
#define TRIAC_ZERO_TAIL_LENGTH 4

class TriacDriver
{
public:
  TriacDriver(Sensors &sensors) {
    sensors_ptr = &sensors;
  }
  // 0..100% of desired triac "power".
  // Will be used to calculate opening phase for each half sine wave
  fix16_t setpoint = 0;

  // 40 kHz
  void tick()
  {
    // Propagate setpoint to sensors (required to calculate proper resistance)
    sensors_ptr->in_triac_setpoint = setpoint;

    // Poor man zero cross check
    if (sensors_ptr->zero_cross_up || sensors_ptr->zero_cross_down) rearm();

    // If period_in_ticks is not yet detected, only increment phase_counter,
    // don't touch triac.
    if (!once_period_counted)
    {
      phase_counter++;
      return;
    }

    // We keep optotriac open continiously after calculated phase shift. Pulse
    // commutation is not safe because triac current can flow after voltage
    // is zero. If we pulse at this moment, next period will be lost.
    //
    // So, we keep optotriac open until the end, and pay for it with 10ma load
    // at 3.3v.
    //
    // We should close opto-triac at the and of half-wave. To be sure - do it
    // in advance, 4 ticks before.

    if ((triac_open_done && !triac_close_done) &&
        (phase_counter + TRIAC_ZERO_TAIL_LENGTH >= positive_period_in_ticks)) {
      triac_close_done = true;
      triac_ignition_off();
    }

    // If ignition was not yet activated - check if we can do this
    if (!triac_open_done) {
      // "Linearize" setpoint to phase shift & scale to 0..1
      fix16_t normalized_setpoint = fix16_sinusize(setpoint);

      // Calculate ticks treshold when ignition should be enabled:
      // "mirror" and "enlarge" normalized setpoint
      uint32_t ticks_threshold = fix16_to_int(
        (fix16_one - normalized_setpoint) * positive_period_in_ticks
      );

      // We can open triack if:
      //
      // 1. Required phase shift found
      // 2. Tail is not too small (last 4 ticks are dead for safety)

      if ((phase_counter >= ticks_threshold) &&
          (phase_counter + TRIAC_ZERO_TAIL_LENGTH < positive_period_in_ticks)) {
        triac_open_done = true;
        triac_ignition_on();
      }
    }

    phase_counter++;
  }

private:
  // Reference to sensors, for "reactive" update of triac state info
  Sensors *sensors_ptr;

  uint32_t phase_counter = 0; // increment every tick
  bool triac_open_done = false;
  bool triac_close_done = false;

  // Holds measured number of ticks per positive half-period
  uint32_t positive_period_in_ticks = 0;

  bool once_zero_crossed = false;
  bool once_period_counted = false;

  // Helpers to switch triac and update related data.
  void inline triac_ignition_on() {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
  }
  void inline triac_ignition_off() {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
  }


  // Happens on every zero cross
  void rearm()
  {
    if (once_zero_crossed) once_period_counted = true;

    once_zero_crossed = true;

    // If full half-period was counted at least once, save number of
    // ticks in half-period
    if (once_period_counted)
    {
      // Measure period on positive half wave only
      if (sensors_ptr->zero_cross_down) positive_period_in_ticks = phase_counter;
    }

    phase_counter = 0;
    triac_open_done = false;
    triac_close_done = false;

    // Make sure to disable triac signal, if reset (zero cross) happens
    // immediately after triac enabled
    triac_ignition_off();
  }
};


#endif
