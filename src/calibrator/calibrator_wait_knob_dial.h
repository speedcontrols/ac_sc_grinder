#ifndef __CALIBRATOR_WAIT_KNOB_DIAL_H__
#define __CALIBRATOR_WAIT_KNOB_DIAL_H__

// Detects when user quickly dials knob 3 times. This sequence is used
// to start calibration sequence.
//
// .tick() should be called with APP_TICK_FREQUENCY/sec, as everything else.
// It returns `true` when dials detected, and `false` in other cases.


#include "../math/fix16_math.h"

#include "../app.h"
#include "../sensors.h"


extern Sensors sensors;

#define KNOB_TRESHOLD F16(0.05)

#define IS_KNOB_LOW(val)  (val < KNOB_TRESHOLD)
#define IS_KNOB_HIGH(val) (val >= KNOB_TRESHOLD)

constexpr int knob_wait_min = APP_TICK_FREQUENCY * 0.2;
constexpr int knob_wait_max = APP_TICK_FREQUENCY * 1.0;


class CalibratorWaitKnobDial
{
public:

  bool tick() {
    fix16_t knob = sensors.knob;

    switch (state) {

    // First, check that knob is zero,
    // prior to start detect dial sequence
    case IDLE:
      if (IS_KNOB_LOW(knob))
      {
        // If knob is zero long enougth - go to 0 -> 1 edge detect
        ticks_cnt++;
        if (ticks_cnt > knob_wait_min) set_state(KNOB_UP_CHECK);
        break;
      }

      // On fail - reset all internal counters
      reset();
      break;

    // Wait knob "up", check that it fits to bounds,
    // and increment dials
    case KNOB_UP_CHECK:
      // Knob and counters are 0 => we are just from IDLE state, do nothing,
      if (IS_KNOB_LOW(knob) && ticks_cnt == 0) break;

      // Finally, knob is up => count time & reset state if too long
      if (IS_KNOB_HIGH(knob))
      {
        ticks_cnt++;
        if (ticks_cnt > knob_wait_max) reset();
        break;
      }

      // If we are here => knob is down again.
      // Check period and proceed on success.
      if (ticks_cnt > knob_wait_min)
      {
        dials_cnt++;

        if (dials_cnt >= 3)
        {
          // Dialed 3 times => Success!
          // Go back to idle state, but return true.
          reset();
          return true;
        }
        // Now go measue low period length
        set_state(KNOB_DOWN_CHECK);
        break;
      }

      // Not enougth long => start from the begining.
      else reset();
      break;

    case KNOB_DOWN_CHECK:
      // Measure down state length and reset if too long
      if (IS_KNOB_LOW(knob))
      {
        ticks_cnt++;
        if (ticks_cnt > knob_wait_max) reset();
        break;
      }

      // If we are here, knob if up!
      // If "down" was long enougth (but not too much) => repeat dial wait.
      if (ticks_cnt > knob_wait_min)
      {
        set_state(KNOB_UP_CHECK);
        break;
      }

      // Not enougth long => start from the begining.
      reset();
      break;
    }

    return false;
  }

private:

  enum State {
    IDLE,
    KNOB_UP_CHECK,
    KNOB_DOWN_CHECK
  } state = IDLE;

  int ticks_cnt = 0;
  int dials_cnt = 0;

  // Helpers to change state and reset counters in one call.

  void set_state(State st)
  {
    state = st;
    ticks_cnt = 0;
  }
  void reset()
  {
    dials_cnt = 0;
    set_state(IDLE);
  }

};

#endif
