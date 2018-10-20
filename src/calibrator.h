#ifndef __CALIBRATOR_H__
#define __CALIBRATOR_H__

// Detect when user dials knob 3 times, start calibration sequence and
// update configuration.

#include "math/fix16_math.h"

#include "app.h"
#include "triac_driver.h"
#include "calibrator/calibrator_wait_knob_dial.h"
#include "calibrator/calibrator_static.h"
#include "calibrator/calibrator_speed.h"
#include "calibrator/calibrator_pid.h"

extern TriacDriver triacDriver;


class Calibrator
{
public:

  // Returns:
  //
  // - false: we should continue in normal mode
  // - true:  calibration started, we should stop other actions in main
  //          loop until finished.
  //
  bool tick() {

    switch (state) {

    case WAIT_START_CONDITION:
      // Wait until user dials knob 3 times to start calibration
      if (wait_knob_dial.tick()) {
        set_state(CALIBRATE_STATIC);
        return true;
      }
      return false;

    case CALIBRATE_STATIC:
      if (calibrate_static.tick()) set_state(CALIBRATE_SPEED);
      return true;

    case CALIBRATE_SPEED:
      if (calibrate_speed.tick()) set_state(CALIBRATE_PID);
      return true;

    case CALIBRATE_PID:
      if (calibrate_pid.tick()) set_state(WAIT_START_CONDITION);
      return true;
    }

    return false; // unreacheable, suppress warning
  }

private:

  enum CalibratorState {
    WAIT_START_CONDITION,
    CALIBRATE_STATIC,
    CALIBRATE_SPEED,
    CALIBRATE_PID
  } state = WAIT_START_CONDITION;

  int ticks_cnt = 0;

  void set_state(CalibratorState st) {
    state = st;
    ticks_cnt = 0;
  }

  // Nested FSM-s
  CalibratorWaitKnobDial wait_knob_dial;
  CalibratorStatic calibrate_static;
  CalibratorSpeed calibrate_speed;
  CalibratorPID calibrate_pid;
};

#endif
