#ifndef __CALIBRATOR_RL__
#define __CALIBRATOR_RL__

// - Measure noise, caused by current OA offset
// - Calculate motor's R.

#include <algorithm>
#include <cmath>

#include "../math/fix16_math.h"

#include "../sensors.h"
#include "../triac_driver.h"

extern Sensors sensors;
extern TriacDriver triacDriver;

// Array size to record up to half of current & voltage positive wave.
// Full period is (APP_TICK_FREQUENCY / 50).
//
// I theory, we it would be enougth to save ~ 1/8 of voltage positive wave,
// and calc the rest on the fly. That will save a lot of memory, but we have
// no time for such optimization.
constexpr int calibrator_rl_buffer_length = APP_TICK_FREQUENCY / 50;


class CalibratorStatic
{
public:

  bool tick(void) {

    switch (state) {

    // Reset variables and wait 1 second to make sure motor stopped.
    case INIT:
      buffer_idx = 0;
      zero_cross_down_offset = 0;

      triacDriver.setpoint = 0;
      triacDriver.tick();

      // Pause 2 sec before calibration start to be sure
      // that motor stopped completely.
      if (ticks_cnt++ >= (2 * APP_TICK_FREQUENCY)) set_state(WAIT_ZERO_CROSS);

      break;

    case WAIT_ZERO_CROSS:
      triacDriver.tick();

      if (!sensors.zero_cross_up) break;

      set_state(RECORD_NOIZE);

      break;

    case RECORD_NOIZE:
      triacDriver.tick();

      voltage_buffer[buffer_idx] = fix16_to_float(sensors.voltage);
      current_buffer[buffer_idx] = fix16_to_float(sensors.current);
      buffer_idx++;

      if (!sensors.zero_cross_down) break;

      process_thresholds();

      buffer_idx = 0;
      set_state(WAIT_ZERO_CROSS_2);
      break;

    // Calibration should be started at the begining of positive period
    case WAIT_ZERO_CROSS_2:

      triacDriver.tick();


      // Fall down to recording immediately, we should not miss data for this tick
      set_state(RECORD_POSITIVE_WAVE);

    case RECORD_POSITIVE_WAVE:
      // turn on triac
      triacDriver.setpoint = fix16_one;
      triacDriver.tick();

      // Safety check. Restart on out of bounds.
      if (buffer_idx >= calibrator_rl_buffer_length) {
        set_state(INIT);
        break;
      }

      voltage_buffer[buffer_idx] = fix16_to_float(sensors.voltage);
      current_buffer[buffer_idx] = fix16_to_float(sensors.current);

      buffer_idx++;

      if (sensors.zero_cross_down) {
        zero_cross_down_offset = buffer_idx;
        set_state(RECORD_NEGATIVE_WAVE);
      }

      break;

    case RECORD_NEGATIVE_WAVE:
      // turn off triac
      triacDriver.setpoint = 0;
      triacDriver.tick();

      // If got enougth data (buffer ended or next zero cross found),
      // go to data processing
      if (sensors.zero_cross_up || buffer_idx >= calibrator_rl_buffer_length)
      {
        set_state(CALCULATE);
        break;
      }

      // Record current & emulate voltage
      voltage_buffer[buffer_idx] = - voltage_buffer[buffer_idx - zero_cross_down_offset];
      current_buffer[buffer_idx] = fix16_to_float(sensors.current);

      buffer_idx++;

      break;

    // Process collected data. That may take a lot of time, but we don't care
    // about triac at this moment
    case CALCULATE:
      process_data();
      set_state(INIT);

      return true;
    }

    return false;
  }

private:

  float voltage_buffer[calibrator_rl_buffer_length];
  float current_buffer[calibrator_rl_buffer_length];

  float i_avg = 0;

  uint32_t buffer_idx = 0;
  uint32_t zero_cross_down_offset = 0;

  MedianIteratorTemplate<float, 32> median_filter;

  enum State {
    INIT,
    WAIT_ZERO_CROSS,
    RECORD_NOIZE,
    WAIT_ZERO_CROSS_2,
    RECORD_POSITIVE_WAVE,
    RECORD_NEGATIVE_WAVE,
    CALCULATE
  } state = INIT;

  int ticks_cnt = 0;

  void set_state(State st)
  {
    ticks_cnt = 0;
    state = st;
  }

  // Calculate thresolds for current and power to drop noise in speed sensor
  void process_thresholds()
  {
    float i_sum = 0;
    float p_sum = 0;

    for (uint i = 0; i < buffer_idx; i++)
    {
      i_sum += current_buffer[i];
      p_sum += voltage_buffer[i] * current_buffer[i];
    }

    sensors.cfg_current_offset = fix16_from_float(i_sum / buffer_idx);
    // Set power treshold 4x of noise value
    sensors.cfg_min_power_treshold = fix16_from_float(p_sum * 4 / buffer_idx);
  }

  void process_data()
  {
    //
    // Process R
    //

    float p_sum = 0;  // active power
    float i2_sum = 0; // square of current

    for (uint32_t i = 0; i < buffer_idx; i++)
    {
      p_sum += voltage_buffer[i] * current_buffer[i];
      i2_sum += current_buffer[i] * current_buffer[i];
    }

    // Active power is equal to Joule power in this case
    // Current^2 * R = P
    // R = P / Current^2
    float R = p_sum / i2_sum;

    eeprom_float_write(CFG_MOTOR_RESISTANCE_ADDR, R);

    // Reload sensor's config.
    sensors.configure();
  }
};

#endif
