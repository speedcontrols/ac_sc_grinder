#ifndef __CALIBRATOR_RL__
#define __CALIBRATOR_RL__

// - Measure noise, caused by current OA offset
// - Calculate motor's R.

#include <algorithm>
#include <cmath>

#include "../math/fix16_math.h"
#include "../math/stability_filter.h"

#include "../sensors.h"
#include "../triac_driver.h"

#define R_MEASURE_ATTEMPTS 3

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
      triacDriver.setpoint = 0;
      triacDriver.tick();

      r_interp_table_index = 0;

      // Pause 2 sec before calibration start to be sure
      // that motor stopped completely.
      if (ticks_cnt++ >= (2 * APP_TICK_FREQUENCY)) set_state(WAIT_ZERO_CROSS);

      break;

    case WAIT_ZERO_CROSS:
      triacDriver.tick();

      if (!sensors.zero_cross_up) break;

      buffer_idx = 0;

      set_state(RECORD_NOIZE);
      break;

    case RECORD_NOIZE:
      triacDriver.tick();

      voltage_buffer[buffer_idx] = fix16_to_float(sensors.voltage);
      current_buffer[buffer_idx] = fix16_to_float(sensors.current);
      buffer_idx++;

      if (!sensors.zero_cross_down) break;

      process_thresholds();

      set_state(R_MEASUE_LOOP);
      break;

    // Reinitialization and 0.5 sec pause
    // before next calibration step
    case R_MEASUE_LOOP:
      r_stability_filter.reset();

    case R_WAIT_STABLE_LOOP:
      buffer_idx = 0;
      zero_cross_down_offset = 0;

      triacDriver.setpoint = 0;
      triacDriver.tick();

      // Pause 0.5 sec before calibration with next setpoint value start
      // to be sure that motor stopped completely.
      if (ticks_cnt++ >= (APP_TICK_FREQUENCY / 2)) set_state(WAIT_ZERO_CROSS_2);

      break;

    // Calibration should be started at the begining of positive period
    case WAIT_ZERO_CROSS_2:
      triacDriver.tick();

      // Wait positive wave
      if (!sensors.zero_cross_up) break;

      // Fall down to recording immediately, we should not miss data for this tick
      set_state(RECORD_POSITIVE_WAVE);

    case RECORD_POSITIVE_WAVE:
      // turn on triac
      triacDriver.setpoint = sensors.cfg_r_table_setpoints[r_interp_table_index];
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
        set_state(WAIT_NEGATIVE_WAVE_2);
        break;
      }

      // Record current & emulate voltage
      voltage_buffer[buffer_idx] = - voltage_buffer[buffer_idx - zero_cross_down_offset];
      current_buffer[buffer_idx] = fix16_to_float(sensors.current);

      buffer_idx++;
      break;

    case WAIT_NEGATIVE_WAVE_2:
      triacDriver.tick();

      if (!sensors.zero_cross_down) break;

      set_state(DEMAGNETIZE);
      break;

    // Open triac on negative wave
    // with the same setpoint
    // to demagnetize motor core
    case DEMAGNETIZE:
      triacDriver.setpoint = sensors.cfg_r_table_setpoints[r_interp_table_index];
      triacDriver.tick();

      if (!sensors.zero_cross_up) break;

      set_state(CALCULATE);

      break;

    // Process collected data. That may take a lot of time, but we don't care
    // about triac at this moment
    case CALCULATE:
      triacDriver.setpoint = 0;
      triacDriver.tick();

      r_stability_filter.push(calculate_r());

      if (!r_stability_filter.is_stable())
      {
        set_state(R_WAIT_STABLE_LOOP);
        break;
      }

      r_interp_result[r_interp_table_index++] = r_stability_filter.average();

      if (r_interp_table_index < CFG_R_INTERP_TABLE_LENGTH)
      {
        set_state(R_MEASUE_LOOP);
        break;
      }

      // Write result to EEPROM
      for (int i = 0; i < CFG_R_INTERP_TABLE_LENGTH; i++)
      {
        eeprom_float_write(CFG_R_INTERP_TABLE_START_ADDR + i, r_interp_result[i]);
      }

      set_state(INIT);
      // Reload sensor's config.
      sensors.configure();
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

  // Holds current index in R interpolation table
  int r_interp_table_index = 0;
  // Holds measured R to write EEPROM all at once
  fix16_t r_interp_result[CFG_R_INTERP_TABLE_LENGTH];

  StabilityFilterTemplate<F16(5)> r_stability_filter;

  enum State {
    INIT,
    WAIT_ZERO_CROSS,
    RECORD_NOIZE,
    R_MEASUE_LOOP,
    R_WAIT_STABLE_LOOP,
    WAIT_ZERO_CROSS_2,
    RECORD_POSITIVE_WAVE,
    RECORD_NEGATIVE_WAVE,
    WAIT_NEGATIVE_WAVE_2,
    DEMAGNETIZE,
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

  float calculate_r()
  {
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

    return R;
  }
};

#endif
