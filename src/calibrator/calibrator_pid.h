#ifndef __CALIBRATOR_PID__
#define __CALIBRATOR_PID__

#include "../math/fix16_math.h"
#include "../app.h"
#include "../sensors.h"
#include "../triac_driver.h"
#include "../speed_controller.h"

#include <limits.h>
#include <cmath>

extern Sensors sensors;
extern TriacDriver triacDriver;
extern SpeedController speedController;

#define MIN_P 1.0
#define INIT_P_ITERATION_STEP 4.0

// Scale down PID to this value for safety
#define PID_SAFETY_SCALE 0.75

#define PID_P_SETPOINT 0.3
#define PID_I_OVERSHOOT_SETPOINT 0.8
#define PID_I_START_SETPOINT 0.3

// Maximum speed oscillation amplitude
// and speed overshoot values
// for PID_P and PID_I adjustment
#define MAX_AMPLITUDE 1.1
#define MAX_OVERSHOOT 0.15

// Setpoint values for start and
// stop time measurement.
#define LOW_SPEED_SETPOINT 0.35
#define HIGH_SPEED_SETPOINT 0.7

// Speed data is noisy, so start/stop
// time is measured when speed crosses
// SPEED_MEASURE_THRESHOLD and then recalculated
// to more narrow SPEED_IDEAL_THRESHOLD.
#define SPEED_MEASURE_THRESHOLD 0.15
#define SPEED_IDEAL_THRESHOLD 0.02

// Assume that motor is 1-st order dynamic system.
// It will cross SPEED_IDEAL_THRESHOLD
// at ideal time = measured time * start_stop_adjust
static const fix16_t start_stop_adjust = fix16_from_float(
  (float)(log(SPEED_IDEAL_THRESHOLD) / log(SPEED_MEASURE_THRESHOLD))
);

// Frequency scale for data save when measure start/stop time.
// Helps to save used memory
#define SPEED_DATA_SAVE_RATIO 10

// Max 10 seconds to store data on start/stop time measure
#define SPEED_DATA_SAVE_TIME_MAX 10

// Lowpass coefficient value
// LOWPASS_FC - cutoff frequency, Hz
// LOWPASS_FD - sampling frequency, Hz
#define LOWPASS_FC 5.0
#define LOWPASS_FS 50.0
#define LOWPASS_FILTER_ALPHA ((2.0 * M_PI / LOWPASS_FS * LOWPASS_FC) \
                              / (2.0 * M_PI / LOWPASS_FS * LOWPASS_FC + 1.0))

class CalibratorPID {
public:

  bool tick() {
    switch (state)
    {
      // Before start time measure
      // motor must run at steady low speed
      case INIT_WAIT_STABLE_LOW_SPEED:
        setpoint = F16(LOW_SPEED_SETPOINT);

        triacDriver.setpoint = setpoint;

        // Init local iteration vars.
        wait_stable_speed_init();
        set_state(WAIT_STABLE_LOW_SPEED);

      case WAIT_STABLE_LOW_SPEED:
        triacDriver.tick();
        if (!sensors.zero_cross_up) break;

        wait_stable_speed(sensors.speed, INIT_MEASURE_START_TIME, INT_MAX);
        break;

      case INIT_MEASURE_START_TIME:
        setpoint = F16(HIGH_SPEED_SETPOINT);

        triacDriver.setpoint = setpoint;

        // Init local iteration vars.
        wait_stable_speed_init();
        speed_data_idx = 0;
        start_time_ticks = 0;
        start_stop_scaler_cnt = 0;
        set_state(WAIT_STABLE_SPEED_UP);

      case WAIT_STABLE_SPEED_UP:
        triacDriver.tick();
        start_time_ticks++;

        if (!sensors.zero_cross_up) break;

        if (speed_data_idx < speed_data_length)
        {
          // Apply lowpass filtration
          filtered_speed = fix16_mul(
            F16(LOWPASS_FILTER_ALPHA),
            sensors.speed
          ) + fix16_mul(
            F16(1.0 - LOWPASS_FILTER_ALPHA),
            (speed_data_idx == 0) ? sensors.speed : filtered_speed
          );

          if (start_stop_scaler_cnt == 0) {
            start_speed_data[speed_data_idx++] = filtered_speed;
          }

          start_stop_scaler_cnt++;
          if (start_stop_scaler_cnt == SPEED_DATA_SAVE_RATIO) start_stop_scaler_cnt = 0;
        }

        wait_stable_speed(sensors.speed, INIT_MEASURE_STOP_TIME, INT_MAX);

        break;

      case INIT_MEASURE_STOP_TIME:
        start_speed_data_len = speed_data_idx;

        setpoint = F16(LOW_SPEED_SETPOINT);
        triacDriver.setpoint = setpoint;

        // Init local iteration vars.
        wait_stable_speed_init();
        speed_data_idx = 0;
        stop_time_ticks = 0;
        start_stop_scaler_cnt = 0;
        set_state(WAIT_STABLE_SPEED_DOWN);

      case WAIT_STABLE_SPEED_DOWN:
        triacDriver.tick();
        stop_time_ticks++;

        if (!sensors.zero_cross_up) break;

        if (speed_data_idx < speed_data_length)
        {
          // Apply lowpass filtration
          filtered_speed = fix16_mul(
            F16(LOWPASS_FILTER_ALPHA),
            sensors.speed
          ) + fix16_mul(
            F16(1.0 - LOWPASS_FILTER_ALPHA),
            (speed_data_idx == 0) ? sensors.speed : filtered_speed
          );

          if (start_stop_scaler_cnt == 0) {
            stop_speed_data[speed_data_idx++] = filtered_speed;
          }

          start_stop_scaler_cnt++;
          if (start_stop_scaler_cnt == SPEED_DATA_SAVE_RATIO) start_stop_scaler_cnt = 0;

        }

        wait_stable_speed(sensors.speed, INIT_P_CALIBRATION, INT_MAX);

        break;

      case INIT_P_CALIBRATION:
        stop_speed_data_len = speed_data_idx;
        motor_start_stop_time = calculate_start_stop_time(
          start_speed_data_len,
          start_speed_data,
          stop_speed_data_len,
          stop_speed_data
        );

        iterations_count = 0;
        iteration_step = F16(INIT_P_ITERATION_STEP);

        pid_param_attempt_value = F16(MIN_P);

        // TODO - Measure period duration for correct operation at 50 and 60 Hz
        measure_amplitude_ticks_max = fix16_to_int(motor_start_stop_time * 50);

        speedController.cfg_pid_i_inv = fix16_div(
          F16(1.0 / APP_PID_FREQUENCY),
          motor_start_stop_time
        );

        set_state(WAIT_STABLE_SPEED_P_CALIBRATION_START);

      case WAIT_STABLE_SPEED_P_CALIBRATION_START:
        wait_stable_speed_init();
        set_state(WAIT_STABLE_SPEED_P_CALIBRATION_CYCLE);

      case WAIT_STABLE_SPEED_P_CALIBRATION_CYCLE:
        // Wait for stable speed with minimal
        // PID_P and maximal PID_I
        speedController.cfg_pid_p = F16(MIN_P);
        speedController.cfg_pid_i_inv = fix16_div(
          F16(1.0 / APP_PID_FREQUENCY),
          motor_start_stop_time
        );

        speedController.in_knob = F16(PID_P_SETPOINT);

        speedController.in_speed = sensors.speed;
        speedController.tick();
        triacDriver.setpoint = speedController.out_power;
        triacDriver.tick();

        if (!sensors.zero_cross_up) break;

        wait_stable_speed(sensors.speed, MEASURE_AMPLITUDE_START, 13);

        break;

      case MEASURE_AMPLITUDE_START:
        speedController.cfg_pid_p = pid_param_attempt_value;
        measure_amplitude_max_speed = 0;
        measure_amplitude_min_speed = fix16_maximum;
        measure_amplitude_ticks = 0;
        median_filter.reset();
        set_state(MEASURE_AMPLITUDE_CYCLE);

      case MEASURE_AMPLITUDE_CYCLE:
        speedController.in_knob = F16(PID_P_SETPOINT);
        speedController.in_speed = sensors.speed;
        speedController.tick();
        triacDriver.setpoint = speedController.out_power;
        triacDriver.tick();

        if (!sensors.zero_cross_up) break;

        ticks_cnt++;
        median_filter.add(sensors.speed);

        if (ticks_cnt >= 12)
        {
          fix16_t filtered_speed = median_filter.result();
          median_filter.reset();
          ticks_cnt = 0;

          if (measure_amplitude_max_speed < filtered_speed)
          {
            measure_amplitude_max_speed = filtered_speed;
          }
          if (measure_amplitude_min_speed > filtered_speed)
          {
            measure_amplitude_min_speed = filtered_speed;
          }
        }

        measure_amplitude_ticks++;

        if (measure_amplitude_ticks >= measure_amplitude_ticks_max)
        {
          fix16_t amplitude = measure_amplitude_max_speed - measure_amplitude_min_speed;

          // Save amplitude of first iteration as reference
          // to compare values of next iterations to this value
          if (iterations_count == 0) first_iteration_amplitude = amplitude;

          // If amplitude is less than margin value
          // step for next iteration should be positive,
          // otherwise - negative
          if (amplitude <= fix16_mul(first_iteration_amplitude, F16(MAX_AMPLITUDE)))
          {
            iteration_step = abs(iteration_step);
          }
          else iteration_step = -abs(iteration_step);

          pid_param_attempt_value += iteration_step;

          iteration_step /= 2;
          iterations_count++;
          if (iterations_count >= max_iterations)
          {
            pid_p_calibrated_value = fix16_mul(pid_param_attempt_value, F16(PID_SAFETY_SCALE));
            set_state(INIT_I_CALIBRATION);
            break;
          }
          set_state(WAIT_STABLE_SPEED_P_CALIBRATION_START);
        }

        break;

      case INIT_I_CALIBRATION:
        iterations_count = 0;
        // Start calibration with safe
        // PID_I value = motor_start_stop_time
        // Motor will work without overshoot
        // with this value.
        iteration_step = -(motor_start_stop_time / 2);

        pid_param_attempt_value = motor_start_stop_time;
        first_iteration_overshoot_speed = 0;

        // Set overshoot measurement time interval
        // to motor_start_stop_time
        // for reability of measurement.
        measure_overshoot_ticks_max = fix16_to_int(motor_start_stop_time * 50);

        speedController.cfg_pid_p = pid_p_calibrated_value;

        set_state(WAIT_STABLE_SPEED_I_CALIBRATION_START);

      case WAIT_STABLE_SPEED_I_CALIBRATION_START:
        wait_stable_speed_init();
        set_state(WAIT_STABLE_SPEED_I_CALIBRATION_CYCLE);

      case WAIT_STABLE_SPEED_I_CALIBRATION_CYCLE:
        // Wait for stable speed with minimal
        // PID_P and maximal PID_I
        speedController.cfg_pid_p = F16(MIN_P);
        speedController.cfg_pid_i_inv = fix16_div(
          F16(1.0 / APP_PID_FREQUENCY),
          motor_start_stop_time / 2
        );

        speedController.in_knob = F16(PID_I_START_SETPOINT);

        speedController.in_speed = sensors.speed;
        speedController.tick();
        triacDriver.setpoint = speedController.out_power;
        triacDriver.tick();

        if (!sensors.zero_cross_up) break;

        wait_stable_speed(sensors.speed, MEASURE_OVERSHOOT_START, 13);

        break;

      case MEASURE_OVERSHOOT_START:
        speedController.cfg_pid_p = pid_p_calibrated_value;
        speedController.cfg_pid_i_inv = fix16_div(
          F16(1.0 / APP_PID_FREQUENCY),
          pid_param_attempt_value
        );

        measure_overshoot_ticks = 0;
        overshoot_speed = 0;
        set_state(MEASURE_OVERSHOOT_CYCLE);

      case MEASURE_OVERSHOOT_CYCLE:
        speedController.in_knob = F16(PID_I_OVERSHOOT_SETPOINT);
        speedController.in_speed = sensors.speed;
        speedController.tick();
        triacDriver.setpoint = speedController.out_power;
        triacDriver.tick();

        if (!sensors.zero_cross_up) break;

        measure_overshoot_ticks++;

        // Apply lowpass filter to speed data
        filtered_speed = fix16_mul(
          F16(LOWPASS_FILTER_ALPHA),
          sensors.speed
        ) + fix16_mul(
          F16(1.0 - LOWPASS_FILTER_ALPHA),
          (measure_overshoot_ticks == 1) ? sensors.speed : filtered_speed
        );

        // Find max speed value
        if (overshoot_speed < filtered_speed) overshoot_speed = filtered_speed;
        if (measure_overshoot_ticks >= measure_overshoot_ticks_max)
        {
          // Save overshoot of first iteration as reference
          // to compare next iterations values to this value
          if (iterations_count == 0) first_iteration_overshoot_speed = overshoot_speed;

          fix16_t overshoot = fix16_div(
            overshoot_speed - first_iteration_overshoot_speed,
            first_iteration_overshoot_speed
          );

          // If overshoot is greater than margin value
          // step for next iteration should be positive,
          // otherwise - negative
          if (overshoot > F16(MAX_OVERSHOOT)) iteration_step = abs(iteration_step);
          else iteration_step = -abs(iteration_step);

          pid_param_attempt_value += iteration_step;

          iteration_step /= 2;
          iterations_count++;
          if (iterations_count >= max_iterations)
          {
            pid_i_calibrated_value = fix16_div(pid_param_attempt_value, F16(PID_SAFETY_SCALE));
            set_state(STOP);
            break;
          }
          set_state(WAIT_STABLE_SPEED_I_CALIBRATION_START);
        }

        break;

      case STOP:
        // Store in flash
        eeprom_float_write(
          CFG_PID_P_ADDR,
          fix16_to_float(pid_p_calibrated_value)
        );
        eeprom_float_write(
          CFG_PID_I_ADDR,
          fix16_to_float(pid_i_calibrated_value)
        );
        speedController.configure();
        set_state(INIT_WAIT_STABLE_LOW_SPEED); // Reset state to initial
        return true;
    }
    return false;
  }

private:

  enum State {
    INIT_WAIT_STABLE_LOW_SPEED,
    WAIT_STABLE_LOW_SPEED,
    INIT_MEASURE_START_TIME,
    WAIT_STABLE_SPEED_UP,
    INIT_MEASURE_STOP_TIME,
    WAIT_STABLE_SPEED_DOWN,
    INIT_P_CALIBRATION,
    WAIT_STABLE_SPEED_P_CALIBRATION_START,
    WAIT_STABLE_SPEED_P_CALIBRATION_CYCLE,
    MEASURE_AMPLITUDE_START,
    MEASURE_AMPLITUDE_CYCLE,
    INIT_I_CALIBRATION,
    WAIT_STABLE_SPEED_I_CALIBRATION_START,
    WAIT_STABLE_SPEED_I_CALIBRATION_CYCLE,
    MEASURE_OVERSHOOT_START,
    MEASURE_OVERSHOOT_CYCLE,
    STOP
  } state = INIT_WAIT_STABLE_LOW_SPEED;

  // Desireable accuracy of PID calibration is 0.1
  // We need 7 iterations to achieve this accuracy
  // because (10 - 1)/2^7 = 0.07 < 0.1
  enum {
    max_iterations = 7
  };

  int ticks_cnt = 0;

  // Holds buffer length for start/stop time calculation
  enum {
    speed_data_length = (int)(SPEED_DATA_SAVE_TIME_MAX * 60 / SPEED_DATA_SAVE_RATIO)
  };

  // Buffers for speed data for start/stop
  // time measurement
  fix16_t start_speed_data[speed_data_length];
  fix16_t stop_speed_data[speed_data_length];
  int speed_data_idx = 0;
  int start_time_ticks = 0;
  int stop_time_ticks = 0;
  int start_stop_scaler_cnt;
  fix16_t filtered_speed = 0;

  fix16_t prev_speed = 0;

  int start_speed_data_len;
  int stop_speed_data_len;

  fix16_t pid_param_attempt_value;

  fix16_t pid_p_calibrated_value;
  fix16_t pid_i_calibrated_value;

  fix16_t iteration_step;

  fix16_t measure_amplitude_max_speed;
  fix16_t measure_amplitude_min_speed;

  // Holds value calculated during first attempt
  fix16_t first_iteration_amplitude;

  int iterations_count = 0;

  fix16_t setpoint = 0;

  // History of measured speed. Used to detect stable values.
  StabilityFilterTemplate<F16(1.0)> speed_tracker;

  int measure_attempts = 0;

  MedianIteratorTemplate<fix16_t, 32> median_filter;
  fix16_t motor_start_stop_time;

  int measure_amplitude_ticks = 0;
  int measure_amplitude_ticks_max;

  int measure_overshoot_ticks = 0;
  // Measure overshoot during period
  // equal to motor start/stop time.
  // One tick is 0.02 sec if power supply frequency is 50 Hz
  int measure_overshoot_ticks_max;

  fix16_t overshoot_speed = 0;
  fix16_t first_iteration_overshoot_speed = 0;

  void set_state(State st)
  {
    state = st;
    ticks_cnt = 0;
  }

  // Returns sum of start and stop times
  fix16_t calculate_start_stop_time(
    int start_len,
    fix16_t start_data[],
    int stop_len,
    fix16_t stop_data[]
  )
  {
    fix16_t start_steady_speed = start_data[start_len - 1];
    fix16_t start_initial_speed = start_data[0];
    fix16_t speed_at_measure_threshold = fix16_mul(
      start_steady_speed,
      F16(1.0 - SPEED_MEASURE_THRESHOLD)
    ) + fix16_mul(
      start_initial_speed,
      F16(SPEED_MEASURE_THRESHOLD)
    );

    fix16_t start_time_measured = 0;

    for (int i = 0; i < start_len; i++)
    {
      if (start_data[i] > speed_at_measure_threshold)
      {
        start_time_measured = fix16_from_float((float)(i * start_time_ticks) / (float)(start_len *  APP_TICK_FREQUENCY));
        break;
      }
    }

    fix16_t stop_steady_speed = stop_data[stop_len - 1];
    fix16_t stop_initial_speed = stop_data[0];
    speed_at_measure_threshold = fix16_mul(
      stop_steady_speed,
      F16(1.0 - SPEED_MEASURE_THRESHOLD)
    ) + fix16_mul(
      stop_initial_speed,
      F16(SPEED_MEASURE_THRESHOLD)
    );

    fix16_t stop_time_measured = 0;

    for (int i = 0; i < stop_len; i++)
    {
      if (stop_data[i] < speed_at_measure_threshold)
      {
        stop_time_measured = fix16_from_float((float)(i * stop_time_ticks) / (float)(stop_len  * APP_TICK_FREQUENCY));
        break;
      }
    }

    fix16_t start_stop_time_measured = start_time_measured + stop_time_measured;

    // Assume that motor is 1-st order dynamic system,
    // calculate time when it will cross SPEED_IDEAL_THRESHOLD
    fix16_t start_stop_time = fix16_mul(
      start_stop_time_measured,
      start_stop_adjust
    );

    return start_stop_time;
  }

  void wait_stable_speed_init() {
    speed_tracker.reset();
    measure_attempts = 0;
    ticks_cnt = 0;
    median_filter.reset();
  }

  void wait_stable_speed(
    fix16_t current_speed,
    State next_state_on_success,
    int max_attempts
  ) {
    median_filter.add(current_speed);
    ticks_cnt++;

    // ~ 0.25s
    if (ticks_cnt >= 12)
    {
      speed_tracker.push(median_filter.result());

      // if speed stable OR waited > 3 sec => record data

      if (measure_attempts > max_attempts) {
        set_state(next_state_on_success);
        return;
      }

      // Speed stable if difference <= 1 %.
      if (speed_tracker.is_stable()) {
        set_state(next_state_on_success);
        return;
      }

      // Prepare for next measure attempt
      measure_attempts++;
      ticks_cnt = 0;
      median_filter.reset();
    }
  }
};



#endif
