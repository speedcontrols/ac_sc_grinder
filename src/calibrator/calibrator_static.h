#ifndef __CALIBRATOR_RL__
#define __CALIBRATOR_RL__

// - Measure noise, caused by current OA offset
// - Calculate motor's R.

#include "../math/fix16_math.h"
#include "../math/stability_filter.h"

#include "../app.h"

#define R_MEASURE_ATTEMPTS 3

// Array size to record up to half of current & voltage positive wave.
// For 50/60Hz, worst case + some reserve is (APP_TICK_FREQUENCY / 49).
constexpr int calibrator_rl_buffer_length = APP_TICK_FREQUENCY / 49;


class CalibratorStatic
{
public:

    bool tick(io_data_t &io_data) {
        YIELDABLE;

        //
        // Reset variables and wait 2 sec to make sure motor stopped.
        //

        io.setpoint = 0;
        r_interp_table_index = 0;

        YIELD_WHILE((ticks_cnt++ < (2 * APP_TICK_FREQUENCY)), false);

        YIELD_UNTIL(io_data.zero_cross_up, false);

        buffer_idx = 0;

        //
        // Record noise
        //

        while (!io_data.zero_cross_down) {
            YIELD(false);

            // TODO: bounds check
            voltage_buffer[buffer_idx] = fix16_to_float(io_data.voltage);
            current_buffer[buffer_idx] = fix16_to_float(io_data.current);
            buffer_idx++;
        }

        process_thresholds();

        //
        // Measure checkpoints
        //

        while (r_interp_table_index < CFG_R_INTERP_TABLE_LENGTH)
        {
            r_stability_filter.reset();

            //
            // Measure single checkpoint until result stable
            // Reinitialize and 0.5 sec pause before start
            //
            while (!r_stability_filter.is_stable())
            {
                buffer_idx = 0;
                zero_cross_down_offset = 0;

                io.setpoint = 0;

                YIELD_WHILE((ticks_cnt++ < (APP_TICK_FREQUENCY / 2)), false);

                // Calibration should be started at the begining of positive period
                YIELD_UNTIL(io_data.zero_cross_up, false);

                //
                // Record positive wave
                //

                io.setpoint = meter.cfg_r_table_setpoints[r_interp_table_index];

                while (!io_data.zero_cross_down) {
                    // Safety check. Restart on out of bounds.
                    if (buffer_idx >= calibrator_rl_buffer_length) return false;

                    voltage_buffer[buffer_idx] = fix16_to_float(io_data.voltage);
                    current_buffer[buffer_idx] = fix16_to_float(io_data.current);
                    buffer_idx++;

                    YIELD(false);
                }

                zero_cross_down_offset = buffer_idx;

                //
                // Record negative wave. Continue when got enough data
                // (buffer ended or next zero cross found),
                //

                io.setpoint = 0;

                while (!io_data.zero_cross_up && buffer_idx < calibrator_rl_buffer_length) {
                    // Record current & emulate voltage
                    voltage_buffer[buffer_idx] = - voltage_buffer[buffer_idx - zero_cross_down_offset];
                    current_buffer[buffer_idx] = fix16_to_float(io_data.current);
                    buffer_idx++;

                    YIELD(false);
                }

                //
                // Demagnetize core. Open triac on negative wave with the same
                // setpoint
                //

                YIELD_UNTIL(io_data.zero_cross_down, false);

                io.setpoint = meter.cfg_r_table_setpoints[r_interp_table_index];

                YIELD_UNTIL(io_data.zero_cross_up, false);

                io.setpoint = 0;

                // Process collected data. That may take a lot of time, but we don't care
                // about triac at this moment
                r_stability_filter.push(fix16_from_float(calculate_r()));

            }

            r_interp_result[r_interp_table_index++] = r_stability_filter.average();

            // Don't measure last point with setpoint = 1.0,
            // duplicate value from setpoint = 0.6
            // Measurement with setpoint = 1.0 is not accurate
            // because it causes motor to rotate.
            if (r_interp_table_index == (CFG_R_INTERP_TABLE_LENGTH - 1))
            {
                r_interp_result[r_interp_table_index++] = r_interp_result[CFG_R_INTERP_TABLE_LENGTH - 2];
            }

        }

        // Write result to EEPROM
        for (uint32_t i = 0; i < CFG_R_INTERP_TABLE_LENGTH; i++)
        {
            eeprom_float_write(
                CFG_R_INTERP_TABLE_START_ADDR + i,
                fix16_to_float(r_interp_result[i])
            );
        }

        // Reload sensor's config.
        meter.configure();
        return true;
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

    // Calculate thresholds for current and power to drop noise in speed sensor
    void process_thresholds()
    {
        float i_sum = 0;
        float p_sum = 0;

        for (uint32_t i = 0; i < buffer_idx; i++)
        {
            i_sum += current_buffer[i];
            p_sum += voltage_buffer[i] * current_buffer[i];
        }

        io.cfg_current_offset = fix16_from_float(i_sum / (float)buffer_idx);
        // Set power treshold 4x of noise value
        meter.cfg_min_power_treshold = fix16_from_float(p_sum * 4 / (float)buffer_idx);
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
