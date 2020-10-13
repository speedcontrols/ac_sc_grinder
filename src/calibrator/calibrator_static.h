#ifndef __CALIBRATOR_RL__
#define __CALIBRATOR_RL__

// - Measure noise, caused by current OA offset
// - Calculate motor's R.

#include "../math/fix16_math.h"
#include "../math/stability_filter.h"

#include "../app.h"

#define R_MEASURE_ATTEMPTS 3

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

        ticks_cnt = 0;
        YIELD_WHILE((ticks_cnt++ < (2 * APP_TICK_FREQUENCY)), false);

        YIELD_UNTIL(io_data.zero_cross_up, false);

        //
        // Calculate thresholds for current and power
        // to drop noise in speed sensor
        //

        acc_counter = 0;
        acc_p_sum_2e32 = 0;
        acc_i_sum = 0;

        while (!io_data.zero_cross_down) {
            YIELD(false);

            acc_p_sum_2e32 += (int64_t)(io_data.voltage) * io_data.current;
            acc_i_sum += io_data.current;
            acc_counter++;
        }


        io.cfg_current_offset = acc_i_sum / acc_counter;
        // Set power treshold 4x of noise value
        {
            if (acc_p_sum_2e32 < 0) acc_p_sum_2e32 = 0;

            uint64_t p_sum_f16 = acc_p_sum_2e32 >> 16;
            uint32_t counter_f16 = fix16_from_int(acc_counter);

            // Normalize to 31 bit, to use fix16_div
            while (p_sum_f16 & 0xFFFFFFFF80000000UL) {
                p_sum_f16 >>= 1;
                counter_f16 >>= 1;
            }

            meter.cfg_min_power_treshold =
                fix16_div((fix16_t)p_sum_f16, (fix16_t)counter_f16) * 4;
        }


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
                acc_counter = 0;
                acc_p_sum_2e32 = 0;
                acc_i2_sum_2e32 = 0;

                io.setpoint = 0;

                ticks_cnt = 0;
                YIELD_WHILE((ticks_cnt++ < (APP_TICK_FREQUENCY / 2)), false);

                // Calibration should be started at the begining of positive period
                YIELD_UNTIL(io_data.zero_cross_up, false);

                //
                // Record positive wave
                //

                io.setpoint = meter.cfg_r_table_setpoints[r_interp_table_index];
                acc_counter = 0;

                // skip current zero point to force `while` wait next cross
                YIELD(false);

                while (!io_data.zero_cross_up)
                {
                    // Don't continue with triac on negative wave, measure only.
                    if (io_data.zero_cross_down) io.setpoint = 0;

                    
                    acc_p_sum_2e32 += (int64_t)io_data.voltage * io_data.current;
                    acc_i2_sum_2e32 += (uint64_t)io_data.current * io_data.current;
                    acc_counter++;

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

                // Active power is equal to Joule power in this case
                // Current^2 * R = P
                // R = P / Current^2
                if (acc_p_sum_2e32 < 0) acc_p_sum_2e32 = 0;

                uint64_t p = acc_p_sum_2e32, i2 = acc_i2_sum_2e32;
                // Normalize to 31 bit, to use fix16_div
                while (p & 0xFFFFFFFF80000000UL) {
                    p = p >> 1;
                    i2 = i2 >> 1;
                }

                r_stability_filter.push(fix16_div((fix16_t)p, (fix16_t)i2));
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

    float i_avg = 0;

    uint16_t acc_counter = 0;

    int64_t acc_p_sum_2e32 = 0;
    uint32_t acc_i_sum = 0;
    uint64_t acc_i2_sum_2e32 = 0;

    // Holds current index in R interpolation table
    int r_interp_table_index = 0;
    // Holds measured R to write EEPROM all at once
    fix16_t r_interp_result[CFG_R_INTERP_TABLE_LENGTH];

    StabilityFilterTemplate<F16(5)> r_stability_filter;

    int ticks_cnt = 0;
};

#endif
