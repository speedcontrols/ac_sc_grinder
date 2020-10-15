#ifndef __METER__
#define __METER__

#include "math/fix16_math.h"
#include "math/truncated_mean.h"
#include "math/median.h"
#include "config_map.h"
#include "app_hal.h"
#include "app.h"


/*
    Meter. Process raw data to calculate virtual params:

    - speed
*/

class Meter
{
public:

    fix16_t speed = 0;
    bool is_r_calibrated = false;

    // Config info
    fix16_t cfg_rekv_to_speed_factor;

    // Calibration params. Non needed on real work
    fix16_t cfg_min_power_treshold = 0;

    // Input from triac driver to reflect triac setpoint. Needed for speed measure
    // to interpolate motor resistance. Autoupdated by triac driver.
    //fix16_t in_triac_setpoint = 0;

    const fix16_t cfg_r_table_setpoints[CFG_R_INTERP_TABLE_LENGTH] = {
        F16(0.1),
        F16(0.15),
        F16(0.2),
        F16(0.3),
        F16(0.4),
        F16(0.6),
        F16(1.0),
    };

    fix16_t cfg_r_table_setpoints_inerp_inv[CFG_R_INTERP_TABLE_LENGTH] = {};

    // Should be called with 40kHz frequency
    void tick(io_data_t &io_data)
    {
        speed_tick(io_data);
    }

    // Load config from emulated EEPROM
    void configure()
    {
        cfg_rekv_to_speed_factor = fix16_from_float(
            eeprom_float_read(CFG_REKV_TO_SPEED_FACTOR_ADDR, CFG_REKV_TO_SPEED_FACTOR_DEFAULT)
        );

        #define R_CAL_CHECK_MARKER 123456789.0f

        for (int i = 0; i < CFG_R_INTERP_TABLE_LENGTH; i++)
        {
            cfg_r_table[i] = fix16_from_float(
                eeprom_float_read(
                    i + CFG_R_INTERP_TABLE_START_ADDR,
                    R_CAL_CHECK_MARKER
                )
            );
        }

        // Pre-calclate inverted coeff-s to avoid divisions
        // in `get_motor_resistance()` 
        cfg_r_table_setpoints_inerp_inv[0] = fix16_one;
        for (int i = 1; i < CFG_R_INTERP_TABLE_LENGTH; i++)
        {
            if (cfg_r_table[i] - cfg_r_table[i - 1] > 0)
            {
                cfg_r_table_setpoints_inerp_inv[i] = fix16_div(
                    fix16_one,
                    cfg_r_table_setpoints[i] - cfg_r_table_setpoints[i - 1]
                );
            }
            else cfg_r_table_setpoints_inerp_inv[i] = fix16_one;
        }

        is_r_calibrated = (cfg_r_table[0] == fix16_from_float(R_CAL_CHECK_MARKER)) ? false : true;

        reset_state();
    }

    void reset_state()
    {
        speed = 0;

        p_sum_2e32 = 0;
        i2_sum_2e32 = 0;
        sum_counter = 0;

        io.out.clear();
    }

private:
    // Motor resistance interpolation table
    fix16_t cfg_r_table[CFG_R_INTERP_TABLE_LENGTH];
    fix16_t cfg_r_interp_scale_inv_table[CFG_R_INTERP_TABLE_LENGTH];

    fix16_t get_motor_resistance(fix16_t setpoint)
    {
        // Bounds check
        if (setpoint < cfg_r_table_setpoints[0]) return cfg_r_table[0];

        if (setpoint >= fix16_one) return cfg_r_table[CFG_R_INTERP_TABLE_LENGTH - 1];

        for (int i = 0; i < CFG_R_INTERP_TABLE_LENGTH - 1; i++)
        {
            if ((setpoint >= cfg_r_table_setpoints[i]) && (setpoint < cfg_r_table_setpoints[i + 1]))
            {
                fix16_t range_start = cfg_r_table[i];
                fix16_t range_end = cfg_r_table[i + 1];
                fix16_t scale = fix16_mul(
                    setpoint - cfg_r_table_setpoints[i],
                    cfg_r_table_setpoints_inerp_inv[i + 1]
                );

                return fix16_mul(range_start, fix16_one - scale) + fix16_mul(range_end, scale);
            }
        }

        return cfg_r_table[0];
    }

    int64_t p_sum_2e32 = 0;  // active power << 32
    uint64_t i2_sum_2e32 = 0; // square of current << 32
    uint16_t sum_counter = 0;


    void speed_tick(io_data_t &io_data)
    {
        // Don't try to calculate speed until R calibrated
        if (!is_r_calibrated)
        {
            speed = 0;
            return;
        }

        // Calculate sums
        // use 64 bits to prevent overflow (but sill keep dsired precision)
        p_sum_2e32 += (int64_t)io_data.voltage * io_data.current;
        i2_sum_2e32 += (uint64_t)io_data.current * io_data.current;
        sum_counter++;

        // Calculate speed at end of negative half-wave
        // In this case active power is equivalent to
        // Joule power, P = R * I^2
        // R = P / I^2
        // r_ekv is equivalent resistance created by back-EMF
        // r_ekv = R - R_motor
        if (io_data.zero_cross_up)
        {
            if (p_sum_2e32 < 0) p_sum_2e32 = 0;

            uint64_t p = p_sum_2e32, i2 = i2_sum_2e32;

            NORMALIZE_TO_31_BIT(p, i2);

            fix16_t r_ekv = fix16_div((fix16_t)p, (fix16_t)i2) - get_motor_resistance(io.setpoint);

            speed = fix16_div(r_ekv, cfg_rekv_to_speed_factor);

            // Attempt to drop noise on calibration phase
            if ((p_sum_2e32 >> 16) < ((int64_t)cfg_min_power_treshold * sum_counter)) speed = 0;

            // Clamp calculated speed value, speed can't be negative
            if (speed < 0) speed = 0;

            p_sum_2e32 = 0;
            i2_sum_2e32 = 0;
            sum_counter = 0;
        }
    }
};


#endif
