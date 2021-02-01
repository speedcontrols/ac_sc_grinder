#ifndef __SPEED_CONTROLLER__
#define __SPEED_CONTROLLER__


#include "app.h"
#include "config_map.h"
#include "math/fix16_math.h"

// ADRC iteration frequency, Hz. To fit math in fix16 without overflow.
// Observers in ADRC system must have performance much higher
// than motor performance, so set iteration frequency to 1000 Hz
#define APP_ADRC_FREQUENCY 1000

// b0 = K/T, where K=1 due to speed and triac setpoint normalization,
// T - motor time constant, estimated by calibration (see calibrator_adrc.h)
#define ADRC_BO 5.0f

constexpr int freq_divisor = APP_TICK_FREQUENCY / APP_ADRC_FREQUENCY;
// Coefficient used by ADRC observers integrators
constexpr fix16_t integr_coeff = F16(1.0 / APP_ADRC_FREQUENCY);

class Regulator
{
public:
    // Output power [0..1] for triac control
    fix16_t out_power = 0;

    // ADRC coefficients

    fix16_t cfg_adrc_Kp;
    fix16_t cfg_adrc_Kobservers;
    fix16_t cfg_adrc_p_corr_coeff;
    
    fix16_t adrc_b0_inv;

    // 1-st order ADRC system inside by this article
    //   https://arxiv.org/pdf/1908.04596.pdf
    //   
    void tick(fix16_t knob, fix16_t speed)
    {
        // Downscale input frequency to avoid fixed poind overflow.
        // 40000Hz => 40Hz

        if (tick_freq_divide_counter >= freq_divisor) tick_freq_divide_counter = 0;

        if (tick_freq_divide_counter > 0)
        {
            tick_freq_divide_counter++;
            return;
        }

        tick_freq_divide_counter++;

        knob_normalized = normalize_knob(knob);

        regulator_speed_out = speed_adrc_tick(speed);
        out_power = regulator_speed_out;
    }

    // Load config from emulated EEPROM
    void configure()
    {
        cfg_dead_zone_width_norm = fix16_from_float(eeprom_float_read(CFG_DEAD_ZONE_WIDTH_ADDR,
            CFG_DEAD_ZONE_WIDTH_DEFAULT) / 100.0f);

        float _rpm_max = eeprom_float_read(CFG_RPM_MAX_ADDR, CFG_RPM_MAX_DEFAULT);

        cfg_rpm_max_limit_norm = fix16_from_float(
            eeprom_float_read(CFG_RPM_MAX_LIMIT_ADDR, CFG_RPM_MAX_LIMIT_DEFAULT) / _rpm_max
        );

        float _rpm_min_limit = eeprom_float_read(CFG_RPM_MIN_LIMIT_ADDR, CFG_RPM_MIN_LIMIT_DEFAULT);
        // Don't allow too small low limit
        // ~ 3000 for 35000 max limit
        if (_rpm_min_limit < _rpm_max * 0.085f) _rpm_min_limit = _rpm_max * 0.085f;

        cfg_rpm_min_limit_norm = fix16_from_float(_rpm_min_limit / _rpm_max);

        knob_norm_coeff =  fix16_div(
            cfg_rpm_max_limit_norm - cfg_rpm_min_limit_norm,
            fix16_one - cfg_dead_zone_width_norm
        );

        cfg_adrc_Kp = fix16_from_float(eeprom_float_read(CFG_ADRC_KP_ADDR,
            CFG_ADRC_KP_DEFAULT));
        cfg_adrc_Kobservers = fix16_from_float(eeprom_float_read(CFG_ADRC_KOBSERVERS_ADDR,
            CFG_ADRC_KOBSERVERS_DEFAULT));

        cfg_adrc_p_corr_coeff = fix16_from_float(eeprom_float_read(CFG_ADRC_P_CORR_COEFF_ADDR,
                                                             CFG_ADRC_P_CORR_COEFF_DEFAULT));

        adrc_b0_inv = F16(1.0f / ADRC_BO);

        adrc_update_observers_parameters();
        reset_state();
    }

    // Calculate internal observers parameters L1, L2
    // based on current cfg_adrc_Kobservers value
    void adrc_update_observers_parameters()
    {
        adrc_L1 = 2 * fix16_mul(cfg_adrc_Kobservers, cfg_adrc_Kp);
        adrc_L2 = fix16_mul(fix16_mul(cfg_adrc_Kobservers, cfg_adrc_Kp),
                            fix16_mul(cfg_adrc_Kobservers, cfg_adrc_Kp));
    }

    // Reset internal regulator state
    void reset_state()
    {
        adrc_speed_estimated = 0;
        adrc_correction = 0;

        regulator_speed_out = 0;
        // Skip iteration to allow meter resync
        tick_freq_divide_counter = 1;
    }

private:
    // Control dead zone width near 0, when motor should not run.
    fix16_t cfg_dead_zone_width_norm;

    // Config limits are now in normalized [0.0..1.0] form of max motor RPM.
    fix16_t cfg_rpm_max_limit_norm;
    fix16_t cfg_rpm_min_limit_norm;

    // Cache for knob normalization, calculated on config load
    fix16_t knob_norm_coeff = F16(1);
    // knob value normalized to range (cfg_rpm_min_limit..cfg_rpm_max_limit)
    fix16_t knob_normalized;

    fix16_t adrc_correction;
    fix16_t adrc_speed_estimated;

    fix16_t adrc_L1;
    fix16_t adrc_L2;

    fix16_t regulator_speed_out = 0;

    uint32_t tick_freq_divide_counter = 0;

    // Apply min/max limits to knob output
    fix16_t normalize_knob(fix16_t knob)
    {
        if (knob < cfg_dead_zone_width_norm) return 0;

        return fix16_mul(
            (knob - cfg_dead_zone_width_norm),
            knob_norm_coeff
        ) + cfg_rpm_min_limit_norm;
    }

    fix16_t speed_adrc_tick(fix16_t speed)
    {
        // 1-st order ADRC by https://arxiv.org/pdf/1908.04596.pdf (augmented)
        
        // Proportional correction signal,
        // makes reaction to motor load change
        // significantly faster
        fix16_t adrc_p_correction = fix16_mul((speed - adrc_speed_estimated), cfg_adrc_p_corr_coeff);
        
        // u0 - output of linear proportional controller in ADRC system
        fix16_t u0 = fix16_mul((knob_normalized - adrc_speed_estimated), cfg_adrc_Kp);
        // 2 state observers:
        //   - speed observer (adrc_speed_estimated)
        //   - generalized disturbance observer (adrc_correction)
        adrc_correction += fix16_mul(fix16_mul(speed - adrc_speed_estimated, adrc_L2), integr_coeff);
        adrc_speed_estimated += fix16_mul(u0 + fix16_mul(adrc_L1, (speed - adrc_speed_estimated)),
         integr_coeff);

        adrc_speed_estimated = fix16_clamp(
            adrc_speed_estimated,
            cfg_rpm_min_limit_norm,
            cfg_rpm_max_limit_norm
        );

        fix16_t output = fix16_mul((u0 - adrc_correction - adrc_p_correction), adrc_b0_inv);

        // Anti-Windup
        // 0 <= output <= 1
        //
        // output = (u0 - adrc_correction)/b0,
        // so if output = cfg_rpm_min_limit_norm -> adrc_correction = u0 - b0 * cfg_rpm_min_limit_norm
        if (output < cfg_rpm_min_limit_norm)
        {
            output = cfg_rpm_min_limit_norm;
            adrc_correction = u0 - fix16_div(cfg_rpm_min_limit_norm, adrc_b0_inv);
        }

        // output = (u0 - adrc_correction)/b0,
        // so if output = cfg_rpm_max_limit_norm -> adrc_correction = u0 - b0 * cfg_rpm_max_limit_norm
        if (output > cfg_rpm_max_limit_norm)
        {
            output = cfg_rpm_max_limit_norm;
            adrc_correction = u0 - fix16_div(cfg_rpm_max_limit_norm, adrc_b0_inv);
        }

        return output;
    }
};


#endif
