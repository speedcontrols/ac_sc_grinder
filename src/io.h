#ifndef __IO__
#define __IO__


#include "math/fix16_math.h"
#include "math/truncated_mean.h"
#include "config_map.h"
#include "app.h"
#include "app_hal.h"
#include "etl/queue_spsc_atomic.h"


// Don't open triac at the end of wave. Helps to avoid issues if measured zero
// cross point drifted a bit.
#define TRIAC_ZERO_TAIL_LENGTH 4


struct io_data_t {
    fix16_t voltage = 0;
    fix16_t current = 0;
    // Speed knob physical value, 0..1
    fix16_t knob = 0;
    // Voltage zero cross flags.
    bool zero_cross_up = false;
    bool zero_cross_down = false;
};


class Io
{
public:
    // 0..100% of desired triac "power".
    // Will be used to calculate opening phase for each half sine wave
    fix16_t setpoint = 0;

    // Calibration params. Non needed on real work
    fix16_t cfg_current_offset = 0;

    // Output data to process in main loop. In theory should have 1 element max.
    // Leave room for 3 more for sure.
    etl::queue_spsc_atomic<io_data_t, 10, etl::memory_model::MEMORY_MODEL_SMALL> out;

    void configure()
    {
        // config shunt resistance - in mOhm (divide by 1000)
        // shunt amplifier gain - 50
        cfg_shunt_resistance_inv = fix16_from_float(1.0f /
            (eeprom_float_read(CFG_SHUNT_RESISTANCE_ADDR, CFG_SHUNT_RESISTANCE_DEFAULT)
            * 50
            / 1000)
        );
    }

    // Eat raw adc data, transform and propagate to triac & message queue
    void consume(
        uint16_t adc_voltage_buf[],
        uint16_t adc_current_buf[],
        uint16_t adc_knob_buf[],
        uint16_t adc_v_refin_buf[]
    )
    {
        io_data_t io_data;

        //
        // Do preliminary filtering of raw data + normalize result
        //

        // Apply filters
        uint16_t adc_voltage = (uint16_t)truncated_mean(adc_voltage_buf, ADC_FETCH_PER_TICK, F16(1.1));
        uint16_t adc_current = (uint16_t)truncated_mean(adc_current_buf, ADC_FETCH_PER_TICK, F16(1.1));
        uint16_t adc_knob = (uint16_t)truncated_mean(adc_knob_buf, ADC_FETCH_PER_TICK, F16(1.1));
        uint16_t adc_v_refin = (uint16_t)truncated_mean(adc_v_refin_buf, ADC_FETCH_PER_TICK, F16(1.1));

        // Now process the rest...

        // 4096 - maximum value of 12-bit integer
        // normalize to fix16_t[0.0..1.0]
        fix16_t knob_new = adc_knob << 4;

        // Use additional mean smoother for knob
        io_data.knob = (prev_knob * 15 + knob_new) >> 4;
        prev_knob = io_data.knob;


        // Vrefin - internal reference voltage, 1.2v
        // Vref - ADC reference voltage, equal to ADC supply voltage (~ 3.3v)
        // adc_vrefin = 1.2 / Vref * 4096
        fix16_t v_ref = fix16_div(F16(1.2), adc_v_refin << 4);

        // maximum ADC input voltage - Vref
        // current = adc_current_norm * v_ref / cfg_shunt_resistance
        io_data.current = fix16_mul(
            fix16_mul(adc_current << 4, cfg_shunt_resistance_inv),
            v_ref
        );

        // Compensate current offset
        io_data.current -= cfg_current_offset;
        if (io_data.current < 0) io_data.current = 0;

        // resistors in voltage divider - [ 2*150 kOhm, 1.5 kOhm ]
        // (divider ratio => 201)
        // voltage = adc_voltage * v_ref * (301.5 / 1.5);
        io_data.voltage = fix16_mul(adc_voltage << 4, v_ref) * 201;


        if (prev_voltage == 0 && io_data.voltage > 0) io_data.zero_cross_up = true;
        else io_data.zero_cross_up = false;

        if (prev_voltage > 0 && io_data.voltage == 0) io_data.zero_cross_down = true;
        else io_data.zero_cross_down = false;

        prev_voltage = io_data.voltage;

        count_phase(io_data);
        triac_update(io_data);

        out.push(io_data); // returns false on overflow, but no exception
    }

private:

    // Previous iteration values
    fix16_t prev_voltage = 0;
    fix16_t prev_knob = 0;

    fix16_t cfg_shunt_resistance_inv = 1; // Fake

    //
    // Triac states
    //

    uint32_t phase_counter = 0; // increment every tick
    bool triac_open_done = false;
    bool triac_close_done = false;

    // Holds measured number of ticks per positive half-period
    uint32_t positive_period_in_ticks = 0;

    bool once_zero_crossed = false;
    bool once_period_counted = false;


    void count_phase(io_data_t &io_data)
    {
        if (io_data.zero_cross_up || io_data.zero_cross_down)
        {
            if (once_zero_crossed) once_period_counted = true;

            once_zero_crossed = true;

            // If full half-period was counted at least once, save number of
            // ticks in half-period
            if (once_period_counted)
            {
                // Measure period on positive half wave only
                if (io_data.zero_cross_down) positive_period_in_ticks = phase_counter;
            }

            phase_counter = 0;
            return;
        }

        phase_counter++;
    }


    void triac_update(io_data_t &io_data)
    {
        // Poor man zero cross check
        if (io_data.zero_cross_up || io_data.zero_cross_down)
        {
            triac_open_done = false;
            triac_close_done = false;

            // Make sure to disable triac signal, if reset (zero cross) happens
            // immediately after triac enabled
            hal::triac_ignition_off();
        }

        // If period_in_ticks is not yet detected, don't touch triac.
        if (!once_period_counted) return;

        // We keep ignition open continiously after calculated phase shift. Pulse
        // commutation is not safe because triac current can flow after voltage
        // is zero. If we pulse at this moment, next period will be lost.
        //
        // So, we keep triac open until the end, and pay for it with 10ma load
        // at 3.3v.
        //
        // We should close ignition at the end of half-wave. To be sure - do it
        // in advance, 4 ticks before.

        if ((triac_open_done && !triac_close_done) &&
            (phase_counter + TRIAC_ZERO_TAIL_LENGTH >= positive_period_in_ticks))
        {
            triac_close_done = true;
            hal::triac_ignition_off();
        }

        // If ignition was not yet activated - check if we can do this
        if (!triac_open_done)
        {
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
                (phase_counter + TRIAC_ZERO_TAIL_LENGTH < positive_period_in_ticks))
            {
                triac_open_done = true;
                hal::triac_ignition_on();
            }
        }
    }
};


#endif
