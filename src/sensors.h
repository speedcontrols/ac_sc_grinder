#ifndef __SENSORS__
#define __SENSORS__

#include <string.h>

#include "eeprom_float.h"
#include "config_map.h"
#include "math/fix16_math.h"
#include "median.h"
#include "app.h"

/*
  Sensors data source:

  - voltage: physical, immediate
  - current: physical, from shunt, immediate (but can have phase shift)
  - power: calculated
  - speed: calculated
*/

// Buffer used for storage only positive half-wave values.
// To prevent buffer overflow if supply frequency < 50 Hz
// it's length is two 50-Hz half-waves
constexpr int voltage_buffer_length = APP_TICK_FREQUENCY / 50;

class Sensors
{
public:

  fix16_t speed = 0;
  fix16_t voltage = 0;
  fix16_t current = 0;
  fix16_t knob = 0; // Speed knob physical value, 0..1

  // Flags to simplify checks in other modules.
  // true on zero cross up/down, false in all other ticks
  bool zero_cross_up = false;
  bool zero_cross_down = false;

  bool is_r_calibrated = false;

  // Config info
  fix16_t cfg_shunt_resistance_inv;
  fix16_t cfg_rekv_to_speed_factor;

  // Calibration params. Non needed on real work
  fix16_t cfg_min_power_treshold = 0;
  fix16_t cfg_current_offset = 0;

  // Input from triac driver to reflect triac setpoint. Needed for speed measure
  // to interpolate motor resistance. Autoupdated by triac driver.
  fix16_t in_triac_setpoint = 0;

  fix16_t cfg_r_table_setpoints[CFG_R_INTERP_TABLE_LENGTH] = {
    F16(1.0 / 10.0),
    F16(1.0 / 9.0),
    F16(1.0 / 8.0),
    F16(1.0 / 7.0),
    F16(1.0 / 6.0),
    F16(1.0 / 5.0),
    F16(1.0 / 4.0),
    F16(1.0 / 3.0),
    F16(1.0 / 2.0),
    F16(1.0 / 1.0),
  };

  // Should be called with 40kHz frequency
  void tick()
  {
    // Do preliminary filtering of raw data + normalize result
    fetch_adc_data();

    if (prev_voltage == 0 && voltage > 0) zero_cross_up = true;
    else zero_cross_up = false;

    if (prev_voltage > 0 && voltage == 0) zero_cross_down = true;
    else zero_cross_down = false;

    speed_tick();

    prev_voltage = voltage;
  }

  // Load config from emulated EEPROM
  void configure()
  {
    // config shunt resistance - in mOhm (divide by 1000)
    // shunt amplifier gain - 50
    cfg_shunt_resistance_inv = fix16_from_float(1.0f /
      (eeprom_float_read(CFG_SHUNT_RESISTANCE_ADDR, CFG_SHUNT_RESISTANCE_DEFAULT)
        * 50
        / 1000)
    );

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

    is_r_calibrated = cfg_r_table[0] == R_CAL_CHECK_MARKER ? false : true;
  }

  // Split raw ADC data by separate buffers
  void adc_raw_data_load(uint16_t ADCBuffer[], uint32_t adc_data_offset)
  {
    for (int sample = 0; sample < ADC_FETCH_PER_TICK; sample++)
    {
      adc_voltage_temp_buf[sample] = ADCBuffer[adc_data_offset++];
      adc_current_temp_buf[sample] = ADCBuffer[adc_data_offset++];
      adc_knob_temp_buf[sample] = ADCBuffer[adc_data_offset++];
      adc_v_refin_temp_buf[sample] = ADCBuffer[adc_data_offset++];
    }
  }

private:
  // Temporary buffers for adc data, filled by "interrupt" (DMA)
  uint16_t adc_voltage_temp_buf[ADC_FETCH_PER_TICK];
  uint16_t adc_current_temp_buf[ADC_FETCH_PER_TICK];
  uint16_t adc_knob_temp_buf[ADC_FETCH_PER_TICK];
  uint16_t adc_v_refin_temp_buf[ADC_FETCH_PER_TICK];

  // Motor resistance interpolation table
  fix16_t cfg_r_table[CFG_R_INTERP_TABLE_LENGTH];

  // 1. Calculate σ (discrete random variable)
  // 2. Drop everything with deviation > 2σ and count mean for the rest.
  //
  // https://upload.wikimedia.org/wikipedia/commons/8/8c/Standard_deviation_diagram.svg
  //
  // For efficiensy, don't use root square (work with σ^2 instead)
  //
  // !!! count sould NOT be > 16
  //
  // src - circular buffer
  // head - index of NEXT data to write
  // count - number of elements BACK from head to process
  // window - sigma multiplier (usually [1..2])
  //
  // Why this work? We use collision avoiding approach. Interrupt can happen,
  // but we work with tail, and data is written to head. If bufer is big enougth,
  // we have time to process tails until override.
  //
  uint32_t truncated_mean(uint16_t *src, int count, fix16_t window)
  {
    int idx = 0;

    // Count mean & sigma in one pass
    // https://en.wikipedia.org/wiki/Algorithms_for_calculating_variance
    idx = count;
    uint32_t s = 0;
    uint32_t s2 = 0;
    while (idx)
    {
      int val = src[--idx];
      s += val;
      s2 += val * val;
    }

    int mean = (s + (count >> 1)) / count;

    int sigma_square = (s2 - (s * s / count)) / (count - 1);
    // quick & dirty multiply to win^2, when win is in fix16 format.
    // we suppose win is 1..2, and sigma^2 - 24 bits max
    int sigma_win_square = ((((window >> 8) * (window >> 8)) >> 12) * sigma_square) >> 4;

    // Drop big deviations and count mean for the rest
    idx = count;
    int s_mean_filtered = 0;
    int s_mean_filtered_cnt = 0;

    while (idx)
    {
      int val = src[--idx];

      if ((mean - val) * (mean - val) < sigma_win_square)
      {
        s_mean_filtered += val;
        s_mean_filtered_cnt++;
      }
    }

    // Protection from zero div. Should never happen
    if (!s_mean_filtered_cnt) return mean;

    return (s_mean_filtered + (s_mean_filtered_cnt >> 1)) / s_mean_filtered_cnt;
  }

  void fetch_adc_data()
  {
    // Apply filters
    uint16_t adc_voltage = truncated_mean(adc_voltage_temp_buf, ADC_FETCH_PER_TICK, F16(1.1));
    uint16_t adc_current = truncated_mean(adc_current_temp_buf, ADC_FETCH_PER_TICK, F16(1.1));
    uint16_t adc_knob = truncated_mean(adc_knob_temp_buf, ADC_FETCH_PER_TICK, F16(1.1));
    uint16_t adc_v_refin =  truncated_mean(adc_v_refin_temp_buf, ADC_FETCH_PER_TICK, F16(1.1));

    // Now process the rest...

    // 4096 - maximum value of 12-bit integer
    // normalize to fix16_t[0.0..1.0]
    fix16_t knob_new = adc_knob << 4;

    // Use additional mean smoother for knob
    knob = (knob * 15 + knob_new) >> 4;


    // Vrefin - internal reference voltage, 1.2v
    // Vref - ADC reference voltage, equal to ADC supply voltage (~ 3.3v)
    // adc_vrefin = 1.2 / Vref * 4096
    fix16_t v_ref = fix16_div(F16(1.2), adc_v_refin << 4);

    // maximum ADC input voltage - Vref
    // current = adc_current_norm * v_ref / cfg_shunt_resistance
    current = fix16_mul(
      fix16_mul(adc_current << 4, cfg_shunt_resistance_inv),
      v_ref
    );

    // Compensate current offset
    current -= cfg_current_offset;
    if (current < 0) current = 0;

    // resistors in voltage divider - [ 2*150 kOhm, 1.5 kOhm ]
    // (divider ratio => 201)
    // voltage = adc_voltage * v_ref * (301.5 / 1.5);
    voltage = fix16_mul(adc_voltage << 4, v_ref) * 201;
  }

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
        fix16_t scale = fix16_div(
          setpoint - cfg_r_table_setpoints[i],
          cfg_r_table_setpoints[i + 1] - cfg_r_table_setpoints[i]
        );

        return fix16_mul(range_start, fix16_one - scale) + fix16_mul(range_end, scale);
      }
    }

    return cfg_r_table[0];
  }

  // Previous iteration values. Used to detect zero cross.
  fix16_t prev_voltage = 0;

  fix16_t p_sum_div_16 = 0;  // active power / 16
  fix16_t i2_sum = 0; // square of current

  // voltage with extrapolated negative half-wave
  fix16_t virtual_voltage;

  // Buffer for extrapolating voltage during negative half-wave
  fix16_t voltage_buffer[voltage_buffer_length];
  uint32_t voltage_buffer_head = 0;
  // Holds number of ticks from the beginning of
  // negative half-wave
  uint32_t voltage_buffer_tick_counter = 0;

  void speed_tick()
  {
    // Don't try to calculate speed until R calibrated
    if (!is_r_calibrated)
    {
      speed = 0;
      return;
    }

    // Reset voltage buffer head at zero crossings
    if (zero_cross_down)
    {
      voltage_buffer_tick_counter = 0;
    }

    // Save voltage samples during positive half-wave
    // to extrapolate during negative half-wave
    if (voltage > 0)
    {
      voltage_buffer[voltage_buffer_head++] = voltage;
      virtual_voltage = voltage;
    }
    else
    {
      virtual_voltage = -voltage_buffer[voltage_buffer_tick_counter++];
    }

    // Calculate sums
    // To prevent p_sum overflow divide power value by 16
    p_sum_div_16 += fix16_mul(virtual_voltage, current) >> 4;
    i2_sum += fix16_mul(current, current);

    // Calculate speed at end of negative half-wave
    // In this case active power is equivalent to
    // Joule power, P = R * I^2
    // R = P / I^2
    // r_ekv is equivalent resistance created by back-EMF
    // r_ekv = R - R_motor
    if (zero_cross_up)
    {
      // p_sum was divided by 16 to prevent overflow,
      // so i2_sum must be divided by 16 now
      fix16_t r_ekv = fix16_div(p_sum_div_16, i2_sum >> 4) - get_motor_resistance(in_triac_setpoint);

      speed = fix16_div(r_ekv, cfg_rekv_to_speed_factor);

      // Attempt to drop noise on calibration phase
      if (p_sum_div_16 / (int)voltage_buffer_head * 16 < cfg_min_power_treshold) speed = 0;

      // Clamp calculated speed value, speed can't be negative
      if (speed < 0) speed = 0;

      p_sum_div_16 = 0;
      i2_sum = 0;
      voltage_buffer_head = 0;
    }
  }
};


#endif
