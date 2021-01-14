#ifndef __CALIBRATOR_NOISE__
#define __CALIBRATOR_NOISE__

// Calibrate noise & OP offset

#include "../math/fix16_math.h"
#include "../yield.h"
#include "../app.h"


class CalibratorNoise
{
public:

    bool tick(io_data_t &io_data) {
        YIELDABLE;

        io.setpoint = 0;

        YIELD_UNTIL(io_data.zero_cross_up, false);

        acc_counter = 0;
        acc_p_sum_2e64 = 0;
        acc_i2_sum_2e64 = 0;
        acc_i_sum = 0;

        while (!io_data.zero_cross_down) {
            YIELD(false);

            acc_p_sum_2e64 += (int64_t)(io_data.voltage) * io_data.current;
            acc_i2_sum_2e64 += (uint64_t)(io_data.current) * io_data.current;
            acc_i_sum += io_data.current;
            acc_counter++;
        }

        // Store OP offset
        io.cfg_current_offset = acc_i_sum / acc_counter;

        // Store noise tresholds (4x of noise value)
        {
            if (acc_p_sum_2e64 < 0) acc_p_sum_2e64 = 0;
            meter.cfg_min_p_sum_2e64 = acc_p_sum_2e64 * 4;
            meter.cfg_min_i2_sum_2e64 = acc_i2_sum_2e64 * 2;
        }

        return true;
    }

private:
    uint16_t acc_counter = 0;
    int64_t acc_p_sum_2e64 = 0;
    int64_t acc_i2_sum_2e64 = 0;
    uint32_t acc_i_sum = 0;
};


#endif
