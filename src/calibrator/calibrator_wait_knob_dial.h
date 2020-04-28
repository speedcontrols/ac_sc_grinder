#ifndef __CALIBRATOR_WAIT_KNOB_DIAL_H__
#define __CALIBRATOR_WAIT_KNOB_DIAL_H__

// Detects when user quickly dials knob 3 times. This sequence is used
// to start calibration sequence.
//
// .tick() should be called with APP_TICK_FREQUENCY/sec, as everything else.
// It returns `true` when dials detected, and `false` in other cases.


#include "../math/fix16_math.h"

#include "../app.h"


#define KNOB_TRESHOLD F16(0.05)

#define IS_KNOB_LOW(val)  (val < KNOB_TRESHOLD)
#define IS_KNOB_HIGH(val) (val >= KNOB_TRESHOLD)

constexpr int knob_wait_min = (int)(APP_TICK_FREQUENCY * 0.2f);
constexpr int knob_wait_max = (int)(APP_TICK_FREQUENCY * 1.0f);


class CalibratorWaitKnobDial
{
public:

    bool tick(io_data_t &io_data) {
        YIELDABLE;

        // Try endless
        while (1)
        {
            // First, check knob is at sstart position (zero),
            // prior to start detect dial sequence
            ticks_cnt = 0;
            YIELD(false);

            while (IS_KNOB_LOW(io_data.knob)) {
                YIELD(false);
                ticks_cnt++;
            }

            if (ticks_cnt < knob_wait_min) continue;

            // If knob is zero long enough => can start detect dials
            dials_cnt = 0;

            while (1) {
                // Measure UP interval
                ticks_cnt = 0;

                while (IS_KNOB_HIGH(io_data.knob)) {
                    YIELD(false);
                    ticks_cnt++;
                }

                // Resart on invalid length
                if (ticks_cnt < knob_wait_min || ticks_cnt > knob_wait_max) break;

                // Finish on success
                // (return without YIELD also resets state to start)
                if (++dials_cnt >= 3) return true;

                // Measure DOWN interval
                ticks_cnt = 0;

                while (IS_KNOB_LOW(io_data.knob)) {
                    YIELD(false);
                    ticks_cnt++;
                }

                // Restart on invalid length
                if (ticks_cnt < knob_wait_min || ticks_cnt > knob_wait_max) break;
            }
        }
    }

private:

    int ticks_cnt = 0;
    int dials_cnt = 0;

};


#endif
