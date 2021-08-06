#ifndef __CONFIG_MAP__
#define __CONFIG_MAP__

//
// Virtual EEPROM addresses & defaults for config variables.
// Emulator uses append-only log & multiphase commits to guarantee
// atomic writes.
//

// Current sensor shunt resistance (mOhm)
#define CFG_SHUNT_RESISTANCE_ADDR 1
#define CFG_SHUNT_RESISTANCE_DEFAULT 10.0f

// RPM at max voltage without load
#define CFG_RPM_MAX_ADDR 2
#define CFG_RPM_MAX_DEFAULT 37500.0f

// Minimal allowed speed (RPM)
#define CFG_RPM_MIN_LIMIT_ADDR 3
#define CFG_RPM_MIN_LIMIT_DEFAULT 5000.0f

// Maximal allowed speed (RPM)
#define CFG_RPM_MAX_LIMIT_ADDR 4
#define CFG_RPM_MAX_LIMIT_DEFAULT 30000.0f

// Knob initial zone where motor should not run (% of max range).
#define CFG_DEAD_ZONE_WIDTH_ADDR 5
#define CFG_DEAD_ZONE_WIDTH_DEFAULT 2.0f

// ADRC parameters (auto-calibrated).
#define CFG_ADRC_KP_ADDR 6
#define CFG_ADRC_KP_DEFAULT 1.0f

#define CFG_ADRC_KOBSERVERS_ADDR 7
#define CFG_ADRC_KOBSERVERS_DEFAULT 1.0f

#define CFG_ADRC_P_CORR_COEFF_ADDR 8
#define CFG_ADRC_P_CORR_COEFF_DEFAULT 0.0f

// Constructive coefficient between normalized motor speed.
// and back-EMF equivalent resistance (auto-calibrated).
#define CFG_REKV_TO_SPEED_FACTOR_ADDR 9
#define CFG_REKV_TO_SPEED_FACTOR_DEFAULT 450.0f

// Active resistance interpolaion table (depends on triac phase).
#define CFG_R_INTERP_TABLE_START_ADDR 10
#define CFG_R_INTERP_TABLE_LENGTH 7


#endif
