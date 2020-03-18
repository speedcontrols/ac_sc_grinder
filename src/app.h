#ifndef __APP_H__
#define __APP_H__

#include <stdint.h>

// "EEPROM" location and size for RPM non-linearity conmensation table.
// Starts after other config variables.
#define CFG_RPM_INTERP_TABLE_START_ADDR 10
#define CFG_RPM_INTERP_TABLE_LENGTH 16

#define CFG_R_INTERP_TABLE_START_ADDR 26
#define CFG_R_INTERP_TABLE_LENGTH 7

float eeprom_float_read(uint32_t addr, float dflt);
void eeprom_float_write(uint32_t addr, float val);

#include "io.h"

extern Io io;

#include "meter.h"

extern Meter meter;

#include "regulator.h"

extern Regulator regulator;

#endif
