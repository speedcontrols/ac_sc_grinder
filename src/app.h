#ifndef __APP_H__
#define __APP_H__

#include <stdint.h>

float eeprom_float_read(uint32_t addr, float dflt);
void eeprom_float_write(uint32_t addr, float val);

#include "io.h"

extern Io io;

#include "meter.h"

extern Meter meter;

#ifdef ADRC
    #include "regulator_adrc.h"
#else
    #include "regulator.h"
#endif

extern Regulator regulator;

#endif
