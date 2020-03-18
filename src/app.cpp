#include <stdint.h>
#include "app.h"
#include "app_hal.h"

#include "eeprom_emu.h"
#include "eeprom_flash_driver.h"

#include "meter.h"
#include "io.h"
#include "regulator.h"
#include "calibrator/calibrator.h"

// Note: update version tag to reset old data
EepromEmu<EepromFlashDriver, 0x0001> eeprom;

Io io;
Meter meter;
Regulator regulator;
Calibrator calibrator;


float eeprom_float_read(uint32_t addr, float dflt) {
    return eeprom.read_float(addr, dflt);
}

void eeprom_float_write(uint32_t addr, float val) {
    return eeprom.write_float(addr, val);
}


int main()
{
    hal::setup();

    // Load config info from emulated EEPROM
    io.configure();
    regulator.configure();
    meter.configure();

    hal::start();

    // Override loop in main.c to reduce patching
    while (1) {
        // Polling for flag which indicates that ADC data is ready
        while (io.out.empty()) {}

        io_data_t io_data;
        io.out.pop(io_data);

        meter.tick(io_data);

        // Detect calibration mode & run calibration procedure if needed.
        // If calibration in progress - skip other steps.
        if (calibrator.tick(io_data)) continue;

        // Normal processing

        if (meter.is_r_calibrated)
        {
            regulator.tick(io_data.knob, meter.speed);
            io.setpoint = regulator.out_power;
        }
        else {
            // Force speed to some slow value when R is not calibrated
            io.setpoint = F16(0.2);
        }
    }
}
