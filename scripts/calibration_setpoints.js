#!/usr/bin/env node

// Generate setpoints to measure motor speed
//
// - step is non-linear, to balance between precision and measurements speed.
// - final data is generated manually.

let setpoint = 0;

do {
  if (setpoint < 0.2) setpoint += 1 / 64;
  else if (setpoint < 0.375) setpoint += 1 / 32;
  else setpoint += 1 / 10;

  if (setpoint >= 1) setpoint = 1; // clamp overflow

  /* eslint-disable no-console */
  console.log(`F16(${setpoint}),`);
} while (setpoint < 1)
