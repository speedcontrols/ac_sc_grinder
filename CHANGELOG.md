3.0.0 / 2021-08-13
------------------

- Support all new boards.
- Replace PID with ADRC.
- Improved calibration.
- Improved noise filter & avoid zero division at start.


2.0.1 / 2020-10-14
------------------

- Fix noise power threshold in calibrator.


2.0.0 / 2020-10-03
------------------

- Added support of STM32F0 family (different kinds of speed optimizations).
- Reduced RAM usage significantly.
- Added HAL-s for new boards.
- Use short pulses to open triac.
- Fixed max possible speed.
- Fixed internal cross-down event timing for symmetry wih cross-up.
- Renamed build targets.
- Code refactoring & cleanup.


1.1.1 / 2020-06-13
------------------

- Fix build error, caused by bug in PlaformIO registry.
- Fix PID_I component disable (regression in 1.1.0).
- Tune calibrator options to provide more stable result.


1.1.0 / 2020-04-29
------------------

- Full refactoring, prior to start more deep changes. Nothing "visible for
  users", but internals improved significantly.
- Disabled regulator's integral component by default.
- Moved triac's logic to ISR & use message queue to emit ADC data. Now logic
  is robust to theoretic computation bursts in event loop.
- Use hand-made "YIELD" macro to improve calibrator's logic readability.
- Renamed classes and reorganized helpers.
- Reduced number of divisions in single event loop pass (use table lookups +
  multiplication for small value ranges).
- Reworked stm32cubemx integration. Keep full sources to avoid conflicts with
  outdated PlatformIO's version. Removed all places with tight-coupled code.
- Added alternate HAL entry for new upcoming PCB.
- New EEPROM emulator.
- Added build tests on Travis-CI for MacOS & Win.


1.0.0 / 2019-08-01
------------------

- First Release
