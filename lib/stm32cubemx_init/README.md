STM32CubeMX generated files
===========================

DON'T change content manually.

How to update:

1. Install and run STM32CubeMX
   (https://www.st.com/en/development-tools/stm32cubemx.html#getsoftware-scroll)
2. Open file `ac_sc_grinder.ioc`.
3. Make nesessary changes.
4. Menu: `File` -> `Save Project`
4. Menu: `Project` -> `Generate Code`.
5. Make sure `Src/main.c` contains this lines: `#include "../../src/app.h"` &
   `app_start();`
6. Run build and check result.
