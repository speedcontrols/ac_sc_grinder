Grinder speed control with stable RPM
=====================================

[![Gitter chat](https://badges.gitter.im/speedcontrols/ac_sc_grinder.svg)](https://gitter.im/speedcontrols/ac_sc_grinder)

> Advanced speed control for grinder AC brushed motor. With RPM stabilization
> via Back EMF measure. Replacement for default board.

[Video](https://youtu.be/6eNhbyeh3mg):

[![Video](https://i.ytimg.com/vi/6eNhbyeh3mg/hqdefault.jpg)](https://youtu.be/6eNhbyeh3mg)


Notes! Due size restrictions, it's impossible to create universal PCB.
We prepeared PCB for `Hilda 180W` - it's very popular and cheap. Boards for
other grinder models are left to volunteers.


### Required components

1. [Hilda 180W](https://www.aliexpress.com/af/hilda-180w.html?SortType=total_tranpro_desc).
   Project is not limited to this device, but existing PCB layout is for
   this grinder model. Also, firmware config defaults are for it's motor's power.
2. Go to [EasyEda project page](https://easyeda.com/speed/AC-speed-control-for-grinder)
    - Order PCB
    - Order details from BOM. We already prepeared links to LCSC for your
      convenience. It's not the cheapest in the wold, but good enougth. You
      may find nice to buy all details in one place and join delivery to
      single package.
3. Get additional details, not included into BOM:
    - [Cheap ST-link/V2 programmer](https://www.aliexpress.com/af/st-link-v2.html?SortType=total_tranpro_desc)
      for stm32 devices (only 2$).
    - [Plastik 70 CRC](https://www.google.com/search?q=Plastik+70+CRC) or any
      other acrylic [insulating lacquer](https://www.google.com/search?q=insulating+lacquer)
      (liquid or spray). We strongly recommend to protect PCB from aspirated dust.
    - [male](https://www.aliexpress.com/item/100pcs-2-8-Inserts-Plug-Spring-Terminal-PCB-Solder-lug-thickness-0-8-one-legs-PCB/32702011692.html) &
      [female](https://www.aliexpress.com/item/100pcs-lot-2-8-insulated-terminal-with-0-5-0-8-male-insert-brass-color-connectors/32593170276.html) 2.8mm power terminals (optional - you can solder wires directly).
4. Extract some components from native grinder board:
    - Speed potentiometer with wheel.
    - Terminal pins for motor contacts.
    - AC filtering capacitor (0.1uF 275v). It's not drawn on schematics, solder
      it after board assembly to AC power pins.


## Building hardware

1. Assemble PCB.
2. Flash firmware (see next chapter).
3. Assemble device with new board, run self-calibration and check everything
   works as expected.
4. Cover PCB with insulating lacquer, to protect it from aspirated dust (we
   strongly recommend to not ignore this step).

**IMPORTANT**. When you turn device on after flash, motor will run at slow speed
and will not react on knob. That means, you should run self-calibration.

To run calibration:

- Move knob to zero.
- Move knob shortly up-and-down 3 times (in 3 seconds).
- Wait couple of minutes until magic finishes and motor stops. Be patient.

Calibration is required only once.


## How to flash firmware

**WARNING! You MUST unplug power cord prior to flash firmware.** Turning power off
via grinder switch is not enougth. If you plug programmator into computer
while AC plug in power socket, your USB interface may be damaged!

1. Install [VS Code](https://code.visualstudio.com/).
2. Clone this repo or download via zip archive and unpack somewhere.
3. Open folder with project AND after VS Code suggests to install plugins - agree
   with everything. That should install platformio, compilers and flashers.
4. Make sure you installed & configured `ST-link/V2` drivers:
   - [Linux](http://docs.platformio.org/en/latest/installation.html#troubleshooting)
     instructions (how to configure udev rules).
   - [Windows](https://www.st.com/en/development-tools/stsw-link009.html) drivers.
5. Run flasher via VS Code menu: `Terminal` -> `Run Task...` -> `PlatformIO: Upload`,
   and wait until complete.

Don't forget to run calibration after firmware upload!


## Development

Everything is done in `PlatformIO`. If you wish to update conig and regenerate
headers, you may need `node.js` and run `npm run config`.

Note, this PCB has no AC isolation! That's ok for normal operation, because
board is placed inside isolated case. But if you plan to debug firmware via USB,
you MUST use [USB isolator](https://ru.aliexpress.com/wholesale?SearchText=USB+isolator)
module.


## License

MIT.
